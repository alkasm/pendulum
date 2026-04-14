use alloc::{boxed::Box, string::String};

use esp_hal::{delay::Delay, peripherals::WIFI, rng::Rng, time::Instant, timer::timg::Timer};
use esp_wifi::{
    init,
    wifi::{self, ClientConfiguration, Configuration, WifiController, WifiDevice},
};
use pendulum_lib::{RuntimeTelemetryFrame, TelemetryPacket, WifiCredentials, WifiProbeResult};
use smoltcp::{
    iface::{Config as IfaceConfig, Interface, SocketHandle, SocketSet, SocketStorage},
    socket::{
        dhcpv4::{Event as DhcpEvent, Socket as DhcpSocket},
        tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer},
    },
    time::{Duration as SmolDuration, Instant as SmolInstant},
    wire::{EthernetAddress, HardwareAddress, IpCidr},
};

const ASSOC_TIMEOUT_MS: u64 = 10_000;
const DHCP_TIMEOUT_MS: u64 = 10_000;
const DHCP_POLL_PERIOD_MS: u32 = 50;
const SOCKET_STORAGE_COUNT: usize = 2;
const TELEMETRY_TCP_RX_BUF_LEN: usize = 256;
const TELEMETRY_TCP_TX_BUF_LEN: usize = 1024;
const TELEMETRY_FRAME_BUF_LEN: usize = 512;

pub struct WifiService<'d> {
    controller: WifiController<'d>,
    sta: WifiDevice<'d>,
    iface: Interface,
    sockets: SocketSet<'static>,
    dhcp_handle: SocketHandle,
    telemetry_handle: SocketHandle,
    applied_credentials: Option<WifiCredentials>,
    ipv4_octets: Option<[u8; 4]>,
}

impl<'d> WifiService<'d> {
    pub fn new(
        timer: Timer<'d>,
        rng: Rng,
        wifi: WIFI<'d>,
    ) -> Result<Self, esp_wifi::InitializationError> {
        let wifi_init = Box::leak(Box::new(init(timer, rng)?));
        let (controller, interfaces) = wifi::new(wifi_init, wifi)?;
        let mut sta = interfaces.sta;

        let mut iface_config = IfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(
            sta.mac_address(),
        )));
        iface_config.random_seed = 1;
        let iface = Interface::new(iface_config, &mut sta, timestamp());

        let socket_storage = Box::leak(Box::new([SocketStorage::EMPTY; SOCKET_STORAGE_COUNT]));
        let mut sockets = SocketSet::new(&mut socket_storage[..]);
        let dhcp_handle = sockets.add(DhcpSocket::new());
        let telemetry_rx = Box::leak(Box::new([0u8; TELEMETRY_TCP_RX_BUF_LEN]));
        let telemetry_tx = Box::leak(Box::new([0u8; TELEMETRY_TCP_TX_BUF_LEN]));
        let telemetry_handle = sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(&mut telemetry_rx[..]),
            TcpSocketBuffer::new(&mut telemetry_tx[..]),
        ));

        Ok(Self {
            controller,
            sta,
            iface,
            sockets,
            dhcp_handle,
            telemetry_handle,
            applied_credentials: None,
            ipv4_octets: None,
        })
    }

    pub fn validate(&mut self, credentials: &WifiCredentials, delay: &Delay) -> WifiProbeResult {
        self.stop_runtime();

        let result = self.validate_inner(credentials, delay);

        self.stop_runtime();
        result
    }

    pub fn stream_runtime_telemetry(
        &mut self,
        credentials: Option<&WifiCredentials>,
        port: u16,
        latest_frame: Option<&RuntimeTelemetryFrame>,
    ) {
        // Wi-Fi owns the long-lived network session. Telemetry just hands us the latest
        // runtime frame plus the desired listening port, and this service keeps the link,
        // DHCP state, and TCP listener alive underneath it.
        let Some(credentials) = credentials else {
            self.stop_runtime();
            return;
        };

        if self.applied_credentials.as_ref() != Some(credentials)
            && self.start_station(credentials).is_err()
        {
            return;
        }

        if !self.ensure_station_connected() {
            return;
        }

        self.poll_network();
        if self.ipv4_octets.is_none() {
            return;
        }

        self.ensure_telemetry_listener(port);
        self.drain_telemetry_socket();

        let Some(frame) = latest_frame else {
            return;
        };

        self.send_runtime_frame(frame);
    }

    fn validate_inner(&mut self, credentials: &WifiCredentials, delay: &Delay) -> WifiProbeResult {
        if let Err(result) = self.start_station(credentials) {
            return result;
        }

        let assoc_started_at = Instant::now();
        while assoc_started_at.elapsed().as_millis() < ASSOC_TIMEOUT_MS {
            match self.controller.is_connected() {
                Ok(true) => break,
                Ok(false) => delay.delay_millis(DHCP_POLL_PERIOD_MS),
                Err(_) => return WifiProbeResult::AssociationFailed,
            }
        }

        match self.controller.is_connected() {
            Ok(true) => {}
            _ => return WifiProbeResult::AssociationTimedOut,
        }

        let dhcp_started_at = Instant::now();
        while dhcp_started_at.elapsed().as_millis() < DHCP_TIMEOUT_MS {
            self.poll_network();

            if let Some(ipv4_octets) = self.ipv4_octets {
                return WifiProbeResult::Success { ipv4_octets };
            }

            delay.delay_millis(DHCP_POLL_PERIOD_MS);
        }

        WifiProbeResult::DhcpTimedOut
    }

    fn start_station(&mut self, credentials: &WifiCredentials) -> Result<(), WifiProbeResult> {
        self.stop_runtime();

        let configuration = Configuration::Client(ClientConfiguration {
            ssid: String::from(credentials.ssid.as_str()),
            password: String::from(credentials.password.as_str()),
            ..Default::default()
        });

        if self.controller.set_configuration(&configuration).is_err() {
            return Err(WifiProbeResult::ConfigurationRejected);
        }

        if self.controller.start().is_err() {
            return Err(WifiProbeResult::StartFailed);
        }

        if self.controller.connect().is_err() {
            let _ = self.controller.stop();
            return Err(WifiProbeResult::AssociationFailed);
        }

        self.applied_credentials = Some(credentials.clone());
        Ok(())
    }

    fn ensure_station_connected(&mut self) -> bool {
        if !matches!(self.controller.is_started(), Ok(true)) && self.controller.start().is_err() {
            return false;
        }

        match self.controller.is_connected() {
            Ok(true) => true,
            _ => {
                self.clear_ip_configuration();
                let _ = self.controller.connect();
                false
            }
        }
    }

    fn poll_network(&mut self) {
        let _ = self
            .iface
            .poll(timestamp(), &mut self.sta, &mut self.sockets);

        let event = self.sockets.get_mut::<DhcpSocket>(self.dhcp_handle).poll();
        match event {
            Some(DhcpEvent::Configured(config)) => {
                self.iface.update_ip_addrs(|addrs| {
                    addrs.clear();
                    let _ = addrs.push(IpCidr::Ipv4(config.address));
                });

                if let Some(router) = config.router {
                    let _ = self.iface.routes_mut().add_default_ipv4_route(router);
                } else {
                    self.iface.routes_mut().remove_default_ipv4_route();
                }

                self.ipv4_octets = Some(config.address.address().octets());
            }
            Some(DhcpEvent::Deconfigured) => self.clear_ip_configuration(),
            None => {}
        }
    }

    fn ensure_telemetry_listener(&mut self, port: u16) {
        let socket = self.sockets.get_mut::<TcpSocket>(self.telemetry_handle);
        if socket.is_open() {
            return;
        }

        if socket.listen(port).is_ok() {
            socket.set_keep_alive(Some(SmolDuration::from_millis(1_000)));
            socket.set_timeout(Some(SmolDuration::from_millis(2_000)));
        }
    }

    fn drain_telemetry_socket(&mut self) {
        let socket = self.sockets.get_mut::<TcpSocket>(self.telemetry_handle);

        if socket.can_recv() {
            let _ = socket.recv(|buffer| (buffer.len(), ()));
        } else if !socket.may_recv() && socket.may_send() {
            socket.close();
        }
    }

    fn send_runtime_frame(&mut self, frame: &RuntimeTelemetryFrame) {
        let socket = self.sockets.get_mut::<TcpSocket>(self.telemetry_handle);
        if !socket.can_send() {
            return;
        }

        let packet = TelemetryPacket::Runtime(*frame);
        let mut buffer = [0u8; TELEMETRY_FRAME_BUF_LEN];
        let Ok(encoded) = postcard::to_slice_cobs(&packet, &mut buffer) else {
            return;
        };

        let _ = socket.send_slice(encoded);
    }

    fn stop_runtime(&mut self) {
        self.close_telemetry_socket();
        self.clear_ip_configuration();
        self.applied_credentials = None;
        let _ = self.controller.disconnect();
        let _ = self.controller.stop();
    }

    fn close_telemetry_socket(&mut self) {
        let socket = self.sockets.get_mut::<TcpSocket>(self.telemetry_handle);
        if socket.is_open() {
            socket.close();
        }
    }

    fn clear_ip_configuration(&mut self) {
        self.iface.update_ip_addrs(|addrs| addrs.clear());
        self.iface.routes_mut().remove_default_ipv4_route();
        self.ipv4_octets = None;
    }
}

fn timestamp() -> SmolInstant {
    SmolInstant::from_micros(Instant::now().duration_since_epoch().as_micros() as i64)
}
