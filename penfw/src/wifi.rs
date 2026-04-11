use alloc::{boxed::Box, string::String};

use esp_hal::{
    delay::Delay,
    peripherals::WIFI,
    rng::Rng,
    timer::timg::Timer,
};
use esp_wifi::{
    init,
    wifi::{self, ClientConfiguration, Configuration, WifiController, WifiDevice},
};
use pendulum_lib::{WifiCredentials, WifiProbeResult};
use smoltcp::{
    iface::{Config as IfaceConfig, Interface, SocketSet, SocketStorage},
    socket::dhcpv4::{Event as DhcpEvent, Socket as DhcpSocket},
    time::Instant as SmolInstant,
    wire::{EthernetAddress, HardwareAddress, IpCidr},
};

const ASSOC_TIMEOUT_MS: u64 = 10_000;
const DHCP_TIMEOUT_MS: u64 = 10_000;
const DHCP_POLL_PERIOD_MS: u32 = 50;

pub struct WifiValidator<'d> {
    controller: WifiController<'d>,
    sta: WifiDevice<'d>,
}

impl<'d> WifiValidator<'d> {
    pub fn new(
        timer: Timer<'d>,
        rng: Rng,
        wifi: WIFI<'d>,
    ) -> Result<Self, esp_wifi::InitializationError> {
        let wifi_init = Box::leak(Box::new(init(timer, rng)?));
        let (controller, interfaces) = wifi::new(wifi_init, wifi)?;
        Ok(Self {
            controller,
            sta: interfaces.sta,
        })
    }

    pub fn validate(
        &mut self,
        credentials: &WifiCredentials,
        delay: &Delay,
    ) -> WifiProbeResult {
        if self.controller.stop().is_err() {
            // Ignore stop failures here; the controller may simply not be started yet.
        }

        let configuration = Configuration::Client(ClientConfiguration {
            ssid: String::from(credentials.ssid.as_str()),
            password: String::from(credentials.password.as_str()),
            ..Default::default()
        });

        if self.controller.set_configuration(&configuration).is_err() {
            return WifiProbeResult::ConfigurationRejected;
        }

        if self.controller.start().is_err() {
            return WifiProbeResult::StartFailed;
        }

        if self.controller.connect().is_err() {
            let _ = self.controller.stop();
            return WifiProbeResult::AssociationFailed;
        }

        let assoc_started_at = esp_hal::time::Instant::now();
        while assoc_started_at.elapsed().as_millis() < ASSOC_TIMEOUT_MS {
            match self.controller.is_connected() {
                Ok(true) => break,
                Ok(false) => delay.delay_millis(DHCP_POLL_PERIOD_MS),
                Err(_) => {
                    let _ = self.controller.stop();
                    return WifiProbeResult::AssociationFailed;
                }
            }
        }

        match self.controller.is_connected() {
            Ok(true) => {}
            _ => {
                let _ = self.controller.stop();
                return WifiProbeResult::AssociationTimedOut;
            }
        }

        let mac_address = self.sta.mac_address();
        let device = &mut self.sta;
        let mut iface_config =
            IfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(mac_address)));
        iface_config.random_seed = 1;
        let now = SmolInstant::from_millis(0);
        let mut iface = Interface::new(iface_config, device, now);
        let mut socket_storage = [SocketStorage::EMPTY];
        let mut sockets = SocketSet::new(&mut socket_storage[..]);
        let dhcp_handle = sockets.add(DhcpSocket::new());

        let dhcp_started_at = esp_hal::time::Instant::now();
        loop {
            let now_ms = dhcp_started_at.elapsed().as_millis() as i64;
            let _ = iface.poll(SmolInstant::from_millis(now_ms), device, &mut sockets);

            let event = sockets.get_mut::<DhcpSocket>(dhcp_handle).poll();
            match event {
                Some(DhcpEvent::Configured(config)) => {
                    iface.update_ip_addrs(|addrs| {
                        addrs.clear();
                        let _ = addrs.push(IpCidr::Ipv4(config.address));
                    });

                    if let Some(router) = config.router {
                        let _ = iface.routes_mut().add_default_ipv4_route(router);
                    } else {
                        iface.routes_mut().remove_default_ipv4_route();
                    }

                    let ip = config.address.address().octets();
                    let _ = self.controller.disconnect();
                    let _ = self.controller.stop();
                    return WifiProbeResult::Success { ipv4_octets: ip };
                }
                Some(DhcpEvent::Deconfigured) => {
                    iface.update_ip_addrs(|addrs| addrs.clear());
                    iface.routes_mut().remove_default_ipv4_route();
                }
                None => {}
            }

            if dhcp_started_at.elapsed().as_millis() >= DHCP_TIMEOUT_MS {
                let _ = self.controller.disconnect();
                let _ = self.controller.stop();
                return WifiProbeResult::DhcpTimedOut;
            }

            delay.delay_millis(DHCP_POLL_PERIOD_MS);
        }
    }
}
