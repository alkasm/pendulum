use std::{
    io::{self, BufRead, BufReader, Write},
    net::{TcpListener, TcpStream},
    thread,
    time::Duration,
};

use penproto::TelemetryPacket;

use crate::{
    packet::{self, PacketReceiver, PacketStream},
    telemetry::{self, TelemetryFrame, TelemetryReceiver, TelemetryStream},
};

pub const DEFAULT_TELEMETRY_ADDR: &str = "127.0.0.1:7001";
pub const DEFAULT_TELEMETRY_SOURCE_ADDR: &str = "127.0.0.1:7002";
pub const DEFAULT_TELEMETRY_SERIAL_BAUD: u32 = 115_200;
const TELEMETRY_CONNECT_RETRY_DELAY: Duration = Duration::from_millis(500);
const SERIAL_PORT_CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

pub fn spawn_tcp_packet_server(
    bind_addr: impl Into<String>,
    packets: PacketStream,
) -> io::Result<thread::JoinHandle<()>> {
    let bind_addr = bind_addr.into();
    let listener = TcpListener::bind(&bind_addr)?;

    Ok(thread::spawn(move || {
        for stream in listener.incoming() {
            match stream {
                Ok(stream) => {
                    let packets = packets.clone();
                    thread::spawn(move || serve_packet_client(stream, packets.subscribe()));
                }
                Err(error) => {
                    panic!("Telemetry packet accept failed on {bind_addr}: {error}");
                }
            }
        }
    }))
}

pub fn spawn_tcp_telemetry_server(
    bind_addr: impl Into<String>,
    telemetry: TelemetryStream,
) -> io::Result<thread::JoinHandle<()>> {
    let bind_addr = bind_addr.into();
    let listener = TcpListener::bind(&bind_addr)?;

    Ok(thread::spawn(move || {
        for stream in listener.incoming() {
            match stream {
                Ok(stream) => {
                    let telemetry = telemetry.clone();
                    thread::spawn(move || serve_client(stream, telemetry.subscribe()));
                }
                Err(error) => {
                    panic!("Telemetry accept failed on {bind_addr}: {error}");
                }
            }
        }
    }))
}

pub fn connect_tcp_packets(addr: &str) -> io::Result<PacketReceiver> {
    let stream = TcpStream::connect(addr)?;
    let _ = stream.set_nodelay(true);
    let reader = BufReader::new(stream);

    spawn_packet_reader(reader, format!("tcp telemetry at {addr}"))
}

pub fn connect_tcp_telemetry(addr: &str) -> io::Result<TelemetryReceiver> {
    let packets = connect_tcp_packets(addr)?;
    Ok(spawn_runtime_filter(packets))
}

pub fn connect_serial_packets(port_name: &str, baud_rate: u32) -> io::Result<PacketReceiver> {
    let port = serialport::new(port_name, baud_rate)
        .timeout(SERIAL_PORT_CONNECT_TIMEOUT)
        .open()
        .map_err(serial_error_to_io_error)?;
    let reader = BufReader::new(port);

    spawn_packet_reader(
        reader,
        format!("serial telemetry at {port_name} @ {baud_rate} baud"),
    )
}

pub fn connect_serial_telemetry(port_name: &str, baud_rate: u32) -> io::Result<TelemetryReceiver> {
    let packets = connect_serial_packets(port_name, baud_rate)?;
    Ok(spawn_runtime_filter(packets))
}

pub fn connect_serial_packets_blocking(port_name: &str, baud_rate: u32) -> PacketReceiver {
    let mut announced_wait = false;

    loop {
        match connect_serial_packets(port_name, baud_rate) {
            Ok(packet_rx) => {
                if announced_wait {
                    println!("Telemetry stream connected at {port_name} @ {baud_rate} baud");
                }
                return packet_rx;
            }
            Err(error) => {
                if !announced_wait {
                    println!("Waiting for telemetry stream at {port_name} @ {baud_rate} baud...");
                    announced_wait = true;
                }
                println!("Telemetry stream not ready at {port_name} @ {baud_rate} baud: {error}");
                thread::sleep(TELEMETRY_CONNECT_RETRY_DELAY);
            }
        }
    }
}

pub fn connect_serial_telemetry_blocking(port_name: &str, baud_rate: u32) -> TelemetryReceiver {
    let mut announced_wait = false;

    loop {
        match connect_serial_telemetry(port_name, baud_rate) {
            Ok(telemetry_rx) => {
                if announced_wait {
                    println!("Telemetry stream connected at {port_name} @ {baud_rate} baud");
                }
                return telemetry_rx;
            }
            Err(error) => {
                if !announced_wait {
                    println!("Waiting for telemetry stream at {port_name} @ {baud_rate} baud...");
                    announced_wait = true;
                }
                println!("Telemetry stream not ready at {port_name} @ {baud_rate} baud: {error}");
                thread::sleep(TELEMETRY_CONNECT_RETRY_DELAY);
            }
        }
    }
}

fn spawn_packet_reader<R>(
    reader: BufReader<R>,
    source_name: String,
) -> io::Result<PacketReceiver>
where
    R: io::Read + Send + 'static,
{
    let mut reader = reader;

    let packets = PacketStream::new();
    let sender = packets.publisher();
    let packet_rx = packets.subscribe();

    thread::spawn(move || {
        loop {
            match read_packet(&mut reader) {
                Ok(packet) => sender.send(packet),
                Err(error) => {
                    println!("Telemetry connection to {source_name} closed: {error}");
                    break;
                }
            }
        }
    });

    Ok(packet_rx)
}

fn spawn_runtime_filter(mut packets: PacketReceiver) -> TelemetryReceiver {
    let telemetry = TelemetryStream::new();
    let sender = telemetry.publisher();
    let telemetry_rx = telemetry.subscribe();

    thread::spawn(move || {
        while let Some(packet) = packet::recv_latest(&mut packets) {
            if let TelemetryPacket::Runtime(frame) = packet {
                sender.send(frame);
            }
        }
    });

    telemetry_rx
}

pub fn connect_tcp_packets_blocking(addr: &str) -> PacketReceiver {
    let mut announced_wait = false;

    loop {
        match connect_tcp_packets(addr) {
            Ok(packet_rx) => {
                if announced_wait {
                    println!("Telemetry stream connected at {addr}");
                }
                return packet_rx;
            }
            Err(error) => {
                if !announced_wait {
                    println!("Waiting for telemetry stream at {addr}...");
                    announced_wait = true;
                }
                println!("Telemetry stream not ready at {addr}: {error}");
                thread::sleep(TELEMETRY_CONNECT_RETRY_DELAY);
            }
        }
    }
}

pub fn connect_tcp_telemetry_blocking(addr: &str) -> TelemetryReceiver {
    let mut announced_wait = false;

    loop {
        match connect_tcp_telemetry(addr) {
            Ok(telemetry_rx) => {
                if announced_wait {
                    println!("Telemetry stream connected at {addr}");
                }
                return telemetry_rx;
            }
            Err(error) => {
                if !announced_wait {
                    println!("Waiting for telemetry stream at {addr}...");
                    announced_wait = true;
                }
                println!("Telemetry stream not ready at {addr}: {error}");
                thread::sleep(TELEMETRY_CONNECT_RETRY_DELAY);
            }
        }
    }
}

fn serve_packet_client(mut stream: TcpStream, mut packets: PacketReceiver) {
    let _ = stream.set_nodelay(true);

    while let Some(packet) = packet::recv_latest(&mut packets) {
        if let Err(error) = write_packet(&mut stream, &packet) {
            println!("Telemetry packet client disconnected: {error}");
            return;
        }
    }
}

fn serve_client(mut stream: TcpStream, mut telemetry: TelemetryReceiver) {
    let _ = stream.set_nodelay(true);

    while let Some(frame) = telemetry::recv_latest(&mut telemetry) {
        if let Err(error) = write_frame(&mut stream, &frame) {
            println!("Telemetry client disconnected: {error}");
            return;
        }
    }
}

pub fn write_packet<W>(writer: &mut W, packet: &TelemetryPacket) -> io::Result<()>
where
    W: Write,
{
    let encoded = postcard::to_allocvec_cobs(packet).map_err(to_invalid_data_error)?;
    writer.write_all(&encoded)
}

fn write_frame<W>(writer: &mut W, frame: &TelemetryFrame) -> io::Result<()>
where
    W: Write,
{
    let packet = TelemetryPacket::Runtime(*frame);
    write_packet(writer, &packet)
}

pub fn read_packet<R>(reader: &mut R) -> io::Result<TelemetryPacket>
where
    R: BufRead,
{
    let mut frame_buf = Vec::new();
    let bytes_read = reader.read_until(0, &mut frame_buf)?;

    if bytes_read == 0 {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "telemetry stream closed",
        ));
    }

    if !matches!(frame_buf.last(), Some(0)) {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "incomplete COBS frame",
        ));
    }

    postcard::from_bytes_cobs(&mut frame_buf).map_err(to_invalid_data_error)
}

fn to_invalid_data_error(error: postcard::Error) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, error)
}

fn serial_error_to_io_error(error: serialport::Error) -> io::Error {
    io::Error::other(error)
}
