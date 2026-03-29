use std::{
    io::{self, BufRead, BufReader, Write},
    net::{TcpListener, TcpStream},
    thread,
    time::Duration,
};

use penproto::TelemetryPacket;

use crate::telemetry::{self, TelemetryFrame, TelemetryReceiver, TelemetryStream};

pub const DEFAULT_TELEMETRY_ADDR: &str = "127.0.0.1:7001";
pub const DEFAULT_TELEMETRY_SOURCE_ADDR: &str = "127.0.0.1:7002";
pub const DEFAULT_TELEMETRY_SERIAL_BAUD: u32 = 115_200;
const TELEMETRY_CONNECT_RETRY_DELAY: Duration = Duration::from_millis(500);
const SERIAL_PORT_CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

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

pub fn connect_tcp_telemetry(addr: &str) -> io::Result<TelemetryReceiver> {
    let stream = TcpStream::connect(addr)?;
    let _ = stream.set_nodelay(true);
    let reader = BufReader::new(stream);

    spawn_telemetry_reader(reader, format!("tcp telemetry at {addr}"))
}

pub fn connect_serial_telemetry(port_name: &str, baud_rate: u32) -> io::Result<TelemetryReceiver> {
    let port = serialport::new(port_name, baud_rate)
        .timeout(SERIAL_PORT_CONNECT_TIMEOUT)
        .open()
        .map_err(serial_error_to_io_error)?;
    let reader = BufReader::new(port);

    spawn_telemetry_reader(
        reader,
        format!("serial telemetry at {port_name} @ {baud_rate} baud"),
    )
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

fn spawn_telemetry_reader<R>(
    reader: BufReader<R>,
    source_name: String,
) -> io::Result<TelemetryReceiver>
where
    R: io::Read + Send + 'static,
{
    let mut reader = reader;

    let telemetry = TelemetryStream::new();
    let sender = telemetry.publisher();
    let telemetry_rx = telemetry.subscribe();

    thread::spawn(move || {
        loop {
            match read_frame(&mut reader) {
                Ok(frame) => sender.send(frame),
                Err(error) => {
                    println!("Telemetry connection to {source_name} closed: {error}");
                    break;
                }
            }
        }
    });

    Ok(telemetry_rx)
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

fn serve_client(mut stream: TcpStream, mut telemetry: TelemetryReceiver) {
    let _ = stream.set_nodelay(true);

    while let Some(frame) = telemetry::recv_latest(&mut telemetry) {
        if let Err(error) = write_frame(&mut stream, &frame) {
            println!("Telemetry client disconnected: {error}");
            return;
        }
    }
}

fn write_frame<W>(writer: &mut W, frame: &TelemetryFrame) -> io::Result<()>
where
    W: Write,
{
    let packet = TelemetryPacket::Runtime(*frame);
    let encoded = postcard::to_allocvec_cobs(&packet).map_err(to_invalid_data_error)?;
    writer.write_all(&encoded)
}

fn read_frame<R>(reader: &mut R) -> io::Result<TelemetryFrame>
where
    R: BufRead,
{
    match read_packet(reader)? {
        TelemetryPacket::Runtime(frame) => Ok(frame),
        TelemetryPacket::Sensor(_) => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "received sensor telemetry on runtime telemetry channel",
        )),
    }
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
