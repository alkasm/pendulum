use std::{
    io::{self, BufRead, BufReader, Write},
    net::{TcpListener, TcpStream},
    thread,
    time::Duration,
};

use crate::telemetry::{self, TelemetryFrame, TelemetryReceiver, TelemetryStream};

pub const DEFAULT_TELEMETRY_ADDR: &str = "127.0.0.1:7001";
const TELEMETRY_CONNECT_RETRY_DELAY: Duration = Duration::from_millis(500);

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
    let mut reader = BufReader::new(stream);

    let telemetry = TelemetryStream::new();
    let sender = telemetry.publisher();
    let telemetry_rx = telemetry.subscribe();
    let addr = addr.to_string();

    thread::spawn(move || {
        loop {
            match read_frame(&mut reader) {
                Ok(frame) => sender.send(frame),
                Err(error) => {
                    println!("Telemetry connection to {addr} closed: {error}");
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
    let encoded = postcard::to_allocvec_cobs(frame).map_err(to_invalid_data_error)?;
    writer.write_all(&encoded)
}

fn read_frame<R>(reader: &mut R) -> io::Result<TelemetryFrame>
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
