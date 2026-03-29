use std::{
    io::{self, BufReader},
    thread,
    time::Duration,
};

use clap::{Parser, Subcommand};
use penproto::TelemetryPacket;
use pendulum_lib::{
    telemetry::{self, TelemetryStream},
    transport,
};

const SENSOR_SERIAL_CONNECT_TIMEOUT: Duration = Duration::from_millis(250);
const SENSOR_SERIAL_RETRY_DELAY: Duration = Duration::from_millis(500);

#[derive(Parser)]
#[command(name = "penproxy")]
struct Cli {
    #[arg(long, default_value = transport::DEFAULT_TELEMETRY_ADDR)]
    bind: String,

    #[arg(long)]
    log: bool,

    #[command(subcommand)]
    source: Option<SourceCommand>,
}

#[derive(Subcommand)]
enum SourceCommand {
    Tcp {
        #[arg(long, default_value = transport::DEFAULT_TELEMETRY_SOURCE_ADDR)]
        upstream: String,
    },
    Serial {
        #[arg(long)]
        port: String,

        #[arg(long, default_value_t = transport::DEFAULT_TELEMETRY_SERIAL_BAUD)]
        baud: u32,
    },
}

enum UpstreamSource {
    Tcp { addr: String },
    Serial { port: String, baud: u32 },
}

fn main() {
    let cli = Cli::parse();
    let bind_addr = cli.bind;
    let log_frames = cli.log;
    let upstream = match cli.source.unwrap_or(SourceCommand::Tcp {
        upstream: transport::DEFAULT_TELEMETRY_SOURCE_ADDR.to_string(),
    }) {
        SourceCommand::Tcp { upstream } => UpstreamSource::Tcp { addr: upstream },
        SourceCommand::Serial { port, baud } => UpstreamSource::Serial { port, baud },
    };

    let telemetry = TelemetryStream::new();
    let sender = telemetry.publisher();

    transport::spawn_tcp_telemetry_server(bind_addr.clone(), telemetry.clone()).unwrap_or_else(
        |error| panic!("Failed to bind proxy telemetry server on {bind_addr}: {error}"),
    );

    println!(
        "Telemetry proxy listening on {bind_addr} and relaying from {}",
        upstream.description()
    );

    loop {
        match &upstream {
            UpstreamSource::Tcp { addr } => {
                let mut upstream_rx = transport::connect_tcp_telemetry_blocking(addr);
                println!(
                    "Proxy connected to upstream telemetry from {}",
                    upstream.description()
                );

                while let Some(frame) = telemetry::recv_latest(&mut upstream_rx) {
                    if log_frames {
                        match serde_json::to_string(&TelemetryPacket::Runtime(frame)) {
                            Ok(json) => println!("{json}"),
                            Err(error) => {
                                eprintln!("Failed to encode telemetry packet as JSON: {error}")
                            }
                        }
                    }
                    sender.send(frame);
                }
            }
            UpstreamSource::Serial { port, baud } => {
                stream_serial_packets(port, *baud, &sender, log_frames);
            }
        }

        println!(
            "Upstream telemetry ended; reconnecting to {}...",
            upstream.description()
        );
    }
}

impl UpstreamSource {
    fn description(&self) -> String {
        match self {
            Self::Tcp { addr } => format!("tcp:{addr}"),
            Self::Serial { port, baud } => format!("serial:{port}@{baud}"),
        }
    }
}

fn stream_serial_packets(
    port: &str,
    baud: u32,
    sender: &pendulum_lib::telemetry::TelemetrySender,
    log_frames: bool,
) {
    let mut announced_wait = false;

    loop {
        match open_serial_reader(port, baud) {
            Ok(mut reader) => {
                if announced_wait {
                    println!("Telemetry connected at {port} @ {baud} baud");
                } else {
                    println!("Reading telemetry from {port} @ {baud} baud");
                }
                announced_wait = false;

                loop {
                    match transport::read_packet(&mut reader) {
                        Ok(packet) => {
                            let should_log = log_frames || matches!(packet, TelemetryPacket::Sensor(_));
                            if should_log {
                                match serde_json::to_string(&packet) {
                                    Ok(json) => println!("{json}"),
                                    Err(error) => {
                                        eprintln!("Failed to encode telemetry packet as JSON: {error}")
                                    }
                                }
                            }

                            if let TelemetryPacket::Runtime(frame) = packet {
                                sender.send(frame);
                            }
                        }
                        Err(error) if error.kind() == io::ErrorKind::InvalidData => {
                            eprintln!("Discarding invalid telemetry packet: {error}");
                        }
                        Err(error) => {
                            if error.kind() == io::ErrorKind::UnexpectedEof {
                                eprintln!("Telemetry stream at {port} @ {baud} baud ended: {error}");
                                return;
                            }
                            eprintln!("Telemetry stream at {port} @ {baud} baud ended: {error}");
                            break;
                        }
                    }
                }
            }
            Err(error) => {
                if !announced_wait {
                    println!("Waiting for telemetry at {port} @ {baud} baud...");
                    announced_wait = true;
                }
                eprintln!("Telemetry not ready at {port} @ {baud} baud: {error}");
                thread::sleep(SENSOR_SERIAL_RETRY_DELAY);
            }
        }
    }
}

fn open_serial_reader(port: &str, baud: u32) -> io::Result<BufReader<Box<dyn serialport::SerialPort>>> {
    let serial = serialport::new(port, baud)
        .timeout(SENSOR_SERIAL_CONNECT_TIMEOUT)
        .open()
        .map_err(io::Error::other)?;
    Ok(BufReader::new(serial))
}
