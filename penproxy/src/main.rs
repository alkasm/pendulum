use clap::{Parser, Subcommand};
use pendulum_lib::{
    packet::{self, PacketStream},
    transport,
};

#[derive(Parser)]
#[command(name = "penproxy")]
struct Cli {
    #[arg(long, default_value = transport::DEFAULT_TELEMETRY_ADDR)]
    bind: String,

    #[arg(short, long)]
    verbose: bool,

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
    let log_frames = cli.verbose;
    let upstream = match cli.source.unwrap_or(SourceCommand::Tcp {
        upstream: transport::DEFAULT_TELEMETRY_SOURCE_ADDR.to_string(),
    }) {
        SourceCommand::Tcp { upstream } => UpstreamSource::Tcp { addr: upstream },
        SourceCommand::Serial { port, baud } => UpstreamSource::Serial { port, baud },
    };

    let packets = PacketStream::new();
    let packet_sender = packets.publisher();

    transport::spawn_tcp_packet_server(bind_addr.clone(), packets.clone()).unwrap_or_else(
        |error| panic!("Failed to bind proxy telemetry server on {bind_addr}: {error}"),
    );

    println!(
        "Telemetry proxy listening on {bind_addr} and relaying from {}",
        upstream.description()
    );

    loop {
        let mut upstream_rx = upstream.connect_blocking();
        println!(
            "Proxy connected to upstream telemetry from {}",
            upstream.description()
        );

        while let Some(packet) = packet::recv_latest(&mut upstream_rx) {
            if log_frames {
                match serde_json::to_string(&packet) {
                    Ok(json) => println!("{json}"),
                    Err(error) => {
                        eprintln!("Failed to encode telemetry packet as JSON: {error}")
                    }
                }
            }
            packet_sender.send(packet);
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
    fn connect_blocking(&self) -> pendulum_lib::packet::PacketReceiver {
        match self {
            Self::Tcp { addr } => transport::connect_tcp_packets_blocking(addr),
            Self::Serial { port, baud } => transport::connect_serial_packets_blocking(port, *baud),
        }
    }
}
