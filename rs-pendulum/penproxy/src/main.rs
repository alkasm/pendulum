use clap::{Parser, Subcommand};
use pendulum_lib::{
    telemetry::{self, TelemetryStream},
    transport,
};

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
        let mut upstream_rx = match &upstream {
            UpstreamSource::Tcp { addr } => transport::connect_tcp_telemetry_blocking(addr),
            UpstreamSource::Serial { port, baud } => {
                transport::connect_serial_telemetry_blocking(port, *baud)
            }
        };
        println!(
            "Proxy connected to upstream telemetry from {}",
            upstream.description()
        );

        while let Some(frame) = telemetry::recv_latest(&mut upstream_rx) {
            if log_frames {
                match serde_json::to_string(&frame) {
                    Ok(json) => println!("{json}"),
                    Err(error) => eprintln!("Failed to encode telemetry frame as JSON: {error}"),
                }
            }
            sender.send(frame);
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
