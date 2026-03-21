mod viewer;

use pendulum_lib::{config::VisualizationConfig, transport};

fn main() {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_ADDR.to_string());
    let telemetry_rx = transport::connect_tcp_telemetry(&addr)
        .unwrap_or_else(|error| panic!("Failed to connect to telemetry stream at {addr}: {error}"));

    viewer::run(telemetry_rx, VisualizationConfig::default());
}
