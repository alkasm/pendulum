mod viewer;

use pendulum_lib::{config::VisualizationConfig, transport};

fn main() {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_ADDR.to_string());
    let telemetry_rx = transport::connect_tcp_telemetry_blocking(&addr);

    viewer::run(telemetry_rx, VisualizationConfig::default());
}
