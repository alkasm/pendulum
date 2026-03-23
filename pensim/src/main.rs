mod runtime;
mod sim;

use pendulum_lib::{telemetry::TelemetryStream, transport};

fn main() {
    let bind_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_SOURCE_ADDR.to_string());
    let sim_config = sim::SimConfig::default();
    let telemetry = TelemetryStream::new();

    transport::spawn_tcp_telemetry_server(bind_addr.clone(), telemetry.clone()).unwrap_or_else(
        |error| panic!("Failed to bind sim telemetry server on {bind_addr}: {error}"),
    );

    println!("Sim daemon streaming telemetry on {bind_addr}");

    let runtime_thread = runtime::spawn_simulation_runtime(sim_config, telemetry.publisher());
    runtime_thread
        .join()
        .expect("Simulation runtime thread terminated unexpectedly.");
}
