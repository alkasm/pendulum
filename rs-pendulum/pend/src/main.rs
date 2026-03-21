#[cfg(target_os = "linux")]
mod hw;
#[cfg(target_os = "linux")]
mod runtime;

#[cfg(target_os = "linux")]
use pendulum_lib::{config::RuntimeConfig, telemetry::TelemetryStream, transport};

#[cfg(target_os = "linux")]
use crate::runtime::spawn_hardware_runtime;

#[cfg(target_os = "linux")]
fn main() {
    let bind_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_ADDR.to_string());
    let telemetry = TelemetryStream::new();

    transport::spawn_tcp_telemetry_server(bind_addr.clone(), telemetry.clone()).unwrap_or_else(
        |error| panic!("Failed to bind hardware telemetry server on {bind_addr}: {error}"),
    );

    println!("Hardware daemon streaming telemetry on {bind_addr}");

    let runtime_thread =
        spawn_hardware_runtime(RuntimeConfig::default(), telemetry.publisher())
            .unwrap_or_else(|error| panic!("Failed to initialize hardware runtime: {error:?}"));

    runtime_thread
        .join()
        .expect("Hardware runtime thread terminated unexpectedly.");
}
