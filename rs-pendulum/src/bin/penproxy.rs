use pendulum_lib::{
    telemetry::{self, TelemetryStream},
    transport,
};

fn main() {
    let mut args = std::env::args().skip(1);
    let upstream_addr = args
        .next()
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_SOURCE_ADDR.to_string());
    let bind_addr = args
        .next()
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_ADDR.to_string());
    let telemetry = TelemetryStream::new();
    let sender = telemetry.publisher();
    let _keepalive = sender.clone();

    transport::spawn_tcp_telemetry_server(bind_addr.clone(), telemetry.clone()).unwrap_or_else(
        |error| panic!("Failed to bind proxy telemetry server on {bind_addr}: {error}"),
    );

    println!(
        "Telemetry proxy listening on {bind_addr} and relaying from {upstream_addr}"
    );

    loop {
        let mut upstream_rx = transport::connect_tcp_telemetry_blocking(&upstream_addr);
        println!("Proxy connected to upstream telemetry at {upstream_addr}");

        while let Some(frame) = telemetry::recv_latest(&mut upstream_rx) {
            sender.send(frame);
        }

        println!("Upstream telemetry ended; reconnecting to {upstream_addr}...");
    }
}
