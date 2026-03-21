mod viewer;

use std::{
    sync::{Arc, Mutex},
    thread,
};

use pendulum_lib::{config::VisualizationConfig, transport};

fn main() {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_ADDR.to_string());
    let pending_connection = Arc::new(Mutex::new(None));
    let pending_connection_slot = pending_connection.clone();

    thread::spawn(move || {
        let telemetry_rx = transport::connect_tcp_telemetry_blocking(&addr);
        let mut slot = pending_connection_slot
            .lock()
            .expect("GUI pending connection mutex poisoned");
        *slot = Some(telemetry_rx);
    });

    viewer::run(pending_connection, VisualizationConfig::default());
}
