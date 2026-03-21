use std::{thread, time::Duration};

use pendulum_lib::{
    telemetry::{self, TelemetryFrame},
    transport,
};

fn main() {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_ADDR.to_string());
    let mut telemetry_rx = transport::connect_tcp_telemetry_blocking(&addr);

    println!("Console visualizer connected to {addr}");

    while let Some(frame) = telemetry::recv_latest(&mut telemetry_rx) {
        log_telemetry(frame);
    }

    println!("Telemetry stream ended.");
}

fn log_telemetry(frame: TelemetryFrame) {
    println!(
        "step={:>5} t={:>6.2}s theta={:+.3} rad ({:+6.1} deg) theta_dot={:+6.3} rad/s wheel_speed={:+7.2} rad/s torque_cmd={:+6.3} Nm torque_applied={:+6.3} Nm avail={:+6.3} Nm current={:+6.3} A speed_ratio={:.3}",
        frame.step,
        frame.sim_time_s,
        frame.theta_rad,
        frame.theta_rad.to_degrees(),
        frame.theta_dot_rad_s,
        frame.wheel_speed_rad_s,
        frame.commanded_torque_nm,
        frame.applied_torque_nm,
        frame.available_torque_nm,
        frame.phase_current_a,
        frame.speed_ratio,
    );
}
