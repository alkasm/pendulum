use pendulum_lib::{
    telemetry::{self, TelemetryFrame},
    transport,
};
use uom::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    electric_current::ampere,
    time::second,
    torque::newton_meter,
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
        frame.sim_time.get::<second>(),
        frame.theta.get::<radian>(),
        frame.theta.get::<radian>().to_degrees(),
        frame.theta_dot.get::<radian_per_second>(),
        frame.wheel_speed.get::<radian_per_second>(),
        frame.commanded_torque.get::<newton_meter>(),
        frame.applied_torque.get::<newton_meter>(),
        frame.available_torque.get::<newton_meter>(),
        frame.phase_current.get::<ampere>(),
        frame.speed_ratio,
    );
}
