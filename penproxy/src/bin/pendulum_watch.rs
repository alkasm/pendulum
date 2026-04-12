use pendulum_lib::{HallTelemetry, PendulumEstimateTelemetry, TelemetryPacket, packet, transport};

fn main() {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| transport::DEFAULT_TELEMETRY_ADDR.to_string());
    let mut packet_rx = transport::connect_tcp_packets_blocking(&addr);

    println!("Watching pendulum telemetry from {addr}");

    while let Some(packet) = packet::recv_latest(&mut packet_rx) {
        if let Some(line) = format_pendulum_brief(&packet) {
            println!("{line}");
        }
    }
}

fn format_pendulum_brief(packet: &TelemetryPacket) -> Option<String> {
    let TelemetryPacket::Pendulum(frame) = packet else {
        return None;
    };

    let (theta_deg, theta_dot_dps, imu_status) = match frame.estimate {
        PendulumEstimateTelemetry::Measurement(measurement) => (
            format!("{:+6.2}", measurement.theta_deg),
            format!("{:+7.2}", measurement.theta_dot_dps),
            "ok".to_string(),
        ),
        other => (
            "   n/a".to_string(),
            "    n/a".to_string(),
            format!("{other:?}"),
        ),
    };

    let hall_status = match frame.hall {
        HallTelemetry::Measurement(_) => "ok".to_string(),
        other => format!("{other:?}"),
    };

    Some(format!(
        concat!(
            "seq={:>5} ",
            "loop={:>5}us ",
            "work={:>5}us ",
            "mode={:?} ",
            "theta={}deg ",
            "err={:+6.2}deg ",
            "theta_dot={}dps ",
            "raw={:+4.2} ",
            "elec={:>7.2}deg ",
            "uq={:+4.2}V ",
            "wheel={:>7.2}deg ",
            "wheel_dot={:>7.2}dps ",
            "drive={:+4.2} ",
            "tau={:+6.3}Nm ",
            "dir={:+3.0} ",
            "tsign={:+3.0} ",
            "step={} ",
            "enabled={} ",
            "iabc=[{:+4.2},{:+4.2},{:+4.2}]A ",
            "imu={} ",
            "hall={}"
        ),
        frame.seq,
        frame.timing.loop_period_us,
        frame.timing.work_time_us,
        frame.control.mode,
        theta_deg,
        frame.control.theta_error_deg,
        theta_dot_dps,
        frame.control.raw_drive_command,
        frame.control.electrical_angle_deg,
        frame.control.uq_v,
        frame.control.wheel_angle_deg,
        frame.control.wheel_speed_dps,
        frame.control.drive_command,
        frame.control.torque_command_nm,
        frame.control.direction_sign,
        frame.control.torque_sign,
        frame.control.commutation_step,
        frame.control.motor_enabled,
        frame.current.ina_u_amps,
        frame.current.ina_v_amps,
        frame.current.ina_w_amps,
        imu_status,
        hall_status,
    ))
}
