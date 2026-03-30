#![no_std]

use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, AngularVelocity, ElectricCurrent, Time, Torque};

pub const DEFAULT_SENSOR_TELEMETRY_BAUD: u32 = 115_200;

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct RuntimeTelemetryFrame {
    pub step: u64,
    pub sim_time: Time,
    pub theta: Angle,
    pub theta_dot: AngularVelocity,
    pub wheel_angle: Angle,
    pub wheel_speed: AngularVelocity,
    pub commanded_torque: Torque,
    pub applied_torque: Torque,
    pub available_torque: Torque,
    pub speed_ratio: f64,
    pub phase_current: ElectricCurrent,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum TelemetryPacket {
    Runtime(RuntimeTelemetryFrame),
    Sensor(SensorTelemetryFrame),
    Pendulum(PendulumTelemetryFrame),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SensorTelemetryFrame {
    pub seq: u32,
    pub uptime_ms: u32,
    pub motor_driver_diag_high: bool,
    pub current: CurrentTelemetry,
    pub hall: HallTelemetry,
    pub imu: ImuTelemetry,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CurrentTelemetry {
    pub mcp6021_counts: u16,
    pub mcp6021_delta_counts: i32,
    pub mcp6021_volts: f32,
    pub ina_u_counts: u16,
    pub ina_u_delta_counts: i32,
    pub ina_u_amps: f32,
    pub ina_v_counts: u16,
    pub ina_v_delta_counts: i32,
    pub ina_v_amps: f32,
    pub ina_w_counts: u16,
    pub ina_w_delta_counts: i32,
    pub ina_w_amps: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum HallTelemetry {
    Missing,
    ConfigError { register: u8 },
    ReadError { register: u8 },
    Measurement(HallMeasurement),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HallMeasurement {
    pub temperature_c: f32,
    pub x_mt: f32,
    pub y_mt: f32,
    pub z_mt: f32,
    pub angle_deg: f32,
    pub magnitude: u8,
    pub set_count: u8,
    pub result_ready: bool,
    pub por: bool,
    pub diag_fail: bool,
    pub int_pin_high: bool,
    pub oscillator_error: bool,
    pub int_pin_error: bool,
    pub otp_crc_error: bool,
    pub vcc_uv_error: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum ImuTelemetry {
    Missing,
    UnexpectedWhoAmI { value: u8 },
    WakeError { register: u8 },
    ReadError { register: u8 },
    Measurement(ImuMeasurement),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImuMeasurement {
    pub ax_g: f32,
    pub ay_g: f32,
    pub az_g: f32,
    pub gx_dps: f32,
    pub gy_dps: f32,
    pub gz_dps: f32,
    pub temperature_c: f32,
    pub theta_deg: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumTelemetryFrame {
    pub seq: u32,
    pub uptime_ms: u32,
    pub motor_driver_diag_high: bool,
    pub current: CurrentTelemetry,
    pub hall: HallTelemetry,
    pub estimate: PendulumEstimateTelemetry,
    pub control: PendulumControlTelemetry,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum PendulumEstimateTelemetry {
    Missing,
    UnexpectedWhoAmI { value: u8 },
    WakeError { register: u8 },
    ReadError { register: u8 },
    Measurement(PendulumEstimateMeasurement),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumEstimateMeasurement {
    pub theta_deg: f32,
    pub theta_dot_dps: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumControlTelemetry {
    pub mode: PendulumControlMode,
    pub torque_command_nm: f32,
    pub drive_command: f32,
    pub electrical_angle_deg: f32,
    pub uq_v: f32,
    pub wheel_angle_deg: f32,
    pub wheel_speed_dps: f32,
    pub commutation_step: u8,
    pub commutation_center_deg: f32,
    pub motor_enabled: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum PendulumControlMode {
    Startup,
    Calibrating,
    Arming,
    WaitingForImu,
    WaitingForHall,
    CaptureOutOfRange,
    CurrentLimited,
    Idle,
    Active,
}
