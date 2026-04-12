use crate::{
    controller::{ControllerInput, ControllerOutput, PendulumController},
    protocol::{
        HallTelemetry, PendulumControlMode, PendulumControlTelemetry, PendulumEstimateTelemetry,
    },
    StoredMotorCalibration,
};
use libm::sinf;

pub const VOLTAGE_LIMIT_V: f32 = 4.6;
const MOTOR_POLE_PAIRS: f32 = 7.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HallElectricalCalibration {
    pub direction_sign: f32,
    pub electrical_offset_deg: f32,
    pub torque_sign: f32,
}

impl HallElectricalCalibration {
    pub fn electrical_angle_deg(&self, hall_angle_deg: f32) -> f32 {
        wrap_degrees(
            self.direction_sign * MOTOR_POLE_PAIRS * hall_angle_deg + self.electrical_offset_deg,
        )
    }
}

impl From<StoredMotorCalibration> for HallElectricalCalibration {
    fn from(value: StoredMotorCalibration) -> Self {
        Self {
            direction_sign: value.direction_sign,
            electrical_offset_deg: value.electrical_offset_deg,
            torque_sign: value.torque_sign,
        }
    }
}

impl From<HallElectricalCalibration> for StoredMotorCalibration {
    fn from(value: HallElectricalCalibration) -> Self {
        Self {
            direction_sign: value.direction_sign,
            electrical_offset_deg: value.electrical_offset_deg,
            torque_sign: value.torque_sign,
        }
    }
}

pub trait ControlDrive {
    fn enable(&mut self);
    fn disable(&mut self);
    fn coast(&mut self);
    fn set_phase_voltages(&mut self, ua_v: f32, ub_v: f32, uc_v: f32);
}

#[derive(Debug, Clone, Copy)]
pub struct MotorDriveState {
    electrical_angle_deg: f32,
    uq_v: f32,
    motor_enabled: bool,
}

impl MotorDriveState {
    pub fn new() -> Self {
        Self {
            electrical_angle_deg: 0.0,
            uq_v: 0.0,
            motor_enabled: false,
        }
    }

    pub fn reset_runtime(&mut self) {
        self.electrical_angle_deg = 0.0;
        self.uq_v = 0.0;
        self.motor_enabled = false;
    }

    pub fn disable_motor<D: ControlDrive>(&mut self, motor_drive: &mut D) {
        motor_drive.disable();
        motor_drive.coast();
        self.uq_v = 0.0;
        self.motor_enabled = false;
    }

    pub fn apply_output<D: ControlDrive>(
        &mut self,
        motor_drive: &mut D,
        output: &ControllerOutput,
        hall_angle_deg: Option<f32>,
        calibration: Option<HallElectricalCalibration>,
    ) {
        if matches!(
            output.mode,
            PendulumControlMode::WaitingForHall | PendulumControlMode::Startup
        ) {
            self.disable_motor(motor_drive);
            self.electrical_angle_deg = 0.0;
            return;
        }

        if !matches!(
            output.mode,
            PendulumControlMode::Idle | PendulumControlMode::Active
        ) {
            self.disable_motor(motor_drive);
            return;
        }

        let (Some(hall_angle_deg), Some(calibration)) = (hall_angle_deg, calibration) else {
            self.disable_motor(motor_drive);
            return;
        };

        let electrical_angle_deg = calibration.electrical_angle_deg(hall_angle_deg);
        if matches!(output.mode, PendulumControlMode::Idle) {
            let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
                0.0,
                degrees_to_radians(electrical_angle_deg),
                VOLTAGE_LIMIT_V,
            );
            motor_drive.enable();
            motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);
            self.electrical_angle_deg = electrical_angle_deg;
            self.uq_v = 0.0;
            self.motor_enabled = true;
            return;
        }

        let uq_v = -VOLTAGE_LIMIT_V * output.drive_command * calibration.torque_sign;
        let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
            uq_v,
            degrees_to_radians(electrical_angle_deg),
            VOLTAGE_LIMIT_V,
        );
        motor_drive.enable();
        motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);
        self.electrical_angle_deg = electrical_angle_deg;
        self.uq_v = uq_v;
        self.motor_enabled = true;
    }

    pub fn to_telemetry(
        &self,
        output: &ControllerOutput,
        calibration: Option<HallElectricalCalibration>,
    ) -> PendulumControlTelemetry {
        let (direction_sign, torque_sign) = if let Some(calibration) = calibration {
            (calibration.direction_sign, calibration.torque_sign)
        } else {
            (0.0, 0.0)
        };

        PendulumControlTelemetry {
            mode: output.mode,
            theta_error_deg: output.theta_error_deg,
            torque_command_nm: output.torque_command_nm,
            raw_drive_command: output.raw_drive_command,
            drive_command: output.drive_command,
            direction_sign,
            torque_sign,
            electrical_angle_deg: self.electrical_angle_deg,
            uq_v: self.uq_v,
            wheel_angle_deg: output.wheel_angle_deg,
            wheel_speed_dps: output.wheel_speed_dps,
            commutation_step: electrical_sector(self.electrical_angle_deg),
            commutation_center_deg: sector_center_deg(self.electrical_angle_deg),
            motor_enabled: self.motor_enabled,
        }
    }
}

pub fn step_controller(
    controller: &mut PendulumController,
    hall_angle_deg: Option<f32>,
    theta_deg: Option<f32>,
    theta_dot_dps: Option<f32>,
    max_phase_current_a: f32,
    actuator_ready: bool,
) -> ControllerOutput {
    controller.step(ControllerInput {
        hall_angle_deg,
        theta_deg,
        theta_dot_dps,
        max_phase_current_a,
        actuator_ready,
    })
}

pub fn run_control_loop<D: ControlDrive>(
    controller: &mut PendulumController,
    drive_state: &mut MotorDriveState,
    motor_drive: &mut D,
    calibration: Option<HallElectricalCalibration>,
    hall: &HallTelemetry,
    estimate: &PendulumEstimateTelemetry,
    max_phase_current_a: f32,
) -> PendulumControlTelemetry {
    let hall_measurement = match *hall {
        HallTelemetry::Measurement(measurement) => Some(measurement),
        _ => None,
    };
    let estimate_measurement = match *estimate {
        PendulumEstimateTelemetry::Measurement(measurement) => Some(measurement),
        _ => None,
    };

    let output = step_controller(
        controller,
        hall_measurement.map(|measurement| measurement.angle_deg),
        estimate_measurement.map(|measurement| measurement.theta_deg),
        estimate_measurement.map(|measurement| measurement.theta_dot_dps),
        max_phase_current_a,
        calibration.is_some(),
    );

    drive_state.apply_output(
        motor_drive,
        &output,
        hall_measurement.map(|measurement| measurement.angle_deg),
        calibration,
    );
    drive_state.to_telemetry(&output, calibration)
}

pub fn max_phase_current_amps(ina_u_a: f32, ina_v_a: f32, ina_w_a: f32) -> f32 {
    let uv = if ina_u_a.abs() > ina_v_a.abs() {
        ina_u_a.abs()
    } else {
        ina_v_a.abs()
    };
    if uv > ina_w_a.abs() {
        uv
    } else {
        ina_w_a.abs()
    }
}

pub fn simplefoc_sine_pwm_phase_voltages(
    uq_v: f32,
    angle_el_rad: f32,
    voltage_limit_v: f32,
) -> (f32, f32, f32) {
    let ualpha = -sinf(angle_el_rad) * uq_v;
    let ubeta = sinf(angle_el_rad + core::f32::consts::FRAC_PI_2) * uq_v;

    let mut ua = ualpha;
    let mut ub = -0.5 * ualpha + 0.866_025_4 * ubeta;
    let mut uc = -0.5 * ualpha - 0.866_025_4 * ubeta;

    let center = voltage_limit_v * 0.5;
    ua += center;
    ub += center;
    uc += center;

    (
        clamp(ua, 0.0, voltage_limit_v),
        clamp(ub, 0.0, voltage_limit_v),
        clamp(uc, 0.0, voltage_limit_v),
    )
}

fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

fn degrees_to_radians(degrees: f32) -> f32 {
    degrees * core::f32::consts::PI / 180.0
}

fn wrap_degrees(degrees: f32) -> f32 {
    let mut wrapped = degrees;
    while wrapped <= -180.0 {
        wrapped += 360.0;
    }
    while wrapped > 180.0 {
        wrapped -= 360.0;
    }
    wrapped
}

fn electrical_sector(electrical_angle_deg: f32) -> u8 {
    ((wrap_degrees(electrical_angle_deg) / 60.0) as u8) % 6
}

fn sector_center_deg(electrical_angle_deg: f32) -> f32 {
    electrical_sector(electrical_angle_deg) as f32 * 60.0 + 30.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simplefoc_voltage_limits_outputs() {
        let (ua, ub, uc) = simplefoc_sine_pwm_phase_voltages(2.0, 0.3, 4.6);
        assert!(ua >= 0.0 && ua <= 4.6);
        assert!(ub >= 0.0 && ub <= 4.6);
        assert!(uc >= 0.0 && uc <= 4.6);
    }

    #[test]
    fn max_phase_current_amps_uses_absolute_max() {
        assert_eq!(max_phase_current_amps(0.5, -1.0, 0.3), 1.0);
    }
}
