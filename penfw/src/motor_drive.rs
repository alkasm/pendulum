use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    mcpwm::{
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
    },
    peripherals::{GPIO5, MCPWM0},
};
use libm::sinf;
use pendulum_lib::{PendulumControlMode, PendulumControlTelemetry, controller::ControllerOutput};

use crate::{
    math::{clamp, degrees_to_radians, wrap_degrees},
    motor_calibration::HallElectricalCalibration,
};

pub const PWM_PERIOD_TICKS: u16 = 2500;
const DEAD_ZONE: f32 = 0.02;
const VOLTAGE_POWER_SUPPLY_V: f32 = 5.0;
pub const VOLTAGE_LIMIT_V: f32 = 4.6;

pub type PwmPinA<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, true>;
pub type PwmPinB<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, false>;

pub struct PwmMotorDrive<'a> {
    enable: Output<'a>,
    uh: PwmPinA<'a, 0>,
    ul: PwmPinB<'a, 0>,
    vh: PwmPinA<'a, 1>,
    vl: PwmPinB<'a, 1>,
    wh: PwmPinA<'a, 2>,
    wl: PwmPinB<'a, 2>,
}

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

    pub fn disable_motor(&mut self, motor_drive: &mut PwmMotorDrive<'_>) {
        motor_drive.disable();
        motor_drive.coast();
        self.uq_v = 0.0;
        self.motor_enabled = false;
    }

    pub fn apply_output(
        &mut self,
        motor_drive: &mut PwmMotorDrive<'_>,
        output: &ControllerOutput,
        hall_angle_deg: Option<f32>,
        calibration: Option<HallElectricalCalibration>,
    ) {
        if matches!(output.mode, PendulumControlMode::WaitingForHall | PendulumControlMode::Startup)
        {
            self.disable_motor(motor_drive);
            self.electrical_angle_deg = 0.0;
            return;
        }

        if !matches!(output.mode, PendulumControlMode::Idle | PendulumControlMode::Active) {
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

impl<'a> PwmMotorDrive<'a> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        enable: GPIO5<'a>,
        uh: PwmPinA<'a, 0>,
        ul: PwmPinB<'a, 0>,
        vh: PwmPinA<'a, 1>,
        vl: PwmPinB<'a, 1>,
        wh: PwmPinA<'a, 2>,
        wl: PwmPinB<'a, 2>,
    ) -> Self {
        Self {
            enable: Output::new(enable, Level::Low, OutputConfig::default()),
            uh,
            ul,
            vh,
            vl,
            wh,
            wl,
        }
    }

    pub fn enable(&mut self) {
        self.enable.set_high();
    }

    pub fn disable(&mut self) {
        self.enable.set_low();
    }

    pub fn coast(&mut self) {
        self.uh.set_timestamp(0);
        self.vh.set_timestamp(0);
        self.wh.set_timestamp(0);
        self.ul.set_timestamp(PWM_PERIOD_TICKS);
        self.vl.set_timestamp(PWM_PERIOD_TICKS);
        self.wl.set_timestamp(PWM_PERIOD_TICKS);
    }

    pub fn set_phase_voltages(&mut self, ua_v: f32, ub_v: f32, uc_v: f32) {
        let dead = DEAD_ZONE * 0.5;
        let dc_a = clamp(ua_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_b = clamp(ub_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_c = clamp(uc_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);

        self.uh.set_timestamp(duty_to_ticks(dc_a - dead));
        self.ul.set_timestamp(duty_to_ticks(dc_a + dead));
        self.vh.set_timestamp(duty_to_ticks(dc_b - dead));
        self.vl.set_timestamp(duty_to_ticks(dc_b + dead));
        self.wh.set_timestamp(duty_to_ticks(dc_c - dead));
        self.wl.set_timestamp(duty_to_ticks(dc_c + dead));
    }
}

pub fn low_side_pwm_config() -> PwmPinConfig<false> {
    PwmPinConfig::new(
        PwmActions::<false>::empty()
            .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
            .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
        PwmUpdateMethod::SYNC_ON_ZERO,
    )
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

fn duty_to_ticks(duty: f32) -> u16 {
    let clamped = clamp(duty, 0.0, 1.0);
    (clamped * PWM_PERIOD_TICKS as f32 + 0.5) as u16
}

fn electrical_sector(electrical_angle_deg: f32) -> u8 {
    ((wrap_degrees(electrical_angle_deg) / 60.0) as u8) % 6
}

fn sector_center_deg(electrical_angle_deg: f32) -> f32 {
    electrical_sector(electrical_angle_deg) as f32 * 60.0 + 30.0
}
