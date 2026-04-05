#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "bringup.rs"]
mod bringup;
#[path = "hw/mod.rs"]
mod hw;

use bringup::{
    HALL_SENSOR_ADDR, i2c_device_present, init_console, init_delay, init_primary_i2c,
    max_clock_config, write_bytes,
};
use esp_hal::{
    Blocking,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::I2c,
    main,
    mcpwm::{
        McPwm, PeripheralClockConfig,
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
        timer::PwmWorkingMode,
    },
    peripherals::{GPIO5, GPIO34, MCPWM0},
    time::Rate,
};
use hw::{CurrentSample, CurrentSensor, GY521_DEFAULT_I2C_ADDR};
use libm::{atan2f, sinf, sqrtf};
use pendulum_lib::{
    config::default_pendulum,
    pendulum::{BodyAxis3, ImuAxesInBody, PendulumGeometry},
};
use penproto::{
    CurrentTelemetry, HallMeasurement, HallTelemetry, PendulumControlMode, PendulumControlTelemetry,
    PendulumEstimateMeasurement, PendulumEstimateTelemetry, PendulumTelemetryFrame, TelemetryPacket,
};
use uom::si::length::millimeter;

const CONTROL_PERIOD_MS: u32 = 5;
const FRAME_BUF_LEN: usize = 512;
const BASELINE_SAMPLES: u32 = 64;
const MAX_COMMAND_TORQUE_NM: f32 = 0.030;
const MAX_PHASE_CURRENT_A: f32 = 1.2;
const ARM_ANGLE_DEG: f32 = 25.0;
const ARM_RATE_DPS: f32 = 30.0;
const ARM_SAMPLE_TARGET: u16 = 3;
const CAPTURE_ANGLE_DEG: f32 = 60.0;
const DRIVE_DEADBAND: f32 = 0.02;
const DRIVE_IDLE_EPSILON: f32 = 0.005;
const MAX_DRIVE_STEP_PER_TICK: f32 = 0.20;
const MAX_DRIVE_REVERSAL_STEP_PER_TICK: f32 = 0.60;
const KP_NM_PER_RAD: f32 = 0.45;
const KD_NM_PER_RAD_S: f32 = 0.035;
const KWHEEL_NM_PER_RAD_S: f32 = 0.0001;

const PWM_FREQUENCY_HZ: u32 = 32_000;
const PWM_PERIOD_TICKS: u16 = 2500;
const DEAD_ZONE: f32 = 0.02;
const VOLTAGE_POWER_SUPPLY_V: f32 = 5.0;
const VOLTAGE_LIMIT_V: f32 = 3.6;
const MOTOR_POLE_PAIRS: f32 = 7.0;
const CALIBRATION_VOLTAGE_V: f32 = 1.2;
const CALIBRATION_WHEEL_SPEED_DPS: f32 = -180.0;
const CALIBRATION_TOTAL_LOOPS: u32 = 800;
const CALIBRATION_SETTLE_LOOPS: u32 = 120;
const MIN_CALIBRATION_HALL_TRAVEL_DEG: f32 = 180.0;
const PHASE_SEARCH_UQ_V: f32 = 0.9;
const PHASE_SEARCH_LOOPS: u32 = 140;
const PHASE_SEARCH_SETTLE_LOOPS: u32 = 30;
const PHASE_SEARCH_OFFSETS_DEG: [f32; 4] = [0.0, 90.0, 180.0, 270.0];

const TMAG5273_REG_DEVICE_CONFIG_1: u8 = 0x00;
const TMAG5273_REG_DEVICE_CONFIG_2: u8 = 0x01;
const TMAG5273_REG_SENSOR_CONFIG_1: u8 = 0x02;
const TMAG5273_REG_SENSOR_CONFIG_2: u8 = 0x03;
const TMAG5273_REG_T_CONFIG: u8 = 0x07;
const TMAG5273_REG_T_MSB_RESULT: u8 = 0x10;
const TMAG5273_RANGE_MT: f32 = 80.0;
const TMAG5273_TEMP_SENSE_T0_C: f32 = 25.0;
const TMAG5273_TEMP_ADC_T0: i16 = 17_508;
const TMAG5273_TEMP_ADC_RES: f32 = 60.1;

const MPU_REG_ACCEL_XOUT_H: u8 = 0x3B;
const MPU_REG_PWR_MGMT_1: u8 = 0x6B;
const MPU_REG_WHO_AM_I: u8 = 0x75;
const MPU6050_WHO_AM_I_VALUE: u8 = 0x68;
const ACCEL_LSB_PER_G: f32 = 16_384.0;
const GYRO_LSB_PER_DPS: f32 = 131.0;
const ACCEL_CORRECTION_GAIN: f32 = 0.08;
const MAX_ACCEL_CORRECTION_STEP_DEG: f32 = 2.0;
const MAX_ACCEL_CORRECTION_ERROR_DEG: f32 = 35.0;
const MIN_ACCEL_GRAVITY_G: f32 = 0.75;
const MAX_ACCEL_GRAVITY_G: f32 = 1.25;

#[derive(Clone, Copy)]
enum ImuProbeError {
    RegisterRead,
    UnexpectedWhoAmI(u8),
}

#[derive(Clone, Copy)]
struct Point3Mm {
    x: f32,
    y: f32,
}

#[derive(Clone, Copy, Default)]
struct Vector3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Vector3 {
    fn norm(self) -> f32 {
        sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

type PwmPinA<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, true>;
type PwmPinB<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, false>;

struct PwmMotorDrive<'a> {
    enable: Output<'a>,
    diag: Input<'a>,
    uh: PwmPinA<'a, 0>,
    ul: PwmPinB<'a, 0>,
    vh: PwmPinA<'a, 1>,
    vl: PwmPinB<'a, 1>,
    wh: PwmPinA<'a, 2>,
    wl: PwmPinB<'a, 2>,
}

#[derive(Clone, Copy)]
struct HallElectricalCalibration {
    direction_sign: f32,
    electrical_offset_deg: f32,
    torque_sign: f32,
}

struct ControlState {
    armed: bool,
    arm_ready_samples: u16,
    last_wheel_angle_deg: Option<f32>,
    last_unwrapped_wheel_angle_deg: Option<f32>,
    filtered_wheel_speed_dps: f32,
    filtered_drive_command: f32,
    electrical_angle_deg: f32,
    uq_v: f32,
    motor_enabled: bool,
}

struct ImuEstimatorState {
    last_theta_dot_dps: Option<f32>,
    filtered_theta_ddot_dps2: f32,
    filtered_theta_deg: Option<f32>,
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let mut current_sensor = CurrentSensor::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
    );
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(160))
        .expect("failed to configure MCPWM clock");
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm.operator2.set_timer(&mcpwm.timer0);
    let (uh, ul) = mcpwm.operator0.with_pins(
        peripherals.GPIO16,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO17,
        low_side_pwm_config(),
    );
    let (vh, vl) = mcpwm.operator1.with_pins(
        peripherals.GPIO18,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO23,
        low_side_pwm_config(),
    );
    let (wh, wl) = mcpwm.operator2.with_pins(
        peripherals.GPIO19,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO33,
        low_side_pwm_config(),
    );
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            PWM_PERIOD_TICKS,
            PwmWorkingMode::UpDown,
            Rate::from_hz(PWM_FREQUENCY_HZ),
        )
        .expect("failed to configure MCPWM timer");
    mcpwm.timer0.start(timer_clock_cfg);
    let mut motor_drive = PwmMotorDrive::new(
        peripherals.GPIO5,
        peripherals.GPIO34,
        uh,
        ul,
        vh,
        vl,
        wh,
        wl,
    );
    let mut i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let mut frame_buf = [0_u8; FRAME_BUF_LEN];
    let mut seq = 0_u32;
    let mut hall_configured = false;
    let mut imu_verified = false;
    let mut imu_awake = false;
    let mut imu_estimator = ImuEstimatorState::new();
    let mut control_state = ControlState::new();
    let pendulum = default_pendulum();

    let _ = current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    motor_drive.disable();
    motor_drive.coast();
    let actuator_calibration =
        calibrate_hall_electrical_cycle(&mut i2c, &mut hall_configured, &delay, &mut motor_drive)
            .and_then(|calibration| {
                refine_torque_phase_offset(
                    &mut i2c,
                    &mut hall_configured,
                    &delay,
                    &mut motor_drive,
                    calibration,
                )
                .or(Some(calibration))
            });

    loop {
        let current_sample = current_sensor.read();
        let current = current_telemetry_from_sample(current_sample);
        let hall = read_hall_telemetry(&mut i2c, &mut hall_configured);
        let estimate = read_pendulum_estimate(
            &mut i2c,
            &mut imu_verified,
            &mut imu_awake,
            &mut imu_estimator,
            &pendulum.geometry,
        );
        let control = update_control_loop(
            &mut control_state,
            &mut motor_drive,
            actuator_calibration,
            &hall,
            &estimate,
            &current_sample,
        );

        let frame = PendulumTelemetryFrame {
            seq,
            uptime_ms: seq.saturating_mul(CONTROL_PERIOD_MS),
            motor_driver_diag_high: motor_drive.diag_is_high(),
            current,
            hall,
            estimate,
            control,
        };
        let packet = TelemetryPacket::Pendulum(frame);

        if let Ok(encoded) = postcard::to_slice_cobs(&packet, &mut frame_buf) {
            write_bytes(&mut serial, encoded);
        }

        seq = seq.wrapping_add(1);
        delay.delay_millis(CONTROL_PERIOD_MS);
    }
}

impl ControlState {
    fn new() -> Self {
        Self {
            armed: false,
            arm_ready_samples: 0,
            last_wheel_angle_deg: None,
            last_unwrapped_wheel_angle_deg: None,
            filtered_wheel_speed_dps: 0.0,
            filtered_drive_command: 0.0,
            electrical_angle_deg: 0.0,
            uq_v: 0.0,
            motor_enabled: false,
        }
    }

    fn disable_motor(&mut self, motor_drive: &mut PwmMotorDrive<'_>) {
        motor_drive.disable();
        motor_drive.coast();
        self.filtered_drive_command = 0.0;
        self.uq_v = 0.0;
        self.motor_enabled = false;
    }

    fn observe_wheel(&mut self, angle_deg: f32) -> f32 {
        let instant_speed_dps = self
            .last_wheel_angle_deg
            .map(|previous| {
                wrap_angle_delta_deg(angle_deg - previous) / (CONTROL_PERIOD_MS as f32 / 1_000.0)
            })
            .unwrap_or(0.0);
        self.filtered_wheel_speed_dps = 0.8 * self.filtered_wheel_speed_dps + 0.2 * instant_speed_dps;
        self.last_wheel_angle_deg = Some(angle_deg);
        self.last_unwrapped_wheel_angle_deg = Some(match self.last_unwrapped_wheel_angle_deg {
            Some(previous) => unwrap_near(previous, angle_deg),
            None => angle_deg,
        });
        self.filtered_wheel_speed_dps
    }

    fn reset_wheel_observer(&mut self) {
        self.last_wheel_angle_deg = None;
        self.last_unwrapped_wheel_angle_deg = None;
        self.filtered_wheel_speed_dps = 0.0;
    }

    fn slew_drive_command(&mut self, target: f32) -> f32 {
        let changing_direction =
            self.filtered_drive_command.abs() > DRIVE_IDLE_EPSILON
                && target.abs() > DRIVE_IDLE_EPSILON
                && self.filtered_drive_command.signum() != target.signum();
        let max_step = if changing_direction {
            MAX_DRIVE_REVERSAL_STEP_PER_TICK
        } else {
            MAX_DRIVE_STEP_PER_TICK
        };
        let delta = clamp(
            target - self.filtered_drive_command,
            -max_step,
            max_step,
        );
        self.filtered_drive_command += delta;
        self.filtered_drive_command
    }

    fn reset_arming(&mut self) {
        self.armed = false;
        self.arm_ready_samples = 0;
    }

    fn step_arming(&mut self, theta_deg: f32, theta_dot_dps: f32) -> bool {
        if self.armed {
            return true;
        }

        let inside_arm_window =
            theta_deg.abs() <= ARM_ANGLE_DEG && theta_dot_dps.abs() <= ARM_RATE_DPS;

        if inside_arm_window {
            self.arm_ready_samples = self.arm_ready_samples.saturating_add(1);
            if self.arm_ready_samples >= ARM_SAMPLE_TARGET {
                self.armed = true;
                return true;
            }
        } else {
            self.arm_ready_samples = 0;
        }

        false
    }
}

impl ImuEstimatorState {
    fn new() -> Self {
        Self {
            last_theta_dot_dps: None,
            filtered_theta_ddot_dps2: 0.0,
            filtered_theta_deg: None,
        }
    }

    fn reset(&mut self) {
        self.last_theta_dot_dps = None;
        self.filtered_theta_ddot_dps2 = 0.0;
        self.filtered_theta_deg = None;
    }

    fn observe_theta_dot(&mut self, theta_dot_dps: f32) -> f32 {
        let instant_theta_ddot_dps2 = self
            .last_theta_dot_dps
            .map(|previous| (theta_dot_dps - previous) / dt_s())
            .unwrap_or(0.0);
        self.last_theta_dot_dps = Some(theta_dot_dps);
        self.filtered_theta_ddot_dps2 =
            0.85 * self.filtered_theta_ddot_dps2 + 0.15 * instant_theta_ddot_dps2;
        self.filtered_theta_ddot_dps2
    }

    fn observe_theta_from_accel(
        &mut self,
        theta_dot_dps: f32,
        accel_theta_deg: f32,
        accel_gravity_magnitude_g: f32,
    ) -> f32 {
        let predicted_theta_deg = self
            .filtered_theta_deg
            .map(|theta_deg| theta_deg + theta_dot_dps * dt_s())
            .unwrap_or(accel_theta_deg);
        let accel_error_deg = wrap_angle_delta_deg(accel_theta_deg - predicted_theta_deg);
        let accel_is_reliable = accel_gravity_magnitude_g >= MIN_ACCEL_GRAVITY_G
            && accel_gravity_magnitude_g <= MAX_ACCEL_GRAVITY_G
            && accel_error_deg.abs() <= MAX_ACCEL_CORRECTION_ERROR_DEG;
        let correction_deg = if accel_is_reliable {
            clamp(
                accel_error_deg * ACCEL_CORRECTION_GAIN,
                -MAX_ACCEL_CORRECTION_STEP_DEG,
                MAX_ACCEL_CORRECTION_STEP_DEG,
            )
        } else {
            0.0
        };
        let theta_deg = wrap_signed_degrees(predicted_theta_deg + correction_deg);
        self.filtered_theta_deg = Some(theta_deg);
        theta_deg
    }
}

fn update_control_loop(
    control_state: &mut ControlState,
    motor_drive: &mut PwmMotorDrive<'_>,
    actuator_calibration: Option<HallElectricalCalibration>,
    hall: &HallTelemetry,
    estimate: &PendulumEstimateTelemetry,
    current_sample: &CurrentSample,
) -> PendulumControlTelemetry {
    let hall_measurement = match *hall {
        HallTelemetry::Measurement(measurement) => Some(measurement),
        _ => None,
    };
    let estimate_measurement = match *estimate {
        PendulumEstimateTelemetry::Measurement(measurement) => Some(measurement),
        _ => None,
    };

    let wheel_angle_deg = hall_measurement.map(|measurement| measurement.angle_deg).unwrap_or(0.0);
    let wheel_speed_dps = if let Some(measurement) = hall_measurement {
        control_state.observe_wheel(measurement.angle_deg)
    } else {
        control_state.reset_wheel_observer();
        0.0
    };

    if hall_measurement.is_none() {
        control_state.reset_arming();
        control_state.disable_motor(motor_drive);
        control_state.electrical_angle_deg = 0.0;
        return PendulumControlTelemetry {
            mode: PendulumControlMode::WaitingForHall,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            electrical_angle_deg: control_state.electrical_angle_deg,
            uq_v: control_state.uq_v,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: 0,
            commutation_center_deg: 0.0,
            motor_enabled: control_state.motor_enabled,
        };
    }

    let hall_measurement = hall_measurement.expect("hall measurement checked above");
    let actuator_calibration = if let Some(calibration) = actuator_calibration {
        calibration
    } else {
        control_state.reset_arming();
        control_state.disable_motor(motor_drive);
        control_state.electrical_angle_deg = 0.0;
        return PendulumControlTelemetry {
            mode: PendulumControlMode::Startup,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            electrical_angle_deg: control_state.electrical_angle_deg,
            uq_v: control_state.uq_v,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: 0,
            commutation_center_deg: 0.0,
            motor_enabled: control_state.motor_enabled,
        };
    };

    let estimate_measurement = if let Some(measurement) = estimate_measurement {
        measurement
    } else {
        control_state.reset_arming();
        control_state.disable_motor(motor_drive);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::WaitingForImu,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            electrical_angle_deg: control_state.electrical_angle_deg,
            uq_v: control_state.uq_v,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: electrical_sector(control_state.electrical_angle_deg),
            commutation_center_deg: sector_center_deg(control_state.electrical_angle_deg),
            motor_enabled: control_state.motor_enabled,
        };
    };

    if !control_state.step_arming(
        estimate_measurement.theta_deg,
        estimate_measurement.theta_dot_dps,
    ) {
        control_state.disable_motor(motor_drive);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::Arming,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            electrical_angle_deg: control_state.electrical_angle_deg,
            uq_v: control_state.uq_v,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: electrical_sector(control_state.electrical_angle_deg),
            commutation_center_deg: sector_center_deg(control_state.electrical_angle_deg),
            motor_enabled: control_state.motor_enabled,
        };
    }

    if estimate_measurement.theta_deg.abs() > CAPTURE_ANGLE_DEG {
        control_state.disable_motor(motor_drive);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::CaptureOutOfRange,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            electrical_angle_deg: control_state.electrical_angle_deg,
            uq_v: control_state.uq_v,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: electrical_sector(control_state.electrical_angle_deg),
            commutation_center_deg: sector_center_deg(control_state.electrical_angle_deg),
            motor_enabled: control_state.motor_enabled,
        };
    }

    if max_phase_current_amps(current_sample) > MAX_PHASE_CURRENT_A {
        control_state.disable_motor(motor_drive);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::CurrentLimited,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            electrical_angle_deg: control_state.electrical_angle_deg,
            uq_v: control_state.uq_v,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: electrical_sector(control_state.electrical_angle_deg),
            commutation_center_deg: sector_center_deg(control_state.electrical_angle_deg),
            motor_enabled: control_state.motor_enabled,
        };
    }

    let torque_command_nm = pd_torque_command_nm(
        estimate_measurement.theta_deg,
        estimate_measurement.theta_dot_dps,
        wheel_speed_dps,
    );
    let raw_drive_command = clamp(torque_command_nm / MAX_COMMAND_TORQUE_NM, -1.0, 1.0);
    let drive_target = if raw_drive_command.abs() < DRIVE_DEADBAND {
        0.0
    } else {
        raw_drive_command
    };
    let drive_command = control_state.slew_drive_command(drive_target);

    if drive_command.abs() < DRIVE_IDLE_EPSILON {
        control_state.disable_motor(motor_drive);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::Idle,
            torque_command_nm,
            drive_command,
            electrical_angle_deg: control_state.electrical_angle_deg,
            uq_v: control_state.uq_v,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: electrical_sector(control_state.electrical_angle_deg),
            commutation_center_deg: sector_center_deg(control_state.electrical_angle_deg),
            motor_enabled: control_state.motor_enabled,
        };
    }

    let electrical_angle_deg = actuator_calibration.electrical_angle_deg(hall_measurement.angle_deg);
    let uq_v = -VOLTAGE_LIMIT_V * drive_command * actuator_calibration.torque_sign;
    let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
        uq_v,
        degrees_to_radians(electrical_angle_deg),
        VOLTAGE_LIMIT_V,
    );
    motor_drive.enable();
    motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);
    control_state.electrical_angle_deg = electrical_angle_deg;
    control_state.uq_v = uq_v;
    control_state.motor_enabled = true;

    PendulumControlTelemetry {
        mode: PendulumControlMode::Active,
        torque_command_nm,
        drive_command,
        electrical_angle_deg,
        uq_v,
        wheel_angle_deg,
        wheel_speed_dps,
        commutation_step: electrical_sector(electrical_angle_deg),
        commutation_center_deg: sector_center_deg(electrical_angle_deg),
        motor_enabled: control_state.motor_enabled,
    }
}

impl<'a> PwmMotorDrive<'a> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        enable: GPIO5<'a>,
        diag: GPIO34<'a>,
        uh: PwmPinA<'a, 0>,
        ul: PwmPinB<'a, 0>,
        vh: PwmPinA<'a, 1>,
        vl: PwmPinB<'a, 1>,
        wh: PwmPinA<'a, 2>,
        wl: PwmPinB<'a, 2>,
    ) -> Self {
        Self {
            enable: Output::new(enable, Level::Low, OutputConfig::default()),
            diag: Input::new(diag, InputConfig::default()),
            uh,
            ul,
            vh,
            vl,
            wh,
            wl,
        }
    }

    fn enable(&mut self) {
        self.enable.set_high();
    }

    fn disable(&mut self) {
        self.enable.set_low();
    }

    fn diag_is_high(&self) -> bool {
        self.diag.is_high()
    }

    fn coast(&mut self) {
        self.uh.set_timestamp(0);
        self.vh.set_timestamp(0);
        self.wh.set_timestamp(0);
        self.ul.set_timestamp(PWM_PERIOD_TICKS);
        self.vl.set_timestamp(PWM_PERIOD_TICKS);
        self.wl.set_timestamp(PWM_PERIOD_TICKS);
    }

    fn set_phase_voltages(&mut self, ua_v: f32, ub_v: f32, uc_v: f32) {
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

impl HallElectricalCalibration {
    fn electrical_angle_deg(&self, hall_angle_deg: f32) -> f32 {
        wrap_degrees(
            self.direction_sign * MOTOR_POLE_PAIRS * hall_angle_deg + self.electrical_offset_deg,
        )
    }
}

fn calibrate_hall_electrical_cycle(
    i2c: &mut I2c<'_, Blocking>,
    hall_configured: &mut bool,
    delay: &esp_hal::delay::Delay,
    motor_drive: &mut PwmMotorDrive<'_>,
) -> Option<HallElectricalCalibration> {
    motor_drive.enable();

    let mut open_loop_shaft_deg = 0.0_f32;
    let mut start_hall_unwrapped_deg = None;
    let mut end_hall_unwrapped_deg = None;
    let mut start_electrical_unwrapped_deg = None;
    let mut end_electrical_unwrapped_deg = None;
    let mut pos_offset_sin_sum = 0.0_f32;
    let mut pos_offset_cos_sum = 0.0_f32;
    let mut neg_offset_sin_sum = 0.0_f32;
    let mut neg_offset_cos_sum = 0.0_f32;
    let mut sample_count = 0_u32;
    let mut last_hall_unwrapped_deg = None;

    let mut loop_index = 0_u32;
    while loop_index < CALIBRATION_TOTAL_LOOPS {
        open_loop_shaft_deg += CALIBRATION_WHEEL_SPEED_DPS * dt_s();
        let electrical_unwrapped_deg = MOTOR_POLE_PAIRS * open_loop_shaft_deg;
        let electrical_angle_deg = wrap_degrees(electrical_unwrapped_deg);
        let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
            CALIBRATION_VOLTAGE_V,
            degrees_to_radians(electrical_angle_deg),
            VOLTAGE_LIMIT_V,
        );
        motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);

        if let HallTelemetry::Measurement(measurement) = read_hall_telemetry(i2c, hall_configured) {
            let hall_unwrapped_deg = match last_hall_unwrapped_deg {
                Some(previous) => unwrap_near(previous, measurement.angle_deg),
                None => measurement.angle_deg,
            };
            last_hall_unwrapped_deg = Some(hall_unwrapped_deg);

            if loop_index >= CALIBRATION_SETTLE_LOOPS {
                if start_hall_unwrapped_deg.is_none() {
                    start_hall_unwrapped_deg = Some(hall_unwrapped_deg);
                    start_electrical_unwrapped_deg = Some(electrical_unwrapped_deg);
                }
                end_hall_unwrapped_deg = Some(hall_unwrapped_deg);
                end_electrical_unwrapped_deg = Some(electrical_unwrapped_deg);

                let pos_offset_deg =
                    wrap_degrees(electrical_angle_deg - MOTOR_POLE_PAIRS * measurement.angle_deg);
                let neg_offset_deg =
                    wrap_degrees(electrical_angle_deg + MOTOR_POLE_PAIRS * measurement.angle_deg);
                pos_offset_sin_sum += sinf(degrees_to_radians(pos_offset_deg));
                pos_offset_cos_sum += sinf(
                    degrees_to_radians(pos_offset_deg) + core::f32::consts::FRAC_PI_2,
                );
                neg_offset_sin_sum += sinf(degrees_to_radians(neg_offset_deg));
                neg_offset_cos_sum += sinf(
                    degrees_to_radians(neg_offset_deg) + core::f32::consts::FRAC_PI_2,
                );
                sample_count += 1;
            }
        }

        delay.delay_millis(CONTROL_PERIOD_MS);
        loop_index += 1;
    }

    motor_drive.coast();
    delay.delay_millis(200);

    let hall_travel_deg = end_hall_unwrapped_deg? - start_hall_unwrapped_deg?;
    let electrical_travel_deg = end_electrical_unwrapped_deg? - start_electrical_unwrapped_deg?;
    if hall_travel_deg.abs() < MIN_CALIBRATION_HALL_TRAVEL_DEG || sample_count == 0 {
        return None;
    }

    let direction_sign = if hall_travel_deg * electrical_travel_deg >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let electrical_offset_deg = if direction_sign > 0.0 {
        wrap_degrees(atan2f(pos_offset_sin_sum, pos_offset_cos_sum) * (180.0 / core::f32::consts::PI))
    } else {
        wrap_degrees(atan2f(neg_offset_sin_sum, neg_offset_cos_sum) * (180.0 / core::f32::consts::PI))
    };

    Some(HallElectricalCalibration {
        direction_sign,
        electrical_offset_deg,
        torque_sign: 1.0,
    })
}

fn refine_torque_phase_offset(
    i2c: &mut I2c<'_, Blocking>,
    hall_configured: &mut bool,
    delay: &esp_hal::delay::Delay,
    motor_drive: &mut PwmMotorDrive<'_>,
    calibration: HallElectricalCalibration,
) -> Option<HallElectricalCalibration> {
    let mut best_offset_delta_deg = 0.0_f32;
    let mut best_torque_sign = 1.0_f32;
    let mut best_score = f32::NEG_INFINITY;

    for candidate_offset_deg in PHASE_SEARCH_OFFSETS_DEG {
        for candidate_torque_sign in [1.0_f32, -1.0_f32] {
            let pos_travel_deg = measure_phase_search_travel(
                i2c,
                hall_configured,
                delay,
                motor_drive,
                calibration,
                candidate_offset_deg,
                candidate_torque_sign * PHASE_SEARCH_UQ_V,
            )?;
            let neg_travel_deg = measure_phase_search_travel(
                i2c,
                hall_configured,
                delay,
                motor_drive,
                calibration,
                candidate_offset_deg,
                -candidate_torque_sign * PHASE_SEARCH_UQ_V,
            )?;

            let opposite_direction = pos_travel_deg * neg_travel_deg < 0.0;
            let symmetry_penalty = (pos_travel_deg + neg_travel_deg).abs();
            let score = if opposite_direction {
                let weaker_travel_deg = if pos_travel_deg.abs() < neg_travel_deg.abs() {
                    pos_travel_deg.abs()
                } else {
                    neg_travel_deg.abs()
                };
                weaker_travel_deg - 0.25 * symmetry_penalty
            } else {
                -symmetry_penalty
            };

            if score > best_score {
                best_score = score;
                best_offset_delta_deg = candidate_offset_deg;
                best_torque_sign = candidate_torque_sign;
            }
        }
    }

    if !best_score.is_finite() || best_score <= 0.0 {
        return None;
    }

    Some(HallElectricalCalibration {
        direction_sign: calibration.direction_sign,
        electrical_offset_deg: wrap_degrees(
            calibration.electrical_offset_deg + best_offset_delta_deg,
        ),
        torque_sign: calibration.torque_sign * best_torque_sign,
    })
}

fn measure_phase_search_travel(
    i2c: &mut I2c<'_, Blocking>,
    hall_configured: &mut bool,
    delay: &esp_hal::delay::Delay,
    motor_drive: &mut PwmMotorDrive<'_>,
    calibration: HallElectricalCalibration,
    candidate_offset_deg: f32,
    uq_v: f32,
) -> Option<f32> {
    motor_drive.enable();
    let mut start_unwrapped_deg = None;
    let mut end_unwrapped_deg = None;
    let mut last_unwrapped_deg = None;

    let mut loop_index = 0_u32;
    while loop_index < PHASE_SEARCH_LOOPS {
        if let HallTelemetry::Measurement(measurement) = read_hall_telemetry(i2c, hall_configured) {
            let hall_unwrapped_deg = match last_unwrapped_deg {
                Some(previous) => unwrap_near(previous, measurement.angle_deg),
                None => measurement.angle_deg,
            };
            last_unwrapped_deg = Some(hall_unwrapped_deg);

            if loop_index >= PHASE_SEARCH_SETTLE_LOOPS {
                if start_unwrapped_deg.is_none() {
                    start_unwrapped_deg = Some(hall_unwrapped_deg);
                }
                end_unwrapped_deg = Some(hall_unwrapped_deg);
            }

            let electrical_angle_deg = wrap_degrees(
                calibration.electrical_angle_deg(measurement.angle_deg) + candidate_offset_deg,
            );
            let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
                uq_v,
                degrees_to_radians(electrical_angle_deg),
                VOLTAGE_LIMIT_V,
            );
            motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);
        }

        delay.delay_millis(CONTROL_PERIOD_MS);
        loop_index += 1;
    }

    motor_drive.coast();
    delay.delay_millis(150);

    match (start_unwrapped_deg, end_unwrapped_deg) {
        (Some(start), Some(end)) => Some(end - start),
        _ => None,
    }
}

fn low_side_pwm_config() -> PwmPinConfig<false> {
    PwmPinConfig::new(
        PwmActions::<false>::empty()
            .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
            .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
        PwmUpdateMethod::SYNC_ON_ZERO,
    )
}

fn simplefoc_sine_pwm_phase_voltages(
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

fn dt_s() -> f32 {
    CONTROL_PERIOD_MS as f32 / 1_000.0
}

fn degrees_to_radians(angle_deg: f32) -> f32 {
    angle_deg * (core::f32::consts::PI / 180.0)
}

fn electrical_sector(electrical_angle_deg: f32) -> u8 {
    ((wrap_degrees(electrical_angle_deg) / 60.0) as u8) % 6
}

fn sector_center_deg(electrical_angle_deg: f32) -> f32 {
    electrical_sector(electrical_angle_deg) as f32 * 60.0 + 30.0
}

fn current_telemetry_from_sample(sample: CurrentSample) -> CurrentTelemetry {
    CurrentTelemetry {
        mcp6021_counts: sample.mcp6021.counts,
        mcp6021_delta_counts: sample.mcp6021.delta_counts,
        mcp6021_volts: sample.mcp6021.volts,
        ina_u_counts: sample.ina_u.counts,
        ina_u_delta_counts: sample.ina_u.delta_counts,
        ina_u_amps: sample.ina_u.amps,
        ina_v_counts: sample.ina_v.counts,
        ina_v_delta_counts: sample.ina_v.delta_counts,
        ina_v_amps: sample.ina_v.amps,
        ina_w_counts: sample.ina_w.counts,
        ina_w_delta_counts: sample.ina_w.delta_counts,
        ina_w_amps: sample.ina_w.amps,
    }
}

fn max_phase_current_amps(sample: &CurrentSample) -> f32 {
    let ina_u = sample.ina_u.amps.abs();
    let ina_v = sample.ina_v.amps.abs();
    let ina_w = sample.ina_w.amps.abs();
    let uv = if ina_u > ina_v { ina_u } else { ina_v };
    if uv > ina_w { uv } else { ina_w }
}

fn pd_torque_command_nm(theta_deg: f32, theta_dot_dps: f32, wheel_speed_dps: f32) -> f32 {
    let theta_rad = theta_deg * (core::f32::consts::PI / 180.0);
    let theta_dot_rad_s = theta_dot_dps * (core::f32::consts::PI / 180.0);
    let wheel_speed_rad_s = wheel_speed_dps * (core::f32::consts::PI / 180.0);
    KP_NM_PER_RAD * theta_rad + KD_NM_PER_RAD_S * theta_dot_rad_s
        - KWHEEL_NM_PER_RAD_S * wheel_speed_rad_s
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

fn read_hall_telemetry(
    i2c: &mut I2c<'_, Blocking>,
    hall_configured: &mut bool,
) -> HallTelemetry {
    if !i2c_device_present(i2c, HALL_SENSOR_ADDR) {
        *hall_configured = false;
        return HallTelemetry::Missing;
    }

    if !*hall_configured {
        match tmag5273_configure_default(i2c, HALL_SENSOR_ADDR) {
            Ok(()) => *hall_configured = true,
            Err(register) => return HallTelemetry::ConfigError { register },
        }
    }

    match tmag5273_read_measurement(i2c, HALL_SENSOR_ADDR) {
        Ok(measurement) => HallTelemetry::Measurement(measurement),
        Err(register) => HallTelemetry::ReadError { register },
    }
}

fn read_pendulum_estimate(
    i2c: &mut I2c<'_, Blocking>,
    imu_verified: &mut bool,
    imu_awake: &mut bool,
    imu_estimator: &mut ImuEstimatorState,
    geometry: &PendulumGeometry,
) -> PendulumEstimateTelemetry {
    if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
        *imu_verified = false;
        *imu_awake = false;
        imu_estimator.reset();
        return PendulumEstimateTelemetry::Missing;
    }

    if !*imu_verified {
        match imu_verify(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_verified = true,
            Err(ImuProbeError::RegisterRead) => {
                imu_estimator.reset();
                return PendulumEstimateTelemetry::Missing;
            }
            Err(ImuProbeError::UnexpectedWhoAmI(value)) => {
                imu_estimator.reset();
                return PendulumEstimateTelemetry::UnexpectedWhoAmI { value };
            }
        }
    }

    if !*imu_awake {
        match imu_wake(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_awake = true,
            Err(register) => {
                imu_estimator.reset();
                return PendulumEstimateTelemetry::WakeError { register };
            }
        }
    }

    match imu_read_pendulum_measurement(i2c, GY521_DEFAULT_I2C_ADDR, imu_estimator, geometry) {
        Ok(measurement) => PendulumEstimateTelemetry::Measurement(measurement),
        Err(register) => {
            imu_estimator.reset();
            PendulumEstimateTelemetry::ReadError { register }
        }
    }
}

fn tmag5273_configure_default(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), u8> {
    tmag5273_update_register(i2c, address, TMAG5273_REG_DEVICE_CONFIG_1, |value| value & !0x03)?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_DEVICE_CONFIG_2, |value| {
        (value & !0x17) | 0x02
    })?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_SENSOR_CONFIG_1, |value| {
        (value & !0xF0) | 0x70
    })?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_SENSOR_CONFIG_2, |value| {
        (value & !0x0F) | 0x07
    })?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_T_CONFIG, |value| value | 0x01)?;
    Ok(())
}

fn tmag5273_update_register(
    i2c: &mut I2c<'_, Blocking>,
    address: u8,
    register: u8,
    update: impl FnOnce(u8) -> u8,
) -> Result<(), u8> {
    let mut value = [0_u8; 1];
    i2c.write_read(address, &[register], &mut value)
        .map_err(|_| register)?;
    let updated = update(value[0]);
    i2c.write(address, &[register, updated]).map_err(|_| register)?;
    Ok(())
}

fn tmag5273_read_measurement(
    i2c: &mut I2c<'_, Blocking>,
    address: u8,
) -> Result<HallMeasurement, u8> {
    let mut buffer = [0_u8; 13];
    i2c.write_read(address, &[TMAG5273_REG_T_MSB_RESULT], &mut buffer)
        .map_err(|_| TMAG5273_REG_T_MSB_RESULT)?;

    Ok(HallMeasurement {
        temperature_c: decode_temperature_c(buffer[0], buffer[1]),
        x_mt: decode_magnetic_mt(buffer[2], buffer[3], TMAG5273_RANGE_MT),
        y_mt: decode_magnetic_mt(buffer[4], buffer[5], TMAG5273_RANGE_MT),
        z_mt: decode_magnetic_mt(buffer[6], buffer[7], TMAG5273_RANGE_MT),
        angle_deg: decode_angle_deg(buffer[9], buffer[10]),
        magnitude: buffer[11],
        set_count: (buffer[8] >> 5) & 0x07,
        result_ready: (buffer[8] & 0x01) != 0,
        por: (buffer[8] & 0x04) != 0,
        diag_fail: (buffer[8] & 0x02) != 0,
        int_pin_high: (buffer[12] & 0x10) != 0,
        oscillator_error: (buffer[12] & 0x08) != 0,
        int_pin_error: (buffer[12] & 0x04) != 0,
        otp_crc_error: (buffer[12] & 0x02) != 0,
        vcc_uv_error: (buffer[12] & 0x01) != 0,
    })
}

fn imu_verify(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), ImuProbeError> {
    let mut who_am_i = [0_u8; 1];
    i2c.write_read(address, &[MPU_REG_WHO_AM_I], &mut who_am_i)
        .map_err(|_| ImuProbeError::RegisterRead)?;
    if who_am_i[0] != MPU6050_WHO_AM_I_VALUE {
        return Err(ImuProbeError::UnexpectedWhoAmI(who_am_i[0]));
    }
    Ok(())
}

fn imu_wake(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), u8> {
    i2c.write(address, &[MPU_REG_PWR_MGMT_1, 0x01])
        .map_err(|_| MPU_REG_PWR_MGMT_1)
}

fn imu_read_pendulum_measurement(
    i2c: &mut I2c<'_, Blocking>,
    address: u8,
    imu_estimator: &mut ImuEstimatorState,
    geometry: &PendulumGeometry,
) -> Result<PendulumEstimateMeasurement, u8> {
    let mut buffer = [0_u8; 14];
    i2c.write_read(address, &[MPU_REG_ACCEL_XOUT_H], &mut buffer)
        .map_err(|_| MPU_REG_ACCEL_XOUT_H)?;

    let ax_raw = i16::from_be_bytes([buffer[0], buffer[1]]);
    let ay_raw = i16::from_be_bytes([buffer[2], buffer[3]]);
    let az_raw = i16::from_be_bytes([buffer[4], buffer[5]]);
    let gx_raw = i16::from_be_bytes([buffer[8], buffer[9]]);
    let gy_raw = i16::from_be_bytes([buffer[10], buffer[11]]);
    let gz_raw = i16::from_be_bytes([buffer[12], buffer[13]]);

    let accel_imu_g = Vector3 {
        x: ax_raw as f32 / ACCEL_LSB_PER_G,
        y: ay_raw as f32 / ACCEL_LSB_PER_G,
        z: az_raw as f32 / ACCEL_LSB_PER_G,
    };
    let gyro_imu_dps = Vector3 {
        x: gx_raw as f32 / GYRO_LSB_PER_DPS,
        y: gy_raw as f32 / GYRO_LSB_PER_DPS,
        z: gz_raw as f32 / GYRO_LSB_PER_DPS,
    };
    let accel_body_g = transform_imu_vector_to_body(accel_imu_g, geometry.imu_mount.axes_in_body);
    let gyro_body_dps = transform_imu_vector_to_body(gyro_imu_dps, geometry.imu_mount.axes_in_body);

    let theta_dot_dps = -gyro_body_dps.z;
    let theta_ddot_dps2 = imu_estimator.observe_theta_dot(theta_dot_dps);
    let pivot_to_imu_body_mm = pivot_to_imu_body_mm(geometry);
    let rotational_specific_force_g = rotational_specific_force_g(
        theta_dot_dps,
        theta_ddot_dps2,
        pivot_to_imu_body_mm,
    );
    let gravity_only_accel_body_g = Vector3 {
        x: accel_body_g.x - rotational_specific_force_g.x,
        y: accel_body_g.y - rotational_specific_force_g.y,
        z: accel_body_g.z - rotational_specific_force_g.z,
    };
    let accel_theta_deg = atan2f(-gravity_only_accel_body_g.x, gravity_only_accel_body_g.y)
        * (180.0 / core::f32::consts::PI);
    let accel_gravity_magnitude_g = gravity_only_accel_body_g.norm();
    let theta_deg = imu_estimator.observe_theta_from_accel(
        theta_dot_dps,
        accel_theta_deg,
        accel_gravity_magnitude_g,
    );

    Ok(PendulumEstimateMeasurement {
        theta_deg,
        theta_dot_dps,
    })
}

fn pivot_to_imu_body_mm(geometry: &PendulumGeometry) -> Point3Mm {
    Point3Mm {
        x: (geometry.motor_mount.center_from_pivot.x + geometry.imu_mount.translation_from_motor.x)
            .get::<millimeter>() as f32,
        y: (geometry.motor_mount.center_from_pivot.y + geometry.imu_mount.translation_from_motor.y)
            .get::<millimeter>() as f32,
    }
}

fn rotational_specific_force_g(
    theta_dot_dps: f32,
    theta_ddot_dps2: f32,
    pivot_to_imu_body_mm: Point3Mm,
) -> Vector3 {
    const G_M_PER_S2: f32 = 9.80665;

    let omega_rad_s = degrees_to_radians(theta_dot_dps);
    let alpha_rad_s2 = degrees_to_radians(theta_ddot_dps2);
    let rx_m = pivot_to_imu_body_mm.x / 1_000.0;
    let ry_m = pivot_to_imu_body_mm.y / 1_000.0;

    Vector3 {
        x: ((-alpha_rad_s2 * ry_m) - (omega_rad_s * omega_rad_s * rx_m)) / G_M_PER_S2,
        y: ((alpha_rad_s2 * rx_m) - (omega_rad_s * omega_rad_s * ry_m)) / G_M_PER_S2,
        z: 0.0,
    }
}

fn transform_imu_vector_to_body(vector: Vector3, axes_in_body: ImuAxesInBody) -> Vector3 {
    let mut body = Vector3::default();
    accumulate_axis_contribution(&mut body, vector.x, axes_in_body.x_axis);
    accumulate_axis_contribution(&mut body, vector.y, axes_in_body.y_axis);
    accumulate_axis_contribution(&mut body, vector.z, axes_in_body.z_axis);
    body
}

fn accumulate_axis_contribution(body: &mut Vector3, value: f32, axis: BodyAxis3) {
    match axis {
        BodyAxis3::Right => body.x += value,
        BodyAxis3::Left => body.x -= value,
        BodyAxis3::Up => body.y += value,
        BodyAxis3::Down => body.y -= value,
        BodyAxis3::TowardViewer => body.z += value,
        BodyAxis3::AwayFromViewer => body.z -= value,
    }
}

fn decode_temperature_c(msb: u8, lsb: u8) -> f32 {
    let raw = i16::from_be_bytes([msb, lsb]);
    TMAG5273_TEMP_SENSE_T0_C + ((raw - TMAG5273_TEMP_ADC_T0) as f32 / TMAG5273_TEMP_ADC_RES)
}

fn decode_magnetic_mt(msb: u8, lsb: u8, range_mt: f32) -> f32 {
    let raw = i16::from_be_bytes([msb, lsb]) as f32;
    (-range_mt * raw) / 32_768.0
}

fn decode_angle_deg(msb: u8, lsb: u8) -> f32 {
    let raw = u16::from_be_bytes([msb, lsb]);
    let integer = ((raw >> 4) & 0x01FF) as f32;
    let fraction = (raw & 0x000F) as f32 / 16.0;
    integer + fraction
}

fn wrap_degrees(angle_deg: f32) -> f32 {
    let mut wrapped = angle_deg;
    while wrapped >= 360.0 {
        wrapped -= 360.0;
    }
    while wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
}

fn wrap_signed_degrees(angle_deg: f32) -> f32 {
    let mut wrapped = angle_deg;
    while wrapped > 180.0 {
        wrapped -= 360.0;
    }
    while wrapped <= -180.0 {
        wrapped += 360.0;
    }
    wrapped
}

fn unwrap_near(reference_unwrapped_deg: f32, raw_wrapped_deg: f32) -> f32 {
    reference_unwrapped_deg
        + wrap_angle_delta_deg(raw_wrapped_deg - wrap_degrees(reference_unwrapped_deg))
}

fn wrap_angle_delta_deg(delta: f32) -> f32 {
    if delta > 180.0 {
        delta - 360.0
    } else if delta < -180.0 {
        delta + 360.0
    } else {
        delta
    }
}
