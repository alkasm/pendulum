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
use esp_hal::{Blocking, i2c::master::I2c, main};
use hw::{CurrentSample, CurrentSensor, GY521_DEFAULT_I2C_ADDR, SIX_STEP_COMMUTATION, Tmc6300};
use libm::atan2f;
use penproto::{
    CurrentTelemetry, HallMeasurement, HallTelemetry, PendulumControlMode, PendulumControlTelemetry,
    PendulumEstimateMeasurement, PendulumEstimateTelemetry, PendulumTelemetryFrame, TelemetryPacket,
};

const CONTROL_PERIOD_MS: u32 = 5;
const FRAME_BUF_LEN: usize = 512;
const BASELINE_SAMPLES: u32 = 64;
const CALIBRATION_HOLD_LOOPS: u8 = 24;
const MAX_COMMAND_TORQUE_NM: f32 = 0.031;
const MAX_PHASE_CURRENT_A: f32 = 1.2;
const ARM_ANGLE_DEG: f32 = 12.0;
const ARM_RATE_DPS: f32 = 12.0;
const ARM_SAMPLE_TARGET: u16 = 12;
const CAPTURE_ANGLE_DEG: f32 = 45.0;
const DRIVE_DEADBAND: f32 = 0.08;
const MIN_STEP_RATE_SPS: f32 = 24.0;
const MAX_STEP_RATE_SPS: f32 = 220.0;
const KP_NM_PER_RAD: f32 = 0.22;
const KD_NM_PER_RAD_S: f32 = 0.001;

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

#[derive(Clone, Copy)]
enum ImuProbeError {
    RegisterRead,
    UnexpectedWhoAmI(u8),
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
enum BodyAxis3 {
    Right,
    Left,
    Up,
    Down,
    TowardViewer,
    AwayFromViewer,
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
struct Point3Mm {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Clone, Copy)]
struct ImuAxesInBody {
    x_axis: BodyAxis3,
    y_axis: BodyAxis3,
    z_axis: BodyAxis3,
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
struct ImuMount {
    translation_from_motor_mm: Point3Mm,
    axes_in_body: ImuAxesInBody,
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
struct MotorMount {
    center_from_pivot_mm: Point3Mm,
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
struct PendulumGeometry {
    motor_mount: MotorMount,
    imu_mount: ImuMount,
}

#[derive(Clone, Copy, Default)]
struct Vector3 {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Clone, Copy)]
enum CalibrationState {
    Uninitialized,
    Running(CalibrationProgress),
    Ready(CommutationCalibration),
}

#[derive(Clone, Copy)]
struct CalibrationProgress {
    step_index: usize,
    hold_loops_remaining: u8,
    centers_by_step_deg: [f32; 6],
}

#[derive(Clone, Copy)]
struct CommutationCalibration {
    centers_by_step_deg: [f32; 6],
    step_order_by_angle: [u8; 6],
}

struct ControlState {
    calibration: CalibrationState,
    armed: bool,
    arm_ready_samples: u16,
    step_phase_accumulator: f32,
    commanded_slot: usize,
    last_wheel_angle_deg: Option<f32>,
    filtered_wheel_speed_dps: f32,
    last_commutation_step: u8,
    last_commutation_center_deg: f32,
    motor_enabled: bool,
}

const PENDULUM_GEOMETRY: PendulumGeometry = PendulumGeometry {
    motor_mount: MotorMount {
        center_from_pivot_mm: Point3Mm {
            x: 0.0,
            y: 0.235,
            z: 0.0,
        },
    },
    imu_mount: ImuMount {
        translation_from_motor_mm: Point3Mm {
            x: -50.0,
            y: 27.36,
            z: 10.0,
        },
        axes_in_body: ImuAxesInBody {
            x_axis: BodyAxis3::Down,
            y_axis: BodyAxis3::Right,
            z_axis: BodyAxis3::TowardViewer,
        },
    },
};

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
    let mut motor_driver = Tmc6300::new(
        peripherals.GPIO5,
        peripherals.GPIO34,
        peripherals.GPIO16,
        peripherals.GPIO17,
        peripherals.GPIO18,
        peripherals.GPIO23,
        peripherals.GPIO19,
        peripherals.GPIO33,
    );
    let mut i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let mut frame_buf = [0_u8; FRAME_BUF_LEN];
    let mut seq = 0_u32;
    let mut hall_configured = false;
    let mut imu_verified = false;
    let mut imu_awake = false;
    let mut control_state = ControlState::new();

    let _ = current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    motor_driver.disable();

    loop {
        let current_sample = current_sensor.read();
        let current = current_telemetry_from_sample(current_sample);
        let hall = read_hall_telemetry(&mut i2c, &mut hall_configured);
        let estimate = read_pendulum_estimate(&mut i2c, &mut imu_verified, &mut imu_awake);
        let control = update_control_loop(
            &mut control_state,
            &mut motor_driver,
            &hall,
            &estimate,
            &current_sample,
        );

        let frame = PendulumTelemetryFrame {
            seq,
            uptime_ms: seq.saturating_mul(CONTROL_PERIOD_MS),
            motor_driver_diag_high: motor_driver.diag_is_high(),
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
            calibration: CalibrationState::Uninitialized,
            armed: false,
            arm_ready_samples: 0,
            step_phase_accumulator: 0.0,
            commanded_slot: 0,
            last_wheel_angle_deg: None,
            filtered_wheel_speed_dps: 0.0,
            last_commutation_step: 0,
            last_commutation_center_deg: 0.0,
            motor_enabled: false,
        }
    }

    fn disable_motor(&mut self, motor_driver: &mut Tmc6300<'_>) {
        motor_driver.disable();
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
        self.filtered_wheel_speed_dps
    }

    fn reset_wheel_observer(&mut self) {
        self.last_wheel_angle_deg = None;
        self.filtered_wheel_speed_dps = 0.0;
    }

    fn reset_arming(&mut self) {
        self.armed = false;
        self.arm_ready_samples = 0;
        self.step_phase_accumulator = 0.0;
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

impl CommutationCalibration {
    fn from_centers(centers_by_step_deg: [f32; 6]) -> Self {
        let mut step_order_by_angle = [0_u8, 1, 2, 3, 4, 5];
        let mut i = 1;
        while i < step_order_by_angle.len() {
            let key = step_order_by_angle[i];
            let key_angle = centers_by_step_deg[key as usize];
            let mut j = i;
            while j > 0 && key_angle < centers_by_step_deg[step_order_by_angle[j - 1] as usize] {
                step_order_by_angle[j] = step_order_by_angle[j - 1];
                j -= 1;
            }
            step_order_by_angle[j] = key;
            i += 1;
        }

        Self {
            centers_by_step_deg,
            step_order_by_angle,
        }
    }

    fn command_step(&self, wheel_angle_deg: f32, drive_sign: i8) -> (u8, f32) {
        let current_slot = self.nearest_slot(wheel_angle_deg);
        let target_slot = if drive_sign >= 0 {
            (current_slot + 1) % self.step_order_by_angle.len()
        } else {
            (current_slot + self.step_order_by_angle.len() - 1) % self.step_order_by_angle.len()
        };
        let step_index = self.step_order_by_angle[target_slot];
        (step_index, self.centers_by_step_deg[step_index as usize])
    }

    fn nearest_slot(&self, wheel_angle_deg: f32) -> usize {
        let mut best_slot = 0_usize;
        let mut best_error_deg = 360.0_f32;

        let mut slot = 0_usize;
        while slot < self.step_order_by_angle.len() {
            let step_index = self.step_order_by_angle[slot] as usize;
            let center_deg = self.centers_by_step_deg[step_index];
            let error_deg = wrap_angle_delta_deg(wheel_angle_deg - center_deg).abs();
            if error_deg < best_error_deg {
                best_error_deg = error_deg;
                best_slot = slot;
            }
            slot += 1;
        }

        best_slot
    }

    fn step_for_slot(&self, slot: usize) -> (u8, f32) {
        let wrapped_slot = slot % self.step_order_by_angle.len();
        let step_index = self.step_order_by_angle[wrapped_slot];
        (step_index, self.centers_by_step_deg[step_index as usize])
    }
}

fn update_control_loop(
    control_state: &mut ControlState,
    motor_driver: &mut Tmc6300<'_>,
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
        control_state.calibration = CalibrationState::Uninitialized;
        control_state.reset_arming();
        control_state.disable_motor(motor_driver);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::WaitingForHall,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: control_state.last_commutation_step,
            commutation_center_deg: control_state.last_commutation_center_deg,
            motor_enabled: control_state.motor_enabled,
        };
    }

    let hall_measurement = hall_measurement.expect("hall measurement checked above");
    if let Some(calibrating) = step_calibration(control_state, motor_driver, hall_measurement) {
        control_state.reset_arming();
        return PendulumControlTelemetry {
            mode: PendulumControlMode::Calibrating,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: calibrating.0,
            commutation_center_deg: calibrating.1,
            motor_enabled: control_state.motor_enabled,
        };
    }

    let estimate_measurement = if let Some(measurement) = estimate_measurement {
        measurement
    } else {
        control_state.reset_arming();
        control_state.disable_motor(motor_driver);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::WaitingForImu,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: control_state.last_commutation_step,
            commutation_center_deg: control_state.last_commutation_center_deg,
            motor_enabled: control_state.motor_enabled,
        };
    };

    if !control_state.step_arming(
        estimate_measurement.theta_deg,
        estimate_measurement.theta_dot_dps,
    ) {
        if let CalibrationState::Ready(calibration) = control_state.calibration {
            control_state.commanded_slot = calibration.nearest_slot(wheel_angle_deg);
        }
        control_state.disable_motor(motor_driver);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::Arming,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: control_state.last_commutation_step,
            commutation_center_deg: control_state.last_commutation_center_deg,
            motor_enabled: control_state.motor_enabled,
        };
    }

    if estimate_measurement.theta_deg.abs() > CAPTURE_ANGLE_DEG {
        control_state.reset_arming();
        control_state.disable_motor(motor_driver);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::CaptureOutOfRange,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: control_state.last_commutation_step,
            commutation_center_deg: control_state.last_commutation_center_deg,
            motor_enabled: control_state.motor_enabled,
        };
    }

    if max_phase_current_amps(current_sample) > MAX_PHASE_CURRENT_A {
        control_state.reset_arming();
        control_state.disable_motor(motor_driver);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::CurrentLimited,
            torque_command_nm: 0.0,
            drive_command: 0.0,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: control_state.last_commutation_step,
            commutation_center_deg: control_state.last_commutation_center_deg,
            motor_enabled: control_state.motor_enabled,
        };
    }

    let torque_command_nm =
        pd_torque_command_nm(estimate_measurement.theta_deg, estimate_measurement.theta_dot_dps);
    let drive_command = clamp(
        torque_command_nm / MAX_COMMAND_TORQUE_NM,
        -1.0,
        1.0,
    );

    if drive_command.abs() < DRIVE_DEADBAND {
        if let CalibrationState::Ready(calibration) = control_state.calibration {
            control_state.commanded_slot = calibration.nearest_slot(wheel_angle_deg);
        }
        control_state.step_phase_accumulator = 0.0;
        control_state.disable_motor(motor_driver);
        return PendulumControlTelemetry {
            mode: PendulumControlMode::Idle,
            torque_command_nm,
            drive_command,
            wheel_angle_deg,
            wheel_speed_dps,
            commutation_step: control_state.last_commutation_step,
            commutation_center_deg: control_state.last_commutation_center_deg,
            motor_enabled: control_state.motor_enabled,
        };
    }

    let calibration = match control_state.calibration {
        CalibrationState::Ready(calibration) => calibration,
        _ => {
            control_state.disable_motor(motor_driver);
            return PendulumControlTelemetry {
                mode: PendulumControlMode::Startup,
                torque_command_nm: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                commutation_step: control_state.last_commutation_step,
                commutation_center_deg: control_state.last_commutation_center_deg,
                motor_enabled: control_state.motor_enabled,
            };
        }
    };

    let drive_sign = if drive_command >= 0.0 { 1 } else { -1 };
    let step_rate_sps = MIN_STEP_RATE_SPS + (MAX_STEP_RATE_SPS - MIN_STEP_RATE_SPS) * drive_command.abs();
    control_state.step_phase_accumulator += step_rate_sps * (CONTROL_PERIOD_MS as f32 / 1_000.0);

    while control_state.step_phase_accumulator >= 1.0 {
        control_state.step_phase_accumulator -= 1.0;
        let slot_count = calibration.step_order_by_angle.len();
        control_state.commanded_slot = if drive_sign >= 0 {
            (control_state.commanded_slot + 1) % slot_count
        } else {
            (control_state.commanded_slot + slot_count - 1) % slot_count
        };
    }

    let (step_index, commutation_center_deg) = calibration.step_for_slot(control_state.commanded_slot);
    motor_driver.enable();
    motor_driver.apply_step(SIX_STEP_COMMUTATION[step_index as usize]);
    control_state.last_commutation_step = step_index;
    control_state.last_commutation_center_deg = commutation_center_deg;
    control_state.motor_enabled = true;

    PendulumControlTelemetry {
        mode: PendulumControlMode::Active,
        torque_command_nm,
        drive_command,
        wheel_angle_deg,
        wheel_speed_dps,
        commutation_step: control_state.last_commutation_step,
        commutation_center_deg: control_state.last_commutation_center_deg,
        motor_enabled: control_state.motor_enabled,
    }
}

fn step_calibration(
    control_state: &mut ControlState,
    motor_driver: &mut Tmc6300<'_>,
    hall_measurement: HallMeasurement,
) -> Option<(u8, f32)> {
    match control_state.calibration {
        CalibrationState::Uninitialized => {
            control_state.calibration = CalibrationState::Running(CalibrationProgress {
                step_index: 0,
            hold_loops_remaining: CALIBRATION_HOLD_LOOPS,
            centers_by_step_deg: [0.0; 6],
        });
            control_state.commanded_slot = 0;
            control_state.step_phase_accumulator = 0.0;
            motor_driver.enable();
            motor_driver.apply_step(SIX_STEP_COMMUTATION[0]);
            control_state.last_commutation_step = 0;
            control_state.last_commutation_center_deg = hall_measurement.angle_deg;
            control_state.motor_enabled = true;
            Some((0, hall_measurement.angle_deg))
        }
        CalibrationState::Running(mut progress) => {
            let step_index = progress.step_index;
            motor_driver.enable();
            motor_driver.apply_step(SIX_STEP_COMMUTATION[step_index]);
            control_state.last_commutation_step = step_index as u8;
            control_state.last_commutation_center_deg = hall_measurement.angle_deg;
            control_state.motor_enabled = true;

            if progress.hold_loops_remaining > 0 {
                progress.hold_loops_remaining -= 1;
                control_state.calibration = CalibrationState::Running(progress);
                return Some((step_index as u8, hall_measurement.angle_deg));
            }

            progress.centers_by_step_deg[step_index] = hall_measurement.angle_deg;
            if step_index + 1 < SIX_STEP_COMMUTATION.len() {
                progress.step_index += 1;
                progress.hold_loops_remaining = CALIBRATION_HOLD_LOOPS;
                let next_step = progress.step_index as u8;
                control_state.calibration = CalibrationState::Running(progress);
                control_state.last_commutation_step = next_step;
                return Some((next_step, hall_measurement.angle_deg));
            }

            control_state.calibration =
                CalibrationState::Ready(CommutationCalibration::from_centers(
                    progress.centers_by_step_deg,
                ));
            if let CalibrationState::Ready(calibration) = control_state.calibration {
                control_state.commanded_slot = calibration.nearest_slot(hall_measurement.angle_deg);
            }
            control_state.step_phase_accumulator = 0.0;
            control_state.disable_motor(motor_driver);
            Some((step_index as u8, hall_measurement.angle_deg))
        }
        CalibrationState::Ready(_) => None,
    }
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

fn pd_torque_command_nm(theta_deg: f32, theta_dot_dps: f32) -> f32 {
    let theta_rad = theta_deg * (core::f32::consts::PI / 180.0);
    let theta_dot_rad_s = theta_dot_dps * (core::f32::consts::PI / 180.0);
    KP_NM_PER_RAD * theta_rad + KD_NM_PER_RAD_S * theta_dot_rad_s
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
) -> PendulumEstimateTelemetry {
    if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
        *imu_verified = false;
        *imu_awake = false;
        return PendulumEstimateTelemetry::Missing;
    }

    if !*imu_verified {
        match imu_verify(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_verified = true,
            Err(ImuProbeError::RegisterRead) => return PendulumEstimateTelemetry::Missing,
            Err(ImuProbeError::UnexpectedWhoAmI(value)) => {
                return PendulumEstimateTelemetry::UnexpectedWhoAmI { value };
            }
        }
    }

    if !*imu_awake {
        match imu_wake(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_awake = true,
            Err(register) => return PendulumEstimateTelemetry::WakeError { register },
        }
    }

    match imu_read_pendulum_measurement(i2c, GY521_DEFAULT_I2C_ADDR) {
        Ok(measurement) => PendulumEstimateTelemetry::Measurement(measurement),
        Err(register) => PendulumEstimateTelemetry::ReadError { register },
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
    let accel_body_g = transform_imu_vector_to_body(accel_imu_g, PENDULUM_GEOMETRY.imu_mount.axes_in_body);
    let gyro_body_dps = transform_imu_vector_to_body(gyro_imu_dps, PENDULUM_GEOMETRY.imu_mount.axes_in_body);

    let theta_deg = atan2f(-accel_body_g.x, accel_body_g.y) * (180.0 / core::f32::consts::PI);
    let theta_dot_dps = -gyro_body_dps.z;

    Ok(PendulumEstimateMeasurement {
        theta_deg,
        theta_dot_dps,
    })
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

fn wrap_angle_delta_deg(delta: f32) -> f32 {
    if delta > 180.0 {
        delta - 360.0
    } else if delta < -180.0 {
        delta + 360.0
    } else {
        delta
    }
}
