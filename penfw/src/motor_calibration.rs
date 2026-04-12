use esp_hal::{Blocking, i2c::master::I2c};
use libm::atan2f;
pub use pendulum_lib::StoredMotorCalibration;
use pendulum_lib::{
    HallTelemetry,
    runtime::{HallElectricalCalibration, VOLTAGE_LIMIT_V, simplefoc_sine_pwm_phase_voltages},
    settings_record::RecordLoad,
};

use crate::{
    hall::{HallSensor, read_hall_telemetry},
    math::{degrees_to_radians, unwrap_near, wrap_degrees},
    motor_drive::PwmMotorDrive,
    settings::{SettingsError, SettingsStorage},
};

const CONTROL_PERIOD_MS: u32 = 5;
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

#[allow(dead_code)]
pub fn load_motor_calibration() -> Result<Option<StoredMotorCalibration>, SettingsError> {
    let mut storage = SettingsStorage::new();
    Ok(match storage.load_motor_calibration_record()? {
        RecordLoad::Valid(calibration) if calibration.is_valid() => Some(StoredMotorCalibration {
            direction_sign: normalize_sign(calibration.direction_sign),
            electrical_offset_deg: wrap_degrees(calibration.electrical_offset_deg),
            torque_sign: normalize_sign(calibration.torque_sign),
        }),
        _ => None,
    })
}

#[allow(dead_code)]
pub fn save_motor_calibration(calibration: StoredMotorCalibration) -> Result<(), SettingsError> {
    let mut storage = SettingsStorage::new();
    storage.save_motor_calibration(&StoredMotorCalibration {
        direction_sign: normalize_sign(calibration.direction_sign),
        electrical_offset_deg: wrap_degrees(calibration.electrical_offset_deg),
        torque_sign: normalize_sign(calibration.torque_sign),
    })
}

pub fn calibrate_hall_electrical_cycle(
    i2c: &mut I2c<'_, Blocking>,
    hall: &mut HallSensor,
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

        if let HallTelemetry::Measurement(measurement) = read_hall_telemetry(i2c, hall) {
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
                pos_offset_sin_sum += libm::sinf(degrees_to_radians(pos_offset_deg));
                pos_offset_cos_sum +=
                    libm::sinf(degrees_to_radians(pos_offset_deg) + core::f32::consts::FRAC_PI_2);
                neg_offset_sin_sum += libm::sinf(degrees_to_radians(neg_offset_deg));
                neg_offset_cos_sum +=
                    libm::sinf(degrees_to_radians(neg_offset_deg) + core::f32::consts::FRAC_PI_2);
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
        wrap_degrees(
            atan2f(pos_offset_sin_sum, pos_offset_cos_sum) * (180.0 / core::f32::consts::PI),
        )
    } else {
        wrap_degrees(
            atan2f(neg_offset_sin_sum, neg_offset_cos_sum) * (180.0 / core::f32::consts::PI),
        )
    };

    Some(HallElectricalCalibration {
        direction_sign,
        electrical_offset_deg,
        torque_sign: 1.0,
    })
}

pub fn refine_torque_phase_offset(
    i2c: &mut I2c<'_, Blocking>,
    hall: &mut HallSensor,
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
                hall,
                delay,
                motor_drive,
                calibration,
                candidate_offset_deg,
                candidate_torque_sign * PHASE_SEARCH_UQ_V,
            )?;
            let neg_travel_deg = measure_phase_search_travel(
                i2c,
                hall,
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
    hall: &mut HallSensor,
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
        if let HallTelemetry::Measurement(measurement) = read_hall_telemetry(i2c, hall) {
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

fn dt_s() -> f32 {
    CONTROL_PERIOD_MS as f32 / 1_000.0
}

fn normalize_sign(value: f32) -> f32 {
    if value.is_sign_negative() { -1.0 } else { 1.0 }
}
