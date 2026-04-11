pub use pendulum_lib::StoredMotorCalibration;
use pendulum_lib::settings_record::RecordLoad;

use crate::settings::{SettingsError, SettingsStorage};

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

fn normalize_sign(value: f32) -> f32 {
    if value.is_sign_negative() { -1.0 } else { 1.0 }
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
