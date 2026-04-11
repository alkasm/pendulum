use crate::{
    CalibrationStatus, DeviceFault, DeviceMode, DeviceState, DeviceStatus, StoredDeviceConfig,
    StoredMotorCalibration, WifiStatus, WifiValidationState, settings_record::RecordLoad,
};

pub fn calibration_status(record: &RecordLoad<StoredMotorCalibration>) -> CalibrationStatus {
    match record {
        RecordLoad::Missing => CalibrationStatus::Missing,
        RecordLoad::Valid(calibration) if calibration.is_valid() => CalibrationStatus::Valid,
        RecordLoad::Valid(_) | RecordLoad::Corrupt => CalibrationStatus::Invalid,
    }
}

pub fn default_wifi_status() -> WifiStatus {
    WifiStatus {
        configured: false,
        ssid: None,
        validation: WifiValidationState::NeverValidated,
    }
}

pub fn boot_status(
    config_record: &RecordLoad<StoredDeviceConfig>,
    calibration_record: &RecordLoad<StoredMotorCalibration>,
) -> DeviceStatus {
    let calibration = calibration_status(calibration_record);

    match config_record {
        RecordLoad::Corrupt => DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Fault,
            fault: Some(DeviceFault::StorageCorrupt),
            wifi: default_wifi_status(),
            calibration,
            control_mode: None,
        },
        RecordLoad::Missing => DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Service,
            fault: None,
            wifi: default_wifi_status(),
            calibration,
            control_mode: None,
        },
        RecordLoad::Valid(config) => match config.mode {
            DeviceMode::Manufacturing => DeviceStatus {
                mode: DeviceMode::Manufacturing,
                state: DeviceState::Service,
                fault: None,
                wifi: config.wifi_status(),
                calibration,
                control_mode: None,
            },
            DeviceMode::Production => {
                let fault = production_fault(config, &calibration);
                DeviceStatus {
                    mode: DeviceMode::Production,
                    state: if fault.is_some() {
                        DeviceState::Fault
                    } else {
                        DeviceState::Running
                    },
                    fault,
                    wifi: config.wifi_status(),
                    calibration,
                    control_mode: None,
                }
            }
        },
    }
}

pub fn production_fault(
    config: &StoredDeviceConfig,
    calibration: &CalibrationStatus,
) -> Option<DeviceFault> {
    match calibration {
        CalibrationStatus::Missing => return Some(DeviceFault::MissingCalibration),
        CalibrationStatus::Invalid => return Some(DeviceFault::InvalidCalibration),
        CalibrationStatus::Valid => {}
    }

    if config.wifi.is_none() {
        return Some(DeviceFault::MissingWifiConfig);
    }

    match config.wifi_validation {
        WifiValidationState::Validated => None,
        WifiValidationState::NeverValidated => Some(DeviceFault::WifiNeverValidated),
        WifiValidationState::ValidationFailed => Some(DeviceFault::WifiValidationFailed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{StoredDeviceConfig, WifiCredentials};

    fn valid_calibration() -> RecordLoad<StoredMotorCalibration> {
        RecordLoad::Valid(StoredMotorCalibration {
            direction_sign: 1.0,
            electrical_offset_deg: 15.0,
            torque_sign: 1.0,
        })
    }

    fn production_config(validation: WifiValidationState) -> StoredDeviceConfig {
        StoredDeviceConfig {
            mode: DeviceMode::Production,
            wifi: Some(WifiCredentials::new("pendulum-net", "password").unwrap()),
            wifi_validation: validation,
        }
    }

    #[test]
    fn blank_config_defaults_to_manufacturing_service() {
        let status = boot_status(&RecordLoad::Missing, &RecordLoad::Missing);
        assert_eq!(status.mode, DeviceMode::Manufacturing);
        assert_eq!(status.state, DeviceState::Service);
        assert_eq!(status.fault, None);
    }

    #[test]
    fn corrupt_config_faults() {
        let status = boot_status(&RecordLoad::Corrupt, &valid_calibration());
        assert_eq!(status.state, DeviceState::Fault);
        assert_eq!(status.fault, Some(DeviceFault::StorageCorrupt));
    }

    #[test]
    fn production_requires_calibration() {
        let status = boot_status(
            &RecordLoad::Valid(production_config(WifiValidationState::Validated)),
            &RecordLoad::Missing,
        );
        assert_eq!(status.state, DeviceState::Fault);
        assert_eq!(status.fault, Some(DeviceFault::MissingCalibration));
    }

    #[test]
    fn production_requires_validated_wifi() {
        let status = boot_status(
            &RecordLoad::Valid(production_config(WifiValidationState::NeverValidated)),
            &valid_calibration(),
        );
        assert_eq!(status.state, DeviceState::Fault);
        assert_eq!(status.fault, Some(DeviceFault::WifiNeverValidated));
    }

    #[test]
    fn production_faults_when_last_wifi_validation_failed() {
        let status = boot_status(
            &RecordLoad::Valid(production_config(WifiValidationState::ValidationFailed)),
            &valid_calibration(),
        );
        assert_eq!(status.state, DeviceState::Fault);
        assert_eq!(status.fault, Some(DeviceFault::WifiValidationFailed));
    }

    #[test]
    fn valid_production_boot_runs() {
        let status = boot_status(
            &RecordLoad::Valid(production_config(WifiValidationState::Validated)),
            &valid_calibration(),
        );
        assert_eq!(status.state, DeviceState::Running);
        assert_eq!(status.fault, None);
    }
}
