use crate::{
    DeviceInfo, DeviceResponse, StoredDeviceConfig, StoredMotorCalibration, WifiCredentials,
    WifiValidationReport,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceReply {
    pub response: DeviceResponse,
    pub reboot: bool,
}

impl DeviceReply {
    pub fn new(response: DeviceResponse) -> Self {
        Self {
            response,
            reboot: false,
        }
    }

    pub fn reboot(response: DeviceResponse) -> Self {
        Self {
            response,
            reboot: true,
        }
    }
}

pub trait DeviceServices {
    type Error;

    fn device_info(&self) -> DeviceInfo;
    fn save_device_config(&mut self, config: &StoredDeviceConfig) -> Result<(), Self::Error>;
    fn save_motor_calibration(
        &mut self,
        calibration: &StoredMotorCalibration,
    ) -> Result<(), Self::Error>;
    fn validate_wifi(&mut self, credentials: &WifiCredentials) -> WifiValidationReport;
    fn calibrate_motor(&mut self) -> Result<Option<StoredMotorCalibration>, Self::Error>;
}

#[derive(Debug, Clone)]
pub enum DeviceAction {
    SaveDeviceConfig(StoredDeviceConfig),
    SaveDeviceConfigAndValidateWifi {
        next_config: StoredDeviceConfig,
        credentials: WifiCredentials,
    },
    ValidateWifi(WifiCredentials),
    CalibrateMotor,
}

#[derive(Debug, Clone)]
pub enum DeviceActionResult {
    Saved,
    WifiValidation(WifiValidationReport),
    Calibration(StoredMotorCalibration),
}

#[derive(Debug, Clone)]
pub enum DeviceActionCompletion {
    SetMode { next_config: StoredDeviceConfig },
    SetWifiConfig { next_config: StoredDeviceConfig },
    ClearWifiConfig { next_config: StoredDeviceConfig },
    ValidateWifi,
    StartMotorCalibration,
}

pub fn execute_device_action<S>(
    action: DeviceAction,
    services: &mut S,
) -> Result<DeviceActionResult, crate::DeviceCommandError>
where
    S: DeviceServices,
{
    match action {
        DeviceAction::SaveDeviceConfig(config) => services
            .save_device_config(&config)
            .map(|_| DeviceActionResult::Saved)
            .map_err(|_| crate::DeviceCommandError::PersistenceFailed),
        DeviceAction::SaveDeviceConfigAndValidateWifi {
            next_config,
            credentials,
        } => match services.save_device_config(&next_config) {
            Ok(()) => {
                let report = services.validate_wifi(&credentials);
                Ok(DeviceActionResult::WifiValidation(WifiValidationReport {
                    status: next_config.wifi_status(),
                    result: report.result,
                }))
            }
            Err(_) => Err(crate::DeviceCommandError::PersistenceFailed),
        },
        DeviceAction::ValidateWifi(credentials) => Ok(DeviceActionResult::WifiValidation(
            services.validate_wifi(&credentials),
        )),
        DeviceAction::CalibrateMotor => match services.calibrate_motor() {
            Ok(Some(calibration)) => services
                .save_motor_calibration(&calibration)
                .map_err(|_| crate::DeviceCommandError::PersistenceFailed)
                .map(|_| DeviceActionResult::Calibration(calibration)),
            Ok(None) => Err(crate::DeviceCommandError::CalibrationFailed),
            Err(_) => Err(crate::DeviceCommandError::CalibrationFailed),
        },
    }
}
