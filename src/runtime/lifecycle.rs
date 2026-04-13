use super::{
    effects::{
        DeviceAction, DeviceActionCompletion, DeviceActionResult, DeviceReply, DeviceServices,
        execute_device_action,
    },
    model::DeviceModel,
};
use crate::{
    DeviceCommandError, DeviceFault, DeviceInfo, DeviceMode, DeviceRequest, DeviceResponse,
    DeviceState, StoredDeviceConfig, StoredMotorCalibration,
    controller::{ControllerConfig, PendulumController},
    device::{boot_status, production_fault},
};

#[derive(Debug, Clone)]
pub enum DeviceRequestPlan {
    Immediate(DeviceReply),
    Pending {
        action: DeviceAction,
        completion: DeviceActionCompletion,
    },
}

pub fn plan_device_request(
    device: &mut DeviceModel,
    request: DeviceRequest,
    device_info: DeviceInfo,
) -> DeviceRequestPlan {
    match request {
        DeviceRequest::GetInfo => {
            DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Info(device_info)))
        }
        DeviceRequest::GetStatus => {
            device.sync_status();
            DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Status(
                device.status.clone(),
            )))
        }
        DeviceRequest::GetWifiStatus => {
            if device.status.mode != DeviceMode::Manufacturing {
                DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
                    DeviceCommandError::UnsupportedInCurrentMode,
                )))
            } else {
                DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::WifiStatus(
                    device.config.wifi_status(),
                )))
            }
        }
        DeviceRequest::GetCalibrationStatus => {
            if device.status.mode != DeviceMode::Manufacturing {
                DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
                    DeviceCommandError::UnsupportedInCurrentMode,
                )))
            } else {
                DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::CalibrationStatus(
                    device.status.calibration.clone(),
                )))
            }
        }
        DeviceRequest::SetMode(mode) => plan_set_mode(device, mode),
        DeviceRequest::SetWifiConfig(credentials) => plan_set_wifi_config(device, credentials),
        DeviceRequest::ClearWifiConfig => plan_clear_wifi_config(device),
        DeviceRequest::ValidateWifi => plan_validate_wifi(device),
        DeviceRequest::StartMotorCalibration => plan_start_motor_calibration(device),
        DeviceRequest::StartRun => plan_start_run(device),
        DeviceRequest::StopRun => plan_stop_run(device),
        DeviceRequest::Reboot => {
            DeviceRequestPlan::Immediate(DeviceReply::reboot(DeviceResponse::Ack))
        }
    }
}

pub fn finalize_device_request(
    device: &mut DeviceModel,
    completion: DeviceActionCompletion,
    outcome: Result<DeviceActionResult, DeviceCommandError>,
) -> DeviceReply {
    match completion {
        DeviceActionCompletion::SetMode { next_config } => {
            if outcome.is_err() {
                return DeviceReply::new(DeviceResponse::Error(
                    DeviceCommandError::PersistenceFailed,
                ));
            }

            device.config = next_config;
            device.sync_status();
            DeviceReply::reboot(DeviceResponse::Ack)
        }
        DeviceActionCompletion::SetWifiConfig { next_config } => match outcome {
            Ok(DeviceActionResult::WifiValidation(report)) => {
                device.config = next_config;
                device.sync_status();
                DeviceReply::new(DeviceResponse::WifiValidation(report))
            }
            Ok(_) | Err(_) => {
                DeviceReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed))
            }
        },
        DeviceActionCompletion::ClearWifiConfig { next_config } => {
            if outcome.is_err() {
                return DeviceReply::new(DeviceResponse::Error(
                    DeviceCommandError::PersistenceFailed,
                ));
            }

            device.config = next_config;
            device.sync_status();
            DeviceReply::new(DeviceResponse::Ack)
        }
        DeviceActionCompletion::ValidateWifi => match outcome {
            Ok(DeviceActionResult::WifiValidation(report)) => {
                DeviceReply::new(DeviceResponse::WifiValidation(report))
            }
            Ok(_) | Err(_) => {
                DeviceReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed))
            }
        },
        DeviceActionCompletion::StartMotorCalibration => match outcome {
            Ok(DeviceActionResult::Calibration(calibration)) => {
                device.set_calibration(calibration);
                device.transition_to_service();
                DeviceReply::new(DeviceResponse::CalibrationStatus(
                    device.status.calibration.clone(),
                ))
            }
            Ok(_) => {
                device.transition_to_service();
                DeviceReply::new(DeviceResponse::Error(DeviceCommandError::CalibrationFailed))
            }
            Err(DeviceCommandError::CalibrationFailed) => {
                device.transition_to_service();
                DeviceReply::new(DeviceResponse::Error(DeviceCommandError::CalibrationFailed))
            }
            Err(DeviceCommandError::PersistenceFailed) => {
                device.transition_to_service();
                DeviceReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed))
            }
            Err(err) => {
                device.transition_to_service();
                DeviceReply::new(DeviceResponse::Error(err))
            }
        },
    }
}

pub fn boot_device_model(
    config_record: &crate::settings_record::RecordLoad<StoredDeviceConfig>,
    calibration_record: &crate::settings_record::RecordLoad<StoredMotorCalibration>,
    controller_config: ControllerConfig,
) -> DeviceModel {
    let status = boot_status(config_record, calibration_record);

    let config = match config_record {
        crate::settings_record::RecordLoad::Valid(config) => config.clone(),
        crate::settings_record::RecordLoad::Missing
        | crate::settings_record::RecordLoad::Corrupt => StoredDeviceConfig::default(),
    };

    let calibration = match calibration_record {
        crate::settings_record::RecordLoad::Valid(calibration) if calibration.is_valid() => {
            Some(*calibration)
        }
        crate::settings_record::RecordLoad::Missing
        | crate::settings_record::RecordLoad::Valid(_)
        | crate::settings_record::RecordLoad::Corrupt => None,
    };

    DeviceModel::new(
        config,
        status,
        PendulumController::new(controller_config),
        calibration,
    )
}

pub fn handle_device_request<S>(
    device: &mut DeviceModel,
    request: DeviceRequest,
    services: &mut S,
) -> DeviceReply
where
    S: DeviceServices,
{
    match plan_device_request(device, request, services.device_info()) {
        DeviceRequestPlan::Immediate(reply) => reply,
        DeviceRequestPlan::Pending { action, completion } => {
            let outcome = execute_device_action(action, services);
            finalize_device_request(device, completion, outcome)
        }
    }
}

fn plan_set_mode(device: &mut DeviceModel, mode: DeviceMode) -> DeviceRequestPlan {
    if let DeviceMode::Production = mode {
        if let Some(fault) = production_fault(&device.config, &device.status.calibration) {
            return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
                DeviceCommandError::ProductionPrecondition(fault),
            )));
        }
    }

    let mut next_config = device.config.clone();
    next_config.mode = mode;
    DeviceRequestPlan::Pending {
        action: DeviceAction::SaveDeviceConfig(next_config.clone()),
        completion: DeviceActionCompletion::SetMode { next_config },
    }
}

fn plan_set_wifi_config(
    device: &mut DeviceModel,
    credentials: crate::WifiCredentials,
) -> DeviceRequestPlan {
    if !device.in_manufacturing_service() {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    let mut next_config = device.config.clone();
    next_config.wifi = Some(credentials.clone());
    DeviceRequestPlan::Pending {
        action: DeviceAction::SaveDeviceConfigAndValidateWifi {
            next_config: next_config.clone(),
            credentials,
        },
        completion: DeviceActionCompletion::SetWifiConfig { next_config },
    }
}

fn plan_clear_wifi_config(device: &mut DeviceModel) -> DeviceRequestPlan {
    if !device.in_manufacturing_service() {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    let mut next_config = device.config.clone();
    next_config.wifi = None;
    DeviceRequestPlan::Pending {
        action: DeviceAction::SaveDeviceConfig(next_config.clone()),
        completion: DeviceActionCompletion::ClearWifiConfig { next_config },
    }
}

fn plan_validate_wifi(device: &mut DeviceModel) -> DeviceRequestPlan {
    if matches!(
        device.status.state,
        DeviceState::Running | DeviceState::Calibrating
    ) {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            DeviceCommandError::InvalidState,
        )));
    }

    let Some(credentials) = device.config.wifi.as_ref() else {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            DeviceCommandError::ProductionPrecondition(DeviceFault::MissingWifiConfig),
        )));
    };

    DeviceRequestPlan::Pending {
        action: DeviceAction::ValidateWifi(credentials.clone()),
        completion: DeviceActionCompletion::ValidateWifi,
    }
}

fn plan_start_motor_calibration(device: &mut DeviceModel) -> DeviceRequestPlan {
    if !device.in_manufacturing_service() {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    device.reset_runtime();
    device.status.state = DeviceState::Calibrating;
    device.status.fault = None;

    DeviceRequestPlan::Pending {
        action: DeviceAction::CalibrateMotor,
        completion: DeviceActionCompletion::StartMotorCalibration,
    }
}

fn plan_start_run(device: &mut DeviceModel) -> DeviceRequestPlan {
    if !device.in_manufacturing_service() {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    if let Some(fault) = device.run_precondition_fault() {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            DeviceCommandError::ProductionPrecondition(fault),
        )));
    }

    device.prepare_for_run();
    DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Ack))
}

fn plan_stop_run(device: &mut DeviceModel) -> DeviceRequestPlan {
    if device.status.mode != DeviceMode::Manufacturing {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            DeviceCommandError::UnsupportedInCurrentMode,
        )));
    }

    if device.status.state != DeviceState::Running {
        return DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Error(
            DeviceCommandError::InvalidState,
        )));
    }

    device.transition_to_service();
    DeviceRequestPlan::Immediate(DeviceReply::new(DeviceResponse::Ack))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CalibrationStatus, DeviceMode, DeviceStatus, FirmwareName, FirmwareVersion,
        WifiProbeResult, WifiStatus, WifiValidationReport, controller::PendulumController,
    };

    struct MockServices {
        device_info: DeviceInfo,
        saved_config: Option<StoredDeviceConfig>,
        saved_calibration: Option<StoredMotorCalibration>,
        wifi_result: WifiProbeResult,
        calibration: Option<StoredMotorCalibration>,
    }

    impl Default for MockServices {
        fn default() -> Self {
            Self {
                device_info: DeviceInfo {
                    firmware_name: FirmwareName::new(),
                    firmware_version: FirmwareVersion::new(),
                    protocol_version: crate::DEVICE_PROTOCOL_VERSION,
                },
                saved_config: None,
                saved_calibration: None,
                wifi_result: WifiProbeResult::Success {
                    ipv4_octets: [127, 0, 0, 1],
                },
                calibration: None,
            }
        }
    }

    impl DeviceServices for MockServices {
        type Error = ();

        fn device_info(&self) -> DeviceInfo {
            self.device_info.clone()
        }

        fn save_device_config(&mut self, config: &StoredDeviceConfig) -> Result<(), Self::Error> {
            self.saved_config = Some(config.clone());
            Ok(())
        }

        fn save_motor_calibration(
            &mut self,
            calibration: &StoredMotorCalibration,
        ) -> Result<(), Self::Error> {
            self.saved_calibration = Some(*calibration);
            self.calibration = Some(*calibration);
            Ok(())
        }

        fn validate_wifi(&mut self, _credentials: &crate::WifiCredentials) -> WifiValidationReport {
            WifiValidationReport {
                status: WifiStatus { ssid: None },
                result: self.wifi_result.clone(),
            }
        }

        fn calibrate_motor(&mut self) -> Result<Option<StoredMotorCalibration>, Self::Error> {
            Ok(self.calibration)
        }
    }

    fn runtime(status: DeviceStatus) -> DeviceModel {
        DeviceModel::new(
            StoredDeviceConfig::default(),
            status,
            PendulumController::new(Default::default()),
            None,
        )
    }

    #[test]
    fn get_status_reflects_state() {
        let mut runtime = runtime(DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Service,
            fault: None,
            wifi: WifiStatus { ssid: None },
            calibration: CalibrationStatus::Missing,
            control_mode: None,
        });
        let mut services = MockServices::default();

        let reply = handle_device_request(&mut runtime, DeviceRequest::GetStatus, &mut services);

        assert!(matches!(reply.response, DeviceResponse::Status(_)));
    }

    #[test]
    fn set_mode_requires_production_prereqs() {
        let mut runtime = runtime(DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Service,
            fault: None,
            wifi: WifiStatus { ssid: None },
            calibration: CalibrationStatus::Missing,
            control_mode: None,
        });
        let mut services = MockServices::default();

        let reply = handle_device_request(
            &mut runtime,
            DeviceRequest::SetMode(DeviceMode::Production),
            &mut services,
        );

        assert_eq!(
            reply.response,
            DeviceResponse::Error(DeviceCommandError::ProductionPrecondition(
                DeviceFault::MissingCalibration,
            )),
        );
    }

    #[test]
    fn wifi_validation_requires_config() {
        let mut runtime = runtime(DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Service,
            fault: None,
            wifi: WifiStatus { ssid: None },
            calibration: CalibrationStatus::Valid,
            control_mode: None,
        });
        let mut services = MockServices::default();

        let reply = handle_device_request(&mut runtime, DeviceRequest::ValidateWifi, &mut services);

        assert_eq!(
            reply.response,
            DeviceResponse::Error(DeviceCommandError::ProductionPrecondition(
                DeviceFault::MissingWifiConfig,
            )),
        );
    }
}
