use super::{
    effects::{
        ManagementAction, ManagementActionCompletion, ManagementActionResult, ManagementServices,
        CommandReply, execute_management_action,
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
pub enum CommandPlan {
    Immediate(CommandReply),
    Pending {
        action: ManagementAction,
        completion: ManagementActionCompletion,
    },
}

pub fn plan_command_request(
    device: &mut DeviceModel,
    request: DeviceRequest,
    device_info: DeviceInfo,
) -> CommandPlan {
    match request {
        DeviceRequest::GetInfo => {
            CommandPlan::Immediate(CommandReply::new(DeviceResponse::Info(device_info)))
        }
        DeviceRequest::GetStatus => {
            device.sync_status();
            CommandPlan::Immediate(CommandReply::new(DeviceResponse::Status(
                device.status.clone(),
            )))
        }
        DeviceRequest::GetWifiStatus => {
            if device.status.mode != DeviceMode::Manufacturing {
                CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
                    DeviceCommandError::UnsupportedInCurrentMode,
                )))
            } else {
                CommandPlan::Immediate(CommandReply::new(DeviceResponse::WifiStatus(
                    device.config.wifi_status(),
                )))
            }
        }
        DeviceRequest::GetCalibrationStatus => {
            if device.status.mode != DeviceMode::Manufacturing {
                CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
                    DeviceCommandError::UnsupportedInCurrentMode,
                )))
            } else {
                CommandPlan::Immediate(CommandReply::new(DeviceResponse::CalibrationStatus(
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
            CommandPlan::Immediate(CommandReply::reboot(DeviceResponse::Ack))
        }
    }
}

pub fn finalize_command_request(
    device: &mut DeviceModel,
    completion: ManagementActionCompletion,
    outcome: Result<ManagementActionResult, DeviceCommandError>,
) -> CommandReply {
    match completion {
        ManagementActionCompletion::SetMode { next_config } => {
            if outcome.is_err() {
                return CommandReply::new(DeviceResponse::Error(
                    DeviceCommandError::PersistenceFailed,
                ));
            }

            device.config = next_config;
            device.sync_status();
            CommandReply::reboot(DeviceResponse::Ack)
        }
        ManagementActionCompletion::SetWifiConfig { next_config } => match outcome {
            Ok(ManagementActionResult::WifiValidation(report)) => {
                device.config = next_config;
                device.sync_status();
                CommandReply::new(DeviceResponse::WifiValidation(report))
            }
            Ok(_) | Err(_) => {
                CommandReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed))
            }
        },
        ManagementActionCompletion::ClearWifiConfig { next_config } => {
            if outcome.is_err() {
                return CommandReply::new(DeviceResponse::Error(
                    DeviceCommandError::PersistenceFailed,
                ));
            }

            device.config = next_config;
            device.sync_status();
            CommandReply::new(DeviceResponse::Ack)
        }
        ManagementActionCompletion::ValidateWifi => match outcome {
            Ok(ManagementActionResult::WifiValidation(report)) => {
                CommandReply::new(DeviceResponse::WifiValidation(report))
            }
            Ok(_) | Err(_) => {
                CommandReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed))
            }
        },
        ManagementActionCompletion::StartMotorCalibration => match outcome {
            Ok(ManagementActionResult::Calibration(calibration)) => {
                device.set_calibration(calibration);
                device.transition_to_service();
                CommandReply::new(DeviceResponse::CalibrationStatus(
                    device.status.calibration.clone(),
                ))
            }
            Ok(_) => {
                device.transition_to_service();
                CommandReply::new(DeviceResponse::Error(DeviceCommandError::CalibrationFailed))
            }
            Err(DeviceCommandError::CalibrationFailed) => {
                device.transition_to_service();
                CommandReply::new(DeviceResponse::Error(DeviceCommandError::CalibrationFailed))
            }
            Err(DeviceCommandError::PersistenceFailed) => {
                device.transition_to_service();
                CommandReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed))
            }
            Err(err) => {
                device.transition_to_service();
                CommandReply::new(DeviceResponse::Error(err))
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

pub fn handle_command_request<S>(
    device: &mut DeviceModel,
    request: DeviceRequest,
    services: &mut S,
) -> CommandReply
where
    S: ManagementServices,
{
    match plan_command_request(device, request, services.device_info()) {
        CommandPlan::Immediate(reply) => reply,
        CommandPlan::Pending { action, completion } => {
            let outcome = execute_management_action(action, services);
            finalize_command_request(device, completion, outcome)
        }
    }
}

fn plan_set_mode(device: &mut DeviceModel, mode: DeviceMode) -> CommandPlan {
    if let DeviceMode::Production = mode {
        if let Some(fault) = production_fault(&device.config, &device.status.calibration) {
            return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
                DeviceCommandError::ProductionPrecondition(fault),
            )));
        }
    }

    let mut next_config = device.config.clone();
    next_config.mode = mode;
    CommandPlan::Pending {
        action: ManagementAction::SaveDeviceConfig(next_config.clone()),
        completion: ManagementActionCompletion::SetMode { next_config },
    }
}

fn plan_set_wifi_config(
    device: &mut DeviceModel,
    credentials: crate::WifiCredentials,
) -> CommandPlan {
    if !device.in_manufacturing_service() {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    let mut next_config = device.config.clone();
    next_config.wifi = Some(credentials.clone());
    CommandPlan::Pending {
        action: ManagementAction::SaveDeviceConfigAndValidateWifi {
            next_config: next_config.clone(),
            credentials,
        },
        completion: ManagementActionCompletion::SetWifiConfig { next_config },
    }
}

fn plan_clear_wifi_config(device: &mut DeviceModel) -> CommandPlan {
    if !device.in_manufacturing_service() {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    let mut next_config = device.config.clone();
    next_config.wifi = None;
    CommandPlan::Pending {
        action: ManagementAction::SaveDeviceConfig(next_config.clone()),
        completion: ManagementActionCompletion::ClearWifiConfig { next_config },
    }
}

fn plan_validate_wifi(device: &mut DeviceModel) -> CommandPlan {
    if matches!(
        device.status.state,
        DeviceState::Running | DeviceState::Calibrating
    ) {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            DeviceCommandError::InvalidState,
        )));
    }

    let Some(credentials) = device.config.wifi.as_ref() else {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            DeviceCommandError::ProductionPrecondition(DeviceFault::MissingWifiConfig),
        )));
    };

    CommandPlan::Pending {
        action: ManagementAction::ValidateWifi(credentials.clone()),
        completion: ManagementActionCompletion::ValidateWifi,
    }
}

fn plan_start_motor_calibration(device: &mut DeviceModel) -> CommandPlan {
    if !device.in_manufacturing_service() {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    device.reset_runtime();
    device.status.state = DeviceState::Calibrating;
    device.status.fault = None;

    CommandPlan::Pending {
        action: ManagementAction::CalibrateMotor,
        completion: ManagementActionCompletion::StartMotorCalibration,
    }
}

fn plan_start_run(device: &mut DeviceModel) -> CommandPlan {
    if !device.in_manufacturing_service() {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            device.service_mutation_error(),
        )));
    }

    if let Some(fault) = device.run_precondition_fault() {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            DeviceCommandError::ProductionPrecondition(fault),
        )));
    }

    device.prepare_for_run();
    CommandPlan::Immediate(CommandReply::new(DeviceResponse::Ack))
}

fn plan_stop_run(device: &mut DeviceModel) -> CommandPlan {
    if device.status.mode != DeviceMode::Manufacturing {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            DeviceCommandError::UnsupportedInCurrentMode,
        )));
    }

    if device.status.state != DeviceState::Running {
        return CommandPlan::Immediate(CommandReply::new(DeviceResponse::Error(
            DeviceCommandError::InvalidState,
        )));
    }

    device.transition_to_service();
    CommandPlan::Immediate(CommandReply::new(DeviceResponse::Ack))
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

    impl ManagementServices for MockServices {
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

        let reply = handle_command_request(&mut runtime, DeviceRequest::GetStatus, &mut services);

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

        let reply = handle_command_request(
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

        let reply = handle_command_request(&mut runtime, DeviceRequest::ValidateWifi, &mut services);

        assert_eq!(
            reply.response,
            DeviceResponse::Error(DeviceCommandError::ProductionPrecondition(
                DeviceFault::MissingWifiConfig,
            )),
        );
    }
}
