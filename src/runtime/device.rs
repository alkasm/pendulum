use crate::{
    CalibrationStatus, DeviceCommandError, DeviceFault, DeviceInfo, DeviceMode, DeviceRequest,
    DeviceResponse, DeviceState, DeviceStatus, PendulumControlMode, StoredDeviceConfig,
    StoredMotorCalibration, WifiCredentials, WifiValidationReport, controller::PendulumController,
    device::production_fault,
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
pub struct DeviceRuntime {
    config: StoredDeviceConfig,
    status: DeviceStatus,
    controller: PendulumController,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CalibrationStatus, DeviceMode, DeviceState, DeviceStatus, StoredDeviceConfig,
        StoredMotorCalibration, WifiCredentials, WifiProbeResult, WifiStatus,
        controller::PendulumController,
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
                    firmware_name: crate::FirmwareName::new(),
                    firmware_version: crate::FirmwareVersion::new(),
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

        fn validate_wifi(&mut self, _credentials: &WifiCredentials) -> WifiValidationReport {
            WifiValidationReport {
                status: WifiStatus { ssid: None },
                result: self.wifi_result.clone(),
            }
        }

        fn calibrate_motor(&mut self) -> Result<Option<StoredMotorCalibration>, Self::Error> {
            Ok(self.calibration)
        }
    }

    fn runtime(status: DeviceStatus) -> DeviceRuntime {
        DeviceRuntime::new(
            StoredDeviceConfig::default(),
            status,
            PendulumController::new(Default::default()),
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

        let reply = runtime.handle_request(DeviceRequest::GetStatus, &mut services);

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

        let reply = runtime.handle_request(
            DeviceRequest::SetMode(DeviceMode::Production),
            &mut services,
        );

        assert!(matches!(
            reply.response,
            DeviceResponse::Error(DeviceCommandError::ProductionPrecondition(_))
        ));
    }

    #[test]
    fn wifi_validation_requires_config() {
        let mut runtime = runtime(DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Service,
            fault: None,
            wifi: WifiStatus { ssid: None },
            calibration: CalibrationStatus::Missing,
            control_mode: None,
        });
        let mut services = MockServices::default();

        let reply = runtime.handle_request(DeviceRequest::ValidateWifi, &mut services);

        assert!(matches!(
            reply.response,
            DeviceResponse::Error(DeviceCommandError::ProductionPrecondition(_))
        ));
    }

    #[test]
    fn calibration_success_updates_status() {
        let mut runtime = runtime(DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Service,
            fault: None,
            wifi: WifiStatus { ssid: None },
            calibration: CalibrationStatus::Missing,
            control_mode: None,
        });
        let mut services = MockServices {
            calibration: Some(StoredMotorCalibration {
                direction_sign: 1.0,
                electrical_offset_deg: 12.0,
                torque_sign: -1.0,
            }),
            ..Default::default()
        };

        let reply = runtime.handle_request(DeviceRequest::StartMotorCalibration, &mut services);

        assert!(matches!(
            reply.response,
            DeviceResponse::CalibrationStatus(CalibrationStatus::Valid)
        ));
        assert_eq!(runtime.status().calibration, CalibrationStatus::Valid);
        assert!(services.saved_calibration.is_some());
    }
}

impl DeviceRuntime {
    pub fn new(
        config: StoredDeviceConfig,
        status: DeviceStatus,
        controller: PendulumController,
    ) -> Self {
        Self {
            config,
            status,
            controller,
        }
    }

    pub fn status(&self) -> &DeviceStatus {
        &self.status
    }

    pub fn status_mut(&mut self) -> &mut DeviceStatus {
        &mut self.status
    }

    pub fn config(&self) -> &StoredDeviceConfig {
        &self.config
    }

    pub fn controller_mut(&mut self) -> &mut PendulumController {
        &mut self.controller
    }

    pub fn controller(&self) -> &PendulumController {
        &self.controller
    }

    pub fn reset_runtime(&mut self) {
        self.controller.reset_runtime();
        self.status.control_mode = None;
    }

    pub fn prepare_for_run(&mut self) {
        self.reset_runtime();
        self.status.state = DeviceState::Running;
        self.status.fault = None;
    }

    pub fn transition_to_service(&mut self) {
        self.reset_runtime();
        self.status.state = DeviceState::Service;
        self.status.fault = None;
    }

    pub fn set_control_mode(&mut self, control_mode: Option<PendulumControlMode>) {
        self.status.control_mode = control_mode;
    }

    pub fn set_fault(&mut self, fault: Option<DeviceFault>) {
        self.status.fault = fault;
    }

    pub fn handle_request<S>(&mut self, request: DeviceRequest, services: &mut S) -> DeviceReply
    where
        S: DeviceServices,
    {
        match request {
            DeviceRequest::GetInfo => {
                DeviceReply::new(DeviceResponse::Info(services.device_info()))
            }
            DeviceRequest::GetStatus => {
                self.sync_status();
                DeviceReply::new(DeviceResponse::Status(self.status.clone()))
            }
            DeviceRequest::GetWifiStatus => {
                if self.status.mode != DeviceMode::Manufacturing {
                    DeviceReply::new(DeviceResponse::Error(
                        DeviceCommandError::UnsupportedInCurrentMode,
                    ))
                } else {
                    DeviceReply::new(DeviceResponse::WifiStatus(self.config.wifi_status()))
                }
            }
            DeviceRequest::GetCalibrationStatus => {
                if self.status.mode != DeviceMode::Manufacturing {
                    DeviceReply::new(DeviceResponse::Error(
                        DeviceCommandError::UnsupportedInCurrentMode,
                    ))
                } else {
                    DeviceReply::new(DeviceResponse::CalibrationStatus(
                        self.status.calibration.clone(),
                    ))
                }
            }
            DeviceRequest::SetMode(mode) => self.handle_set_mode(mode, services),
            DeviceRequest::SetWifiConfig(credentials) => {
                self.handle_set_wifi_config(credentials, services)
            }
            DeviceRequest::ClearWifiConfig => self.handle_clear_wifi_config(services),
            DeviceRequest::ValidateWifi => self.handle_validate_wifi(services),
            DeviceRequest::StartMotorCalibration => self.handle_start_motor_calibration(services),
            DeviceRequest::StartRun => self.handle_start_run(services),
            DeviceRequest::StopRun => self.handle_stop_run(),
            DeviceRequest::Reboot => DeviceReply::reboot(DeviceResponse::Ack),
        }
    }

    fn handle_set_mode<S>(&mut self, mode: DeviceMode, services: &mut S) -> DeviceReply
    where
        S: DeviceServices,
    {
        if let DeviceMode::Production = mode {
            if let Some(fault) = production_fault(&self.config, &self.status.calibration) {
                return DeviceReply::new(DeviceResponse::Error(
                    DeviceCommandError::ProductionPrecondition(fault),
                ));
            }
        }

        self.config.mode = mode;
        self.sync_status();
        if services.save_device_config(&self.config).is_err() {
            return DeviceReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed));
        }

        DeviceReply::reboot(DeviceResponse::Ack)
    }

    fn handle_set_wifi_config<S>(
        &mut self,
        credentials: WifiCredentials,
        services: &mut S,
    ) -> DeviceReply
    where
        S: DeviceServices,
    {
        if !self.in_manufacturing_service() {
            return DeviceReply::new(DeviceResponse::Error(self.service_mutation_error()));
        }

        let mut pending_config = self.config.clone();
        pending_config.wifi = Some(credentials.clone());
        if services.save_device_config(&pending_config).is_err() {
            return DeviceReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed));
        }

        self.config = pending_config;
        self.sync_status();
        let result = services.validate_wifi(&credentials);
        DeviceReply::new(DeviceResponse::WifiValidation(WifiValidationReport {
            status: self.config.wifi_status(),
            result: result.result,
        }))
    }

    fn handle_clear_wifi_config<S>(&mut self, services: &mut S) -> DeviceReply
    where
        S: DeviceServices,
    {
        if !self.in_manufacturing_service() {
            return DeviceReply::new(DeviceResponse::Error(self.service_mutation_error()));
        }

        let mut next_config = self.config.clone();
        next_config.wifi = None;
        if services.save_device_config(&next_config).is_err() {
            return DeviceReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed));
        }

        self.config = next_config;
        self.sync_status();
        DeviceReply::new(DeviceResponse::Ack)
    }

    fn handle_validate_wifi<S>(&mut self, services: &mut S) -> DeviceReply
    where
        S: DeviceServices,
    {
        if matches!(
            self.status.state,
            DeviceState::Running | DeviceState::Calibrating
        ) {
            return DeviceReply::new(DeviceResponse::Error(DeviceCommandError::InvalidState));
        }

        let Some(credentials) = self.config.wifi.as_ref() else {
            return DeviceReply::new(DeviceResponse::Error(
                DeviceCommandError::ProductionPrecondition(DeviceFault::MissingWifiConfig),
            ));
        };

        let result = services.validate_wifi(credentials);
        DeviceReply::new(DeviceResponse::WifiValidation(result))
    }

    fn handle_start_motor_calibration<S>(&mut self, services: &mut S) -> DeviceReply
    where
        S: DeviceServices,
    {
        if !self.in_manufacturing_service() {
            return DeviceReply::new(DeviceResponse::Error(self.service_mutation_error()));
        }

        self.status.state = DeviceState::Calibrating;
        let calibration = services.calibrate_motor();
        let Some(calibration) = calibration.ok().flatten() else {
            self.transition_to_service();
            return DeviceReply::new(DeviceResponse::Error(DeviceCommandError::CalibrationFailed));
        };

        if services.save_motor_calibration(&calibration).is_err() {
            self.transition_to_service();
            return DeviceReply::new(DeviceResponse::Error(DeviceCommandError::PersistenceFailed));
        }

        self.status.calibration = CalibrationStatus::Valid;
        self.transition_to_service();
        DeviceReply::new(DeviceResponse::CalibrationStatus(
            self.status.calibration.clone(),
        ))
    }

    fn handle_start_run<S>(&mut self, _services: &mut S) -> DeviceReply
    where
        S: DeviceServices,
    {
        if !self.in_manufacturing_service() {
            return DeviceReply::new(DeviceResponse::Error(self.service_mutation_error()));
        }

        if let Some(fault) = self.run_precondition_fault() {
            return DeviceReply::new(DeviceResponse::Error(
                DeviceCommandError::ProductionPrecondition(fault),
            ));
        }

        self.prepare_for_run();
        DeviceReply::new(DeviceResponse::Ack)
    }

    fn handle_stop_run(&mut self) -> DeviceReply {
        if self.status.mode != DeviceMode::Manufacturing {
            return DeviceReply::new(DeviceResponse::Error(
                DeviceCommandError::UnsupportedInCurrentMode,
            ));
        }

        if self.status.state != DeviceState::Running {
            return DeviceReply::new(DeviceResponse::Error(DeviceCommandError::InvalidState));
        }

        self.transition_to_service();
        DeviceReply::new(DeviceResponse::Ack)
    }

    fn sync_status(&mut self) {
        self.status.mode = self.config.mode.clone();
        self.status.wifi = self.config.wifi_status();
        self.status.calibration = self.status.calibration.clone();
    }

    fn in_manufacturing_service(&self) -> bool {
        self.status.mode == DeviceMode::Manufacturing && self.status.state == DeviceState::Service
    }

    fn service_mutation_error(&self) -> DeviceCommandError {
        if self.status.mode != DeviceMode::Manufacturing {
            DeviceCommandError::UnsupportedInCurrentMode
        } else {
            DeviceCommandError::InvalidState
        }
    }

    fn run_precondition_fault(&self) -> Option<DeviceFault> {
        match self.status.calibration {
            CalibrationStatus::Missing => Some(DeviceFault::MissingCalibration),
            CalibrationStatus::Invalid => Some(DeviceFault::InvalidCalibration),
            CalibrationStatus::Valid => None,
        }
    }
}

impl From<StoredDeviceConfig> for DeviceRuntime {
    fn from(config: StoredDeviceConfig) -> Self {
        let status = DeviceStatus {
            mode: config.mode.clone(),
            state: DeviceState::Service,
            fault: None,
            wifi: config.wifi_status(),
            calibration: CalibrationStatus::Missing,
            control_mode: None,
        };

        Self::new(config, status, PendulumController::new(Default::default()))
    }
}
