use crate::{
    CalibrationStatus, DeviceCommandError, DeviceFault, DeviceMode, DeviceState, DeviceStatus,
    StoredDeviceConfig, StoredMotorCalibration, controller::PendulumController,
    device::production_fault,
};

#[derive(Debug, Clone)]
pub struct DeviceModel {
    pub config: StoredDeviceConfig,
    pub status: DeviceStatus,
    pub controller: PendulumController,
    pub calibration: Option<StoredMotorCalibration>,
}

impl DeviceModel {
    pub fn new(
        config: StoredDeviceConfig,
        status: DeviceStatus,
        controller: PendulumController,
        calibration: Option<StoredMotorCalibration>,
    ) -> Self {
        Self {
            config,
            status,
            controller,
            calibration,
        }
    }

    pub fn sync_status(&mut self) {
        self.status.mode = self.config.mode.clone();
        self.status.wifi = self.config.wifi_status();
    }

    pub fn reset_runtime(&mut self) {
        self.controller.reset_runtime();
        self.status.control_mode = None;
    }

    pub fn transition_to_service(&mut self) {
        self.reset_runtime();
        self.status.state = DeviceState::Service;
        self.status.fault = None;
    }

    pub fn prepare_for_run(&mut self) {
        self.reset_runtime();
        self.status.state = DeviceState::Running;
        self.status.fault = None;
    }

    pub fn set_control_mode(&mut self, control_mode: Option<crate::PendulumControlMode>) {
        self.status.control_mode = control_mode;
    }

    pub fn set_fault(&mut self, fault: Option<DeviceFault>) {
        self.status.fault = fault;
    }

    pub fn set_calibration(&mut self, calibration: StoredMotorCalibration) {
        self.calibration = Some(calibration);
        self.status.calibration = CalibrationStatus::Valid;
    }

    pub fn in_manufacturing_service(&self) -> bool {
        self.status.mode == DeviceMode::Manufacturing && self.status.state == DeviceState::Service
    }

    pub fn service_mutation_error(&self) -> DeviceCommandError {
        if self.status.mode != DeviceMode::Manufacturing {
            DeviceCommandError::UnsupportedInCurrentMode
        } else {
            DeviceCommandError::InvalidState
        }
    }

    pub fn run_precondition_fault(&self) -> Option<DeviceFault> {
        production_fault(&self.config, &self.status.calibration)
    }

    pub fn calibration(&self) -> Option<&StoredMotorCalibration> {
        self.calibration.as_ref()
    }
}
