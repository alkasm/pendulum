#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();
#[path = "bringup.rs"]
mod bringup;
mod command;
#[path = "hw/mod.rs"]
mod hw;
mod hall;
mod imu;
mod math;
mod motor_drive;
mod motor_calibration;
mod settings;
mod wifi;

use bringup::{init_console, init_delay, init_primary_i2c, max_clock_config};
use command::{CommandPort, write_response};
use esp_hal::{
    Blocking,
    i2c::master::I2c,
    main,
    mcpwm::{
        McPwm, PeripheralClockConfig,
        operator::PwmPinConfig,
        timer::PwmWorkingMode,
    },
    rng::Rng,
    timer::timg::TimerGroup,
    time::{Duration, Instant, Rate},
    uart::Uart,
};
use hw::{CurrentSample, CurrentSensor};
use hall::read_hall_telemetry;
use imu::read_pendulum_estimate;
use motor_calibration::{
    HallElectricalCalibration, StoredMotorCalibration, calibrate_hall_electrical_cycle,
    refine_torque_phase_offset,
};
use motor_drive::{
    MotorDriveState, PWM_PERIOD_TICKS, PwmMotorDrive, low_side_pwm_config,
};
use math::wrap_degrees;
use pendulum_lib::{
    config::default_pendulum,
    controller::{ControllerInput, PendulumController},
    device::{boot_status, production_fault},
    estimation::PendulumImuEstimator,
    pendulum::PendulumGeometry,
    settings_record::RecordLoad,
    CalibrationStatus, DEVICE_PROTOCOL_VERSION, DeviceCommandError,
    DeviceFault, DeviceInfo, DeviceMode, DeviceRequest, DeviceResponse, DeviceState,
    DeviceStatus, FirmwareName, FirmwareVersion, HallTelemetry,
    PendulumControlMode, PendulumControlTelemetry, PendulumEstimateTelemetry,
    StoredDeviceConfig, WifiCredentials, WifiValidationReport,
};
use settings::SettingsStorage;
use wifi::WifiValidator;

const CONTROL_PERIOD_MS: u32 = 5;
const BASELINE_SAMPLES: u32 = 64;

const PWM_FREQUENCY_HZ: u32 = 32_000;

const SERVICE_POLL_DELAY_MS: u32 = 10;

struct App<'d> {
    serial: Uart<'d, Blocking>,
    delay: esp_hal::delay::Delay,
    settings: SettingsStorage,
    command_port: CommandPort,
    wifi_validator: WifiValidator<'d>,
    current_sensor: CurrentSensor<'d>,
    motor_drive: PwmMotorDrive<'d>,
    i2c: I2c<'d, Blocking>,
    hall_configured: bool,
    imu_verified: bool,
    imu_awake: bool,
    imu_estimator: PendulumImuEstimator,
    controller: PendulumController,
    motor_drive_state: MotorDriveState,
    geometry: PendulumGeometry,
    control_period: Duration,
    last_loop_start: Option<Instant>,
    config: StoredDeviceConfig,
    calibration: Option<HallElectricalCalibration>,
    calibration_status: CalibrationStatus,
    status: DeviceStatus,
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let mut current_sensor = CurrentSensor::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
    );
    let mut settings = SettingsStorage::new();
    let config_record = settings.load_device_config().unwrap_or(RecordLoad::Corrupt);
    let calibration_record = settings
        .load_motor_calibration_record()
        .unwrap_or(RecordLoad::Corrupt);
    let status = boot_status(&config_record, &calibration_record);
    let calibration_status = status.calibration.clone();
    let config = match config_record {
        RecordLoad::Valid(config) => config,
        RecordLoad::Missing | RecordLoad::Corrupt => StoredDeviceConfig::default(),
    };
    let calibration = match calibration_record {
        RecordLoad::Valid(calibration) if calibration.is_valid() => Some(calibration.into()),
        RecordLoad::Missing | RecordLoad::Valid(_) | RecordLoad::Corrupt => None,
    };

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
        uh,
        ul,
        vh,
        vl,
        wh,
        wl,
    );
    let i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let _ = current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    motor_drive.disable();
    motor_drive.coast();
    let timer_group = TimerGroup::new(peripherals.TIMG0);
    let wifi_validator =
        WifiValidator::new(timer_group.timer0, Rng::new(peripherals.RNG), peripherals.WIFI)
        .expect("failed to initialize Wi-Fi validator");

    let mut app = App {
        serial,
        delay,
        settings,
        command_port: CommandPort::new(),
        wifi_validator,
        current_sensor,
        motor_drive,
        i2c,
        hall_configured: false,
        imu_verified: false,
        imu_awake: false,
        imu_estimator: PendulumImuEstimator::new(dt_s()),
        controller: PendulumController::new(Default::default()),
        motor_drive_state: MotorDriveState::new(),
        geometry: default_pendulum().geometry,
        control_period: Duration::from_millis(CONTROL_PERIOD_MS as u64),
        last_loop_start: None,
        config,
        calibration,
        calibration_status,
        status,
    };

    app.run()
}

impl<'d> App<'d> {
    fn run(&mut self) -> ! {
        loop {
            match self.status.state {
                DeviceState::Service => self.service_loop(),
                DeviceState::Running => self.running_loop(),
                DeviceState::Fault => self.fault_loop(),
                DeviceState::Boot => {
                    self.transition_to_service();
                }
                DeviceState::Calibrating => {
                    self.transition_to_service();
                }
            }
        }
    }

    fn service_loop(&mut self) {
        self.transition_to_service();

        while matches!(self.status.state, DeviceState::Service) {
            if let Some(request) = self.command_port.poll(&mut self.serial) {
                self.handle_request(request);
                continue;
            }

            self.delay.delay_millis(SERVICE_POLL_DELAY_MS);
        }
    }

    fn fault_loop(&mut self) {
        self.disable_outputs();
        self.status.control_mode = None;

        while matches!(self.status.state, DeviceState::Fault) {
            if let Some(request) = self.command_port.poll(&mut self.serial) {
                self.handle_request(request);
                continue;
            }

            self.delay.delay_millis(SERVICE_POLL_DELAY_MS);
        }
    }

    fn running_loop(&mut self) {
        while matches!(self.status.state, DeviceState::Running) {
            let loop_start = Instant::now();
            let current_sample = self.current_sensor.read();
            let hall = read_hall_telemetry(&mut self.i2c, &mut self.hall_configured);
            let estimate = read_pendulum_estimate(
                &mut self.i2c,
                &mut self.imu_verified,
                &mut self.imu_awake,
                &mut self.imu_estimator,
                &self.geometry,
            );
            let control = update_control_loop(
                &mut self.controller,
                &mut self.motor_drive_state,
                &mut self.motor_drive,
                self.calibration,
                &hall,
                &estimate,
                &current_sample,
            );
            self.status.control_mode = Some(control.mode);
            self.status.fault = None;

            if let Some(request) = self.command_port.poll(&mut self.serial) {
                self.handle_request(request);
            }

            if !matches!(self.status.state, DeviceState::Running) {
                return;
            }

            while loop_start.elapsed() < self.control_period {
                core::hint::spin_loop();
            }
            self.last_loop_start = Some(loop_start);
        }
    }

    fn handle_request(&mut self, request: DeviceRequest) {
        match request {
            DeviceRequest::GetInfo => self.respond(DeviceResponse::Info(self.device_info())),
            DeviceRequest::GetStatus => {
                self.sync_status();
                self.respond(DeviceResponse::Status(self.status.clone()));
            }
            DeviceRequest::GetWifiStatus => {
                if self.status.mode != DeviceMode::Manufacturing {
                    self.respond_error(DeviceCommandError::UnsupportedInCurrentMode);
                } else {
                    self.respond(DeviceResponse::WifiStatus(self.config.wifi_status()));
                }
            }
            DeviceRequest::GetCalibrationStatus => {
                if self.status.mode != DeviceMode::Manufacturing {
                    self.respond_error(DeviceCommandError::UnsupportedInCurrentMode);
                } else {
                    self.respond(DeviceResponse::CalibrationStatus(
                        self.calibration_status.clone(),
                    ));
                }
            }
            DeviceRequest::SetMode(mode) => self.handle_set_mode(mode),
            DeviceRequest::SetWifiConfig(credentials) => self.handle_set_wifi_config(credentials),
            DeviceRequest::ClearWifiConfig => self.handle_clear_wifi_config(),
            DeviceRequest::ValidateWifi => self.handle_validate_wifi(),
            DeviceRequest::StartMotorCalibration => self.handle_start_motor_calibration(),
            DeviceRequest::StartRun => self.handle_start_run(),
            DeviceRequest::StopRun => self.handle_stop_run(),
            DeviceRequest::Reboot => self.reboot_with_ack(),
        }
    }

    fn handle_set_mode(&mut self, mode: DeviceMode) {
        if let DeviceMode::Production = mode {
            if let Some(fault) = production_fault(&self.config, &self.calibration_status) {
                self.respond_error(DeviceCommandError::ProductionPrecondition(fault));
                return;
            }
        }

        self.config.mode = mode;
        self.sync_status();
        if self.settings.save_device_config(&self.config).is_err() {
            self.respond_error(DeviceCommandError::PersistenceFailed);
            return;
        }

        self.reboot_with_ack();
    }

    fn handle_set_wifi_config(&mut self, credentials: WifiCredentials) {
        if !self.in_manufacturing_service() {
            self.respond_error(self.service_mutation_error());
            return;
        }

        let mut pending_config = self.config.clone();
        pending_config.wifi = Some(credentials.clone());
        if self.settings.save_device_config(&pending_config).is_err() {
            self.respond_error(DeviceCommandError::PersistenceFailed);
            return;
        }

        self.config = pending_config;
        self.sync_status();
        let result = self.wifi_validator.validate(&credentials, &self.delay);
        self.respond(DeviceResponse::WifiValidation(WifiValidationReport {
            status: self.config.wifi_status(),
            result,
        }));
    }

    fn handle_clear_wifi_config(&mut self) {
        if !self.in_manufacturing_service() {
            self.respond_error(self.service_mutation_error());
            return;
        }

        let mut next_config = self.config.clone();
        next_config.wifi = None;
        if self.settings.save_device_config(&next_config).is_err() {
            self.respond_error(DeviceCommandError::PersistenceFailed);
            return;
        }

        self.config = next_config;
        self.sync_status();
        self.respond(DeviceResponse::Ack);
    }

    fn handle_validate_wifi(&mut self) {
        if matches!(self.status.state, DeviceState::Running | DeviceState::Calibrating) {
            self.respond_error(DeviceCommandError::InvalidState);
            return;
        }

        let Some(credentials) = self.config.wifi.as_ref() else {
            self.respond_error(DeviceCommandError::ProductionPrecondition(
                DeviceFault::MissingWifiConfig,
            ));
            return;
        };

        let result = self.wifi_validator.validate(credentials, &self.delay);
        self.respond(DeviceResponse::WifiValidation(WifiValidationReport {
            status: self.config.wifi_status(),
            result,
        }));
    }

    fn handle_start_motor_calibration(&mut self) {
        if !self.in_manufacturing_service() {
            self.respond_error(self.service_mutation_error());
            return;
        }

        self.disable_outputs();
        self.status.state = DeviceState::Calibrating;
        self.status.control_mode = Some(PendulumControlMode::Calibrating);

        let calibration = calibrate_hall_electrical_cycle(
            &mut self.i2c,
            &mut self.hall_configured,
            &self.delay,
            &mut self.motor_drive,
        )
        .and_then(|calibration| {
            refine_torque_phase_offset(
                &mut self.i2c,
                &mut self.hall_configured,
                &self.delay,
                &mut self.motor_drive,
                calibration,
            )
            .or(Some(calibration))
        });

        let Some(calibration) = calibration else {
            self.transition_to_service();
            self.respond_error(DeviceCommandError::CalibrationFailed);
            return;
        };

        let stored = StoredMotorCalibration {
            direction_sign: calibration.direction_sign,
            electrical_offset_deg: wrap_degrees(calibration.electrical_offset_deg),
            torque_sign: calibration.torque_sign,
        };
        if self.settings.save_motor_calibration(&stored).is_err() {
            self.transition_to_service();
            self.respond_error(DeviceCommandError::PersistenceFailed);
            return;
        }

        self.calibration = Some(calibration);
        self.calibration_status = CalibrationStatus::Valid;
        self.transition_to_service();
        self.respond(DeviceResponse::CalibrationStatus(
            self.calibration_status.clone(),
        ));
    }

    fn handle_start_run(&mut self) {
        if !self.in_manufacturing_service() {
            self.respond_error(self.service_mutation_error());
            return;
        }

        if let Some(fault) = self.run_precondition_fault() {
            self.respond_error(DeviceCommandError::ProductionPrecondition(fault));
            return;
        }

        self.prepare_for_run();
        self.respond(DeviceResponse::Ack);
    }

    fn handle_stop_run(&mut self) {
        if self.status.mode != DeviceMode::Manufacturing {
            self.respond_error(DeviceCommandError::UnsupportedInCurrentMode);
            return;
        }

        if self.status.state != DeviceState::Running {
            self.respond_error(DeviceCommandError::InvalidState);
            return;
        }

        self.transition_to_service();
        self.respond(DeviceResponse::Ack);
    }

    fn prepare_for_run(&mut self) {
        self.disable_outputs();
        self.controller.reset_runtime();
        self.motor_drive_state.reset_runtime();
        self.imu_estimator.reset();
        self.last_loop_start = None;
        self.status.state = DeviceState::Running;
        self.status.fault = None;
        self.status.control_mode = None;
        self.sync_status();
    }

    fn transition_to_service(&mut self) {
        self.disable_outputs();
        self.controller.reset_runtime();
        self.motor_drive_state.reset_runtime();
        self.imu_estimator.reset();
        self.last_loop_start = None;
        self.status.state = DeviceState::Service;
        self.status.fault = None;
        self.status.control_mode = None;
        self.sync_status();
    }

    fn sync_status(&mut self) {
        self.status.mode = self.config.mode.clone();
        self.status.wifi = self.config.wifi_status();
        self.status.calibration = self.calibration_status.clone();
    }

    fn disable_outputs(&mut self) {
        self.motor_drive_state.disable_motor(&mut self.motor_drive);
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
        match self.calibration_status {
            CalibrationStatus::Missing => Some(DeviceFault::MissingCalibration),
            CalibrationStatus::Invalid => Some(DeviceFault::InvalidCalibration),
            CalibrationStatus::Valid => None,
        }
    }

    fn device_info(&self) -> DeviceInfo {
        let mut firmware_name = FirmwareName::new();
        let _ = firmware_name.push_str(env!("CARGO_PKG_NAME"));

        let mut firmware_version = FirmwareVersion::new();
        let _ = firmware_version.push_str(env!("CARGO_PKG_VERSION"));

        DeviceInfo {
            firmware_name,
            firmware_version,
            protocol_version: DEVICE_PROTOCOL_VERSION,
        }
    }

    fn reboot_with_ack(&mut self) -> ! {
        self.disable_outputs();
        write_response(&mut self.serial, &DeviceResponse::Ack);
        esp_hal::system::software_reset()
    }

    fn respond(&mut self, response: DeviceResponse) {
        write_response(&mut self.serial, &response);
    }

    fn respond_error(&mut self, error: DeviceCommandError) {
        self.respond(DeviceResponse::Error(error));
    }
}

fn update_control_loop(
    controller: &mut PendulumController,
    drive_state: &mut MotorDriveState,
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

    let output = controller.step(ControllerInput {
        hall_angle_deg: hall_measurement.map(|measurement| measurement.angle_deg),
        theta_deg: estimate_measurement.map(|measurement| measurement.theta_deg),
        theta_dot_dps: estimate_measurement.map(|measurement| measurement.theta_dot_dps),
        max_phase_current_a: max_phase_current_amps(current_sample),
        actuator_ready: actuator_calibration.is_some(),
    });

    drive_state.apply_output(
        motor_drive,
        &output,
        hall_measurement.map(|measurement| measurement.angle_deg),
        actuator_calibration,
    );
    drive_state.to_telemetry(&output, actuator_calibration)
}

fn dt_s() -> f32 {
    CONTROL_PERIOD_MS as f32 / 1_000.0
}

fn max_phase_current_amps(sample: &CurrentSample) -> f32 {
    let ina_u = sample.ina_u.amps.abs();
    let ina_v = sample.ina_v.amps.abs();
    let ina_w = sample.ina_w.amps.abs();
    let uv = if ina_u > ina_v { ina_u } else { ina_v };
    if uv > ina_w { uv } else { ina_w }
}
