#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();
#[path = "bringup.rs"]
mod bringup;
mod command;
#[path = "hw/mod.rs"]
mod hw;
mod math;
mod motor_calibration;
mod settings;
mod wifi;

use bringup::{
    HALL_SENSOR_ADDR, i2c_device_present, init_console, init_delay, init_primary_i2c,
    max_clock_config,
};
use command::{CommandPort, write_response};
use esp_hal::{
    Blocking,
    gpio::{Level, Output, OutputConfig},
    i2c::master::I2c,
    main,
    mcpwm::{
        McPwm, PeripheralClockConfig,
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
        timer::PwmWorkingMode,
    },
    peripherals::{GPIO5, MCPWM0},
    rng::Rng,
    timer::timg::TimerGroup,
    time::{Duration, Instant, Rate},
    uart::Uart,
};
use hw::{
    CurrentSample, CurrentSensor, GY521_DEFAULT_I2C_ADDR, read_raw_measurement, verify_address,
    wake_device,
};
use libm::{atan2f, sinf};
use math::{
    clamp, degrees_to_radians, unwrap_near, wrap_degrees,
};
use pendulum_lib::{
    config::default_pendulum,
    controller::{ControllerInput, PendulumController},
    device::{boot_status, production_fault},
    estimation::{PendulumImuEstimator, RawImuSample, Vector3f},
    pendulum::PendulumGeometry,
    settings_record::RecordLoad,
    CalibrationStatus, DEVICE_PROTOCOL_VERSION, DeviceCommandError,
    DeviceFault, DeviceInfo, DeviceMode, DeviceRequest, DeviceResponse, DeviceState,
    DeviceStatus, FirmwareName, FirmwareVersion, HallMeasurement, HallTelemetry,
    PendulumControlMode, PendulumControlTelemetry, PendulumEstimateTelemetry,
    StoredDeviceConfig, StoredMotorCalibration, WifiCredentials,
    WifiValidationReport,
};
use settings::SettingsStorage;
use wifi::WifiValidator;

const CONTROL_PERIOD_MS: u32 = 5;
const BASELINE_SAMPLES: u32 = 64;

const PWM_FREQUENCY_HZ: u32 = 32_000;
const PWM_PERIOD_TICKS: u16 = 2500;
const DEAD_ZONE: f32 = 0.02;
const VOLTAGE_POWER_SUPPLY_V: f32 = 5.0;
const VOLTAGE_LIMIT_V: f32 = 4.6;
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

const SERVICE_POLL_DELAY_MS: u32 = 10;

type PwmPinA<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, true>;
type PwmPinB<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, false>;

struct PwmMotorDrive<'a> {
    enable: Output<'a>,
    uh: PwmPinA<'a, 0>,
    ul: PwmPinB<'a, 0>,
    vh: PwmPinA<'a, 1>,
    vl: PwmPinB<'a, 1>,
    wh: PwmPinA<'a, 2>,
    wl: PwmPinB<'a, 2>,
}

#[derive(Clone, Copy)]
struct HallElectricalCalibration {
    direction_sign: f32,
    electrical_offset_deg: f32,
    torque_sign: f32,
}

impl From<StoredMotorCalibration> for HallElectricalCalibration {
    fn from(value: StoredMotorCalibration) -> Self {
        Self {
            direction_sign: value.direction_sign,
            electrical_offset_deg: value.electrical_offset_deg,
            torque_sign: value.torque_sign,
        }
    }
}

struct MotorDriveState {
    electrical_angle_deg: f32,
    uq_v: f32,
    motor_enabled: bool,
}

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

impl MotorDriveState {
    fn new() -> Self {
        Self {
            electrical_angle_deg: 0.0,
            uq_v: 0.0,
            motor_enabled: false,
        }
    }

    fn reset_runtime(&mut self) {
        self.electrical_angle_deg = 0.0;
        self.uq_v = 0.0;
        self.motor_enabled = false;
    }

    fn disable_motor(&mut self, motor_drive: &mut PwmMotorDrive<'_>) {
        motor_drive.disable();
        motor_drive.coast();
        self.uq_v = 0.0;
        self.motor_enabled = false;
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

    if matches!(output.mode, PendulumControlMode::WaitingForHall | PendulumControlMode::Startup) {
        drive_state.disable_motor(motor_drive);
        drive_state.electrical_angle_deg = 0.0;
    } else if !matches!(output.mode, PendulumControlMode::Idle | PendulumControlMode::Active) {
        drive_state.disable_motor(motor_drive);
    } else if let (Some(hall_measurement), Some(actuator_calibration)) =
        (hall_measurement, actuator_calibration)
    {
        let electrical_angle_deg =
            actuator_calibration.electrical_angle_deg(hall_measurement.angle_deg);

        if matches!(output.mode, PendulumControlMode::Idle) {
            let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
                0.0,
                degrees_to_radians(electrical_angle_deg),
                VOLTAGE_LIMIT_V,
            );
            motor_drive.enable();
            motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);
            drive_state.electrical_angle_deg = electrical_angle_deg;
            drive_state.uq_v = 0.0;
            drive_state.motor_enabled = true;
        } else {
            let uq_v = -VOLTAGE_LIMIT_V * output.drive_command * actuator_calibration.torque_sign;
            let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
                uq_v,
                degrees_to_radians(electrical_angle_deg),
                VOLTAGE_LIMIT_V,
            );
            motor_drive.enable();
            motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);
            drive_state.electrical_angle_deg = electrical_angle_deg;
            drive_state.uq_v = uq_v;
            drive_state.motor_enabled = true;
        }
    } else {
        drive_state.disable_motor(motor_drive);
    }

    let (direction_sign, torque_sign) = if let Some(actuator_calibration) = actuator_calibration {
        (
            actuator_calibration.direction_sign,
            actuator_calibration.torque_sign,
        )
    } else {
        (0.0, 0.0)
    };

    PendulumControlTelemetry {
        mode: output.mode,
        theta_error_deg: output.theta_error_deg,
        torque_command_nm: output.torque_command_nm,
        raw_drive_command: output.raw_drive_command,
        drive_command: output.drive_command,
        direction_sign,
        torque_sign,
        electrical_angle_deg: drive_state.electrical_angle_deg,
        uq_v: drive_state.uq_v,
        wheel_angle_deg: output.wheel_angle_deg,
        wheel_speed_dps: output.wheel_speed_dps,
        commutation_step: electrical_sector(drive_state.electrical_angle_deg),
        commutation_center_deg: sector_center_deg(drive_state.electrical_angle_deg),
        motor_enabled: drive_state.motor_enabled,
    }
}

impl<'a> PwmMotorDrive<'a> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        enable: GPIO5<'a>,
        uh: PwmPinA<'a, 0>,
        ul: PwmPinB<'a, 0>,
        vh: PwmPinA<'a, 1>,
        vl: PwmPinB<'a, 1>,
        wh: PwmPinA<'a, 2>,
        wl: PwmPinB<'a, 2>,
    ) -> Self {
        Self {
            enable: Output::new(enable, Level::Low, OutputConfig::default()),
            uh,
            ul,
            vh,
            vl,
            wh,
            wl,
        }
    }

    fn enable(&mut self) {
        self.enable.set_high();
    }

    fn disable(&mut self) {
        self.enable.set_low();
    }

    fn coast(&mut self) {
        self.uh.set_timestamp(0);
        self.vh.set_timestamp(0);
        self.wh.set_timestamp(0);
        self.ul.set_timestamp(PWM_PERIOD_TICKS);
        self.vl.set_timestamp(PWM_PERIOD_TICKS);
        self.wl.set_timestamp(PWM_PERIOD_TICKS);
    }

    fn set_phase_voltages(&mut self, ua_v: f32, ub_v: f32, uc_v: f32) {
        let dead = DEAD_ZONE * 0.5;
        let dc_a = clamp(ua_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_b = clamp(ub_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_c = clamp(uc_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);

        self.uh.set_timestamp(duty_to_ticks(dc_a - dead));
        self.ul.set_timestamp(duty_to_ticks(dc_a + dead));
        self.vh.set_timestamp(duty_to_ticks(dc_b - dead));
        self.vl.set_timestamp(duty_to_ticks(dc_b + dead));
        self.wh.set_timestamp(duty_to_ticks(dc_c - dead));
        self.wl.set_timestamp(duty_to_ticks(dc_c + dead));
    }
}

impl HallElectricalCalibration {
    fn electrical_angle_deg(&self, hall_angle_deg: f32) -> f32 {
        wrap_degrees(
            self.direction_sign * MOTOR_POLE_PAIRS * hall_angle_deg + self.electrical_offset_deg,
        )
    }
}

fn calibrate_hall_electrical_cycle(
    i2c: &mut I2c<'_, Blocking>,
    hall_configured: &mut bool,
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

        if let HallTelemetry::Measurement(measurement) = read_hall_telemetry(i2c, hall_configured) {
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
                pos_offset_sin_sum += sinf(degrees_to_radians(pos_offset_deg));
                pos_offset_cos_sum += sinf(
                    degrees_to_radians(pos_offset_deg) + core::f32::consts::FRAC_PI_2,
                );
                neg_offset_sin_sum += sinf(degrees_to_radians(neg_offset_deg));
                neg_offset_cos_sum += sinf(
                    degrees_to_radians(neg_offset_deg) + core::f32::consts::FRAC_PI_2,
                );
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
        wrap_degrees(atan2f(pos_offset_sin_sum, pos_offset_cos_sum) * (180.0 / core::f32::consts::PI))
    } else {
        wrap_degrees(atan2f(neg_offset_sin_sum, neg_offset_cos_sum) * (180.0 / core::f32::consts::PI))
    };

    Some(HallElectricalCalibration {
        direction_sign,
        electrical_offset_deg,
        torque_sign: 1.0,
    })
}

fn refine_torque_phase_offset(
    i2c: &mut I2c<'_, Blocking>,
    hall_configured: &mut bool,
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
                hall_configured,
                delay,
                motor_drive,
                calibration,
                candidate_offset_deg,
                candidate_torque_sign * PHASE_SEARCH_UQ_V,
            )?;
            let neg_travel_deg = measure_phase_search_travel(
                i2c,
                hall_configured,
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
    hall_configured: &mut bool,
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
        if let HallTelemetry::Measurement(measurement) = read_hall_telemetry(i2c, hall_configured) {
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

fn low_side_pwm_config() -> PwmPinConfig<false> {
    PwmPinConfig::new(
        PwmActions::<false>::empty()
            .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
            .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
        PwmUpdateMethod::SYNC_ON_ZERO,
    )
}

fn simplefoc_sine_pwm_phase_voltages(
    uq_v: f32,
    angle_el_rad: f32,
    voltage_limit_v: f32,
) -> (f32, f32, f32) {
    let ualpha = -sinf(angle_el_rad) * uq_v;
    let ubeta = sinf(angle_el_rad + core::f32::consts::FRAC_PI_2) * uq_v;

    let mut ua = ualpha;
    let mut ub = -0.5 * ualpha + 0.866_025_4 * ubeta;
    let mut uc = -0.5 * ualpha - 0.866_025_4 * ubeta;

    let center = voltage_limit_v * 0.5;
    ua += center;
    ub += center;
    uc += center;

    (
        clamp(ua, 0.0, voltage_limit_v),
        clamp(ub, 0.0, voltage_limit_v),
        clamp(uc, 0.0, voltage_limit_v),
    )
}

fn duty_to_ticks(duty: f32) -> u16 {
    let clamped = clamp(duty, 0.0, 1.0);
    (clamped * PWM_PERIOD_TICKS as f32 + 0.5) as u16
}

fn dt_s() -> f32 {
    CONTROL_PERIOD_MS as f32 / 1_000.0
}

fn electrical_sector(electrical_angle_deg: f32) -> u8 {
    ((wrap_degrees(electrical_angle_deg) / 60.0) as u8) % 6
}

fn sector_center_deg(electrical_angle_deg: f32) -> f32 {
    electrical_sector(electrical_angle_deg) as f32 * 60.0 + 30.0
}

fn max_phase_current_amps(sample: &CurrentSample) -> f32 {
    let ina_u = sample.ina_u.amps.abs();
    let ina_v = sample.ina_v.amps.abs();
    let ina_w = sample.ina_w.amps.abs();
    let uv = if ina_u > ina_v { ina_u } else { ina_v };
    if uv > ina_w { uv } else { ina_w }
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
    imu_estimator: &mut PendulumImuEstimator,
    geometry: &PendulumGeometry,
) -> PendulumEstimateTelemetry {
    if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
        *imu_verified = false;
        *imu_awake = false;
        imu_estimator.reset();
        return PendulumEstimateTelemetry::Missing;
    }

    if !*imu_verified {
        match verify_address(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_verified = true,
            Err(hw::Gy521Error::RegisterRead(_)) => {
                imu_estimator.reset();
                return PendulumEstimateTelemetry::Missing;
            }
            Err(hw::Gy521Error::UnexpectedWhoAmI(value)) => {
                imu_estimator.reset();
                return PendulumEstimateTelemetry::UnexpectedWhoAmI { value };
            }
        }
    }

    if !*imu_awake {
        match wake_device(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_awake = true,
            Err(register) => {
                imu_estimator.reset();
                return PendulumEstimateTelemetry::WakeError { register };
            }
        }
    }

    match read_raw_measurement(i2c, GY521_DEFAULT_I2C_ADDR) {
        Ok(measurement) => PendulumEstimateTelemetry::Measurement(imu_estimator.step(
            geometry,
            RawImuSample {
                accel_g: Vector3f {
                    x: measurement.ax_g,
                    y: measurement.ay_g,
                    z: measurement.az_g,
                },
                gyro_dps: Vector3f {
                    x: measurement.gx_dps,
                    y: measurement.gy_dps,
                    z: measurement.gz_dps,
                },
            },
        )),
        Err(register) => {
            imu_estimator.reset();
            PendulumEstimateTelemetry::ReadError { register }
        }
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
