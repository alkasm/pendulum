#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();
#[path = "bringup.rs"]
mod bringup;
mod command;
mod hall;
#[path = "hw/mod.rs"]
mod hw;
mod imu;
mod math;
mod motor_calibration;
mod motor_drive;
mod settings;
mod wifi;

use bringup::{init_console, init_delay, init_primary_i2c, max_clock_config};
use command::{CommandPort, write_response};
use esp_hal::{
    Blocking,
    i2c::master::I2c,
    main,
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
    rng::Rng,
    time::{Duration, Instant, Rate},
    timer::timg::TimerGroup,
    uart::Uart,
};
use hall::{HallSensor, read_hall_telemetry};
use hw::CurrentSensor;
use imu::{Gy521Session, read_pendulum_estimate};
use math::wrap_degrees;
use motor_calibration::{StoredMotorCalibration, calibrate_hall_electrical_cycle, refine_torque_phase_offset};
use motor_drive::{PWM_PERIOD_TICKS, PwmMotorDrive, low_side_pwm_config};
use pendulum_lib::{
    CalibrationStatus, DEVICE_PROTOCOL_VERSION, DeviceInfo, DeviceRequest, DeviceResponse,
    DeviceState, DeviceStatus, FirmwareName, FirmwareVersion, StoredDeviceConfig,
    WifiCredentials, WifiStatus, WifiValidationReport,
    config::default_pendulum,
    controller::PendulumController,
    device::boot_status,
    estimation::PendulumImuEstimator,
    pendulum::PendulumGeometry,
    runtime::{
        DeviceRuntime, DeviceServices, HallElectricalCalibration, MotorDriveState,
        max_phase_current_amps, run_control_loop,
    },
    settings_record::RecordLoad,
};
use settings::SettingsError;
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
    hall_sensor: HallSensor,
    imu_sensor: Gy521Session,
    imu_estimator: PendulumImuEstimator,
    controller: PendulumController,
    motor_drive_state: MotorDriveState,
    device_runtime: DeviceRuntime,
    geometry: PendulumGeometry,
    control_period: Duration,
    last_loop_start: Option<Instant>,
    config: StoredDeviceConfig,
    calibration: Option<HallElectricalCalibration>,
    calibration_status: CalibrationStatus,
    status: DeviceStatus,
}

struct FirmwareServices<'a, 'd> {
    settings: &'a mut SettingsStorage,
    wifi_validator: &'a mut WifiValidator<'d>,
    delay: &'a esp_hal::delay::Delay,
    i2c: &'a mut I2c<'d, Blocking>,
    hall_sensor: &'a mut HallSensor,
    motor_drive: &'a mut PwmMotorDrive<'d>,
    last_calibration: Option<StoredMotorCalibration>,
}

impl<'a, 'd> DeviceServices for FirmwareServices<'a, 'd> {
    type Error = SettingsError;

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

    fn save_device_config(&mut self, config: &StoredDeviceConfig) -> Result<(), SettingsError> {
        self.settings.save_device_config(config)
    }

    fn save_motor_calibration(
        &mut self,
        calibration: &StoredMotorCalibration,
    ) -> Result<(), SettingsError> {
        self.settings.save_motor_calibration(calibration)
    }

    fn validate_wifi(&mut self, credentials: &WifiCredentials) -> WifiValidationReport {
        WifiValidationReport {
            status: WifiStatus {
                ssid: Some(credentials.ssid.clone()),
            },
            result: self.wifi_validator.validate(credentials, self.delay),
        }
    }

    fn calibrate_motor(&mut self) -> Result<Option<StoredMotorCalibration>, SettingsError> {
        self.motor_drive.disable();
        self.motor_drive.coast();
        let calibration = calibrate_hall_electrical_cycle(
            self.i2c,
            self.hall_sensor,
            self.delay,
            self.motor_drive,
        )
        .and_then(|calibration| {
            refine_torque_phase_offset(
                self.i2c,
                self.hall_sensor,
                self.delay,
                self.motor_drive,
                calibration,
            )
            .or(Some(calibration))
        });

        self.last_calibration = calibration.map(|calibration| StoredMotorCalibration {
            direction_sign: calibration.direction_sign,
            electrical_offset_deg: wrap_degrees(calibration.electrical_offset_deg),
            torque_sign: calibration.torque_sign,
        });

        Ok(self.last_calibration)
    }
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
    let mut motor_drive = PwmMotorDrive::new(peripherals.GPIO5, uh, ul, vh, vl, wh, wl);
    let i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let _ = current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    motor_drive.disable();
    motor_drive.coast();
    let timer_group = TimerGroup::new(peripherals.TIMG0);
    let wifi_validator = WifiValidator::new(
        timer_group.timer0,
        Rng::new(peripherals.RNG),
        peripherals.WIFI,
    )
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
        hall_sensor: HallSensor::new(),
        imu_sensor: Gy521Session::new(),
        imu_estimator: PendulumImuEstimator::new(dt_s()),
        controller: PendulumController::new(Default::default()),
        motor_drive_state: MotorDriveState::new(),
        device_runtime: DeviceRuntime::new(
            config.clone(),
            status.clone(),
            PendulumController::new(Default::default()),
        ),
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
            let hall = read_hall_telemetry(&mut self.i2c, &mut self.hall_sensor);
            let estimate = read_pendulum_estimate(
                &mut self.i2c,
                &mut self.imu_sensor,
                &mut self.imu_estimator,
                &self.geometry,
            );
            let control = run_control_loop(
                &mut self.controller,
                &mut self.motor_drive_state,
                &mut self.motor_drive,
                self.calibration,
                &hall,
                &estimate,
                max_phase_current_amps(
                    current_sample.ina_u.amps,
                    current_sample.ina_v.amps,
                    current_sample.ina_w.amps,
                ),
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
        let mut runtime =
            DeviceRuntime::new(self.config.clone(), self.status.clone(), self.controller);
        let mut services = FirmwareServices {
            settings: &mut self.settings,
            wifi_validator: &mut self.wifi_validator,
            delay: &self.delay,
            i2c: &mut self.i2c,
            hall_sensor: &mut self.hall_sensor,
            motor_drive: &mut self.motor_drive,
            last_calibration: None,
        };
        let reply = runtime.handle_request(request, &mut services);

        self.config = runtime.config().clone();
        self.status = runtime.status().clone();
        self.controller = *runtime.controller();
        self.device_runtime = runtime;
        if let Some(stored_calibration) = services.last_calibration {
            self.calibration = Some(stored_calibration.into());
        }
        self.calibration_status = self.status.calibration.clone();

        self.respond(reply.response);
        if reply.reboot {
            self.reboot_now();
        }
    }

    fn transition_to_service(&mut self) {
        self.disable_outputs();
        self.controller.reset_runtime();
        self.motor_drive_state.reset_runtime();
        self.hall_sensor.reset();
        self.imu_sensor.reset();
        self.imu_estimator.reset();
        self.last_loop_start = None;
        self.status.state = DeviceState::Service;
        self.status.fault = None;
        self.status.control_mode = None;
    }

    fn disable_outputs(&mut self) {
        self.motor_drive_state.disable_motor(&mut self.motor_drive);
    }

    fn reboot_now(&mut self) -> ! {
        self.disable_outputs();
        esp_hal::system::software_reset()
    }

    fn respond(&mut self, response: DeviceResponse) {
        write_response(&mut self.serial, &response);
    }
}

fn dt_s() -> f32 {
    CONTROL_PERIOD_MS as f32 / 1_000.0
}
