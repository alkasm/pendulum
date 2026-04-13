use bevy_ecs::prelude::*;
use esp_hal::{
    Blocking,
    i2c::master::I2c,
    time::{Duration, Instant},
    uart::Uart,
};
use pendulum_lib::{
    DEVICE_PROTOCOL_VERSION, DeviceInfo, FirmwareName, FirmwareVersion, StoredDeviceConfig,
    StoredMotorCalibration,
    config::{RuntimeConfig, default_pendulum},
    estimation::PendulumImuEstimator,
    imu::ImuSample,
    pendulum::PendulumGeometry,
    runtime::{
        ControlInputs, ControlOutputs, DeviceModelResource, DeviceServices,
        HallElectricalCalibration, MotorDriveState, MotorTelemetryResource,
        PendingDeviceActionResult, PendingDevicePlan, PendingDeviceRequest, PendingDeviceResponse,
        PendingReboot, advance_clock_system, control_system, device_request_finalize_system,
        device_request_system, execute_device_action, initialize_runtime_world,
        max_phase_current_amps,
    },
    settings_record::RecordLoad,
};
use uom::si::{
    angle::degree,
    angular_velocity::degree_per_second,
    electric_current::ampere,
    f64::{Angle, AngularVelocity},
    time::second,
};

use crate::{
    command::{CommandPort, write_response},
    hall::{HallSensor, read_hall_telemetry},
    hw::CurrentSensor,
    imu::{Gy521Session, read_pendulum_estimate},
    math::wrap_degrees,
    motor_calibration::{
        StoredMotorCalibration as StoredCalibration, calibrate_hall_electrical_cycle,
        refine_torque_phase_offset,
    },
    motor_drive::PwmMotorDrive,
    settings::{SettingsError, SettingsStorage},
    wifi::WifiValidator,
};

const BASELINE_SAMPLES: u32 = 64;

pub struct FirmwareBoot {
    pub config_record: RecordLoad<StoredDeviceConfig>,
    pub calibration_record: RecordLoad<StoredMotorCalibration>,
    pub runtime_config: RuntimeConfig,
    pub geometry: PendulumGeometry,
}

pub fn load_boot_snapshot(mut settings: SettingsStorage) -> (SettingsStorage, FirmwareBoot) {
    let boot = FirmwareBoot {
        config_record: settings.load_device_config().unwrap_or(RecordLoad::Corrupt),
        calibration_record: settings
            .load_motor_calibration_record()
            .unwrap_or(RecordLoad::Corrupt),
        runtime_config: RuntimeConfig::default(),
        geometry: default_pendulum().geometry,
    };

    (settings, boot)
}

#[derive(Resource)]
pub struct FirmwarePlatform<'d> {
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
    motor_drive_state: MotorDriveState,
    geometry: PendulumGeometry,
}

impl<'d> FirmwarePlatform<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        serial: Uart<'d, Blocking>,
        delay: esp_hal::delay::Delay,
        settings: SettingsStorage,
        wifi_validator: WifiValidator<'d>,
        current_sensor: CurrentSensor<'d>,
        motor_drive: PwmMotorDrive<'d>,
        i2c: I2c<'d, Blocking>,
        geometry: PendulumGeometry,
        control_dt_s: f32,
    ) -> Self {
        let current_sensor = {
            let mut current_sensor = current_sensor;
            let _ = current_sensor.calibrate_baseline(BASELINE_SAMPLES);
            current_sensor
        };
        let motor_drive = {
            let mut motor_drive = motor_drive;
            motor_drive.disable();
            motor_drive.coast();
            motor_drive
        };

        Self {
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
            imu_estimator: PendulumImuEstimator::new(control_dt_s),
            motor_drive_state: MotorDriveState::new(),
            geometry,
        }
    }

    fn disable_outputs(&mut self) {
        self.motor_drive_state.disable_motor(&mut self.motor_drive);
    }
}

pub struct FirmwareRuntime<'d> {
    world: World,
    command_schedule: Schedule,
    control_schedule: Schedule,
    control_period: Duration,
    last_loop_start: Option<Instant>,
    _marker: core::marker::PhantomData<&'d ()>,
}

impl<'d> FirmwareRuntime<'d> {
    pub fn new(platform: FirmwarePlatform<'d>, boot: FirmwareBoot) -> Self {
        let mut world = World::new();
        initialize_runtime_world(
            &mut world,
            firmware_device_info(),
            boot.runtime_config.controller_config(),
            boot.runtime_config.dt,
            &boot.config_record,
            &boot.calibration_record,
        );
        world.insert_resource(platform);
        world.insert_resource(CommandScheduleActivity::default());

        let mut command_schedule = Schedule::default();
        command_schedule.add_systems(
            (
                reset_command_activity_system,
                poll_command_system,
                device_request_system,
                firmware_execute_effects_system,
                device_request_finalize_system,
                write_response_system,
            )
                .chain(),
        );

        let mut control_schedule = Schedule::default();
        control_schedule.add_systems(
            (
                sample_current_system,
                sample_hall_system,
                sample_imu_system,
                control_system,
                apply_motor_output_system,
                advance_clock_system,
            )
                .chain(),
        );

        Self {
            world,
            command_schedule,
            control_schedule,
            control_period: Duration::from_micros(
                (boot.runtime_config.dt.get::<second>() * 1_000_000.0) as u64,
            ),
            last_loop_start: None,
            _marker: core::marker::PhantomData,
        }
    }

    pub fn run(&mut self) -> ! {
        loop {
            self.drain_command_schedule();
            let loop_start = Instant::now();
            self.control_schedule.run(&mut self.world);
            self.drain_command_schedule();

            while loop_start.elapsed() < self.control_period {
                core::hint::spin_loop();
            }
            self.last_loop_start = Some(loop_start);
        }
    }

    fn drain_command_schedule(&mut self) {
        loop {
            self.command_schedule.run(&mut self.world);
            let progressed = self
                .world
                .get_resource::<CommandScheduleActivity>()
                .expect("command schedule activity missing")
                .progressed;
            if !progressed {
                break;
            }
        }
    }
}

struct FirmwareServices<'a, 'd> {
    settings: &'a mut SettingsStorage,
    wifi_validator: &'a mut WifiValidator<'d>,
    delay: &'a esp_hal::delay::Delay,
    i2c: &'a mut I2c<'d, Blocking>,
    hall_sensor: &'a mut HallSensor,
    motor_drive: &'a mut PwmMotorDrive<'d>,
}

impl<'a, 'd> DeviceServices for FirmwareServices<'a, 'd> {
    type Error = SettingsError;

    fn device_info(&self) -> DeviceInfo {
        firmware_device_info()
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

    fn validate_wifi(
        &mut self,
        credentials: &pendulum_lib::WifiCredentials,
    ) -> pendulum_lib::WifiValidationReport {
        pendulum_lib::WifiValidationReport {
            status: pendulum_lib::WifiStatus {
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

        Ok(calibration.map(|calibration| StoredCalibration {
            direction_sign: calibration.direction_sign,
            electrical_offset_deg: wrap_degrees(calibration.electrical_offset_deg),
            torque_sign: calibration.torque_sign,
        }))
    }
}

#[derive(Resource, Default)]
struct CommandScheduleActivity {
    progressed: bool,
}

fn firmware_device_info() -> DeviceInfo {
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

fn reset_command_activity_system(mut activity: ResMut<'_, CommandScheduleActivity>) {
    activity.progressed = false;
}

fn poll_command_system(
    mut platform: ResMut<'_, FirmwarePlatform<'_>>,
    mut request: ResMut<'_, PendingDeviceRequest>,
    mut activity: ResMut<'_, CommandScheduleActivity>,
) {
    if request.0.is_some() {
        return;
    }

    let Some(next_request) = platform.command_port.poll(&mut platform.serial) else {
        return;
    };

    request.0 = Some(next_request);
    activity.progressed = true;
}

fn firmware_execute_effects_system(
    mut platform: ResMut<'_, FirmwarePlatform<'_>>,
    plan: Res<'_, PendingDevicePlan>,
    mut action_result: ResMut<'_, PendingDeviceActionResult>,
) {
    if action_result.0.is_some() {
        return;
    }

    let Some(pending_plan) = plan.0.as_ref() else {
        return;
    };

    let mut services = FirmwareServices {
        settings: &mut platform.settings,
        wifi_validator: &mut platform.wifi_validator,
        delay: &platform.delay,
        i2c: &mut platform.i2c,
        hall_sensor: &mut platform.hall_sensor,
        motor_drive: &mut platform.motor_drive,
    };
    action_result.0 = Some(execute_device_action(
        pending_plan.action.clone(),
        &mut services,
    ));
}

fn write_response_system(
    mut platform: ResMut<'_, FirmwarePlatform<'_>>,
    mut response: ResMut<'_, PendingDeviceResponse>,
    mut reboot: ResMut<'_, PendingReboot>,
) {
    let Some(reply) = response.0.take() else {
        return;
    };

    write_response(&mut platform.serial, &reply.response);
    let should_reboot = reboot.0;
    reboot.0 = false;
    if should_reboot {
        platform.disable_outputs();
        esp_hal::system::software_reset()
    }
}

fn sample_current_system(
    mut platform: ResMut<'_, FirmwarePlatform<'_>>,
    mut inputs: ResMut<'_, ControlInputs>,
    mut motor_telemetry: ResMut<'_, MotorTelemetryResource>,
) {
    let current_sample = platform.current_sensor.read();
    let phase_current = uom::si::f64::ElectricCurrent::new::<ampere>(max_phase_current_amps(
        current_sample.ina_u.amps,
        current_sample.ina_v.amps,
        current_sample.ina_w.amps,
    ) as f64);

    inputs.phase_current = phase_current;
    motor_telemetry.0.phase_current = phase_current;
}

fn sample_hall_system(
    mut platform: ResMut<'_, FirmwarePlatform<'_>>,
    mut inputs: ResMut<'_, ControlInputs>,
) {
    let hall = read_hall_telemetry(&mut platform.i2c, &mut platform.hall_sensor);
    inputs.wheel_angle = match hall {
        pendulum_lib::HallTelemetry::Measurement(measurement) => {
            Some(Angle::new::<degree>(measurement.angle_deg as f64))
        }
        _ => None,
    };
}

fn sample_imu_system(
    mut platform: ResMut<'_, FirmwarePlatform<'_>>,
    mut inputs: ResMut<'_, ControlInputs>,
) {
    let estimate = read_pendulum_estimate(
        &mut platform.i2c,
        &mut platform.imu_sensor,
        &mut platform.imu_estimator,
        &platform.geometry,
    );

    inputs.imu = match estimate {
        pendulum_lib::PendulumEstimateTelemetry::Measurement(measurement) => Some(ImuSample {
            theta: Angle::new::<degree>(measurement.theta_deg as f64),
            theta_dot: AngularVelocity::new::<degree_per_second>(measurement.theta_dot_dps as f64),
        }),
        _ => None,
    };
}

fn apply_motor_output_system(
    mut platform: ResMut<'_, FirmwarePlatform<'_>>,
    device: Res<'_, DeviceModelResource>,
    inputs: Res<'_, ControlInputs>,
    outputs: Res<'_, ControlOutputs>,
) {
    let Some(output) = outputs.controller_output else {
        platform.disable_outputs();
        return;
    };

    let hall_angle = inputs.wheel_angle.map(|angle| angle.get::<degree>() as f32);
    let calibration = device.0.calibration.map(HallElectricalCalibration::from);
    platform.motor_drive_state.apply_output(
        &mut platform.motor_drive,
        &output,
        hall_angle,
        calibration,
    );
}
