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
        ControlInputs, ControlOutputs, DeviceModelResource, HallElectricalCalibration,
        ManagementServices, MotorDriveState, MotorTelemetryResource, PendingDeviceActionResult,
        PendingDevicePlan, PendingDeviceRequest, PendingDeviceResponse, PendingReboot,
        TelemetrySubsystem, advance_clock_system, capture_runtime_telemetry_system, control_system,
        device_request_finalize_system, device_request_system, execute_management_action,
        initialize_runtime_world, max_phase_current_amps,
    },
    settings_record::RecordLoad,
};
use uom::num_traits::float::FloatCore;
use uom::si::{
    angle::degree,
    angular_velocity::degree_per_second,
    electric_current::ampere,
    f64::{Angle, AngularVelocity, Time},
    time::microsecond,
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
    wifi::WifiService,
};

const BASELINE_SAMPLES: u32 = 64;

pub struct StartupConfig {
    pub device: RecordLoad<StoredDeviceConfig>,
    pub motor_calibration: RecordLoad<StoredMotorCalibration>,
    pub params: RuntimeConfig,
    pub geometry: PendulumGeometry,
}

pub fn load_startup_config(mut settings: SettingsStorage) -> (SettingsStorage, StartupConfig) {
    let startup = StartupConfig {
        device: settings.load_device_config().unwrap_or(RecordLoad::Corrupt),
        motor_calibration: settings
            .load_motor_calibration_record()
            .unwrap_or(RecordLoad::Corrupt),
        params: RuntimeConfig::default(),
        geometry: default_pendulum().geometry,
    };

    (settings, startup)
}

#[derive(Resource)]
pub struct Board {
    serial: Uart<'static, Blocking>,
    current_sensor: CurrentSensor<'static>,
    motor_drive: PwmMotorDrive<'static>,
    i2c: I2c<'static, Blocking>,
}

impl Board {
    pub fn new(
        serial: Uart<'static, Blocking>,
        current_sensor: CurrentSensor<'static>,
        motor_drive: PwmMotorDrive<'static>,
        i2c: I2c<'static, Blocking>,
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
            current_sensor,
            motor_drive,
            i2c,
        }
    }
}

#[derive(Resource)]
struct ManagementResources {
    delay: esp_hal::delay::Delay,
    settings: SettingsStorage,
    wifi: WifiService<'static>,
}

#[derive(Resource)]
struct RuntimeState {
    command_port: CommandPort,
    hall_sensor: HallSensor,
    imu_sensor: Gy521Session,
    imu_estimator: PendulumImuEstimator,
    motor_drive_state: MotorDriveState,
    geometry: PendulumGeometry,
}

impl RuntimeState {
    fn new(geometry: PendulumGeometry, control_dt: Time) -> Self {
        Self {
            command_port: CommandPort::new(),
            hall_sensor: HallSensor::new(),
            imu_sensor: Gy521Session::new(),
            imu_estimator: PendulumImuEstimator::new(control_dt),
            motor_drive_state: MotorDriveState::new(),
            geometry,
        }
    }

    fn disable_outputs<D: pendulum_lib::runtime::ControlDrive>(&mut self, motor_drive: &mut D) {
        self.motor_drive_state.disable_motor(motor_drive);
    }
}

pub struct FirmwareRuntime {
    world: World,
    // Handles variable-length command/response work. We only run this in the slack
    // before the next scheduled control tick so management traffic cannot arbitrarily
    // delay the control cadence.
    command_schedule: Schedule,
    // Handles the fixed-rate sensor -> controller -> actuator pipeline.
    control_schedule: Schedule,
    control_period: Duration,
}

impl FirmwareRuntime {
    pub fn new(
        board: Board,
        settings: SettingsStorage,
        delay: esp_hal::delay::Delay,
        wifi: WifiService<'static>,
        startup: StartupConfig,
    ) -> Self {
        let mut world = World::new();
        initialize_runtime_world(
            &mut world,
            firmware_device_info(),
            startup.params.controller_config(),
            startup.params.dt,
            &startup.device,
            &startup.motor_calibration,
        );
        world.insert_resource(board);
        world.insert_resource(ManagementResources {
            delay,
            settings,
            wifi,
        });
        world.insert_resource(RuntimeState::new(startup.geometry, startup.params.dt));
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
                capture_runtime_telemetry_system,
                firmware_stream_telemetry_system,
            )
                .chain(),
        );

        Self {
            world,
            command_schedule,
            control_schedule,
            control_period: Duration::from_micros(
                startup.params.dt.get::<microsecond>().round() as u64
            ),
        }
    }

    pub fn run(&mut self) -> ! {
        let mut next_control_start = Instant::now();

        loop {
            // Spend any slack before the next control tick on command processing.
            self.drain_command_schedule_until(next_control_start);
            // Once the control deadline arrives, run exactly one control tick.
            while Instant::now() < next_control_start {
                core::hint::spin_loop();
            }
            self.control_schedule.run(&mut self.world);
            next_control_start += self.control_period;
        }
    }

    fn drain_command_schedule_until(&mut self, deadline: Instant) {
        loop {
            if Instant::now() >= deadline {
                break;
            }

            self.command_schedule.run(&mut self.world);
            let progressed = self
                .world
                .get_resource::<CommandScheduleActivity>()
                .expect("command schedule activity missing")
                .progressed;
            // Stop once a full command pass made no forward progress, or once we've
            // consumed the slack before the next control deadline.
            if !progressed {
                break;
            }
        }
    }
}

struct ManagementAdapter<'a> {
    settings: &'a mut SettingsStorage,
    wifi: &'a mut WifiService<'static>,
    delay: &'a esp_hal::delay::Delay,
    i2c: &'a mut I2c<'static, Blocking>,
    hall_sensor: &'a mut HallSensor,
    motor_drive: &'a mut PwmMotorDrive<'static>,
}

impl<'a> ManagementServices for ManagementAdapter<'a> {
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
            result: self.wifi.validate(credentials, self.delay),
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
    // Set when a command pass consumed input from transport. This lets the outer loop rerun the
    // schedule while there is still both pending work and time left before the next control tick.
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
    mut board: ResMut<'_, Board>,
    mut runtime_state: ResMut<'_, RuntimeState>,
    mut request: ResMut<'_, PendingDeviceRequest>,
    mut activity: ResMut<'_, CommandScheduleActivity>,
) {
    if request.0.is_some() {
        return;
    }

    let Some(next_request) = runtime_state.command_port.poll(&mut board.serial) else {
        return;
    };

    request.0 = Some(next_request);
    activity.progressed = true;
}

fn firmware_execute_effects_system(
    mut resources: ResMut<'_, ManagementResources>,
    mut board: ResMut<'_, Board>,
    mut runtime_state: ResMut<'_, RuntimeState>,
    plan: Res<'_, PendingDevicePlan>,
    mut action_result: ResMut<'_, PendingDeviceActionResult>,
) {
    if action_result.0.is_some() {
        return;
    }

    let Some(pending_plan) = plan.0.as_ref() else {
        return;
    };

    let ManagementResources {
        delay,
        settings,
        wifi,
    } = &mut *resources;
    let Board {
        i2c, motor_drive, ..
    } = &mut *board;
    let RuntimeState { hall_sensor, .. } = &mut *runtime_state;
    let mut action_services = ManagementAdapter {
        settings,
        wifi,
        delay: &*delay,
        i2c,
        hall_sensor,
        motor_drive,
    };
    action_result.0 = Some(execute_management_action(
        pending_plan.action.clone(),
        &mut action_services,
    ));
}

fn firmware_stream_telemetry_system(
    device: Res<'_, DeviceModelResource>,
    telemetry: Res<'_, TelemetrySubsystem>,
    mut resources: ResMut<'_, ManagementResources>,
) {
    resources.wifi.stream_runtime_telemetry(
        device.0.config.wifi.as_ref(),
        telemetry.port,
        telemetry.latest_frame.as_ref(),
    );
}

fn write_response_system(
    mut board: ResMut<'_, Board>,
    mut runtime_state: ResMut<'_, RuntimeState>,
    mut response: ResMut<'_, PendingDeviceResponse>,
    mut reboot: ResMut<'_, PendingReboot>,
) {
    let Some(reply) = response.0.take() else {
        return;
    };

    write_response(&mut board.serial, &reply.response);
    let should_reboot = reboot.0;
    reboot.0 = false;
    if should_reboot {
        runtime_state.disable_outputs(&mut board.motor_drive);
        esp_hal::system::software_reset()
    }
}

fn sample_current_system(
    mut board: ResMut<'_, Board>,
    mut inputs: ResMut<'_, ControlInputs>,
    mut motor_telemetry: ResMut<'_, MotorTelemetryResource>,
) {
    let current_sample = board.current_sensor.read();
    let phase_current = uom::si::f64::ElectricCurrent::new::<ampere>(max_phase_current_amps(
        current_sample.ina_u.amps,
        current_sample.ina_v.amps,
        current_sample.ina_w.amps,
    ) as f64);

    inputs.phase_current = phase_current;
    motor_telemetry.0.phase_current = phase_current;
}

fn sample_hall_system(
    mut board: ResMut<'_, Board>,
    mut runtime_state: ResMut<'_, RuntimeState>,
    mut inputs: ResMut<'_, ControlInputs>,
) {
    let hall = read_hall_telemetry(&mut board.i2c, &mut runtime_state.hall_sensor);
    inputs.wheel_angle = match hall {
        pendulum_lib::HallTelemetry::Measurement(measurement) => {
            Some(Angle::new::<degree>(measurement.angle_deg as f64))
        }
        _ => None,
    };
}

fn sample_imu_system(
    mut board: ResMut<'_, Board>,
    mut runtime_state: ResMut<'_, RuntimeState>,
    mut inputs: ResMut<'_, ControlInputs>,
) {
    let RuntimeState {
        imu_sensor,
        imu_estimator,
        geometry,
        ..
    } = &mut *runtime_state;
    let estimate = read_pendulum_estimate(&mut board.i2c, imu_sensor, imu_estimator, &*geometry);

    inputs.imu = match estimate {
        pendulum_lib::PendulumEstimateTelemetry::Measurement(measurement) => Some(ImuSample {
            theta: Angle::new::<degree>(measurement.theta_deg as f64),
            theta_dot: AngularVelocity::new::<degree_per_second>(measurement.theta_dot_dps as f64),
        }),
        _ => None,
    };
}

fn apply_motor_output_system(
    mut board: ResMut<'_, Board>,
    mut runtime_state: ResMut<'_, RuntimeState>,
    device: Res<'_, DeviceModelResource>,
    inputs: Res<'_, ControlInputs>,
    outputs: Res<'_, ControlOutputs>,
) {
    let Some(output) = outputs.controller_output else {
        runtime_state.disable_outputs(&mut board.motor_drive);
        return;
    };

    let hall_angle = inputs.wheel_angle.map(|angle| angle.get::<degree>() as f32);
    let calibration = device.0.calibration.map(HallElectricalCalibration::from);
    runtime_state.motor_drive_state.apply_output(
        &mut board.motor_drive,
        &output,
        hall_angle,
        calibration,
    );
}
