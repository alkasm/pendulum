use std::{
    io::BufReader,
    net::{TcpListener, TcpStream},
    sync::mpsc::{self, Receiver, Sender, TryRecvError},
    thread,
    time::Duration,
};

use bevy_ecs::prelude::*;

use pendulum_lib::{
    DeviceInfo, DeviceRequest, DeviceResponse, FirmwareName, FirmwareVersion, StoredDeviceConfig,
    StoredMotorCalibration, WifiCredentials, WifiProbeResult, WifiStatus,
    controller::ControllerConfig,
    imu::Imu,
    motor::Motor,
    runtime::{
        ControlClock, ControlInputs, ControlOutputs, DeviceModelResource, ManagementServices,
        MotorTelemetryResource, PendingDeviceActionResult, PendingDevicePlan, PendingDeviceRequest,
        PendingDeviceResponse, PendingReboot, StepRuntime, TelemetryPublisher,
        advance_clock_system, boot_device_model, capture_runtime_telemetry_system, control_system,
        device_request_finalize_system, device_request_system, execute_management_action,
        initialize_runtime_world, publish_telemetry_system, run_loop,
    },
    settings_record::RecordLoad,
    transport,
};
use uom::si::time::second;

use crate::sim::{SimConfig, SimImu, SimMotor, SimPlant};

pub const DEFAULT_COMMAND_ADDR: &str = "127.0.0.1:7003";

#[derive(Resource)]
struct SimPlantResource {
    plant: SimPlant,
}

#[derive(Resource)]
struct SimImuResource {
    imu: SimImu,
}

#[derive(Resource)]
struct SimMotorResource {
    motor: SimMotor,
}

struct SimCommand {
    request: DeviceRequest,
    response: Sender<DeviceResponse>,
}

#[derive(Resource)]
struct SimServices {
    device_info: DeviceInfo,
    stored_config: StoredDeviceConfig,
    stored_calibration: Option<StoredMotorCalibration>,
}

impl SimServices {
    fn new() -> Self {
        let mut firmware_name = FirmwareName::new();
        let _ = firmware_name.push_str("pensim");

        let mut firmware_version = FirmwareVersion::new();
        let _ = firmware_version.push_str(env!("CARGO_PKG_VERSION"));

        Self {
            device_info: DeviceInfo {
                firmware_name,
                firmware_version,
                protocol_version: pendulum_lib::DEVICE_PROTOCOL_VERSION,
            },
            stored_config: StoredDeviceConfig::default(),
            stored_calibration: Some(StoredMotorCalibration {
                direction_sign: 1.0,
                electrical_offset_deg: 0.0,
                torque_sign: 1.0,
            }),
        }
    }
}

impl ManagementServices for SimServices {
    type Error = core::convert::Infallible;

    fn device_info(&self) -> DeviceInfo {
        self.device_info.clone()
    }

    fn save_device_config(&mut self, config: &StoredDeviceConfig) -> Result<(), Self::Error> {
        self.stored_config = config.clone();
        Ok(())
    }

    fn save_motor_calibration(
        &mut self,
        calibration: &StoredMotorCalibration,
    ) -> Result<(), Self::Error> {
        self.stored_calibration = Some(*calibration);
        Ok(())
    }

    fn validate_wifi(
        &mut self,
        credentials: &WifiCredentials,
    ) -> pendulum_lib::WifiValidationReport {
        pendulum_lib::WifiValidationReport {
            status: WifiStatus {
                ssid: Some(credentials.ssid.clone()),
            },
            result: WifiProbeResult::Success {
                ipv4_octets: [127, 0, 0, 1],
            },
        }
    }

    fn calibrate_motor(&mut self) -> Result<Option<StoredMotorCalibration>, Self::Error> {
        Ok(self.stored_calibration.or(Some(StoredMotorCalibration {
            direction_sign: 1.0,
            electrical_offset_deg: 0.0,
            torque_sign: 1.0,
        })))
    }
}

pub struct SimulationRuntime {
    world: World,
    device_plan_schedule: Schedule,
    device_finalize_schedule: Schedule,
    control_schedule: Schedule,
    controller_config: ControllerConfig,
    step_dt: Duration,
    command_rx: Receiver<SimCommand>,
    services: SimServices,
}

impl SimulationRuntime {
    pub fn new(
        config: SimConfig,
        telemetry: pendulum_lib::telemetry::TelemetrySender,
        command_addr: impl Into<String>,
    ) -> Self {
        let runtime = config.runtime;
        let plant = SimPlant::new(config.plant_params(), config.initial_state());
        let initial_state = plant.state();
        let (imu, imu_sample) = {
            let mut imu = SimImu::new();
            imu.sample_from_state(initial_state);
            let imu_sample = imu.read().expect("sim IMU should never fail");
            (imu, imu_sample)
        };
        let services = SimServices::new();

        let (command_tx, command_rx) = mpsc::channel();
        spawn_tcp_command_server(command_addr.into(), command_tx);

        let mut world = World::new();
        initialize_runtime_world(
            &mut world,
            services.device_info(),
            runtime.controller_config(),
            runtime.dt,
            &RecordLoad::Valid(services.stored_config.clone()),
            &RecordLoad::Valid(
                services
                    .stored_calibration
                    .expect("sim calibration should exist"),
            ),
        );
        *world
            .get_resource_mut::<ControlInputs>()
            .expect("control inputs missing") = ControlInputs {
            wheel_angle: Some(initial_state.wheel_angle),
            imu: Some(imu_sample),
            phase_current: Default::default(),
        };
        world.insert_resource(TelemetryPublisher { sender: telemetry });
        world.insert_resource(SimPlantResource { plant });
        world.insert_resource(SimImuResource { imu });
        world.insert_resource(SimMotorResource {
            motor: SimMotor::new(
                runtime.max_motor_torque,
                runtime.motor_no_load_speed,
                runtime.motor_torque_constant_nm_per_a,
            ),
        });
        let mut device_plan_schedule = Schedule::default();
        device_plan_schedule.add_systems(device_request_system);

        let mut device_finalize_schedule = Schedule::default();
        device_finalize_schedule.add_systems(device_request_finalize_system);

        let mut control_schedule = Schedule::default();
        control_schedule.add_systems(
            (
                sim_sample_imu_system,
                control_system,
                sim_command_motor_system,
                sim_step_plant_system,
                advance_clock_system,
                capture_runtime_telemetry_system,
                publish_telemetry_system,
            )
                .chain(),
        );

        Self {
            world,
            device_plan_schedule,
            device_finalize_schedule,
            control_schedule,
            controller_config: runtime.controller_config(),
            step_dt: Duration::from_secs_f64(runtime.dt.get::<second>()),
            command_rx,
            services,
        }
    }
}

impl StepRuntime for SimulationRuntime {
    fn step(&mut self) {
        self.drain_commands();
        self.control_schedule.run(&mut self.world);
        self.drain_commands();
    }

    fn step_dt(&self) -> Duration {
        self.step_dt
    }
}

impl SimulationRuntime {
    fn drain_commands(&mut self) {
        loop {
            match self.command_rx.try_recv() {
                Ok(command) => {
                    {
                        let mut request = self
                            .world
                            .get_resource_mut::<PendingDeviceRequest>()
                            .expect("pending request resource missing");
                        request.0 = Some(command.request);
                    }
                    self.device_plan_schedule.run(&mut self.world);
                    if let Some(plan) = self
                        .world
                        .get_resource_mut::<PendingDevicePlan>()
                        .expect("pending plan resource missing")
                        .0
                        .take()
                    {
                        let result = execute_management_action(plan.action, &mut self.services);
                        {
                            let mut action_result = self
                                .world
                                .get_resource_mut::<PendingDeviceActionResult>()
                                .expect("pending action result resource missing");
                            action_result.0 = Some(result);
                        }
                        self.device_finalize_schedule.run(&mut self.world);
                    }
                    let response = self
                        .world
                        .get_resource_mut::<PendingDeviceResponse>()
                        .expect("pending response resource missing")
                        .0
                        .take()
                        .expect("device request produced no response");
                    let reboot = self
                        .world
                        .get_resource_mut::<PendingReboot>()
                        .expect("pending reboot resource missing")
                        .0;
                    self.world
                        .get_resource_mut::<PendingReboot>()
                        .expect("pending reboot resource missing")
                        .0 = false;
                    let _ = command.response.send(response.response);
                    if reboot {
                        self.reset_device_runtime();
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }
    }

    fn reset_device_runtime(&mut self) {
        let calibration_record = match self.services.stored_calibration {
            Some(calibration) => RecordLoad::Valid(calibration),
            None => RecordLoad::Missing,
        };
        let mut device = self
            .world
            .get_resource_mut::<DeviceModelResource>()
            .expect("device model missing");
        *device = DeviceModelResource(boot_device_model(
            &RecordLoad::Valid(self.services.stored_config.clone()),
            &calibration_record,
            self.controller_config,
        ));
    }
}

pub fn spawn_simulation_runtime(
    config: SimConfig,
    telemetry: pendulum_lib::telemetry::TelemetrySender,
    command_addr: String,
) -> thread::JoinHandle<()> {
    thread::spawn(move || run_loop(SimulationRuntime::new(config, telemetry, command_addr)))
}

fn spawn_tcp_command_server(bind_addr: String, command_tx: Sender<SimCommand>) {
    thread::spawn(move || {
        let listener = TcpListener::bind(&bind_addr).unwrap_or_else(|error| {
            panic!("Failed to bind sim command server on {bind_addr}: {error}")
        });

        for stream in listener.incoming() {
            match stream {
                Ok(stream) => {
                    let command_tx = command_tx.clone();
                    thread::spawn(move || serve_command_client(stream, command_tx));
                }
                Err(error) => panic!("Command accept failed on {bind_addr}: {error}"),
            }
        }
    });
}

fn serve_command_client(mut stream: TcpStream, command_tx: Sender<SimCommand>) {
    let mut reader = BufReader::new(
        stream
            .try_clone()
            .expect("failed to clone sim command stream"),
    );

    let request = match transport::read_cobs_message::<_, DeviceRequest>(&mut reader) {
        Ok(request) => request,
        Err(error) => {
            println!("Sim command client disconnected before request: {error}");
            return;
        }
    };

    let (response_tx, response_rx) = mpsc::channel();
    if command_tx
        .send(SimCommand {
            request,
            response: response_tx,
        })
        .is_err()
    {
        println!("Sim command runtime is unavailable");
        return;
    }

    let response = match response_rx.recv() {
        Ok(response) => response,
        Err(error) => {
            println!("Sim command response failed: {error}");
            return;
        }
    };

    if let Err(error) = transport::write_cobs_message(&mut stream, &response) {
        println!("Failed to write sim command response: {error}");
    }
}

fn sim_sample_imu_system(
    plant: Res<'_, SimPlantResource>,
    mut imu_device: ResMut<'_, SimImuResource>,
    mut inputs: ResMut<'_, ControlInputs>,
) {
    imu_device.imu.sample_from_state(plant.plant.state());
    let sample = imu_device.imu.read().expect("sim IMU should never fail");
    inputs.imu = Some(sample);
    inputs.wheel_angle = Some(plant.plant.state().wheel_angle);
}

fn sim_command_motor_system(
    plant: Res<'_, SimPlantResource>,
    mut motor_device: ResMut<'_, SimMotorResource>,
    mut control_outputs: ResMut<'_, ControlOutputs>,
    mut motor_telemetry: ResMut<'_, MotorTelemetryResource>,
) {
    control_outputs.motor_command.observed_wheel_speed = plant.plant.state().wheel_speed;
    motor_telemetry.0 = motor_device
        .motor
        .command(control_outputs.motor_command)
        .expect("sim motor should never fail");
}

fn sim_step_plant_system(
    clock: Res<'_, ControlClock>,
    mut plant_resource: ResMut<'_, SimPlantResource>,
    motor_telemetry: Res<'_, MotorTelemetryResource>,
    mut inputs: ResMut<'_, ControlInputs>,
) {
    plant_resource.plant.step(
        motor_telemetry.0.applied_torque,
        Duration::from_secs_f64(clock.dt.get::<second>()),
    );
    inputs.wheel_angle = Some(plant_resource.plant.state().wheel_angle);
}
