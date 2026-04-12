use std::{
    io::BufReader,
    net::{TcpListener, TcpStream},
    sync::mpsc::{self, Receiver, Sender, TryRecvError},
    thread,
    time::Duration,
};

use bevy_ecs::prelude::*;

use pendulum_lib::{
    DeviceInfo, DeviceMode, DeviceRequest, DeviceResponse, DeviceState, DeviceStatus, FirmwareName,
    FirmwareVersion, StoredDeviceConfig, StoredMotorCalibration, WifiCredentials, WifiProbeResult,
    WifiStatus,
    controller::PendulumController,
    imu::Imu,
    motor::Motor,
    runtime::{
        DeviceReply, DeviceRuntime, DeviceServices, StepRuntime,
        core::{
            ControlClock, ControllerResource, ImuReading, MotorState, TelemetryPublisher,
            WheelAngleEstimate, advance_clock_system, control_system, publish_telemetry_system,
            run_loop,
        },
    },
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

impl DeviceServices for SimServices {
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
    schedule: Schedule,
    step_dt: Duration,
    device_runtime: DeviceRuntime,
    services: SimServices,
    command_rx: Receiver<SimCommand>,
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
        let mut imu = SimImu::new();
        imu.sample_from_state(initial_state);
        let imu_sample = imu.read().expect("sim IMU should never fail");

        let (command_tx, command_rx) = mpsc::channel();
        spawn_tcp_command_server(command_addr.into(), command_tx);

        let mut world = World::new();
        world.insert_resource(ControlClock::new(runtime.dt));
        world.insert_resource(ControllerResource::new(PendulumController::new(
            runtime.controller_config(),
        )));
        world.insert_resource(ImuReading { sample: imu_sample });
        world.insert_resource(MotorState::default());
        world.insert_resource(WheelAngleEstimate {
            angle: initial_state.wheel_angle,
        });
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

        let mut schedule = Schedule::default();
        schedule.add_systems(
            (
                sim_sample_imu_system,
                control_system,
                sim_command_motor_system,
                sim_step_plant_system,
                advance_clock_system,
                publish_telemetry_system,
            )
                .chain(),
        );

        let device_status = DeviceStatus {
            mode: DeviceMode::Manufacturing,
            state: DeviceState::Service,
            fault: None,
            wifi: WifiStatus { ssid: None },
            calibration: pendulum_lib::CalibrationStatus::Valid,
            control_mode: None,
        };
        let device_runtime = DeviceRuntime::new(
            StoredDeviceConfig::default(),
            device_status,
            PendulumController::new(Default::default()),
        );

        Self {
            world,
            schedule,
            step_dt: Duration::from_secs_f64(runtime.dt.get::<second>()),
            device_runtime,
            services: SimServices::new(),
            command_rx,
        }
    }
}

impl StepRuntime for SimulationRuntime {
    fn step(&mut self) {
        self.drain_commands();
        self.schedule.run(&mut self.world);
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
                    let DeviceReply { response, reboot } = self
                        .device_runtime
                        .handle_request(command.request, &mut self.services);
                    let _ = command.response.send(response);
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
        self.device_runtime = DeviceRuntime::new(
            self.services.stored_config.clone(),
            DeviceStatus {
                mode: self.services.stored_config.mode.clone(),
                state: DeviceState::Service,
                fault: None,
                wifi: self.services.stored_config.wifi_status(),
                calibration: if self.services.stored_calibration.is_some() {
                    pendulum_lib::CalibrationStatus::Valid
                } else {
                    pendulum_lib::CalibrationStatus::Missing
                },
                control_mode: None,
            },
            PendulumController::new(Default::default()),
        );
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
    mut imu: ResMut<'_, pendulum_lib::runtime::core::ImuReading>,
) {
    imu_device.imu.sample_from_state(plant.plant.state());
    imu.sample = imu_device.imu.read().expect("sim IMU should never fail");
}

fn sim_command_motor_system(
    plant: Res<'_, SimPlantResource>,
    mut motor_device: ResMut<'_, SimMotorResource>,
    mut motor_state: ResMut<'_, pendulum_lib::runtime::core::MotorState>,
) {
    motor_state.command.observed_wheel_speed = plant.plant.state().wheel_speed;
    motor_state.telemetry = motor_device
        .motor
        .command(motor_state.command)
        .expect("sim motor should never fail");
}

fn sim_step_plant_system(
    clock: Res<'_, pendulum_lib::runtime::core::ControlClock>,
    mut plant_resource: ResMut<'_, SimPlantResource>,
    motor_state: Res<'_, pendulum_lib::runtime::core::MotorState>,
    mut wheel_angle: ResMut<'_, pendulum_lib::runtime::core::WheelAngleEstimate>,
) {
    plant_resource.plant.step(
        motor_state.telemetry.applied_torque,
        Duration::from_secs_f64(clock.dt.get::<second>()),
    );
    wheel_angle.angle = plant_resource.plant.state().wheel_angle;
}
