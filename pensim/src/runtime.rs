use std::{thread, time::Duration};

use bevy_ecs::prelude::*;

use pendulum_lib::{
    imu::Imu,
    motor::Motor,
    runtime::core::{
        ControlClock, ControllerResource, ImuReading, MotorState, StepRuntime, TelemetryPublisher,
        WheelAngleEstimate, advance_clock_system, pd_control_system, publish_telemetry_system,
        run_loop,
    },
    telemetry::TelemetrySender,
};
use uom::si::time::second;

use crate::sim::{SimConfig, SimImu, SimMotor, SimPlant};

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

pub struct SimulationRuntime {
    world: World,
    schedule: Schedule,
    step_dt: Duration,
}

impl SimulationRuntime {
    pub fn new(config: SimConfig, telemetry: TelemetrySender) -> Self {
        let runtime = config.runtime;
        let plant = SimPlant::new(config.plant_params(), config.initial_state());
        let initial_state = plant.state();
        let mut imu = SimImu::new();
        imu.sample_from_state(initial_state);
        let imu_sample = imu.read().expect("sim IMU should never fail");

        let mut world = World::new();
        world.insert_resource(ControlClock::new(runtime.dt));
        world.insert_resource(ControllerResource::new(
            runtime.controller_kp,
            runtime.controller_kd,
        ));
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
                pd_control_system,
                sim_command_motor_system,
                sim_step_plant_system,
                advance_clock_system,
                publish_telemetry_system,
            )
                .chain(),
        );

        Self {
            world,
            schedule,
            step_dt: Duration::from_secs_f64(runtime.dt.get::<second>()),
        }
    }
}

impl StepRuntime for SimulationRuntime {
    fn step(&mut self) {
        self.schedule.run(&mut self.world);
    }

    fn step_dt(&self) -> Duration {
        self.step_dt
    }
}

pub fn spawn_simulation_runtime(
    config: SimConfig,
    telemetry: TelemetrySender,
) -> thread::JoinHandle<()> {
    thread::spawn(move || run_loop(SimulationRuntime::new(config, telemetry)))
}

fn sim_sample_imu_system(
    plant: Res<'_, SimPlantResource>,
    mut imu_device: ResMut<'_, SimImuResource>,
    mut imu: ResMut<'_, ImuReading>,
) {
    imu_device.imu.sample_from_state(plant.plant.state());
    imu.sample = imu_device.imu.read().expect("sim IMU should never fail");
}

fn sim_command_motor_system(
    plant: Res<'_, SimPlantResource>,
    mut motor_device: ResMut<'_, SimMotorResource>,
    mut motor_state: ResMut<'_, MotorState>,
) {
    motor_state.command.observed_wheel_speed = plant.plant.state().wheel_speed;
    motor_state.telemetry = motor_device
        .motor
        .command(motor_state.command)
        .expect("sim motor should never fail");
}

fn sim_step_plant_system(
    clock: Res<'_, ControlClock>,
    mut plant_resource: ResMut<'_, SimPlantResource>,
    motor_state: Res<'_, MotorState>,
    mut wheel_angle: ResMut<'_, WheelAngleEstimate>,
) {
    plant_resource.plant.step(
        motor_state.telemetry.applied_torque,
        Duration::from_secs_f64(clock.dt.get::<second>()),
    );
    wheel_angle.angle = plant_resource.plant.state().wheel_angle;
}
