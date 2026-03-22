use std::{sync::Mutex, thread, time::Duration};

use bevy_ecs::prelude::*;

use pendulum_lib::{
    config::RuntimeConfig,
    imu::Imu,
    motor::Motor,
    runtime::core::{
        ControlClock, ControllerResource, ImuReading, MotorState, StepRuntime, TelemetryPublisher,
        WheelAngleEstimate, advance_clock_system, pd_control_system, publish_telemetry_system,
        run_loop,
    },
    telemetry::TelemetrySender,
};

use crate::hw::{Mpu6050Imu, SparkfunIotMotor, SparkfunIotMotorError};

#[derive(Debug)]
pub enum HardwareRuntimeError {
    Imu(crate::hw::mpu6050::Mpu6050ImuError),
    Motor(SparkfunIotMotorError),
}

#[derive(Resource)]
struct HwImuResource {
    imu: Mutex<Mpu6050Imu>,
}

#[derive(Resource)]
struct HwMotorResource {
    motor: Mutex<SparkfunIotMotor>,
}

pub struct HardwareRuntime {
    world: World,
    schedule: Schedule,
    step_dt: Duration,
}

impl HardwareRuntime {
    pub fn new(
        config: RuntimeConfig,
        telemetry: TelemetrySender,
    ) -> Result<Self, HardwareRuntimeError> {
        let imu = Mpu6050Imu::new().map_err(HardwareRuntimeError::Imu)?;
        let motor = SparkfunIotMotor::new(
            config.max_motor_torque_nm,
            config.motor_no_load_speed_rad_s,
            config.motor_torque_constant_nm_per_a,
        )
        .map_err(HardwareRuntimeError::Motor)?;

        let mut world = World::new();
        world.insert_resource(ControlClock::new(config.dt_s));
        world.insert_resource(ControllerResource::new(
            config.controller_kp,
            config.controller_kd,
        ));
        world.insert_resource(ImuReading::default());
        world.insert_resource(MotorState::default());
        world.insert_resource(WheelAngleEstimate::default());
        world.insert_resource(TelemetryPublisher { sender: telemetry });
        world.insert_resource(HwImuResource {
            imu: Mutex::new(imu),
        });
        world.insert_resource(HwMotorResource {
            motor: Mutex::new(motor),
        });

        let mut schedule = Schedule::default();
        schedule.add_systems(
            (
                hw_sample_imu_system,
                pd_control_system,
                hw_command_motor_system,
                integrate_wheel_angle_system,
                advance_clock_system,
                publish_telemetry_system,
            )
                .chain(),
        );

        Ok(Self {
            world,
            schedule,
            step_dt: Duration::from_secs_f64(config.dt_s),
        })
    }
}

impl StepRuntime for HardwareRuntime {
    fn step(&mut self) {
        self.schedule.run(&mut self.world);
    }

    fn step_dt(&self) -> Duration {
        self.step_dt
    }
}

pub fn spawn_hardware_runtime(
    config: RuntimeConfig,
    telemetry: TelemetrySender,
) -> Result<thread::JoinHandle<()>, HardwareRuntimeError> {
    let runtime = HardwareRuntime::new(config, telemetry)?;
    Ok(thread::spawn(move || run_loop(runtime)))
}

fn hw_sample_imu_system(imu_device: Res<'_, HwImuResource>, mut imu: ResMut<'_, ImuReading>) {
    let mut imu_device = imu_device.imu.lock().expect("imu mutex poisoned");
    match imu_device.read() {
        Ok(sample) => imu.sample = sample,
        Err(error) => panic!("MPU-6050 read failed: {error:?}"),
    }
}

fn hw_command_motor_system(
    motor_device: Res<'_, HwMotorResource>,
    mut motor_state: ResMut<'_, MotorState>,
) {
    let mut motor_device = motor_device.motor.lock().expect("motor mutex poisoned");
    match motor_device.command(motor_state.command) {
        Ok(telemetry) => motor_state.telemetry = telemetry,
        Err(error) => panic!("SparkFun motor command failed: {error:?}"),
    }
}

fn integrate_wheel_angle_system(
    clock: Res<'_, ControlClock>,
    motor_state: Res<'_, MotorState>,
    mut wheel_angle: ResMut<'_, WheelAngleEstimate>,
) {
    wheel_angle.angle_rad += motor_state.telemetry.wheel_speed_rad_s * clock.dt_s;
}
