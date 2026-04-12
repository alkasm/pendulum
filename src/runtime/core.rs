use std::{
    thread,
    time::{Duration, Instant},
};

use bevy_ecs::prelude::*;

use crate::{
    controller::PdController,
    imu::ImuSample,
    motor::{MotorCommand, MotorTelemetry},
    telemetry::{TelemetryFrame, TelemetrySender},
};
use uom::si::{f64::{Angle, Time}, time::second};

pub trait StepRuntime: Send + 'static {
    fn step(&mut self);
    fn step_dt(&self) -> Duration;
}

#[derive(Resource, Debug, Clone, Copy)]
pub struct ControllerResource {
    controller: PdController,
}

impl ControllerResource {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self {
            controller: PdController::new(kp, kd),
        }
    }
}

#[derive(Resource, Debug, Clone, Copy)]
pub struct ControlClock {
    pub step: u64,
    pub sim_time: Time,
    pub dt: Time,
}

impl ControlClock {
    pub fn new(dt: Time) -> Self {
        Self {
            step: 0,
            sim_time: Time::new::<second>(0.0),
            dt,
        }
    }
}

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct ImuReading {
    pub sample: ImuSample,
}

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct MotorState {
    pub command: MotorCommand,
    pub telemetry: MotorTelemetry,
}

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct WheelAngleEstimate {
    pub angle: Angle,
}

#[derive(Resource, Clone)]
pub struct TelemetryPublisher {
    pub sender: TelemetrySender,
}

pub fn run_loop<R>(mut runtime: R)
where
    R: StepRuntime,
{
    loop {
        let loop_start = Instant::now();
        runtime.step();
        let elapsed = loop_start.elapsed();
        let step_dt = runtime.step_dt();

        if elapsed < step_dt {
            thread::sleep(step_dt - elapsed);
        }
    }
}

pub fn pd_control_system(
    controller: Res<'_, ControllerResource>,
    imu: Res<'_, ImuReading>,
    mut motor_state: ResMut<'_, MotorState>,
) {
    let torque_command = controller
        .controller
        .torque_command(imu.sample.theta, imu.sample.theta_dot);
    motor_state.command = MotorCommand {
        torque_command,
        observed_wheel_speed: motor_state.telemetry.wheel_speed,
    };
}

pub fn advance_clock_system(mut clock: ResMut<'_, ControlClock>) {
    clock.step += 1;
    let dt = clock.dt;
    clock.sim_time += dt;
}

pub fn publish_telemetry_system(
    publisher: Res<'_, TelemetryPublisher>,
    clock: Res<'_, ControlClock>,
    imu: Res<'_, ImuReading>,
    motor_state: Res<'_, MotorState>,
    wheel_angle: Res<'_, WheelAngleEstimate>,
) {
    publisher.sender.send(TelemetryFrame {
        step: clock.step,
        sim_time: clock.sim_time,
        theta: imu.sample.theta,
        theta_dot: imu.sample.theta_dot,
        wheel_angle: wheel_angle.angle,
        wheel_speed: motor_state.telemetry.wheel_speed,
        commanded_torque: motor_state.command.torque_command,
        applied_torque: motor_state.telemetry.applied_torque,
        available_torque: motor_state.telemetry.available_torque,
        speed_ratio: motor_state.telemetry.speed_ratio,
        phase_current: motor_state.telemetry.phase_current,
    });
}
