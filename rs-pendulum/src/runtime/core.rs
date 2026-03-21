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
    pub sim_time_s: f64,
    pub dt_s: f64,
}

impl ControlClock {
    pub fn new(dt_s: f64) -> Self {
        Self {
            step: 0,
            sim_time_s: 0.0,
            dt_s,
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
    pub angle_rad: f64,
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
    let torque_command_nm = controller
        .controller
        .torque_command(imu.sample.theta, imu.sample.theta_dot);
    motor_state.command = MotorCommand {
        torque_command_nm,
        observed_wheel_speed_rad_s: motor_state.telemetry.wheel_speed_rad_s,
    };
}

pub fn advance_clock_system(mut clock: ResMut<'_, ControlClock>) {
    clock.step += 1;
    clock.sim_time_s += clock.dt_s;
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
        sim_time_s: clock.sim_time_s,
        theta_rad: imu.sample.theta,
        theta_dot_rad_s: imu.sample.theta_dot,
        wheel_angle_rad: wheel_angle.angle_rad,
        wheel_speed_rad_s: motor_state.telemetry.wheel_speed_rad_s,
        commanded_torque_nm: motor_state.command.torque_command_nm,
        applied_torque_nm: motor_state.telemetry.applied_torque_nm,
        available_torque_nm: motor_state.telemetry.available_torque_nm,
        speed_ratio: motor_state.telemetry.speed_ratio,
        phase_current_a: motor_state.telemetry.phase_current_a,
    });
}
