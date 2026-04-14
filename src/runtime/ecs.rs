use bevy_ecs::prelude::*;

use super::{
    effects::{CommandReply, ManagementAction, ManagementActionCompletion, ManagementActionResult},
    lifecycle::{CommandPlan, boot_device_model, finalize_command_request, plan_command_request},
    model::DeviceModel,
};
use crate::{
    DeviceCommandError, DeviceInfo, DeviceRequest, DeviceState, StoredDeviceConfig,
    StoredMotorCalibration,
    controller::ControllerConfig,
    controller::ControllerOutput,
    motor::{MotorCommand, MotorTelemetry},
    runtime::step_controller,
};
use uom::si::{
    angle::degree,
    angular_velocity::degree_per_second,
    electric_current::ampere,
    f64::{Angle, AngularVelocity, Time, Torque},
    time::second,
    torque::newton_meter,
};

#[derive(Resource, Debug, Clone)]
pub struct DeviceModelResource(pub DeviceModel);

#[derive(Resource, Debug, Clone)]
pub struct DeviceInfoResource(pub DeviceInfo);

#[derive(Resource, Debug, Clone, Default)]
pub struct PendingDeviceRequest(pub Option<DeviceRequest>);

#[derive(Resource, Debug, Clone, Default)]
pub struct PendingDeviceResponse(pub Option<CommandReply>);

#[derive(Resource, Debug, Clone, Default)]
pub struct PendingDevicePlan(pub Option<PendingDeviceRequestPlan>);

#[derive(Debug, Clone)]
pub struct PendingDeviceRequestPlan {
    pub action: ManagementAction,
    pub completion: ManagementActionCompletion,
}

#[derive(Resource, Debug, Clone, Default)]
pub struct PendingDeviceActionResult(
    pub Option<Result<ManagementActionResult, DeviceCommandError>>,
);

#[derive(Resource, Debug, Clone, Default)]
pub struct PendingReboot(pub bool);

#[derive(Resource, Debug, Clone, Copy, Default)]
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
pub struct ControlInputs {
    pub wheel_angle: Option<Angle>,
    pub imu: Option<crate::imu::ImuSample>,
    pub phase_current: uom::si::f64::ElectricCurrent,
}

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct ControlOutputs {
    pub motor_command: MotorCommand,
    pub controller_output: Option<ControllerOutput>,
}

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct MotorTelemetryResource(pub MotorTelemetry);

#[derive(Resource, Debug, Clone, Copy)]
pub struct TelemetryState {
    // Shared runtime telemetry state. Platforms all capture the same latest frame here,
    // then their own transport systems decide how to publish it.
    pub port: u16,
    pub latest_frame: Option<crate::RuntimeTelemetryFrame>,
}

impl Default for TelemetryState {
    fn default() -> Self {
        Self {
            port: crate::DEFAULT_RUNTIME_TELEMETRY_PORT,
            latest_frame: None,
        }
    }
}

#[cfg(feature = "std")]
#[derive(Resource, Clone)]
pub struct TelemetryPublisher {
    pub sender: crate::telemetry::TelemetrySender,
}

pub fn runtime_telemetry_frame(
    clock: &ControlClock,
    inputs: &ControlInputs,
    outputs: &ControlOutputs,
    motor_telemetry: &MotorTelemetryResource,
) -> crate::RuntimeTelemetryFrame {
    let imu = inputs.imu.unwrap_or_default();
    let wheel_angle = inputs.wheel_angle.unwrap_or_default();

    crate::RuntimeTelemetryFrame {
        step: clock.step,
        sim_time: clock.sim_time,
        theta: imu.theta,
        theta_dot: imu.theta_dot,
        wheel_angle,
        wheel_speed: motor_telemetry.0.wheel_speed,
        commanded_torque: outputs.motor_command.torque_command,
        applied_torque: motor_telemetry.0.applied_torque,
        available_torque: motor_telemetry.0.available_torque,
        speed_ratio: motor_telemetry.0.speed_ratio,
        phase_current: motor_telemetry.0.phase_current,
    }
}

pub fn initialize_runtime_world(
    world: &mut World,
    device_info: DeviceInfo,
    controller_config: ControllerConfig,
    control_dt: Time,
    config_record: &crate::settings_record::RecordLoad<StoredDeviceConfig>,
    calibration_record: &crate::settings_record::RecordLoad<StoredMotorCalibration>,
) {
    world.insert_resource(ControlClock::new(control_dt));
    world.insert_resource(ControlInputs::default());
    world.insert_resource(ControlOutputs::default());
    world.insert_resource(MotorTelemetryResource::default());
    world.insert_resource(TelemetryState::default());
    world.insert_resource(PendingDeviceRequest::default());
    world.insert_resource(PendingDeviceResponse::default());
    world.insert_resource(PendingDevicePlan::default());
    world.insert_resource(PendingDeviceActionResult::default());
    world.insert_resource(PendingReboot::default());
    world.insert_resource(DeviceInfoResource(device_info));
    world.insert_resource(DeviceModelResource(boot_device_model(
        config_record,
        calibration_record,
        controller_config,
    )));
}

pub fn device_request_system(
    mut device: ResMut<'_, DeviceModelResource>,
    device_info: Res<'_, DeviceInfoResource>,
    mut request: ResMut<'_, PendingDeviceRequest>,
    mut response: ResMut<'_, PendingDeviceResponse>,
    mut plan: ResMut<'_, PendingDevicePlan>,
    mut action_result: ResMut<'_, PendingDeviceActionResult>,
    mut reboot: ResMut<'_, PendingReboot>,
) {
    let Some(request) = request.0.take() else {
        return;
    };

    plan.0 = None;
    action_result.0 = None;

    match plan_command_request(&mut device.0, request, device_info.0.clone()) {
        CommandPlan::Immediate(reply) => {
            reboot.0 = reply.reboot;
            response.0 = Some(reply);
        }
        CommandPlan::Pending { action, completion } => {
            plan.0 = Some(PendingDeviceRequestPlan { action, completion });
        }
    }
}

pub fn device_request_finalize_system(
    mut device: ResMut<'_, DeviceModelResource>,
    mut plan: ResMut<'_, PendingDevicePlan>,
    mut action_result: ResMut<'_, PendingDeviceActionResult>,
    mut response: ResMut<'_, PendingDeviceResponse>,
    mut reboot: ResMut<'_, PendingReboot>,
) {
    let Some(pending_plan) = plan.0.take() else {
        return;
    };

    let Some(outcome) = action_result.0.take() else {
        plan.0 = Some(pending_plan);
        return;
    };

    let reply = finalize_command_request(&mut device.0, pending_plan.completion, outcome);
    reboot.0 = reply.reboot;
    response.0 = Some(reply);
}

pub fn control_system(
    mut device: ResMut<'_, DeviceModelResource>,
    inputs: Res<'_, ControlInputs>,
    mut outputs: ResMut<'_, ControlOutputs>,
) {
    let actuator_ready = matches!(device.0.status.state, DeviceState::Running);
    let output = step_controller(
        &mut device.0.controller,
        inputs.wheel_angle.map(|angle| angle.get::<degree>() as f32),
        inputs.imu.map(|imu| imu.theta.get::<degree>() as f32),
        inputs
            .imu
            .map(|imu| imu.theta_dot.get::<degree_per_second>() as f32),
        inputs.phase_current.get::<ampere>() as f32,
        actuator_ready,
    );
    outputs.motor_command = MotorCommand {
        torque_command: Torque::new::<newton_meter>(output.torque_command_nm as f64),
        observed_wheel_speed: AngularVelocity::new::<degree_per_second>(
            output.wheel_speed_dps as f64,
        ),
    };
    outputs.controller_output = Some(output);
    device
        .0
        .set_control_mode(actuator_ready.then_some(output.mode));
    if actuator_ready {
        device.0.set_fault(None);
    }
}

pub fn advance_clock_system(mut clock: ResMut<'_, ControlClock>) {
    clock.step += 1;
    let dt = clock.dt;
    clock.sim_time += dt;
}

pub fn capture_runtime_telemetry_system(
    clock: Res<'_, ControlClock>,
    inputs: Res<'_, ControlInputs>,
    outputs: Res<'_, ControlOutputs>,
    motor_telemetry: Res<'_, MotorTelemetryResource>,
    mut telemetry: ResMut<'_, TelemetryState>,
) {
    telemetry.latest_frame = Some(runtime_telemetry_frame(
        &clock,
        &inputs,
        &outputs,
        &motor_telemetry,
    ));
}

#[cfg(feature = "std")]
pub fn publish_telemetry_system(
    publisher: Res<'_, TelemetryPublisher>,
    telemetry: Res<'_, TelemetryState>,
) {
    let Some(frame) = telemetry.latest_frame else {
        return;
    };

    publisher.sender.send(frame);
}
