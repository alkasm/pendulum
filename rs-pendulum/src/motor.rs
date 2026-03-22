use uom::si::f64::{AngularVelocity, ElectricCurrent, Torque};

#[derive(Debug, Clone, Copy, Default)]
pub struct MotorCommand {
    pub torque_command: Torque,
    pub observed_wheel_speed: AngularVelocity,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MotorTelemetry {
    pub applied_torque: Torque,
    pub available_torque: Torque,
    pub speed_ratio: f64,
    pub wheel_speed: AngularVelocity,
    pub phase_current: ElectricCurrent,
}

pub trait Motor {
    type Error;

    fn command(&mut self, command: MotorCommand) -> Result<MotorTelemetry, Self::Error>;
}
