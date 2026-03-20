#[derive(Debug, Clone, Copy)]
pub struct MotorCommand {
    pub torque_command_nm: f64,
    pub observed_wheel_speed_rad_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorTelemetry {
    pub applied_torque_nm: f64,
    pub available_torque_nm: f64,
    pub speed_ratio: f64,
    pub wheel_speed_rad_s: f64,
}

pub trait Motor {
    type Error;

    fn command(&mut self, command: MotorCommand) -> Result<MotorTelemetry, Self::Error>;
}
