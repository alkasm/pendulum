use crate::motor::{Motor, MotorCommand, MotorTelemetry};

use super::current_sensor::SimCurrentSensor;
use super::hall_sensor::SimHallSensor;
use super::motor_driver::SimMotorDriver;

#[derive(Debug, Clone, Copy)]
pub struct SimMotor {
    max_torque_nm: f64,
    no_load_speed_rad_s: f64,
    motor_driver: SimMotorDriver,
    hall_sensor: SimHallSensor,
    current_sensor: SimCurrentSensor,
}

impl SimMotor {
    pub fn new(max_torque_nm: f64, no_load_speed_rad_s: f64) -> Self {
        Self {
            max_torque_nm,
            no_load_speed_rad_s,
            motor_driver: SimMotorDriver::new(),
            hall_sensor: SimHallSensor::new(),
            current_sensor: SimCurrentSensor::new(),
        }
    }
}

impl Motor for SimMotor {
    type Error = core::convert::Infallible;

    fn command(&mut self, command: MotorCommand) -> Result<MotorTelemetry, Self::Error> {
        self.hall_sensor
            .sample_wheel_speed_rad_s(command.observed_wheel_speed_rad_s);
        let hall = self.hall_sensor.read();
        let speed_ratio = (hall.wheel_speed_rad_s.abs() / self.no_load_speed_rad_s).clamp(0.0, 1.0);
        let available_torque_nm = self.max_torque_nm * (1.0 - speed_ratio);
        let applied_torque_nm = command
            .torque_command_nm
            .clamp(-available_torque_nm, available_torque_nm);
        self.motor_driver.command_torque(applied_torque_nm);
        self.current_sensor.sample_phase_current_a(0.0);
        let _ = self.current_sensor.read();

        Ok(MotorTelemetry {
            applied_torque_nm,
            available_torque_nm,
            speed_ratio,
            wheel_speed_rad_s: hall.wheel_speed_rad_s,
        })
    }
}
