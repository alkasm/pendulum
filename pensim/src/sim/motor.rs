use pendulum_lib::motor::{Motor, MotorCommand, MotorTelemetry};
use uom::si::{
    angular_velocity::radian_per_second,
    electric_current::ampere,
    f64::{AngularVelocity, ElectricCurrent, Torque},
    torque::newton_meter,
};

use super::current_sensor::SimCurrentSensor;
use super::hall_sensor::SimHallSensor;
use super::motor_driver::SimMotorDriver;

#[derive(Debug, Clone, Copy)]
pub struct SimMotor {
    max_torque: Torque,
    no_load_speed: AngularVelocity,
    torque_constant_nm_per_a: f64,
    motor_driver: SimMotorDriver,
    hall_sensor: SimHallSensor,
    current_sensor: SimCurrentSensor,
}

impl SimMotor {
    pub fn new(
        max_torque: Torque,
        no_load_speed: AngularVelocity,
        torque_constant_nm_per_a: f64,
    ) -> Self {
        Self {
            max_torque,
            no_load_speed,
            torque_constant_nm_per_a,
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
            .sample_wheel_speed(command.observed_wheel_speed);
        let hall = self.hall_sensor.read();
        let speed_ratio = (hall.wheel_speed.get::<radian_per_second>().abs()
            / self.no_load_speed.get::<radian_per_second>())
        .clamp(0.0, 1.0);
        let available_torque = self.max_torque * (1.0 - speed_ratio);
        let applied_torque = if command.torque_command > available_torque {
            available_torque
        } else if command.torque_command < -available_torque {
            -available_torque
        } else {
            command.torque_command
        };
        self.motor_driver.command_torque(applied_torque);
        let phase_current = if self.torque_constant_nm_per_a > 0.0 {
            ElectricCurrent::new::<ampere>(
                applied_torque.get::<newton_meter>() / self.torque_constant_nm_per_a,
            )
        } else {
            ElectricCurrent::new::<ampere>(0.0)
        };
        self.current_sensor.sample_phase_current(phase_current);
        let current = self.current_sensor.read();

        Ok(MotorTelemetry {
            applied_torque,
            available_torque,
            speed_ratio,
            wheel_speed: hall.wheel_speed,
            phase_current: current.phase_current,
        })
    }
}
