use pendulum_lib::motor::{Motor, MotorCommand, MotorTelemetry};
use uom::si::{
    angular_velocity::radian_per_second,
    f64::{AngularVelocity, Torque},
    torque::newton_meter,
};

use super::ina240a1::Ina240A1;
use super::tmag5273::Tmag5273;
use super::tmc6300::Tmc6300;

#[derive(Debug)]
pub enum SparkfunIotMotorError {
    InvalidNoLoadSpeed,
}

pub struct SparkfunIotMotor {
    max_torque: Torque,
    no_load_speed: AngularVelocity,
    torque_constant_nm_per_a: f64,
    tmc6300: Tmc6300,
    tmag5273: Tmag5273,
    ina240a1: Ina240A1,
}

impl SparkfunIotMotor {
    pub fn new(
        max_torque: Torque,
        no_load_speed: AngularVelocity,
        torque_constant_nm_per_a: f64,
    ) -> Result<Self, SparkfunIotMotorError> {
        if no_load_speed <= AngularVelocity::new::<radian_per_second>(0.0) {
            return Err(SparkfunIotMotorError::InvalidNoLoadSpeed);
        }

        Ok(Self {
            max_torque,
            no_load_speed,
            torque_constant_nm_per_a,
            tmc6300: Tmc6300::new(),
            tmag5273: Tmag5273::new(),
            ina240a1: Ina240A1::new(),
        })
    }
}

impl Motor for SparkfunIotMotor {
    type Error = SparkfunIotMotorError;

    fn command(&mut self, command: MotorCommand) -> Result<MotorTelemetry, Self::Error> {
        let hall = self.tmag5273.read();
        let wheel_speed = if hall.wheel_speed != AngularVelocity::new::<radian_per_second>(0.0) {
            hall.wheel_speed
        } else {
            command.observed_wheel_speed
        };
        let speed_ratio = (wheel_speed.get::<radian_per_second>().abs()
            / self.no_load_speed.get::<radian_per_second>())
            .clamp(0.0, 1.0);
        let available_torque = self.max_torque * (1.0 - speed_ratio);
        let applied_torque = command
            .torque_command
            .clamp(-available_torque, available_torque);
        self.tmc6300.command_torque(applied_torque);
        let current = self.ina240a1.read();

        Ok(MotorTelemetry {
            applied_torque,
            available_torque,
            speed_ratio,
            wheel_speed,
            phase_current: current.phase_current,
        })
    }
}
