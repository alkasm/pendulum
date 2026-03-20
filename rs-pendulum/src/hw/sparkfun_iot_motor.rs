use crate::motor::{Motor, MotorCommand, MotorTelemetry};

use super::ina240a1::Ina240A1;
use super::tmag5273::Tmag5273;
use super::tmc6300::Tmc6300;

#[derive(Debug)]
pub enum SparkfunIotMotorError {
    InvalidNoLoadSpeed,
}

pub struct SparkfunIotMotor {
    max_torque_nm: f64,
    no_load_speed_rad_s: f64,
    tmc6300: Tmc6300,
    tmag5273: Tmag5273,
    ina240a1: Ina240A1,
}

impl SparkfunIotMotor {
    pub fn new(
        max_torque_nm: f64,
        no_load_speed_rad_s: f64,
    ) -> Result<Self, SparkfunIotMotorError> {
        if no_load_speed_rad_s <= 0.0 {
            return Err(SparkfunIotMotorError::InvalidNoLoadSpeed);
        }

        Ok(Self {
            max_torque_nm,
            no_load_speed_rad_s,
            tmc6300: Tmc6300::new(),
            tmag5273: Tmag5273::new(),
            ina240a1: Ina240A1::new(),
        })
    }

    pub fn set_mock_wheel_speed_rad_s(&mut self, wheel_speed_rad_s: f64) {
        self.tmag5273.set_mock_wheel_speed_rad_s(wheel_speed_rad_s);
    }

    pub fn set_mock_phase_current_a(&mut self, phase_current_a: f64) {
        self.ina240a1.set_mock_phase_current_a(phase_current_a);
    }
}

impl Motor for SparkfunIotMotor {
    type Error = SparkfunIotMotorError;

    fn command(&mut self, command: MotorCommand) -> Result<MotorTelemetry, Self::Error> {
        let hall = self.tmag5273.read();
        let wheel_speed_rad_s = if hall.wheel_speed_rad_s != 0.0 {
            hall.wheel_speed_rad_s
        } else {
            command.observed_wheel_speed_rad_s
        };
        let speed_ratio = (wheel_speed_rad_s.abs() / self.no_load_speed_rad_s).clamp(0.0, 1.0);
        let available_torque_nm = self.max_torque_nm * (1.0 - speed_ratio);
        let applied_torque_nm = command
            .torque_command_nm
            .clamp(-available_torque_nm, available_torque_nm);
        self.tmc6300.command_torque(applied_torque_nm);
        let _ = self.ina240a1.read();

        Ok(MotorTelemetry {
            applied_torque_nm,
            available_torque_nm,
            speed_ratio,
            wheel_speed_rad_s,
        })
    }
}
