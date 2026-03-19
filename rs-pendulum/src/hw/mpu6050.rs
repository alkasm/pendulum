use linux_embedded_hal::{Delay, I2cdev};
use mpu6050::Mpu6050;

use crate::imu::{Imu, ImuSample};

// Default I²C bus on Raspberry Pi.
const I2C_BUS: &str = "/dev/i2c-1";

#[derive(Debug)]
pub enum Mpu6050ImuError {
    I2cOpen(std::io::Error),
    Driver(String),
}

pub struct Mpu6050Imu {
    driver: Mpu6050<I2cdev>,
}

impl Mpu6050Imu {
    pub fn new() -> Result<Self, Mpu6050ImuError> {
        let i2c = I2cdev::new(I2C_BUS).map_err(Mpu6050ImuError::I2cOpen)?;
        let mut driver = Mpu6050::new(i2c);
        driver
            .init(&mut Delay)
            .map_err(|e| Mpu6050ImuError::Driver(format!("{e:?}")))?;
        Ok(Self { driver })
    }
}

impl Imu for Mpu6050Imu {
    type Error = Mpu6050ImuError;

    fn read(&mut self) -> Result<ImuSample, Self::Error> {
        let acc = self
            .driver
            .get_acc()
            .map_err(|e| Mpu6050ImuError::Driver(format!("{e:?}")))?;
        let gyro = self
            .driver
            .get_gyro()
            .map_err(|e| Mpu6050ImuError::Driver(format!("{e:?}")))?;

        // theta: pitch angle from accelerometer (rad).
        // Assumes the chip is mounted with its Z-axis along the pendulum arm
        // (pointing up when balanced). At balance acc ≈ [0, 0, 1g]; a tilt of
        // theta in the XZ plane gives acc.x = sin(θ), acc.z = cos(θ).
        // TODO: allow for arbitrary translation + rotation transform for the IMU.
        let theta = (acc.x as f64).atan2(acc.z as f64);

        // theta_dot: pitch rate from gyroscope Y-axis (rad/s).
        let theta_dot = gyro.y as f64;

        Ok(ImuSample { theta, theta_dot })
    }
}
