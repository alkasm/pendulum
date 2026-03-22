use linux_embedded_hal::{Delay, I2cdev};
use mpu6050::Mpu6050;

use pendulum_lib::imu::{Imu, ImuSample};
use uom::si::{angle::radian, angular_velocity::radian_per_second, f64::{Angle, AngularVelocity}};

#[derive(Debug)]
pub enum Mpu6050ImuError {
    I2cOpen(std::io::Error),
    Driver(String),
}

pub struct Mpu6050Imu {
    driver: Mpu6050<I2cdev>,
}

impl Mpu6050Imu {
    pub fn new(i2c_bus: &str) -> Result<Self, Mpu6050ImuError> {
        let i2c = I2cdev::new(i2c_bus).map_err(Mpu6050ImuError::I2cOpen)?;
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

        let theta = Angle::new::<radian>((acc.x as f64).atan2(acc.z as f64));
        let theta_dot = AngularVelocity::new::<radian_per_second>(gyro.y as f64);

        Ok(ImuSample { theta, theta_dot })
    }
}
