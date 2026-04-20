use esp_hal::{i2c::master::I2c, Blocking};
use pendulum_lib::{
    estimation::{Acceleration3, AngularVelocity3, PendulumImuEstimator, RawImuSample},
    pendulum::PendulumGeometry,
    PendulumEstimateTelemetry,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angular_velocity::degree_per_second,
    f32::{Acceleration, AngularVelocity},
};

use crate::{
    board_init::i2c_device_present,
    hw::{read_raw_measurement, verify_address, wake_device, Gy521Error, GY521_DEFAULT_I2C_ADDR},
};

pub struct Gy521Session {
    verified: bool,
    awake: bool,
}

impl Gy521Session {
    pub fn new() -> Self {
        Self {
            verified: false,
            awake: false,
        }
    }

    pub fn reset(&mut self) {
        self.verified = false;
        self.awake = false;
    }

    pub fn read_estimate(
        &mut self,
        i2c: &mut I2c<'_, Blocking>,
        imu_estimator: &mut PendulumImuEstimator,
        geometry: &PendulumGeometry,
    ) -> PendulumEstimateTelemetry {
        if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
            self.reset();
            imu_estimator.reset();
            return PendulumEstimateTelemetry::Missing;
        }

        if !self.verified {
            match verify_address(i2c, GY521_DEFAULT_I2C_ADDR) {
                Ok(()) => self.verified = true,
                Err(Gy521Error::RegisterRead(_)) => {
                    imu_estimator.reset();
                    return PendulumEstimateTelemetry::Missing;
                }
                Err(Gy521Error::UnexpectedWhoAmI(value)) => {
                    imu_estimator.reset();
                    return PendulumEstimateTelemetry::UnexpectedWhoAmI { value };
                }
            }
        }

        if !self.awake {
            match wake_device(i2c, GY521_DEFAULT_I2C_ADDR) {
                Ok(()) => self.awake = true,
                Err(register) => {
                    imu_estimator.reset();
                    return PendulumEstimateTelemetry::WakeError { register };
                }
            }
        }

        match read_raw_measurement(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(measurement) => PendulumEstimateTelemetry::Measurement(imu_estimator.step(
                geometry,
                RawImuSample {
                    accel: Acceleration3 {
                        x: accel_from_g(measurement.ax_g),
                        y: accel_from_g(measurement.ay_g),
                        z: accel_from_g(measurement.az_g),
                    },
                    gyro: AngularVelocity3 {
                        x: AngularVelocity::new::<degree_per_second>(measurement.gx_dps),
                        y: AngularVelocity::new::<degree_per_second>(measurement.gy_dps),
                        z: AngularVelocity::new::<degree_per_second>(measurement.gz_dps),
                    },
                },
            )),
            Err(register) => {
                imu_estimator.reset();
                PendulumEstimateTelemetry::ReadError { register }
            }
        }
    }
}

pub fn read_pendulum_estimate(
    i2c: &mut I2c<'_, Blocking>,
    imu: &mut Gy521Session,
    imu_estimator: &mut PendulumImuEstimator,
    geometry: &PendulumGeometry,
) -> PendulumEstimateTelemetry {
    imu.read_estimate(i2c, imu_estimator, geometry)
}

fn accel_from_g(value_g: f32) -> Acceleration {
    Acceleration::new::<meter_per_second_squared>(value_g * 9.80665)
}
