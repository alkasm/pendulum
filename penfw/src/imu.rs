use esp_hal::{Blocking, i2c::master::I2c};
use pendulum_lib::{
    PendulumEstimateTelemetry,
    estimation::{Acceleration3, AngularVelocity3, PendulumImuEstimator, RawImuSample},
    pendulum::PendulumGeometry,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angular_velocity::degree_per_second,
    f32::{Acceleration, AngularVelocity},
};

use crate::{
    bringup::i2c_device_present,
    hw::{
        GY521_DEFAULT_I2C_ADDR, Gy521Error, read_raw_measurement, verify_address, wake_device,
    },
};

pub fn read_pendulum_estimate(
    i2c: &mut I2c<'_, Blocking>,
    imu_verified: &mut bool,
    imu_awake: &mut bool,
    imu_estimator: &mut PendulumImuEstimator,
    geometry: &PendulumGeometry,
) -> PendulumEstimateTelemetry {
    if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
        *imu_verified = false;
        *imu_awake = false;
        imu_estimator.reset();
        return PendulumEstimateTelemetry::Missing;
    }

    if !*imu_verified {
        match verify_address(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_verified = true,
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

    if !*imu_awake {
        match wake_device(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_awake = true,
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

fn accel_from_g(value_g: f32) -> Acceleration {
    Acceleration::new::<meter_per_second_squared>(value_g * 9.80665)
}
