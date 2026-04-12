use esp_hal::{Blocking, i2c::master::I2c};
use libm::{atan2f, sqrtf};
use pendulum_lib::{
    PendulumEstimateMeasurement, PendulumEstimateTelemetry,
    pendulum::{BodyAxis3, ImuAxesInBody, PendulumGeometry},
};
use uom::si::length::millimeter;

use crate::{
    bringup::i2c_device_present,
    hw::GY521_DEFAULT_I2C_ADDR,
    math::{clamp, degrees_to_radians, wrap_angle_delta_deg, wrap_signed_degrees},
};

const MPU_REG_ACCEL_XOUT_H: u8 = 0x3B;
const MPU_REG_CONFIG: u8 = 0x1A;
const MPU_REG_GYRO_CONFIG: u8 = 0x1B;
const MPU_REG_ACCEL_CONFIG: u8 = 0x1C;
const MPU_REG_PWR_MGMT_1: u8 = 0x6B;
const MPU_REG_WHO_AM_I: u8 = 0x75;
const MPU6050_WHO_AM_I_VALUE: u8 = 0x68;
const ACCEL_LSB_PER_G: f32 = 8_192.0;
const GYRO_LSB_PER_DPS: f32 = 32.8;
const ACCEL_CORRECTION_GAIN: f32 = 0.08;
const MAX_ACCEL_CORRECTION_STEP_DEG: f32 = 2.0;
const MAX_ACCEL_CORRECTION_ERROR_DEG: f32 = 35.0;
const MIN_ACCEL_GRAVITY_G: f32 = 0.75;
const MAX_ACCEL_GRAVITY_G: f32 = 1.25;

#[derive(Clone, Copy)]
pub enum ImuProbeError {
    RegisterRead,
    UnexpectedWhoAmI(u8),
}

#[derive(Clone, Copy)]
struct Point2Mm {
    x: f32,
    y: f32,
}

#[derive(Clone, Copy, Default)]
struct BodyVector3 {
    x: f32,
    y: f32,
    z: f32,
}

impl BodyVector3 {
    fn norm(self) -> f32 {
        sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

pub struct ImuEstimator {
    dt_s: f32,
    last_theta_dot_dps: Option<f32>,
    filtered_theta_ddot_dps2: f32,
    filtered_theta_deg: Option<f32>,
}

impl ImuEstimator {
    pub fn new(dt_s: f32) -> Self {
        Self {
            dt_s,
            last_theta_dot_dps: None,
            filtered_theta_ddot_dps2: 0.0,
            filtered_theta_deg: None,
        }
    }

    pub fn reset(&mut self) {
        self.last_theta_dot_dps = None;
        self.filtered_theta_ddot_dps2 = 0.0;
        self.filtered_theta_deg = None;
    }

    pub fn read_pendulum_estimate(
        &mut self,
        i2c: &mut I2c<'_, Blocking>,
        imu_verified: &mut bool,
        imu_awake: &mut bool,
        geometry: &PendulumGeometry,
    ) -> PendulumEstimateTelemetry {
        if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
            *imu_verified = false;
            *imu_awake = false;
            self.reset();
            return PendulumEstimateTelemetry::Missing;
        }

        if !*imu_verified {
            match imu_verify(i2c, GY521_DEFAULT_I2C_ADDR) {
                Ok(()) => *imu_verified = true,
                Err(ImuProbeError::RegisterRead) => {
                    self.reset();
                    return PendulumEstimateTelemetry::Missing;
                }
                Err(ImuProbeError::UnexpectedWhoAmI(value)) => {
                    self.reset();
                    return PendulumEstimateTelemetry::UnexpectedWhoAmI { value };
                }
            }
        }

        if !*imu_awake {
            match imu_wake(i2c, GY521_DEFAULT_I2C_ADDR) {
                Ok(()) => *imu_awake = true,
                Err(register) => {
                    self.reset();
                    return PendulumEstimateTelemetry::WakeError { register };
                }
            }
        }

        match self.read_measurement(i2c, GY521_DEFAULT_I2C_ADDR, geometry) {
            Ok(measurement) => PendulumEstimateTelemetry::Measurement(measurement),
            Err(register) => {
                self.reset();
                PendulumEstimateTelemetry::ReadError { register }
            }
        }
    }

    fn observe_theta_dot(&mut self, theta_dot_dps: f32) -> f32 {
        let instant_theta_ddot_dps2 = self
            .last_theta_dot_dps
            .map(|previous| (theta_dot_dps - previous) / self.dt_s)
            .unwrap_or(0.0);
        self.last_theta_dot_dps = Some(theta_dot_dps);
        self.filtered_theta_ddot_dps2 =
            0.85 * self.filtered_theta_ddot_dps2 + 0.15 * instant_theta_ddot_dps2;
        self.filtered_theta_ddot_dps2
    }

    fn observe_theta_from_accel(
        &mut self,
        theta_dot_dps: f32,
        accel_theta_deg: f32,
        accel_gravity_magnitude_g: f32,
    ) -> f32 {
        let predicted_theta_deg = self
            .filtered_theta_deg
            .map(|theta_deg| theta_deg + theta_dot_dps * self.dt_s)
            .unwrap_or(accel_theta_deg);
        let accel_error_deg = wrap_angle_delta_deg(accel_theta_deg - predicted_theta_deg);
        let accel_is_reliable = accel_gravity_magnitude_g >= MIN_ACCEL_GRAVITY_G
            && accel_gravity_magnitude_g <= MAX_ACCEL_GRAVITY_G
            && accel_error_deg.abs() <= MAX_ACCEL_CORRECTION_ERROR_DEG;
        let correction_deg = if accel_is_reliable {
            clamp(
                accel_error_deg * ACCEL_CORRECTION_GAIN,
                -MAX_ACCEL_CORRECTION_STEP_DEG,
                MAX_ACCEL_CORRECTION_STEP_DEG,
            )
        } else {
            0.0
        };
        let theta_deg = wrap_signed_degrees(predicted_theta_deg + correction_deg);
        self.filtered_theta_deg = Some(theta_deg);
        theta_deg
    }

    fn read_measurement(
        &mut self,
        i2c: &mut I2c<'_, Blocking>,
        address: u8,
        geometry: &PendulumGeometry,
    ) -> Result<PendulumEstimateMeasurement, u8> {
        let mut buffer = [0_u8; 14];
        i2c.write_read(address, &[MPU_REG_ACCEL_XOUT_H], &mut buffer)
            .map_err(|_| MPU_REG_ACCEL_XOUT_H)?;

        let ax_raw = i16::from_be_bytes([buffer[0], buffer[1]]);
        let ay_raw = i16::from_be_bytes([buffer[2], buffer[3]]);
        let az_raw = i16::from_be_bytes([buffer[4], buffer[5]]);
        let gx_raw = i16::from_be_bytes([buffer[8], buffer[9]]);
        let gy_raw = i16::from_be_bytes([buffer[10], buffer[11]]);
        let gz_raw = i16::from_be_bytes([buffer[12], buffer[13]]);

        let accel_imu_g = BodyVector3 {
            x: ax_raw as f32 / ACCEL_LSB_PER_G,
            y: ay_raw as f32 / ACCEL_LSB_PER_G,
            z: az_raw as f32 / ACCEL_LSB_PER_G,
        };
        let gyro_imu_dps = BodyVector3 {
            x: gx_raw as f32 / GYRO_LSB_PER_DPS,
            y: gy_raw as f32 / GYRO_LSB_PER_DPS,
            z: gz_raw as f32 / GYRO_LSB_PER_DPS,
        };
        let accel_body_g = transform_imu_vector_to_body(accel_imu_g, geometry.imu_mount.axes_in_body);
        let gyro_body_dps = transform_imu_vector_to_body(gyro_imu_dps, geometry.imu_mount.axes_in_body);

        let theta_dot_dps = -gyro_body_dps.z;
        let theta_ddot_dps2 = self.observe_theta_dot(theta_dot_dps);
        let pivot_to_imu_body_mm = pivot_to_imu_body_mm(geometry);
        let rotational_specific_force_g = rotational_specific_force_g(
            theta_dot_dps,
            theta_ddot_dps2,
            pivot_to_imu_body_mm,
        );
        let gravity_only_accel_body_g = BodyVector3 {
            x: accel_body_g.x - rotational_specific_force_g.x,
            y: accel_body_g.y - rotational_specific_force_g.y,
            z: accel_body_g.z - rotational_specific_force_g.z,
        };
        let accel_theta_deg = atan2f(-gravity_only_accel_body_g.x, gravity_only_accel_body_g.y)
            * (180.0 / core::f32::consts::PI);
        let accel_gravity_magnitude_g = gravity_only_accel_body_g.norm();
        let theta_deg =
            self.observe_theta_from_accel(theta_dot_dps, accel_theta_deg, accel_gravity_magnitude_g);

        Ok(PendulumEstimateMeasurement {
            theta_deg,
            theta_dot_dps,
        })
    }
}

fn imu_verify(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), ImuProbeError> {
    let mut who_am_i = [0_u8; 1];
    i2c.write_read(address, &[MPU_REG_WHO_AM_I], &mut who_am_i)
        .map_err(|_| ImuProbeError::RegisterRead)?;
    if who_am_i[0] != MPU6050_WHO_AM_I_VALUE {
        return Err(ImuProbeError::UnexpectedWhoAmI(who_am_i[0]));
    }
    Ok(())
}

fn imu_wake(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), u8> {
    i2c.write(address, &[MPU_REG_PWR_MGMT_1, 0x01])
        .map_err(|_| MPU_REG_PWR_MGMT_1)?;
    i2c.write(address, &[MPU_REG_CONFIG, 0x03])
        .map_err(|_| MPU_REG_CONFIG)?;
    i2c.write(address, &[MPU_REG_GYRO_CONFIG, 0x10])
        .map_err(|_| MPU_REG_GYRO_CONFIG)?;
    i2c.write(address, &[MPU_REG_ACCEL_CONFIG, 0x08])
        .map_err(|_| MPU_REG_ACCEL_CONFIG)?;
    Ok(())
}

fn pivot_to_imu_body_mm(geometry: &PendulumGeometry) -> Point2Mm {
    Point2Mm {
        x: (geometry.motor_mount.center_from_pivot.x + geometry.imu_mount.translation_from_motor.x)
            .get::<millimeter>() as f32,
        y: (geometry.motor_mount.center_from_pivot.y + geometry.imu_mount.translation_from_motor.y)
            .get::<millimeter>() as f32,
    }
}

fn rotational_specific_force_g(
    theta_dot_dps: f32,
    theta_ddot_dps2: f32,
    pivot_to_imu_body_mm: Point2Mm,
) -> BodyVector3 {
    const G_M_PER_S2: f32 = 9.80665;

    let omega_rad_s = degrees_to_radians(theta_dot_dps);
    let alpha_rad_s2 = degrees_to_radians(theta_ddot_dps2);
    let rx_m = pivot_to_imu_body_mm.x / 1_000.0;
    let ry_m = pivot_to_imu_body_mm.y / 1_000.0;

    BodyVector3 {
        x: ((-alpha_rad_s2 * ry_m) - (omega_rad_s * omega_rad_s * rx_m)) / G_M_PER_S2,
        y: ((alpha_rad_s2 * rx_m) - (omega_rad_s * omega_rad_s * ry_m)) / G_M_PER_S2,
        z: 0.0,
    }
}

fn transform_imu_vector_to_body(vector: BodyVector3, axes_in_body: ImuAxesInBody) -> BodyVector3 {
    let mut body = BodyVector3::default();
    accumulate_axis_contribution(&mut body, vector.x, axes_in_body.x_axis);
    accumulate_axis_contribution(&mut body, vector.y, axes_in_body.y_axis);
    accumulate_axis_contribution(&mut body, vector.z, axes_in_body.z_axis);
    body
}

fn accumulate_axis_contribution(body: &mut BodyVector3, value: f32, axis: BodyAxis3) {
    match axis {
        BodyAxis3::Right => body.x += value,
        BodyAxis3::Left => body.x -= value,
        BodyAxis3::Up => body.y += value,
        BodyAxis3::Down => body.y -= value,
        BodyAxis3::TowardViewer => body.z += value,
        BodyAxis3::AwayFromViewer => body.z -= value,
    }
}
