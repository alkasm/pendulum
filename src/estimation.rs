use libm::{atan2f, sqrtf};
use uom::si::length::millimeter;

use crate::{
    pendulum::{BodyAxis3, ImuAxesInBody, PendulumGeometry},
    protocol::PendulumEstimateMeasurement,
};

const ACCEL_CORRECTION_GAIN: f32 = 0.08;
const MAX_ACCEL_CORRECTION_STEP_DEG: f32 = 2.0;
const MAX_ACCEL_CORRECTION_ERROR_DEG: f32 = 35.0;
const MIN_ACCEL_GRAVITY_G: f32 = 0.75;
const MAX_ACCEL_GRAVITY_G: f32 = 1.25;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vector3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3f {
    pub fn norm(self) -> f32 {
        sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RawImuSample {
    pub accel_g: Vector3f,
    pub gyro_dps: Vector3f,
}

#[derive(Debug, Clone, Copy, Default)]
struct Point2Mm {
    x: f32,
    y: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct PendulumImuEstimator {
    dt_s: f32,
    last_theta_dot_dps: Option<f32>,
    filtered_theta_ddot_dps2: f32,
    filtered_theta_deg: Option<f32>,
}

impl PendulumImuEstimator {
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

    pub fn step(
        &mut self,
        geometry: &PendulumGeometry,
        sample: RawImuSample,
    ) -> PendulumEstimateMeasurement {
        let accel_body_g = transform_imu_vector_to_body(sample.accel_g, geometry.imu_mount.axes_in_body);
        let gyro_body_dps = transform_imu_vector_to_body(sample.gyro_dps, geometry.imu_mount.axes_in_body);

        let theta_dot_dps = -gyro_body_dps.z;
        let theta_ddot_dps2 = self.observe_theta_dot(theta_dot_dps);
        let pivot_to_imu_body_mm = pivot_to_imu_body_mm(geometry);
        let rotational_specific_force_g = rotational_specific_force_g(
            theta_dot_dps,
            theta_ddot_dps2,
            pivot_to_imu_body_mm,
        );
        let gravity_only_accel_body_g = Vector3f {
            x: accel_body_g.x - rotational_specific_force_g.x,
            y: accel_body_g.y - rotational_specific_force_g.y,
            z: accel_body_g.z - rotational_specific_force_g.z,
        };
        let accel_theta_deg = atan2f(-gravity_only_accel_body_g.x, gravity_only_accel_body_g.y)
            * (180.0 / core::f32::consts::PI);
        let accel_gravity_magnitude_g = gravity_only_accel_body_g.norm();
        let theta_deg =
            self.observe_theta_from_accel(theta_dot_dps, accel_theta_deg, accel_gravity_magnitude_g);

        PendulumEstimateMeasurement {
            theta_deg,
            theta_dot_dps,
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
) -> Vector3f {
    const G_M_PER_S2: f32 = 9.80665;

    let omega_rad_s = degrees_to_radians(theta_dot_dps);
    let alpha_rad_s2 = degrees_to_radians(theta_ddot_dps2);
    let rx_m = pivot_to_imu_body_mm.x / 1_000.0;
    let ry_m = pivot_to_imu_body_mm.y / 1_000.0;

    Vector3f {
        x: ((-alpha_rad_s2 * ry_m) - (omega_rad_s * omega_rad_s * rx_m)) / G_M_PER_S2,
        y: ((alpha_rad_s2 * rx_m) - (omega_rad_s * omega_rad_s * ry_m)) / G_M_PER_S2,
        z: 0.0,
    }
}

fn transform_imu_vector_to_body(vector: Vector3f, axes_in_body: ImuAxesInBody) -> Vector3f {
    let mut body = Vector3f::default();
    accumulate_axis_contribution(&mut body, vector.x, axes_in_body.x_axis);
    accumulate_axis_contribution(&mut body, vector.y, axes_in_body.y_axis);
    accumulate_axis_contribution(&mut body, vector.z, axes_in_body.z_axis);
    body
}

fn accumulate_axis_contribution(body: &mut Vector3f, value: f32, axis: BodyAxis3) {
    match axis {
        BodyAxis3::Right => body.x += value,
        BodyAxis3::Left => body.x -= value,
        BodyAxis3::Up => body.y += value,
        BodyAxis3::Down => body.y -= value,
        BodyAxis3::TowardViewer => body.z += value,
        BodyAxis3::AwayFromViewer => body.z -= value,
    }
}

fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

fn degrees_to_radians(angle_deg: f32) -> f32 {
    angle_deg * (core::f32::consts::PI / 180.0)
}

fn wrap_signed_degrees(angle_deg: f32) -> f32 {
    let mut wrapped = angle_deg;
    while wrapped > 180.0 {
        wrapped -= 360.0;
    }
    while wrapped <= -180.0 {
        wrapped += 360.0;
    }
    wrapped
}

fn wrap_angle_delta_deg(delta: f32) -> f32 {
    if delta > 180.0 {
        delta - 360.0
    } else if delta < -180.0 {
        delta + 360.0
    } else {
        delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::default_pendulum;

    #[test]
    fn estimator_produces_finite_output_for_resting_sample() {
        let mut estimator = PendulumImuEstimator::new(0.005);
        let geometry = default_pendulum().geometry;
        let estimate = estimator.step(
            &geometry,
            RawImuSample {
                accel_g: Vector3f {
                    x: 0.0,
                    y: -1.0,
                    z: 0.0,
                },
                gyro_dps: Vector3f::default(),
            },
        );

        assert!(estimate.theta_deg.is_finite());
        assert!(estimate.theta_dot_dps.is_finite());
    }
}
