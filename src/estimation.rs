use libm::{atan2f, sqrtf};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_velocity::degree_per_second,
    f32::{Acceleration, Angle, AngularVelocity},
};

use crate::{
    pendulum::{BodyAxis3, ImuAxesInBody, PendulumGeometry},
    protocol::PendulumEstimateMeasurement,
};

const COMPLEMENTARY_FILTER_ALPHA: f32 = 0.98;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Acceleration3 {
    pub x: Acceleration,
    pub y: Acceleration,
    pub z: Acceleration,
}

impl Acceleration3 {
    pub fn norm(self) -> Acceleration {
        let x = self.x.get::<meter_per_second_squared>();
        let y = self.y.get::<meter_per_second_squared>();
        let z = self.z.get::<meter_per_second_squared>();
        Acceleration::new::<meter_per_second_squared>(sqrtf(x * x + y * y + z * z))
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AngularVelocity3 {
    pub x: AngularVelocity,
    pub y: AngularVelocity,
    pub z: AngularVelocity,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RawImuSample {
    pub accel: Acceleration3,
    pub gyro: AngularVelocity3,
}

#[derive(Debug, Clone, Copy)]
pub struct PendulumImuEstimator {
    dt_s: f32,
    alpha: f32,
    filtered_theta: Option<Angle>,
}

impl PendulumImuEstimator {
    pub fn new(dt_s: f32) -> Self {
        Self::with_alpha(dt_s, COMPLEMENTARY_FILTER_ALPHA)
    }

    pub fn with_alpha(dt_s: f32, alpha: f32) -> Self {
        Self {
            dt_s,
            alpha,
            filtered_theta: None,
        }
    }

    pub fn reset(&mut self) {
        self.filtered_theta = None;
    }

    pub fn step(
        &mut self,
        geometry: &PendulumGeometry,
        sample: RawImuSample,
    ) -> PendulumEstimateMeasurement {
        let accel_body = transform_acceleration_to_body(sample.accel, geometry.imu_mount.axes_in_body);
        let gyro_body = transform_angular_velocity_to_body(sample.gyro, geometry.imu_mount.axes_in_body);

        let theta_dot = -gyro_body.z;
        let accel_theta = Angle::new::<degree>(
            atan2f(
                -accel_body.x.get::<meter_per_second_squared>(),
                accel_body.y.get::<meter_per_second_squared>(),
            ) * (180.0 / core::f32::consts::PI),
        );
        let theta = self.observe_theta(theta_dot, accel_theta);

        PendulumEstimateMeasurement {
            theta_deg: theta.get::<degree>(),
            theta_dot_dps: theta_dot.get::<degree_per_second>(),
        }
    }

    fn observe_theta(
        &mut self,
        theta_dot: AngularVelocity,
        accel_theta: Angle,
    ) -> Angle {
        let predicted_theta = self
            .filtered_theta
            .map(|theta| {
                Angle::new::<degree>(
                    theta.get::<degree>() + theta_dot.get::<degree_per_second>() * self.dt_s,
                )
            })
            .unwrap_or(accel_theta);
        let accel_error_deg =
            wrap_angle_delta_deg(accel_theta.get::<degree>() - predicted_theta.get::<degree>());
        let theta = Angle::new::<degree>(wrap_signed_degrees(
            predicted_theta.get::<degree>() + (1.0 - self.alpha) * accel_error_deg,
        ));
        self.filtered_theta = Some(theta);
        theta
    }
}

fn transform_acceleration_to_body(vector: Acceleration3, axes_in_body: ImuAxesInBody) -> Acceleration3 {
    let mut body = Acceleration3::default();
    accumulate_axis_contribution(&mut body, vector.x, axes_in_body.x_axis);
    accumulate_axis_contribution(&mut body, vector.y, axes_in_body.y_axis);
    accumulate_axis_contribution(&mut body, vector.z, axes_in_body.z_axis);
    body
}

fn transform_angular_velocity_to_body(
    vector: AngularVelocity3,
    axes_in_body: ImuAxesInBody,
) -> AngularVelocity3 {
    let mut body = AngularVelocity3::default();
    accumulate_angular_axis_contribution(&mut body, vector.x, axes_in_body.x_axis);
    accumulate_angular_axis_contribution(&mut body, vector.y, axes_in_body.y_axis);
    accumulate_angular_axis_contribution(&mut body, vector.z, axes_in_body.z_axis);
    body
}

fn accumulate_axis_contribution(body: &mut Acceleration3, value: Acceleration, axis: BodyAxis3) {
    match axis {
        BodyAxis3::Right => body.x += value,
        BodyAxis3::Left => body.x -= value,
        BodyAxis3::Up => body.y += value,
        BodyAxis3::Down => body.y -= value,
        BodyAxis3::TowardViewer => body.z += value,
        BodyAxis3::AwayFromViewer => body.z -= value,
    }
}

fn accumulate_angular_axis_contribution(
    body: &mut AngularVelocity3,
    value: AngularVelocity,
    axis: BodyAxis3,
) {
    match axis {
        BodyAxis3::Right => body.x += value,
        BodyAxis3::Left => body.x -= value,
        BodyAxis3::Up => body.y += value,
        BodyAxis3::Down => body.y -= value,
        BodyAxis3::TowardViewer => body.z += value,
        BodyAxis3::AwayFromViewer => body.z -= value,
    }
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
                accel: Acceleration3 {
                    x: Acceleration::new::<meter_per_second_squared>(0.0),
                    y: Acceleration::new::<meter_per_second_squared>(-9.80665),
                    z: Acceleration::new::<meter_per_second_squared>(0.0),
                },
                gyro: AngularVelocity3::default(),
            },
        );

        assert!(estimate.theta_deg.is_finite());
        assert!(estimate.theta_dot_dps.is_finite());
    }
}
