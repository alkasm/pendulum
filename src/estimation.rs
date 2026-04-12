use libm::{atan2f, sqrtf};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::radian_per_second_squared,
    angular_velocity::{degree_per_second, radian_per_second},
    f32::{Acceleration, Angle, AngularAcceleration, AngularVelocity, Length},
    length::{meter, millimeter},
};

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

#[derive(Debug, Clone, Copy, Default)]
struct Point2Length {
    x: Length,
    y: Length,
}

#[derive(Debug, Clone, Copy)]
pub struct PendulumImuEstimator {
    dt_s: f32,
    last_theta_dot: Option<AngularVelocity>,
    filtered_theta_ddot: AngularAcceleration,
    filtered_theta: Option<Angle>,
}

impl PendulumImuEstimator {
    pub fn new(dt_s: f32) -> Self {
        Self {
            dt_s,
            last_theta_dot: None,
            filtered_theta_ddot: AngularAcceleration::new::<radian_per_second_squared>(0.0),
            filtered_theta: None,
        }
    }

    pub fn reset(&mut self) {
        self.last_theta_dot = None;
        self.filtered_theta_ddot = AngularAcceleration::new::<radian_per_second_squared>(0.0);
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
        let theta_ddot = self.observe_theta_dot(theta_dot);
        let pivot_to_imu_body = pivot_to_imu_body(geometry);
        let rotational_specific_force = rotational_specific_force(
            theta_dot,
            theta_ddot,
            pivot_to_imu_body,
        );
        let gravity_only_accel = Acceleration3 {
            x: accel_body.x - rotational_specific_force.x,
            y: accel_body.y - rotational_specific_force.y,
            z: accel_body.z - rotational_specific_force.z,
        };
        let accel_theta = Angle::new::<degree>(
            atan2f(
                -gravity_only_accel.x.get::<meter_per_second_squared>(),
                gravity_only_accel.y.get::<meter_per_second_squared>(),
            ) * (180.0 / core::f32::consts::PI),
        );
        let accel_gravity_magnitude = gravity_only_accel.norm();
        let theta = self.observe_theta_from_accel(theta_dot, accel_theta, accel_gravity_magnitude);

        PendulumEstimateMeasurement {
            theta_deg: theta.get::<degree>(),
            theta_dot_dps: theta_dot.get::<degree_per_second>(),
        }
    }

    fn observe_theta_dot(&mut self, theta_dot: AngularVelocity) -> AngularAcceleration {
        let instant_theta_ddot = self
            .last_theta_dot
            .map(|previous| {
                AngularAcceleration::new::<radian_per_second_squared>(
                    (theta_dot.get::<radian_per_second>() - previous.get::<radian_per_second>())
                        / self.dt_s,
                )
            })
            .unwrap_or_else(|| AngularAcceleration::new::<radian_per_second_squared>(0.0));
        self.last_theta_dot = Some(theta_dot);
        self.filtered_theta_ddot = AngularAcceleration::new::<radian_per_second_squared>(
            0.85 * self.filtered_theta_ddot.get::<radian_per_second_squared>()
                + 0.15 * instant_theta_ddot.get::<radian_per_second_squared>(),
        );
        self.filtered_theta_ddot
    }

    fn observe_theta_from_accel(
        &mut self,
        theta_dot: AngularVelocity,
        accel_theta: Angle,
        accel_gravity_magnitude: Acceleration,
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
        let accel_gravity_magnitude_mps2 = accel_gravity_magnitude.get::<meter_per_second_squared>();
        let min_gravity = MIN_ACCEL_GRAVITY_G * standard_gravity_mps2();
        let max_gravity = MAX_ACCEL_GRAVITY_G * standard_gravity_mps2();
        let accel_is_reliable = accel_gravity_magnitude_mps2 >= min_gravity
            && accel_gravity_magnitude_mps2 <= max_gravity
            && accel_error_deg.abs() <= MAX_ACCEL_CORRECTION_ERROR_DEG;
        let correction = if accel_is_reliable {
            Angle::new::<degree>(clamp(
                accel_error_deg * ACCEL_CORRECTION_GAIN,
                -MAX_ACCEL_CORRECTION_STEP_DEG,
                MAX_ACCEL_CORRECTION_STEP_DEG,
            ))
        } else {
            Angle::new::<degree>(0.0)
        };
        let theta = Angle::new::<degree>(wrap_signed_degrees(
            predicted_theta.get::<degree>() + correction.get::<degree>(),
        ));
        self.filtered_theta = Some(theta);
        theta
    }
}

fn pivot_to_imu_body(geometry: &PendulumGeometry) -> Point2Length {
    Point2Length {
        x: Length::new::<millimeter>(
            (geometry.motor_mount.center_from_pivot.x + geometry.imu_mount.translation_from_motor.x)
                .get::<millimeter>() as f32,
        ),
        y: Length::new::<millimeter>(
            (geometry.motor_mount.center_from_pivot.y + geometry.imu_mount.translation_from_motor.y)
                .get::<millimeter>() as f32,
        ),
    }
}

fn rotational_specific_force(
    theta_dot: AngularVelocity,
    theta_ddot: AngularAcceleration,
    pivot_to_imu_body: Point2Length,
) -> Acceleration3 {
    let omega_rad_s = theta_dot.get::<radian_per_second>();
    let alpha_rad_s2 = theta_ddot.get::<radian_per_second_squared>();
    let rx_m = pivot_to_imu_body.x.get::<meter>();
    let ry_m = pivot_to_imu_body.y.get::<meter>();

    Acceleration3 {
        x: Acceleration::new::<meter_per_second_squared>(
            (-alpha_rad_s2 * ry_m) - (omega_rad_s * omega_rad_s * rx_m),
        ),
        y: Acceleration::new::<meter_per_second_squared>(
            (alpha_rad_s2 * rx_m) - (omega_rad_s * omega_rad_s * ry_m),
        ),
        z: Acceleration::new::<meter_per_second_squared>(0.0),
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

fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

fn standard_gravity_mps2() -> f32 {
    9.80665
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
                    y: Acceleration::new::<meter_per_second_squared>(-standard_gravity_mps2()),
                    z: Acceleration::new::<meter_per_second_squared>(0.0),
                },
                gyro: AngularVelocity3::default(),
            },
        );

        assert!(estimate.theta_deg.is_finite());
        assert!(estimate.theta_dot_dps.is_finite());
    }
}
