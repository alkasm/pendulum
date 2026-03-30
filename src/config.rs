use crate::pendulum::{
    BodyAxis3, ImuAxesInBody, ImuMount, MotorMount, Pendulum, PendulumGeometry,
    PendulumHardware, Point2, Point3, RightTriangularBody,
};
use uom::si::{
    angular_velocity::radian_per_second,
    f64::{AngularVelocity, Length, Time, Torque},
    length::{meter, millimeter},
    time::second,
    torque::newton_meter,
};

pub fn default_body_side_length() -> Length {
    Length::new::<meter>(0.14)
}

pub fn default_wheel_radius() -> Length {
    Length::new::<meter>(0.03)
}

pub fn default_body_depth() -> Length {
    Length::new::<meter>(0.02)
}

pub fn default_pendulum() -> Pendulum {
    let body = RightTriangularBody::new(
        default_body_side_length(),
        default_body_side_length(),
        default_body_depth(),
    );
    let center_of_mass_from_pivot = Point2::new(body.leg_x / 3.0, body.leg_y / 3.0);
    let motor_mount = MotorMount::new(Point3::new(
        Length::new::<millimeter>(0.0),
        Length::new::<millimeter>(0.235),
        Length::new::<millimeter>(0.0),
    ));
    let imu_mount = ImuMount::new(
        Point3::new(
            Length::new::<millimeter>(-50.0),
            Length::new::<millimeter>(27.36),
            Length::new::<millimeter>(10.0),
        ),
        ImuAxesInBody::new(
        BodyAxis3::Down,
        BodyAxis3::Right,
        BodyAxis3::TowardViewer,
        ),
    );

    Pendulum::new(
        PendulumGeometry::new(
            body,
            center_of_mass_from_pivot,
            motor_mount,
            imu_mount,
        ),
        PendulumHardware::new(0x68, 0x22),
    )
}

#[derive(Debug, Clone, Copy)]
pub struct RuntimeConfig {
    pub controller_kp: f64,
    pub controller_kd: f64,
    pub dt: Time,
    pub max_motor_torque: Torque,
    pub motor_no_load_speed: AngularVelocity,
    pub motor_torque_constant_nm_per_a: f64,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            controller_kp: 0.22,
            controller_kd: 0.001,
            dt: Time::new::<second>(0.01),
            // Datasheet stall/start torque: 320 gf*cm ~= 0.031 N*m.
            max_motor_torque: Torque::new::<newton_meter>(0.031),
            // Datasheet no-load speed: 2000 rpm ~= 209.4 rad/s.
            motor_no_load_speed: AngularVelocity::new::<radian_per_second>(209.4),
            // Approximate torque constant from datasheet values: 0.031 N*m / 0.8 A ~= 0.039 N*m/A.
            motor_torque_constant_nm_per_a: 0.039,
        }
    }
}
