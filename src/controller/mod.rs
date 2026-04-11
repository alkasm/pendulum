use uom::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{Angle, AngularVelocity, Torque},
    torque::newton_meter,
};

#[derive(Debug, Clone, Copy)]
pub struct PdController {
    kp: f64,
    kd: f64,
    kwheel: f64,
    kwheel_soft_limit: f64,
    wheel_speed_soft_limit: AngularVelocity,
}

impl PdController {
    pub fn new(
        kp: f64,
        kd: f64,
        kwheel: f64,
        kwheel_soft_limit: f64,
        wheel_speed_soft_limit: AngularVelocity,
    ) -> Self {
        Self {
            kp,
            kd,
            kwheel,
            kwheel_soft_limit,
            wheel_speed_soft_limit,
        }
    }

    pub fn torque_command(
        &self,
        theta: Angle,
        theta_dot: AngularVelocity,
        wheel_speed: AngularVelocity,
    ) -> Torque {
        let wheel_speed_rad_s = wheel_speed.get::<radian_per_second>();
        let wheel_speed_soft_limit_rad_s = self.wheel_speed_soft_limit.get::<radian_per_second>();
        let wheel_speed_excess_rad_s = wheel_speed_rad_s.signum()
            * (wheel_speed_rad_s.abs() - wheel_speed_soft_limit_rad_s).max(0.0);
        let command = self.kp * theta.get::<radian>()
            + self.kd * theta_dot.get::<radian_per_second>()
            - self.kwheel * wheel_speed_rad_s
            - self.kwheel_soft_limit * wheel_speed_excess_rad_s;
        Torque::new::<newton_meter>(command)
    }
}
