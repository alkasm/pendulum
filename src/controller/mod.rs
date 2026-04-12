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
}

impl PdController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self { kp, kd }
    }

    pub fn torque_command(&self, theta: Angle, theta_dot: AngularVelocity) -> Torque {
        let command = self.kp * theta.get::<radian>()
            + self.kd * theta_dot.get::<radian_per_second>();
        Torque::new::<newton_meter>(command)
    }
}
