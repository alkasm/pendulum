#[derive(Debug, Clone, Copy)]
pub struct PdController {
    kp: f64,
    kd: f64,
    // Extra wheel-speed feedback gain; this makes the controller resist
    // dumping excessive angular momentum into the reaction wheel.
    kw: f64,
}

impl PdController {
    pub fn new(kp: f64, kd: f64, kw: f64) -> Self {
        Self { kp, kd, kw }
    }

    pub fn torque_command(&self, theta: f64, theta_dot: f64, wheel_speed: f64) -> f64 {
        self.kp * theta + self.kd * theta_dot - self.kw * wheel_speed
    }
}
