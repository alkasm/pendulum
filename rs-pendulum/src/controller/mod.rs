#[derive(Debug, Clone, Copy)]
pub struct PdController {
    kp: f64,
    kd: f64,
}

impl PdController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self { kp, kd }
    }

    pub fn torque_command(&self, theta: f64, theta_dot: f64) -> f64 {
        self.kp * theta + self.kd * theta_dot
    }
}
