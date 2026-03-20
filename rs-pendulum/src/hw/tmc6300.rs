#[derive(Debug, Clone, Copy)]
pub struct Tmc6300 {
    enabled: bool,
    requested_torque_nm: f64,
}

impl Tmc6300 {
    pub fn new() -> Self {
        Self {
            enabled: true,
            requested_torque_nm: 0.0,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    pub fn command_torque(&mut self, torque_nm: f64) {
        self.requested_torque_nm = if self.enabled { torque_nm } else { 0.0 };
    }

    pub fn requested_torque_nm(&self) -> f64 {
        self.requested_torque_nm
    }
}
