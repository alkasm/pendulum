use uom::si::{f64::Torque, torque::newton_meter};

#[derive(Debug, Clone, Copy)]
pub struct Tmc6300 {
    enabled: bool,
    requested_torque: Torque,
}

impl Tmc6300 {
    pub fn new() -> Self {
        Self {
            enabled: true,
            requested_torque: Torque::new::<newton_meter>(0.0),
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    pub fn command_torque(&mut self, torque: Torque) {
        self.requested_torque = if self.enabled {
            torque
        } else {
            Torque::new::<newton_meter>(0.0)
        };
    }

    pub fn requested_torque(&self) -> Torque {
        self.requested_torque
    }
}
