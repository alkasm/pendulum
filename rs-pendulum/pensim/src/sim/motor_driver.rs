use uom::si::{f64::Torque, torque::newton_meter};

#[derive(Debug, Clone, Copy)]
pub struct SimMotorDriver {
    enabled: bool,
    requested_torque: Torque,
}

impl SimMotorDriver {
    pub fn new() -> Self {
        Self {
            enabled: true,
            requested_torque: Torque::new::<newton_meter>(0.0),
        }
    }

    pub fn command_torque(&mut self, torque: Torque) {
        self.requested_torque = if self.enabled {
            torque
        } else {
            Torque::new::<newton_meter>(0.0)
        };
    }
}
