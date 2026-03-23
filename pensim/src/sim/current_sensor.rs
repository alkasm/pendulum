use uom::si::{electric_current::ampere, f64::ElectricCurrent};

#[derive(Debug, Clone, Copy)]
pub struct SimCurrentSensorSample {
    pub phase_current: ElectricCurrent,
}

#[derive(Debug, Clone, Copy)]
pub struct SimCurrentSensor {
    sample: SimCurrentSensorSample,
}

impl SimCurrentSensor {
    pub fn new() -> Self {
        Self {
            sample: SimCurrentSensorSample {
                phase_current: ElectricCurrent::new::<ampere>(0.0),
            },
        }
    }

    pub fn read(&mut self) -> SimCurrentSensorSample {
        self.sample
    }

    pub fn sample_phase_current(&mut self, phase_current: ElectricCurrent) {
        self.sample.phase_current = phase_current;
    }
}
