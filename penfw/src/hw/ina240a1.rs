use uom::si::{electric_current::ampere, f64::ElectricCurrent};

#[derive(Debug, Clone, Copy)]
pub struct Ina240A1Sample {
    pub phase_current: ElectricCurrent,
}

#[derive(Debug, Clone, Copy)]
pub struct Ina240A1 {
    sample: Ina240A1Sample,
}

impl Ina240A1 {
    pub fn new() -> Self {
        Self {
            sample: Ina240A1Sample {
                phase_current: ElectricCurrent::new::<ampere>(0.0),
            },
        }
    }

    pub fn read(&mut self) -> Ina240A1Sample {
        self.sample
    }
}
