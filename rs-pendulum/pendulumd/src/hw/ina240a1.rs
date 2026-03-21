#[derive(Debug, Clone, Copy)]
pub struct Ina240A1Sample {
    pub phase_current_a: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Ina240A1 {
    sample: Ina240A1Sample,
}

impl Ina240A1 {
    pub fn new() -> Self {
        Self {
            sample: Ina240A1Sample {
                phase_current_a: 0.0,
            },
        }
    }

    pub fn read(&mut self) -> Ina240A1Sample {
        self.sample
    }

    pub fn set_mock_phase_current_a(&mut self, phase_current_a: f64) {
        self.sample.phase_current_a = phase_current_a;
    }
}
