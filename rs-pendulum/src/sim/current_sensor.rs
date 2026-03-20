#[derive(Debug, Clone, Copy)]
pub struct SimCurrentSensorSample {
    pub phase_current_a: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct SimCurrentSensor {
    sample: SimCurrentSensorSample,
}

impl SimCurrentSensor {
    pub fn new() -> Self {
        Self {
            sample: SimCurrentSensorSample {
                phase_current_a: 0.0,
            },
        }
    }

    pub fn read(&mut self) -> SimCurrentSensorSample {
        self.sample
    }

    pub fn sample_phase_current_a(&mut self, phase_current_a: f64) {
        self.sample.phase_current_a = phase_current_a;
    }
}
