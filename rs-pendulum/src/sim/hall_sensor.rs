#[derive(Debug, Clone, Copy)]
pub struct SimHallSensorSample {
    pub wheel_speed_rad_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct SimHallSensor {
    sample: SimHallSensorSample,
}

impl SimHallSensor {
    pub fn new() -> Self {
        Self {
            sample: SimHallSensorSample {
                wheel_speed_rad_s: 0.0,
            },
        }
    }

    pub fn read(&mut self) -> SimHallSensorSample {
        self.sample
    }

    pub fn sample_wheel_speed_rad_s(&mut self, wheel_speed_rad_s: f64) {
        self.sample.wheel_speed_rad_s = wheel_speed_rad_s;
    }
}
