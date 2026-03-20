#[derive(Debug, Clone, Copy)]
pub struct Tmag5273Sample {
    pub wheel_speed_rad_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Tmag5273 {
    sample: Tmag5273Sample,
}

impl Tmag5273 {
    pub fn new() -> Self {
        Self {
            sample: Tmag5273Sample {
                wheel_speed_rad_s: 0.0,
            },
        }
    }

    pub fn read(&mut self) -> Tmag5273Sample {
        self.sample
    }

    pub fn set_mock_wheel_speed_rad_s(&mut self, wheel_speed_rad_s: f64) {
        self.sample.wheel_speed_rad_s = wheel_speed_rad_s;
    }
}
