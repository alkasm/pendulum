use uom::si::{angular_velocity::radian_per_second, f64::AngularVelocity};

#[derive(Debug, Clone, Copy)]
pub struct Tmag5273Sample {
    pub wheel_speed: AngularVelocity,
}

#[derive(Debug, Clone, Copy)]
pub struct Tmag5273 {
    sample: Tmag5273Sample,
}

impl Tmag5273 {
    pub fn new() -> Self {
        Self {
            sample: Tmag5273Sample {
                wheel_speed: AngularVelocity::new::<radian_per_second>(0.0),
            },
        }
    }

    pub fn read(&mut self) -> Tmag5273Sample {
        self.sample
    }
}
