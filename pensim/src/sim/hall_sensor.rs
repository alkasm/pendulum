use uom::si::{angular_velocity::radian_per_second, f64::AngularVelocity};

#[derive(Debug, Clone, Copy)]
pub struct SimHallSensorSample {
    pub wheel_speed: AngularVelocity,
}

#[derive(Debug, Clone, Copy)]
pub struct SimHallSensor {
    sample: SimHallSensorSample,
}

impl SimHallSensor {
    pub fn new() -> Self {
        Self {
            sample: SimHallSensorSample {
                wheel_speed: AngularVelocity::new::<radian_per_second>(0.0),
            },
        }
    }

    pub fn read(&mut self) -> SimHallSensorSample {
        self.sample
    }

    pub fn sample_wheel_speed(&mut self, wheel_speed: AngularVelocity) {
        self.sample.wheel_speed = wheel_speed;
    }
}
