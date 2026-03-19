use crate::imu::{Imu, ImuSample};
use crate::sim::physics::PlantState;

#[derive(Debug, Clone, Copy)]
pub struct SimImu {
    sample: ImuSample,
}

impl SimImu {
    pub fn new() -> Self {
        Self {
            sample: ImuSample {
                theta: 0.0,
                theta_dot: 0.0,
            },
        }
    }

    pub fn sample_from_state(&mut self, state: PlantState) {
        self.sample.theta = state.theta;
        self.sample.theta_dot = state.theta_dot;
    }
}

impl Imu for SimImu {
    type Error = core::convert::Infallible;

    fn read(&mut self) -> Result<ImuSample, Self::Error> {
        Ok(self.sample)
    }
}
