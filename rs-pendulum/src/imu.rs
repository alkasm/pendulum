use uom::si::f64::{Angle, AngularVelocity};

#[derive(Debug, Clone, Copy, Default)]
pub struct ImuSample {
    pub theta: Angle,
    pub theta_dot: AngularVelocity,
}

pub trait Imu {
    type Error;
    fn read(&mut self) -> Result<ImuSample, Self::Error>;
}
