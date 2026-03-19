#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub theta: f64,
    pub theta_dot: f64,
}

pub trait Imu {
    type Error;
    fn read(&mut self) -> Result<ImuSample, Self::Error>;
}
