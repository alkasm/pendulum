use std::{
    thread,
    time::{Duration, Instant},
};

pub trait StepRuntime: Send + 'static {
    fn step(&mut self);
    fn step_dt(&self) -> Duration;
}

pub fn run_loop<R>(mut runtime: R)
where
    R: StepRuntime,
{
    loop {
        let loop_start = Instant::now();
        runtime.step();
        let elapsed = loop_start.elapsed();
        let step_dt = runtime.step_dt();

        if elapsed < step_dt {
            thread::sleep(step_dt - elapsed);
        }
    }
}
