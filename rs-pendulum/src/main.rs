#[cfg(feature = "sim")]
use pendulumd::sim;

#[cfg(feature = "hw")]
use pendulumd::hw;

use pendulumd::imu::Imu;

fn main() {
    println!("pendulumd starting...");

    #[cfg(feature = "sim")]
    run_sim();

    #[cfg(feature = "hw")]
    run_hw();
}

#[cfg(feature = "sim")]
fn run_sim() {
    use pendulumd::controller::PdController;
    use std::time::Duration;

    let dt = Duration::from_millis(10);
    let mut plant = sim::SimPlant::new(sim::PlantParams::default(), sim::PlantState::default());
    let mut imu = sim::SimImu::new();
    let controller = PdController::new(20.0, 3.0);

    imu.sample_from_state(plant.state());

    for step in 0..500 {
        let sample = imu.read().expect("sim imu should not fail");
        let wheel_torque = controller
            .torque_command(sample.theta, sample.theta_dot)
            .clamp(-1.0, 1.0);

        plant.step(wheel_torque, dt);
        imu.sample_from_state(plant.state());

        let state = plant.state();
        println!(
            "step={step:>3} theta={:+.4} theta_dot={:+.4} torque={:+.3} Nm",
            state.theta, state.theta_dot, wheel_torque
        );
    }
}

#[cfg(feature = "hw")]
fn run_hw() {
    use std::time::Duration;

    let mut imu = match hw::Mpu6050Imu::new() {
        Ok(imu) => imu,
        Err(e) => {
            eprintln!("hw IMU init failed: {e:?}");
            return;
        }
    };

    loop {
        match imu.read() {
            Ok(sample) => println!(
                "theta={:+.4} theta_dot={:+.4}",
                sample.theta, sample.theta_dot
            ),
            Err(e) => {
                eprintln!("hw IMU read failed: {e:?}");
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}
