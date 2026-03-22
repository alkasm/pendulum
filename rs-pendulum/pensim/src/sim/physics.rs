use std::time::Duration;

use pendulum_lib::config::{default_body_side_length, default_wheel_radius, RuntimeConfig};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{Acceleration, Angle, AngularVelocity, Length, Mass, MomentOfInertia, Torque},
    length::meter,
    mass::kilogram,
    moment_of_inertia::kilogram_square_meter,
    torque::newton_meter,
};

fn max_theta() -> Angle {
    Angle::new::<radian>(std::f64::consts::FRAC_PI_4)
}

#[derive(Debug, Clone, Copy)]
pub struct SimConfig {
    pub runtime: RuntimeConfig,
    pub body_mass: Mass,
    pub body_side_length: Length,
    pub wheel_mass: Mass,
    pub wheel_radius: Length,
    pub gravity: Acceleration,
    pub initial_theta: Angle,
    pub initial_theta_dot: AngularVelocity,
    pub initial_wheel_angle: Angle,
    pub initial_wheel_speed: AngularVelocity,
}

impl SimConfig {
    pub fn min_body_side_length_for_wheel(&self) -> Length {
        self.wheel_radius * (3.0 * std::f64::consts::SQRT_2)
    }

    pub fn effective_body_side_length(&self) -> Length {
        self.body_side_length.max(self.min_body_side_length_for_wheel())
    }

    pub fn plant_params(&self) -> PlantParams {
        PlantParams::from_uniform_rod_body_and_wheel_disk(
            self.body_mass,
            self.effective_body_side_length(),
            self.wheel_mass,
            self.wheel_radius,
            self.gravity,
        )
    }

    pub fn initial_state(&self) -> PlantState {
        PlantState {
            theta: self.initial_theta,
            theta_dot: self.initial_theta_dot,
            wheel_angle: self.initial_wheel_angle,
            wheel_speed: self.initial_wheel_speed,
        }
    }
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            runtime: RuntimeConfig::default(),
            body_mass: Mass::new::<kilogram>(0.3),
            body_side_length: default_body_side_length(),
            wheel_mass: Mass::new::<kilogram>(0.10),
            wheel_radius: default_wheel_radius(),
            gravity: Acceleration::new::<meter_per_second_squared>(9.81),
            initial_theta: Angle::new::<radian>(0.02),
            initial_theta_dot: AngularVelocity::new::<radian_per_second>(0.0),
            initial_wheel_angle: Angle::new::<radian>(0.0),
            initial_wheel_speed: AngularVelocity::new::<radian_per_second>(0.0),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlantParams {
    pub i_body: MomentOfInertia,
    pub i_wheel: MomentOfInertia,
    pub mgl: Torque,
}

impl PlantParams {
    pub fn from_inertias(
        i_body: MomentOfInertia,
        i_wheel: MomentOfInertia,
        body_mass: Mass,
        com_length: Length,
        gravity: Acceleration,
    ) -> Self {
        Self {
            i_body,
            i_wheel,
            mgl: Torque::new::<newton_meter>(
                body_mass.get::<kilogram>()
                    * gravity.get::<meter_per_second_squared>()
                    * com_length.get::<meter>(),
            ),
        }
    }

    pub fn from_uniform_rod_body_and_wheel_disk(
        body_mass: Mass,
        body_length: Length,
        wheel_mass: Mass,
        wheel_radius: Length,
        gravity: Acceleration,
    ) -> Self {
        let body_mass_kg = body_mass.get::<kilogram>();
        let body_length_m = body_length.get::<meter>();
        let wheel_mass_kg = wheel_mass.get::<kilogram>();
        let wheel_radius_m = wheel_radius.get::<meter>();
        let i_body =
            MomentOfInertia::new::<kilogram_square_meter>((body_mass_kg * body_length_m.powi(2)) / 3.0);
        let com_length = body_length / 2.0;
        let i_wheel =
            MomentOfInertia::new::<kilogram_square_meter>(0.5 * wheel_mass_kg * wheel_radius_m.powi(2));
        Self::from_inertias(i_body, i_wheel, body_mass, com_length, gravity)
    }
}

impl Default for PlantParams {
    fn default() -> Self {
        Self {
            i_body: MomentOfInertia::new::<kilogram_square_meter>(1.0),
            i_wheel: MomentOfInertia::new::<kilogram_square_meter>(1.0),
            mgl: Torque::new::<newton_meter>(1.0),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlantState {
    pub theta: Angle,
    pub theta_dot: AngularVelocity,
    pub wheel_angle: Angle,
    pub wheel_speed: AngularVelocity,
}

impl Default for PlantState {
    fn default() -> Self {
        Self {
            theta: Angle::new::<radian>(1.0),
            theta_dot: AngularVelocity::new::<radian_per_second>(0.0),
            wheel_angle: Angle::new::<radian>(0.0),
            wheel_speed: AngularVelocity::new::<radian_per_second>(0.0),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SimPlant {
    params: PlantParams,
    state: PlantState,
}

impl SimPlant {
    pub fn new(params: PlantParams, initial_state: PlantState) -> Self {
        let mut plant = Self {
            params,
            state: initial_state,
        };
        plant.enforce_angle_bounds();
        plant
    }

    pub fn state(&self) -> PlantState {
        self.state
    }

    pub fn step(&mut self, wheel_torque: Torque, dt: Duration) {
        let dt_s = dt.as_secs_f64();
        let theta = self.state.theta.get::<radian>();
        let theta_dot = self.state.theta_dot.get::<radian_per_second>();
        let wheel_angle = self.state.wheel_angle.get::<radian>();
        let wheel_speed = self.state.wheel_speed.get::<radian_per_second>();
        let wheel_torque_nm = wheel_torque.get::<newton_meter>();

        let theta_ddot = ((self.params.mgl.get::<newton_meter>() * theta.sin()) - wheel_torque_nm)
            / self.params.i_body.get::<kilogram_square_meter>();
        let wheel_ddot = wheel_torque_nm / self.params.i_wheel.get::<kilogram_square_meter>();

        let next_theta_dot = theta_dot + theta_ddot * dt_s;
        let next_theta = theta + next_theta_dot * dt_s;

        self.state.theta_dot = AngularVelocity::new::<radian_per_second>(next_theta_dot);
        self.state.theta = Angle::new::<radian>(next_theta);
        self.enforce_angle_bounds();

        let next_wheel_speed = wheel_speed + wheel_ddot * dt_s;
        let next_wheel_angle = wheel_angle + next_wheel_speed * dt_s;
        self.state.wheel_speed = AngularVelocity::new::<radian_per_second>(next_wheel_speed);
        self.state.wheel_angle = Angle::new::<radian>(next_wheel_angle);
    }

    fn enforce_angle_bounds(&mut self) {
        if self.state.theta > max_theta() {
            self.state.theta = max_theta();
        } else if self.state.theta < -max_theta() {
            self.state.theta = -max_theta();
        }

        if (self.state.theta >= max_theta()
            && self.state.theta_dot > AngularVelocity::new::<radian_per_second>(0.0))
            || (self.state.theta <= -max_theta()
                && self.state.theta_dot < AngularVelocity::new::<radian_per_second>(0.0))
        {
            self.state.theta_dot = AngularVelocity::new::<radian_per_second>(0.0);
        }
    }
}
