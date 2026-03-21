use std::time::Duration;

use pendulum_lib::config::{DEFAULT_BODY_SIDE_LENGTH_M, DEFAULT_WHEEL_RADIUS_M};

const MAX_THETA_RAD: f64 = std::f64::consts::FRAC_PI_4;

#[derive(Debug, Clone, Copy)]
pub struct SimConfig {
    pub body_mass_kg: f64,
    pub body_side_length_m: f64,
    pub wheel_mass_kg: f64,
    pub wheel_radius_m: f64,
    pub gravity_m_s2: f64,
    pub initial_theta: f64,
    pub initial_theta_dot: f64,
    pub initial_wheel_angle: f64,
    pub initial_wheel_speed: f64,
    pub max_motor_torque_nm: f64,
    pub motor_no_load_speed_rad_s: f64,
    pub motor_torque_constant_nm_per_a: f64,
    pub controller_kp: f64,
    pub controller_kd: f64,
    pub dt_s: f64,
}

impl SimConfig {
    pub fn min_body_side_length_m_for_wheel(&self) -> f64 {
        3.0 * std::f64::consts::SQRT_2 * self.wheel_radius_m
    }

    pub fn effective_body_side_length_m(&self) -> f64 {
        self.body_side_length_m
            .max(self.min_body_side_length_m_for_wheel())
    }

    pub fn plant_params(&self) -> PlantParams {
        PlantParams::from_uniform_rod_body_and_wheel_disk(
            self.body_mass_kg,
            self.effective_body_side_length_m(),
            self.wheel_mass_kg,
            self.wheel_radius_m,
            self.gravity_m_s2,
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
            body_mass_kg: 0.3,
            body_side_length_m: DEFAULT_BODY_SIDE_LENGTH_M,
            wheel_mass_kg: 0.10,
            wheel_radius_m: DEFAULT_WHEEL_RADIUS_M,
            gravity_m_s2: 9.81,
            initial_theta: 0.02,
            initial_theta_dot: 0.0,
            initial_wheel_angle: 0.0,
            initial_wheel_speed: 0.0,
            // Datasheet stall/start torque: 320 gf*cm ~= 0.031 N*m.
            max_motor_torque_nm: 0.031,
            // Datasheet no-load speed: 2000 rpm ~= 209.4 rad/s.
            motor_no_load_speed_rad_s: 209.4,
            // Approximate torque constant from datasheet values: 0.031 N*m / 0.8 A ~= 0.039 N*m/A.
            motor_torque_constant_nm_per_a: 0.039,
            controller_kp: 0.22,
            controller_kd: 0.03,
            dt_s: 0.01,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlantParams {
    pub i_body: f64,
    pub i_wheel: f64,
    pub mgl: f64,
}

impl PlantParams {
    pub fn from_inertias(
        i_body_kg_m2: f64,
        i_wheel_kg_m2: f64,
        body_mass_kg: f64,
        com_length_m: f64,
        gravity_m_s2: f64,
    ) -> Self {
        Self {
            i_body: i_body_kg_m2,
            i_wheel: i_wheel_kg_m2,
            mgl: body_mass_kg * gravity_m_s2 * com_length_m,
        }
    }

    pub fn from_uniform_rod_body_and_wheel_disk(
        body_mass_kg: f64,
        body_length_m: f64,
        wheel_mass_kg: f64,
        wheel_radius_m: f64,
        gravity_m_s2: f64,
    ) -> Self {
        let i_body = (body_mass_kg * body_length_m.powi(2)) / 3.0;
        let com_length_m = body_length_m / 2.0;
        let i_wheel = 0.5 * wheel_mass_kg * wheel_radius_m.powi(2);
        Self::from_inertias(i_body, i_wheel, body_mass_kg, com_length_m, gravity_m_s2)
    }
}

impl Default for PlantParams {
    fn default() -> Self {
        Self {
            i_body: 1.0,
            i_wheel: 1.0,
            mgl: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlantState {
    pub theta: f64,
    pub theta_dot: f64,
    pub wheel_angle: f64,
    pub wheel_speed: f64,
}

impl Default for PlantState {
    fn default() -> Self {
        Self {
            theta: 1.0,
            theta_dot: 0.0,
            wheel_angle: 0.0,
            wheel_speed: 0.0,
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

    pub fn step(&mut self, wheel_torque_nm: f64, dt: Duration) {
        let dt_s = dt.as_secs_f64();

        let theta_ddot =
            (self.params.mgl * self.state.theta.sin() - wheel_torque_nm) / self.params.i_body;
        let wheel_ddot = wheel_torque_nm / self.params.i_wheel;

        self.state.theta_dot += theta_ddot * dt_s;
        self.state.theta += self.state.theta_dot * dt_s;
        self.enforce_angle_bounds();

        self.state.wheel_speed += wheel_ddot * dt_s;
        self.state.wheel_angle += self.state.wheel_speed * dt_s;
    }

    fn enforce_angle_bounds(&mut self) {
        self.state.theta = self.state.theta.clamp(-MAX_THETA_RAD, MAX_THETA_RAD);

        if (self.state.theta >= MAX_THETA_RAD && self.state.theta_dot > 0.0)
            || (self.state.theta <= -MAX_THETA_RAD && self.state.theta_dot < 0.0)
        {
            self.state.theta_dot = 0.0;
        }
    }
}
