use std::time::Duration;

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

    pub fn from_lumped_body_mass_and_wheel_disk(
        body_mass_kg: f64,
        com_length_m: f64,
        wheel_mass_kg: f64,
        wheel_radius_m: f64,
        gravity_m_s2: f64,
    ) -> Self {
        let i_body = body_mass_kg * com_length_m.powi(2);
        let i_wheel = 0.5 * wheel_mass_kg * wheel_radius_m.powi(2);
        Self::from_inertias(i_body, i_wheel, body_mass_kg, com_length_m, gravity_m_s2)
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
            theta: 0.1,
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
        Self {
            params,
            state: initial_state,
        }
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

        self.state.wheel_speed += wheel_ddot * dt_s;
        self.state.wheel_angle += self.state.wheel_speed * dt_s;
    }
}
