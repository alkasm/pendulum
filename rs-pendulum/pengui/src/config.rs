use pendulum_lib::config::{DEFAULT_BODY_SIDE_LENGTH_M, DEFAULT_WHEEL_RADIUS_M};

pub const DEFAULT_PIXELS_PER_METER: f32 = 3200.0;

#[derive(Debug, Clone, Copy)]
pub struct VisualizationConfig {
    pub body_side_length_m: f32,
    pub wheel_radius_m: f32,
    pub pixels_per_meter: f32,
}

impl VisualizationConfig {
    pub const fn new(body_side_length_m: f32, wheel_radius_m: f32, pixels_per_meter: f32) -> Self {
        Self {
            body_side_length_m,
            wheel_radius_m,
            pixels_per_meter,
        }
    }

    pub fn triangle_leg_length_px(&self) -> f32 {
        self.body_side_length_m * self.pixels_per_meter
    }

    pub fn motor_radius_px(&self) -> f32 {
        self.wheel_radius_m * self.pixels_per_meter
    }
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self::new(
            DEFAULT_BODY_SIDE_LENGTH_M as f32,
            DEFAULT_WHEEL_RADIUS_M as f32,
            DEFAULT_PIXELS_PER_METER,
        )
    }
}
