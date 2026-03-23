use pendulum_lib::config::{default_body_side_length, default_wheel_radius};
use uom::si::{f64::Length, length::meter};

pub const DEFAULT_PIXELS_PER_METER: f32 = 3200.0;

#[derive(Debug, Clone, Copy)]
pub struct VisualizationConfig {
    pub body_side_length: Length,
    pub wheel_radius: Length,
    pub pixels_per_meter: f32,
}

impl VisualizationConfig {
    pub fn new(body_side_length: Length, wheel_radius: Length, pixels_per_meter: f32) -> Self {
        Self {
            body_side_length,
            wheel_radius,
            pixels_per_meter,
        }
    }

    pub fn triangle_leg_length_px(&self) -> f32 {
        self.body_side_length.get::<meter>() as f32 * self.pixels_per_meter
    }

    pub fn motor_radius_px(&self) -> f32 {
        self.wheel_radius.get::<meter>() as f32 * self.pixels_per_meter
    }
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self::new(
            default_body_side_length(),
            default_wheel_radius(),
            DEFAULT_PIXELS_PER_METER,
        )
    }
}
