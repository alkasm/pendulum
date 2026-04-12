pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub fn degrees_to_radians(angle_deg: f32) -> f32 {
    angle_deg * (core::f32::consts::PI / 180.0)
}

pub fn wrap_degrees(angle_deg: f32) -> f32 {
    let mut wrapped = angle_deg;
    while wrapped >= 360.0 {
        wrapped -= 360.0;
    }
    while wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
}

pub fn wrap_signed_degrees(angle_deg: f32) -> f32 {
    let mut wrapped = angle_deg;
    while wrapped > 180.0 {
        wrapped -= 360.0;
    }
    while wrapped <= -180.0 {
        wrapped += 360.0;
    }
    wrapped
}

pub fn unwrap_near(reference_unwrapped_deg: f32, raw_wrapped_deg: f32) -> f32 {
    reference_unwrapped_deg
        + wrap_angle_delta_deg(raw_wrapped_deg - wrap_degrees(reference_unwrapped_deg))
}

pub fn wrap_angle_delta_deg(delta: f32) -> f32 {
    if delta > 180.0 {
        delta - 360.0
    } else if delta < -180.0 {
        delta + 360.0
    } else {
        delta
    }
}
