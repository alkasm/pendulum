pub const DEFAULT_BODY_SIDE_LENGTH_M: f64 = 0.14;
pub const DEFAULT_WHEEL_RADIUS_M: f64 = 0.03;

#[derive(Debug, Clone, Copy)]
pub struct RuntimeConfig {
    pub controller_kp: f64,
    pub controller_kd: f64,
    pub dt_s: f64,
    pub max_motor_torque_nm: f64,
    pub motor_no_load_speed_rad_s: f64,
    pub motor_torque_constant_nm_per_a: f64,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            controller_kp: 0.22,
            controller_kd: 0.001,
            dt_s: 0.01,
            // Datasheet stall/start torque: 320 gf*cm ~= 0.031 N*m.
            max_motor_torque_nm: 0.031,
            // Datasheet no-load speed: 2000 rpm ~= 209.4 rad/s.
            motor_no_load_speed_rad_s: 209.4,
            // Approximate torque constant from datasheet values: 0.031 N*m / 0.8 A ~= 0.039 N*m/A.
            motor_torque_constant_nm_per_a: 0.039,
        }
    }
}
