use crate::protocol::PendulumControlMode;

#[derive(Debug, Clone, Copy)]
pub struct ControllerConfig {
    pub dt_s: f32,
    pub kp_nm_per_rad: f32,
    pub kd_nm_per_rad_s: f32,
    pub max_phase_current_a: f32,
    pub arm_angle_deg: f32,
    pub arm_rate_dps: f32,
    pub arm_sample_target: u16,
    pub capture_angle_deg: f32,
    pub theta_target_deg: f32,
    pub max_command_torque_nm: f32,
    pub drive_deadband: f32,
    pub drive_idle_epsilon: f32,
    pub max_drive_step_per_tick: f32,
    pub max_drive_reversal_step_per_tick: f32,
}

impl ControllerConfig {
    pub fn basic_pd(kp_nm_per_rad: f32, kd_nm_per_rad_s: f32, dt_s: f32) -> Self {
        Self {
            kp_nm_per_rad,
            kd_nm_per_rad_s,
            dt_s,
            ..Self::default()
        }
    }
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            dt_s: 0.005,
            kp_nm_per_rad: 0.22,
            kd_nm_per_rad_s: 0.007,
            max_phase_current_a: 1.2,
            arm_angle_deg: 12.0,
            arm_rate_dps: 20.0,
            arm_sample_target: 3,
            capture_angle_deg: 60.0,
            theta_target_deg: -2.10,
            max_command_torque_nm: 0.030,
            drive_deadband: 0.0,
            drive_idle_epsilon: 0.0,
            max_drive_step_per_tick: 0.08,
            max_drive_reversal_step_per_tick: 0.08,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ControllerInput {
    pub hall_angle_deg: Option<f32>,
    pub theta_deg: Option<f32>,
    pub theta_dot_dps: Option<f32>,
    pub max_phase_current_a: f32,
    pub actuator_ready: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct ControllerOutput {
    pub mode: PendulumControlMode,
    pub theta_error_deg: f32,
    pub torque_command_nm: f32,
    pub raw_drive_command: f32,
    pub drive_command: f32,
    pub wheel_angle_deg: f32,
    pub wheel_speed_dps: f32,
    pub motor_enabled: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct PendulumController {
    config: ControllerConfig,
    state: ControllerState,
}

impl PendulumController {
    pub fn new(config: ControllerConfig) -> Self {
        Self {
            config,
            state: ControllerState::default(),
        }
    }

    pub fn reset_runtime(&mut self) {
        self.state = ControllerState::default();
    }

    pub fn step(&mut self, input: ControllerInput) -> ControllerOutput {
        let wheel_angle_deg = input.hall_angle_deg.unwrap_or(0.0);
        let wheel_speed_dps = match input.hall_angle_deg {
            Some(angle_deg) => self.state.observe_wheel(angle_deg, self.config.dt_s),
            None => {
                self.state.reset_wheel_observer();
                0.0
            }
        };

        let hall_missing = input.hall_angle_deg.is_none();
        if hall_missing {
            self.state.reset_arming();
            self.state.stop_output();
            return ControllerOutput {
                mode: PendulumControlMode::WaitingForHall,
                theta_error_deg: 0.0,
                torque_command_nm: 0.0,
                raw_drive_command: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                motor_enabled: false,
            };
        }

        if !input.actuator_ready {
            self.state.reset_arming();
            self.state.stop_output();
            return ControllerOutput {
                mode: PendulumControlMode::Startup,
                theta_error_deg: 0.0,
                torque_command_nm: 0.0,
                raw_drive_command: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                motor_enabled: false,
            };
        }

        let Some(theta_deg) = input.theta_deg else {
            self.state.reset_arming();
            self.state.stop_output();
            return ControllerOutput {
                mode: PendulumControlMode::WaitingForImu,
                theta_error_deg: 0.0,
                torque_command_nm: 0.0,
                raw_drive_command: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                motor_enabled: false,
            };
        };

        let Some(theta_dot_dps) = input.theta_dot_dps else {
            self.state.reset_arming();
            self.state.stop_output();
            return ControllerOutput {
                mode: PendulumControlMode::WaitingForImu,
                theta_error_deg: 0.0,
                torque_command_nm: 0.0,
                raw_drive_command: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                motor_enabled: false,
            };
        };

        let theta_error_deg = theta_deg - self.config.theta_target_deg;

        if !self
            .state
            .step_arming(theta_deg, theta_dot_dps, &self.config)
        {
            self.state.stop_output();
            return ControllerOutput {
                mode: PendulumControlMode::Arming,
                theta_error_deg,
                torque_command_nm: 0.0,
                raw_drive_command: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                motor_enabled: false,
            };
        }

        if theta_deg.abs() > self.config.capture_angle_deg {
            self.state.stop_output();
            return ControllerOutput {
                mode: PendulumControlMode::CaptureOutOfRange,
                theta_error_deg,
                torque_command_nm: 0.0,
                raw_drive_command: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                motor_enabled: false,
            };
        }

        if input.max_phase_current_a > self.config.max_phase_current_a {
            self.state.stop_output();
            return ControllerOutput {
                mode: PendulumControlMode::CurrentLimited,
                theta_error_deg,
                torque_command_nm: 0.0,
                raw_drive_command: 0.0,
                drive_command: 0.0,
                wheel_angle_deg,
                wheel_speed_dps,
                motor_enabled: false,
            };
        }

        let torque_command_nm = pd_torque_command_nm(theta_error_deg, theta_dot_dps, &self.config);
        let raw_drive_command = clamp(
            torque_command_nm / self.config.max_command_torque_nm,
            -1.0,
            1.0,
        );
        let drive_target = if raw_drive_command.abs() < self.config.drive_deadband {
            0.0
        } else {
            raw_drive_command
        };
        let drive_command = self.state.slew_drive_command(drive_target, &self.config);
        let mode = if drive_command.abs() < self.config.drive_idle_epsilon {
            PendulumControlMode::Idle
        } else {
            PendulumControlMode::Active
        };

        ControllerOutput {
            mode,
            theta_error_deg,
            torque_command_nm,
            raw_drive_command,
            drive_command,
            wheel_angle_deg,
            wheel_speed_dps,
            motor_enabled: true,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct ControllerState {
    armed: bool,
    arm_ready_samples: u16,
    last_wheel_angle_deg: Option<f32>,
    filtered_wheel_speed_dps: f32,
    filtered_drive_command: f32,
}

impl ControllerState {
    fn observe_wheel(&mut self, angle_deg: f32, dt_s: f32) -> f32 {
        let instant_speed_dps = self
            .last_wheel_angle_deg
            .map(|previous| wrap_angle_delta_deg(angle_deg - previous) / dt_s)
            .unwrap_or(0.0);
        self.filtered_wheel_speed_dps =
            0.8 * self.filtered_wheel_speed_dps + 0.2 * instant_speed_dps;
        self.last_wheel_angle_deg = Some(angle_deg);
        self.filtered_wheel_speed_dps
    }

    fn reset_wheel_observer(&mut self) {
        self.last_wheel_angle_deg = None;
        self.filtered_wheel_speed_dps = 0.0;
    }

    fn stop_output(&mut self) {
        self.filtered_drive_command = 0.0;
    }

    fn slew_drive_command(&mut self, target: f32, config: &ControllerConfig) -> f32 {
        let changing_direction = self.filtered_drive_command.abs() > config.drive_idle_epsilon
            && target.abs() > config.drive_idle_epsilon
            && self.filtered_drive_command.signum() != target.signum();
        let max_step = if changing_direction {
            config.max_drive_reversal_step_per_tick
        } else {
            config.max_drive_step_per_tick
        };
        let delta = clamp(target - self.filtered_drive_command, -max_step, max_step);
        self.filtered_drive_command += delta;
        self.filtered_drive_command
    }

    fn reset_arming(&mut self) {
        self.armed = false;
        self.arm_ready_samples = 0;
    }

    fn step_arming(
        &mut self,
        theta_deg: f32,
        theta_dot_dps: f32,
        config: &ControllerConfig,
    ) -> bool {
        if self.armed {
            return true;
        }

        let inside_arm_window =
            theta_deg.abs() <= config.arm_angle_deg && theta_dot_dps.abs() <= config.arm_rate_dps;

        if inside_arm_window {
            self.arm_ready_samples = self.arm_ready_samples.saturating_add(1);
            if self.arm_ready_samples >= config.arm_sample_target {
                self.armed = true;
                return true;
            }
        } else {
            self.arm_ready_samples = 0;
        }

        false
    }
}

fn pd_torque_command_nm(
    theta_error_deg: f32,
    theta_dot_dps: f32,
    config: &ControllerConfig,
) -> f32 {
    let theta_rad = degrees_to_radians(theta_error_deg);
    let theta_dot_rad_s = degrees_to_radians(theta_dot_dps);
    config.kp_nm_per_rad * theta_rad + config.kd_nm_per_rad_s * theta_dot_rad_s
}

fn degrees_to_radians(angle_deg: f32) -> f32 {
    angle_deg * (core::f32::consts::PI / 180.0)
}

fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

fn wrap_angle_delta_deg(delta_deg: f32) -> f32 {
    if delta_deg > 180.0 {
        delta_deg - 360.0
    } else if delta_deg < -180.0 {
        delta_deg + 360.0
    } else {
        delta_deg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waits_for_hall_before_actuating() {
        let mut controller = PendulumController::new(ControllerConfig::default());
        let output = controller.step(ControllerInput {
            hall_angle_deg: None,
            theta_deg: Some(0.0),
            theta_dot_dps: Some(0.0),
            max_phase_current_a: 0.0,
            actuator_ready: true,
        });

        assert_eq!(output.mode, PendulumControlMode::WaitingForHall);
        assert!(!output.motor_enabled);
    }

    #[test]
    fn arms_then_runs_pd_control() {
        let mut controller = PendulumController::new(ControllerConfig::default());
        let mut output = ControllerOutput {
            mode: PendulumControlMode::Startup,
            theta_error_deg: 0.0,
            torque_command_nm: 0.0,
            raw_drive_command: 0.0,
            drive_command: 0.0,
            wheel_angle_deg: 0.0,
            wheel_speed_dps: 0.0,
            motor_enabled: false,
        };

        for _ in 0..3 {
            output = controller.step(ControllerInput {
                hall_angle_deg: Some(15.0),
                theta_deg: Some(-2.10),
                theta_dot_dps: Some(0.0),
                max_phase_current_a: 0.0,
                actuator_ready: true,
            });
        }

        assert!(matches!(
            output.mode,
            PendulumControlMode::Idle | PendulumControlMode::Active
        ));
        assert!(output.motor_enabled);
    }
}
