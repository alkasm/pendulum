#[cfg(feature = "gui")]
use bevy::prelude::*;
#[cfg(feature = "gui")]
use bevy::render::render_asset::RenderAssetUsages;
#[cfg(feature = "gui")]
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

use std::time::Duration;

#[cfg(feature = "sim")]
use pendulumd::controller::PdController;
#[cfg(all(feature = "hw", target_os = "linux"))]
use pendulumd::hw;
#[cfg(feature = "sim")]
use pendulumd::imu::Imu;
#[cfg(all(feature = "hw", target_os = "linux"))]
use pendulumd::imu::Imu;
#[cfg(feature = "sim")]
use pendulumd::sim;

#[cfg(all(feature = "hw", not(target_os = "linux")))]
compile_error!(
    "The `hw` feature is currently supported only on Linux targets (e.g. Raspberry Pi Linux)."
);

#[cfg(feature = "gui")]
const PENDULUM_LENGTH_PX: f32 = 220.0;
#[cfg(feature = "gui")]
const TRIANGLE_LEG_LENGTH_PX: f32 = PENDULUM_LENGTH_PX;
#[cfg(feature = "gui")]
const MAX_VISUAL_TILT_RAD: f32 = std::f32::consts::FRAC_PI_4;
#[cfg(feature = "gui")]
const MOTOR_RADIUS_PX: f32 = 18.0;
#[cfg(feature = "gui")]
const MOTOR_TEXTURE_SIZE_PX: u32 = 128;
#[cfg(not(feature = "sim"))]
const FIXED_DT_S: f64 = 0.01;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RunMode {
    Gui,
    Console,
}

impl RunMode {
    fn from_args() -> Self {
        let mut mode = if cfg!(feature = "gui") {
            Self::Gui
        } else {
            Self::Console
        };

        for arg in std::env::args().skip(1) {
            match arg.as_str() {
                "--gui" => mode = Self::Gui,
                "--console" | "--nogui" => mode = Self::Console,
                _ => {}
            }
        }

        mode
    }
}

fn main() {
    let mode = RunMode::from_args();

    match mode {
        RunMode::Gui => {
            #[cfg(feature = "gui")]
            {
                run_gui();
                return;
            }

            #[cfg(not(feature = "gui"))]
            {
                eprintln!(
                    "GUI mode requested, but this binary was built without the `gui` feature."
                );
                std::process::exit(2);
            }
        }
        RunMode::Console => run_console(),
    }
}

#[cfg(feature = "gui")]
fn run_gui() {
    #[cfg(feature = "sim")]
    let sim_config = sim::SimConfig::default();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::srgb(0.08, 0.08, 0.1)))
        .insert_resource(PendulumUiState::default())
        .add_systems(Startup, setup_scene)
        .add_systems(Update, render_pendulum_system);

    #[cfg(feature = "sim")]
    {
        app.insert_resource(SimBackend::new(sim_config))
            .add_systems(Update, update_from_sim_system);
    }

    #[cfg(all(feature = "hw", not(feature = "sim"), target_os = "linux"))]
    {
        match hw::Mpu6050Imu::new() {
            Ok(imu) => {
                app.insert_resource(HwBackend { imu })
                    .add_systems(Update, update_from_hw_system);
            }
            Err(error) => {
                eprintln!("Failed to initialize MPU-6050: {error:?}");
                return;
            }
        }
    }

    app.run();
}

fn run_console() {
    #[cfg(feature = "sim")]
    {
        run_console_sim(sim::SimConfig::default());
        return;
    }

    #[cfg(all(not(feature = "sim"), feature = "hw", target_os = "linux"))]
    {
        run_console_hw();
        return;
    }

    #[cfg(not(any(feature = "sim", all(feature = "hw", target_os = "linux"))))]
    eprintln!("No runnable backend is enabled. Build with `sim` or `hw`.");
}

#[cfg(feature = "gui")]
#[cfg_attr(feature = "gui", derive(Resource))]
#[derive(Debug, Clone, Copy)]
struct PendulumUiState {
    theta: f64,
    theta_dot: f64,
    wheel_angle: f64,
    torque_nm: f64,
}

#[cfg(feature = "gui")]
impl Default for PendulumUiState {
    fn default() -> Self {
        Self {
            theta: 0.0,
            theta_dot: 0.0,
            wheel_angle: 0.0,
            torque_nm: 0.0,
        }
    }
}

#[cfg(feature = "sim")]
#[derive(Debug, Clone, Copy)]
struct SimStepTelemetry {
    state: sim::PlantState,
    torque_cmd: f64,
    torque_applied: f64,
    available_torque: f64,
    speed_ratio: f64,
    at_bound: bool,
    sim_time_s: f64,
}

#[cfg(feature = "sim")]
#[cfg_attr(feature = "gui", derive(Resource))]
struct SimBackend {
    config: sim::SimConfig,
    plant: sim::SimPlant,
    imu: sim::SimImu,
    controller: PdController,
    sim_time_s: f64,
    #[cfg(feature = "gui")]
    dt_accum_s: f64,
}

#[cfg(feature = "sim")]
impl SimBackend {
    fn new(config: sim::SimConfig) -> Self {
        let plant = sim::SimPlant::new(config.plant_params(), config.initial_state());
        let mut imu = sim::SimImu::new();
        imu.sample_from_state(plant.state());
        let initial_state = plant.state();
        let effective_body_side_length_m = config.effective_body_side_length_m();
        eprintln!(
            "sim init: theta={:+.3} rad ({:+.1} deg), body_side={:.3} m, body_side_effective={:.3} m, body_mass={:.3} kg, wheel_r={:.3} m, wheel_m={:.3} kg, max_tau={:.3} Nm, no_load_speed={:.1} rad/s, kp={:.3}, kd={:.3}",
            initial_state.theta,
            initial_state.theta.to_degrees(),
            config.body_side_length_m,
            effective_body_side_length_m,
            config.body_mass_kg,
            config.wheel_radius_m,
            config.wheel_mass_kg,
            config.max_motor_torque_nm,
            config.motor_no_load_speed_rad_s,
            config.controller_kp,
            config.controller_kd,
        );
        Self {
            config,
            plant,
            imu,
            controller: PdController::new(config.controller_kp, config.controller_kd),
            sim_time_s: 0.0,
            #[cfg(feature = "gui")]
            dt_accum_s: 0.0,
        }
    }

    fn step_once(&mut self) -> SimStepTelemetry {
        let dt_s = self.config.dt_s;
        let wheel_speed = self.plant.state().wheel_speed;
        let speed_ratio =
            (wheel_speed.abs() / self.config.motor_no_load_speed_rad_s).clamp(0.0, 1.0);
        let available_torque = self.config.max_motor_torque_nm * (1.0 - speed_ratio);

        let sample = self.imu.read().expect("sim IMU should never fail");
        let torque_cmd = self
            .controller
            .torque_command(sample.theta, sample.theta_dot);
        let wheel_torque = torque_cmd.clamp(-available_torque, available_torque);

        self.plant.step(wheel_torque, Duration::from_secs_f64(dt_s));
        let state = self.plant.state();
        self.imu.sample_from_state(state);
        self.sim_time_s += dt_s;

        SimStepTelemetry {
            state,
            torque_cmd,
            torque_applied: wheel_torque,
            available_torque,
            speed_ratio,
            at_bound: state.theta.abs() >= (std::f64::consts::FRAC_PI_4 - 1e-6),
            sim_time_s: self.sim_time_s,
        }
    }
}

#[cfg(feature = "sim")]
fn log_sim_step(step_idx: u64, telemetry: SimStepTelemetry) {
    eprintln!(
        "sim step={:>5} t={:>6.2}s theta={:+.3} rad ({:+6.1} deg) theta_dot={:+6.3} rad/s wheel_speed={:+7.2} rad/s torque_cmd={:+6.3} Nm torque_applied={:+6.3} Nm avail={:+6.3} Nm speed_ratio={:.3} bound={}",
        step_idx,
        telemetry.sim_time_s,
        telemetry.state.theta,
        telemetry.state.theta.to_degrees(),
        telemetry.state.theta_dot,
        telemetry.state.wheel_speed,
        telemetry.torque_cmd,
        telemetry.torque_applied,
        telemetry.available_torque,
        telemetry.speed_ratio,
        telemetry.at_bound,
    );
}

#[cfg(feature = "sim")]
fn run_console_sim(config: sim::SimConfig) {
    let mut backend = SimBackend::new(config);
    let mut step_idx = 0_u64;
    let dt = Duration::from_secs_f64(backend.config.dt_s);

    loop {
        let telemetry = backend.step_once();
        step_idx += 1;
        log_sim_step(step_idx, telemetry);
        std::thread::sleep(dt);
    }
}

#[cfg(all(feature = "hw", target_os = "linux"))]
fn run_console_hw() {
    let mut imu = match hw::Mpu6050Imu::new() {
        Ok(imu) => imu,
        Err(error) => {
            eprintln!("Failed to initialize MPU-6050: {error:?}");
            return;
        }
    };

    loop {
        match imu.read() {
            Ok(sample) => {
                eprintln!(
                    "hw sample: theta={:+.3} rad ({:+6.1} deg) theta_dot={:+6.3} rad/s",
                    sample.theta,
                    sample.theta.to_degrees(),
                    sample.theta_dot,
                );
            }
            Err(error) => {
                eprintln!("MPU-6050 read failed: {error:?}");
                return;
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

#[cfg(all(feature = "hw", not(feature = "sim"), target_os = "linux"))]
#[cfg_attr(feature = "gui", derive(Resource))]
struct HwBackend {
    imu: hw::Mpu6050Imu,
}

#[cfg(feature = "gui")]
#[derive(Component)]
struct MotorDisk;

#[cfg(feature = "gui")]
fn setup_scene(mut commands: Commands<'_, '_>, mut images: ResMut<'_, Assets<Image>>) {
    commands.spawn(Camera2d);

    let motor_disk = images.add(make_motor_disk_image());
    commands.spawn((
        Sprite::from_image(motor_disk),
        Transform::default(),
        MotorDisk,
    ));
}

#[cfg(feature = "gui")]
fn make_motor_disk_image() -> Image {
    let extent = Extent3d {
        width: MOTOR_TEXTURE_SIZE_PX,
        height: MOTOR_TEXTURE_SIZE_PX,
        depth_or_array_layers: 1,
    };
    let radius = MOTOR_TEXTURE_SIZE_PX as f32 * 0.5;
    let center = radius - 0.5;
    let mut data = vec![0_u8; (MOTOR_TEXTURE_SIZE_PX * MOTOR_TEXTURE_SIZE_PX * 4) as usize];

    for y in 0..MOTOR_TEXTURE_SIZE_PX {
        for x in 0..MOTOR_TEXTURE_SIZE_PX {
            let dx = x as f32 - center;
            let dy = center - y as f32;
            let idx = ((y * MOTOR_TEXTURE_SIZE_PX + x) * 4) as usize;

            if dx * dx + dy * dy > radius * radius {
                data[idx + 3] = 0;
                continue;
            }

            let white = (dx >= 0.0 && dy >= 0.0) || (dx < 0.0 && dy < 0.0);
            let value = if white { 242 } else { 13 };
            data[idx] = value;
            data[idx + 1] = value;
            data[idx + 2] = value;
            data[idx + 3] = 255;
        }
    }

    Image::new(
        extent,
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

#[cfg(all(feature = "sim", feature = "gui"))]
fn update_from_sim_system(
    time: Res<'_, Time>,
    mut sim_backend: ResMut<'_, SimBackend>,
    mut ui_state: ResMut<'_, PendulumUiState>,
    mut step_idx: Local<'_, u64>,
    mut print_accum_s: Local<'_, f64>,
) {
    let dt_s = sim_backend.config.dt_s;
    let frame_dt_s = time.delta_secs_f64().min(0.05);
    sim_backend.dt_accum_s += frame_dt_s;

    let mut last_telemetry: Option<SimStepTelemetry> = None;
    let mut substeps = 0;
    while sim_backend.dt_accum_s >= dt_s && substeps < 4 {
        let telemetry = sim_backend.step_once();
        sim_backend.dt_accum_s -= dt_s;
        last_telemetry = Some(telemetry);
        *step_idx += 1;
        *print_accum_s += dt_s;
        substeps += 1;
    }

    let state = sim_backend.plant.state();
    ui_state.theta = state.theta;
    ui_state.theta_dot = state.theta_dot;
    ui_state.wheel_angle = state.wheel_angle;
    ui_state.torque_nm = last_telemetry
        .map(|telemetry| telemetry.torque_applied)
        .unwrap_or(0.0);

    if substeps > 0 && *print_accum_s >= 0.1 {
        *print_accum_s = 0.0;
        if let Some(telemetry) = last_telemetry {
            log_sim_step(*step_idx, telemetry);
        }
    }
}

#[cfg(all(
    feature = "hw",
    feature = "gui",
    not(feature = "sim"),
    target_os = "linux"
))]
fn update_from_hw_system(
    mut hw_backend: ResMut<'_, HwBackend>,
    mut ui_state: ResMut<'_, PendulumUiState>,
) {
    match hw_backend.imu.read() {
        Ok(sample) => {
            ui_state.theta = sample.theta;
            ui_state.theta_dot = sample.theta_dot;
            ui_state.torque_nm = 0.0;
        }
        Err(error) => {
            eprintln!("MPU-6050 read failed: {error:?}");
        }
    }
}

#[cfg(feature = "gui")]
fn render_pendulum_system(
    ui_state: Res<'_, PendulumUiState>,
    #[cfg(feature = "sim")] sim_backend: Option<Res<'_, SimBackend>>,
    mut motor_query: Query<'_, '_, (&mut Transform, &mut Sprite), With<MotorDisk>>,
    mut gizmos: Gizmos<'_, '_>,
) {
    let theta = (ui_state.theta as f32).clamp(-MAX_VISUAL_TILT_RAD, MAX_VISUAL_TILT_RAD);
    let wheel_angle = ui_state.wheel_angle as f32;

    #[cfg(feature = "sim")]
    let triangle_leg_length_px = sim_backend
        .as_ref()
        .map(|backend| {
            backend.config.effective_body_side_length_m() as f32 * backend.config.pixels_per_meter
        })
        .unwrap_or(TRIANGLE_LEG_LENGTH_PX);
    #[cfg(not(feature = "sim"))]
    let triangle_leg_length_px = TRIANGLE_LEG_LENGTH_PX;

    #[cfg(feature = "sim")]
    let motor_radius_px = sim_backend
        .as_ref()
        .map(|backend| backend.config.wheel_radius_m as f32 * backend.config.pixels_per_meter)
        .unwrap_or(MOTOR_RADIUS_PX);
    #[cfg(not(feature = "sim"))]
    let motor_radius_px = MOTOR_RADIUS_PX;

    let leg_half_axis = triangle_leg_length_px / std::f32::consts::SQRT_2;
    let pivot = Vec2::ZERO;
    let p1_local = Vec2::new(-leg_half_axis, leg_half_axis);
    let p2_local = Vec2::new(leg_half_axis, leg_half_axis);

    let rotation = Mat2::from_angle(-theta);
    let p1 = pivot + rotation * p1_local;
    let p2 = pivot + rotation * p2_local;
    let com_local = (p1_local + p2_local) / 3.0;
    let com = pivot + rotation * com_local;

    let body_color = Color::srgb(0.85, 0.85, 0.88);
    let pivot_color = Color::srgb(0.95, 0.35, 0.25);
    let motor_outline = Color::srgb(0.7, 0.7, 0.75);

    gizmos.line_2d(pivot, p1, body_color);
    gizmos.line_2d(p1, p2, body_color);
    gizmos.line_2d(p2, pivot, body_color);

    gizmos.circle_2d(pivot, 5.0, pivot_color);

    let motor_world_angle = -theta + wheel_angle;
    for (mut transform, mut sprite) in &mut motor_query {
        transform.translation = Vec3::new(com.x, com.y, 1.0);
        transform.rotation = Quat::from_rotation_z(motor_world_angle);
        sprite.custom_size = Some(Vec2::splat(motor_radius_px * 2.0));
    }

    gizmos.circle_2d(com, motor_radius_px, motor_outline);
}
