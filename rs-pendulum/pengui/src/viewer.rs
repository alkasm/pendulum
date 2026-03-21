use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use bevy::render::render_asset::RenderAssetUsages;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

use pendulum_lib::{
    config::VisualizationConfig,
    telemetry::{self, TelemetryFrame, TelemetryReceiver},
};

const MAX_VISUAL_TILT_RAD: f32 = std::f32::consts::FRAC_PI_4;
const MOTOR_TEXTURE_SIZE_PX: u32 = 128;

#[derive(Resource)]
struct GuiTelemetryReceiver {
    pending_connection: Arc<Mutex<Option<TelemetryReceiver>>>,
    receiver: Option<TelemetryReceiver>,
}

#[derive(Resource, Debug, Clone, Copy)]
struct VisualConfigResource {
    visual_config: VisualizationConfig,
}

#[derive(Resource, Debug, Clone, Copy)]
struct PendulumUiState {
    theta: f64,
    theta_dot: f64,
    wheel_angle: f64,
    torque_nm: f64,
}

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

impl PendulumUiState {
    fn apply_frame(&mut self, frame: TelemetryFrame) {
        self.theta = frame.theta_rad;
        self.theta_dot = frame.theta_dot_rad_s;
        self.wheel_angle = frame.wheel_angle_rad;
        self.torque_nm = frame.applied_torque_nm;
    }
}

#[derive(Component)]
struct MotorDisk;

pub fn run(pending_connection: Arc<Mutex<Option<TelemetryReceiver>>>, visual_config: VisualizationConfig) {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::srgb(0.08, 0.08, 0.1)))
        .insert_resource(PendulumUiState::default())
        .insert_resource(GuiTelemetryReceiver {
            pending_connection,
            receiver: None,
        })
        .insert_resource(VisualConfigResource { visual_config })
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                update_connection_system,
                poll_telemetry_system,
                render_pendulum_system,
            ),
        );

    app.run();
}

fn setup_scene(mut commands: Commands<'_, '_>, mut images: ResMut<'_, Assets<Image>>) {
    commands.spawn(Camera2d);

    let motor_disk = images.add(make_motor_disk_image());
    commands.spawn((
        Sprite::from_image(motor_disk),
        Transform::default(),
        MotorDisk,
    ));
}

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

fn update_connection_system(mut telemetry_receiver: ResMut<'_, GuiTelemetryReceiver>) {
    if telemetry_receiver.receiver.is_some() {
        return;
    }

    let maybe_receiver = telemetry_receiver
        .pending_connection
        .lock()
        .expect("GUI pending connection mutex poisoned")
        .take();

    if let Some(receiver) = maybe_receiver {
        println!("GUI telemetry connected.");
        telemetry_receiver.receiver = Some(receiver);
    }
}

fn poll_telemetry_system(
    mut telemetry_receiver: ResMut<'_, GuiTelemetryReceiver>,
    mut ui_state: ResMut<'_, PendulumUiState>,
) {
    let Some(receiver) = telemetry_receiver.receiver.as_mut() else {
        return;
    };

    if let Some(frame) = telemetry::drain_latest(receiver) {
        ui_state.apply_frame(frame);
    }
}

fn render_pendulum_system(
    ui_state: Res<'_, PendulumUiState>,
    visual_config: Res<'_, VisualConfigResource>,
    mut motor_query: Query<'_, '_, (&mut Transform, &mut Sprite), With<MotorDisk>>,
    mut gizmos: Gizmos<'_, '_>,
) {
    let theta = (ui_state.theta as f32).clamp(-MAX_VISUAL_TILT_RAD, MAX_VISUAL_TILT_RAD);
    let wheel_angle = ui_state.wheel_angle as f32;
    let triangle_leg_length_px = visual_config.visual_config.triangle_leg_length_px();
    let motor_radius_px = visual_config.visual_config.motor_radius_px();

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
