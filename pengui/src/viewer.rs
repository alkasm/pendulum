use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use bevy::render::render_asset::RenderAssetUsages;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

use pendulum_lib::{
    telemetry::{self, TelemetryFrame, TelemetryReceiver},
};
use uom::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    electric_current::ampere,
    time::second,
    torque::newton_meter,
};

use crate::config::VisualizationConfig;

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
    connected: bool,
    step: u64,
    sim_time: f64,
    theta: f64,
    theta_dot: f64,
    wheel_angle: f64,
    wheel_speed: f64,
    commanded_torque: f64,
    torque: f64,
    available_torque: f64,
    phase_current: f64,
    speed_ratio: f64,
}

impl Default for PendulumUiState {
    fn default() -> Self {
        Self {
            connected: false,
            step: 0,
            sim_time: 0.0,
            theta: 0.0,
            theta_dot: 0.0,
            wheel_angle: 0.0,
            wheel_speed: 0.0,
            commanded_torque: 0.0,
            torque: 0.0,
            available_torque: 0.0,
            phase_current: 0.0,
            speed_ratio: 0.0,
        }
    }
}

impl PendulumUiState {
    fn apply_frame(&mut self, frame: TelemetryFrame) {
        self.connected = true;
        self.step = frame.step;
        self.sim_time = frame.sim_time.get::<second>();
        self.theta = frame.theta.get::<radian>();
        self.theta_dot = frame.theta_dot.get::<radian_per_second>();
        self.wheel_angle = frame.wheel_angle.get::<radian>();
        self.wheel_speed = frame.wheel_speed.get::<radian_per_second>();
        self.commanded_torque = frame.commanded_torque.get::<newton_meter>();
        self.torque = frame.applied_torque.get::<newton_meter>();
        self.available_torque = frame.available_torque.get::<newton_meter>();
        self.phase_current = frame.phase_current.get::<ampere>();
        self.speed_ratio = frame.speed_ratio;
    }
}

#[derive(Component)]
struct MotorDisk;

#[derive(Component)]
struct TelemetryStatusText;

#[derive(Component, Clone, Copy)]
enum TelemetryValueKind {
    Step,
    SimTime,
    ThetaRad,
    ThetaDeg,
    ThetaDot,
    WheelAngle,
    WheelSpeed,
    CommandedTorque,
    AppliedTorque,
    AvailableTorque,
    PhaseCurrent,
    SpeedRatio,
}

#[derive(Component)]
struct TelemetryValueText {
    kind: TelemetryValueKind,
}

pub fn run(
    pending_connection: Arc<Mutex<Option<TelemetryReceiver>>>,
    visual_config: VisualizationConfig,
) {
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
                update_telemetry_panel_system,
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

    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(0.0),
                top: Val::Px(0.0),
                bottom: Val::Px(0.0),
                width: Val::Px(320.0),
                padding: UiRect::axes(Val::Px(16.0), Val::Px(14.0)),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(10.0),
                ..default()
            },
            BackgroundColor(Color::srgba(0.06, 0.07, 0.09, 0.92)),
        ))
        .with_children(|parent| {
            parent.spawn((
                Text::new("Telemetry"),
                TextFont {
                    font_size: 20.0,
                    ..default()
                },
                TextColor(Color::srgb(0.92, 0.93, 0.95)),
            ));

            parent.spawn((
                Text::new("Waiting for telemetry..."),
                TextFont {
                    font_size: 15.0,
                    ..default()
                },
                TextColor(Color::srgb(0.62, 0.74, 0.88)),
                TelemetryStatusText,
            ));

            spawn_telemetry_grid(parent);
        });
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

fn update_telemetry_panel_system(
    ui_state: Res<'_, PendulumUiState>,
    mut text_queries: ParamSet<
        '_,
        '_,
        (
            Query<'_, '_, &mut Text, With<TelemetryStatusText>>,
            Query<'_, '_, (&TelemetryValueText, &mut Text)>,
        ),
    >,
) {
    if !ui_state.is_changed() {
        return;
    }

    for mut text in &mut text_queries.p0() {
        text.0 = if ui_state.connected {
            "Connected".to_string()
        } else {
            "Waiting for telemetry...".to_string()
        };
    }

    for (value_kind, mut text) in &mut text_queries.p1() {
        text.0 = format_telemetry_value(value_kind.kind, &ui_state);
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

fn spawn_telemetry_grid(parent: &mut ChildBuilder) {
    parent
        .spawn(Node {
            width: Val::Percent(100.0),
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(6.0),
            ..default()
        })
        .with_children(|parent| {
            spawn_telemetry_row(parent, "Step", TelemetryValueKind::Step);
            spawn_telemetry_row(parent, "Time", TelemetryValueKind::SimTime);
            spawn_telemetry_row(parent, "Theta", TelemetryValueKind::ThetaRad);
            spawn_telemetry_row(parent, "Theta Deg", TelemetryValueKind::ThetaDeg);
            spawn_telemetry_row(parent, "Theta Dot", TelemetryValueKind::ThetaDot);
            spawn_telemetry_row(parent, "Wheel Angle", TelemetryValueKind::WheelAngle);
            spawn_telemetry_row(parent, "Wheel Speed", TelemetryValueKind::WheelSpeed);
            spawn_telemetry_row(parent, "Tau Cmd", TelemetryValueKind::CommandedTorque);
            spawn_telemetry_row(parent, "Tau", TelemetryValueKind::AppliedTorque);
            spawn_telemetry_row(parent, "Tau Avail", TelemetryValueKind::AvailableTorque);
            spawn_telemetry_row(parent, "Current", TelemetryValueKind::PhaseCurrent);
            spawn_telemetry_row(parent, "Speed Ratio", TelemetryValueKind::SpeedRatio);
        });
}

fn spawn_telemetry_row(parent: &mut ChildBuilder, label: &'static str, kind: TelemetryValueKind) {
    parent
        .spawn(Node {
            width: Val::Percent(100.0),
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            ..default()
        })
        .with_children(|parent| {
            parent.spawn((
                Text::new(label),
                TextFont {
                    font_size: 14.0,
                    ..default()
                },
                TextColor(Color::srgb(0.72, 0.77, 0.84)),
            ));

            parent.spawn((
                Text::new(format_telemetry_value(kind, &PendulumUiState::default())),
                TextFont {
                    font_size: 14.0,
                    ..default()
                },
                TextColor(Color::srgb(0.95, 0.96, 0.98)),
                TelemetryValueText { kind },
            ));
        });
}

fn format_telemetry_value(kind: TelemetryValueKind, ui_state: &PendulumUiState) -> String {
    match kind {
        TelemetryValueKind::Step => ui_state.step.to_string(),
        TelemetryValueKind::SimTime => format!("{:.2} s", ui_state.sim_time),
        TelemetryValueKind::ThetaRad => format!("{:+.3} rad", ui_state.theta),
        TelemetryValueKind::ThetaDeg => format!("{:+.1} deg", ui_state.theta.to_degrees()),
        TelemetryValueKind::ThetaDot => format!("{:+.3} rad/s", ui_state.theta_dot),
        TelemetryValueKind::WheelAngle => format!("{:+.3} rad", ui_state.wheel_angle),
        TelemetryValueKind::WheelSpeed => format!("{:+.2} rad/s", ui_state.wheel_speed),
        TelemetryValueKind::CommandedTorque => format!("{:+.3} Nm", ui_state.commanded_torque),
        TelemetryValueKind::AppliedTorque => format!("{:+.3} Nm", ui_state.torque),
        TelemetryValueKind::AvailableTorque => format!("{:+.3} Nm", ui_state.available_torque),
        TelemetryValueKind::PhaseCurrent => format!("{:+.3} A", ui_state.phase_current),
        TelemetryValueKind::SpeedRatio => format!("{:.3}", ui_state.speed_ratio),
    }
}
