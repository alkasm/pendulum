#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;
#[path = "../hw/mod.rs"]
mod hw;

use core::fmt::Write;

use bringup::{HALL_SENSOR_ADDR, init_console, init_delay, max_clock_config, write_line};
use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    mcpwm::{
        McPwm, PeripheralClockConfig,
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
        timer::PwmWorkingMode,
    },
    main,
    peripherals::{GPIO5, GPIO34, MCPWM0},
    time::Rate,
};
use hw::{CurrentSensor, Tmag5273};
use libm::sinf;

const BASELINE_SAMPLES: u32 = 64;
const CONTROL_PERIOD_MS: u32 = 5;
const PRINT_EVERY_LOOPS: u32 = 20;

const PWM_FREQUENCY_HZ: u32 = 32_000;
const PWM_PERIOD_TICKS: u16 = 2500;
const DEAD_ZONE: f32 = 0.02;

const VOLTAGE_POWER_SUPPLY_V: f32 = 5.0;
const VOLTAGE_LIMIT_V: f32 = 1.8;
const TARGET_VELOCITY_RAD_S: f32 = 25.0;
const MOTOR_POLE_PAIRS: f32 = 7.0;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

type PwmPinA<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, true>;
type PwmPinB<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, false>;

struct PwmMotorDrive<'a> {
    enable: Output<'a>,
    diag: Input<'a>,
    uh: PwmPinA<'a, 0>,
    ul: PwmPinB<'a, 0>,
    vh: PwmPinA<'a, 1>,
    vl: PwmPinB<'a, 1>,
    wh: PwmPinA<'a, 2>,
    wl: PwmPinB<'a, 2>,
}

struct OpenLoopState {
    shaft_angle_rad: f32,
}

struct WheelObserver {
    last_raw_angle_deg: Option<f32>,
    filtered_speed_dps: f32,
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();

    write_line(&mut serial, "motor_open_loop gentle start");

    let mut current_sensor = CurrentSensor::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
    );
    let mut hall = Tmag5273::new(
        peripherals.I2C0,
        peripherals.GPIO21,
        peripherals.GPIO22,
        HALL_SENSOR_ADDR,
    );

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(160))
        .expect("failed to configure MCPWM clock");
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm.operator2.set_timer(&mcpwm.timer0);

    let (uh, ul) = mcpwm.operator0.with_pins(
        peripherals.GPIO16,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO17,
        low_side_pwm_config(),
    );
    let (vh, vl) = mcpwm.operator1.with_pins(
        peripherals.GPIO18,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO23,
        low_side_pwm_config(),
    );
    let (wh, wl) = mcpwm.operator2.with_pins(
        peripherals.GPIO19,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO33,
        low_side_pwm_config(),
    );

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            PWM_PERIOD_TICKS,
            PwmWorkingMode::UpDown,
            Rate::from_hz(PWM_FREQUENCY_HZ),
        )
        .expect("failed to configure MCPWM timer");
    mcpwm.timer0.start(timer_clock_cfg);

    let mut motor_drive = PwmMotorDrive::new(
        peripherals.GPIO5,
        peripherals.GPIO34,
        uh,
        ul,
        vh,
        vl,
        wh,
        wl,
    );

    current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    let hall_available = configure_hall_best_effort(&mut hall);

    motor_drive.disable();
    motor_drive.coast();

    let mut open_loop = OpenLoopState {
        shaft_angle_rad: 0.0,
    };
    let mut wheel = WheelObserver::new();
    let mut print_divider = 0_u32;

    let _ = writeln!(
        serial,
        "vbus={:.1}V vlimit={:.1}V vel={:.1}rad/s pole_pairs={} pwm={}Hz dead_zone={:.3}\r",
        VOLTAGE_POWER_SUPPLY_V,
        VOLTAGE_LIMIT_V,
        TARGET_VELOCITY_RAD_S,
        MOTOR_POLE_PAIRS as u32,
        PWM_FREQUENCY_HZ,
        DEAD_ZONE
    );
    let _ = serial.flush();

    delay.delay_millis(1_000);
    motor_drive.enable();
    write_line(&mut serial, "motor_open_loop enabled");

    loop {
        open_loop.shaft_angle_rad =
            wrap_angle_rad(open_loop.shaft_angle_rad + TARGET_VELOCITY_RAD_S * dt_s());
        let electrical_angle_rad = open_loop.shaft_angle_rad * MOTOR_POLE_PAIRS;
        let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
            VOLTAGE_LIMIT_V,
            electrical_angle_rad,
            VOLTAGE_LIMIT_V,
        );
        motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);

        let current = current_sensor.read();
        let (hall_label, hall_angle_deg, wheel_speed_dps) = if hall_available {
            if let Ok(measurement) = hall.read_measurement() {
                (
                    "ok",
                    measurement.angle_deg,
                    wheel.observe(measurement.angle_deg),
                )
            } else {
                ("err", 0.0, 0.0)
            }
        } else {
            ("missing", 0.0, 0.0)
        };

        if print_divider == 0 {
            let _ = writeln!(
                serial,
                concat!(
                    "shaft={:>6.2}rad elec={:>7.2}deg uvw=[{:>4.2},{:>4.2},{:>4.2}]V ",
                    "hall={} angle={:>7.2}deg wheel_dot={:+7.2}dps diag={} ",
                    "iabc=[{:+5.2},{:+5.2},{:+5.2}]A\r"
                ),
                open_loop.shaft_angle_rad,
                wrap_degrees(electrical_angle_rad * (180.0 / core::f32::consts::PI)),
                ua_v,
                ub_v,
                uc_v,
                hall_label,
                hall_angle_deg,
                wheel_speed_dps,
                if motor_drive.diag_is_high() { "high" } else { "low" },
                current.ina_u.amps,
                current.ina_v.amps,
                current.ina_w.amps,
            );
            let _ = serial.flush();
        }

        print_divider = (print_divider + 1) % PRINT_EVERY_LOOPS;
        delay.delay_millis(CONTROL_PERIOD_MS);
    }
}

impl<'a> PwmMotorDrive<'a> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        enable: GPIO5<'a>,
        diag: GPIO34<'a>,
        uh: PwmPinA<'a, 0>,
        ul: PwmPinB<'a, 0>,
        vh: PwmPinA<'a, 1>,
        vl: PwmPinB<'a, 1>,
        wh: PwmPinA<'a, 2>,
        wl: PwmPinB<'a, 2>,
    ) -> Self {
        Self {
            enable: Output::new(enable, Level::Low, OutputConfig::default()),
            diag: Input::new(diag, InputConfig::default()),
            uh,
            ul,
            vh,
            vl,
            wh,
            wl,
        }
    }

    fn enable(&mut self) {
        self.enable.set_high();
    }

    fn disable(&mut self) {
        self.enable.set_low();
    }

    fn diag_is_high(&self) -> bool {
        self.diag.is_high()
    }

    fn coast(&mut self) {
        self.uh.set_timestamp(0);
        self.vh.set_timestamp(0);
        self.wh.set_timestamp(0);
        self.ul.set_timestamp(PWM_PERIOD_TICKS);
        self.vl.set_timestamp(PWM_PERIOD_TICKS);
        self.wl.set_timestamp(PWM_PERIOD_TICKS);
    }

    fn set_phase_voltages(&mut self, ua_v: f32, ub_v: f32, uc_v: f32) {
        let dead = DEAD_ZONE * 0.5;
        let dc_a = clamp(ua_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_b = clamp(ub_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_c = clamp(uc_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);

        self.uh.set_timestamp(duty_to_ticks(dc_a - dead));
        self.ul.set_timestamp(duty_to_ticks(dc_a + dead));
        self.vh.set_timestamp(duty_to_ticks(dc_b - dead));
        self.vl.set_timestamp(duty_to_ticks(dc_b + dead));
        self.wh.set_timestamp(duty_to_ticks(dc_c - dead));
        self.wl.set_timestamp(duty_to_ticks(dc_c + dead));
    }
}

impl WheelObserver {
    fn new() -> Self {
        Self {
            last_raw_angle_deg: None,
            filtered_speed_dps: 0.0,
        }
    }

    fn observe(&mut self, raw_angle_deg: f32) -> f32 {
        let delta_deg = self
            .last_raw_angle_deg
            .map(|previous| wrap_angle_delta_deg(raw_angle_deg - previous))
            .unwrap_or(0.0);
        let instant_speed_dps = delta_deg / dt_s();
        self.filtered_speed_dps = 0.85 * self.filtered_speed_dps + 0.15 * instant_speed_dps;
        self.last_raw_angle_deg = Some(raw_angle_deg);
        self.filtered_speed_dps
    }
}

fn low_side_pwm_config() -> PwmPinConfig<false> {
    PwmPinConfig::new(
        PwmActions::<false>::empty()
            .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
            .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
        PwmUpdateMethod::SYNC_ON_ZERO,
    )
}

fn configure_hall_best_effort(hall: &mut Tmag5273<'_>) -> bool {
    if !hall.is_present() {
        return false;
    }
    hall.configure_default().is_ok()
}

fn simplefoc_sine_pwm_phase_voltages(
    uq_v: f32,
    angle_el_rad: f32,
    voltage_limit_v: f32,
) -> (f32, f32, f32) {
    let ualpha = -sinf(angle_el_rad) * uq_v;
    let ubeta = sinf(angle_el_rad + core::f32::consts::FRAC_PI_2) * uq_v;

    let mut ua = ualpha;
    let mut ub = -0.5 * ualpha + 0.866_025_4 * ubeta;
    let mut uc = -0.5 * ualpha - 0.866_025_4 * ubeta;

    let center = voltage_limit_v * 0.5;
    ua += center;
    ub += center;
    uc += center;

    (
        clamp(ua, 0.0, voltage_limit_v),
        clamp(ub, 0.0, voltage_limit_v),
        clamp(uc, 0.0, voltage_limit_v),
    )
}

fn duty_to_ticks(duty: f32) -> u16 {
    let clamped = clamp(duty, 0.0, 1.0);
    (clamped * PWM_PERIOD_TICKS as f32 + 0.5) as u16
}

fn dt_s() -> f32 {
    CONTROL_PERIOD_MS as f32 / 1_000.0
}

fn wrap_angle_rad(angle_rad: f32) -> f32 {
    let two_pi = 2.0 * core::f32::consts::PI;
    let mut wrapped = angle_rad;
    while wrapped >= two_pi {
        wrapped -= two_pi;
    }
    while wrapped < 0.0 {
        wrapped += two_pi;
    }
    wrapped
}

fn wrap_degrees(angle_deg: f32) -> f32 {
    let mut wrapped = angle_deg;
    while wrapped >= 360.0 {
        wrapped -= 360.0;
    }
    while wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
}

fn wrap_angle_delta_deg(delta: f32) -> f32 {
    if delta > 180.0 {
        delta - 360.0
    } else if delta < -180.0 {
        delta + 360.0
    } else {
        delta
    }
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
