#![no_std]
#![no_main]

//! Hall-locked sinusoidal commutation bring-up binary.
//!
//! This firmware:
//! - performs a slow open-loop electrical sweep to calibrate hall angle to electrical angle
//! - searches a small set of torque-phase offsets to find the one that produces motion
//! - then runs sinusoidal commutation from the measured hall angle instead of an open-loop phase ramp

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../board_init.rs"]
mod board_init;
#[path = "../hw/mod.rs"]
mod hw;

use core::fmt::Write;

use board_init::{HALL_SENSOR_ADDR, init_console, init_delay, max_clock_config, write_line};
use esp_hal::{
    Blocking,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    main,
    mcpwm::{
        McPwm, PeripheralClockConfig,
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
        timer::PwmWorkingMode,
    },
    peripherals::{GPIO5, GPIO34, MCPWM0},
    time::Rate,
    uart::Uart,
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
const VOLTAGE_LIMIT_V: f32 = 1.6;
const TARGET_WHEEL_SPEED_DPS: f32 = -360.0;
const SPEED_FEEDFORWARD_V: f32 = 0.40;
const SPEED_KP_V_PER_DPS: f32 = 0.003;

const MOTOR_POLE_PAIRS: f32 = 7.0;
const CALIBRATION_VOLTAGE_V: f32 = 1.2;
const CALIBRATION_WHEEL_SPEED_DPS: f32 = -180.0;
const CALIBRATION_TOTAL_LOOPS: u32 = 800;
const CALIBRATION_SETTLE_LOOPS: u32 = 120;
const MIN_CALIBRATION_HALL_TRAVEL_DEG: f32 = 180.0;
const PHASE_SEARCH_UQ_V: f32 = 0.9;
const PHASE_SEARCH_LOOPS: u32 = 140;
const PHASE_SEARCH_SETTLE_LOOPS: u32 = 30;
const PHASE_SEARCH_OFFSETS_DEG: [f32; 4] = [0.0, 90.0, 180.0, 270.0];

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

struct WheelObserver {
    last_raw_angle_deg: Option<f32>,
    unwrapped_angle_deg: Option<f32>,
    filtered_speed_dps: f32,
}

struct WheelObservation {
    unwrapped_angle_deg: f32,
    speed_dps: f32,
}

#[derive(Clone, Copy)]
struct HallElectricalCalibration {
    direction_sign: f32,
    electrical_offset_deg: f32,
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();

    write_line(&mut serial, "motor_hall_lock start");

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

    let _ = writeln!(
        serial,
        concat!(
            "target={:+.1}dps vlimit={:.1}V kp={:.4}V/dps ff={:.2}V ",
            "pwm={}Hz dead_zone={:.3}\r"
        ),
        TARGET_WHEEL_SPEED_DPS,
        VOLTAGE_LIMIT_V,
        SPEED_KP_V_PER_DPS,
        SPEED_FEEDFORWARD_V,
        PWM_FREQUENCY_HZ,
        DEAD_ZONE
    );
    let _ = serial.flush();

    if !hall_available {
        write_line(&mut serial, "hall missing; idling");
        loop {
            delay.delay_millis(1_000);
        }
    }

    let mut wheel = WheelObserver::new();
    write_line(&mut serial, "motor_hall_lock enabling in 1s");
    delay.delay_millis(1_000);

    let calibration = match calibrate_hall_electrical_cycle(
        &mut serial,
        &delay,
        &mut motor_drive,
        &mut hall,
        &mut wheel,
    ) {
        Some(calibration) => calibration,
        None => {
            motor_drive.disable();
            motor_drive.coast();
            write_line(&mut serial, "hall calibration failed; idling");
            loop {
                delay.delay_millis(1_000);
            }
        }
    };
    let calibration = refine_torque_phase_offset(
        &mut serial,
        &delay,
        &mut motor_drive,
        &mut hall,
        &mut wheel,
        calibration,
    )
    .unwrap_or(calibration);

    let _ = writeln!(
        serial,
        "cal done dir={:+.0} offset={:.2}deg\r",
        calibration.direction_sign, calibration.electrical_offset_deg,
    );
    let _ = serial.flush();

    motor_drive.enable();
    let mut print_divider = 0_u32;

    loop {
        let current = current_sensor.read();

        if let Ok(measurement) = hall.read_measurement() {
            let observation = wheel.observe(measurement.angle_deg);
            let electrical_angle_deg = calibration.electrical_angle_deg(measurement.angle_deg);
            let speed_error_dps = TARGET_WHEEL_SPEED_DPS - observation.speed_dps;
            // The calibration sweep established the hall->electrical angle mapping.
            // Torque polarity is a separate sign convention, and on this hardware
            // it is opposite the original guess.
            let uq_v = clamp(
                -(SPEED_FEEDFORWARD_V * signum_nonzero(TARGET_WHEEL_SPEED_DPS)
                    + SPEED_KP_V_PER_DPS * speed_error_dps),
                -VOLTAGE_LIMIT_V,
                VOLTAGE_LIMIT_V,
            );
            let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
                uq_v,
                degrees_to_radians(electrical_angle_deg),
                VOLTAGE_LIMIT_V,
            );
            motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);

            if print_divider == 0 {
                let _ = writeln!(
                    serial,
                    concat!(
                        "hall={:>7.2}deg unwrap={:>8.2}deg elec={:>7.2}deg ",
                        "wheel_dot={:+7.2}dps err={:+7.2}dps uq={:+4.2}V uvw=[{:>4.2},{:>4.2},{:>4.2}]V ",
                        "diag={} iabc=[{:+5.2},{:+5.2},{:+5.2}]A\r"
                    ),
                    measurement.angle_deg,
                    observation.unwrapped_angle_deg,
                    electrical_angle_deg,
                    observation.speed_dps,
                    speed_error_dps,
                    uq_v,
                    ua_v,
                    ub_v,
                    uc_v,
                    if motor_drive.diag_is_high() {
                        "high"
                    } else {
                        "low"
                    },
                    current.ina_u.amps,
                    current.ina_v.amps,
                    current.ina_w.amps,
                );
                let _ = serial.flush();
            }
        } else {
            motor_drive.coast();
            if print_divider == 0 {
                let _ = writeln!(
                    serial,
                    "hall=missing diag={} iabc=[{:+5.2},{:+5.2},{:+5.2}]A\r",
                    if motor_drive.diag_is_high() {
                        "high"
                    } else {
                        "low"
                    },
                    current.ina_u.amps,
                    current.ina_v.amps,
                    current.ina_w.amps,
                );
                let _ = serial.flush();
            }
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
            unwrapped_angle_deg: None,
            filtered_speed_dps: 0.0,
        }
    }

    fn observe(&mut self, raw_angle_deg: f32) -> WheelObservation {
        let unwrapped_angle_deg = match self.unwrapped_angle_deg {
            Some(previous) => unwrap_near(previous, raw_angle_deg),
            None => raw_angle_deg,
        };
        let delta_deg = self
            .last_raw_angle_deg
            .map(|previous| wrap_angle_delta_deg(raw_angle_deg - previous))
            .unwrap_or(0.0);
        let instant_speed_dps = delta_deg / dt_s();
        self.filtered_speed_dps = 0.85 * self.filtered_speed_dps + 0.15 * instant_speed_dps;
        self.last_raw_angle_deg = Some(raw_angle_deg);
        self.unwrapped_angle_deg = Some(unwrapped_angle_deg);
        WheelObservation {
            unwrapped_angle_deg,
            speed_dps: self.filtered_speed_dps,
        }
    }
}

impl HallElectricalCalibration {
    fn electrical_angle_deg(&self, hall_angle_deg: f32) -> f32 {
        wrap_degrees(
            self.direction_sign * MOTOR_POLE_PAIRS * hall_angle_deg + self.electrical_offset_deg,
        )
    }
}

fn calibrate_hall_electrical_cycle(
    serial: &mut Uart<'_, Blocking>,
    delay: &esp_hal::delay::Delay,
    motor_drive: &mut PwmMotorDrive<'_>,
    hall: &mut Tmag5273<'_>,
    wheel: &mut WheelObserver,
) -> Option<HallElectricalCalibration> {
    write_line(serial, "hall calibration begin");
    motor_drive.enable();

    let mut open_loop_shaft_deg = 0.0_f32;
    let mut start_hall_unwrapped_deg = None;
    let mut end_hall_unwrapped_deg = None;
    let mut start_electrical_unwrapped_deg = None;
    let mut end_electrical_unwrapped_deg = None;
    let mut pos_offset_sin_sum = 0.0_f32;
    let mut pos_offset_cos_sum = 0.0_f32;
    let mut neg_offset_sin_sum = 0.0_f32;
    let mut neg_offset_cos_sum = 0.0_f32;
    let mut sample_count = 0_u32;

    let mut loop_index = 0_u32;
    while loop_index < CALIBRATION_TOTAL_LOOPS {
        open_loop_shaft_deg = open_loop_shaft_deg + CALIBRATION_WHEEL_SPEED_DPS * dt_s();
        let electrical_unwrapped_deg = MOTOR_POLE_PAIRS * open_loop_shaft_deg;
        let electrical_angle_deg = wrap_degrees(electrical_unwrapped_deg);
        let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
            CALIBRATION_VOLTAGE_V,
            degrees_to_radians(electrical_angle_deg),
            VOLTAGE_LIMIT_V,
        );
        motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);

        if let Ok(measurement) = hall.read_measurement() {
            let observation = wheel.observe(measurement.angle_deg);
            if loop_index >= CALIBRATION_SETTLE_LOOPS {
                if start_hall_unwrapped_deg.is_none() {
                    start_hall_unwrapped_deg = Some(observation.unwrapped_angle_deg);
                    start_electrical_unwrapped_deg = Some(electrical_unwrapped_deg);
                }
                end_hall_unwrapped_deg = Some(observation.unwrapped_angle_deg);
                end_electrical_unwrapped_deg = Some(electrical_unwrapped_deg);

                let pos_offset_deg =
                    wrap_degrees(electrical_angle_deg - MOTOR_POLE_PAIRS * measurement.angle_deg);
                let neg_offset_deg =
                    wrap_degrees(electrical_angle_deg + MOTOR_POLE_PAIRS * measurement.angle_deg);
                pos_offset_sin_sum += sinf(degrees_to_radians(pos_offset_deg));
                pos_offset_cos_sum +=
                    sinf(degrees_to_radians(pos_offset_deg) + core::f32::consts::FRAC_PI_2);
                neg_offset_sin_sum += sinf(degrees_to_radians(neg_offset_deg));
                neg_offset_cos_sum +=
                    sinf(degrees_to_radians(neg_offset_deg) + core::f32::consts::FRAC_PI_2);
                sample_count += 1;
            }
        }

        if loop_index == CALIBRATION_SETTLE_LOOPS || loop_index + 1 == CALIBRATION_TOTAL_LOOPS {
            let _ = writeln!(
                serial,
                "cal sweep loop={} shaft={:.2}deg elec={:.2}deg\r",
                loop_index, open_loop_shaft_deg, electrical_angle_deg,
            );
            let _ = serial.flush();
        }

        delay.delay_millis(CONTROL_PERIOD_MS);
        loop_index += 1;
    }

    motor_drive.coast();
    delay.delay_millis(200);

    let hall_travel_deg = end_hall_unwrapped_deg? - start_hall_unwrapped_deg?;
    let electrical_travel_deg = end_electrical_unwrapped_deg? - start_electrical_unwrapped_deg?;
    if hall_travel_deg.abs() < MIN_CALIBRATION_HALL_TRAVEL_DEG || sample_count == 0 {
        let _ = writeln!(
            serial,
            "cal invalid travel hall={:.2}deg elec={:.2}deg samples={}\r",
            hall_travel_deg, electrical_travel_deg, sample_count,
        );
        let _ = serial.flush();
        return None;
    }

    let direction_sign = if hall_travel_deg * electrical_travel_deg >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let electrical_offset_deg = if direction_sign > 0.0 {
        wrap_degrees(atan2f_degrees(pos_offset_sin_sum, pos_offset_cos_sum))
    } else {
        wrap_degrees(atan2f_degrees(neg_offset_sin_sum, neg_offset_cos_sum))
    };

    let _ = writeln!(
        serial,
        "cal travel hall={:.2}deg elec={:.2}deg samples={}\r",
        hall_travel_deg, electrical_travel_deg, sample_count,
    );
    let _ = serial.flush();

    Some(HallElectricalCalibration {
        direction_sign,
        electrical_offset_deg,
    })
}

fn refine_torque_phase_offset(
    serial: &mut Uart<'_, Blocking>,
    delay: &esp_hal::delay::Delay,
    motor_drive: &mut PwmMotorDrive<'_>,
    hall: &mut Tmag5273<'_>,
    wheel: &mut WheelObserver,
    calibration: HallElectricalCalibration,
) -> Option<HallElectricalCalibration> {
    write_line(serial, "phase search begin");

    let target_sign = signum_nonzero(TARGET_WHEEL_SPEED_DPS);
    let mut best_offset_delta_deg = 0.0_f32;
    let mut best_score = f32::NEG_INFINITY;

    for candidate_offset_deg in PHASE_SEARCH_OFFSETS_DEG {
        motor_drive.enable();
        let mut start_unwrapped_deg = None;
        let mut end_unwrapped_deg = None;

        let mut loop_index = 0_u32;
        while loop_index < PHASE_SEARCH_LOOPS {
            if let Ok(measurement) = hall.read_measurement() {
                let observation = wheel.observe(measurement.angle_deg);
                if loop_index >= PHASE_SEARCH_SETTLE_LOOPS {
                    if start_unwrapped_deg.is_none() {
                        start_unwrapped_deg = Some(observation.unwrapped_angle_deg);
                    }
                    end_unwrapped_deg = Some(observation.unwrapped_angle_deg);
                }

                let electrical_angle_deg = wrap_degrees(
                    calibration.electrical_angle_deg(measurement.angle_deg) + candidate_offset_deg,
                );
                let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
                    PHASE_SEARCH_UQ_V,
                    degrees_to_radians(electrical_angle_deg),
                    VOLTAGE_LIMIT_V,
                );
                motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);
            }

            delay.delay_millis(CONTROL_PERIOD_MS);
            loop_index += 1;
        }

        motor_drive.coast();
        delay.delay_millis(150);

        let travel_deg = match (start_unwrapped_deg, end_unwrapped_deg) {
            (Some(start), Some(end)) => end - start,
            _ => continue,
        };
        let score = target_sign * travel_deg;
        let _ = writeln!(
            serial,
            "phase candidate offset={:.0}deg travel={:+.2}deg score={:+.2}\r",
            candidate_offset_deg, travel_deg, score,
        );
        let _ = serial.flush();

        if score > best_score {
            best_score = score;
            best_offset_delta_deg = candidate_offset_deg;
        }
    }

    if !best_score.is_finite() {
        write_line(serial, "phase search failed");
        return None;
    }

    let chosen = HallElectricalCalibration {
        direction_sign: calibration.direction_sign,
        electrical_offset_deg: wrap_degrees(
            calibration.electrical_offset_deg + best_offset_delta_deg,
        ),
    };
    let _ = writeln!(
        serial,
        "phase search chose offset_delta={:.0}deg final_offset={:.2}deg score={:+.2}\r",
        best_offset_delta_deg, chosen.electrical_offset_deg, best_score,
    );
    let _ = serial.flush();
    Some(chosen)
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

fn degrees_to_radians(angle_deg: f32) -> f32 {
    angle_deg * (core::f32::consts::PI / 180.0)
}

fn unwrap_near(reference_unwrapped_deg: f32, raw_wrapped_deg: f32) -> f32 {
    reference_unwrapped_deg
        + wrap_angle_delta_deg(raw_wrapped_deg - wrap_degrees(reference_unwrapped_deg))
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

fn wrap_angle_delta_deg(delta_deg: f32) -> f32 {
    if delta_deg > 180.0 {
        delta_deg - 360.0
    } else if delta_deg < -180.0 {
        delta_deg + 360.0
    } else {
        delta_deg
    }
}

fn signum_nonzero(value: f32) -> f32 {
    if value >= 0.0 { 1.0 } else { -1.0 }
}

fn atan2f_degrees(y: f32, x: f32) -> f32 {
    libm::atan2f(y, x) * (180.0 / core::f32::consts::PI)
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
