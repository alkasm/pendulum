#![no_std]
#![no_main]

//! One-shot motor calibration binary.
//!
//! This performs a longer hall/electrical sweep, refines torque polarity and
//! phase offset, saves the result into NVS, then idles. The normal `penfw`
//! binary can reuse the saved values on later boots.

esp_bootloader_esp_idf::esp_app_desc!();
use esp_alloc as _;

#[path = "../bringup.rs"]
mod bringup;
#[path = "../hw/mod.rs"]
mod hw;
#[path = "../motor_calibration.rs"]
mod motor_calibration;
#[path = "../settings.rs"]
mod settings;

use core::fmt::Write;

use bringup::{HALL_SENSOR_ADDR, init_console, init_delay, max_clock_config, write_line};
use esp_hal::{
    Blocking,
    gpio::{Level, Output, OutputConfig},
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
use hw::Tmag5273;
use libm::{atan2f, sinf};
use motor_calibration::{StoredMotorCalibration, save_motor_calibration};

const CONTROL_PERIOD_MS: u32 = 5;

const PWM_FREQUENCY_HZ: u32 = 32_000;
const PWM_PERIOD_TICKS: u16 = 2500;
const DEAD_ZONE: f32 = 0.02;

const VOLTAGE_POWER_SUPPLY_V: f32 = 5.0;
const VOLTAGE_LIMIT_V: f32 = 3.6;
const MOTOR_POLE_PAIRS: f32 = 7.0;

const CALIBRATION_VOLTAGE_V: f32 = 1.2;
const CALIBRATION_WHEEL_SPEED_DPS: f32 = -180.0;
const CALIBRATION_TOTAL_LOOPS: u32 = 1_600;
const CALIBRATION_SETTLE_LOOPS: u32 = 240;
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
    uh: PwmPinA<'a, 0>,
    ul: PwmPinB<'a, 0>,
    vh: PwmPinA<'a, 1>,
    vl: PwmPinB<'a, 1>,
    wh: PwmPinA<'a, 2>,
    wl: PwmPinB<'a, 2>,
}

#[derive(Clone, Copy)]
struct HallElectricalCalibration {
    direction_sign: f32,
    electrical_offset_deg: f32,
    torque_sign: f32,
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
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

    motor_drive.disable();
    motor_drive.coast();

    write_line(&mut serial, "calibrate_motor start");
    if !configure_hall_best_effort(&mut hall) {
        write_line(&mut serial, "hall missing; idling");
        loop {
            delay.delay_millis(1_000);
        }
    }

    let calibration =
        match calibrate_hall_electrical_cycle(&mut serial, &delay, &mut motor_drive, &mut hall) {
            Some(calibration) => calibration,
            None => {
                motor_drive.disable();
                motor_drive.coast();
                write_line(&mut serial, "motor calibration failed; idling");
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
        calibration,
    )
    .unwrap_or(calibration);

    let stored = StoredMotorCalibration {
        direction_sign: calibration.direction_sign,
        electrical_offset_deg: calibration.electrical_offset_deg,
        torque_sign: calibration.torque_sign,
    };

    match save_motor_calibration(stored) {
        Ok(()) => {
            let _ = writeln!(
                serial,
                concat!("saved dir={:+.0} offset={:.2}deg torque={:+.0}\r"),
                stored.direction_sign, stored.electrical_offset_deg, stored.torque_sign,
            );
            let _ = serial.flush();
        }
        Err(_) => {
            write_line(&mut serial, "save failed; idling");
        }
    }

    motor_drive.disable();
    motor_drive.coast();
    loop {
        delay.delay_millis(1_000);
    }
}

impl<'a> PwmMotorDrive<'a> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        enable: GPIO5<'a>,
        _diag: GPIO34<'a>,
        uh: PwmPinA<'a, 0>,
        ul: PwmPinB<'a, 0>,
        vh: PwmPinA<'a, 1>,
        vl: PwmPinB<'a, 1>,
        wh: PwmPinA<'a, 2>,
        wl: PwmPinB<'a, 2>,
    ) -> Self {
        Self {
            enable: Output::new(enable, Level::Low, OutputConfig::default()),
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
    let mut last_hall_unwrapped_deg = None;

    let mut loop_index = 0_u32;
    while loop_index < CALIBRATION_TOTAL_LOOPS {
        open_loop_shaft_deg += CALIBRATION_WHEEL_SPEED_DPS * dt_s();
        let electrical_unwrapped_deg = MOTOR_POLE_PAIRS * open_loop_shaft_deg;
        let electrical_angle_deg = wrap_degrees(electrical_unwrapped_deg);
        let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
            CALIBRATION_VOLTAGE_V,
            degrees_to_radians(electrical_angle_deg),
            VOLTAGE_LIMIT_V,
        );
        motor_drive.set_phase_voltages(ua_v, ub_v, uc_v);

        if let Ok(measurement) = hall.read_measurement() {
            let hall_unwrapped_deg = match last_hall_unwrapped_deg {
                Some(previous) => unwrap_near(previous, measurement.angle_deg),
                None => measurement.angle_deg,
            };
            last_hall_unwrapped_deg = Some(hall_unwrapped_deg);

            if loop_index >= CALIBRATION_SETTLE_LOOPS {
                if start_hall_unwrapped_deg.is_none() {
                    start_hall_unwrapped_deg = Some(hall_unwrapped_deg);
                    start_electrical_unwrapped_deg = Some(electrical_unwrapped_deg);
                }
                end_hall_unwrapped_deg = Some(hall_unwrapped_deg);
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
        torque_sign: 1.0,
    })
}

fn refine_torque_phase_offset(
    serial: &mut Uart<'_, Blocking>,
    delay: &esp_hal::delay::Delay,
    motor_drive: &mut PwmMotorDrive<'_>,
    hall: &mut Tmag5273<'_>,
    calibration: HallElectricalCalibration,
) -> Option<HallElectricalCalibration> {
    write_line(serial, "phase search begin");

    let mut best_offset_delta_deg = 0.0_f32;
    let mut best_torque_sign = 1.0_f32;
    let mut best_score = f32::NEG_INFINITY;

    for candidate_offset_deg in PHASE_SEARCH_OFFSETS_DEG {
        for candidate_torque_sign in [1.0_f32, -1.0_f32] {
            let pos_travel_deg = measure_phase_search_travel(
                delay,
                motor_drive,
                hall,
                calibration,
                candidate_offset_deg,
                candidate_torque_sign * PHASE_SEARCH_UQ_V,
            )?;
            let neg_travel_deg = measure_phase_search_travel(
                delay,
                motor_drive,
                hall,
                calibration,
                candidate_offset_deg,
                -candidate_torque_sign * PHASE_SEARCH_UQ_V,
            )?;

            let opposite_direction = pos_travel_deg * neg_travel_deg < 0.0;
            let symmetry_penalty = (pos_travel_deg + neg_travel_deg).abs();
            let score = if opposite_direction {
                let weaker_travel_deg = if pos_travel_deg.abs() < neg_travel_deg.abs() {
                    pos_travel_deg.abs()
                } else {
                    neg_travel_deg.abs()
                };
                weaker_travel_deg - 0.25 * symmetry_penalty
            } else {
                -symmetry_penalty
            };

            let _ = writeln!(
                serial,
                concat!(
                    "phase candidate offset={:.0}deg torque={:+.0} ",
                    "pos={:+.2}deg neg={:+.2}deg score={:+.2}\r"
                ),
                candidate_offset_deg, candidate_torque_sign, pos_travel_deg, neg_travel_deg, score,
            );
            let _ = serial.flush();

            if score > best_score {
                best_score = score;
                best_offset_delta_deg = candidate_offset_deg;
                best_torque_sign = candidate_torque_sign;
            }
        }
    }

    if !best_score.is_finite() || best_score <= 0.0 {
        write_line(serial, "phase search failed");
        return None;
    }

    let chosen = HallElectricalCalibration {
        direction_sign: calibration.direction_sign,
        electrical_offset_deg: wrap_degrees(
            calibration.electrical_offset_deg + best_offset_delta_deg,
        ),
        torque_sign: calibration.torque_sign * best_torque_sign,
    };
    let _ = writeln!(
        serial,
        concat!(
            "phase search chose offset_delta={:.0}deg final_offset={:.2}deg ",
            "torque={:+.0} score={:+.2}\r"
        ),
        best_offset_delta_deg, chosen.electrical_offset_deg, chosen.torque_sign, best_score,
    );
    let _ = serial.flush();
    Some(chosen)
}

fn measure_phase_search_travel(
    delay: &esp_hal::delay::Delay,
    motor_drive: &mut PwmMotorDrive<'_>,
    hall: &mut Tmag5273<'_>,
    calibration: HallElectricalCalibration,
    candidate_offset_deg: f32,
    uq_v: f32,
) -> Option<f32> {
    motor_drive.enable();
    let mut start_unwrapped_deg = None;
    let mut end_unwrapped_deg = None;
    let mut last_unwrapped_deg = None;

    let mut loop_index = 0_u32;
    while loop_index < PHASE_SEARCH_LOOPS {
        if let Ok(measurement) = hall.read_measurement() {
            let hall_unwrapped_deg = match last_unwrapped_deg {
                Some(previous) => unwrap_near(previous, measurement.angle_deg),
                None => measurement.angle_deg,
            };
            last_unwrapped_deg = Some(hall_unwrapped_deg);

            if loop_index >= PHASE_SEARCH_SETTLE_LOOPS {
                if start_unwrapped_deg.is_none() {
                    start_unwrapped_deg = Some(hall_unwrapped_deg);
                }
                end_unwrapped_deg = Some(hall_unwrapped_deg);
            }

            let electrical_angle_deg = wrap_degrees(
                calibration.electrical_angle_deg(measurement.angle_deg) + candidate_offset_deg,
            );
            let (ua_v, ub_v, uc_v) = simplefoc_sine_pwm_phase_voltages(
                uq_v,
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

    match (start_unwrapped_deg, end_unwrapped_deg) {
        (Some(start), Some(end)) => Some(end - start),
        _ => None,
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

fn degrees_to_radians(angle_deg: f32) -> f32 {
    angle_deg * (core::f32::consts::PI / 180.0)
}

fn unwrap_near(reference_unwrapped_deg: f32, raw_wrapped_deg: f32) -> f32 {
    reference_unwrapped_deg
        + wrap_angle_delta_deg(raw_wrapped_deg - wrap_degrees(reference_unwrapped_deg))
}

fn wrap_angle_delta_deg(delta_deg: f32) -> f32 {
    let mut wrapped = delta_deg;
    while wrapped >= 180.0 {
        wrapped -= 360.0;
    }
    while wrapped < -180.0 {
        wrapped += 360.0;
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

fn atan2f_degrees(y: f32, x: f32) -> f32 {
    atan2f(y, x) * (180.0 / core::f32::consts::PI)
}

fn clamp(value: f32, lo: f32, hi: f32) -> f32 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}
