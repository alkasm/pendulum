#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;

use core::fmt::Write;

use bringup::{
    HALL_SENSOR_ADDR, i2c_device_present, init_console, init_delay, init_primary_i2c,
    max_clock_config, write_line,
};
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::main;

const BASELINE_SAMPLES: u32 = 16;
const STEP_HOLD_MS: u32 = 500;

const TMAG5273_REG_DEVICE_CONFIG_1: u8 = 0x00;
const TMAG5273_REG_DEVICE_CONFIG_2: u8 = 0x01;
const TMAG5273_REG_SENSOR_CONFIG_1: u8 = 0x02;
const TMAG5273_REG_SENSOR_CONFIG_2: u8 = 0x03;
const TMAG5273_REG_T_CONFIG: u8 = 0x07;
const TMAG5273_REG_T_MSB_RESULT: u8 = 0x10;
const TMAG5273_RANGE_MT: f32 = 80.0;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[derive(Clone, Copy)]
struct CommutationStep {
    name: &'static str,
    uh: bool,
    ul: bool,
    vh: bool,
    vl: bool,
    wh: bool,
    wl: bool,
}

const STEPS: [CommutationStep; 6] = [
    CommutationStep {
        name: "U+ V-",
        uh: true,
        ul: false,
        vh: false,
        vl: true,
        wh: false,
        wl: false,
    },
    CommutationStep {
        name: "U+ W-",
        uh: true,
        ul: false,
        vh: false,
        vl: false,
        wh: false,
        wl: true,
    },
    CommutationStep {
        name: "V+ W-",
        uh: false,
        ul: false,
        vh: true,
        vl: false,
        wh: false,
        wl: true,
    },
    CommutationStep {
        name: "V+ U-",
        uh: false,
        ul: true,
        vh: true,
        vl: false,
        wh: false,
        wl: false,
    },
    CommutationStep {
        name: "W+ U-",
        uh: false,
        ul: true,
        vh: false,
        vl: false,
        wh: true,
        wl: false,
    },
    CommutationStep {
        name: "W+ V-",
        uh: false,
        ul: false,
        vh: false,
        vl: true,
        wh: true,
        wl: false,
    },
];

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();

    let mut adc_config = AdcConfig::new();
    let mut gpio32 = adc_config.enable_pin(peripherals.GPIO32, Attenuation::_11dB);
    let mut gpio35 = adc_config.enable_pin(peripherals.GPIO35, Attenuation::_11dB);
    let mut gpio36 = adc_config.enable_pin(peripherals.GPIO36, Attenuation::_11dB);
    let mut gpio39 = adc_config.enable_pin(peripherals.GPIO39, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);

    let mut i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let mut hall_result_buffer = [0_u8; 13];

    let mut driver_enable = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let diag = Input::new(peripherals.GPIO34, InputConfig::default());
    let mut uh = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());
    let mut ul = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());
    let mut vh = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut vl = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let mut wh = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    let mut wl = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());

    write_line(&mut serial, "motor_driver_integration_test ready");
    write_line(
        &mut serial,
        "sampling current-sense ADC counts: MCP6021=GPIO32, INA240 U=GPIO35, V=GPIO36, W=GPIO39",
    );

    let mut sum32: u32 = 0;
    let mut sum35: u32 = 0;
    let mut sum36: u32 = 0;
    let mut sum39: u32 = 0;

    for _ in 0..BASELINE_SAMPLES {
        let ch32 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio32) {
                break value;
            }
        };
        let ch35 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio35) {
                break value;
            }
        };
        let ch36 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio36) {
                break value;
            }
        };
        let ch39 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio39) {
                break value;
            }
        };

        sum32 += ch32 as u32;
        sum35 += ch35 as u32;
        sum36 += ch36 as u32;
        sum39 += ch39 as u32;
    }

    let baseline = (
        (sum32 / BASELINE_SAMPLES) as u16,
        (sum35 / BASELINE_SAMPLES) as u16,
        (sum36 / BASELINE_SAMPLES) as u16,
        (sum39 / BASELINE_SAMPLES) as u16,
    );
    let _ = writeln!(
        serial,
        "baseline mcp6021={} ina_u={} ina_v={} ina_w={}\r",
        baseline.0, baseline.1, baseline.2, baseline.3
    );
    let _ = serial.flush();

    write_line(&mut serial, "probing TMAG5273 at 0x22");
    loop {
        if i2c_device_present(&mut i2c, HALL_SENSOR_ADDR) {
            write_line(&mut serial, "TMAG5273 present at 0x22");
            match configure_tmag5273(&mut i2c) {
                Ok(()) => {
                    write_line(
                        &mut serial,
                        "configured TMAG5273 for continuous measurement, XYZ channels, temperature, XY angle",
                    );
                    break;
                }
                Err(register) => {
                    let _ = writeln!(
                        serial,
                        "failed to configure TMAG5273 at register 0x{register:02X}\r"
                    );
                    let _ = serial.flush();
                }
            }
        } else {
            write_line(&mut serial, "TMAG5273 missing at 0x22");
        }

        delay.delay_millis(1_000);
    }

    write_line(
        &mut serial,
        "holding TMC6300 VIO low for 2 seconds before stepping six-phase commutation",
    );
    delay.delay_millis(2_000);

    driver_enable.set_high();
    write_line(&mut serial, "driver enabled on GPIO5");

    let mut step_index = 0_usize;
    loop {
        let step = STEPS[step_index];
        apply_step(step, &mut uh, &mut ul, &mut vh, &mut vl, &mut wh, &mut wl);

        let ch32 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio32) {
                break value;
            }
        };
        let ch35 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio35) {
                break value;
            }
        };
        let ch36 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio36) {
                break value;
            }
        };
        let ch39 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio39) {
                break value;
            }
        };

        let hall_text = match i2c.write_read(
            HALL_SENSOR_ADDR,
            &[TMAG5273_REG_T_MSB_RESULT],
            &mut hall_result_buffer,
        ) {
            Ok(()) => {
                let x_mt = decode_magnetic_mt(hall_result_buffer[2], hall_result_buffer[3], TMAG5273_RANGE_MT);
                let y_mt = decode_magnetic_mt(hall_result_buffer[4], hall_result_buffer[5], TMAG5273_RANGE_MT);
                let z_mt = decode_magnetic_mt(hall_result_buffer[6], hall_result_buffer[7], TMAG5273_RANGE_MT);
                let angle_deg = decode_angle_deg(hall_result_buffer[9], hall_result_buffer[10]);
                let magnitude = hall_result_buffer[11];
                let device_status = hall_result_buffer[12];

                let _ = write!(
                    serial,
                    "step={} pattern={} diag={} mcp6021={:>4} ({:+5}) ina_u={:>4} ({:+5}) ina_v={:>4} ({:+5}) ina_w={:>4} ({:+5}) hall_x={:+7.2}mT hall_y={:+7.2}mT hall_z={:+7.2}mT angle={:>7.2}deg mag=0x{:02X} hall_dev=0x{:02X}\r\n",
                    step_index,
                    step.name,
                    if diag.is_high() { "high" } else { "low" },
                    ch32,
                    signed_delta(ch32, baseline.0),
                    ch35,
                    signed_delta(ch35, baseline.1),
                    ch36,
                    signed_delta(ch36, baseline.2),
                    ch39,
                    signed_delta(ch39, baseline.3),
                    x_mt,
                    y_mt,
                    z_mt,
                    angle_deg,
                    magnitude,
                    device_status,
                );
                let _ = serial.flush();
                true
            }
            Err(_) => false,
        };

        if !hall_text {
            let _ = writeln!(
                serial,
                "step={} pattern={} diag={} mcp6021={:>4} ({:+5}) ina_u={:>4} ({:+5}) ina_v={:>4} ({:+5}) ina_w={:>4} ({:+5}) hall_read=error\r",
                step_index,
                step.name,
                if diag.is_high() { "high" } else { "low" },
                ch32,
                signed_delta(ch32, baseline.0),
                ch35,
                signed_delta(ch35, baseline.1),
                ch36,
                signed_delta(ch36, baseline.2),
                ch39,
                signed_delta(ch39, baseline.3),
            );
            let _ = serial.flush();
        }

        delay.delay_millis(STEP_HOLD_MS);
        step_index = (step_index + 1) % STEPS.len();
    }
}

fn apply_step(
    step: CommutationStep,
    uh: &mut Output<'_>,
    ul: &mut Output<'_>,
    vh: &mut Output<'_>,
    vl: &mut Output<'_>,
    wh: &mut Output<'_>,
    wl: &mut Output<'_>,
) {
    uh.set_level(if step.uh { Level::High } else { Level::Low });
    ul.set_level(if step.ul { Level::High } else { Level::Low });
    vh.set_level(if step.vh { Level::High } else { Level::Low });
    vl.set_level(if step.vl { Level::High } else { Level::Low });
    wh.set_level(if step.wh { Level::High } else { Level::Low });
    wl.set_level(if step.wl { Level::High } else { Level::Low });
}

fn configure_tmag5273(
    i2c: &mut esp_hal::i2c::master::I2c<'_, esp_hal::Blocking>,
) -> Result<(), u8> {
    update_register(i2c, TMAG5273_REG_DEVICE_CONFIG_1, |value| value & !0x03)?;
    update_register(i2c, TMAG5273_REG_DEVICE_CONFIG_2, |value| (value & !0x17) | 0x02)?;
    update_register(i2c, TMAG5273_REG_SENSOR_CONFIG_1, |value| (value & !0xF0) | 0x70)?;
    update_register(i2c, TMAG5273_REG_SENSOR_CONFIG_2, |value| (value & !0x0F) | 0x07)?;
    update_register(i2c, TMAG5273_REG_T_CONFIG, |value| value | 0x01)?;
    Ok(())
}

fn update_register(
    i2c: &mut esp_hal::i2c::master::I2c<'_, esp_hal::Blocking>,
    register: u8,
    update: impl FnOnce(u8) -> u8,
) -> Result<(), u8> {
    let mut value = [0_u8; 1];
    if i2c.write_read(HALL_SENSOR_ADDR, &[register], &mut value).is_err() {
        return Err(register);
    }

    let updated = update(value[0]);
    if i2c.write(HALL_SENSOR_ADDR, &[register, updated]).is_err() {
        return Err(register);
    }

    Ok(())
}

fn decode_magnetic_mt(msb: u8, lsb: u8, range_mt: f32) -> f32 {
    let raw = i16::from_be_bytes([msb, lsb]) as f32;
    (-range_mt * raw) / 32_768.0
}

fn decode_angle_deg(msb: u8, lsb: u8) -> f32 {
    let raw = u16::from_be_bytes([msb, lsb]);
    let integer = ((raw >> 4) & 0x01FF) as f32;
    let fraction = (raw & 0x000F) as f32 / 16.0;
    integer + fraction
}

fn signed_delta(value: u16, baseline: u16) -> i32 {
    i32::from(value) - i32::from(baseline)
}
