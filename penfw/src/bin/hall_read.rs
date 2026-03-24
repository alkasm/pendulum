#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;

use core::fmt::Write;

use bringup::{
    HALL_SENSOR_ADDR, i2c_device_present, init_console, init_delay, init_primary_i2c,
    max_clock_config, write_fmt_line, write_line,
};
use esp_hal::main;

const TMAG5273_REG_DEVICE_CONFIG_1: u8 = 0x00;
const TMAG5273_REG_DEVICE_CONFIG_2: u8 = 0x01;
const TMAG5273_REG_SENSOR_CONFIG_1: u8 = 0x02;
const TMAG5273_REG_SENSOR_CONFIG_2: u8 = 0x03;
const TMAG5273_REG_T_CONFIG: u8 = 0x07;
const TMAG5273_REG_T_MSB_RESULT: u8 = 0x10;
const TMAG5273_TEMP_SENSE_T0_C: f32 = 25.0;
const TMAG5273_TEMP_ADC_T0: i16 = 17_508;
const TMAG5273_TEMP_ADC_RES: f32 = 60.1;
const TMAG5273_RANGE_MT: f32 = 80.0;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let mut i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let mut reg_value = [0_u8; 1];
    let mut result_buffer = [0_u8; 13];

    write_line(&mut serial, "hall_read ready");
    write_line(&mut serial, "scanning primary I2C bus on GPIO21/22 for TMAG5273 at 0x22");

    loop {
        let hall_present = i2c_device_present(&mut i2c, HALL_SENSOR_ADDR);
        write_fmt_line(
            &mut serial,
            format_args!(
                "TMAG5273 {} at 0x{:02X}",
                if hall_present { "present" } else { "missing" },
                HALL_SENSOR_ADDR
            ),
        );

        let _ = serial.write_str("I2C devices:");
        for address in 0x03..=0x77 {
            if i2c_device_present(&mut i2c, address) {
                let _ = write!(serial, " 0x{address:02X}");
            }
        }
        let _ = serial.write_str("\r\n");
        let _ = serial.flush();

        if hall_present {
            let _ = serial.write_str("registers:");
            for register in 0x00..=0x0f {
                if i2c
                    .write_read(HALL_SENSOR_ADDR, &[register], &mut reg_value)
                    .is_ok()
                {
                    let _ = write!(serial, " {:02X}={:02X}", register, reg_value[0]);
                }
            }
            let _ = serial.write_str("\r\n");
            let _ = serial.flush();

            match configure_tmag5273(&mut i2c) {
                Ok(()) => write_line(
                    &mut serial,
                    "configured TMAG5273 for continuous measurement, XYZ channels, temperature, XY angle",
                ),
                Err(register) => {
                    write_fmt_line(
                        &mut serial,
                        format_args!("failed to configure TMAG5273 at register 0x{register:02X}"),
                    );
                    delay.delay_millis(1_000);
                    continue;
                }
            }

            break;
        }

        delay.delay_millis(1_000);
    }

    loop {
        match i2c.write_read(HALL_SENSOR_ADDR, &[TMAG5273_REG_T_MSB_RESULT], &mut result_buffer) {
            Ok(()) => {
                let temperature_c = decode_temperature_c(result_buffer[0], result_buffer[1]);
                let x_mt = decode_magnetic_mt(result_buffer[2], result_buffer[3], TMAG5273_RANGE_MT);
                let y_mt = decode_magnetic_mt(result_buffer[4], result_buffer[5], TMAG5273_RANGE_MT);
                let z_mt = decode_magnetic_mt(result_buffer[6], result_buffer[7], TMAG5273_RANGE_MT);
                let conv_status = result_buffer[8];
                let angle_deg = decode_angle_deg(result_buffer[9], result_buffer[10]);
                let magnitude = result_buffer[11];
                let device_status = result_buffer[12];

                write_fmt_line(
                    &mut serial,
                    format_args!(
                        "temp={temperature_c:.1}C x={x_mt:.2}mT y={y_mt:.2}mT z={z_mt:.2}mT angle={angle_deg:.2}deg mag=0x{magnitude:02X} conv=0x{conv_status:02X} dev=0x{device_status:02X}"
                    ),
                );
            }
            Err(_) => {
                write_fmt_line(
                    &mut serial,
                    format_args!(
                        "failed to read TMAG5273 result registers starting at 0x{TMAG5273_REG_T_MSB_RESULT:02X}"
                    ),
                );
            }
        }

        delay.delay_millis(250);
    }
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

fn decode_temperature_c(msb: u8, lsb: u8) -> f32 {
    let raw = i16::from_be_bytes([msb, lsb]);
    TMAG5273_TEMP_SENSE_T0_C + ((raw - TMAG5273_TEMP_ADC_T0) as f32 / TMAG5273_TEMP_ADC_RES)
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
