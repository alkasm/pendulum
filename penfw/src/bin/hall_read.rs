#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../board_init.rs"]
mod board_init;
#[path = "../hw/mod.rs"]
mod hw;

use core::fmt::Write;

use board_init::{HALL_SENSOR_ADDR, init_console, init_delay, max_clock_config, write_line};
use esp_hal::main;
use hw::Tmag5273;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let mut hall = Tmag5273::new(
        peripherals.I2C0,
        peripherals.GPIO21,
        peripherals.GPIO22,
        HALL_SENSOR_ADDR,
    );

    write_line(&mut serial, "hall_read ready");
    write_line(
        &mut serial,
        "scanning primary I2C bus on GPIO21/22 for TMAG5273 at 0x22",
    );

    loop {
        if hall.is_present() {
            write_line(&mut serial, "TMAG5273 present at 0x22");
            if let Ok(identity) = hall.read_identity() {
                let _ = writeln!(
                    serial,
                    "id addr=0x{:02X} device_id=0x{:02X} manufacturer=0x{:04X}\r",
                    identity.address, identity.device_id, identity.manufacturer_id
                );
                let _ = serial.flush();
            }

            match hall.configure_default() {
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

    loop {
        match hall.read_measurement() {
            Ok(measurement) => {
                let _ = writeln!(
                    serial,
                    "temp={:>5.1}C x={:+7.2}mT y={:+7.2}mT z={:+7.2}mT angle={:>7.2}deg mag=0x{:02X} conv=set{}{}{}{} dev={}{}{}{}{}\r",
                    measurement.temperature_c,
                    measurement.x_mt,
                    measurement.y_mt,
                    measurement.z_mt,
                    measurement.angle_deg,
                    measurement.magnitude,
                    measurement.conv_status.set_count,
                    if measurement.conv_status.result_ready {
                        " ready"
                    } else {
                        ""
                    },
                    if measurement.conv_status.por {
                        " por"
                    } else {
                        ""
                    },
                    if measurement.conv_status.diag_fail {
                        " diag"
                    } else {
                        ""
                    },
                    if measurement.device_status.int_pin_high {
                        " int_high"
                    } else {
                        " int_low"
                    },
                    if measurement.device_status.oscillator_error {
                        " osc"
                    } else {
                        ""
                    },
                    if measurement.device_status.int_pin_error {
                        " int_err"
                    } else {
                        ""
                    },
                    if measurement.device_status.otp_crc_error {
                        " otp_crc"
                    } else {
                        ""
                    },
                    if measurement.device_status.vcc_uv_error {
                        " uv"
                    } else {
                        ""
                    },
                );
                let _ = serial.flush();
            }
            Err(register) => {
                let _ = writeln!(
                    serial,
                    "failed to read TMAG5273 measurement at register 0x{register:02X}\r"
                );
                let _ = serial.flush();
            }
        }

        delay.delay_millis(250);
    }
}
