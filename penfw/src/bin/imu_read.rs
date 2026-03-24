#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;

use bringup::{
    MPU6050_ADDR_ALTERNATE, MPU6050_ADDR_PRIMARY, MPU6050_WHO_AM_I_REG, init_console,
    init_delay, init_primary_i2c, max_clock_config, write_fmt_line, write_line,
};
use esp_hal::main;

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
    let mut who_am_i = [0_u8; 1];

    write_line(&mut serial, "imu_read ready");
    write_line(
        &mut serial,
        "probing MPU-6050 WHO_AM_I on primary I2C bus (expected addresses 0x68 or 0x69)",
    );

    loop {
        let primary = i2c.write_read(
            MPU6050_ADDR_PRIMARY,
            &[MPU6050_WHO_AM_I_REG],
            &mut who_am_i,
        );
        match primary {
            Ok(()) => write_fmt_line(
                &mut serial,
                format_args!(
                    "MPU-6050 present at 0x{:02X}, WHO_AM_I=0x{:02X}",
                    MPU6050_ADDR_PRIMARY, who_am_i[0]
                ),
            ),
            Err(_) => write_fmt_line(
                &mut serial,
                format_args!("no response from 0x{:02X}", MPU6050_ADDR_PRIMARY),
            ),
        }

        let alternate = i2c.write_read(
            MPU6050_ADDR_ALTERNATE,
            &[MPU6050_WHO_AM_I_REG],
            &mut who_am_i,
        );
        match alternate {
            Ok(()) => write_fmt_line(
                &mut serial,
                format_args!(
                    "MPU-6050 present at 0x{:02X}, WHO_AM_I=0x{:02X}",
                    MPU6050_ADDR_ALTERNATE, who_am_i[0]
                ),
            ),
            Err(_) => write_fmt_line(
                &mut serial,
                format_args!("no response from 0x{:02X}", MPU6050_ADDR_ALTERNATE),
            ),
        }

        delay.delay_millis(1_000);
    }
}
