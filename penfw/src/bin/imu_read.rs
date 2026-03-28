#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;
#[path = "../hw/mod.rs"]
mod hw;

use core::fmt::Write;

use bringup::{init_console, init_delay, max_clock_config, write_line};
use esp_hal::main;
use hw::{GY521_DEFAULT_I2C_ADDR, Gy521Error, Gy521Imu};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let mut imu = match Gy521Imu::new(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22) {
        Ok(imu) => imu,
        Err(Gy521Error::RegisterRead(register)) => loop {
            let _ = writeln!(
                serial,
                "failed to read MPU-6050 register 0x{register:02X} at address 0x{GY521_DEFAULT_I2C_ADDR:02X}\r"
            );
            let _ = serial.flush();
            delay.delay_millis(1_000);
        },
        Err(Gy521Error::UnexpectedWhoAmI(who_am_i)) => loop {
            let _ = writeln!(
                serial,
                "unexpected WHO_AM_I 0x{who_am_i:02X} at address 0x{GY521_DEFAULT_I2C_ADDR:02X}\r"
            );
            let _ = serial.flush();
            delay.delay_millis(1_000);
        },
    };

    write_line(&mut serial, "imu_read ready");
    write_line(
        &mut serial,
        "reading GY-521 accel/gyro data on GPIO21/22 I2C at default address 0x68",
    );

    match imu.wake() {
        Ok(()) => write_line(&mut serial, "woke IMU and selected PLL clock"),
        Err(register) => {
            let _ = writeln!(serial, "failed to wake IMU via register 0x{register:02X}\r");
            let _ = serial.flush();
            loop {
                delay.delay_millis(1_000);
            }
        }
    }

    loop {
        match imu.read_measurement() {
            Ok(measurement) => {
                let _ = writeln!(
                    serial,
                    "ax={:+6.3}g ay={:+6.3}g az={:+6.3}g gx={:+7.2}dps gy={:+7.2}dps gz={:+7.2}dps temp={:>5.1}C theta={:+7.2}deg\r",
                    measurement.ax_g,
                    measurement.ay_g,
                    measurement.az_g,
                    measurement.gx_dps,
                    measurement.gy_dps,
                    measurement.gz_dps,
                    measurement.temperature_c,
                    measurement.theta_deg,
                );
                let _ = serial.flush();
            }
            Err(register) => {
                let _ = writeln!(
                    serial,
                    "failed to read IMU measurement at register 0x{register:02X}\r"
                );
                let _ = serial.flush();
            }
        }
        delay.delay_millis(250);
    }
}
