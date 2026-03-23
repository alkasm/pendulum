#![no_std]
#![no_main]

#[path = "../bringup.rs"]
mod bringup;

use core::fmt::Write;

use bringup::{
    HALL_SENSOR_ADDR, i2c_device_present, init_console, init_delay, init_primary_i2c,
    max_clock_config, write_fmt_line, write_line,
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
    let mut reg_value = [0_u8; 1];

    write_line(&mut serial, "hall_read ready");
    write_line(&mut serial, "scanning primary I2C bus on GPIO21/22 for TMAG5273 at 0x35");

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
        }

        delay.delay_millis(1_000);
    }
}
