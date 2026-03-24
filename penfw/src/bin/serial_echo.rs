#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;

use bringup::{init_console, init_delay, max_clock_config, write_bytes, write_line};
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
    let mut buf = [0_u8; 64];

    write_line(&mut serial, "serial_echo ready on UART0");
    write_line(&mut serial, "type bytes and they will be echoed back");

    loop {
        if let Ok(read) = serial.read(&mut buf) {
            if read > 0 {
                write_bytes(&mut serial, &buf[..read]);
            }
        }

        delay.delay_millis(10);
    }
}
