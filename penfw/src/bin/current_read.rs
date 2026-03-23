#![no_std]
#![no_main]

#[path = "../bringup.rs"]
mod bringup;

use core::fmt::Write;

use bringup::{init_console, init_delay, max_clock_config, write_line};
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
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

    let mut adc_config = AdcConfig::new();
    let mut gpio32 = adc_config.enable_pin(peripherals.GPIO32, Attenuation::_11dB);
    let mut gpio35 = adc_config.enable_pin(peripherals.GPIO35, Attenuation::_11dB);
    let mut gpio36 = adc_config.enable_pin(peripherals.GPIO36, Attenuation::_11dB);
    let mut gpio39 = adc_config.enable_pin(peripherals.GPIO39, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);

    write_line(&mut serial, "current_read ready");
    write_line(
        &mut serial,
        "sampling raw ADC counts from GPIO32/GPIO35/GPIO36/GPIO39",
    );

    loop {
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

        let _ = writeln!(
            serial,
            "adc gpio32={ch32:>4} gpio35={ch35:>4} gpio36={ch36:>4} gpio39={ch39:>4}\r"
        );
        let _ = serial.flush();

        delay.delay_millis(500);
    }
}
