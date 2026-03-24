#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;

use core::fmt::Write;

use bringup::{init_console, init_delay, max_clock_config, write_line};
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::main;

const BASELINE_SAMPLES: u32 = 16;

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
            "adc mcp6021={ch32:>4} ({:+5}) ina_u={ch35:>4} ({:+5}) ina_v={ch36:>4} ({:+5}) ina_w={ch39:>4} ({:+5})\r",
            signed_delta(ch32, baseline.0),
            signed_delta(ch35, baseline.1),
            signed_delta(ch36, baseline.2),
            signed_delta(ch39, baseline.3),
        );
        let _ = serial.flush();

        delay.delay_millis(500);
    }
}

fn signed_delta(value: u16, baseline: u16) -> i32 {
    i32::from(value) - i32::from(baseline)
}
