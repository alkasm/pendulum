#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../board_init.rs"]
mod board_init;
#[path = "../hw/mod.rs"]
mod hw;

use core::fmt::Write;

use board_init::{init_console, init_delay, max_clock_config, write_line};
use esp_hal::main;
use hw::CurrentSensor;

const BASELINE_SAMPLES: u32 = 16;

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

    let mut current_sensor = CurrentSensor::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
    );

    write_line(&mut serial, "current_read ready");
    write_line(
        &mut serial,
        "sampling current ADC counts: MCP6021=GPIO32, INA240 U=GPIO35, V=GPIO36, W=GPIO39",
    );

    let baseline = current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    let _ = writeln!(
        serial,
        "baseline mcp6021={} ina_u={} ina_v={} ina_w={}\r",
        baseline.mcp6021_counts,
        baseline.ina_u_counts,
        baseline.ina_v_counts,
        baseline.ina_w_counts
    );
    let _ = serial.flush();

    loop {
        let sample = current_sensor.read();
        let _ = writeln!(
            serial,
            "adc mcp6021={:>4} ({:+5}, {:>4.2}V) ina_u={:>4} ({:+5}, {:+5.2}A) ina_v={:>4} ({:+5}, {:+5.2}A) ina_w={:>4} ({:+5}, {:+5.2}A)\r",
            sample.mcp6021.counts,
            sample.mcp6021.delta_counts,
            sample.mcp6021.volts,
            sample.ina_u.counts,
            sample.ina_u.delta_counts,
            sample.ina_u.amps,
            sample.ina_v.counts,
            sample.ina_v.delta_counts,
            sample.ina_v.amps,
            sample.ina_w.counts,
            sample.ina_w.delta_counts,
            sample.ina_w.amps,
        );
        let _ = serial.flush();

        delay.delay_millis(500);
    }
}
