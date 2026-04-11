#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();
use esp_alloc as _;

use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    main,
    time::{Duration, Instant},
};
const BLINK_PERIOD_MS: u64 = 500;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let mut led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());

    loop {
        led.toggle();
        busy_wait_ms(BLINK_PERIOD_MS);
    }
}

fn busy_wait_ms(ms: u64) {
    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(ms) {}
}
