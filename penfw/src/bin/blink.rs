#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use esp_hal::{
    clock::CpuClock,
    main,
    rmt::Rmt,
    time::{Duration, Instant, Rate},
};
use esp_hal_smartled::{RmtSmartLeds, Ws2812Timing, buffer_size, color_order};
use smart_leds::{RGB8, SmartLedsWrite};

const LED_BRIGHTNESS: u8 = 8;
const BLINK_PERIOD_MS: u64 = 500;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let mut led =
        RmtSmartLeds::<{ buffer_size::<RGB8>(1) }, _, RGB8, color_order::Grb, Ws2812Timing>::new(
            rmt.channel0,
            peripherals.GPIO2,
        )
        .unwrap();

    loop {
        let _ = led.write(
            [RGB8 {
                r: LED_BRIGHTNESS,
                g: 0,
                b: 0,
            }]
            .into_iter(),
        );
        busy_wait_ms(BLINK_PERIOD_MS);

        let _ = led.write([RGB8::default()].into_iter());
        busy_wait_ms(BLINK_PERIOD_MS);
    }
}

fn busy_wait_ms(ms: u64) {
    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(ms) {}
}
