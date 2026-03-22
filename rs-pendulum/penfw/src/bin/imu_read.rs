#![no_std]
#![no_main]

use esp_hal::main;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
