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
use hw::{MotorDriverBoard, SIX_STEP_COMMUTATION};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();

    let mut board = MotorDriverBoard::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
        peripherals.I2C0,
        peripherals.GPIO21,
        peripherals.GPIO22,
        bringup::HALL_SENSOR_ADDR,
        peripherals.GPIO5,
        peripherals.GPIO34,
        peripherals.GPIO16,
        peripherals.GPIO17,
        peripherals.GPIO18,
        peripherals.GPIO23,
        peripherals.GPIO19,
        peripherals.GPIO33,
    );

    write_line(&mut serial, "motor_open_loop ready");
    write_line(
        &mut serial,
        "holding TMC6300 VIO low for 2 seconds before stepping six-phase commutation",
    );
    delay.delay_millis(2_000);

    board.motor_driver.enable();
    write_line(&mut serial, "driver enabled on GPIO5");

    let mut step_index = 0_usize;
    loop {
        let step = SIX_STEP_COMMUTATION[step_index];
        board.motor_driver.apply_step(step);
        let _ = writeln!(
            serial,
            "step={} pattern={} diag={}\r",
            step_index,
            step.name,
            if board.motor_driver.diag_is_high() { "high" } else { "low" }
        );
        let _ = serial.flush();
        delay.delay_millis(500);
        step_index = (step_index + 1) % SIX_STEP_COMMUTATION.len();
    }
}
