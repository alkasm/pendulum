#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();
#[path = "board_init.rs"]
mod board_init;
mod command;
mod hall;
#[path = "hw/mod.rs"]
mod hw;
mod imu;
mod math;
mod motor_calibration;
mod motor_drive;
mod runtime;
mod settings;
mod wifi;

use board_init::{init_console, init_delay, init_primary_i2c, max_clock_config};
use esp_hal::{main, rng::Rng, timer::timg::TimerGroup};
use hw::CurrentSensor;
use motor_drive::{PwmMotorDrive, PwmMotorDriveParts};
use runtime::{Board, FirmwareRuntime, load_startup_config};
use settings::SettingsService;
use wifi::WifiService;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let current_sensor = CurrentSensor::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
    );

    let (settings, startup) = load_startup_config(SettingsService::new());

    let motor_drive = PwmMotorDrive::new(PwmMotorDriveParts {
        peripheral: peripherals.MCPWM0,
        enable: peripherals.GPIO5,
        uh: peripherals.GPIO16,
        ul: peripherals.GPIO17,
        vh: peripherals.GPIO18,
        vl: peripherals.GPIO23,
        wh: peripherals.GPIO19,
        wl: peripherals.GPIO33,
    });

    let i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let timer_group = TimerGroup::new(peripherals.TIMG0);
    let wifi = WifiService::new(
        timer_group.timer0,
        Rng::new(peripherals.RNG),
        peripherals.WIFI,
    )
    .expect("failed to initialize Wi-Fi service");

    let board = Board::new(serial, current_sensor, motor_drive, i2c);
    FirmwareRuntime::new(board, settings, delay, wifi, startup).run()
}
