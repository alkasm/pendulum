#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();
#[path = "bringup.rs"]
mod bringup;
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

use bringup::{init_console, init_delay, init_primary_i2c, max_clock_config};
use esp_hal::{
    main,
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
};
use hw::CurrentSensor;
use motor_drive::{PWM_PERIOD_TICKS, PwmMotorDrive, low_side_pwm_config};
use runtime::{FirmwarePlatform, FirmwareRuntime, load_boot_snapshot};
use settings::SettingsStorage;
use wifi::WifiValidator;

const PWM_FREQUENCY_HZ: u32 = 32_000;

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

    let mut settings = SettingsStorage::new();
    let boot = load_boot_snapshot(&mut settings);

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(160))
        .expect("failed to configure MCPWM clock");
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm.operator2.set_timer(&mcpwm.timer0);
    let (uh, ul) = mcpwm.operator0.with_pins(
        peripherals.GPIO16,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO17,
        low_side_pwm_config(),
    );
    let (vh, vl) = mcpwm.operator1.with_pins(
        peripherals.GPIO18,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO23,
        low_side_pwm_config(),
    );
    let (wh, wl) = mcpwm.operator2.with_pins(
        peripherals.GPIO19,
        PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
        peripherals.GPIO33,
        low_side_pwm_config(),
    );
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            PWM_PERIOD_TICKS,
            PwmWorkingMode::UpDown,
            Rate::from_hz(PWM_FREQUENCY_HZ),
        )
        .expect("failed to configure MCPWM timer");
    mcpwm.timer0.start(timer_clock_cfg);
    let motor_drive = PwmMotorDrive::new(peripherals.GPIO5, uh, ul, vh, vl, wh, wl);

    let i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let timer_group = TimerGroup::new(peripherals.TIMG0);
    let wifi_validator = WifiValidator::new(
        timer_group.timer0,
        Rng::new(peripherals.RNG),
        peripherals.WIFI,
    )
    .expect("failed to initialize Wi-Fi validator");

    let platform = FirmwarePlatform::new(
        serial,
        delay,
        settings,
        wifi_validator,
        current_sensor,
        motor_drive,
        i2c,
        boot.geometry,
        boot.runtime_config.dt.get::<uom::si::time::second>() as f32,
    );
    let mut runtime = FirmwareRuntime::new(platform, boot);
    runtime.run()
}
