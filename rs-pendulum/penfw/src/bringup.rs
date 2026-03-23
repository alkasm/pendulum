#![allow(dead_code)]

use core::fmt::Write;

use esp_hal::{
    Blocking,
    clock::CpuClock,
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    peripherals,
    time::Rate,
    uart::{Config as UartConfig, Uart},
};

pub const HALL_SENSOR_ADDR: u8 = 0x35;
pub const MPU6050_ADDR_PRIMARY: u8 = 0x68;
pub const MPU6050_ADDR_ALTERNATE: u8 = 0x69;
pub const MPU6050_WHO_AM_I_REG: u8 = 0x75;

pub fn max_clock_config() -> esp_hal::Config {
    esp_hal::Config::default().with_cpu_clock(CpuClock::max())
}

pub fn init_console<'d>(
    uart0: peripherals::UART0<'d>,
    tx: peripherals::GPIO1<'d>,
    rx: peripherals::GPIO3<'d>,
) -> Uart<'d, Blocking> {
    Uart::new(uart0, UartConfig::default())
        .expect("UART0 init failed")
        .with_tx(tx)
        .with_rx(rx)
}

pub fn init_primary_i2c<'d>(
    i2c0: peripherals::I2C0<'d>,
    sda: peripherals::GPIO21<'d>,
    scl: peripherals::GPIO22<'d>,
) -> I2c<'d, Blocking> {
    I2c::new(i2c0, I2cConfig::default().with_frequency(Rate::from_khz(100)))
        .expect("I2C0 init failed")
        .with_sda(sda)
        .with_scl(scl)
}

pub fn init_delay() -> Delay {
    Delay::new()
}

pub fn write_line(serial: &mut Uart<'_, Blocking>, line: &str) {
    let _ = serial.write_str(line);
    let _ = serial.write_str("\r\n");
    let _ = serial.flush();
}

pub fn write_fmt_line(serial: &mut Uart<'_, Blocking>, args: core::fmt::Arguments<'_>) {
    let _ = serial.write_fmt(args);
    let _ = serial.write_str("\r\n");
    let _ = serial.flush();
}

pub fn write_bytes(serial: &mut Uart<'_, Blocking>, mut bytes: &[u8]) {
    while !bytes.is_empty() {
        match serial.write(bytes) {
            Ok(written) if written > 0 => bytes = &bytes[written..],
            _ => {}
        }
    }
    let _ = serial.flush();
}

pub fn i2c_device_present(i2c: &mut I2c<'_, Blocking>, address: u8) -> bool {
    let mut probe = [0_u8; 1];
    i2c.read(address, &mut probe).is_ok()
}
