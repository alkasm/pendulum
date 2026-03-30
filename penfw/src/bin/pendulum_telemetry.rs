#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;
#[path = "../hw/mod.rs"]
mod hw;

use bringup::{
    i2c_device_present, init_console, init_delay, init_primary_i2c, max_clock_config, write_bytes,
};
use esp_hal::{Blocking, i2c::master::I2c, main};
use libm::atan2f;
use penproto::{
    PendulumEstimateMeasurement, PendulumEstimateTelemetry, PendulumTelemetryFrame,
    TelemetryPacket,
};

use hw::GY521_DEFAULT_I2C_ADDR;

const SAMPLE_PERIOD_MS: u32 = 20;
const FRAME_BUF_LEN: usize = 128;

const MPU_REG_ACCEL_XOUT_H: u8 = 0x3B;
const MPU_REG_PWR_MGMT_1: u8 = 0x6B;
const MPU_REG_WHO_AM_I: u8 = 0x75;
const MPU6050_WHO_AM_I_VALUE: u8 = 0x68;
const ACCEL_LSB_PER_G: f32 = 16_384.0;
const GYRO_LSB_PER_DPS: f32 = 131.0;

#[derive(Clone, Copy)]
enum ImuProbeError {
    RegisterRead,
    UnexpectedWhoAmI(u8),
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let mut i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let mut frame_buf = [0_u8; FRAME_BUF_LEN];
    let mut seq = 0_u32;
    let mut imu_verified = false;
    let mut imu_awake = false;

    loop {
        let estimate = read_pendulum_estimate(&mut i2c, &mut imu_verified, &mut imu_awake);
        let frame = PendulumTelemetryFrame {
            seq,
            uptime_ms: seq.saturating_mul(SAMPLE_PERIOD_MS),
            estimate,
        };
        let packet = TelemetryPacket::Pendulum(frame);

        if let Ok(encoded) = postcard::to_slice_cobs(&packet, &mut frame_buf) {
            write_bytes(&mut serial, encoded);
        }

        seq = seq.wrapping_add(1);
        delay.delay_millis(SAMPLE_PERIOD_MS);
    }
}

fn read_pendulum_estimate(
    i2c: &mut I2c<'_, Blocking>,
    imu_verified: &mut bool,
    imu_awake: &mut bool,
) -> PendulumEstimateTelemetry {
    if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
        *imu_verified = false;
        *imu_awake = false;
        return PendulumEstimateTelemetry::Missing;
    }

    if !*imu_verified {
        match imu_verify(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_verified = true,
            Err(ImuProbeError::RegisterRead) => return PendulumEstimateTelemetry::Missing,
            Err(ImuProbeError::UnexpectedWhoAmI(value)) => {
                return PendulumEstimateTelemetry::UnexpectedWhoAmI { value };
            }
        }
    }

    if !*imu_awake {
        match imu_wake(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_awake = true,
            Err(register) => return PendulumEstimateTelemetry::WakeError { register },
        }
    }

    match imu_read_pendulum_measurement(i2c, GY521_DEFAULT_I2C_ADDR) {
        Ok(measurement) => PendulumEstimateTelemetry::Measurement(measurement),
        Err(register) => PendulumEstimateTelemetry::ReadError { register },
    }
}

fn imu_verify(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), ImuProbeError> {
    let mut who_am_i = [0_u8; 1];
    i2c.write_read(address, &[MPU_REG_WHO_AM_I], &mut who_am_i)
        .map_err(|_| ImuProbeError::RegisterRead)?;
    if who_am_i[0] != MPU6050_WHO_AM_I_VALUE {
        return Err(ImuProbeError::UnexpectedWhoAmI(who_am_i[0]));
    }
    Ok(())
}

fn imu_wake(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), u8> {
    i2c.write(address, &[MPU_REG_PWR_MGMT_1, 0x01])
        .map_err(|_| MPU_REG_PWR_MGMT_1)
}

fn imu_read_pendulum_measurement(
    i2c: &mut I2c<'_, Blocking>,
    address: u8,
) -> Result<PendulumEstimateMeasurement, u8> {
    let mut buffer = [0_u8; 14];
    i2c.write_read(address, &[MPU_REG_ACCEL_XOUT_H], &mut buffer)
        .map_err(|_| MPU_REG_ACCEL_XOUT_H)?;

    let ax_raw = i16::from_be_bytes([buffer[0], buffer[1]]);
    let ay_raw = i16::from_be_bytes([buffer[2], buffer[3]]);
    let gz_raw = i16::from_be_bytes([buffer[12], buffer[13]]);

    let ax_g = ax_raw as f32 / ACCEL_LSB_PER_G;
    let ay_g = ay_raw as f32 / ACCEL_LSB_PER_G;
    let gz_dps = gz_raw as f32 / GYRO_LSB_PER_DPS;

    // Front-view body frame convention:
    // +x = right, +y = up, +z = toward the viewer.
    //
    // Measured IMU mounting:
    // - IMU X points down
    // - IMU Y points right
    // - IMU Z points toward the viewer
    //
    // So in body coordinates:
    // - body_x = imu_y
    // - body_y = -imu_x
    let body_x_g = ay_g;
    let body_y_g = -ax_g;

    // Upright should report theta = 0 deg.
    let theta_deg = atan2f(-body_x_g, body_y_g) * (180.0 / core::f32::consts::PI);

    // Positive theta means leaning farther to the right in the front view.
    // With +z toward the viewer, that is the negative right-hand rotation
    // about z, so theta_dot uses the negated IMU z gyro.
    let theta_dot_dps = -gz_dps;

    Ok(PendulumEstimateMeasurement {
        theta_deg,
        theta_dot_dps,
    })
}
