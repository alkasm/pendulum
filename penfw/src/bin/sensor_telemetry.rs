#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../board_init.rs"]
mod board_init;
#[path = "../hw/mod.rs"]
mod hw;

use board_init::{
    HALL_SENSOR_ADDR, i2c_device_present, init_console, init_delay, init_primary_i2c,
    max_clock_config, write_bytes,
};
use esp_hal::{Blocking, i2c::master::I2c, main};
use hw::{CurrentSensor, GY521_DEFAULT_I2C_ADDR, Tmc6300};
use libm::atan2f;
use pendulum_lib::{
    CurrentTelemetry, HallMeasurement, HallTelemetry, ImuMeasurement, ImuTelemetry,
    SensorTelemetryFrame, TelemetryPacket,
};

const BASELINE_SAMPLES: u32 = 16;
const SAMPLE_PERIOD_MS: u32 = 100;
const SENSOR_FRAME_BUF_LEN: usize = 512;

const TMAG5273_REG_DEVICE_CONFIG_1: u8 = 0x00;
const TMAG5273_REG_DEVICE_CONFIG_2: u8 = 0x01;
const TMAG5273_REG_SENSOR_CONFIG_1: u8 = 0x02;
const TMAG5273_REG_SENSOR_CONFIG_2: u8 = 0x03;
const TMAG5273_REG_T_CONFIG: u8 = 0x07;
const TMAG5273_REG_T_MSB_RESULT: u8 = 0x10;
const TMAG5273_RANGE_MT: f32 = 80.0;
const TMAG5273_TEMP_SENSE_T0_C: f32 = 25.0;
const TMAG5273_TEMP_ADC_T0: i16 = 17_508;
const TMAG5273_TEMP_ADC_RES: f32 = 60.1;

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
    let mut current_sensor = CurrentSensor::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
    );
    let mut motor_driver = Tmc6300::new(
        peripherals.GPIO5,
        peripherals.GPIO34,
        peripherals.GPIO16,
        peripherals.GPIO17,
        peripherals.GPIO18,
        peripherals.GPIO23,
        peripherals.GPIO19,
        peripherals.GPIO33,
    );
    let mut i2c = init_primary_i2c(peripherals.I2C0, peripherals.GPIO21, peripherals.GPIO22);
    let mut frame_buf = [0_u8; SENSOR_FRAME_BUF_LEN];
    let mut seq = 0_u32;
    let mut hall_configured = false;
    let mut imu_verified = false;
    let mut imu_awake = false;

    current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    motor_driver.disable();

    loop {
        let current_sample = current_sensor.read();
        let hall = read_hall_telemetry(&mut i2c, &mut hall_configured);
        let imu = read_imu_telemetry(&mut i2c, &mut imu_verified, &mut imu_awake);

        let frame = SensorTelemetryFrame {
            seq,
            uptime_ms: seq.saturating_mul(SAMPLE_PERIOD_MS),
            motor_driver_diag_high: motor_driver.diag_is_high(),
            current: CurrentTelemetry {
                mcp6021_counts: current_sample.mcp6021.counts,
                mcp6021_delta_counts: current_sample.mcp6021.delta_counts,
                mcp6021_volts: current_sample.mcp6021.volts,
                ina_u_counts: current_sample.ina_u.counts,
                ina_u_delta_counts: current_sample.ina_u.delta_counts,
                ina_u_amps: current_sample.ina_u.amps,
                ina_v_counts: current_sample.ina_v.counts,
                ina_v_delta_counts: current_sample.ina_v.delta_counts,
                ina_v_amps: current_sample.ina_v.amps,
                ina_w_counts: current_sample.ina_w.counts,
                ina_w_delta_counts: current_sample.ina_w.delta_counts,
                ina_w_amps: current_sample.ina_w.amps,
            },
            hall,
            imu,
        };
        let packet = TelemetryPacket::Sensor(frame);

        if let Ok(encoded) = postcard::to_slice_cobs(&packet, &mut frame_buf) {
            write_bytes(&mut serial, encoded);
        }

        seq = seq.wrapping_add(1);
        delay.delay_millis(SAMPLE_PERIOD_MS);
    }
}

fn read_hall_telemetry(i2c: &mut I2c<'_, Blocking>, hall_configured: &mut bool) -> HallTelemetry {
    if !i2c_device_present(i2c, HALL_SENSOR_ADDR) {
        *hall_configured = false;
        return HallTelemetry::Missing;
    }

    if !*hall_configured {
        match tmag5273_configure_default(i2c, HALL_SENSOR_ADDR) {
            Ok(()) => *hall_configured = true,
            Err(register) => return HallTelemetry::ConfigError { register },
        }
    }

    match tmag5273_read_measurement(i2c, HALL_SENSOR_ADDR) {
        Ok(measurement) => HallTelemetry::Measurement(measurement),
        Err(register) => HallTelemetry::ReadError { register },
    }
}

fn read_imu_telemetry(
    i2c: &mut I2c<'_, Blocking>,
    imu_verified: &mut bool,
    imu_awake: &mut bool,
) -> ImuTelemetry {
    if !i2c_device_present(i2c, GY521_DEFAULT_I2C_ADDR) {
        *imu_verified = false;
        *imu_awake = false;
        return ImuTelemetry::Missing;
    }

    if !*imu_verified {
        match imu_verify(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_verified = true,
            Err(ImuProbeError::RegisterRead) => return ImuTelemetry::Missing,
            Err(ImuProbeError::UnexpectedWhoAmI(value)) => {
                return ImuTelemetry::UnexpectedWhoAmI { value };
            }
        }
    }

    if !*imu_awake {
        match imu_wake(i2c, GY521_DEFAULT_I2C_ADDR) {
            Ok(()) => *imu_awake = true,
            Err(register) => return ImuTelemetry::WakeError { register },
        }
    }

    match imu_read_measurement(i2c, GY521_DEFAULT_I2C_ADDR) {
        Ok(measurement) => ImuTelemetry::Measurement(measurement),
        Err(register) => ImuTelemetry::ReadError { register },
    }
}

fn tmag5273_configure_default(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<(), u8> {
    tmag5273_update_register(i2c, address, TMAG5273_REG_DEVICE_CONFIG_1, |value| {
        value & !0x03
    })?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_DEVICE_CONFIG_2, |value| {
        (value & !0x17) | 0x02
    })?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_SENSOR_CONFIG_1, |value| {
        (value & !0xF0) | 0x70
    })?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_SENSOR_CONFIG_2, |value| {
        (value & !0x0F) | 0x07
    })?;
    tmag5273_update_register(i2c, address, TMAG5273_REG_T_CONFIG, |value| value | 0x01)?;
    Ok(())
}

fn tmag5273_update_register(
    i2c: &mut I2c<'_, Blocking>,
    address: u8,
    register: u8,
    update: impl FnOnce(u8) -> u8,
) -> Result<(), u8> {
    let mut value = [0_u8; 1];
    i2c.write_read(address, &[register], &mut value)
        .map_err(|_| register)?;
    let updated = update(value[0]);
    i2c.write(address, &[register, updated])
        .map_err(|_| register)?;
    Ok(())
}

fn tmag5273_read_measurement(
    i2c: &mut I2c<'_, Blocking>,
    address: u8,
) -> Result<HallMeasurement, u8> {
    let mut buffer = [0_u8; 13];
    i2c.write_read(address, &[TMAG5273_REG_T_MSB_RESULT], &mut buffer)
        .map_err(|_| TMAG5273_REG_T_MSB_RESULT)?;

    Ok(HallMeasurement {
        temperature_c: decode_temperature_c(buffer[0], buffer[1]),
        x_mt: decode_magnetic_mt(buffer[2], buffer[3], TMAG5273_RANGE_MT),
        y_mt: decode_magnetic_mt(buffer[4], buffer[5], TMAG5273_RANGE_MT),
        z_mt: decode_magnetic_mt(buffer[6], buffer[7], TMAG5273_RANGE_MT),
        angle_deg: decode_angle_deg(buffer[9], buffer[10]),
        magnitude: buffer[11],
        set_count: (buffer[8] >> 5) & 0x07,
        result_ready: (buffer[8] & 0x01) != 0,
        por: (buffer[8] & 0x04) != 0,
        diag_fail: (buffer[8] & 0x02) != 0,
        int_pin_high: (buffer[12] & 0x10) != 0,
        oscillator_error: (buffer[12] & 0x08) != 0,
        int_pin_error: (buffer[12] & 0x04) != 0,
        otp_crc_error: (buffer[12] & 0x02) != 0,
        vcc_uv_error: (buffer[12] & 0x01) != 0,
    })
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

fn imu_read_measurement(i2c: &mut I2c<'_, Blocking>, address: u8) -> Result<ImuMeasurement, u8> {
    let mut buffer = [0_u8; 14];
    i2c.write_read(address, &[MPU_REG_ACCEL_XOUT_H], &mut buffer)
        .map_err(|_| MPU_REG_ACCEL_XOUT_H)?;

    let ax_raw = i16::from_be_bytes([buffer[0], buffer[1]]);
    let ay_raw = i16::from_be_bytes([buffer[2], buffer[3]]);
    let az_raw = i16::from_be_bytes([buffer[4], buffer[5]]);
    let temp_raw = i16::from_be_bytes([buffer[6], buffer[7]]);
    let gx_raw = i16::from_be_bytes([buffer[8], buffer[9]]);
    let gy_raw = i16::from_be_bytes([buffer[10], buffer[11]]);
    let gz_raw = i16::from_be_bytes([buffer[12], buffer[13]]);

    let ax_g = ax_raw as f32 / ACCEL_LSB_PER_G;
    let ay_g = ay_raw as f32 / ACCEL_LSB_PER_G;
    let az_g = az_raw as f32 / ACCEL_LSB_PER_G;
    let gx_dps = gx_raw as f32 / GYRO_LSB_PER_DPS;
    let gy_dps = gy_raw as f32 / GYRO_LSB_PER_DPS;
    let gz_dps = gz_raw as f32 / GYRO_LSB_PER_DPS;
    let temperature_c = temp_raw as f32 / 340.0 + 36.53;
    let theta_deg = atan2f(ax_g, az_g) * (180.0 / core::f32::consts::PI);

    Ok(ImuMeasurement {
        ax_g,
        ay_g,
        az_g,
        gx_dps,
        gy_dps,
        gz_dps,
        temperature_c,
        theta_deg,
    })
}

fn decode_temperature_c(msb: u8, lsb: u8) -> f32 {
    let raw = i16::from_be_bytes([msb, lsb]);
    TMAG5273_TEMP_SENSE_T0_C + ((raw - TMAG5273_TEMP_ADC_T0) as f32 / TMAG5273_TEMP_ADC_RES)
}

fn decode_magnetic_mt(msb: u8, lsb: u8, range_mt: f32) -> f32 {
    let raw = i16::from_be_bytes([msb, lsb]) as f32;
    (-range_mt * raw) / 32_768.0
}

fn decode_angle_deg(msb: u8, lsb: u8) -> f32 {
    let raw = u16::from_be_bytes([msb, lsb]);
    let integer = ((raw >> 4) & 0x01FF) as f32;
    let fraction = (raw & 0x000F) as f32 / 16.0;
    integer + fraction
}
