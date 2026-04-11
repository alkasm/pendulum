use libm::atan2f;

use esp_hal::{
    Blocking,
    i2c::master::{Config as I2cConfig, I2c},
    peripherals::{GPIO21, GPIO22, I2C0},
    time::Rate,
};

const MPU_REG_ACCEL_XOUT_H: u8 = 0x3B;
const MPU_REG_CONFIG: u8 = 0x1A;
const MPU_REG_GYRO_CONFIG: u8 = 0x1B;
const MPU_REG_ACCEL_CONFIG: u8 = 0x1C;
const MPU_REG_PWR_MGMT_1: u8 = 0x6B;
const MPU_REG_WHO_AM_I: u8 = 0x75;

pub const GY521_DEFAULT_I2C_ADDR: u8 = 0x68;

const MPU6050_WHO_AM_I_VALUE: u8 = 0x68;

const ACCEL_LSB_PER_G: f32 = 8_192.0;
const GYRO_LSB_PER_DPS: f32 = 32.8;

pub struct Gy521Imu<'d> {
    i2c: I2c<'d, Blocking>,
    address: u8,
}

#[derive(Clone, Copy)]
pub enum Gy521Error {
    RegisterRead(u8),
    UnexpectedWhoAmI(u8),
}

#[derive(Clone, Copy)]
pub struct Gy521Measurement {
    pub ax_g: f32,
    pub ay_g: f32,
    pub az_g: f32,
    pub gx_dps: f32,
    pub gy_dps: f32,
    pub gz_dps: f32,
    pub temperature_c: f32,
    pub theta_deg: f32,
}

impl<'d> Gy521Imu<'d> {
    pub fn new(i2c0: I2C0<'d>, sda: GPIO21<'d>, scl: GPIO22<'d>) -> Result<Self, Gy521Error> {
        Self::new_with_address(i2c0, sda, scl, GY521_DEFAULT_I2C_ADDR)
    }

    pub fn new_with_address(
        i2c0: I2C0<'d>,
        sda: GPIO21<'d>,
        scl: GPIO22<'d>,
        address: u8,
    ) -> Result<Self, Gy521Error> {
        let mut i2c = I2c::new(
            i2c0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .expect("I2C0 init failed")
        .with_sda(sda)
        .with_scl(scl);
        let mut who_am_i = [0_u8; 1];
        i2c.write_read(address, &[MPU_REG_WHO_AM_I], &mut who_am_i)
            .map_err(|_| Gy521Error::RegisterRead(MPU_REG_WHO_AM_I))?;
        if who_am_i[0] != MPU6050_WHO_AM_I_VALUE {
            return Err(Gy521Error::UnexpectedWhoAmI(who_am_i[0]));
        }

        Ok(Self { i2c, address })
    }

    pub fn wake(&mut self) -> Result<(), u8> {
        self.i2c
            .write(self.address, &[MPU_REG_PWR_MGMT_1, 0x01])
            .map_err(|_| MPU_REG_PWR_MGMT_1)?;
        self.i2c
            .write(self.address, &[MPU_REG_CONFIG, 0x03])
            .map_err(|_| MPU_REG_CONFIG)?;
        self.i2c
            .write(self.address, &[MPU_REG_GYRO_CONFIG, 0x10])
            .map_err(|_| MPU_REG_GYRO_CONFIG)?;
        self.i2c
            .write(self.address, &[MPU_REG_ACCEL_CONFIG, 0x08])
            .map_err(|_| MPU_REG_ACCEL_CONFIG)?;
        Ok(())
    }

    pub fn read_measurement(&mut self) -> Result<Gy521Measurement, u8> {
        let mut buffer = [0_u8; 14];
        self.i2c
            .write_read(self.address, &[MPU_REG_ACCEL_XOUT_H], &mut buffer)
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
        let theta_rad = atan2f(ax_g, az_g);
        let theta_deg = theta_rad * (180.0 / core::f32::consts::PI);

        Ok(Gy521Measurement {
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
}
