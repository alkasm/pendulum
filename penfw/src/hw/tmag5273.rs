use esp_hal::{
    Blocking,
    i2c::master::{Config as I2cConfig, I2c},
    peripherals::{GPIO21, GPIO22, I2C0},
    time::Rate,
};

const TMAG5273_REG_DEVICE_CONFIG_1: u8 = 0x00;
const TMAG5273_REG_DEVICE_CONFIG_2: u8 = 0x01;
const TMAG5273_REG_SENSOR_CONFIG_1: u8 = 0x02;
const TMAG5273_REG_SENSOR_CONFIG_2: u8 = 0x03;
const TMAG5273_REG_T_CONFIG: u8 = 0x07;
const TMAG5273_REG_I2C_ADDRESS: u8 = 0x0C;
const TMAG5273_REG_T_MSB_RESULT: u8 = 0x10;

const TMAG5273_RANGE_MT: f32 = 80.0;
const TMAG5273_TEMP_SENSE_T0_C: f32 = 25.0;
const TMAG5273_TEMP_ADC_T0: i16 = 17_508;
const TMAG5273_TEMP_ADC_RES: f32 = 60.1;

pub struct Tmag5273<'d> {
    i2c: I2c<'d, Blocking>,
    address: u8,
}

#[derive(Clone, Copy)]
pub struct Tmag5273Identity {
    pub address: u8,
    pub device_id: u8,
    pub manufacturer_id: u16,
}

#[derive(Clone, Copy)]
pub struct Tmag5273ConvStatus {
    pub set_count: u8,
    pub por: bool,
    pub diag_fail: bool,
    pub result_ready: bool,
}

#[derive(Clone, Copy)]
pub struct Tmag5273DeviceStatus {
    pub int_pin_high: bool,
    pub oscillator_error: bool,
    pub int_pin_error: bool,
    pub otp_crc_error: bool,
    pub vcc_uv_error: bool,
}

#[derive(Clone, Copy)]
pub struct Tmag5273Measurement {
    pub temperature_c: f32,
    pub x_mt: f32,
    pub y_mt: f32,
    pub z_mt: f32,
    pub angle_deg: f32,
    pub magnitude: u8,
    pub conv_status: Tmag5273ConvStatus,
    pub device_status: Tmag5273DeviceStatus,
}

impl<'d> Tmag5273<'d> {
    pub fn new(i2c0: I2C0<'d>, sda: GPIO21<'d>, scl: GPIO22<'d>, address: u8) -> Self {
        let i2c = I2c::new(
            i2c0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .expect("I2C0 init failed")
        .with_sda(sda)
        .with_scl(scl);
        Self { i2c, address }
    }

    pub fn is_present(&mut self) -> bool {
        let mut probe = [0_u8; 1];
        self.i2c.read(self.address, &mut probe).is_ok()
    }

    pub fn read_identity(&mut self) -> Result<Tmag5273Identity, u8> {
        let mut identity = [0_u8; 4];
        self.i2c
            .write_read(self.address, &[TMAG5273_REG_I2C_ADDRESS], &mut identity)
            .map_err(|_| TMAG5273_REG_I2C_ADDRESS)?;

        Ok(Tmag5273Identity {
            address: identity[0] >> 1,
            device_id: identity[1] & 0x03,
            manufacturer_id: (u16::from(identity[3]) << 8) | u16::from(identity[2]),
        })
    }

    pub fn configure_default(&mut self) -> Result<(), u8> {
        self.update_register(TMAG5273_REG_DEVICE_CONFIG_1, |value| value & !0x03)?;
        self.update_register(TMAG5273_REG_DEVICE_CONFIG_2, |value| (value & !0x17) | 0x02)?;
        self.update_register(TMAG5273_REG_SENSOR_CONFIG_1, |value| (value & !0xF0) | 0x70)?;
        self.update_register(TMAG5273_REG_SENSOR_CONFIG_2, |value| (value & !0x0F) | 0x07)?;
        self.update_register(TMAG5273_REG_T_CONFIG, |value| value | 0x01)?;
        Ok(())
    }

    pub fn read_measurement(&mut self) -> Result<Tmag5273Measurement, u8> {
        let mut buffer = [0_u8; 13];
        self.i2c
            .write_read(self.address, &[TMAG5273_REG_T_MSB_RESULT], &mut buffer)
            .map_err(|_| TMAG5273_REG_T_MSB_RESULT)?;

        Ok(Tmag5273Measurement {
            temperature_c: decode_temperature_c(buffer[0], buffer[1]),
            x_mt: decode_magnetic_mt(buffer[2], buffer[3], TMAG5273_RANGE_MT),
            y_mt: decode_magnetic_mt(buffer[4], buffer[5], TMAG5273_RANGE_MT),
            z_mt: decode_magnetic_mt(buffer[6], buffer[7], TMAG5273_RANGE_MT),
            conv_status: decode_conv_status(buffer[8]),
            angle_deg: decode_angle_deg(buffer[9], buffer[10]),
            magnitude: buffer[11],
            device_status: decode_device_status(buffer[12]),
        })
    }

    fn update_register(&mut self, register: u8, update: impl FnOnce(u8) -> u8) -> Result<(), u8> {
        let mut value = [0_u8; 1];
        self.i2c
            .write_read(self.address, &[register], &mut value)
            .map_err(|_| register)?;
        let updated = update(value[0]);
        self.i2c
            .write(self.address, &[register, updated])
            .map_err(|_| register)?;
        Ok(())
    }
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

fn decode_conv_status(raw: u8) -> Tmag5273ConvStatus {
    Tmag5273ConvStatus {
        set_count: (raw >> 5) & 0x07,
        por: (raw & 0x10) != 0,
        diag_fail: (raw & 0x02) != 0,
        result_ready: (raw & 0x01) != 0,
    }
}

fn decode_device_status(raw: u8) -> Tmag5273DeviceStatus {
    Tmag5273DeviceStatus {
        int_pin_high: (raw & 0x10) != 0,
        oscillator_error: (raw & 0x08) != 0,
        int_pin_error: (raw & 0x04) != 0,
        otp_crc_error: (raw & 0x02) != 0,
        vcc_uv_error: (raw & 0x01) != 0,
    }
}
