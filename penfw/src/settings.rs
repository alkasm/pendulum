use embedded_storage::{ReadStorage, Storage};
use esp_storage::{FlashStorage, FlashStorageError};
use pendulum_lib::{
    StoredDeviceConfig, StoredMotorCalibration,
    settings_record::{
        RecordEncodeError, RecordLoad, SETTINGS_SLOT_SIZE, decode_record, encode_record,
    },
};

const SETTINGS_BASE_OFFSET: u32 = 0x9000;
const DEVICE_CONFIG_OFFSET: u32 = SETTINGS_BASE_OFFSET;
const MOTOR_CALIBRATION_OFFSET: u32 = SETTINGS_BASE_OFFSET + SETTINGS_SLOT_SIZE as u32;
const _CONTROL_CONFIG_OFFSET: u32 = SETTINGS_BASE_OFFSET + 2 * SETTINGS_SLOT_SIZE as u32;

const DEVICE_CONFIG_MAGIC: u32 = u32::from_le_bytes(*b"DCFG");
const MOTOR_CALIBRATION_MAGIC: u32 = u32::from_le_bytes(*b"MCAL");
const DEVICE_CONFIG_RECORD_VERSION: u16 = 2;
const MOTOR_CALIBRATION_RECORD_VERSION: u16 = 1;

#[derive(Debug)]
pub enum SettingsError {
    Flash(FlashStorageError),
    Encode(RecordEncodeError),
}

impl core::fmt::Display for SettingsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(error) => write!(f, "flash error: {error:?}"),
            Self::Encode(error) => write!(f, "record encode error: {error:?}"),
        }
    }
}

impl From<FlashStorageError> for SettingsError {
    fn from(value: FlashStorageError) -> Self {
        Self::Flash(value)
    }
}

impl From<RecordEncodeError> for SettingsError {
    fn from(value: RecordEncodeError) -> Self {
        Self::Encode(value)
    }
}

pub struct SettingsService {
    flash: FlashStorage,
}

impl SettingsService {
    pub fn new() -> Self {
        Self {
            flash: FlashStorage::new(),
        }
    }

    pub fn load_device_config(&mut self) -> Result<RecordLoad<StoredDeviceConfig>, SettingsError> {
        let mut slot = [0_u8; SETTINGS_SLOT_SIZE];
        self.flash.read(DEVICE_CONFIG_OFFSET, &mut slot)?;
        Ok(decode_record(
            &slot,
            DEVICE_CONFIG_MAGIC,
            DEVICE_CONFIG_RECORD_VERSION,
        ))
    }

    pub fn save_device_config(&mut self, config: &StoredDeviceConfig) -> Result<(), SettingsError> {
        self.write_record(
            DEVICE_CONFIG_OFFSET,
            DEVICE_CONFIG_MAGIC,
            DEVICE_CONFIG_RECORD_VERSION,
            config,
        )
    }

    pub fn load_motor_calibration_record(
        &mut self,
    ) -> Result<RecordLoad<StoredMotorCalibration>, SettingsError> {
        self.read_record(
            MOTOR_CALIBRATION_OFFSET,
            MOTOR_CALIBRATION_MAGIC,
            MOTOR_CALIBRATION_RECORD_VERSION,
        )
    }

    pub fn save_motor_calibration(
        &mut self,
        calibration: &StoredMotorCalibration,
    ) -> Result<(), SettingsError> {
        self.write_record(
            MOTOR_CALIBRATION_OFFSET,
            MOTOR_CALIBRATION_MAGIC,
            MOTOR_CALIBRATION_RECORD_VERSION,
            calibration,
        )
    }

    fn read_record<T>(
        &mut self,
        offset: u32,
        magic: u32,
        version: u16,
    ) -> Result<RecordLoad<T>, SettingsError>
    where
        T: for<'de> serde::Deserialize<'de>,
    {
        let mut slot = [0_u8; SETTINGS_SLOT_SIZE];
        self.flash.read(offset, &mut slot)?;
        Ok(decode_record(&slot, magic, version))
    }

    fn write_record<T>(
        &mut self,
        offset: u32,
        magic: u32,
        version: u16,
        value: &T,
    ) -> Result<(), SettingsError>
    where
        T: serde::Serialize,
    {
        let mut slot = [0_u8; SETTINGS_SLOT_SIZE];
        encode_record(&mut slot, magic, version, value)?;
        self.flash.write(offset, &slot)?;
        Ok(())
    }
}
