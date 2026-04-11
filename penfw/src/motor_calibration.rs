use embedded_storage::{
    ReadStorage,
    Storage,
};
use esp_hal::peripherals::FLASH;
use esp_storage::{
    FlashStorage,
    FlashStorageError,
};

const NVS_PARTITION_OFFSET: u32 = 0x9000;
const CALIBRATION_RECORD_OFFSET: u32 = NVS_PARTITION_OFFSET;
const CALIBRATION_RECORD_WORDS: usize = 6;
const CALIBRATION_RECORD_BYTES: usize = CALIBRATION_RECORD_WORDS * 4;
const CALIBRATION_MAGIC: u32 = u32::from_le_bytes(*b"MCAL");
const CALIBRATION_VERSION: u32 = 1;

#[derive(Clone, Copy)]
pub struct StoredMotorCalibration {
    pub direction_sign: f32,
    pub electrical_offset_deg: f32,
    pub torque_sign: f32,
}

#[allow(dead_code)]
pub fn load_motor_calibration(
    flash: FLASH<'_>,
) -> Result<Option<StoredMotorCalibration>, FlashStorageError> {
    let mut storage = FlashStorage::new(flash).multicore_auto_park();
    let mut bytes = [0_u8; CALIBRATION_RECORD_BYTES];
    storage.read(CALIBRATION_RECORD_OFFSET, &mut bytes)?;

    let words = decode_words(bytes);
    if words[0] != CALIBRATION_MAGIC || words[1] != CALIBRATION_VERSION {
        return Ok(None);
    }
    if words[5] != checksum(words[0], words[1], words[2], words[3], words[4]) {
        return Ok(None);
    }

    let direction_sign = f32::from_bits(words[2]);
    let electrical_offset_deg = f32::from_bits(words[3]);
    let torque_sign = f32::from_bits(words[4]);

    if !direction_sign.is_finite()
        || !electrical_offset_deg.is_finite()
        || !torque_sign.is_finite()
    {
        return Ok(None);
    }

    Ok(Some(StoredMotorCalibration {
        direction_sign: normalize_sign(direction_sign),
        electrical_offset_deg: wrap_degrees(electrical_offset_deg),
        torque_sign: normalize_sign(torque_sign),
    }))
}

#[allow(dead_code)]
pub fn save_motor_calibration(
    flash: FLASH<'_>,
    calibration: StoredMotorCalibration,
) -> Result<(), FlashStorageError> {
    let mut storage = FlashStorage::new(flash).multicore_auto_park();
    let words = [
        CALIBRATION_MAGIC,
        CALIBRATION_VERSION,
        normalize_sign(calibration.direction_sign).to_bits(),
        wrap_degrees(calibration.electrical_offset_deg).to_bits(),
        normalize_sign(calibration.torque_sign).to_bits(),
        checksum(
            CALIBRATION_MAGIC,
            CALIBRATION_VERSION,
            normalize_sign(calibration.direction_sign).to_bits(),
            wrap_degrees(calibration.electrical_offset_deg).to_bits(),
            normalize_sign(calibration.torque_sign).to_bits(),
        ),
    ];
    storage.write(CALIBRATION_RECORD_OFFSET, &encode_words(words))?;
    Ok(())
}

#[allow(dead_code)]
fn encode_words(words: [u32; CALIBRATION_RECORD_WORDS]) -> [u8; CALIBRATION_RECORD_BYTES] {
    let mut bytes = [0_u8; CALIBRATION_RECORD_BYTES];
    let mut index = 0;
    while index < CALIBRATION_RECORD_WORDS {
        let offset = index * 4;
        bytes[offset..offset + 4].copy_from_slice(&words[index].to_le_bytes());
        index += 1;
    }
    bytes
}

#[allow(dead_code)]
fn decode_words(bytes: [u8; CALIBRATION_RECORD_BYTES]) -> [u32; CALIBRATION_RECORD_WORDS] {
    let mut words = [0_u32; CALIBRATION_RECORD_WORDS];
    let mut index = 0;
    while index < CALIBRATION_RECORD_WORDS {
        let offset = index * 4;
        words[index] = u32::from_le_bytes([
            bytes[offset],
            bytes[offset + 1],
            bytes[offset + 2],
            bytes[offset + 3],
        ]);
        index += 1;
    }
    words
}

fn checksum(a: u32, b: u32, c: u32, d: u32, e: u32) -> u32 {
    a ^ b ^ c ^ d ^ e ^ 0x6d74_636c
}

fn normalize_sign(value: f32) -> f32 {
    if value.is_sign_negative() { -1.0 } else { 1.0 }
}

fn wrap_degrees(angle_deg: f32) -> f32 {
    let mut wrapped = angle_deg;
    while wrapped >= 360.0 {
        wrapped -= 360.0;
    }
    while wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
}
