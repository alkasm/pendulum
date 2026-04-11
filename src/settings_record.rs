use serde::{Deserialize, Serialize};

pub const SETTINGS_SLOT_SIZE: usize = 4096;
const HEADER_SIZE: usize = 12;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecordLoad<T> {
    Missing,
    Valid(T),
    Corrupt,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecordEncodeError {
    BufferTooSmall,
    Serialize(postcard::Error),
}

pub fn encode_record<T>(
    slot: &mut [u8],
    magic: u32,
    version: u16,
    value: &T,
) -> Result<(), RecordEncodeError>
where
    T: Serialize,
{
    if slot.len() < HEADER_SIZE {
        return Err(RecordEncodeError::BufferTooSmall);
    }

    slot.fill(0xFF);
    let payload = postcard::to_slice(value, &mut slot[HEADER_SIZE..])
        .map_err(RecordEncodeError::Serialize)?;
    let payload_len = payload.len();

    if payload_len > u16::MAX as usize {
        return Err(RecordEncodeError::BufferTooSmall);
    }

    slot[..4].copy_from_slice(&magic.to_le_bytes());
    slot[4..6].copy_from_slice(&version.to_le_bytes());
    slot[6..8].copy_from_slice(&(payload_len as u16).to_le_bytes());
    let record_checksum = checksum(
        magic,
        version,
        payload_len as u16,
        &slot[HEADER_SIZE..HEADER_SIZE + payload_len],
    );
    slot[8..12].copy_from_slice(&record_checksum.to_le_bytes());

    Ok(())
}

pub fn decode_record<T>(slot: &[u8], magic: u32, version: u16) -> RecordLoad<T>
where
    T: for<'de> Deserialize<'de>,
{
    if is_blank(slot) {
        return RecordLoad::Missing;
    }

    if slot.len() < HEADER_SIZE {
        return RecordLoad::Corrupt;
    }

    let stored_magic = u32::from_le_bytes([slot[0], slot[1], slot[2], slot[3]]);
    let stored_version = u16::from_le_bytes([slot[4], slot[5]]);
    let payload_len = u16::from_le_bytes([slot[6], slot[7]]) as usize;
    let stored_checksum = u32::from_le_bytes([slot[8], slot[9], slot[10], slot[11]]);

    if stored_magic != magic || stored_version != version {
        return RecordLoad::Corrupt;
    }

    if HEADER_SIZE + payload_len > slot.len() {
        return RecordLoad::Corrupt;
    }

    let payload = &slot[HEADER_SIZE..HEADER_SIZE + payload_len];
    if checksum(magic, version, payload_len as u16, payload) != stored_checksum {
        return RecordLoad::Corrupt;
    }

    match postcard::from_bytes::<T>(payload) {
        Ok(value) => RecordLoad::Valid(value),
        Err(_) => RecordLoad::Corrupt,
    }
}

fn is_blank(slot: &[u8]) -> bool {
    slot.iter().all(|byte| *byte == 0xFF)
}

fn checksum(magic: u32, version: u16, payload_len: u16, payload: &[u8]) -> u32 {
    let mut acc = magic ^ u32::from(version) ^ u32::from(payload_len) ^ 0x7352_4543;
    for byte in payload {
        acc = acc.rotate_left(5) ^ u32::from(*byte);
    }
    acc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DeviceMode, StoredDeviceConfig};

    const MAGIC: u32 = u32::from_le_bytes(*b"CONF");
    const VERSION: u16 = 1;

    #[test]
    fn blank_slot_is_missing() {
        let slot = [0xFF_u8; SETTINGS_SLOT_SIZE];
        let decoded = decode_record::<StoredDeviceConfig>(&slot, MAGIC, VERSION);
        assert_eq!(decoded, RecordLoad::Missing);
    }

    #[test]
    fn record_roundtrips() {
        let config = StoredDeviceConfig {
            mode: DeviceMode::Production,
            wifi: None,
        };

        let mut slot = [0_u8; SETTINGS_SLOT_SIZE];
        encode_record(&mut slot, MAGIC, VERSION, &config).unwrap();

        let decoded = decode_record::<StoredDeviceConfig>(&slot, MAGIC, VERSION);
        assert_eq!(decoded, RecordLoad::Valid(config));
    }

    #[test]
    fn checksum_mismatch_is_corrupt() {
        let config = StoredDeviceConfig::default();
        let mut slot = [0_u8; SETTINGS_SLOT_SIZE];
        encode_record(&mut slot, MAGIC, VERSION, &config).unwrap();
        slot[HEADER_SIZE] ^= 0x01;

        let decoded = decode_record::<StoredDeviceConfig>(&slot, MAGIC, VERSION);
        assert_eq!(decoded, RecordLoad::Corrupt);
    }
}
