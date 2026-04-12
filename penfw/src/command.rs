use esp_hal::{Blocking, uart::Uart};
use pendulum_lib::{DeviceRequest, DeviceResponse};

use crate::bringup::write_bytes;

const COMMAND_FRAME_CAPACITY: usize = 512;
const READ_CHUNK_CAPACITY: usize = 64;

pub struct CommandPort {
    frame_buf: [u8; COMMAND_FRAME_CAPACITY],
    frame_len: usize,
}

impl CommandPort {
    pub fn new() -> Self {
        Self {
            frame_buf: [0_u8; COMMAND_FRAME_CAPACITY],
            frame_len: 0,
        }
    }

    pub fn poll(&mut self, serial: &mut Uart<'_, Blocking>) -> Option<DeviceRequest> {
        let mut read_buf = [0_u8; READ_CHUNK_CAPACITY];
        let bytes_read = serial.read_buffered(&mut read_buf).ok()?;
        if bytes_read == 0 {
            return None;
        }

        for byte in &read_buf[..bytes_read] {
            if *byte == 0 {
                let request = postcard::from_bytes_cobs::<DeviceRequest>(
                    &mut self.frame_buf[..self.frame_len],
                )
                .ok();
                self.frame_len = 0;
                if request.is_some() {
                    return request;
                }
                continue;
            }

            if self.frame_len < self.frame_buf.len() {
                self.frame_buf[self.frame_len] = *byte;
                self.frame_len += 1;
            } else {
                self.frame_len = 0;
            }
        }

        None
    }
}

pub fn write_response(serial: &mut Uart<'_, Blocking>, response: &DeviceResponse) {
    let mut encoded = [0_u8; COMMAND_FRAME_CAPACITY];
    if let Ok(frame) = postcard::to_slice_cobs(response, &mut encoded) {
        write_bytes(serial, frame);
    }
}
