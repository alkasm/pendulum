use esp_hal::{Blocking, i2c::master::I2c};
use pendulum_lib::{HallMeasurement, HallTelemetry};

use crate::{
    bringup::{HALL_SENSOR_ADDR, i2c_device_present},
    hw::{tmag5273_configure_default_on_bus, tmag5273_read_measurement_on_bus},
};

pub fn read_hall_telemetry(
    i2c: &mut I2c<'_, Blocking>,
    hall_configured: &mut bool,
) -> HallTelemetry {
    if !i2c_device_present(i2c, HALL_SENSOR_ADDR) {
        *hall_configured = false;
        return HallTelemetry::Missing;
    }

    if !*hall_configured {
        match tmag5273_configure_default_on_bus(i2c, HALL_SENSOR_ADDR) {
            Ok(()) => *hall_configured = true,
            Err(register) => return HallTelemetry::ConfigError { register },
        }
    }

    match tmag5273_read_measurement_on_bus(i2c, HALL_SENSOR_ADDR) {
        Ok(measurement) => HallTelemetry::Measurement(HallMeasurement {
            temperature_c: measurement.temperature_c,
            x_mt: measurement.x_mt,
            y_mt: measurement.y_mt,
            z_mt: measurement.z_mt,
            angle_deg: measurement.angle_deg,
            magnitude: measurement.magnitude,
            set_count: measurement.conv_status.set_count,
            result_ready: measurement.conv_status.result_ready,
            por: measurement.conv_status.por,
            diag_fail: measurement.conv_status.diag_fail,
            int_pin_high: measurement.device_status.int_pin_high,
            oscillator_error: measurement.device_status.oscillator_error,
            int_pin_error: measurement.device_status.int_pin_error,
            otp_crc_error: measurement.device_status.otp_crc_error,
            vcc_uv_error: measurement.device_status.vcc_uv_error,
        }),
        Err(register) => HallTelemetry::ReadError { register },
    }
}
