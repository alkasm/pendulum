use esp_hal::peripherals::{
    ADC1, GPIO5, GPIO16, GPIO17, GPIO18, GPIO19, GPIO21, GPIO22, GPIO23, GPIO32, GPIO33, GPIO34,
    GPIO35, GPIO36, GPIO39, I2C0,
};

use super::{
    current_sensor::CurrentSensor,
    tmag5273::Tmag5273,
    tmc6300::{CommutationStep, SIX_STEP_COMMUTATION, Tmc6300},
};

pub struct MotorDriverBoard<'d> {
    pub current_sensor: CurrentSensor<'d>,
    pub hall_sensor: Tmag5273<'d>,
    pub motor_driver: Tmc6300<'d>,
}

impl<'d> MotorDriverBoard<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        adc1: ADC1<'d>,
        gpio32: GPIO32<'d>,
        gpio35: GPIO35<'d>,
        gpio36: GPIO36<'d>,
        gpio39: GPIO39<'d>,
        i2c0: I2C0<'d>,
        sda: GPIO21<'d>,
        scl: GPIO22<'d>,
        hall_address: u8,
        enable: GPIO5<'d>,
        diag: GPIO34<'d>,
        uh: GPIO16<'d>,
        ul: GPIO17<'d>,
        vh: GPIO18<'d>,
        vl: GPIO23<'d>,
        wh: GPIO19<'d>,
        wl: GPIO33<'d>,
    ) -> Self {
        Self {
            current_sensor: CurrentSensor::new(adc1, gpio32, gpio35, gpio36, gpio39),
            hall_sensor: Tmag5273::new(i2c0, sda, scl, hall_address),
            motor_driver: Tmc6300::new(enable, diag, uh, ul, vh, vl, wh, wl),
        }
    }

    pub fn step_pattern(index: usize) -> CommutationStep {
        SIX_STEP_COMMUTATION[index % SIX_STEP_COMMUTATION.len()]
    }

    pub fn step_count() -> usize {
        SIX_STEP_COMMUTATION.len()
    }
}
