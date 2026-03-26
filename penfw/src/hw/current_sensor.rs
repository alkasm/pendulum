use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    peripherals::{ADC1, GPIO32, GPIO35, GPIO36, GPIO39},
    Blocking,
};

const ADC_FULL_SCALE_COUNTS: f32 = 4095.0;
const ADC_REFERENCE_V: f32 = 3.3;
const INA240A1_SHUNT_OHM: f32 = 0.01;
const INA240A1_GAIN_V_PER_V: f32 = 20.0;

pub struct CurrentSensor<'d> {
    adc1: Adc<'d, ADC1<'d>, Blocking>,
    mcp6021: AdcPin<GPIO32<'d>, ADC1<'d>>,
    ina_u: AdcPin<GPIO35<'d>, ADC1<'d>>,
    ina_v: AdcPin<GPIO36<'d>, ADC1<'d>>,
    ina_w: AdcPin<GPIO39<'d>, ADC1<'d>>,
    baseline: CurrentBaseline,
}

#[derive(Clone, Copy)]
pub struct CurrentBaseline {
    pub mcp6021_counts: u16,
    pub ina_u_counts: u16,
    pub ina_v_counts: u16,
    pub ina_w_counts: u16,
}

#[derive(Clone, Copy)]
pub struct CurrentChannel {
    pub counts: u16,
    pub delta_counts: i32,
    pub volts: f32,
}

#[derive(Clone, Copy)]
pub struct Ina240Channel {
    pub counts: u16,
    pub delta_counts: i32,
    pub amps: f32,
}

#[derive(Clone, Copy)]
pub struct CurrentSample {
    pub mcp6021: CurrentChannel,
    pub ina_u: Ina240Channel,
    pub ina_v: Ina240Channel,
    pub ina_w: Ina240Channel,
}

impl<'d> CurrentSensor<'d> {
    pub fn new(
        adc1: ADC1<'d>,
        gpio32: GPIO32<'d>,
        gpio35: GPIO35<'d>,
        gpio36: GPIO36<'d>,
        gpio39: GPIO39<'d>,
    ) -> Self {
        let mut adc_config = AdcConfig::new();
        let mcp6021 = adc_config.enable_pin(gpio32, Attenuation::_11dB);
        let ina_u = adc_config.enable_pin(gpio35, Attenuation::_11dB);
        let ina_v = adc_config.enable_pin(gpio36, Attenuation::_11dB);
        let ina_w = adc_config.enable_pin(gpio39, Attenuation::_11dB);
        let adc1 = Adc::new(adc1, adc_config);

        Self {
            adc1,
            mcp6021,
            ina_u,
            ina_v,
            ina_w,
            baseline: CurrentBaseline {
                mcp6021_counts: 0,
                ina_u_counts: 0,
                ina_v_counts: 0,
                ina_w_counts: 0,
            },
        }
    }

    pub fn calibrate_baseline(&mut self, samples: u32) -> CurrentBaseline {
        let mut sum32: u32 = 0;
        let mut sum35: u32 = 0;
        let mut sum36: u32 = 0;
        let mut sum39: u32 = 0;

        for _ in 0..samples {
            sum32 += self.read_mcp6021_counts() as u32;
            sum35 += self.read_ina_u_counts() as u32;
            sum36 += self.read_ina_v_counts() as u32;
            sum39 += self.read_ina_w_counts() as u32;
        }

        self.baseline = CurrentBaseline {
            mcp6021_counts: (sum32 / samples) as u16,
            ina_u_counts: (sum35 / samples) as u16,
            ina_v_counts: (sum36 / samples) as u16,
            ina_w_counts: (sum39 / samples) as u16,
        };
        self.baseline
    }

    pub fn read(&mut self) -> CurrentSample {
        let mcp6021_counts = self.read_mcp6021_counts();
        let ina_u_counts = self.read_ina_u_counts();
        let ina_v_counts = self.read_ina_v_counts();
        let ina_w_counts = self.read_ina_w_counts();

        CurrentSample {
            mcp6021: CurrentChannel {
                counts: mcp6021_counts,
                delta_counts: signed_delta(mcp6021_counts, self.baseline.mcp6021_counts),
                volts: counts_to_volts(mcp6021_counts),
            },
            ina_u: self.read_ina_channel(ina_u_counts, self.baseline.ina_u_counts),
            ina_v: self.read_ina_channel(ina_v_counts, self.baseline.ina_v_counts),
            ina_w: self.read_ina_channel(ina_w_counts, self.baseline.ina_w_counts),
        }
    }

    fn read_ina_channel(&self, counts: u16, baseline_counts: u16) -> Ina240Channel {
        let measured_volts = counts_to_volts(counts);
        let baseline_volts = counts_to_volts(baseline_counts);

        Ina240Channel {
            counts,
            delta_counts: signed_delta(counts, baseline_counts),
            amps: ina240a1_amps(measured_volts, baseline_volts),
        }
    }

    fn read_mcp6021_counts(&mut self) -> u16 {
        loop {
            if let Ok(value) = self.adc1.read_oneshot(&mut self.mcp6021) {
                break value;
            }
        }
    }

    fn read_ina_u_counts(&mut self) -> u16 {
        loop {
            if let Ok(value) = self.adc1.read_oneshot(&mut self.ina_u) {
                break value;
            }
        }
    }

    fn read_ina_v_counts(&mut self) -> u16 {
        loop {
            if let Ok(value) = self.adc1.read_oneshot(&mut self.ina_v) {
                break value;
            }
        }
    }

    fn read_ina_w_counts(&mut self) -> u16 {
        loop {
            if let Ok(value) = self.adc1.read_oneshot(&mut self.ina_w) {
                break value;
            }
        }
    }
}

fn counts_to_volts(counts: u16) -> f32 {
    (f32::from(counts) / ADC_FULL_SCALE_COUNTS) * ADC_REFERENCE_V
}

fn ina240a1_amps(measured_volts: f32, baseline_volts: f32) -> f32 {
    let delta_volts = measured_volts - baseline_volts;
    delta_volts / (INA240A1_GAIN_V_PER_V * INA240A1_SHUNT_OHM)
}

fn signed_delta(value: u16, baseline: u16) -> i32 {
    i32::from(value) - i32::from(baseline)
}
