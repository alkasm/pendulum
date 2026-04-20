use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    mcpwm::{
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
        timer::PwmWorkingMode,
        McPwm, PeripheralClockConfig,
    },
    peripherals::{GPIO16, GPIO17, GPIO18, GPIO19, GPIO23, GPIO33, GPIO5, MCPWM0},
    time::Rate,
};
use pendulum_lib::runtime::ControlDrive;

pub const PWM_PERIOD_TICKS: u16 = 2500;
const PWM_FREQUENCY_HZ: u32 = 32_000;
const DEAD_ZONE: f32 = 0.02;
const VOLTAGE_POWER_SUPPLY_V: f32 = 5.0;

pub type PwmPinA<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, true>;
pub type PwmPinB<'a, const OP: u8> = PwmPin<'a, MCPWM0<'a>, OP, false>;

pub struct PwmMotorDriveParts<'a> {
    pub peripheral: MCPWM0<'a>,
    pub enable: GPIO5<'a>,
    pub uh: GPIO16<'a>,
    pub ul: GPIO17<'a>,
    pub vh: GPIO18<'a>,
    pub vl: GPIO23<'a>,
    pub wh: GPIO19<'a>,
    pub wl: GPIO33<'a>,
}

pub struct PwmMotorDrive<'a> {
    enable: Output<'a>,
    uh: PwmPinA<'a, 0>,
    ul: PwmPinB<'a, 0>,
    vh: PwmPinA<'a, 1>,
    vl: PwmPinB<'a, 1>,
    wh: PwmPinA<'a, 2>,
    wl: PwmPinB<'a, 2>,
}

impl<'a> PwmMotorDrive<'a> {
    pub fn new(parts: PwmMotorDriveParts<'a>) -> Self {
        let PwmMotorDriveParts {
            peripheral,
            enable,
            uh,
            ul,
            vh,
            vl,
            wh,
            wl,
        } = parts;

        let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(160))
            .expect("failed to configure MCPWM clock");
        let mut mcpwm = McPwm::new(peripheral, clock_cfg);
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);
        mcpwm.operator2.set_timer(&mcpwm.timer0);

        let (uh, ul) = mcpwm.operator0.with_pins(
            uh,
            PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
            ul,
            low_side_pwm_config(),
        );
        let (vh, vl) = mcpwm.operator1.with_pins(
            vh,
            PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
            vl,
            low_side_pwm_config(),
        );
        let (wh, wl) = mcpwm.operator2.with_pins(
            wh,
            PwmPinConfig::UP_DOWN_ACTIVE_HIGH,
            wl,
            low_side_pwm_config(),
        );

        let timer_clock_cfg = clock_cfg
            .timer_clock_with_frequency(
                PWM_PERIOD_TICKS,
                PwmWorkingMode::UpDown,
                Rate::from_hz(PWM_FREQUENCY_HZ),
            )
            .expect("failed to configure MCPWM timer");
        mcpwm.timer0.start(timer_clock_cfg);

        Self::from_pwm_pins(enable, uh, ul, vh, vl, wh, wl)
    }

    #[allow(clippy::too_many_arguments)]
    fn from_pwm_pins(
        enable: GPIO5<'a>,
        uh: PwmPinA<'a, 0>,
        ul: PwmPinB<'a, 0>,
        vh: PwmPinA<'a, 1>,
        vl: PwmPinB<'a, 1>,
        wh: PwmPinA<'a, 2>,
        wl: PwmPinB<'a, 2>,
    ) -> Self {
        Self {
            enable: Output::new(enable, Level::Low, OutputConfig::default()),
            uh,
            ul,
            vh,
            vl,
            wh,
            wl,
        }
    }

    pub fn enable(&mut self) {
        self.enable.set_high();
    }

    pub fn disable(&mut self) {
        self.enable.set_low();
    }

    pub fn coast(&mut self) {
        self.uh.set_timestamp(0);
        self.vh.set_timestamp(0);
        self.wh.set_timestamp(0);
        self.ul.set_timestamp(PWM_PERIOD_TICKS);
        self.vl.set_timestamp(PWM_PERIOD_TICKS);
        self.wl.set_timestamp(PWM_PERIOD_TICKS);
    }

    pub fn set_phase_voltages(&mut self, ua_v: f32, ub_v: f32, uc_v: f32) {
        let dead = DEAD_ZONE * 0.5;
        let dc_a = clamp(ua_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_b = clamp(ub_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);
        let dc_c = clamp(uc_v / VOLTAGE_POWER_SUPPLY_V, 0.0, 1.0);

        self.uh.set_timestamp(duty_to_ticks(dc_a - dead));
        self.ul.set_timestamp(duty_to_ticks(dc_a + dead));
        self.vh.set_timestamp(duty_to_ticks(dc_b - dead));
        self.vl.set_timestamp(duty_to_ticks(dc_b + dead));
        self.wh.set_timestamp(duty_to_ticks(dc_c - dead));
        self.wl.set_timestamp(duty_to_ticks(dc_c + dead));
    }
}

impl<'a> ControlDrive for PwmMotorDrive<'a> {
    fn enable(&mut self) {
        PwmMotorDrive::enable(self);
    }

    fn disable(&mut self) {
        PwmMotorDrive::disable(self);
    }

    fn coast(&mut self) {
        PwmMotorDrive::coast(self);
    }

    fn set_phase_voltages(&mut self, ua_v: f32, ub_v: f32, uc_v: f32) {
        PwmMotorDrive::set_phase_voltages(self, ua_v, ub_v, uc_v);
    }
}

pub fn low_side_pwm_config() -> PwmPinConfig<false> {
    PwmPinConfig::new(
        PwmActions::<false>::empty()
            .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
            .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
        PwmUpdateMethod::SYNC_ON_ZERO,
    )
}

fn duty_to_ticks(duty: f32) -> u16 {
    let clamped = clamp(duty, 0.0, 1.0);
    (clamped * PWM_PERIOD_TICKS as f32 + 0.5) as u16
}

fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}
