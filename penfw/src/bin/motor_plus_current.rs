#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;

use core::fmt::Write;

use bringup::{init_console, init_delay, max_clock_config, write_line};
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::main;

const BASELINE_SAMPLES: u32 = 16;
const STEP_HOLD_MS: u32 = 500;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[derive(Clone, Copy)]
struct CommutationStep {
    name: &'static str,
    uh: bool,
    ul: bool,
    vh: bool,
    vl: bool,
    wh: bool,
    wl: bool,
}

const STEPS: [CommutationStep; 6] = [
    CommutationStep {
        name: "U+ V-",
        uh: true,
        ul: false,
        vh: false,
        vl: true,
        wh: false,
        wl: false,
    },
    CommutationStep {
        name: "U+ W-",
        uh: true,
        ul: false,
        vh: false,
        vl: false,
        wh: false,
        wl: true,
    },
    CommutationStep {
        name: "V+ W-",
        uh: false,
        ul: false,
        vh: true,
        vl: false,
        wh: false,
        wl: true,
    },
    CommutationStep {
        name: "V+ U-",
        uh: false,
        ul: true,
        vh: true,
        vl: false,
        wh: false,
        wl: false,
    },
    CommutationStep {
        name: "W+ U-",
        uh: false,
        ul: true,
        vh: false,
        vl: false,
        wh: true,
        wl: false,
    },
    CommutationStep {
        name: "W+ V-",
        uh: false,
        ul: false,
        vh: false,
        vl: true,
        wh: true,
        wl: false,
    },
];

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();

    let mut adc_config = AdcConfig::new();
    let mut gpio32 = adc_config.enable_pin(peripherals.GPIO32, Attenuation::_11dB);
    let mut gpio35 = adc_config.enable_pin(peripherals.GPIO35, Attenuation::_11dB);
    let mut gpio36 = adc_config.enable_pin(peripherals.GPIO36, Attenuation::_11dB);
    let mut gpio39 = adc_config.enable_pin(peripherals.GPIO39, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);

    let mut driver_enable = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let diag = Input::new(peripherals.GPIO34, InputConfig::default());
    let mut uh = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());
    let mut ul = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());
    let mut vh = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut vl = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let mut wh = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    let mut wl = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());

    write_line(&mut serial, "motor_plus_current ready");
    write_line(
        &mut serial,
        "sampling current-sense ADC counts: MCP6021=GPIO32, INA240 U=GPIO35, V=GPIO36, W=GPIO39",
    );

    let mut sum32: u32 = 0;
    let mut sum35: u32 = 0;
    let mut sum36: u32 = 0;
    let mut sum39: u32 = 0;

    for _ in 0..BASELINE_SAMPLES {
        let ch32 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio32) {
                break value;
            }
        };
        let ch35 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio35) {
                break value;
            }
        };
        let ch36 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio36) {
                break value;
            }
        };
        let ch39 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio39) {
                break value;
            }
        };

        sum32 += ch32 as u32;
        sum35 += ch35 as u32;
        sum36 += ch36 as u32;
        sum39 += ch39 as u32;
    }

    let baseline = (
        (sum32 / BASELINE_SAMPLES) as u16,
        (sum35 / BASELINE_SAMPLES) as u16,
        (sum36 / BASELINE_SAMPLES) as u16,
        (sum39 / BASELINE_SAMPLES) as u16,
    );
    let _ = writeln!(
        serial,
        "baseline mcp6021={} ina_u={} ina_v={} ina_w={}\r",
        baseline.0, baseline.1, baseline.2, baseline.3
    );
    let _ = serial.flush();

    write_line(
        &mut serial,
        "holding TMC6300 VIO low for 2 seconds before stepping six-phase commutation",
    );
    delay.delay_millis(2_000);

    driver_enable.set_high();
    write_line(&mut serial, "driver enabled on GPIO5");

    let mut step_index = 0_usize;
    loop {
        let step = STEPS[step_index];
        apply_step(step, &mut uh, &mut ul, &mut vh, &mut vl, &mut wh, &mut wl);

        let ch32 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio32) {
                break value;
            }
        };
        let ch35 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio35) {
                break value;
            }
        };
        let ch36 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio36) {
                break value;
            }
        };
        let ch39 = loop {
            if let Ok(value) = adc1.read_oneshot(&mut gpio39) {
                break value;
            }
        };

        let _ = writeln!(
            serial,
            "step={} pattern={} diag={} mcp6021={:>4} ({:+5}) ina_u={:>4} ({:+5}) ina_v={:>4} ({:+5}) ina_w={:>4} ({:+5})\r",
            step_index,
            step.name,
            if diag.is_high() { "high" } else { "low" },
            ch32,
            signed_delta(ch32, baseline.0),
            ch35,
            signed_delta(ch35, baseline.1),
            ch36,
            signed_delta(ch36, baseline.2),
            ch39,
            signed_delta(ch39, baseline.3),
        );
        let _ = serial.flush();

        delay.delay_millis(STEP_HOLD_MS);
        step_index = (step_index + 1) % STEPS.len();
    }
}

fn apply_step(
    step: CommutationStep,
    uh: &mut Output<'_>,
    ul: &mut Output<'_>,
    vh: &mut Output<'_>,
    vl: &mut Output<'_>,
    wh: &mut Output<'_>,
    wl: &mut Output<'_>,
) {
    uh.set_level(if step.uh { Level::High } else { Level::Low });
    ul.set_level(if step.ul { Level::High } else { Level::Low });
    vh.set_level(if step.vh { Level::High } else { Level::Low });
    vl.set_level(if step.vl { Level::High } else { Level::Low });
    wh.set_level(if step.wh { Level::High } else { Level::Low });
    wl.set_level(if step.wl { Level::High } else { Level::Low });
}

fn signed_delta(value: u16, baseline: u16) -> i32 {
    i32::from(value) - i32::from(baseline)
}
