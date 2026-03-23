#![no_std]
#![no_main]

#[path = "../bringup.rs"]
mod bringup;

use bringup::{init_console, init_delay, max_clock_config, write_fmt_line, write_line};
use esp_hal::main;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};

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

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();
    let mut driver_enable =
        Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let diag = Input::new(peripherals.GPIO34, InputConfig::default());
    let mut uh = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());
    let mut ul = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());
    let mut vh = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let mut vl = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let mut wh = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    let mut wl = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());

    write_line(&mut serial, "motor_open_loop ready");
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
        write_fmt_line(
            &mut serial,
            format_args!(
                "step={} pattern={} diag={}",
                step_index,
                step.name,
                if diag.is_high() { "high" } else { "low" }
            ),
        );
        delay.delay_millis(500);
        step_index = (step_index + 1) % STEPS.len();
    }
}
