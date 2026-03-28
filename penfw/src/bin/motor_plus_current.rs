#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;
#[path = "../hw/mod.rs"]
mod hw;

use core::fmt::Write;

use bringup::{init_console, init_delay, max_clock_config, write_line};
use esp_hal::main;
use hw::{MotorDriverBoard, SIX_STEP_COMMUTATION};

const BASELINE_SAMPLES: u32 = 16;
const STEP_HOLD_MS: u32 = 500;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    esp_hal::system::software_reset()
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(max_clock_config());
    let mut serial = init_console(peripherals.UART0, peripherals.GPIO1, peripherals.GPIO3);
    let delay = init_delay();

    let mut board = MotorDriverBoard::new(
        peripherals.ADC1,
        peripherals.GPIO32,
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO39,
        peripherals.I2C0,
        peripherals.GPIO21,
        peripherals.GPIO22,
        bringup::HALL_SENSOR_ADDR,
        peripherals.GPIO5,
        peripherals.GPIO34,
        peripherals.GPIO16,
        peripherals.GPIO17,
        peripherals.GPIO18,
        peripherals.GPIO23,
        peripherals.GPIO19,
        peripherals.GPIO33,
    );

    write_line(&mut serial, "motor_plus_current ready");
    write_line(
        &mut serial,
        "sampling current ADC counts: MCP6021=GPIO32, INA240 U=GPIO35, V=GPIO36, W=GPIO39",
    );

    let baseline = board.current_sensor.calibrate_baseline(BASELINE_SAMPLES);
    let _ = writeln!(
        serial,
        "baseline mcp6021={} ina_u={} ina_v={} ina_w={}\r",
        baseline.mcp6021_counts,
        baseline.ina_u_counts,
        baseline.ina_v_counts,
        baseline.ina_w_counts
    );
    let _ = serial.flush();

    write_line(
        &mut serial,
        "holding TMC6300 VIO low for 2 seconds before stepping six-phase commutation",
    );
    delay.delay_millis(2_000);

    board.motor_driver.enable();
    write_line(&mut serial, "driver enabled on GPIO5");

    let mut step_index = 0_usize;
    loop {
        let step = SIX_STEP_COMMUTATION[step_index];
        board.motor_driver.apply_step(step);
        let sample = board.current_sensor.read();

        let _ = writeln!(
            serial,
            "step={} pattern={} diag={} mcp6021={:>4} ({:+5}) ina_u={:>4} ({:+5}) ina_v={:>4} ({:+5}) ina_w={:>4} ({:+5})\r",
            step_index,
            step.name,
            if board.motor_driver.diag_is_high() {
                "high"
            } else {
                "low"
            },
            sample.mcp6021.counts,
            sample.mcp6021.delta_counts,
            sample.ina_u.counts,
            sample.ina_u.delta_counts,
            sample.ina_v.counts,
            sample.ina_v.delta_counts,
            sample.ina_w.counts,
            sample.ina_w.delta_counts,
        );
        let _ = serial.flush();

        delay.delay_millis(STEP_HOLD_MS);
        step_index = (step_index + 1) % SIX_STEP_COMMUTATION.len();
    }
}
