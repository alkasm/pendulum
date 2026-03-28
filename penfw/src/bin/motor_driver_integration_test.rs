#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[path = "../bringup.rs"]
mod bringup;
#[path = "../hw/mod.rs"]
mod hw;

use core::fmt::Write;

use bringup::{HALL_SENSOR_ADDR, init_console, init_delay, max_clock_config, write_line};
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
        HALL_SENSOR_ADDR,
        peripherals.GPIO5,
        peripherals.GPIO34,
        peripherals.GPIO16,
        peripherals.GPIO17,
        peripherals.GPIO18,
        peripherals.GPIO23,
        peripherals.GPIO19,
        peripherals.GPIO33,
    );

    write_line(&mut serial, "motor_driver_integration_test ready");
    write_line(
        &mut serial,
        "sampling current ADC counts: MCP6021=GPIO32, INA240 U=GPIO35, V=GPIO36, W=GPIO39",
    );
    write_line(
        &mut serial,
        "current estimates use SparkFun's documented 0.01 ohm shunt and INA240 gain of 20 V/V",
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

    write_line(&mut serial, "probing TMAG5273 at 0x22");
    loop {
        if board.hall_sensor.is_present() {
            write_line(&mut serial, "TMAG5273 present at 0x22");
            if let Ok(identity) = board.hall_sensor.read_identity() {
                let _ = writeln!(
                    serial,
                    "TMAG5273 id: addr=0x{:02X} device_id=0x{:02X} manufacturer=0x{:04X}\r",
                    identity.address, identity.device_id, identity.manufacturer_id
                );
                let _ = serial.flush();
            }

            match board.hall_sensor.configure_default() {
                Ok(()) => {
                    write_line(
                        &mut serial,
                        "configured TMAG5273 for continuous measurement, XYZ channels, temperature, XY angle",
                    );
                    break;
                }
                Err(register) => {
                    let _ = writeln!(
                        serial,
                        "failed to configure TMAG5273 at register 0x{register:02X}\r"
                    );
                    let _ = serial.flush();
                }
            }
        } else {
            write_line(&mut serial, "TMAG5273 missing at 0x22");
        }

        delay.delay_millis(1_000);
    }

    write_line(
        &mut serial,
        "holding TMC6300 VIO low for 2 seconds before stepping six-phase commutation",
    );
    delay.delay_millis(2_000);

    board.motor_driver.enable();
    write_line(&mut serial, "driver enabled on GPIO5");

    let mut step_index = 0_usize;
    let mut last_angle_deg: Option<f32> = None;
    loop {
        let step = SIX_STEP_COMMUTATION[step_index];
        board.motor_driver.apply_step(step);

        let current = board.current_sensor.read();
        match board.hall_sensor.read_measurement() {
            Ok(measurement) => {
                let angle_delta_deg = last_angle_deg
                    .map(|previous| wrap_angle_delta_deg(measurement.angle_deg - previous))
                    .unwrap_or(0.0);
                last_angle_deg = Some(measurement.angle_deg);

                let _ = write!(
                    serial,
                    "step={} pattern={} diag={} mcp6021={:>4} ({:+5}, {:>4.2}V) ina_u={:>4} ({:+5}, {:+5.2}A) ina_v={:>4} ({:+5}, {:+5.2}A) ina_w={:>4} ({:+5}, {:+5.2}A) hall_t={:>5.1}C hall_x={:+7.2}mT hall_y={:+7.2}mT hall_z={:+7.2}mT angle={:>7.2}deg d_angle={:+6.2}deg mag=0x{:02X} conv=set{}{}{}{} dev={}{}{}{}{}\r\n",
                    step_index,
                    step.name,
                    if board.motor_driver.diag_is_high() {
                        "high"
                    } else {
                        "low"
                    },
                    current.mcp6021.counts,
                    current.mcp6021.delta_counts,
                    current.mcp6021.volts,
                    current.ina_u.counts,
                    current.ina_u.delta_counts,
                    current.ina_u.amps,
                    current.ina_v.counts,
                    current.ina_v.delta_counts,
                    current.ina_v.amps,
                    current.ina_w.counts,
                    current.ina_w.delta_counts,
                    current.ina_w.amps,
                    measurement.temperature_c,
                    measurement.x_mt,
                    measurement.y_mt,
                    measurement.z_mt,
                    measurement.angle_deg,
                    angle_delta_deg,
                    measurement.magnitude,
                    measurement.conv_status.set_count,
                    if measurement.conv_status.result_ready {
                        " ready"
                    } else {
                        ""
                    },
                    if measurement.conv_status.por {
                        " por"
                    } else {
                        ""
                    },
                    if measurement.conv_status.diag_fail {
                        " diag"
                    } else {
                        ""
                    },
                    if measurement.device_status.int_pin_high {
                        " int_high"
                    } else {
                        " int_low"
                    },
                    if measurement.device_status.oscillator_error {
                        " osc"
                    } else {
                        ""
                    },
                    if measurement.device_status.int_pin_error {
                        " int_err"
                    } else {
                        ""
                    },
                    if measurement.device_status.otp_crc_error {
                        " otp_crc"
                    } else {
                        ""
                    },
                    if measurement.device_status.vcc_uv_error {
                        " uv"
                    } else {
                        ""
                    },
                );
                let _ = serial.flush();
            }
            Err(register) => {
                let _ = writeln!(
                    serial,
                    "step={} pattern={} diag={} mcp6021={:>4} ({:+5}) ina_u={:>4} ({:+5}) ina_v={:>4} ({:+5}) ina_w={:>4} ({:+5}) hall_read_error=0x{register:02X}\r",
                    step_index,
                    step.name,
                    if board.motor_driver.diag_is_high() {
                        "high"
                    } else {
                        "low"
                    },
                    current.mcp6021.counts,
                    current.mcp6021.delta_counts,
                    current.ina_u.counts,
                    current.ina_u.delta_counts,
                    current.ina_v.counts,
                    current.ina_v.delta_counts,
                    current.ina_w.counts,
                    current.ina_w.delta_counts,
                );
                let _ = serial.flush();
            }
        }

        delay.delay_millis(STEP_HOLD_MS);
        step_index = (step_index + 1) % SIX_STEP_COMMUTATION.len();
    }
}

fn wrap_angle_delta_deg(delta: f32) -> f32 {
    if delta > 180.0 {
        delta - 360.0
    } else if delta < -180.0 {
        delta + 360.0
    } else {
        delta
    }
}
