use std::{
    io::{BufReader, Write},
    time::Duration,
};

use clap::{Parser, Subcommand, ValueEnum};
use pendulum_lib::{
    CalibrationStatus, DEFAULT_SENSOR_TELEMETRY_BAUD, DeviceInfo, DeviceMode, DeviceRequest,
    DeviceResponse, DeviceStatus, WifiCredentials, WifiStatus, transport,
};

const DEFAULT_SERIAL_PORT: &str = "/dev/cu.usbserial-110";

#[derive(Parser, Debug)]
#[command(name = "pendev")]
struct Cli {
    #[arg(long, default_value = DEFAULT_SERIAL_PORT)]
    port: String,

    #[arg(long, default_value_t = DEFAULT_SENSOR_TELEMETRY_BAUD)]
    baud: u32,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    Info,
    Status,
    Mode {
        #[command(subcommand)]
        command: ModeCommand,
    },
    Wifi {
        #[command(subcommand)]
        command: WifiCommand,
    },
    Motor {
        #[command(subcommand)]
        command: MotorCommand,
    },
    Run {
        #[command(subcommand)]
        command: RunCommand,
    },
    Reboot,
}

#[derive(Subcommand, Debug)]
enum ModeCommand {
    Get,
    Set { mode: ModeArg },
}

#[derive(Subcommand, Debug)]
enum WifiCommand {
    Status,
    Set { ssid: String, password: String },
    Clear,
    Validate,
}

#[derive(Subcommand, Debug)]
enum MotorCommand {
    Calibrate,
}

#[derive(Subcommand, Debug)]
enum RunCommand {
    Start,
    Stop,
}

#[derive(ValueEnum, Clone, Debug)]
enum ModeArg {
    Manufacturing,
    Production,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    match &cli.command {
        Command::Info => {
            let response = send_request(&cli, DeviceRequest::GetInfo)?;
            print_info(&expect_info(response)?);
        }
        Command::Status => {
            let response = send_request(&cli, DeviceRequest::GetStatus)?;
            print_status(&expect_status(response)?);
        }
        Command::Mode { command } => match command {
            ModeCommand::Get => {
                let response = send_request(&cli, DeviceRequest::GetStatus)?;
                println!("{:?}", expect_status(response)?.mode);
            }
            ModeCommand::Set { mode } => {
                let request = DeviceRequest::SetMode(mode.clone().into());
                let response = send_request(&cli, request)?;
                expect_ack(response)?;
                println!("mode change accepted");
            }
        },
        Command::Wifi { command } => match command {
            WifiCommand::Status => {
                let response = send_request(&cli, DeviceRequest::GetWifiStatus)?;
                print_wifi_status(&expect_wifi_status(response)?);
            }
            WifiCommand::Set { ssid, password } => {
                let credentials = WifiCredentials::new(ssid, password)
                    .map_err(|error| format!("invalid Wi-Fi credentials: {error:?}"))?;
                let response = send_request(&cli, DeviceRequest::SetWifiConfig(credentials))?;
                print_response(response)?;
            }
            WifiCommand::Clear => {
                let response = send_request(&cli, DeviceRequest::ClearWifiConfig)?;
                print_response(response)?;
            }
            WifiCommand::Validate => {
                let response = send_request(&cli, DeviceRequest::ValidateWifi)?;
                print_response(response)?;
            }
        },
        Command::Motor { command } => match command {
            MotorCommand::Calibrate => {
                let response = send_request(&cli, DeviceRequest::StartMotorCalibration)?;
                print_response(response)?;
            }
        },
        Command::Run { command } => match command {
            RunCommand::Start => {
                let response = send_request(&cli, DeviceRequest::StartRun)?;
                expect_ack(response)?;
                println!("run started");
            }
            RunCommand::Stop => {
                let response = send_request(&cli, DeviceRequest::StopRun)?;
                expect_ack(response)?;
                println!("run stopped");
            }
        },
        Command::Reboot => {
            let response = send_request(&cli, DeviceRequest::Reboot)?;
            expect_ack(response)?;
            println!("reboot accepted");
        }
    }

    Ok(())
}

fn send_request(cli: &Cli, request: DeviceRequest) -> Result<DeviceResponse, Box<dyn std::error::Error>> {
    let timeout = command_timeout(&cli.command);
    let port = serialport::new(&cli.port, cli.baud).timeout(timeout).open()?;
    let mut port = BufReader::new(port);
    transport::write_cobs_message(port.get_mut(), &request)?;
    port.get_mut().flush()?;
    Ok(transport::read_cobs_message(&mut port)?)
}

fn command_timeout(command: &Command) -> Duration {
    match command {
        Command::Wifi { command } => match command {
            WifiCommand::Set { .. } | WifiCommand::Validate => Duration::from_secs(20),
            _ => Duration::from_secs(5),
        },
        Command::Motor { .. } => Duration::from_secs(120),
        _ => Duration::from_secs(5),
    }
}

fn expect_ack(response: DeviceResponse) -> Result<(), Box<dyn std::error::Error>> {
    match response {
        DeviceResponse::Ack => Ok(()),
        DeviceResponse::Error(error) => Err(format!("device returned error: {error:?}").into()),
        other => Err(format!("unexpected response: {other:?}").into()),
    }
}

fn expect_info(response: DeviceResponse) -> Result<DeviceInfo, Box<dyn std::error::Error>> {
    match response {
        DeviceResponse::Info(info) => Ok(info),
        DeviceResponse::Error(error) => Err(format!("device returned error: {error:?}").into()),
        other => Err(format!("unexpected response: {other:?}").into()),
    }
}

fn expect_status(response: DeviceResponse) -> Result<DeviceStatus, Box<dyn std::error::Error>> {
    match response {
        DeviceResponse::Status(status) => Ok(status),
        DeviceResponse::Error(error) => Err(format!("device returned error: {error:?}").into()),
        other => Err(format!("unexpected response: {other:?}").into()),
    }
}

fn expect_wifi_status(response: DeviceResponse) -> Result<WifiStatus, Box<dyn std::error::Error>> {
    match response {
        DeviceResponse::WifiStatus(status) => Ok(status),
        DeviceResponse::Error(error) => Err(format!("device returned error: {error:?}").into()),
        other => Err(format!("unexpected response: {other:?}").into()),
    }
}

fn print_response(response: DeviceResponse) -> Result<(), Box<dyn std::error::Error>> {
    match response {
        DeviceResponse::Ack => {
            println!("ok");
            Ok(())
        }
        DeviceResponse::Info(info) => {
            print_info(&info);
            Ok(())
        }
        DeviceResponse::Status(status) => {
            print_status(&status);
            Ok(())
        }
        DeviceResponse::WifiStatus(status) => {
            print_wifi_status(&status);
            Ok(())
        }
        DeviceResponse::CalibrationStatus(status) => {
            print_calibration_status(status);
            Ok(())
        }
        DeviceResponse::WifiValidation(report) => {
            print_wifi_status(&report.status);
            println!("validation: {:?}", report.result);
            Ok(())
        }
        DeviceResponse::Error(error) => Err(format!("device returned error: {error:?}").into()),
    }
}

fn print_info(info: &DeviceInfo) {
    println!("firmware: {}", info.firmware_name);
    println!("version: {}", info.firmware_version);
    println!("protocol: {}", info.protocol_version);
}

fn print_status(status: &DeviceStatus) {
    println!("mode: {:?}", status.mode);
    println!("state: {:?}", status.state);
    println!("fault: {:?}", status.fault);
    print_wifi_status(&status.wifi);
    print_calibration_status(status.calibration.clone());
    println!("control_mode: {:?}", status.control_mode);
}

fn print_wifi_status(status: &WifiStatus) {
    println!("wifi.ssid: {:?}", status.ssid);
}

fn print_calibration_status(status: CalibrationStatus) {
    println!("calibration: {:?}", status);
}

impl From<ModeArg> for DeviceMode {
    fn from(value: ModeArg) -> Self {
        match value {
            ModeArg::Manufacturing => DeviceMode::Manufacturing,
            ModeArg::Production => DeviceMode::Production,
        }
    }
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::{Cli, Command, ModeArg, ModeCommand, WifiCommand};

    #[test]
    fn parses_mode_set_command() {
        let cli = Cli::try_parse_from(["pendev", "mode", "set", "production"]).unwrap();
        match cli.command {
            Command::Mode { command: ModeCommand::Set { mode } } => {
                assert!(matches!(mode, ModeArg::Production));
            }
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_wifi_set_arguments() {
        let cli =
            Cli::try_parse_from(["pendev", "wifi", "set", "pendulum-net", "super-secret"])
                .unwrap();

        match cli.command {
            Command::Wifi {
                command: WifiCommand::Set { ssid, password },
            } => {
                assert_eq!(ssid, "pendulum-net");
                assert_eq!(password, "super-secret");
            }
            other => panic!("unexpected command: {other:?}"),
        }
    }
}
