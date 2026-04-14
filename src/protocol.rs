use heapless::String as FixedString;
use serde::{Deserialize, Serialize};
use uom::si::f64::{Angle, AngularVelocity, ElectricCurrent, Time, Torque};

pub const DEFAULT_SENSOR_TELEMETRY_BAUD: u32 = 115_200;
pub const DEFAULT_RUNTIME_TELEMETRY_PORT: u16 = 7001;
pub const DEVICE_PROTOCOL_VERSION: u16 = 1;

pub type FirmwareName = FixedString<24>;
pub type FirmwareVersion = FixedString<24>;
pub type WifiSsid = FixedString<32>;
pub type WifiPassword = FixedString<64>;

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct RuntimeTelemetryFrame {
    pub step: u64,
    pub sim_time: Time,
    pub theta: Angle,
    pub theta_dot: AngularVelocity,
    pub wheel_angle: Angle,
    pub wheel_speed: AngularVelocity,
    pub commanded_torque: Torque,
    pub applied_torque: Torque,
    pub available_torque: Torque,
    pub speed_ratio: f64,
    pub phase_current: ElectricCurrent,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum TelemetryPacket {
    Runtime(RuntimeTelemetryFrame),
    Sensor(SensorTelemetryFrame),
    Pendulum(PendulumTelemetryFrame),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SensorTelemetryFrame {
    pub seq: u32,
    pub uptime_ms: u32,
    pub motor_driver_diag_high: bool,
    pub current: CurrentTelemetry,
    pub hall: HallTelemetry,
    pub imu: ImuTelemetry,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CurrentTelemetry {
    pub mcp6021_counts: u16,
    pub mcp6021_delta_counts: i32,
    pub mcp6021_volts: f32,
    pub ina_u_counts: u16,
    pub ina_u_delta_counts: i32,
    pub ina_u_amps: f32,
    pub ina_v_counts: u16,
    pub ina_v_delta_counts: i32,
    pub ina_v_amps: f32,
    pub ina_w_counts: u16,
    pub ina_w_delta_counts: i32,
    pub ina_w_amps: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum HallTelemetry {
    Missing,
    ConfigError { register: u8 },
    ReadError { register: u8 },
    Measurement(HallMeasurement),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HallMeasurement {
    pub temperature_c: f32,
    pub x_mt: f32,
    pub y_mt: f32,
    pub z_mt: f32,
    pub angle_deg: f32,
    pub magnitude: u8,
    pub set_count: u8,
    pub result_ready: bool,
    pub por: bool,
    pub diag_fail: bool,
    pub int_pin_high: bool,
    pub oscillator_error: bool,
    pub int_pin_error: bool,
    pub otp_crc_error: bool,
    pub vcc_uv_error: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum ImuTelemetry {
    Missing,
    UnexpectedWhoAmI { value: u8 },
    WakeError { register: u8 },
    ReadError { register: u8 },
    Measurement(ImuMeasurement),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImuMeasurement {
    pub ax_g: f32,
    pub ay_g: f32,
    pub az_g: f32,
    pub gx_dps: f32,
    pub gy_dps: f32,
    pub gz_dps: f32,
    pub temperature_c: f32,
    pub theta_deg: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumTelemetryFrame {
    pub seq: u32,
    pub uptime_ms: u32,
    pub timing: PendulumTimingTelemetry,
    pub motor_driver_diag_high: bool,
    pub current: CurrentTelemetry,
    pub hall: HallTelemetry,
    pub estimate: PendulumEstimateTelemetry,
    pub control: PendulumControlTelemetry,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumTimingTelemetry {
    pub loop_period_us: u32,
    pub work_time_us: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum PendulumEstimateTelemetry {
    Missing,
    UnexpectedWhoAmI { value: u8 },
    WakeError { register: u8 },
    ReadError { register: u8 },
    Measurement(PendulumEstimateMeasurement),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumEstimateMeasurement {
    pub theta_deg: f32,
    pub theta_dot_dps: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PendulumControlTelemetry {
    pub mode: PendulumControlMode,
    pub theta_error_deg: f32,
    pub torque_command_nm: f32,
    pub raw_drive_command: f32,
    pub drive_command: f32,
    pub direction_sign: f32,
    pub torque_sign: f32,
    pub electrical_angle_deg: f32,
    pub uq_v: f32,
    pub wheel_angle_deg: f32,
    pub wheel_speed_dps: f32,
    pub commutation_step: u8,
    pub commutation_center_deg: f32,
    pub motor_enabled: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum PendulumControlMode {
    Startup,
    Calibrating,
    Arming,
    WaitingForImu,
    WaitingForHall,
    CaptureOutOfRange,
    CurrentLimited,
    Idle,
    Active,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum DeviceMode {
    Manufacturing,
    Production,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum DeviceState {
    Boot,
    Service,
    Calibrating,
    Running,
    Fault,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum DeviceFault {
    StorageCorrupt,
    MissingCalibration,
    InvalidCalibration,
    MissingWifiConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum CalibrationStatus {
    Missing,
    Valid,
    Invalid,
}

impl CalibrationStatus {
    pub fn is_valid(&self) -> bool {
        matches!(self, Self::Valid)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct WifiCredentials {
    pub ssid: WifiSsid,
    pub password: WifiPassword,
}

impl WifiCredentials {
    pub fn new(ssid: &str, password: &str) -> Result<Self, DeviceValueError> {
        if ssid.is_empty() {
            return Err(DeviceValueError::EmptySsid);
        }

        let mut ssid_buf = WifiSsid::new();
        ssid_buf
            .push_str(ssid)
            .map_err(|_| DeviceValueError::SsidTooLong)?;

        let mut password_buf = WifiPassword::new();
        password_buf
            .push_str(password)
            .map_err(|_| DeviceValueError::PasswordTooLong)?;

        Ok(Self {
            ssid: ssid_buf,
            password: password_buf,
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct WifiStatus {
    pub ssid: Option<WifiSsid>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct DeviceStatus {
    pub mode: DeviceMode,
    pub state: DeviceState,
    pub fault: Option<DeviceFault>,
    pub wifi: WifiStatus,
    pub calibration: CalibrationStatus,
    pub control_mode: Option<PendulumControlMode>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct DeviceInfo {
    pub firmware_name: FirmwareName,
    pub firmware_version: FirmwareVersion,
    pub protocol_version: u16,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum WifiProbeResult {
    Success { ipv4_octets: [u8; 4] },
    ConfigurationRejected,
    StartFailed,
    AssociationTimedOut,
    AssociationFailed,
    DhcpTimedOut,
    DisconnectFailed,
    StopFailed,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct WifiValidationReport {
    pub status: WifiStatus,
    pub result: WifiProbeResult,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct StoredDeviceConfig {
    pub mode: DeviceMode,
    pub wifi: Option<WifiCredentials>,
}

impl Default for StoredDeviceConfig {
    fn default() -> Self {
        Self {
            mode: DeviceMode::Manufacturing,
            wifi: None,
        }
    }
}

impl StoredDeviceConfig {
    pub fn wifi_status(&self) -> WifiStatus {
        WifiStatus {
            ssid: self.wifi.as_ref().map(|wifi| wifi.ssid.clone()),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct StoredMotorCalibration {
    pub direction_sign: f32,
    pub electrical_offset_deg: f32,
    pub torque_sign: f32,
}

impl StoredMotorCalibration {
    pub fn is_valid(&self) -> bool {
        self.direction_sign.is_finite()
            && self.electrical_offset_deg.is_finite()
            && self.torque_sign.is_finite()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum DeviceRequest {
    GetInfo,
    GetStatus,
    GetWifiStatus,
    GetCalibrationStatus,
    SetMode(DeviceMode),
    SetWifiConfig(WifiCredentials),
    ClearWifiConfig,
    ValidateWifi,
    StartMotorCalibration,
    StartRun,
    StopRun,
    Reboot,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum DeviceValueError {
    EmptySsid,
    SsidTooLong,
    PasswordTooLong,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum DeviceCommandError {
    InvalidState,
    UnsupportedInCurrentMode,
    ProductionPrecondition(DeviceFault),
    PersistenceFailed,
    CalibrationFailed,
    InvalidValue(DeviceValueError),
}

impl From<DeviceValueError> for DeviceCommandError {
    fn from(value: DeviceValueError) -> Self {
        Self::InvalidValue(value)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum DeviceResponse {
    Ack,
    Info(DeviceInfo),
    Status(DeviceStatus),
    WifiStatus(WifiStatus),
    CalibrationStatus(CalibrationStatus),
    WifiValidation(WifiValidationReport),
    Error(DeviceCommandError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wifi_credentials_reject_empty_ssid() {
        assert_eq!(
            WifiCredentials::new("", "pw").unwrap_err(),
            DeviceValueError::EmptySsid
        );
    }

    #[test]
    fn wifi_credentials_reject_too_long_ssid() {
        let ssid = "a".repeat(33);
        assert_eq!(
            WifiCredentials::new(&ssid, "pw").unwrap_err(),
            DeviceValueError::SsidTooLong
        );
    }

    #[test]
    fn request_roundtrips_through_postcard_cobs() {
        let request = DeviceRequest::SetWifiConfig(
            WifiCredentials::new("pendulum-net", "super-secret").unwrap(),
        );
        let mut buffer = [0_u8; 256];
        let encoded = postcard::to_slice_cobs(&request, &mut buffer).unwrap();
        let decoded = postcard::from_bytes_cobs::<DeviceRequest>(encoded).unwrap();
        assert_eq!(decoded, request);
    }

    #[test]
    fn response_roundtrips_through_postcard_cobs() {
        let mut firmware_name = FirmwareName::new();
        firmware_name.push_str("penfw").unwrap();
        let mut firmware_version = FirmwareVersion::new();
        firmware_version.push_str("0.1.0").unwrap();

        let response = DeviceResponse::Info(DeviceInfo {
            firmware_name,
            firmware_version,
            protocol_version: DEVICE_PROTOCOL_VERSION,
        });

        let mut buffer = [0_u8; 256];
        let encoded = postcard::to_slice_cobs(&response, &mut buffer).unwrap();
        let decoded = postcard::from_bytes_cobs::<DeviceResponse>(encoded).unwrap();
        assert_eq!(decoded, response);
    }
}
