//! All compile-time constants, ported 1:1 from `firmware/include/config.h`.

// ---------------- Hardware ----------------
/// GPIO of the on-board WS2812 status LED (ESP32-S3-DevKitC-1).
pub const LED_PIN: u8 = 48;
pub const NUM_LEDS: usize = 1;
/// UART peripheral connected to the flight controller.
pub const FC_UART_NUM: u8 = 0;
/// FC UART lines: RX = GPIO44, TX = GPIO43.
pub const FC_UART_RX: i32 = 44;
pub const FC_UART_TX: i32 = 43;

// ---------------- Buffers / ports ----------------
pub const FC_RX_BUF: usize = 16_384;
pub const FC_TX_BUF: usize = 4_096;
pub const BRIDGE_BUF_SIZE: usize = 2_048;
pub const UDP_PORT: u16 = 14_550;
pub const TCP_PORT: u16 = 14_553;

// ---------------- LED colours (0xRRGGBB) ----------------
pub const LED_OFF: u32 = 0x00_00_00;
pub const LED_WHITE: u32 = 0xFF_FF_FF;
pub const LED_CYAN: u32 = 0x00_FF_FF;
pub const LED_BLUE: u32 = 0x00_00_FF;
pub const LED_LILAC: u32 = 0x80_00_FF;
pub const LED_MAGENTA: u32 = 0xFF_00_FF;
pub const LED_YELLOW: u32 = 0xFF_FF_00;
pub const LED_GREEN: u32 = 0x00_FF_00;
pub const LED_RED: u32 = 0xFF_00_00;
pub const LED_CHERRY_D: u32 = 0xFF_00_40;

// ---------------- EKF health flags ----------------
pub const EKF_ATTITUDE: u16 = 0x01;
pub const EKF_POS_HORIZ_ABS: u16 = 0x10;

// ---------------- ArduPilot flight modes ----------------
pub const MODE_STABILIZE: u32 = 0;
pub const MODE_ACRO: u32 = 1;
pub const MODE_AUTO: u32 = 3;
pub const MODE_LAND: u32 = 9;

// ---------------- Timeouts (ms) ----------------
pub const WIFI_TIMEOUT_MS: u32 = 60_000;
pub const ARM_RETRY_INTERVAL_MS: u32 = 5_000;
pub const MODE_RETRY_INTERVAL_MS: u32 = 10_000;
pub const REASON_REPORT_MS: u32 = 30_000;

// ---------------- Magnetometer calibration ----------------
pub const STATUS_QUEUE_SIZE: usize = 16;
pub const REASON_QUEUE_SIZE: usize = 12;
pub const DIA_TOLERANCE: f32 = 0.15;
pub const CAL_MAX_RETRIES: u8 = 5;

/// `MAV_CMD_DO_START_MAG_CAL` may be missing from older MAVLink headers.
pub const MAV_CMD_DO_START_MAG_CAL: u16 = 42_424;

// ---------------- MAVLink enums ----------------
pub const MAV_COMP_ID_AUTOPILOT1: u8 = 1;
pub const MAV_COMP_ID_ONBOARD_COMPUTER: u8 = 191;

pub const MAV_TYPE_ONBOARD_CONTROLLER: u8 = 18;
pub const MAV_AUTOPILOT_INVALID: u8 = 8;

pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 128;
pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: u8 = 1;

pub const MAV_CMD_COMPONENT_ARM_DISARM: u16 = 400;
pub const MAV_CMD_DO_SET_MODE: u16 = 176;
pub const MAV_CMD_DO_SET_RELAY: u16 = 181;
pub const MAV_CMD_DO_ACCEPT_MAG_CAL: u16 = 42_425;
pub const MAV_CMD_PREFLIGHT_STORAGE: u16 = 245;
pub const MAV_CMD_NAV_LAND: u16 = 21;

/// Battery / status severity levels.
pub const MAV_SEVERITY_EMERGENCY: u8 = 0;
pub const MAV_SEVERITY_WARNING: u8 = 4;
pub const MAV_SEVERITY_INFO: u8 = 6;

pub const MAV_SYS_STATUS_SENSOR_3D_MAG: u32 = 4;
pub const MAV_MISSION_ACCEPTED: u8 = 0;
pub const MAV_MISSION_TYPE_MISSION: u8 = 0;
pub const MAV_LANDED_STATE_ON_GROUND: u8 = 1;

/// Radians -> degrees, matching the C++ `RAD_TO_DEG` macro.
pub const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

// ---------------- System states ----------------
/// The bridge's finite state machine, numerically identical to the original
/// `enum SystemState : uint8_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum State {
    InitWifi = 1,
    InitMavlink = 2,
    MagError = 3,
    MagOk = 4,
    Calibration = 5,
    CalibrationEnd = 6,
    NoArm = 7,
    Arming = 8,
    Armed = 9,
    StartMission = 10,
    Mission = 11,
    RelayControl = 12,
}

impl State {
    pub fn from_u8(v: u8) -> Option<State> {
        use State::*;
        Some(match v {
            1 => InitWifi,
            2 => InitMavlink,
            3 => MagError,
            4 => MagOk,
            5 => Calibration,
            6 => CalibrationEnd,
            7 => NoArm,
            8 => Arming,
            9 => Armed,
            10 => StartMission,
            11 => Mission,
            12 => RelayControl,
            _ => return None,
        })
    }

    pub fn as_u8(self) -> u8 {
        self as u8
    }

    /// The `STATE_ARMED` value (9) is defined by the original enum but never
    /// used by the C++ state machine; kept for completeness.
    pub fn is_armed_state(self) -> bool {
        matches!(self, State::Armed)
    }
}

/// `mdfly == 60` means "flight loop disabled" (mode not STABILIZE/AUTO, or a
/// calibration failure that requires a power cycle).
pub const MDFLY_STOP: i32 = 60;
