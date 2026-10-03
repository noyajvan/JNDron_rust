//! Board-specific constants for the JNDron firmware.
//!
//! These mirror `firmware/include/config.h` from the original C++ project.

// The relay's address is deliberately *not* here. This repository is public, and
// the relay accepts any client, so a hard-coded address publishes an invitation to
// command the aircraft. It lives in NVS on the device instead: `HOST=<ip>` on the
// console, then `SAVE`. See `Config::gcs_host`.

/// Outgoing TCP relay port on the VPS (primary, reliable transport).
pub const GCS_PORT_TCP: u16 = 14_553;
/// UDP fallback port (drained by the VPS and by Mission Planner).
pub const GCS_PORT_UDP: u16 = 14_550;

/// WS2812 status LED data pin (GPIO48 on the DevKitC-1).
pub const LED_PIN: u8 = 48;
/// RMT channel used to drive the WS2812 (0..=3 on the ESP32-S3).
pub const LED_RMT_CHANNEL: u8 = 0;

/// Credentials a board with nothing stored starts with, so it connects without
/// any setup step. They are only defaults: `SSID=`/`PASS=`/`SAVE` on the console
/// replace them.
pub const DEFAULT_SSID: &str = "LEO";
pub const DEFAULT_PASS: &str = "88888888";

/// NVS namespace holding the Wi-Fi credentials (matches the C++ firmware).
pub const NVS_NAMESPACE: &str = "dbridge";

/// Bytes held back when the relay socket cannot take them immediately.
///
/// A parameter download arrives as one long burst (ArduPilot sends a list of
/// 1129 parameters, about 42 KiB, on request) and that does not fit in a socket
/// send buffer. Queuing it here instead of dropping it is what makes Mission
/// Planner's "Getting params" finish.
pub const TCP_TX_QUEUE: usize = 64 * 1024;
// Flight-controller UART pins on the ESP32-S3-DevKitC-1: TX = GPIO43,
// RX = GPIO44. They are wired directly in `Platform::new`, because the HAL
// needs the concrete pin singletons rather than a number.
