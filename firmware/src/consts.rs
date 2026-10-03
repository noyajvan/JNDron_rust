//! Board-specific constants for the JNDron firmware.
//!
//! These mirror `firmware/include/config.h` from the original C++ project.

use std::net::Ipv4Addr;

/// Oracle VPS that relays telemetry to Mission Planner.
pub const GCS_IP: Ipv4Addr = Ipv4Addr::new(152, 70, 51, 224);
/// Outgoing TCP relay port on the VPS (primary, reliable transport).
pub const GCS_PORT_TCP: u16 = 14_553;
/// UDP fallback port (drained by the VPS and by Mission Planner).
pub const GCS_PORT_UDP: u16 = 14_550;

/// WS2812 status LED data pin (GPIO48 on the DevKitC-1).
pub const LED_PIN: u8 = 48;
/// RMT channel used to drive the WS2812 (0..=3 on the ESP32-S3).
pub const LED_RMT_CHANNEL: u8 = 0;

/// Name advertised over BLE while waiting for Wi-Fi credentials.
pub const PROV_DEVICE_NAME: &str = "JNDron";

/// Credentials a board with nothing stored starts with, so it connects without
/// any setup step. They are only defaults: `SSID=`/`PASS=`/`SAVE` on the console
/// replace them.
pub const DEFAULT_SSID: &str = "LEO";
pub const DEFAULT_PASS: &str = "88888888";

/// Wi-Fi provisioning over BLE is written and working (`provisioning.rs`), but
/// parked for now: the built-in credentials above mean a board is never without
/// a network, and keeping BLE off keeps the radio free for Wi-Fi and saves the
/// ~290 KiB it costs. Flip this to `true` to bring the BLE setup path back.
pub const ENABLE_BLE_PROVISIONING: bool = false;

/// NVS namespace holding the Wi-Fi credentials (matches the C++ firmware).
pub const NVS_NAMESPACE: &str = "dbridge";
// Flight-controller UART pins on the ESP32-S3-DevKitC-1: TX = GPIO43,
// RX = GPIO44. They are wired directly in `Platform::new`, because the HAL
// needs the concrete pin singletons rather than a number.
