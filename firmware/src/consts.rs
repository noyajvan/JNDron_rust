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

/// WS2812 status LED data pin.
pub const LED_PIN: gpio_num_t = 48;
/// `Pixel::new_with_gamma` wants the raw voltage/colour-order flag from the
/// driver crate; GRB + 5 V is what the on-board LED expects.
pub const LED_VCC_GRB: u32 = 0;

/// NVS namespace holding the Wi-Fi credentials (matches the C++ firmware).
pub const NVS_NAMESPACE: &str = "dbridge";

/// Flight-controller UART pins (ESP32-S3-DevKitC-1).
pub const TX_GPIO: i32 = 43;
pub const RX_GPIO: i32 = 44;

/// Re-exported so `platform.rs` can name the pin type without pulling in the
/// HAL at the crate root.
pub type gpio_num_t = i32;
