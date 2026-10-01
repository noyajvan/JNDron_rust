//! # flight-core
//!
//! Portable flight logic for **JNDron**, a Rust re-implementation of the
//! `DroneBridge` firmware (originally ESP32-S3 / Arduino / C++).
//!
//! The crate contains *no hardware access at all*. Every side effect is routed
//! through the [`io::Io`] trait, which makes the whole bridge - the protocol
//! codec, the crash detector, the LED logic and the state machine - testable on
//! a normal desktop with `cargo test`.
//!
//! The ESP32 firmware crate (`firmware/`) only provides an `Io` implementation
//! backed by UART, Wi-Fi, UDP/TCP and the NeoPixel driver.
//!
//! ## Layout
//! * [`mavlink`]  - dependency-free MAVLink v1/v2 codec (CRC-16/X.25 + CRC_EXTRA)
//! * [`messages`] - typed pack/unpack for the messages the bridge speaks
//! * [`bridge`]   - the central [`bridge::Bridge`] state object
//! * [`crash`]    - crash / hard-landing detection
//! * [`led`]      - LED colour + breathing logic
//! * [`queues`]   - the STATUS/RESPONSE ring buffers
//! * [`config`]   - persistent configuration model
//! * [`terminal`] - USB serial command parser
//! * [`io`]       - the hardware abstraction trait

pub mod bridge;
pub mod config;
pub mod consts;
pub mod crash;
pub mod io;
pub mod led;
pub mod mavlink;
pub mod messages;
pub mod queues;
pub mod terminal;

pub use bridge::Bridge;
pub use config::Config;
pub use consts::{State, MAV_COMP_ID_ONBOARD_COMPUTER};
pub use io::Io;

/// Convenience: decode the numeric `mdfly` field into a human string.
pub fn mdfly_label(mdfly: i32) -> &'static str {
    if mdfly == 60 {
        "stopped"
    } else {
        "running"
    }
}
