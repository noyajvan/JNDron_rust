//! Dependency-free MAVLink 1/2 codec.
//!
//! * [`crc`]     - CRC-16/X.25
//! * [`schema`]  - message field layouts + CRC_EXTRA computation
//! * [`frame`]   - framing, payload writer/reader, streaming parser
//!
//! The typed message structs and pack/unpack helpers live in
//! [`crate::messages`].

pub mod crc;
pub mod frame;
pub mod schema;

pub use crc::crc16_x25;
pub use frame::{encode_v1, encode_v2, Frame, Parser, Reader, Writer};
pub use schema::{Field, MsgDef, Ty};
