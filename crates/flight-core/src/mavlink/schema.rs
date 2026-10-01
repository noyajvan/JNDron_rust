//! MAVLink message *schemas*.
//!
//! A schema is only needed for two things:
//!  1. computing the per-message `CRC_EXTRA` byte, and
//!  2. validating that our hand-written field layouts match the wire order.
//!
//! The field ordering rule below is the one `mavgen` uses: fields are sorted
//! by decreasing primitive size, stably (i.e. fields of equal size keep their
//! XML declaration order). Message *extensions* are excluded from `CRC_EXTRA`.

use super::crc::crc16_x25;

/// Primitive MAVLink field types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ty {
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    I64,
    F32,
    F64,
    Char,
}

impl Ty {
    /// Size in bytes of one element.
    pub const fn size(self) -> usize {
        use Ty::*;
        match self {
            U8 | I8 | Char => 1,
            U16 | I16 => 2,
            U32 | I32 | F32 => 4,
            U64 | I64 | F64 => 8,
        }
    }

    /// The exact type spelling `mavgen` feeds into the CRC.
    pub const fn name(self) -> &'static str {
        use Ty::*;
        match self {
            U8 => "uint8_t",
            I8 => "int8_t",
            U16 => "uint16_t",
            I16 => "int16_t",
            U32 => "uint32_t",
            I32 => "int32_t",
            U64 => "uint64_t",
            I64 => "int64_t",
            F32 => "float",
            F64 => "double",
            Char => "char",
        }
    }
}

/// One field of a message. `array == 0` means a scalar.
#[derive(Debug, Clone, Copy)]
pub struct Field {
    pub name: &'static str,
    pub ty: Ty,
    pub array: usize,
    /// `true` for MAVLink 2 *extension* fields (excluded from CRC_EXTRA and
    /// allowed to be absent on the wire).
    pub extension: bool,
}

/// Shorthand for declaring scalar/array fields in the tables below.
pub const fn scalar(name: &'static str, ty: Ty) -> Field {
    Field {
        name,
        ty,
        array: 0,
        extension: false,
    }
}

pub const fn array(name: &'static str, ty: Ty, n: usize) -> Field {
    Field {
        name,
        ty,
        array: n,
        extension: false,
    }
}

pub const fn ext(name: &'static str, ty: Ty) -> Field {
    Field {
        name,
        ty,
        array: 0,
        extension: true,
    }
}

/// A message definition.
#[derive(Debug, Clone, Copy)]
pub struct MsgDef {
    pub id: u32,
    pub name: &'static str,
    pub fields: &'static [Field],
}

impl MsgDef {
    /// Base (non-extension) fields, in wire order.
    pub fn wire_fields(&self) -> Vec<Field> {
        let mut v: Vec<Field> = self
            .fields
            .iter()
            .copied()
            .filter(|f| !f.extension)
            .collect();
        // Stable sort by decreasing element size (single-key form).
        v.sort_by_key(|f| std::cmp::Reverse(f.ty.size()));
        v
    }

    /// Byte length of the base payload (extensions omitted).
    pub fn base_len(&self) -> usize {
        self.wire_fields()
            .iter()
            .map(|f| f.ty.size() * f.array.max(1))
            .sum()
    }

    /// The message's `CRC_EXTRA` byte, computed exactly as `mavgen` does.
    pub fn crc_extra(&self) -> u8 {
        let mut buf: Vec<u8> = Vec::new();
        buf.extend_from_slice(self.name.as_bytes());
        buf.push(b' ');
        for f in self.wire_fields() {
            buf.extend_from_slice(f.ty.name().as_bytes());
            buf.push(b' ');
            buf.extend_from_slice(f.name.as_bytes());
            buf.push(b' ');
            if f.array > 0 {
                buf.push(f.array as u8);
            }
        }
        let crc = crc16_x25(&buf);
        ((crc & 0xFF) as u8) ^ ((crc >> 8) as u8)
    }
}

#[cfg(test)]
mod tests {
    use crate::messages::defs;

    /// The CRC_EXTRA values below are the canonical, published values from the
    /// MAVLink `common`/`ardupilotmega` dialects. If any hand-written field
    /// layout in `messages::defs` diverges from the real dialect, one of these
    /// assertions fires - which is exactly the safety net we want.
    #[test]
    fn crc_extra_matches_canonical_values() {
        assert_eq!(defs::HEARTBEAT.crc_extra(), 50, "HEARTBEAT");
        assert_eq!(defs::SYS_STATUS.crc_extra(), 124, "SYS_STATUS");
        assert_eq!(defs::ATTITUDE.crc_extra(), 39, "ATTITUDE");
        assert_eq!(defs::VFR_HUD.crc_extra(), 20, "VFR_HUD");
        assert_eq!(defs::GPS_RAW_INT.crc_extra(), 24, "GPS_RAW_INT");
        assert_eq!(defs::RAW_IMU.crc_extra(), 144, "RAW_IMU");
        assert_eq!(defs::STATUSTEXT.crc_extra(), 83, "STATUSTEXT");
        assert_eq!(defs::COMMAND_LONG.crc_extra(), 152, "COMMAND_LONG");
        assert_eq!(defs::MISSION_COUNT.crc_extra(), 221, "MISSION_COUNT");
        assert_eq!(defs::MISSION_ITEM_INT.crc_extra(), 38, "MISSION_ITEM_INT");
        assert_eq!(defs::MISSION_ACK.crc_extra(), 153, "MISSION_ACK");
        assert_eq!(
            defs::MISSION_REQUEST_LIST.crc_extra(),
            132,
            "MISSION_REQUEST_LIST"
        );
        assert_eq!(
            defs::EXTENDED_SYS_STATE.crc_extra(),
            130,
            "EXTENDED_SYS_STATE"
        );
        assert_eq!(defs::EKF_STATUS_REPORT.crc_extra(), 71, "EKF_STATUS_REPORT");
    }
}
