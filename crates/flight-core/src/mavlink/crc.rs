//! CRC primitives used by MAVLink.
//!
//! MAVLink uses CRC-16/X.25 (`crc16_x25`) for the frame checksum and a 1-byte
//! `CRC_EXTRA` derived from each message definition (see [`super::schema`]).

/// CRC-16/X.25, the exact nibble algorithm used by `mavgen`'s `x25crc`.
pub fn crc16_x25(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        let mut tmp = (b as u16) ^ (crc & 0xFF);
        tmp ^= tmp << 4;
        // The low byte is what feeds the feedback terms; `mavgen` masks here.
        tmp &= 0xFF;
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn x25_known_vectors() {
        // MAVLink uses the raw CRC-16/X.25 accumulator *without* the final
        // XOR-0xFFFF, so the classic check vector 0x906E becomes 0x6F91.
        assert_eq!(crc16_x25(b"123456789"), 0x6F91);
        // Empty input yields the initial value.
        assert_eq!(crc16_x25(&[]), 0xFFFF);
    }
}
