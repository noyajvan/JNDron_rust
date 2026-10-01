//! MAVLink v1/v2 framing: little-endian payload writer/reader, frame encoder
//! and an incremental streaming parser.

use super::crc::crc16_x25;
use super::schema::MsgDef;

pub const STX_V1: u8 = 0xFE;
pub const STX_V2: u8 = 0xFD;

/// A decoded MAVLink frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Protocol version: 1 or 2.
    pub version: u8,
    pub seq: u8,
    pub sysid: u8,
    pub compid: u8,
    pub msgid: u32,
    /// Raw payload (exactly `len` bytes from the wire).
    pub payload: Vec<u8>,
    /// `true` when the message id is known and its CRC was verified.
    pub crc_checked: bool,
}

impl Frame {
    /// Re-encode this frame as a MAVLink 2 frame with the given sequence
    /// number. Unknown message ids are re-emitted without truncation.
    pub fn encode_v2(&self, seq: u8, def: Option<&MsgDef>) -> Vec<u8> {
        let extra = def.map(|d| d.crc_extra());
        encode_v2(
            seq,
            self.sysid,
            self.compid,
            self.msgid,
            &self.payload,
            extra,
        )
    }
}

// ---------------------------------------------------------------------------
// Payload writer / reader
// ---------------------------------------------------------------------------

/// Little-endian payload writer.
#[derive(Default)]
pub struct Writer {
    pub buf: Vec<u8>,
}

impl Writer {
    pub fn new() -> Self {
        Writer {
            buf: Vec::with_capacity(32),
        }
    }
    pub fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }
    pub fn i8(&mut self, v: i8) {
        self.buf.push(v as u8);
    }
    pub fn u16(&mut self, v: u16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn i16(&mut self, v: i16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn i32(&mut self, v: i32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn u64(&mut self, v: u64) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn f32(&mut self, v: f32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn f64(&mut self, v: f64) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    /// Fixed-width `char[N]` field: NUL-padded, silently truncated.
    pub fn char_array(&mut self, s: &str, n: usize) {
        let b = s.as_bytes();
        let take = b.len().min(n);
        self.buf.extend_from_slice(&b[..take]);
        self.buf.resize(self.buf.len() + (n - take), 0);
    }
    pub fn bytes(&mut self, b: &[u8]) {
        self.buf.extend_from_slice(b);
    }
    pub fn into_vec(self) -> Vec<u8> {
        self.buf
    }
}

/// Little-endian payload reader that zero-fills past the end of a truncated
/// MAVLink 2 payload (extension fields are often omitted).
pub struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Reader { data, pos: 0 }
    }

    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0u8; N];
        let end = (self.pos + N).min(self.data.len());
        if self.pos < end {
            out[..end - self.pos].copy_from_slice(&self.data[self.pos..end]);
        }
        self.pos += N;
        out
    }

    pub fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }
    pub fn i8(&mut self) -> i8 {
        self.u8() as i8
    }
    pub fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take::<2>())
    }
    pub fn i16(&mut self) -> i16 {
        i16::from_le_bytes(self.take::<2>())
    }
    pub fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take::<4>())
    }
    pub fn i32(&mut self) -> i32 {
        i32::from_le_bytes(self.take::<4>())
    }
    pub fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take::<8>())
    }
    pub fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take::<4>())
    }
    pub fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.take::<8>())
    }
    /// Read a NUL-terminated / NUL-padded `char[N]` field as a `String`.
    pub fn char_array(&mut self, n: usize) -> String {
        let mut raw = vec![0u8; n];
        let end = (self.pos + n).min(self.data.len());
        if self.pos < end {
            raw[..end - self.pos].copy_from_slice(&self.data[self.pos..end]);
        }
        self.pos += n;
        let stop = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
        String::from_utf8_lossy(&raw[..stop]).into_owned()
    }
    /// Skip `n` bytes.
    pub fn skip(&mut self, n: usize) {
        self.pos += n;
    }
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

/// Encode a MAVLink 2 frame. When `crc_extra` is `None` the payload is emitted
/// verbatim (used for pass-through of messages we have no schema for).
pub fn encode_v2(
    seq: u8,
    sysid: u8,
    compid: u8,
    msgid: u32,
    payload: &[u8],
    crc_extra: Option<u8>,
) -> Vec<u8> {
    let payload = if crc_extra.is_some() {
        trim_trailing_zeros(payload)
    } else {
        payload.to_vec()
    };
    let mut out = Vec::with_capacity(12 + payload.len());
    out.push(STX_V2);
    out.push(payload.len() as u8);
    out.push(0); // incompat_flags
    out.push(0); // compat_flags
    out.push(seq);
    out.push(sysid);
    out.push(compid);
    out.extend_from_slice(&msgid.to_le_bytes()[..3]);
    out.extend_from_slice(&payload);

    let mut crc = crc16_x25(&out[1..]);
    if let Some(x) = crc_extra {
        crc = crc16_x25_continue(crc, &[x]);
    }
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

/// Encode a MAVLink 1 frame (message ids must fit in 8 bits).
pub fn encode_v1(
    seq: u8,
    sysid: u8,
    compid: u8,
    msgid: u32,
    payload: &[u8],
    crc_extra: u8,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(8 + payload.len());
    out.push(STX_V1);
    out.push(payload.len() as u8);
    out.push(seq);
    out.push(sysid);
    out.push(compid);
    out.push(msgid as u8);
    out.extend_from_slice(payload);
    let crc = crc16_x25(&out[1..]);
    let crc = crc16_x25_continue(crc, &[crc_extra]);
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

/// Continue a CRC-16/X.25 over more bytes.
fn crc16_x25_continue(seed: u16, data: &[u8]) -> u16 {
    // Re-implemented by prefixing: x25 is streaming, so we rebuild the state.
    let mut crc = seed;
    for &b in data {
        let mut tmp = (b as u16) ^ (crc & 0xFF);
        tmp ^= tmp << 4;
        tmp &= 0xFF;
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    crc
}

fn trim_trailing_zeros(payload: &[u8]) -> Vec<u8> {
    let mut end = payload.len();
    while end > 0 && payload[end - 1] == 0 {
        end -= 1;
    }
    payload[..end].to_vec()
}

/// Incremental frame parser. Feed it bytes as they arrive from the flight
/// controller; it returns one [`Frame`] per complete message.
#[derive(Debug, Default, Clone)]
pub struct Parser {
    buf: Vec<u8>,
    pub frames_ok: u32,
    pub crc_errors: u32,
    pub bytes_in: u32,
}

impl Parser {
    pub fn new() -> Self {
        Parser::default()
    }

    /// Bytes currently held while waiting for the rest of a frame.
    ///
    /// Bounded by `10 + 255 + 2` for MAVLink 2, because a candidate frame is
    /// parsed (and then dropped on checksum failure) as soon as it is complete.
    pub fn buffered_len(&self) -> usize {
        self.buf.len()
    }

    /// The largest buffer the parser can hold. Used by tests to prove the
    /// parser cannot be made to grow without bound by hostile input.
    pub const MAX_BUFFERED: usize = 10 + 255 + 2;

    /// Feed a single byte, returning a frame if one just completed.
    pub fn push(&mut self, b: u8) -> Option<Frame> {
        self.bytes_in = self.bytes_in.wrapping_add(1);
        self.buf.push(b);
        self.parse_one()
    }

    /// Feed a whole slice, appending every decoded frame to `out`.
    pub fn push_slice(&mut self, data: &[u8], out: &mut Vec<Frame>) {
        for &b in data {
            if let Some(f) = self.push(b) {
                out.push(f);
            }
        }
    }

    fn parse_one(&mut self) -> Option<Frame> {
        loop {
            // Locate the start-of-frame marker, discarding leading noise.
            let start = match self.buf.iter().position(|&b| b == STX_V1 || b == STX_V2) {
                Some(s) => s,
                None => {
                    self.buf.clear();
                    return None;
                }
            };
            if start > 0 {
                self.buf.drain(..start);
            }

            let v2 = self.buf[0] == STX_V2;
            let hdr = if v2 { 10 } else { 6 };
            if self.buf.len() < hdr {
                return None;
            }
            let len = self.buf[1] as usize;
            let total = hdr + len + 2;
            if self.buf.len() < total {
                return None;
            }

            let hdr_bytes: Vec<u8> = self.buf[..hdr].to_vec();
            let payload: Vec<u8> = self.buf[hdr..hdr + len].to_vec();
            let rx_crc = u16::from_le_bytes([self.buf[total - 2], self.buf[total - 1]]);

            let (seq, sysid, compid, msgid) = if v2 {
                (
                    hdr_bytes[4],
                    hdr_bytes[5],
                    hdr_bytes[6],
                    (hdr_bytes[7] as u32)
                        | ((hdr_bytes[8] as u32) << 8)
                        | ((hdr_bytes[9] as u32) << 16),
                )
            } else {
                (
                    hdr_bytes[2],
                    hdr_bytes[3],
                    hdr_bytes[4],
                    hdr_bytes[5] as u32,
                )
            };

            let def = crate::messages::defs::find(msgid);
            let crc_checked = match def {
                Some(d) => {
                    let mut crc = crc16_x25(&self.buf[1..total - 2]);
                    crc = crc16_x25_continue(crc, &[d.crc_extra()]);
                    if crc != rx_crc {
                        self.crc_errors += 1;
                        self.buf.drain(..total);
                        continue;
                    }
                    true
                }
                None => false,
            };

            self.buf.drain(..total);
            self.frames_ok += 1;
            return Some(Frame {
                version: if v2 { 2 } else { 1 },
                seq,
                sysid,
                compid,
                msgid,
                payload,
                crc_checked,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::defs;

    #[test]
    fn roundtrip_heartbeat_v2() {
        let mut w = Writer::new();
        w.u32(42); // custom_mode
        w.u8(2); // type
        w.u8(8); // autopilot
        w.u8(128); // base_mode
        w.u8(4); // system_status
        w.u8(3); // mavlink_version
        let payload = w.into_vec();
        let bytes = encode_v2(7, 1, 191, 0, &payload, Some(defs::HEARTBEAT.crc_extra()));

        let mut p = Parser::new();
        let mut frames = Vec::new();
        for &b in &bytes {
            if let Some(f) = p.push(b) {
                frames.push(f);
            }
        }
        assert_eq!(frames.len(), 1);
        let f = &frames[0];
        assert_eq!(f.version, 2);
        assert_eq!(f.seq, 7);
        assert_eq!(f.sysid, 1);
        assert_eq!(f.compid, 191);
        assert_eq!(f.msgid, 0);
        assert!(f.crc_checked);
        assert_eq!(f.payload, payload);
    }

    #[test]
    fn parser_resyncs_after_garbage() {
        let mut p = Parser::new();
        let mut frames = Vec::new();
        p.push_slice(&[0x00, 0x11, 0x22, 0xFF], &mut frames);
        assert!(frames.is_empty());

        let mut w = Writer::new();
        w.u32(0);
        w.u8(0);
        w.u8(0);
        w.u8(0);
        w.u8(0);
        w.u8(3);
        let payload = w.into_vec();
        let bytes = encode_v2(1, 1, 1, 0, &payload, Some(defs::HEARTBEAT.crc_extra()));
        p.push_slice(&bytes, &mut frames);
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn corrupted_frame_is_rejected() {
        let mut w = Writer::new();
        w.u32(1);
        w.u8(2);
        w.u8(8);
        w.u8(0);
        w.u8(3);
        w.u8(3);
        let payload = w.into_vec();
        let mut bytes = encode_v2(1, 1, 1, 0, &payload, Some(defs::HEARTBEAT.crc_extra()));
        let n = bytes.len();
        bytes[n - 3] ^= 0xFF; // corrupt payload/checksum region

        let mut p = Parser::new();
        let mut frames = Vec::new();
        p.push_slice(&bytes, &mut frames);
        assert!(frames.is_empty());
    }

    #[test]
    fn trailing_zeros_are_trimmed() {
        let payload = [1u8, 2, 3, 0, 0, 0];
        let bytes = encode_v2(1, 1, 1, 0, &payload, Some(50));
        assert_eq!(bytes[1], 3, "payload should be truncated to 3 bytes");
    }

    #[test]
    fn truncated_payload_reads_as_zero() {
        // A reader over a 2-byte buffer can still pull a u32 (zero-extended).
        let mut r = Reader::new(&[0xAA, 0xBB]);
        assert_eq!(r.u32(), 0x0000_BBAA);
        assert_eq!(r.u32(), 0);
    }

    #[test]
    fn char_array_roundtrip() {
        let mut w = Writer::new();
        w.char_array("hi", 6);
        let v = w.into_vec();
        assert_eq!(v, vec![b'h', b'i', 0, 0, 0, 0]);
        let mut r = Reader::new(&v);
        assert_eq!(r.char_array(6), "hi");
    }
}
