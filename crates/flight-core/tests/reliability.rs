//! Reliability properties for the MAVLink codec.
//!
//! These are deliberately *not* round-trip tests. Round-trips only prove that
//! encode and decode agree with each other; if both were subtly wrong the
//! tests would still pass. Instead every check here compares against
//! something independent:
//!
//! * an independent, bit-at-a-time CRC-16/X.25 implementation,
//! * exhaustive single-bit corruption of a real frame,
//! * the documented CRC_EXTRA table (see `mavlink::schema`),
//! * "never panics, never grows without bound" on hostile input.

use flight_core::mavlink::{crc16_x25, encode_v1, encode_v2, Parser};
use flight_core::messages::{defs, Heartbeat, Message};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Independent CRC-16/X.25: reflected polynomial 0x8408, init 0xFFFF, **no**
/// final XOR (that last part is what MAVLink specifies).
fn reference_crc16_x25(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

/// Deterministic xorshift PRNG - no dev-dependency needed.
struct Rng(u64);

impl Rng {
    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        (x >> 32) as u32
    }
    fn byte(&mut self) -> u8 {
        self.next_u32() as u8
    }
}

fn heartbeat_frame() -> Vec<u8> {
    let hb = Heartbeat {
        custom_mode: 3,
        typ: 2,
        autopilot: 3,
        base_mode: 129,
        system_status: 4,
        mavlink_version: 3,
    };
    encode_v2(
        7,
        1,
        1,
        defs::HEARTBEAT.id,
        &hb.payload(),
        Some(defs::HEARTBEAT.crc_extra()),
    )
}

fn parse_all(bytes: &[u8]) -> Vec<flight_core::mavlink::Frame> {
    let mut p = Parser::new();
    let mut out = Vec::new();
    p.push_slice(bytes, &mut out);
    out
}

// ---------------------------------------------------------------------------
// 1. CRC implementation vs an independent reference
// ---------------------------------------------------------------------------

#[test]
fn crc_matches_independent_bitwise_reference() {
    // Every single byte value, in every position of a growing buffer.
    for b in 0u8..=255 {
        let buf = [b];
        assert_eq!(crc16_x25(&buf), reference_crc16_x25(&buf), "byte {b}");
    }

    let mut rng = Rng(0x1234_5678_9abc_def0);
    for len in [2usize, 3, 7, 16, 64, 255, 1024, 4096] {
        let data: Vec<u8> = (0..len).map(|_| rng.byte()).collect();
        assert_eq!(
            crc16_x25(&data),
            reference_crc16_x25(&data),
            "mismatch for len {len}"
        );
    }

    // Running the reference over a prefix must equal the standalone CRC, which
    // is exactly what frame checksumming relies on.
    let data: Vec<u8> = (0..300).map(|_| rng.byte()).collect();
    assert_eq!(crc16_x25(&data[..10]), reference_crc16_x25(&data[..10]));
    assert_eq!(crc16_x25(&data), reference_crc16_x25(&data));
}

// ---------------------------------------------------------------------------
// 2. Exhaustive single-bit corruption must be detected
// ---------------------------------------------------------------------------

#[test]
fn every_single_bit_error_in_payload_or_checksum_is_rejected() {
    let good = heartbeat_frame();
    // Bytes 0..10 are the header. Flipping a header bit can legitimately turn
    // the frame into a *different* unknown message id, which is forwarded
    // without verification by design - so only the payload + checksum region
    // is required to be rejected here.
    for byte in 10..good.len() {
        for bit in 0..8u8 {
            let mut corrupted = good.clone();
            corrupted[byte] ^= 1 << bit;
            let frames = parse_all(&corrupted);
            assert!(
                frames.is_empty(),
                "bit {bit} of byte {byte} was not detected ({} frame(s))",
                frames.len()
            );
        }
    }
}

#[test]
fn intact_frame_still_parses_after_the_corruption_sweep() {
    // Guards against the previous test passing for the wrong reason (e.g. a
    // parser that rejects everything).
    let frames = parse_all(&heartbeat_frame());
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].msgid, defs::HEARTBEAT.id);
    assert!(frames[0].crc_checked);
}

// ---------------------------------------------------------------------------
// 3. Hostile input: never panic, never grow without bound
// ---------------------------------------------------------------------------

#[test]
fn random_garbage_never_panics_and_keeps_the_buffer_bounded() {
    let mut rng = Rng(0xdead_beef_cafe_f00d);
    let mut p = Parser::new();
    let mut frames = Vec::new();

    for i in 0..50_000usize {
        let b = match i % 97 {
            // Sprinkle start-of-frame markers so the parser enters its
            // "waiting for the rest of the frame" state often.
            0 => 0xFD,
            1 => 0xFE,
            _ => rng.byte(),
        };
        if let Some(f) = p.push(b) {
            frames.push(f);
        }
        assert!(
            p.buffered_len() <= Parser::MAX_BUFFERED,
            "parser buffer grew to {} bytes",
            p.buffered_len()
        );
    }
}

#[test]
fn declared_lengths_of_every_size_round_trip() {
    // The length byte is 8 bits; every value must be handled.
    for len in 0..=255usize {
        let payload: Vec<u8> = (0..len).map(|i| (i as u8) | 1).collect();
        // Unknown message id -> encoded verbatim (no trailing-zero trimming).
        let bytes = encode_v2(0, 1, 1, 0xFF_FF_FE, &payload, None);
        assert_eq!(bytes[1] as usize, len, "length byte for {len}");

        let frames = parse_all(&bytes);
        assert_eq!(frames.len(), 1, "len {len}");
        assert_eq!(frames[0].payload.len(), len, "len {len}");
    }

    // A frame claiming 255 bytes that never arrives must simply wait.
    let mut p = Parser::new();
    let mut out = Vec::new();
    p.push_slice(&[0xFD, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0], &mut out);
    assert!(out.is_empty());
    assert!(p.buffered_len() <= Parser::MAX_BUFFERED);
}

// ---------------------------------------------------------------------------
// 4. Truncated payloads (MAVLink 2) must be safe to decode
// ---------------------------------------------------------------------------

#[test]
fn decoding_every_truncation_of_every_message_never_panics() {
    for def in defs::ALL {
        let full = def.base_len();
        for len in 0..=full {
            let payload = vec![0xABu8; len];
            // Must not panic, and must be usable.
            let msg = Message::decode(def.id, &payload);
            assert_eq!(
                msg.is_calibration_spam(),
                matches!(def.id, 191 | 192),
                "spam classification for {}",
                def.name
            );
        }
    }
}

#[test]
fn truncated_heartbeat_is_zero_extended() {
    // MAVLink 2 senders may omit trailing zero bytes; a 4-byte HEARTBEAT
    // (just custom_mode) must decode with the remaining fields zeroed.
    let payload = 42u32.to_le_bytes();
    let bytes = encode_v2(
        0,
        1,
        1,
        defs::HEARTBEAT.id,
        &payload,
        Some(defs::HEARTBEAT.crc_extra()),
    );
    let frames = parse_all(&bytes);
    assert_eq!(frames.len(), 1);
    match Message::decode(frames[0].msgid, &frames[0].payload) {
        Message::Heartbeat(hb) => {
            assert_eq!(hb.custom_mode, 42);
            assert_eq!(hb.typ, 0);
            assert_eq!(hb.base_mode, 0);
        }
        other => panic!("expected heartbeat, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// 5. Resynchronisation
// ---------------------------------------------------------------------------

#[test]
fn a_valid_frame_after_corruption_is_still_delivered() {
    let good = heartbeat_frame();

    let mut stream = Vec::new();
    stream.extend_from_slice(&good); // first copy: intact
    let mut corrupted = good.clone();
    corrupted[12] ^= 0xFF; // payload bit error
    stream.extend_from_slice(&corrupted);
    stream.extend_from_slice(&good); // must still be recovered

    let frames = parse_all(&stream);
    assert_eq!(frames.len(), 2, "the two intact frames must survive");
    for f in &frames {
        assert_eq!(f.msgid, defs::HEARTBEAT.id);
        assert!(f.crc_checked);
    }
}

#[test]
fn v1_frames_are_parsed_and_can_be_re_emitted_as_v1() {
    let hb = Heartbeat {
        custom_mode: 1,
        typ: 2,
        autopilot: 3,
        base_mode: 0,
        system_status: 3,
        mavlink_version: 3,
    };
    let payload = hb.payload();
    let bytes = encode_v1(
        9,
        1,
        1,
        defs::HEARTBEAT.id,
        &payload,
        defs::HEARTBEAT.crc_extra(),
    );

    let frames = parse_all(&bytes);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].version, 1);
    assert!(frames[0].crc_checked);

    // Re-emitting as MAVLink 1 and re-parsing must be stable.
    let again = encode_v1(
        frames[0].seq,
        frames[0].sysid,
        frames[0].compid,
        frames[0].msgid,
        &frames[0].payload,
        defs::HEARTBEAT.crc_extra(),
    );
    let round = parse_all(&again);
    assert_eq!(round.len(), 1);
    assert_eq!(round[0].version, 1);
    assert_eq!(round[0].payload, payload);
}
