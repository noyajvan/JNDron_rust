# Porting notes: NDron (C++/Arduino) → JNDron (Rust)

This document maps every original source file to its Rust counterpart and
records the behavioural decisions taken during the port.

## File map

| Original (C++) | Rust | Notes |
|---|---|---|
| `firmware/src/main_DrnBrdg.cpp` | `crates/flight-core/src/bridge.rs` (`Bridge::boot`, `Bridge::tick`) + `firmware/src/main.rs` | Globals became fields on `Bridge`; `setup()` split into `boot()` + platform init; `loop()` is `tick()` plus the platform pumps. |
| `firmware/src/state_machine.cpp` | `crates/flight-core/src/bridge.rs` (`update_system_state`, `step_*`) | Every `static` local was lifted into `FsmStatics`. |
| `firmware/src/mavlink_util.cpp` | `crates/flight-core/src/bridge.rs` (`handle_message`, `forward_to_wifi`, `feed_*`) + `crates/flight-core/src/messages.rs` | `mavlink_*_pack/decode` replaced by the in-tree codec. Crash detection moved to `crash.rs`. |
| `firmware/src/led.cpp` | `crates/flight-core/src/led.rs` | `updateLED()` is now a pure function returning `LedOut { color, brightness }`. |
| `firmware/src/terminal.cpp` | `crates/flight-core/src/terminal.rs` | `handleTerminalConfig()` → `Bridge::terminal_char` / `Bridge::run_command`. |
| `firmware/src/config.cpp` | `crates/flight-core/src/config.rs` + `firmware/src/platform.rs` (`Platform::save_config`, NVS) | `Preferences` → `EspNvs`, namespace `dbridge`. |
| `firmware/src/wifi_mgr.cpp` | `crates/flight-core/src/bridge.rs` (policy) + `firmware/src/platform.rs` (driver) | `tcpLinkService()` is now `Platform::service_tcp()`; the Wi-Fi watchdogs stay in the core. |
| `firmware/include/config.h` | `crates/flight-core/src/consts.rs` + `firmware/src/consts.rs` | Pins/ports in the firmware crate, protocol constants in the core. |
| `firmware/include/fsm_types.h` | `crates/flight-core/src/consts.rs` | `SystemState` → `State` (`#[repr(u8)]`, identical numbers). |
| `firmware/platformio.ini` | `firmware/Cargo.toml`, `firmware/.cargo/config.toml`, `firmware/sdkconfig.defaults` | PlatformIO → Cargo + ESP-IDF. |

## Behavioural equivalences worth calling out

1. **CRC_EXTRA is computed, not memorised.** `schema::MsgDef::crc_extra()`
   reimplements `mavgen`'s algorithm (`name`, then each wire-ordered field's
   type and name, with array lengths). `crc_extra_matches_canonical_values`
   asserts 14 well-known values, so a field-list typo fails the build.
2. **Wire order is derived, not hand-written twice.** `wire_fields()` performs
   the same stable sort by decreasing element size that `mavgen` does; the
   pack/unpack helpers follow that order and are covered by tests.
3. **MAVLink 2 truncation.** `encode_v2` strips trailing zero bytes before
   computing the checksum, like `mavlink_msg_to_send_buffer`.
4. **Forwarding preserves the original header.** Telemetry re-emitted towards
   the GCS keeps the FC's `sysid`/`compid`/`seq`. The output is always
   MAVLink 2.
5. **Unknown message ids.** The core forwards frames whose `CRC_EXTRA` it does
   not know without verifying them; known ids are always verified and dropped
   on mismatch.
6. **Unsigned-time arithmetic.** All timers use `u32::wrapping_sub`, matching
   Arduino `millis()` overflow behaviour.
7. **Buffer bound.** The frame parser can hold at most `10 + 255 + 2` bytes,
   because a candidate frame is parsed (and dropped on checksum failure) as
   soon as it is complete. `Parser::MAX_BUFFERED` documents this and
   `random_garbage_never_panics_and_keeps_the_buffer_bounded` enforces it.

## Deliberate differences

* **No Arduino `String`** - the console buffer and config use `String`
  (allocated) as before, but the queues use fixed-size rings with 71-byte
  truncation, identical to the `char[N][72]` arrays.
* **`send_statustext` / `send_statustext_udp` split** is kept: the former writes
  to the FC only, the latter to the radio only.
* **`cal_accept_sent`, `land_stop_ms`, `tip_bounce_ms`, ...** are fields, not
  `static` locals, so two bridges in the same process do not interfere (the C++
  code could not have two).
* **`STATE_ARMED` (9)** exists in the enum for parity but, as in the original,
  the state machine never enters it.
* **MAVLink 2 only.** MAVLink 1 support was dropped deliberately: message ids
  fit in 8 bits there, extensions do not exist, and both ArduPilot (once it
  sees a MAVLink 2 peer, which this bridge is) and Mission Planner speak
  MAVLink 2. A `0xFE` byte is treated as stream noise, so a MAVLink 1 frame is
  skipped rather than converted. `encode_v1` and `Frame::version` are gone.
* **Dead globals dropped.** `arm_cmd_sent`, `mode_cmd_sent`, `last_arm_retry_ms`,
  `last_mode_retry_ms` and `last_reason_report_ms` were declared and (some)
  assigned in the C++ but never read; they are gone, along with the unreachable
  `STATE_START_MISSION` arm.
* **No `unwrap`/`panic!` in library code.** The only `unwrap()` calls in the
  crate are inside `#[cfg(test)]` code.

## Testing strategy

| Layer | How it is checked |
|---|---|
| CRC / framing | Known-answer tests (`x25_known_vectors`), round-trip, corruption rejection, resync, truncation. |
| CRC independence | `tests/reliability.rs` re-implements CRC-16/X.25 bit-at-a-time and compares it against the production nibble implementation over every byte value and random buffers up to 4 KiB. |
| Corruption | Every single bit of every byte in a frame's payload/checksum is flipped and must be rejected. |
| Hostile input | 50 000 pseudo-random bytes (with start markers sprinkled in) must never panic and must never grow the parser buffer past `Parser::MAX_BUFFERED`. |
| Length byte | All 256 declared payload lengths round-trip. |
| Truncation | Every truncation of every known message decodes without panicking. |
| Message layouts | Canonical `CRC_EXTRA`, byte-offset assertions, pack/decode round-trips. |
| Crash detection | Six scenarios including "fell then flew again". |
| LED | Blink/breathe tables, calibration breathing speed-up. |
| FSM | Boot → MAG_OK, rotation → calibration → success, bad-DIA retry, mode guard stop, landing relay, crash relay. |
| Transport | TCP forwarding, UDP double-send, MAVLink 1 rejection, GCS→FC, calibration-spam suppression. |
| Console | Every command, persistence, restart, junk input. |

Run with:

```bash
cargo test
```
