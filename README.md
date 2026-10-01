# JNDron_rust

A **Rust re-implementation** of [`noyajvan/NDron`](https://github.com/noyajvan/NDron)
(project *DroneBridge*) - an ESP32-S3 telemetry bridge that links an ArduPilot
flight controller to Mission Planner through a phone hotspot, 4G and a VPS
relay.

The original firmware was C++ / Arduino. This project ports it to Rust, with a
hard split between **testable protocol logic** and **hardware glue**.

```text
 Дрон (ESP32-S3) ─UART─▶ flight controller (ArduPilot)
        │
        └─ Wi-Fi STA ─▶ 📱 hotspot ─▶ 4G ─▶ ☁️ VPS ─▶ 🌐 ─▶ 💻 Mission Planner
```

## Repository layout

| Path | What |
|---|---|
| `crates/flight-core/` | Portable, dependency-free, **host-tested** logic: MAVLink codec, crash detector, LED logic, FSM, console parser, config. |
| `firmware/` | ESP32-S3 binary (`esp-idf-svc`). The only place that touches UART / Wi-Fi / sockets / NVS / NeoPixel. |

`cargo test` at the repository root only builds `flight-core`; the firmware
crate is excluded because it targets `xtensa-esp32s3-espidf`.

## Quick start

```bash
# Logic tests - no hardware, no network, no heavy downloads.
cargo test

# Build the firmware (requires the Xtensa toolchain + ESP-IDF)
cargo install espup && espup install
cargo install espflash
cd firmware
cargo run --release
```

## Why the split?

The interesting behaviour in a telemetry bridge is *protocol and state
machine*, not GPIO. By routing every side effect through the `flight_core::io::Io`
trait, the whole bridge can be exercised on a desktop:

* 56 unit/integration tests run in ~10 ms (`cargo test`),
* `MockIo` records every byte written to the FC UART, the TCP relay and UDP,
* the state machine can be driven through a full flight (boot → MAG_OK →
  calibration → ARM → AUTO → mission → LAND → relay) in milliseconds,
* `tests/reliability.rs` cross-checks the codec against an *independent*
  bit-at-a-time CRC implementation, exhaustive single-bit corruption and
  hostile input, rather than only round-tripping against itself.

See [`docs/PORTING.md`](docs/PORTING.md) for the file-by-file C++ → Rust map
and the behavioural notes.

## What `flight-core` contains

* **`mavlink`** - a dependency-free MAVLink **2** implementation (MAVLink 1 is
  not spoken; `0xFE` bytes are treated as noise):
  * CRC-16/X.25 (the exact `mavgen` nibble algorithm, *without* the final
    XOR-0xFFFF),
  * message schemas and `CRC_EXTRA` **computed from the field layouts** rather
    than hard-coded, so a wrong field list is caught by a test,
  * a streaming frame parser with CRC verification and resynchronisation,
  * MAVLink 2 payload truncation of trailing zeros.
* **`messages`** - typed pack/unpack for HEARTBEAT, SYS_STATUS, ATTITUDE,
  VFR_HUD, RAW_IMU, GPS_RAW_INT, EKF_STATUS_REPORT, EXTENDED_SYS_STATE,
  STATUSTEXT, MAG_CAL_PROGRESS/REPORT, MISSION_* and COMMAND_LONG.
* **`bridge`** - the finite state machine, telemetry polling, mission
  bookkeeping, Wi-Fi watchdogs and the TCP/UDP forwarding policy.
* **`crash`** - Net / Tumble / RapidDescent detection gated on "actually flew".
* **`led`** - the blink/breathe patterns of the on-board WS2812.
* **`terminal`** - the USB console command set.

## Behaviour preserved from the C++ firmware

* Wi-Fi credentials and all timers behave as before (60 s activation timeout,
  retry every 30 s, "radio dead" restart, 30 s stuck-link restart).
* The flight loop stops (`mdfly = 60`) when the FC is not in `STABILIZE`/`AUTO`.
* The blue "calibration window" only opens once the EKF attitude is stable
  **and** the magnetometer is healthy, with the same 30 s/60 s force timeouts.
* Compass-calibration messages (`MAG_CAL_PROGRESS`/`REPORT`) are never relayed.
* Telemetry is relayed over TCP when the relay is up, otherwise over UDP
  (twice, with the 250 ms backoff).
* The relay fires on mission end / landed / tip-bounce / crash, but only if the
  drone actually flew.
* `sys_id` is forced to `1`, exactly like the original `loadConfig()`.

## Fidelity notes / intentional differences

* The Rust core parses only the messages it understands; **other** MAVLink
  messages are still forwarded, just without CRC verification (their CRC_EXTRA
  is unknown). The original included the full `ardupilotmega` dialect and could
  verify everything.
* Relayed frames are always re-emitted as MAVLink 2, keeping the FC's
  `sysid`/`compid`/`seq`. MAVLink 1 input is ignored, not converted.
* The console is polled character-by-character (`terminal_char`), matching the
  original `Serial` loop.
* `esp-idf-svc`/`esp-idf-hal` versions are pinned to the ESP-IDF 5.1 era to
  match the Arduino core the firmware used; the HAL constructors in
  `firmware/src/platform.rs` are the only place likely to need tweaking if you
  move to a newer ESP-IDF.

## Console commands

```text
STATUS | SSID=name | PASS=pass | BAUD= | SYSID= | WIFI OFF | WIFI ON | RELAY | DISARM | SAVE
```

## License

MIT - see [LICENSE](LICENSE).
