# Project state and handoff

Living document. It exists so that work can resume **without the previous
conversation**: read this file, then `docs/TROUBLESHOOTING.md`, then the board file
`C:\Projects\_boards\ESP32-S3-SuperMini.md`. Everything else is in git.

## Working agreement for context

Jcode compacts a session automatically at **80%** context usage, so the risk is not
losing the transcript, it is losing *reasoning that was never written down*. To
avoid both that and paying to re-read a long session:

1. **When context usage passes ~55%**: stop and update this file - current state,
   what was verified and how, what is pending, anything a fresh reader would need.
   Commit it.
2. Then the session can be cleared or compacted and work continues from the files.
3. Keep findings out of chat and inside files: a bug found belongs in
   `TROUBLESHOOTING.md` (stable knowledge, with the log line, the cause and the
   commit) or here (current state, which changes).
4. Token discipline while working: search before reading (`agentgrep`), filter tool
   output instead of dumping it, read the region of a file that matters rather than
   the whole file, and prefer one batch of independent calls over several rounds.
   Command output that is not needed as evidence should never reach the context.

## What this project is

An ESP32-S3 (Super Mini) MAVLink telemetry bridge: ArduPilot flight controller →
UART → Wi-Fi station → phone hotspot → 4G → Oracle VPS relay (`<relay-ip>`) →
Mission Planner. A Rust port of the C++/Arduino DroneBridge variant in
`noyajvan/NDron`, split into a portable, host-tested core (`crates/flight-core`)
and a hardware layer (`firmware`).

## State: working, verified end to end

Last verified on hardware: **2026-10-03, evening**.

| Check | Result |
| --- | --- |
| Wi-Fi join | `wifi: 'LEO' offers WPA2Personal`, DHCP lease, `RSSI -34 dBm` |
| Relay | `relay: TCP connected to <relay-ip>:14553` |
| Telemetry to a ground station | MAVLink 2 frames received on port 14552, checksums intact |
| Full parameter download | **1129 of 1129**, while the probe re-sent `PARAM_REQUEST_LIST` every 5 s |
| Bridge send queue | no overflow warnings, no dropped frames |
| Watchdog resets | none |
| Console | `STATUS` shows real IP, RSSI, TX power, FC counters |

Tests that must stay green:

```bash
cargo test                       # 55 + 9 + 9 tests, ~10 ms, no hardware
cd firmware && build-firmware.cmd build --release   # Xtensa + ESP-IDF v5.5.5
```

Building takes ~30 s warm, and a full IDF rebuild after an `sdkconfig` change took
13 minutes - budget for it.

## Fixed today (each with its own commit)

| Symptom | Cause | Commit |
| --- | --- | --- |
| Wi-Fi never associates (`auth -> init` after 1 s) | driver's stale NVS profile shadowed our config | `62ef710` |
| Telemetry ~60% lost | `tcp_send` dropped what did not fit the socket | `bbb7888` |
| "Getting params" never finishes | FC restarts its walk on every list request, MP re-sends it | `c45d697` |
| Link hangs after ~15 s | associated with no address → silence → relay drops the drone | `589fe95` |
| Watchdog reset under load | draining 49 KiB of queue in one pass | `f8613dd` |
| Heat, current, weak link | max TX power and modem sleep on, unlike the C++ original | `f8613dd` |

## Pending / worth doing next

- **Bridge-side parameter cache.** Serve a repeated `PARAM_REQUEST_LIST` from
  parameters already seen, so a ground station gets the list instantly regardless
  of link speed. The current hold-and-replay works but still depends on the link.
- **`RADIO_STATUS` injection** (DroneBridge does this): put the Wi-Fi signal into
  the MAVLink stream so the ground station shows link quality. `Io::rssi_dbm` now
  returns a real value, so the data is there.
- **Command rate during a walk.** Individual `PARAM_REQUEST_READ` sometimes goes
  unanswered while a walk is running; worth confirming whether that is the FC
  pacing or something in the bridge.
- **Finishing state.** The FSM sits in "waiting FC init (mag/SYS_STATUS)" until a
  timeout forces it on; harmless so far, but the arming watchdogs only arm after
  that - understand what else in the original depended on it.
- **Long-run soak.** The bridge has not been left running for hours on the current
  build; a soak with counters (queue drops, resets, reconnects) would tell whether
  anything leaks.

## Tools in `firmware/scripts/`

| Tool | Purpose |
| --- | --- |
| `gcs_probe.py` | Act as a ground station against the relay: request the parameter list, report what arrives, in what order, whether frames were duplicated or corrupted. Run it on the VPS. |
| `set_fc_streams.py` | Read and write the flight controller's `SRx_*` stream rates, verify by read-back, store them. |
| `crc_extra.py` | MAVLink `CRC_EXTRA` values, self-tested against published ones. |
| `usbreset.py` | Put the chip into the ROM bootloader without reading the port (needed when the application is crash-looping). |

## Environment

- **The relay address is configuration, not source.** Set it on the device:
  `HOST=<ip>` on the console, then `SAVE`. It lives in NVS and never appears in the
  repository, because the repository is public and the relay accepts any client.
- Build: `I:\esp-rs\build-firmware.cmd build --release`; flash: `I:\esp-rs\flash.cmd COM4`.
- `C:\j` is a junction to this repository; `CARGO_TARGET_DIR=I:\t`.
- Board notes (shared with every future project on this hardware):
  `C:\Projects\_boards\ESP32-S3-SuperMini.md`.
