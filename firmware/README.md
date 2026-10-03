# JNDron firmware (ESP32-S3)

The Rust replacement for the original `firmware/` (C++/Arduino). All protocol
and state-machine logic lives in [`flight-core`](../crates/flight-core); this
crate is only the hardware layer plus `setup()`/`loop()`.

## Prerequisites

```powershell
cargo install espup espflash
# Minimal STD-only toolchain for the S3 (no GCC: esp-idf-sys brings its own).
espup install -t esp32s3 -s
```

ESP-IDF 5.1 itself is downloaded and built automatically by `esp-idf-sys` on
the first `cargo build`. It is large (several GB), so point it at a roomy disk
if your system drive is tight:

```powershell
$env:IDF_TOOLS_PATH   = "I:\esp-rs\espressif"
$env:CARGO_TARGET_DIR = "I:\esp-rs\target"
```

## Build

```powershell
cd firmware
cargo build --release
```

## Flash

The DevKitC-1's native USB-Serial/JTAG port does **not** answer the classic
DTR/RTS reset, so `espflash` needs the USB-JTAG reset sequence and, on this
board, no RAM stub:

```powershell
$env:ESPFLASH_PORT = "COM4"          # Espressif USB-Serial/JTAG port
cargo run --release                  # flashes + opens the monitor
```

Equivalent manual invocation:

```powershell
espflash flash --port COM4 --before usb-reset --no-stub --monitor `
  target\xtensa-esp32s3-espidf\release\jndron-firmware
```

Check the board is reachable first:

```powershell
espflash board-info --port COM4 --before usb-reset --no-stub
```

Expected: `Chip type: esp32s3 (revision v0.2)`, `Flash size: 4MB`.

## Wiring

| ESP32-S3 | Connects to |
|---|---|
| GPIO43 (TX) | flight-controller RX |
| GPIO44 (RX) | flight-controller TX |
| GPIO48 | on-board WS2812 status LED |
| USB (native) | host, for flashing + the `STATUS`/`SSID=`/… console |

## Console

Over the USB port, 115200:

```text
STATUS | SSID=name | PASS=pass | BAUD= | SYSID= | WIFI OFF | WIFI ON | RELAY | DISARM | SAVE
```

## Protocol

MAVLink **2** only. The flight controller must be configured for MAVLink 2 on
the telemetry port wired to GPIO43/44 (`SERIALx_PROTOCOL = 2`, or `Auto`).
