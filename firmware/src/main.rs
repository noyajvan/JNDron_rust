//! JNDron firmware entry point.
//!
//! This is the Rust equivalent of `firmware/src/main_DrnBrdg.cpp`: a very
//! small `setup()` + `loop()` that owns a [`Platform`] (the hardware layer)
//! and a [`flight_core::Bridge`] (the tested protocol/state logic).
//!
//! ```text
//!   ESP32-S3 ──UART0──▶ flight controller (ArduPilot)
//!        │
//!        ├── Wi-Fi STA ──▶ phone hotspot ──▶ 4G ──▶ VPS ──▶ Mission Planner
//!        └── USB Serial/JTAG console (STATUS / SSID= / PASS= / ...)
//! ```

mod consts;
mod platform;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use flight_core::{Bridge, Config, Io};
use platform::Platform;

fn main() -> anyhow::Result<()> {
    // Patch the ESP-IDF `std` shims (required before any std I/O).
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs_partition = EspDefaultNvsPartition::take()?;

    let mut platform = Platform::new(peripherals, sys_loop, nvs_partition)?;

    // `loadConfig()`: NVS values, with the same defaults as the C++ firmware.
    // Built-in credentials: a board with nothing stored (or with an explicitly
    // cleared SSID) starts with these, so there is never a mandatory setup step
    // and the board is never left without a network.
    let mut cfg = Config::default();
    cfg.apply_stored(platform.load_stored_config());
    if !cfg.has_ssid() {
        cfg.sta_ssid = consts::DEFAULT_SSID.to_string();
        cfg.sta_pass = consts::DEFAULT_PASS.to_string();
        log::info!(
            "using built-in credentials: ssid='{}' (change with SSID=/PASS=/SAVE)",
            cfg.sta_ssid
        );
    }

    // The relay address is configuration, not source: this repository is public and
    // the relay accepts any client, so the address belongs on the device.
    platform.set_relay_host(&cfg.gcs_host);
    if !cfg.has_relay() {
        log::warn!("relay address not stored: use HOST=<ip>, then SAVE");
    }

    let mut bridge = Bridge::new(cfg);
    bridge.boot(&mut platform);

    log::info!(
        "JNDron ready (relay={})",
        if bridge.cfg.has_relay() {
            format!("{}:{}", bridge.cfg.gcs_host, consts::GCS_PORT_TCP)
        } else {
            "not configured".to_string()
        }
    );

    loop {
        // 1. USB console commands (handleTerminalConfig).
        platform.pump_console(&mut bridge);
        // 2. Keep the TCP relay link alive (tcpLinkService).
        platform.service_tcp();
        // 3. Mission Planner -> FC (bridgeWiFiToFC).
        platform.pump_gcs(&mut bridge);
        // 4. FC -> Mission Planner (bridgeFCtoWiFi).
        platform.pump_fc(&mut bridge);
        // 5. Heartbeats, FSM, timeout watchdogs (loop()).
        bridge.tick(&mut platform);
        // 6. Status LED (updateLED).
        let led = bridge.led(platform.now_ms());
        platform.set_led(led);

        FreeRtos::delay_ms(1);
    }
}
