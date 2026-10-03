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
mod provisioning;

use std::net::SocketAddr;

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

    let mut platform = Platform::new(peripherals, sys_loop, nvs_partition, consts::GCS_IP)?;

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

    let mut bridge = Bridge::new(cfg);
    bridge.boot(&mut platform);

    // BLE provisioning is parked (see `consts::ENABLE_BLE_PROVISIONING`); with
    // the built-in credentials above there is always an SSID, so this only runs
    // when explicitly re-enabled.
    if consts::ENABLE_BLE_PROVISIONING && !bridge.cfg.has_ssid() {
        if let Err(e) = provisioning::start(consts::PROV_DEVICE_NAME) {
            log::error!("BLE provisioning did not start: {e:?}");
        }
    }

    log::info!(
        "JNDron ready (gcs={})",
        SocketAddr::from((consts::GCS_IP, consts::GCS_PORT_TCP))
    );

    // Set when BLE provisioning delivered credentials: the BLE service is kept
    // alive for a while so the phone app can query the Wi-Fi state and report
    // success, then released.
    let mut prov_deadline: Option<u32> = None;

    loop {
        // Wi-Fi credentials handed over by the phone app: store them and go.
        if let Some((ssid, pass)) = provisioning::take_credentials() {
            log::info!("provisioned over BLE: ssid='{ssid}'");
            bridge.cfg.sta_ssid = ssid;
            bridge.cfg.sta_pass = pass;
            let cfg = bridge.cfg.clone();
            flight_core::config::persist(&mut platform, &cfg);
            bridge.wifi_activate(&mut platform);
            prov_deadline = Some(platform.now_ms().wrapping_add(30_000));
        }

        if let Some(deadline) = prov_deadline {
            // Stop once Wi-Fi is up, or after the grace period, whichever is
            // first: a failed password should not keep the BLE service alive
            // forever, and a successful one should not wait for the timeout.
            let connected = platform.wifi_connected();
            let expired = platform.now_ms().wrapping_sub(deadline) < 0x8000_0000;
            if connected || expired {
                log::info!("BLE provisioning finished (connected={connected}), releasing BLE");
                provisioning::stop();
                prov_deadline = None;
            }
        }

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
