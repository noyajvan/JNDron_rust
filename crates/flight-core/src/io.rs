//! Hardware abstraction.
//!
//! `flight-core` never touches a peripheral directly; the ESP32 firmware
//! implements [`Io`] with real UART/Wi-Fi/NeoPixel drivers, while the tests use
//! a mock that records everything.
//!
//! The transport split mirrors the original firmware exactly: telemetry is
//! forwarded over the outgoing **TCP** relay when it is up, and otherwise
//! falls back to **UDP** (sent twice, because 4G drops UDP).

/// Everything the bridge needs from the outside world.
pub trait Io {
    /// Milliseconds since boot (wrapping, exactly like Arduino `millis()`).
    fn now_ms(&self) -> u32;

    /// Write raw bytes to the flight-controller UART.
    fn fc_write(&mut self, data: &[u8]);

    /// Try to write to the outgoing TCP relay. Returns `false` when the link
    /// broke (the caller then drops it), mirroring `tcpLink.write() == 0`.
    fn tcp_send(&mut self, data: &[u8]) -> bool {
        let _ = data;
        true
    }

    /// Send a UDP datagram to the GCS. Returns `false` when the lwIP stack
    /// rejected it (`udp.endPacket() == 0`), which triggers the send backoff.
    fn udp_send(&mut self, data: &[u8]) -> bool {
        let _ = data;
        true
    }

    /// Emit a human-readable line on the USB console (no trailing newline).
    fn log(&mut self, line: &str);

    /// Point the relay link at `host` (empty disables it). Called at startup with
    /// the stored configuration and whenever the operator changes it.
    fn set_relay_host(&mut self, _host: &str) {}

    /// Persist the configuration (NVS `dbridge` namespace).
    fn save_config(&mut self, cfg: &crate::Config);

    /// Reboot the MCU.
    fn restart(&mut self);

    /// Current Wi-Fi station status (connected / not). Used by the LED logic
    /// and the Wi-Fi watchdog.
    fn wifi_connected(&self) -> bool;

    /// True while the *radio* itself is dead (`WL_NO_SHIELD`, status 255 in
    /// the original firmware).
    fn wifi_radio_dead(&self) -> bool {
        false
    }

    /// True when the outgoing TCP relay link is established.
    fn tcp_connected(&self) -> bool {
        false
    }

    /// (Re)start the Wi-Fi station and the UDP/TCP sockets.
    fn wifi_activate(&mut self, cfg: &crate::Config);
    /// Power the Wi-Fi radio down.
    fn wifi_deactivate(&mut self);
    /// Full radio restart.
    fn wifi_full_restart(&mut self, cfg: &crate::Config);
    /// Lightweight reconnect attempt.
    fn wifi_retry_connect(&mut self);

    // --- status reporting (used by the `STATUS` console command) ---

    /// MCU temperature in degrees Celsius (`temperatureRead()`).
    fn temperature_c(&self) -> f32 {
        0.0
    }
    /// Current CPU clock in MHz (`getCpuFrequencyMhz()`).
    fn cpu_mhz(&self) -> u32 {
        80
    }
    /// Wi-Fi RSSI in dBm.
    fn rssi_dbm(&self) -> i32 {
        0
    }
    /// Wi-Fi TX power in dBm.
    fn tx_power_dbm(&self) -> i32 {
        11
    }
    /// Station IP address (`WiFi.localIP()`).
    fn local_ip(&self) -> String {
        "0.0.0.0".to_string()
    }
}

/// A no-op `Io` useful as a default.
#[derive(Debug, Default)]
pub struct NullIo {
    pub now: u32,
}

impl Io for NullIo {
    fn now_ms(&self) -> u32 {
        self.now
    }
    fn fc_write(&mut self, _data: &[u8]) {}
    fn log(&mut self, _line: &str) {}
    fn save_config(&mut self, _cfg: &crate::Config) {}
    fn restart(&mut self) {}
    fn wifi_connected(&self) -> bool {
        false
    }
    fn wifi_activate(&mut self, _cfg: &crate::Config) {}
    fn wifi_deactivate(&mut self) {}
    fn wifi_full_restart(&mut self, _cfg: &crate::Config) {}
    fn wifi_retry_connect(&mut self) {}
}

/// A recording `Io` for tests: captures FC writes, TCP/UDP writes and logs.
#[derive(Debug, Default)]
pub struct MockIo {
    pub now: u32,
    /// Address reported by `local_ip`. Empty means "no address yet", the state a
    /// station is in after a reconnect whose DHCP never completed.
    pub ip: String,
    /// Last value passed to `set_relay_host`.
    pub relay_host: String,
    pub fc_out: Vec<u8>,
    pub tcp_out: Vec<u8>,
    pub udp_out: Vec<u8>,
    pub logs: Vec<String>,
    pub restarts: u32,
    pub saved: Option<crate::Config>,
    pub connected: bool,
    pub radio_dead: bool,
    pub tcp: bool,
    /// When true, `udp_send` reports failure (simulated congested link).
    pub udp_fails: bool,
    pub activated: u32,
    pub deactivated: u32,
    pub full_restarts: u32,
    pub retries: u32,
}

impl MockIo {
    pub fn new() -> Self {
        MockIo::default()
    }

    /// Take everything written to the network since the last call.
    pub fn take_net(&mut self) -> Vec<u8> {
        let mut v = std::mem::take(&mut self.tcp_out);
        v.extend_from_slice(&self.udp_out);
        self.udp_out.clear();
        v
    }

    pub fn take_fc(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.fc_out)
    }
}

impl Io for MockIo {
    fn now_ms(&self) -> u32 {
        self.now
    }
    fn fc_write(&mut self, data: &[u8]) {
        self.fc_out.extend_from_slice(data);
    }
    fn tcp_send(&mut self, data: &[u8]) -> bool {
        if !self.tcp {
            return false;
        }
        self.tcp_out.extend_from_slice(data);
        true
    }
    fn udp_send(&mut self, data: &[u8]) -> bool {
        if self.udp_fails {
            return false;
        }
        self.udp_out.extend_from_slice(data);
        true
    }
    fn log(&mut self, line: &str) {
        self.logs.push(line.to_string());
    }
    fn save_config(&mut self, cfg: &crate::Config) {
        self.saved = Some(cfg.clone());
    }
    fn set_relay_host(&mut self, host: &str) {
        self.relay_host = host.to_string();
    }
    fn restart(&mut self) {
        self.restarts += 1;
    }
    fn wifi_connected(&self) -> bool {
        self.connected
    }
    fn wifi_radio_dead(&self) -> bool {
        self.radio_dead
    }
    fn tcp_connected(&self) -> bool {
        self.tcp
    }
    fn wifi_activate(&mut self, _cfg: &crate::Config) {
        self.activated += 1;
    }
    fn wifi_deactivate(&mut self) {
        self.deactivated += 1;
    }
    fn wifi_full_restart(&mut self, _cfg: &crate::Config) {
        self.full_restarts += 1;
    }
    fn wifi_retry_connect(&mut self) {
        self.retries += 1;
    }
}
