//! Persistent configuration, ported from `config.cpp` (ESP32 `Preferences`/NVS).

use crate::consts;
use crate::io::Io;

/// Bridge configuration stored in the NVS namespace `dbridge`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Config {
    pub sta_ssid: String,
    pub sta_pass: String,
    pub baud: u32,
    /// MAVLink system id of the bridge itself.
    pub sys_id: u8,
    /// Relay endpoint (the VPS that forwards telemetry to the ground station), as
    /// an IPv4 address in text form. Empty means "not configured", and the bridge
    /// then stays off the network entirely.
    ///
    /// Deliberately *not* a compile-time constant: this repository is public, and a
    /// relay that accepts any client means publishing its address invites strangers
    /// to command the aircraft. The address belongs on the device, in NVS.
    pub gcs_host: String,
    /// Shared secret the relay expects from a drone before it accepts telemetry.
    /// Empty means the relay requires none. Also device configuration, for the same
    /// reason as `gcs_host`.
    pub relay_token: String,
}

impl Default for Config {
    fn default() -> Self {
        // Mirrors `Config cfg = { "", "", 921600, 1 };`
        Config {
            sta_ssid: String::new(),
            sta_pass: String::new(),
            baud: 921_600,
            sys_id: 1,
            gcs_host: String::new(),
            relay_token: String::new(),
        }
    }
}

impl Config {
    /// SSID as a fixed `[u8; 32]` buffer, matching the C++ `char[32]` field.
    pub fn ssid_bytes(&self) -> [u8; 32] {
        let mut out = [0u8; 32];
        let b = self.sta_ssid.as_bytes();
        let n = b.len().min(31);
        out[..n].copy_from_slice(&b[..n]);
        out
    }

    /// Password as a fixed `[u8; 64]` buffer.
    pub fn pass_bytes(&self) -> [u8; 64] {
        let mut out = [0u8; 64];
        let b = self.sta_pass.as_bytes();
        let n = b.len().min(63);
        out[..n].copy_from_slice(&b[..n]);
        out
    }

    pub fn has_ssid(&self) -> bool {
        !self.sta_ssid.is_empty()
    }

    /// Is a relay endpoint configured?
    pub fn has_relay(&self) -> bool {
        !self.gcs_host.is_empty()
    }

    /// Validate a baud rate, matching the C++ `BAUD=` command range check.
    pub fn is_valid_baud(b: u32) -> bool {
        (9_600..=921_600).contains(&b)
    }
}

/// Values loaded from NVS. The firmware maps this onto its `Preferences` API.
#[derive(Debug, Clone, Default)]
pub struct StoredConfig {
    pub ssid: Option<String>,
    pub pass: Option<String>,
    pub baud: Option<u32>,
    pub sys_id: Option<u8>,
    pub gcs_host: Option<String>,
    pub relay_token: Option<String>,
}

impl Config {
    /// Apply values read from persistent storage, reproducing the defaults in
    /// `loadConfig()`. Note the original firmware hard-codes `sys_id = 1`
    /// after reading it, which we reproduce faithfully.
    pub fn apply_stored(&mut self, stored: StoredConfig) {
        if let Some(s) = stored.ssid {
            self.sta_ssid = s;
        }
        if let Some(p) = stored.pass {
            self.sta_pass = p;
        }
        if let Some(h) = stored.gcs_host {
            self.gcs_host = h;
        }
        if let Some(t) = stored.relay_token {
            self.relay_token = t;
        }
        self.baud = stored.baud.unwrap_or(921_600);
        // `cfg.sys_id = p.getUInt("sys_id", 1); cfg.sys_id = 1;`
        let _ = stored.sys_id;
        self.sys_id = 1;
    }
}

/// Convenience used by the terminal `SAVE` command.
pub fn persist(io: &mut dyn Io, cfg: &Config) {
    io.save_config(cfg);
}

/// Sanity-check the constants the config layer depends on.
#[allow(dead_code)]
const _: () = {
    assert!(consts::UDP_PORT != 0);
};
