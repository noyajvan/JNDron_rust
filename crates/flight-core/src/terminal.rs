//! USB-console command handling, ported from `terminal.cpp`.
//!
//! The original `handleTerminalConfig()` polled `Serial` on every loop
//! iteration. Here the firmware feeds one character at a time into
//! [`Bridge::terminal_char`], which reproduces the echo, the prompt and the
//! exact command set:
//!
//! ```text
//! CMD: STATUS | SSID=name | PASS=pass | BAUD= | SYSID= |
//!      WIFI OFF | WIFI ON | RELAY | DISARM | SAVE
//! ```

use crate::bridge::Bridge;
use crate::config;
use crate::io::Io;

fn ieq(a: &str, b: &str) -> bool {
    a.eq_ignore_ascii_case(b)
}

impl Bridge {
    /// Feed one byte from the USB console.
    pub fn terminal_char(&mut self, io: &mut dyn Io, c: char) {
        if c == '\n' || c == '\r' {
            io.log("");
            if !self.input_buffer.is_empty() {
                let cmd = self.input_buffer.trim().to_string();
                self.run_command(io, &cmd);
                self.input_buffer.clear();
            }
            io.log("> ");
        } else {
            io.log(&c.to_string());
            self.input_buffer.push(c);
        }
    }

    /// Execute one already-parsed console command.
    pub fn run_command(&mut self, io: &mut dyn Io, cmd: &str) {
        if ieq(cmd, "STATUS") {
            self.print_status(io);
        } else if let Some(v) = cmd.strip_prefix("SSID=") {
            let val = v.trim();
            if val.is_empty() {
                self.cfg.sta_ssid.clear();
                self.cfg.sta_pass.clear();
                io.log(">> SSID cleared, WiFi OFF");
            } else {
                self.cfg.sta_ssid = val.to_string();
                io.log(&format!(">> SSID = {}", self.cfg.sta_ssid));
            }
        } else if let Some(v) = cmd.strip_prefix("PASS=") {
            self.cfg.sta_pass = v.to_string();
            io.log(&format!(">> PASS = {}", self.cfg.sta_pass));
        } else if let Some(v) = cmd.strip_prefix("BAUD=") {
            let b: u32 = v.trim().parse().unwrap_or(0);
            if crate::Config::is_valid_baud(b) {
                self.cfg.baud = b;
                io.log(&format!(">> BAUD = {}", self.cfg.baud));
            } else {
                io.log(">> Invalid baud");
            }
        } else if let Some(v) = cmd.strip_prefix("HOST=") {
            let val = v.trim();
            if val.is_empty() {
                self.cfg.gcs_host.clear();
                io.set_relay_host("");
                io.log(">> Relay address cleared, relay off");
            } else if val.parse::<std::net::Ipv4Addr>().is_ok() {
                self.cfg.gcs_host = val.to_string();
                io.set_relay_host(val);
                io.log(&format!(">> Relay host = {}", self.cfg.gcs_host));
            } else {
                io.log(">> Invalid address, expected HOST=1.2.3.4");
            }
        } else if let Some(v) = cmd.strip_prefix("TOKEN=") {
            let val = v.trim();
            self.cfg.relay_token = val.to_string();
            io.set_relay_token(val);
            // Never echo a secret back; say only whether one is set.
            io.log(if val.is_empty() {
                ">> Relay token cleared"
            } else {
                ">> Relay token set"
            });
        } else if let Some(v) = cmd.strip_prefix("SYSID=") {
            let sid: u32 = v.trim().parse().unwrap_or(0);
            if (1..=255).contains(&sid) {
                self.cfg.sys_id = sid as u8;
                io.log(&format!(">> SYSID = {}", self.cfg.sys_id));
            }
        } else if ieq(cmd, "WIFI OFF") {
            self.wifi_deactivate(io);
            io.log(">> WiFi OFF");
        } else if ieq(cmd, "WIFI ON") {
            self.wifi_activate(io);
            io.log(">> WiFi ON");
        } else if ieq(cmd, "RELAY") {
            self.send_set_relay(io);
            io.log(">> RELAY sent");
        } else if ieq(cmd, "DISARM") {
            self.send_force_disarm(io);
            io.log(">> DISARM sent");
        } else if ieq(cmd, "SAVE") {
            config::persist(io, &self.cfg);
            io.log(">> Saved. Rebooting...");
            io.restart();
        } else {
            io.log(
                "CMD: STATUS | SSID=name | PASS=pass | BAUD= | HOST=ip | TOKEN=secret | SYSID= | \
                 WIFI OFF | WIFI ON | RELAY | DISARM | SAVE",
            );
        }
    }

    fn print_status(&self, io: &mut dyn Io) {
        io.log("--- STATUS ---");
        io.log(&format!(
            "State: {}  mdfly: {}",
            self.state.as_u8(),
            self.mdfly
        ));
        io.log(&format!(
            "WiFi: {}  IP: {}",
            if io.wifi_connected() { "OK" } else { "NO" },
            io.local_ip()
        ));
        io.log(&format!(
            "hasServer: {}  hb: {}  armed: {}  mode: {}",
            self.has_server as u8,
            self.heartbeat_received as u8,
            self.is_armed as u8,
            self.current_custom_mode
        ));
        io.log(&format!(
            "EKF: 0x{:04X}  mag: {:.3}  mission: {}/{}",
            self.ekf_flags, self.mag_test_ratio, self.mission_loaded as u8, self.mission_count
        ));
        io.log(&format!(
            "FC: {} bytes  {} msgs",
            self.fc_bytes, self.fc_msgs
        ));
        io.log(&format!(
            "Temp: {:.1} C  CPU: {} MHz  RSSI: {} dBm  TX: {} dBm",
            io.temperature_c(),
            io.cpu_mhz(),
            io.rssi_dbm(),
            io.tx_power_dbm()
        ));
        io.log(&format!(
            "WiFi cfg: {} / {}",
            self.cfg.sta_ssid,
            if self.cfg.sta_pass.is_empty() {
                "no pass"
            } else {
                "pass set"
            }
        ));
        io.log(&format!(
            "Relay: {}  token: {}",
            if self.cfg.has_relay() {
                self.cfg.gcs_host.clone()
            } else {
                "not set - HOST=<ip> then SAVE".to_string()
            },
            // Never print the secret itself.
            if self.cfg.relay_token.is_empty() {
                "none"
            } else {
                "set"
            }
        ));
    }
}
