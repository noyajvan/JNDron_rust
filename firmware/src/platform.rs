//! Hardware layer: an [`Io`] implementation for the ESP32-S3.
//!
//! This file is the *only* place that knows about UART, Wi-Fi, sockets, NVS
//! and the NeoPixel. Everything above it (`flight-core`) is portable and
//! unit-tested on the host.
//!
//! ## Toolchain note
//! Building this crate requires the Xtensa Rust toolchain and ESP-IDF:
//!
//! ```text
//! cargo install espup && espup install
//! cargo install espflash
//! cd firmware && cargo run --release
//! ```
//!
//! The crate versions pinned in `Cargo.toml` (`esp-idf-svc 0.49`,
//! `esp-idf-hal 0.44`, ESP-IDF v5.1) match the Arduino core that the original
//! `platformio.ini` used. If you bump ESP-IDF, some HAL constructors below
//! (`UartDriver::new`, the RMT NeoPixel) may need signature adjustments - this
//! is the layer where API drift shows up, by design.

use std::net::{Ipv4Addr, SocketAddr, TcpStream, UdpSocket};
use std::collections::VecDeque;
use std::time::Duration;

use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::uart::{UartConfig, UartDriver};
use esp_idf_hal::units::Hertz;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use ws2812_esp32_rmt_driver::Ws2812Esp32RmtDriver;

use flight_core::config::StoredConfig;
use flight_core::io::Io;
use flight_core::Config;

use crate::consts::{GCS_PORT_TCP, GCS_PORT_UDP, LED_PIN, LED_RMT_CHANNEL, NVS_NAMESPACE};

/// The board.
pub struct Platform {
    uart: UartDriver<'static>,
    led: Ws2812Esp32RmtDriver,
    nvs: EspNvs<NvsDefault>,
    wifi: BlockingWifi<EspWifi<'static>>,
    wifi_on: bool,
    /// Security mode to ask for, refreshed from a scan before joining.
    auth_method: AuthMethod,

    udp: UdpSocket,
    tcp: Option<TcpStream>,
    /// Relay address for the outgoing TCP link, once configured.
    gcs_tcp: Option<SocketAddr>,
    /// Relay address for the UDP fallback (same host, UDP port).
    gcs_udp: Option<SocketAddr>,
    tcp_last_try_ms: u32,
    tcp_was_up: bool,
    /// Rate limit for the "TCP connect failed" log line.
    tcp_last_log_ms: u32,
    /// Bytes read from the flight controller but not yet accepted by the relay
    /// socket, in order. Without this the bridge dropped whatever did not fit
    /// the socket buffer, which measured at roughly 60% of the telemetry - and
    /// a parameter download needs the whole list, not 40% of it.
    tx_queue: VecDeque<u8>,
    /// Total bytes dropped because the queue overflowed (should stay 0).
    tx_dropped: u64,
    /// Rate limit for the "send queue full" log line.
    tx_last_drop_log_ms: u32,
}

impl Platform {
    pub fn new(
        peripherals: esp_idf_hal::peripherals::Peripherals,
        sys_loop: esp_idf_svc::eventloop::EspSystemEventLoop,
        nvs_partition: EspDefaultNvsPartition,
    ) -> anyhow::Result<Self> {
        // --- Flight-controller UART (UART0, TX=GPIO43, RX=GPIO44) ---
        let uart = UartDriver::new(
            peripherals.uart0,
            peripherals.pins.gpio43,
            peripherals.pins.gpio44,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &UartConfig::new()
                .baudrate(Hertz(921_600))
                .rx_fifo_size(flight_core::consts::FC_RX_BUF)
                .tx_fifo_size(flight_core::consts::FC_TX_BUF),
        )?;

        // --- WS2812 status LED on GPIO48 ---
        // `Ws2812Esp32RmtDriver` owns RMT channel 0 and drives the pixel through
        // the ESP-IDF (legacy) RMT driver, so it takes a channel number and a
        // raw GPIO number rather than HAL pin singletons.
        let led = Ws2812Esp32RmtDriver::new(LED_RMT_CHANNEL, LED_PIN as u32)
            .map_err(|e| anyhow::anyhow!("ws2812 rmt init: {e:?}"))?;

        // --- NVS namespace for the configuration ---
        let nvs = EspNvs::new(nvs_partition.clone(), NVS_NAMESPACE, true)?;

        // --- Wi-Fi station ---
        let wifi = BlockingWifi::wrap(
            EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs_partition))?,
            sys_loop,
        )?;

        // The driver keeps its *own* copy of the station config in NVS
        // (`WIFI_STORAGE_FLASH` is the default), including the BSSID it last
        // associated with. That stale profile survives reflashes, credential
        // changes and reboots alike, so once the hotspot was recreated with a
        // new BSSID every association timed out on the auth step (`auth -> init`
        // after exactly one second).
        //
        // This is *only* about the driver's private copy. The network that is
        // actually joined is the one configured in `wifi_activate`, i.e. the
        // SSID and password held in our own NVS namespace (default `LEO` /
        // `88888888` from `consts`, changeable with `SSID=` / `PASS=` / `SAVE`).
        // There is no network selection by scanning anywhere in the firmware.
        unsafe {
            let restored = esp_idf_svc::sys::esp_wifi_restore();
            let storage = esp_idf_svc::sys::esp_wifi_set_storage(
                esp_idf_svc::sys::wifi_storage_t_WIFI_STORAGE_RAM,
            );
            log::info!("wifi: cleared stored profile (restore={restored}, storage={storage})");
        }

        // --- USB Serial/JTAG console (input) ---
        // The console output goes through `Io::log`; input needs the driver so
        // that the main loop can poll it without blocking. Installing it also
        // switches the console VFS to the driver's ring buffers.
        let mut console_cfg = esp_idf_svc::sys::usb_serial_jtag_driver_config_t {
            tx_buffer_size: 256,
            rx_buffer_size: 1024,
        };
        unsafe {
            esp_idf_svc::sys::usb_serial_jtag_driver_install(&mut console_cfg);
        }

        // --- Sockets ---
        let udp = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, GCS_PORT_UDP))?;
        udp.set_nonblocking(true)?;

        Ok(Platform {
            uart,
            led,
            nvs,
            wifi,
            wifi_on: false,
            auth_method: AuthMethod::WPA2Personal,
            udp,
            tcp: None,
            // Set by `set_relay_host` once the stored configuration is known.
            gcs_tcp: None,
            gcs_udp: None,
            tcp_last_try_ms: 0,
            tcp_was_up: false,
            tcp_last_log_ms: 0,
            tx_queue: VecDeque::new(),
            tx_dropped: 0,
            tx_last_drop_log_ms: 0,
        })
    }

    // -----------------------------------------------------------------
    // Configuration (NVS, namespace "dbridge")
    // -----------------------------------------------------------------

    fn nvs_get_str(&self, key: &str) -> Option<String> {
        let mut buf = [0u8; 64];
        self.nvs.get_str(key, &mut buf).ok().flatten().map(|s| s.to_string())
    }

    fn nvs_get_u32(&self, key: &str) -> Option<u32> {
        self.nvs.get_u32(key).ok().flatten()
    }

    pub fn load_stored_config(&self) -> StoredConfig {
        StoredConfig {
            ssid: self.nvs_get_str("ssid"),
            pass: self.nvs_get_str("pass"),
            baud: self.nvs_get_u32("baud"),
            sys_id: self.nvs_get_u32("sys_id").map(|v| v as u8),
            gcs_host: self.nvs_get_str("gcs_host"),
        }
    }

    // -----------------------------------------------------------------
    // Console
    // -----------------------------------------------------------------

    /// Read whatever is available on the USB console and hand it to the bridge.
    pub fn pump_console(&mut self, bridge: &mut flight_core::Bridge) {
        let mut buf = [0u8; 64];
        // A zero-length timeout makes this a non-blocking poll.
        if let Ok(n) = self.console_read(&mut buf) {
            for &b in &buf[..n] {
                if b.is_ascii() {
                    bridge.terminal_char(self, b as char);
                }
            }
        }
    }

    fn console_read(&mut self, buf: &mut [u8]) -> anyhow::Result<usize> {
        // The console is the USB Serial/JTAG port configured in
        // `sdkconfig.defaults`. Reading it through the driver with a zero tick
        // timeout keeps the 1 ms main loop non-blocking; `pump_console` above
        // was previously a stub, which made the whole command set (SSID=,
        // PASS=, SAVE, ...) unusable on real hardware.
        let n = unsafe {
            esp_idf_svc::sys::usb_serial_jtag_read_bytes(
                buf.as_mut_ptr() as *mut core::ffi::c_void,
                buf.len() as u32,
                0,
            )
        };
        if n <= 0 {
            Ok(0)
        } else {
            Ok(n as usize)
        }
    }

    // -----------------------------------------------------------------
    // Transport
    // -----------------------------------------------------------------

    /// Maintain the outgoing TCP relay link to the VPS (mirrors
    /// `tcpLinkService()`).
    pub fn service_tcp(&mut self) {
        if self.tcp.is_none() && !self.tx_queue.is_empty() {
            // The link went away (dropped, failed or Wi-Fi down): telemetry queued
            // for it is stale, and a fresh socket must not start with old frames.
            self.tx_queue.clear();
        }
        if !self.wifi_on || !self.wifi_connected() {
            if self.tcp_was_up {
                self.tcp_was_up = false;
                self.tcp = None;
            }
            return;
        }
        if self.tcp.is_some() {
            self.tcp_was_up = true;
            self.flush_tx();
            return;
        }
        let Some(gcs_tcp) = self.gcs_tcp else {
            // No relay configured: stand still instead of dialling nowhere, and say
            // so once in a while so the reason is visible on the console.
            let now = self.now_ms();
            if now.wrapping_sub(self.tcp_last_log_ms) >= 30_000 {
                self.tcp_last_log_ms = now;
                log::warn!("relay: no endpoint configured (use HOST=<ip>, then SAVE)");
            }
            return;
        };
        self.tcp_was_up = false;
        let now = self.now_ms();
        if now.wrapping_sub(self.tcp_last_try_ms) < 3_000 {
            return;
        }
        self.tcp_last_try_ms = now;
        match TcpStream::connect_timeout(&gcs_tcp, Duration::from_millis(2_000)) {
            Ok(s) => {
                let _ = s.set_nodelay(true);
                let _ = s.set_nonblocking(true);
                self.tcp = Some(s);
                self.tcp_was_up = true;
                log::info!("relay: TCP connected to {gcs_tcp}");
            }
            Err(e) => {
                self.tcp = None;
                // Rate limited, otherwise a dead VPS floods the console.
                if now.wrapping_sub(self.tcp_last_log_ms) >= 30_000 {
                    self.tcp_last_log_ms = now;
                    log::warn!("relay: TCP connect to {gcs_tcp} failed: {e:?}");
                }
            }
        }
    }

    /// Read from the GCS and push it into the bridge (mirrors
    /// `bridgeWiFiToFC()`).
    pub fn pump_gcs(&mut self, bridge: &mut flight_core::Bridge) {
        if !self.wifi_on {
            return;
        }
        let mut buf = [0u8; flight_core::consts::BRIDGE_BUF_SIZE];

        for _ in 0..16 {
            let n = match self.tcp.as_mut() {
                Some(s) => match std::io::Read::read(s, &mut buf) {
                    Ok(0) => break,
                    Ok(n) => n,
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => {
                        self.tcp = None;
                        break;
                    }
                },
                None => break,
            };
            bridge.feed_gcs_bytes(self, &buf[..n]);
        }

        for _ in 0..16 {
            match self.udp.recv_from(&mut buf) {
                Ok((n, _from)) if n > 0 => bridge.feed_gcs_bytes(self, &buf[..n]),
                _ => break,
            }
        }
    }

    /// Read the flight controller's UART and push it into the bridge (mirrors
    /// `bridgeFCtoWiFi()`).
    pub fn pump_fc(&mut self, bridge: &mut flight_core::Bridge) {
        // Deliberately *not* pausing the FC link when the queue is deep. That was
        // tried and it is worse than dropping frames: it stops outgoing traffic
        // altogether, the VPS relay drops a drone that is silent for 15 seconds, and
        // the ground station sees the link hang. Losing telemetry is something
        // MAVLink recovers from; going mute is not. The queue absorbs bursts, and if
        // it does fill, `queue_tx` counts what it had to drop.
        let mut buf = [0u8; flight_core::consts::BRIDGE_BUF_SIZE];
        let mut total = 0usize;
        // Cap the work per loop iteration, like the C++ `for (pass < 16)`.
        while total < 16 * flight_core::consts::BRIDGE_BUF_SIZE {
            match self.uart.read(&mut buf, 0) {
                Ok(0) => break,
                Ok(n) => {
                    bridge.feed_fc_bytes(self, &buf[..n]);
                    total += n;
                }
                Err(_) => break,
            }
        }
    }

    /// Append to the pending send queue, counting anything that does not fit.
    ///
    /// The relay socket is non-blocking, so a burst (a parameter list is about
    /// 42 KiB) can arrive faster than the socket accepts it. Those bytes are
    /// kept, not thrown away: measurement showed dropping them cost roughly 60%
    /// of the telemetry and made Mission Planner's parameter download
    /// impossible, since it needs the whole list.
    fn queue_tx(&mut self, data: &[u8]) {
        let room = crate::consts::TCP_TX_QUEUE.saturating_sub(self.tx_queue.len());
        let take = room.min(data.len());
        self.tx_queue.extend(data[..take].iter().copied());
        let dropped = data.len() - take;
        if dropped > 0 {
            self.tx_dropped += dropped as u64;
            let now = self.now_ms();
            if now.wrapping_sub(self.tx_last_drop_log_ms) >= 10_000 {
                self.tx_last_drop_log_ms = now;
                log::warn!(
                    "relay: send queue full, dropped {dropped} bytes ({} in total)",
                    self.tx_dropped
                );
            }
        }
    }

    /// Push as much of the pending queue into the relay socket as it will take.
    fn flush_tx(&mut self) {
        let mut broken = false;
        // Cap the work per call. The main loop has to keep servicing the UART, the
        // console and the Wi-Fi driver; draining an arbitrarily deep queue in one go
        // is what invited an interrupt watchdog reset ("Interrupt wdt timeout on
        // CPU1") when 49 KiB had piled up against a weak 4G link.
        let mut budget: usize = 8 * 1024;
        while !self.tx_queue.is_empty() && budget > 0 {
            if self.tx_queue.as_slices().0.is_empty() {
                // Wrapped ring: make it contiguous before writing from the front.
                self.tx_queue.make_contiguous();
            }
            let (front, _) = self.tx_queue.as_slices();
            let sent = match self.tcp.as_mut() {
                Some(s) => match std::io::Write::write(s, front) {
                    Ok(n) => n,
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => {
                        broken = true;
                        break;
                    }
                },
                None => break,
            };
            if sent == 0 {
                break;
            }
            budget = budget.saturating_sub(sent);
            self.tx_queue.drain(..sent);
        }
        if broken {
            log::warn!("relay: TCP write failed while flushing, dropping the link");
            self.tcp = None;
            self.tx_queue.clear();
        }
    }

    /// Push the LED colour computed by the core.
    pub fn set_led(&mut self, out: flight_core::led::LedOut) {
        let r = ((out.color >> 16) & 0xFF) as u8;
        let g = ((out.color >> 8) & 0xFF) as u8;
        let b = (out.color & 0xFF) as u8;
        // Scale by brightness (0..=255), exactly like setBrightness().
        let scale = |c: u8| ((c as u16 * out.brightness as u16) / 255) as u8;
        // The on-board WS2812 on the DevKitC-1 is wired G-R-B.
        let _ = self.led.write(&[scale(g), scale(r), scale(b)]);
    }

    /// Ask for the security mode the access point actually offers.
    ///
    /// Asking for WPA2 against a hotspot that offers only WPA3, or WPA2/WPA3
    /// transition, fails the four-way handshake even with the correct password -
    /// measured on this board as `wifi:state: init -> auth -> assoc` followed by
    /// `assoc -> init (0x400)` once a second, which looks exactly like a bad key.
    /// The scan costs a couple of seconds and runs once per activation (not on the
    /// 30 s retries), and the result is cached for the next attempt.
    fn refresh_auth_method(&mut self, ssid: &str) {
        match self.wifi.scan() {
            Ok(aps) => {
                let offered = aps
                    .iter()
                    .find(|ap| ap.ssid.as_str() == ssid)
                    .and_then(|ap| ap.auth_method);
                match offered {
                    Some(method) if method != self.auth_method => {
                        log::info!("wifi: '{ssid}' offers {method:?}, asking for that");
                        self.auth_method = method;
                    }
                    Some(method) => log::info!("wifi: '{ssid}' offers {method:?}"),
                    None => log::warn!(
                        "wifi: '{ssid}' not in the scan, keeping {:?}",
                        self.auth_method
                    ),
                }
            }
            Err(e) => log::warn!("wifi: scan failed, keeping {:?} ({e:?})", self.auth_method),
        }
    }
}

impl Io for Platform {
    fn now_ms(&self) -> u32 {
        // ESP-IDF's esp_timer gives microseconds since boot.
        (unsafe { esp_idf_svc::sys::esp_timer_get_time() } as u64 / 1000) as u32
    }

    fn fc_write(&mut self, data: &[u8]) {
        let _ = self.uart.write(data);
    }

    fn tcp_send(&mut self, data: &[u8]) -> bool {
        if self.tcp.is_none() {
            return false;
        }

        // Anything already queued must go first, otherwise frames would be sent
        // out of order. MAVLink tolerates a gap, not reordering.
        if !self.tx_queue.is_empty() {
            self.queue_tx(data);
            return true;
        }

        let mut written = 0usize;
        let mut broken = false;
        if let Some(s) = self.tcp.as_mut() {
            match std::io::Write::write(s, data) {
                Ok(n) => written = n,
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => written = 0,
                Err(_) => broken = true,
            }
        }

        if broken {
            log::warn!("relay: TCP write failed, dropping the link");
            self.tcp = None;
            self.tx_queue.clear();
            return false;
        }
        if written < data.len() {
            self.queue_tx(&data[written..]);
        }
        true
    }

    fn udp_send(&mut self, data: &[u8]) -> bool {
        match self.gcs_udp {
            Some(addr) => self.udp.send_to(data, addr).is_ok(),
            None => false,
        }
    }

    fn log(&mut self, line: &str) {
        // The console is meant to echo immediately (the original firmware used
        // `Serial.print`, which is unbuffered). Rust's stdout is line buffered
        // and `log` is called with fragments that carry no newline at all.
        use std::io::Write;
        let stdout = std::io::stdout();
        let mut out = stdout.lock();
        let _ = out.write_all(line.as_bytes());
        let _ = out.flush();
        // `flush` only empties Rust's own buffer. On the USB Serial/JTAG
        // console the bytes then sit in a hardware FIFO which is drained by the
        // VFS `fsync` hook, so without this the output stays invisible until
        // something else happens to push it out.
        unsafe {
            esp_idf_svc::sys::fsync(1);
        }
    }

    fn save_config(&mut self, cfg: &Config) {
        let _ = self.nvs.set_str("ssid", &cfg.sta_ssid);
        let _ = self.nvs.set_str("pass", &cfg.sta_pass);
        let _ = self.nvs.set_u32("baud", cfg.baud);
        let _ = self.nvs.set_u32("sys_id", cfg.sys_id as u32);
        let _ = self.nvs.set_str("gcs_host", &cfg.gcs_host);
    }

    fn restart(&mut self) {
        unsafe { esp_idf_svc::sys::esp_restart() }
    }

    fn wifi_connected(&self) -> bool {
        self.wifi.is_connected().unwrap_or(false)
    }

    fn set_relay_host(&mut self, host: &str) {
        let parsed = host
            .trim()
            .parse::<Ipv4Addr>()
            .ok()
            .filter(|ip| !ip.is_unspecified());
        self.gcs_tcp = parsed.map(|ip| SocketAddr::from((ip, GCS_PORT_TCP)));
        self.gcs_udp = parsed.map(|ip| SocketAddr::from((ip, GCS_PORT_UDP)));
        // A different endpoint means the existing socket points at the wrong place.
        self.tcp = None;
        self.tx_queue.clear();
        match parsed {
            Some(ip) => log::info!("relay: endpoint {ip}:{GCS_PORT_TCP}"),
            None => log::warn!("relay: no endpoint configured (use HOST=<ip>, then SAVE)"),
        }
    }

    fn local_ip(&self) -> String {
        // The station interface's address, or "0.0.0.0" while DHCP has not delivered
        // one. The bridge treats that as a failed connection (see `Bridge::tick`),
        // because the driver reports "connected" either way and every relay attempt
        // then fails with "host unreachable".
        //
        // Done through the C API rather than `EspWifi::sta_netif()`: that would need a
        // mutable borrow for a read-only question.
        unsafe {
            let handle =
                esp_idf_svc::sys::esp_netif_get_handle_from_ifkey(c"WIFI_STA_DEF".as_ptr());
            if !handle.is_null() {
                let mut info = esp_idf_svc::sys::esp_netif_ip_info_t::default();
                if esp_idf_svc::sys::esp_netif_get_ip_info(handle, &mut info)
                    == esp_idf_svc::sys::ESP_OK
                    && info.ip.addr != 0
                {
                    let o = info.ip.addr.to_le_bytes();
                    return format!("{}.{}.{}.{}", o[0], o[1], o[2], o[3]);
                }
            }
        }
        "0.0.0.0".to_string()
    }

    fn rssi_dbm(&self) -> i32 {
        // What the driver sees for the access point we are associated with.
        let mut ap: esp_idf_svc::sys::wifi_ap_record_t = unsafe { core::mem::zeroed() };
        let err = unsafe { esp_idf_svc::sys::esp_wifi_sta_get_ap_info(&mut ap) };
        if err == esp_idf_svc::sys::ESP_OK {
            ap.rssi as i32
        } else {
            0
        }
    }

    fn tx_power_dbm(&self) -> i32 {
        // ESP-IDF reports the configured maximum in quarter-dBm steps.
        let mut quarter_dbm: i8 = 0;
        let err = unsafe { esp_idf_svc::sys::esp_wifi_get_max_tx_power(&mut quarter_dbm) };
        if err == esp_idf_svc::sys::ESP_OK {
            quarter_dbm as i32 / 4
        } else {
            0
        }
    }

    fn tcp_connected(&self) -> bool {
        self.tcp.is_some()
    }

    fn wifi_activate(&mut self, cfg: &Config) {
        // The joined network is exactly the configured one: no scan-based
        // selection and no driver-side profile (see `Platform::new`).
        log::info!("wifi: joining configured network '{}'", cfg.sta_ssid);
        let _ = self.wifi.start();
        self.refresh_auth_method(cfg.sta_ssid.as_str());
        // The original C++ firmware set the radio up like this, and says why in a
        // comment: "adaptive power removed: fixed 11 dBm is a verified working value
        // (at 2 dBm the link is weaker, the maximum is not needed)", together with
        // `WiFi.setSleep(false)`. Fewer dBm means less heat and less current, which
        // matters on a marginal USB supply. ESP-IDF takes quarter-dBm units, so
        // 11 dBm is 44. Modem sleep is off because its latency spikes make a relay
        // link look unreliable.
        unsafe {
            let tx = esp_idf_svc::sys::esp_wifi_set_max_tx_power(44);
            let ps = esp_idf_svc::sys::esp_wifi_set_ps(
                esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_NONE,
            );
            log::info!("wifi: tx power 11 dBm (err={tx}), power save off (err={ps})");
        }
        let conf = Configuration::Client(ClientConfiguration {
            ssid: cfg.sta_ssid.as_str().try_into().unwrap_or_default(),
            password: cfg.sta_pass.as_str().try_into().unwrap_or_default(),
            auth_method: self.auth_method,
            ..Default::default()
        });
        let _ = self.wifi.set_configuration(&conf);
        let _ = self.wifi.connect();
        self.wifi_on = true;
    }

    fn wifi_deactivate(&mut self) {
        let _ = self.wifi.disconnect();
        let _ = self.wifi.stop();
        self.wifi_on = false;
        self.tcp = None;
    }

    fn wifi_full_restart(&mut self, cfg: &Config) {
        let _ = self.wifi.disconnect();
        let _ = self.wifi.stop();
        self.tcp = None;
        // `stop()` above tears down the driver state; `wifi_activate` reconfigures
        // and restarts it.
        self.wifi_on = false;
        self.wifi_activate(cfg);
    }

    fn wifi_retry_connect(&mut self) {
        let _ = self.wifi.connect();
    }
}
