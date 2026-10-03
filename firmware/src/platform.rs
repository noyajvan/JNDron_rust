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

    udp: UdpSocket,
    tcp: Option<TcpStream>,
    /// GCS address for the outgoing TCP relay.
    gcs_tcp: SocketAddr,
    /// GCS address for UDP fallback (same host, UDP port).
    gcs_udp: SocketAddr,
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
    /// Rate limit for the "paused the FC link" log line.
    tx_last_pause_log_ms: u32,
}

impl Platform {
    pub fn new(
        peripherals: esp_idf_hal::peripherals::Peripherals,
        sys_loop: esp_idf_svc::eventloop::EspSystemEventLoop,
        nvs_partition: EspDefaultNvsPartition,
        gcs: Ipv4Addr,
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
            udp,
            tcp: None,
            gcs_tcp: SocketAddr::from((gcs, GCS_PORT_TCP)),
            gcs_udp: SocketAddr::from((gcs, GCS_PORT_UDP)),
            tcp_last_try_ms: 0,
            tcp_was_up: false,
            tcp_last_log_ms: 0,
            tx_queue: VecDeque::new(),
            tx_dropped: 0,
            tx_last_drop_log_ms: 0,
            tx_last_pause_log_ms: 0,
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
        self.tcp_was_up = false;
        let now = self.now_ms();
        if now.wrapping_sub(self.tcp_last_try_ms) < 3_000 {
            return;
        }
        self.tcp_last_try_ms = now;
        match TcpStream::connect_timeout(&self.gcs_tcp, Duration::from_millis(2_000)) {
            Ok(s) => {
                let _ = s.set_nodelay(true);
                let _ = s.set_nonblocking(true);
                self.tcp = Some(s);
                self.tcp_was_up = true;
                log::info!("relay: TCP connected to {}", self.gcs_tcp);
            }
            Err(e) => {
                self.tcp = None;
                // Rate limited, otherwise a dead VPS floods the console.
                if now.wrapping_sub(self.tcp_last_log_ms) >= 30_000 {
                    self.tcp_last_log_ms = now;
                    log::warn!("relay: TCP connect to {} failed: {e:?}", self.gcs_tcp);
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
        // Backpressure instead of loss: while a lot is still queued towards the
        // relay, stop reading the flight controller. Its own UART buffer then
        // fills and ArduPilot paces its output, which is exactly what MAVLink
        // expects, where dropping frames is not.
        if self.tx_queue.len() >= crate::consts::TCP_TX_HIGH_WATER {
            let now = self.now_ms();
            if now.wrapping_sub(self.tx_last_pause_log_ms) >= 10_000 {
                self.tx_last_pause_log_ms = now;
                log::warn!(
                    "relay: {} bytes queued, pausing the FC link until the 4G link drains",
                    self.tx_queue.len()
                );
            }
            return;
        }
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
        while !self.tx_queue.is_empty() {
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
        self.udp.send_to(data, self.gcs_udp).is_ok()
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
    }

    fn restart(&mut self) {
        unsafe { esp_idf_svc::sys::esp_restart() }
    }

    fn wifi_connected(&self) -> bool {
        self.wifi.is_connected().unwrap_or(false)
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

    fn tcp_connected(&self) -> bool {
        self.tcp.is_some()
    }

    fn wifi_activate(&mut self, cfg: &Config) {
        // The joined network is exactly the configured one: no scan-based
        // selection and no driver-side profile (see `Platform::new`).
        log::info!("wifi: joining configured network '{}'", cfg.sta_ssid);
        let conf = Configuration::Client(ClientConfiguration {
            ssid: cfg.sta_ssid.as_str().try_into().unwrap_or_default(),
            password: cfg.sta_pass.as_str().try_into().unwrap_or_default(),
            auth_method: AuthMethod::WPA2Personal,
            ..Default::default()
        });
        let _ = self.wifi.set_configuration(&conf);
        let _ = self.wifi.start();
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
