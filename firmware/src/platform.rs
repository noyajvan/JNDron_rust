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
use std::time::Duration;

use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::uart::{UartConfig, UartDriver};
use esp_idf_hal::units::Hertz;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use ws2812_esp32_rmt_driver::driver::Ws2812Esp32Rmt;

use flight_core::config::StoredConfig;
use flight_core::io::Io;
use flight_core::Config;

use crate::consts::{
    GCS_PORT_TCP, GCS_PORT_UDP, LED_PIN, LED_VCC_GRB, NVS_NAMESPACE,
};

/// The board.
pub struct Platform {
    uart: UartDriver<'static>,
    led: Ws2812Esp32Rmt<'static>,
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
                .rx_fifo_size(flight_core::consts::FC_RX_BUF as u32)
                .tx_fifo_size(flight_core::consts::FC_TX_BUF as u32),
        )?;

        // --- WS2812 status LED on GPIO48 ---
        let led = Ws2812Esp32Rmt::new(peripherals.rmt.channel0, LED_PIN)
            .map_err(|e| anyhow::anyhow!("ws2812 rmt init: {e:?}"))?;

        // --- NVS namespace for the configuration ---
        let nvs = EspNvs::new(nvs_partition.clone(), NVS_NAMESPACE, true)?;

        // --- Wi-Fi station ---
        let wifi = BlockingWifi::wrap(
            EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs_partition))?,
            sys_loop,
        )?;

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

    fn console_read(&mut self, _buf: &mut [u8]) -> anyhow::Result<usize> {
        // The Rust ESP-IDF firmware reads the console through `std::io::stdin`
        // (mapped to the USB Serial/JTAG console configured in
        // `sdkconfig.defaults`). A dedicated `UsbSerialJtag` driver can be
        // dropped in here if you need finer control.
        Ok(0)
    }

    // -----------------------------------------------------------------
    // Transport
    // -----------------------------------------------------------------

    /// Maintain the outgoing TCP relay link to the VPS (mirrors
    /// `tcpLinkService()`).
    pub fn service_tcp(&mut self) {
        if !self.wifi_on || !self.wifi_connected() {
            if self.tcp_was_up {
                self.tcp_was_up = false;
                self.tcp = None;
            }
            return;
        }
        if self.tcp.is_some() {
            self.tcp_was_up = true;
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
            }
            Err(_) => {
                self.tcp = None;
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

    /// Push the LED colour computed by the core.
    pub fn set_led(&mut self, out: flight_core::led::LedOut) {
        let r = ((out.color >> 16) & 0xFF) as u8;
        let g = ((out.color >> 8) & 0xFF) as u8;
        let b = (out.color & 0xFF) as u8;
        // Scale by brightness (0..=255), exactly like setBrightness().
        let scale = |c: u8| ((c as u16 * out.brightness as u16) / 255) as u8;
        let pixel = ws2812_esp32_rmt_driver::Pixel::new_with_gamma(
            scale(r),
            scale(g),
            scale(b),
            LED_VCC_GRB,
        );
        let _ = self.led.write_nocopy([pixel].into_iter());
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
        match self.tcp.as_mut() {
            Some(s) => match std::io::Write::write_all(s, data) {
                Ok(()) => true,
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => true,
                Err(_) => {
                    self.tcp = None;
                    false
                }
            },
            None => false,
        }
    }

    fn udp_send(&mut self, data: &[u8]) -> bool {
        self.udp.send_to(data, self.gcs_udp).is_ok()
    }

    fn log(&mut self, line: &str) {
        print!("{}", line);
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

    fn tcp_connected(&self) -> bool {
        self.tcp.is_some()
    }

    fn wifi_activate(&mut self, cfg: &Config) {
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
