//! Wi-Fi provisioning over BLE (NimBLE host).
//!
//! A fresh device has no Wi-Fi credentials, and the bridge cannot do anything
//! until it has some. Rather than requiring a USB cable and a serial terminal
//! in the field, the firmware advertises the standard ESP-IDF provisioning
//! service, which the "ESP BLE Provisioning" phone app drives: pick the device,
//! enter SSID/password, done.
//!
//! Credentials arrive asynchronously in [`on_prov_event`], are stashed in the
//! buffers below, and `main` picks them up with [`take_credentials`], stores
//! them in NVS and starts Wi-Fi.

use core::ffi::c_void;
use std::ffi::CString;
use std::sync::atomic::{AtomicBool, Ordering};

use esp_idf_svc::sys as sys;

/// Proof of possession the phone app must be given, printed on the console.
pub const POP: &str = "jndron";

static GOT_CREDS: AtomicBool = AtomicBool::new(false);
static mut SSID_BUF: [u8; 33] = [0; 33];
static mut PASS_BUF: [u8; 65] = [0; 65];

unsafe extern "C" fn on_prov_event(
    _user_data: *mut c_void,
    event: sys::wifi_prov_cb_event_t,
    event_data: *mut c_void,
) {
    if event == sys::wifi_prov_cb_event_t_WIFI_PROV_CRED_RECV && !event_data.is_null() {
        let sta = &*(event_data as *const sys::wifi_sta_config_t);
        copy_nul_terminated(&sta.ssid, &mut *core::ptr::addr_of_mut!(SSID_BUF));
        copy_nul_terminated(&sta.password, &mut *core::ptr::addr_of_mut!(PASS_BUF));
        GOT_CREDS.store(true, Ordering::Release);
    }
}

fn copy_nul_terminated(src: &[u8], dst: &mut [u8]) {
    let end = src.iter().position(|b| *b == 0).unwrap_or(src.len());
    let n = end.min(dst.len() - 1);
    dst[..n].copy_from_slice(&src[..n]);
    dst[n] = 0;
}

fn buf_to_string(buf: &[u8]) -> String {
    let end = buf.iter().position(|b| *b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..end]).into_owned()
}

/// Start advertising the provisioning service. Non-blocking: the manager runs
/// in its own task and `main` keeps servicing the console and the FSM.
pub fn start(device_name: &str) -> anyhow::Result<()> {
    unsafe {
        // The BLE provisioning scheme wants the station interface.
        sys::esp_wifi_set_mode(sys::wifi_mode_t_WIFI_MODE_STA);

        // Frees the Bluetooth controller memory once provisioning is over.
        let mut scheme_handler = sys::wifi_prov_event_handler_t::default();
        scheme_handler.event_cb = Some(sys::wifi_prov_scheme_ble_event_cb_free_btdm);

        let mut app_handler = sys::wifi_prov_event_handler_t::default();
        app_handler.event_cb = Some(on_prov_event);

        let mut config = sys::wifi_prov_mgr_config_t::default();
        config.scheme = sys::wifi_prov_scheme_ble;
        config.scheme_event_handler = scheme_handler;
        config.app_event_handler = app_handler;

        let err = sys::wifi_prov_mgr_init(config);
        if err != sys::ESP_OK {
            anyhow::bail!("wifi_prov_mgr_init failed ({err})");
        }

        let pop = CString::new(POP)?;
        let name = CString::new(device_name)?;
        let err = sys::wifi_prov_mgr_start_provisioning(
            sys::wifi_prov_security_WIFI_PROV_SECURITY_1,
            pop.as_ptr() as *const c_void,
            name.as_ptr(),
            core::ptr::null(),
        );
        if err != sys::ESP_OK {
            anyhow::bail!("wifi_prov_mgr_start_provisioning failed ({err})");
        }
    }

    log::info!("BLE provisioning: name '{device_name}', PoP '{POP}'");
    Ok(())
}

/// Credentials received from the phone, if any arrived since the last call.
pub fn take_credentials() -> Option<(String, String)> {
    if !GOT_CREDS.load(Ordering::Acquire) {
        return None;
    }
    unsafe {
        let ssid = buf_to_string(&*core::ptr::addr_of!(SSID_BUF));
        let pass = buf_to_string(&*core::ptr::addr_of!(PASS_BUF));
        GOT_CREDS.store(false, Ordering::Release);
        Some((ssid, pass))
    }
}

/// Stop provisioning and release the BLE controller.
pub fn stop() {
    unsafe {
        sys::wifi_prov_mgr_deinit();
    }
}
