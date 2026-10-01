//! The bridge: a faithful, hardware-free port of `main_DrnBrdg.cpp`,
//! `mavlink_util.cpp`, `state_machine.cpp` and the transport part of
//! `wifi_mgr.cpp`.
//!
//! One [`Bridge`] owns *all* mutable state that the C++ firmware kept in
//! globals. Every I/O call goes through an [`Io`], so the whole thing can be
//! driven from a test.

use crate::consts::*;
use crate::crash::{CrashDetector, CrashInput};
use crate::io::Io;
use crate::led::{self, LedInput, LedOut};
use crate::mavlink::{self, Frame, Parser};
use crate::messages::{self, defs, Heartbeat, Message, Statustext};
use crate::queues::TextRing;
use crate::Config;

/// `static` locals of the state machine, lifted into the struct.
#[derive(Debug, Clone)]
struct FsmStatics {
    last_state: State,
    wifi_timeout_chk_ms: u32,
    ekf_ok_prev: bool,
    ekf_att_stable_ms: u32,
    ekf_queued: bool,
    init_wait_log: u32,
    last_log_wifi: u32,
    last_log_hb: u32,
    last_log_ekf: u32,
    ekf_timeout_log: u32,
    auto_sent: bool,
    land_stop_ms: u32,
    tip_bounce_ms: u32,
    cal_pct_ms: u32,
    cal_accept_sent: bool,
}

impl Default for FsmStatics {
    fn default() -> Self {
        FsmStatics {
            last_state: State::InitWifi,
            wifi_timeout_chk_ms: 0,
            ekf_ok_prev: false,
            ekf_att_stable_ms: 0,
            ekf_queued: false,
            init_wait_log: 0,
            last_log_wifi: 0,
            last_log_hb: 0,
            last_log_ekf: 0,
            ekf_timeout_log: 0,
            auto_sent: false,
            land_stop_ms: 0,
            tip_bounce_ms: 0,
            cal_pct_ms: 0,
            cal_accept_sent: false,
        }
    }
}

/// `static` locals of the MAVLink layer.
#[derive(Debug, Clone)]
struct MavStatics {
    first_hb: bool,
    last_cal_status: u8,
    last_fail_status: u8,
    last_fail_fit: f32,
    last_fwd_fail_ms: u32,
    last_do_connect_msg: u32,
}

impl Default for MavStatics {
    fn default() -> Self {
        MavStatics {
            first_hb: true,
            last_cal_status: 255,
            last_fail_status: 255,
            last_fail_fit: -1.0,
            last_fwd_fail_ms: 0,
            last_do_connect_msg: 0,
        }
    }
}

/// The whole drone-bridge state.
#[derive(Debug, Clone)]
pub struct Bridge {
    pub cfg: Config,
    pub state: State,
    /// `60` = flight loop stopped, otherwise running.
    pub mdfly: i32,

    // --- Wi-Fi / transport ---
    pub has_wifi: bool,
    pub has_server: bool,
    pub wifi_on: bool,
    pub wifi_activating: bool,
    pub wifi_try_start: u32,
    pub sta_was_connected: bool,
    pub last_sta_status: i32,
    wifi_last_retry_ms: u32,
    wifi_last_ok_ms: u32,
    wifi_restart_timer: u32,

    // --- MAVLink ---
    pub parser: Parser,
    pub tx_seq: u8,
    pub fc_sys_id: u8,

    // --- FC telemetry ---
    pub heartbeat_received: bool,
    pub mag_sensor_ready: bool,
    pub fc_hb_first_ms: u32,
    pub current_custom_mode: u32,
    pub is_armed: bool,
    pub system_status: u8,
    pub gps_fix_type: u8,
    pub gps_sats: u8,
    pub battery_voltage: f32,
    pub battery_remaining: i8,
    pub sys_status_received: bool,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub vfr_alt: f32,
    pub vfr_climb: f32,

    pub ekf_flags: u16,
    pub mag_test_ratio: f32,
    pub ekf_report_received: bool,

    pub rot_snap_roll: f32,
    pub rot_snap_pitch: f32,
    pub rot_detected: bool,

    // --- magnetometer calibration ---
    pub mag_error_msg_sent: bool,
    pub mag_ok_msg_sent: bool,
    pub cal_cmd_sent: bool,
    pub cal_success: bool,
    pub cal_fc_failed: bool,
    pub cal_finalized: bool,
    pub cal_completion_pct: u8,
    pub last_cal_pct: u8,
    pub cal_fitness: f32,
    pub cal_dia_x: f32,
    pub cal_dia_y: f32,
    pub cal_dia_z: f32,
    pub cal_ofs_x: f32,
    pub cal_ofs_y: f32,
    pub cal_ofs_z: f32,
    pub cal_retries: u8,
    pub cal_dia_reported: bool,

    // --- arming / mission ---
    pub no_arm_init: bool,
    pub arm_cmd_sent: bool,
    pub mode_cmd_sent: bool,
    pub mission_start_msg: bool,
    pub flew_above_1m: bool,
    pub mission_base_alt: f32,
    pub mission_count: u16,
    pub mission_loaded: bool,
    pub mission_first_parsed: bool,
    pub last_mission_req: u32,
    pub mission_has_land: bool,
    pub landed_state: u8,

    // --- clocks ---
    pub start_time: u32,
    pub last_wifi_hb: u32,
    pub last_serial_log: u32,
    pub state_entry_ms: u32,
    pub last_arm_retry_ms: u32,
    pub last_mode_retry_ms: u32,
    pub last_reason_report_ms: u32,
    pub last_server_pkt_ms: u32,

    // --- queues / console ---
    pub status_queue: TextRing<STATUS_QUEUE_SIZE>,
    pub reason_queue: TextRing<REASON_QUEUE_SIZE>,
    pub input_buffer: String,

    // --- crash ---
    pub crash_triggered: bool,
    pub crash: CrashDetector,
    pub crash_ground_speed: f32,
    pub crash_throttle: u16,
    pub crash_climb: f32,
    pub crash_gyro_x: f32,
    pub crash_gyro_y: f32,

    // --- counters ---
    pub fc_bytes: u32,
    pub fc_msgs: u32,

    fsm: FsmStatics,
    mav: MavStatics,
}

impl Bridge {
    /// Fresh bridge, mirroring the C++ global initialisers.
    pub fn new(cfg: Config) -> Self {
        Bridge {
            cfg,
            state: State::InitWifi,
            mdfly: 0,

            has_wifi: false,
            has_server: false,
            wifi_on: false,
            wifi_activating: false,
            wifi_try_start: 0,
            sta_was_connected: false,
            last_sta_status: -1,
            wifi_last_retry_ms: 0,
            wifi_last_ok_ms: 0,
            wifi_restart_timer: 0,

            parser: Parser::new(),
            tx_seq: 0,
            fc_sys_id: 1,

            heartbeat_received: false,
            mag_sensor_ready: false,
            fc_hb_first_ms: 0,
            current_custom_mode: 0,
            is_armed: false,
            system_status: 0,
            gps_fix_type: 0,
            gps_sats: 0,
            battery_voltage: 0.0,
            battery_remaining: -1,
            sys_status_received: false,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            vfr_alt: 0.0,
            vfr_climb: 0.0,

            ekf_flags: 0,
            mag_test_ratio: 0.0,
            ekf_report_received: false,

            rot_snap_roll: 0.0,
            rot_snap_pitch: 0.0,
            rot_detected: false,

            mag_error_msg_sent: false,
            mag_ok_msg_sent: false,
            cal_cmd_sent: false,
            cal_success: false,
            cal_fc_failed: false,
            cal_finalized: false,
            cal_completion_pct: 0,
            last_cal_pct: 0,
            cal_fitness: 0.0,
            cal_dia_x: 0.0,
            cal_dia_y: 0.0,
            cal_dia_z: 0.0,
            cal_ofs_x: 0.0,
            cal_ofs_y: 0.0,
            cal_ofs_z: 0.0,
            cal_retries: 0,
            cal_dia_reported: false,

            no_arm_init: false,
            arm_cmd_sent: false,
            mode_cmd_sent: false,
            mission_start_msg: false,
            flew_above_1m: false,
            mission_base_alt: 0.0,
            mission_count: 0,
            mission_loaded: false,
            mission_first_parsed: false,
            last_mission_req: 0,
            mission_has_land: false,
            landed_state: 0,

            start_time: 0,
            last_wifi_hb: 0,
            last_serial_log: 0,
            state_entry_ms: 0,
            last_arm_retry_ms: 0,
            last_mode_retry_ms: 0,
            last_reason_report_ms: 0,
            last_server_pkt_ms: 0,

            status_queue: TextRing::new(),
            reason_queue: TextRing::new(),
            input_buffer: String::new(),

            crash_triggered: false,
            crash: CrashDetector::new(),
            crash_ground_speed: 0.0,
            crash_throttle: 0,
            crash_climb: 0.0,
            crash_gyro_x: 0.0,
            crash_gyro_y: 0.0,

            fc_bytes: 0,
            fc_msgs: 0,

            fsm: FsmStatics::default(),
            mav: MavStatics::default(),
        }
    }

    /// Perform the part of `setup()` that touches configuration.
    pub fn boot(&mut self, io: &mut dyn Io) {
        self.start_time = io.now_ms();
        self.state_entry_ms = self.start_time;
        io.log("\n=== JNDron Rust DroneBridge ===");
        io.log("> ");
        self.queue_statustext("Bridge Ready");
        if self.cfg.has_ssid() {
            self.wifi_activate(io);
        }
    }

    // ---------------------------------------------------------------------
    // Outgoing MAVLink helpers
    // ---------------------------------------------------------------------

    /// Frame + sequence a payload as MAVLink 2 from the bridge's own system id.
    fn encode(&mut self, msgid: u32, payload: Vec<u8>) -> Vec<u8> {
        let extra = defs::find(msgid).map(|d| d.crc_extra());
        let seq = self.tx_seq;
        self.tx_seq = self.tx_seq.wrapping_add(1);
        mavlink::encode_v2(
            seq,
            self.cfg.sys_id,
            MAV_COMP_ID_ONBOARD_COMPUTER,
            msgid,
            &payload,
            extra,
        )
    }

    /// Forward already-decoded FC traffic towards the GCS (TCP first, then UDP).
    pub fn forward_to_wifi(&mut self, io: &mut dyn Io, data: &[u8]) {
        if !self.wifi_on || !io.wifi_connected() {
            return;
        }
        if io.tcp_connected() {
            if !io.tcp_send(data) {
                // `tcpLink.write() == 0` -> drop the link.
                self.tcp_link_stop(io);
            }
            return;
        }
        // UDP fallback: two copies, with a 250 ms backoff after a failure.
        let now = io.now_ms();
        if self.mav.last_fwd_fail_ms != 0 && now.wrapping_sub(self.mav.last_fwd_fail_ms) < 250 {
            return;
        }
        if !io.udp_send(data) {
            self.mav.last_fwd_fail_ms = now;
        }
        if !io.udp_send(data) {
            self.mav.last_fwd_fail_ms = now;
        }
    }

    fn tcp_link_stop(&mut self, io: &mut dyn Io) {
        // The platform closes the socket; we just make sure we stop believing
        // it is connected.
        let _ = io;
    }

    /// `sendToBoth`: FC UART and the radio link.
    pub fn send_to_both(&mut self, io: &mut dyn Io, data: &[u8]) {
        io.fc_write(data);
        self.forward_to_wifi(io, data);
    }

    /// Encode + send to the FC only.
    fn send_fc(&mut self, io: &mut dyn Io, msgid: u32, payload: Vec<u8>) {
        let bytes = self.encode(msgid, payload);
        io.fc_write(&bytes);
    }

    /// `send_statustext()` - FC only.
    pub fn send_statustext(&mut self, io: &mut dyn Io, text: &str) {
        let st = Statustext {
            severity: MAV_SEVERITY_INFO,
            text: text.to_string(),
        };
        self.send_fc(io, messages::id::STATUSTEXT, st.payload());
    }

    /// `send_statustext_udp()` - radio only.
    pub fn send_statustext_udp(&mut self, io: &mut dyn Io, text: &str) {
        if !self.wifi_on || !io.wifi_connected() {
            return;
        }
        let st = Statustext {
            severity: MAV_SEVERITY_INFO,
            text: text.to_string(),
        };
        let bytes = self.encode(messages::id::STATUSTEXT, st.payload());
        self.forward_to_wifi(io, &bytes);
    }

    /// Queue a STATUS_TEXT for the 500 ms heartbeat drain.
    pub fn queue_statustext(&mut self, text: &str) {
        self.status_queue.push(text);
    }

    /// `send_queued_statustext()` - up to 5 queued lines per call.
    pub fn send_queued_statustext(&mut self, io: &mut dyn Io) {
        for _ in 0..5 {
            let Some(text) = self.status_queue.pop() else {
                break;
            };
            let st = Statustext {
                severity: MAV_SEVERITY_INFO,
                text,
            };
            let bytes = self.encode(messages::id::STATUSTEXT, st.payload());
            self.send_to_both(io, &bytes);
        }
    }

    /// `send_heartbeat()`.
    pub fn send_heartbeat(&mut self, io: &mut dyn Io) {
        let hb = Heartbeat {
            custom_mode: 0,
            typ: MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot: MAV_AUTOPILOT_INVALID,
            base_mode: 0,
            system_status: 0,
            // mavgen fills MAVLINK_VERSION here.
            mavlink_version: 3,
        };
        let bytes = self.encode(messages::id::HEARTBEAT, hb.payload());
        io.fc_write(&bytes);
        self.forward_to_wifi(io, &bytes);
    }

    /// `send_mission_request_list()`.
    pub fn send_mission_request_list(&mut self, io: &mut dyn Io) {
        let p = messages::mission_request_list_payload(
            self.fc_sys_id,
            MAV_COMP_ID_AUTOPILOT1,
            MAV_MISSION_TYPE_MISSION,
        );
        self.send_fc(io, messages::id::MISSION_REQUEST_LIST, p);
    }

    /// `send_command_long()`.
    pub fn send_command_long(&mut self, io: &mut dyn Io, cmd: u16, p: [f32; 7]) {
        let payload = messages::command_long_payload(
            self.fc_sys_id,
            MAV_COMP_ID_AUTOPILOT1,
            cmd,
            0,
            p[0],
            p[1],
            p[2],
            p[3],
            p[4],
            p[5],
            p[6],
        );
        self.send_fc(io, messages::id::COMMAND_LONG, payload);
    }

    pub fn send_arm(&mut self, io: &mut dyn Io) {
        self.send_command_long(
            io,
            MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
    }
    pub fn send_force_disarm(&mut self, io: &mut dyn Io) {
        self.send_command_long(
            io,
            MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
    }
    pub fn send_set_mode(&mut self, io: &mut dyn Io, mode: u8) {
        self.send_command_long(
            io,
            MAV_CMD_DO_SET_MODE,
            [
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED as f32,
                mode as f32,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        );
    }
    pub fn send_set_relay(&mut self, io: &mut dyn Io) {
        self.send_command_long(
            io,
            MAV_CMD_DO_SET_RELAY,
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
    }
    pub fn send_start_mag_cal(&mut self, io: &mut dyn Io) {
        self.send_command_long(
            io,
            MAV_CMD_DO_START_MAG_CAL,
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        );
    }
    pub fn send_accept_mag_cal(&mut self, io: &mut dyn Io) {
        self.send_command_long(io, MAV_CMD_DO_ACCEPT_MAG_CAL, [0.0; 7]);
    }
    pub fn send_preflight_storage(&mut self, io: &mut dyn Io) {
        self.send_command_long(
            io,
            MAV_CMD_PREFLIGHT_STORAGE,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
    }

    // ---------------------------------------------------------------------
    // Wi-Fi control
    // ---------------------------------------------------------------------

    pub fn wifi_activate(&mut self, io: &mut dyn Io) {
        if self.wifi_on || !self.cfg.has_ssid() {
            return;
        }
        self.wifi_on = true;
        self.has_wifi = false;
        self.has_server = false;
        io.wifi_activate(&self.cfg);
        self.wifi_activating = true;
        self.wifi_try_start = io.now_ms();
    }

    pub fn wifi_deactivate(&mut self, io: &mut dyn Io) {
        if !self.wifi_on {
            return;
        }
        self.wifi_on = false;
        self.wifi_activating = false;
        self.sta_was_connected = false;
        self.has_wifi = false;
        self.has_server = false;
        io.wifi_deactivate();
        self.queue_statustext("WiFi OFF");
    }

    pub fn wifi_retry_connect(&mut self, io: &mut dyn Io) {
        if !self.wifi_on || !self.cfg.has_ssid() {
            return;
        }
        io.wifi_retry_connect();
    }

    pub fn wifi_full_restart(&mut self, io: &mut dyn Io) {
        if !self.wifi_on {
            return;
        }
        self.queue_statustext("WiFi restart");
        io.wifi_full_restart(&self.cfg);
        self.wifi_activating = true;
        self.wifi_try_start = io.now_ms();
    }

    // ---------------------------------------------------------------------
    // Inbound data
    // ---------------------------------------------------------------------

    /// Feed bytes read from the flight controller. Returns how many bytes were
    /// consumed and how many messages were decoded.
    pub fn feed_fc_bytes(&mut self, io: &mut dyn Io, data: &[u8]) {
        let mut frames: Vec<Frame> = Vec::new();
        self.parser.push_slice(data, &mut frames);
        self.fc_bytes = self.fc_bytes.wrapping_add(data.len() as u32);
        for frame in frames {
            self.fc_msgs = self.fc_msgs.wrapping_add(1);
            let def = defs::find(frame.msgid);
            // Re-emit with the *original* header (sysid/compid/seq), like
            // `mavlink_msg_to_send_buffer(&mavMsg)` did.
            let bytes = frame.encode_v2(frame.seq, def);
            let msg = Message::decode(frame.msgid, &frame.payload);
            let spam = msg.is_calibration_spam();
            if !spam {
                self.forward_to_wifi(io, &bytes);
            }
            self.handle_message(io, msg);
        }
    }

    /// Feed bytes received from the GCS (TCP or UDP).
    pub fn feed_gcs_bytes(&mut self, io: &mut dyn Io, data: &[u8]) {
        let now = io.now_ms();
        self.last_server_pkt_ms = now;
        if !self.has_server {
            self.has_server = true;
            if now.wrapping_sub(self.mav.last_do_connect_msg) > 60_000 {
                self.queue_statustext("DO connected");
                self.mav.last_do_connect_msg = now;
            }
        }
        io.fc_write(data);
    }

    // ---------------------------------------------------------------------
    // Message handling (mavlink_util.cpp)
    // ---------------------------------------------------------------------

    fn handle_message(&mut self, io: &mut dyn Io, msg: Message) {
        match msg {
            Message::Heartbeat(hb) => {
                // Note: sysid comes from the frame; kept at the bridge level.
                self.heartbeat_received = true;
                if self.fc_hb_first_ms == 0 {
                    self.fc_hb_first_ms = io.now_ms();
                }
                self.current_custom_mode = hb.custom_mode;
                self.system_status = hb.system_status;
                self.is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                if self.mav.first_hb {
                    self.mav.first_hb = false;
                    self.queue_statustext("Mavlink OK");
                }
            }

            Message::Statustext(stxt) => {
                if stxt.severity <= MAV_SEVERITY_WARNING {
                    self.reason_queue.push(&stxt.text);
                }
            }

            Message::EkfStatusReport(ekf) => {
                self.ekf_flags = ekf.flags;
                self.mag_test_ratio = ekf.compass_variance;
                self.ekf_report_received = true;
            }

            Message::Attitude(att) => {
                self.roll_deg = att.roll * RAD_TO_DEG;
                self.pitch_deg = att.pitch * RAD_TO_DEG;
            }

            Message::SysStatus(ss) => {
                self.battery_voltage = ss.voltage_battery as f32 / 1000.0;
                self.battery_remaining = ss.battery_remaining;
                self.sys_status_received = true;
                self.mag_sensor_ready =
                    (ss.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_3D_MAG) != 0
                        && (ss.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG) != 0
                        && (ss.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG) != 0;
            }

            Message::VfrHud(vfr) => {
                self.vfr_alt = vfr.alt;
                self.vfr_climb = vfr.climb;
                self.crash_ground_speed = vfr.groundspeed;
                self.crash_throttle = vfr.throttle;
                self.crash_climb = vfr.climb;
                self.check_crash(io);
            }

            Message::ExtendedSysState(es) => {
                self.landed_state = es.landed_state;
            }

            Message::RawImu(imu) => {
                self.crash_gyro_x = imu.xgyro as f32;
                self.crash_gyro_y = imu.ygyro as f32;
                self.check_crash(io);
            }

            Message::GpsRawInt(gps) => {
                self.gps_fix_type = gps.fix_type;
                self.gps_sats = gps.satellites_visible;
            }

            Message::MagCalProgress(cal) => {
                self.cal_completion_pct = cal.completion_pct;
                if cal.cal_status != self.mav.last_cal_status
                    || (cal.cal_status == 4 && !self.cal_success)
                {
                    self.mav.last_cal_status = cal.cal_status;
                    io.log(&format!(
                        "[CAL] status={} pct={} compass={}",
                        cal.cal_status, cal.completion_pct, cal.compass_id
                    ));
                }
                if cal.cal_status == 4 {
                    io.log("[CAL] PROGRESS SUCCESS");
                    self.cal_success = true;
                }
            }

            Message::MagCalReport(rep) => {
                io.log(&format!(
                    "[CAL] report status={} fitness={:.3} ofs=({:.1},{:.1},{:.1})",
                    rep.cal_status, rep.fitness, rep.ofs_x, rep.ofs_y, rep.ofs_z
                ));
                self.cal_dia_x = rep.diag_x;
                self.cal_dia_y = rep.diag_y;
                self.cal_dia_z = rep.diag_z;
                self.cal_ofs_x = rep.ofs_x;
                self.cal_ofs_y = rep.ofs_y;
                self.cal_ofs_z = rep.ofs_z;
                self.cal_fitness = rep.fitness;

                if rep.cal_status == 4 {
                    io.log("[CAL] REPORT SUCCESS");
                    self.cal_success = true;
                    if !self.cal_dia_reported {
                        self.cal_dia_reported = true;
                        let d = |name: &str, v: f32| {
                            format!(
                                "{}: {:.2} (0.85-1.15){}",
                                name,
                                v,
                                if (1.0 - v).abs() <= DIA_TOLERANCE {
                                    " ok"
                                } else {
                                    " !"
                                }
                            )
                        };
                        let o = |name: &str, v: f32| {
                            format!(
                                "{}: {:.0} (-1000..1000){}",
                                name,
                                v,
                                if v.abs() <= 1000.0 { " ok" } else { " !" }
                            )
                        };
                        self.queue_statustext("Calibration coefficients:");
                        self.queue_statustext(&d("DIA X", rep.diag_x));
                        self.queue_statustext(&d("DIA Y", rep.diag_y));
                        self.queue_statustext(&d("DIA Z", rep.diag_z));
                        self.queue_statustext(&o("OFS X", rep.ofs_x));
                        self.queue_statustext(&o("OFS Y", rep.ofs_y));
                        self.queue_statustext(&o("OFS Z", rep.ofs_z));
                    }
                } else if rep.cal_status == 5 || rep.cal_status == 6 {
                    self.cal_fc_failed = true;
                    if rep.cal_status != self.mav.last_fail_status
                        || (rep.fitness - self.mav.last_fail_fit).abs() > 0.01
                    {
                        self.mav.last_fail_status = rep.cal_status;
                        self.mav.last_fail_fit = rep.fitness;
                        let text = if rep.cal_status == 5 {
                            format!("Cal FAILED fit={:.2}", rep.fitness)
                        } else {
                            format!("Cal bad orientation fit={:.2}", rep.fitness)
                        };
                        self.queue_statustext(&text);
                    }
                    io.log(&format!(
                        "[CAL] REPORT {}",
                        if rep.cal_status == 5 {
                            "FAILED"
                        } else {
                            "BAD_ORIENTATION"
                        }
                    ));
                }
            }

            Message::MissionCount(mc) => {
                self.mission_count = mc.count;
            }

            Message::MissionItemInt(mi) => {
                if mi.command == MAV_CMD_NAV_LAND {
                    self.mission_has_land = true;
                }
            }

            Message::MissionAck(ack) => {
                if ack.ack_type == MAV_MISSION_ACCEPTED {
                    self.mission_loaded = true;
                }
            }

            Message::Other(_) => {}
        }
    }

    /// `checkCrashDetection()`.
    pub fn check_crash(&mut self, io: &mut dyn Io) {
        if self.crash_triggered {
            return;
        }
        let input = CrashInput {
            state: self.state,
            is_armed: self.is_armed,
            vfr_alt: self.vfr_alt,
            mission_base_alt: self.mission_base_alt,
            ground_speed: self.crash_ground_speed,
            throttle: self.crash_throttle,
            climb: self.crash_climb,
            gyro_x: self.crash_gyro_x,
            gyro_y: self.crash_gyro_y,
        };
        if let Some(kind) = self.crash.update(io.now_ms(), &input) {
            self.crash_triggered = true;
            io.log(&format!("[CRASH] {}", kind.label()));
        }
    }

    // ---------------------------------------------------------------------
    // Main loop
    // ---------------------------------------------------------------------

    /// Compute the LED output for the current tick.
    pub fn led(&self, now: u32) -> LedOut {
        led::update_led(
            now,
            &LedInput {
                mdfly: self.mdfly,
                heartbeat_received: self.heartbeat_received,
                has_server: self.has_server,
                wifi_on: self.wifi_on,
                has_wifi: self.has_wifi,
                state: self.state,
                cal_completion_pct: self.cal_completion_pct,
            },
        )
    }

    /// One iteration of `loop()` minus the terminal and the LED push.
    pub fn tick(&mut self, io: &mut dyn Io) {
        let now = io.now_ms();

        if now.wrapping_sub(self.last_wifi_hb) >= 500 {
            self.send_heartbeat(io);
            self.send_queued_statustext(io);
            self.last_wifi_hb = now;
        }

        self.update_system_state(io);

        let st_connected = io.wifi_connected();
        let st_dead = io.wifi_radio_dead();
        let st: i32 = if st_connected {
            3
        } else if st_dead {
            255
        } else {
            6
        };

        // Wi-Fi just came up.
        if self.wifi_activating && st_connected {
            self.wifi_activating = false;
            self.sta_was_connected = true;
            self.has_wifi = true;
            let ssid = self.cfg.sta_ssid.clone();
            let buf = format!("WiFi: {}", ssid);
            self.queue_statustext(&buf);
            self.send_statustext_udp(io, &buf);
            self.send_statustext_udp(io, "Bridge Ready");
            self.send_statustext_udp(io, "Mavlink OK");
        }

        // Retry while still activating.
        if self.wifi_activating && !st_connected {
            if now.wrapping_sub(self.wifi_try_start) > 60_000 {
                self.queue_statustext("WiFi full restart");
                self.wifi_full_restart(io);
                self.wifi_last_retry_ms = 0;
            } else if now.wrapping_sub(self.wifi_last_retry_ms) > 30_000 {
                self.wifi_last_retry_ms = now;
                self.queue_statustext("WiFi retry");
                self.wifi_retry_connect(io);
            }
        }

        // Wi-Fi watchdog.
        if self.wifi_on {
            if st_connected {
                self.wifi_last_ok_ms = now;
                self.wifi_restart_timer = 0;
            } else if st_dead {
                if self.wifi_restart_timer == 0 {
                    self.wifi_restart_timer = now;
                }
                if now.wrapping_sub(self.wifi_restart_timer) > 5_000 {
                    self.queue_statustext("WiFi radio dead, restarting");
                    self.wifi_full_restart(io);
                    self.wifi_restart_timer = 0;
                }
            } else if self.sta_was_connected {
                if self.wifi_last_ok_ms != 0 && now.wrapping_sub(self.wifi_last_ok_ms) > 30_000 {
                    self.queue_statustext("WiFi stuck, full restart");
                    self.wifi_full_restart(io);
                    self.wifi_last_ok_ms = 0;
                } else if self.wifi_last_ok_ms != 0
                    && now.wrapping_sub(self.wifi_last_ok_ms) > 10_000
                {
                    if self.wifi_restart_timer == 0 {
                        self.wifi_restart_timer = now;
                    }
                    if now.wrapping_sub(self.wifi_restart_timer) > 5_000 {
                        io.wifi_retry_connect();
                        self.wifi_restart_timer = now;
                    }
                }
            }
        }

        if st != self.last_sta_status && self.wifi_on {
            self.last_sta_status = st;
            if st == 3 {
                self.sta_was_connected = true;
                self.has_wifi = true;
            } else {
                self.has_wifi = false;
            }
        }

        if self.has_server
            && !io.tcp_connected()
            && now.wrapping_sub(self.last_server_pkt_ms) > 30_000
        {
            self.has_server = false;
        }

        if now.wrapping_sub(self.last_serial_log) >= 10_000 {
            io.log(&format!(
                "s={} h={} a={} m={} w={} f={} ekf=0x{:04X} mag={:.2} fc={}/{}",
                self.state.as_u8(),
                self.heartbeat_received as u8,
                self.is_armed as u8,
                self.current_custom_mode,
                st,
                self.mdfly,
                self.ekf_flags,
                self.mag_test_ratio,
                self.fc_bytes,
                self.fc_msgs
            ));
            self.last_serial_log = now;
        }
    }

    // ---------------------------------------------------------------------
    // State machine (state_machine.cpp)
    // ---------------------------------------------------------------------

    fn enter_state(&mut self, io: &mut dyn Io, new: State) {
        self.state = new;
        self.state_entry_ms = io.now_ms();
        self.crash_triggered = false;
        io.log(&format!(
            "[S] {}->{}",
            self.fsm.last_state.as_u8(),
            new.as_u8()
        ));

        match new {
            State::MagError => {
                self.mag_error_msg_sent = false;
                self.rot_snap_roll = self.roll_deg;
                self.rot_snap_pitch = self.pitch_deg;
                self.rot_detected = false;
                self.send_statustext_udp(io, "Bridge: MAG_ERROR");
            }
            State::MagOk => {
                self.mag_ok_msg_sent = false;
                self.rot_snap_roll = self.roll_deg;
                self.rot_snap_pitch = self.pitch_deg;
                self.rot_detected = false;
                self.send_statustext_udp(io, "Bridge: MAG_OK");
            }
            State::Calibration => {
                self.cal_cmd_sent = false;
                self.cal_success = false;
                self.cal_completion_pct = 0;
                self.cal_fitness = 0.0;
                self.last_cal_pct = 0;
                self.cal_retries = 0;
                self.cal_dia_reported = false;
                self.fsm.cal_accept_sent = false;
                self.cal_fc_failed = false;
                self.fsm.cal_pct_ms = self.state_entry_ms;
                self.send_statustext_udp(io, "Bridge: compass calibration");
            }
            State::CalibrationEnd => {
                self.cal_finalized = false;
            }
            State::NoArm => {
                self.no_arm_init = false;
            }
            State::StartMission => {
                self.mode_cmd_sent = false;
                self.last_mode_retry_ms = 0;
            }
            State::Mission => {
                self.mission_start_msg = false;
                self.flew_above_1m = false;
                self.fsm.land_stop_ms = 0;
                self.fsm.tip_bounce_ms = 0;
                self.mission_base_alt = -1.0;
            }
            _ => {}
        }
        self.fsm.last_state = new;
    }

    /// `updateSystemState()`.
    pub fn update_system_state(&mut self, io: &mut dyn Io) {
        let now = io.now_ms();

        self.has_wifi = self.wifi_on && io.wifi_connected();

        if !self.sta_was_connected
            && self.wifi_on
            && now.wrapping_sub(self.fsm.wifi_timeout_chk_ms) > WIFI_TIMEOUT_MS
        {
            self.fsm.wifi_timeout_chk_ms = now;
            self.wifi_full_restart(io);
        }

        if self.current_custom_mode != MODE_STABILIZE
            && self.current_custom_mode != MODE_AUTO
            && self.mdfly != MDFLY_STOP
        {
            self.mdfly = MDFLY_STOP;
            self.queue_statustext("mode != STAB/AUTO -> stop");
            self.send_statustext_udp(io, "Bridge: mode != STAB/AUTO -> stop");
        }
        if self.mdfly == MDFLY_STOP {
            return;
        }

        if self.heartbeat_received && !self.mission_first_parsed {
            if self.last_mission_req == 0 && now.wrapping_sub(self.start_time) > 2_000 {
                self.mission_count = 0;
                self.mission_loaded = false;
                self.mission_has_land = false;
                self.send_mission_request_list(io);
                self.last_mission_req = now;
            } else if self.last_mission_req != 0 && now.wrapping_sub(self.last_mission_req) > 5_000
            {
                self.mission_first_parsed = true;
                self.last_mission_req = 0;
                if self.mission_count > 0 {
                    self.mission_loaded = true;
                }
            }
        }

        if self.state != self.fsm.last_state {
            self.enter_state(io, self.state);
        }

        match self.state {
            State::InitWifi => {
                if self.state_entry_ms == 0 {
                    self.state_entry_ms = now;
                }
                if now.wrapping_sub(self.state_entry_ms) > 2_000 {
                    io.log("[S] INIT_WIFI -> INIT_MAVLINK");
                    self.enter_state(io, State::InitMavlink);
                } else if now.wrapping_sub(self.fsm.last_log_wifi) > 1_000 {
                    self.fsm.last_log_wifi = now;
                    io.log("[S] waiting INIT_WIFI 2s");
                }
            }

            State::InitMavlink => self.step_init_mavlink(io, now),

            State::MagError => {
                if !self.mag_error_msg_sent {
                    let buf = format!("mag={:.3} -> calibrating", self.mag_test_ratio);
                    self.queue_statustext(&buf);
                    self.send_statustext_udp(io, &buf);
                    self.send_statustext(io, &buf);
                    self.mag_error_msg_sent = true;
                }
                self.enter_state(io, State::Calibration);
            }

            State::MagOk => {
                if !self.mag_ok_msg_sent {
                    let buf = format!("mag={:.3} waiting rotation", self.mag_test_ratio);
                    self.queue_statustext(&buf);
                    self.send_statustext_udp(io, &buf);
                    self.send_statustext(io, &buf);
                    self.mag_ok_msg_sent = true;
                }

                if (self.roll_deg - self.rot_snap_roll).abs() > 45.0
                    || (self.pitch_deg - self.rot_snap_pitch).abs() > 45.0
                {
                    self.rot_detected = true;
                    self.send_statustext_udp(io, "Bridge: rotation detected");
                    io.log(&format!(
                        "[ROT] MAG_OK dR={:.1} dP={:.1}!",
                        self.roll_deg - self.rot_snap_roll,
                        self.pitch_deg - self.rot_snap_pitch
                    ));
                }

                if self.rot_detected {
                    self.enter_state(io, State::Calibration);
                } else if now.wrapping_sub(self.state_entry_ms) > 20_000 {
                    self.send_statustext_udp(io, "Bridge: no rotation, skip calibration");
                    self.enter_state(io, State::NoArm);
                }
            }

            State::Calibration => self.step_calibration(io, now),

            State::CalibrationEnd => {
                if !self.cal_finalized {
                    self.send_accept_mag_cal(io);
                    self.send_preflight_storage(io);
                    self.queue_statustext("calibration OK - power cycle FC");
                    self.send_statustext_udp(io, "Bridge: calibration OK - power cycle FC");
                    self.mdfly = MDFLY_STOP;
                    self.cal_finalized = true;
                }
            }

            State::NoArm => {
                if !self.no_arm_init {
                    self.mission_loaded = false;
                    self.mission_has_land = false;
                    self.send_mission_request_list(io);
                    self.no_arm_init = true;
                }

                if self.gps_fix_type >= 3 && now.wrapping_sub(self.state_entry_ms) >= 5_000 {
                    self.send_arm(io);
                    self.queue_statustext("ARM >>");
                    self.state_entry_ms = now;
                }

                if self.is_armed {
                    self.enter_state(io, State::Arming);
                }
            }

            State::Arming => {
                if !self.is_armed {
                    self.queue_statustext("disarmed -> NO_ARM");
                    self.enter_state(io, State::NoArm);
                } else {
                    let can_auto =
                        self.gps_fix_type >= 3 && (self.ekf_flags & EKF_POS_HORIZ_ABS) != 0;
                    if can_auto
                        && (!self.fsm.auto_sent || now.wrapping_sub(self.state_entry_ms) >= 20_000)
                    {
                        self.send_set_mode(io, MODE_AUTO as u8);
                        self.queue_statustext("AUTO >>");
                        self.fsm.auto_sent = true;
                        self.state_entry_ms = now;
                    }
                    if self.is_armed && self.current_custom_mode == MODE_AUTO {
                        self.enter_state(io, State::Mission);
                    }
                }
            }

            State::Mission => self.step_mission(io, now),

            State::RelayControl | State::Armed | State::StartMission => {}
        }
    }

    fn step_init_mavlink(&mut self, io: &mut dyn Io, now: u32) {
        if !self.heartbeat_received {
            if now.wrapping_sub(self.fsm.last_log_hb) > 2_000 {
                self.fsm.last_log_hb = now;
                io.log("[S] waiting heartbeat...");
            }
            return;
        }
        if !self.ekf_report_received {
            if now.wrapping_sub(self.fsm.last_log_ekf) > 2_000 {
                self.fsm.last_log_ekf = now;
                io.log("[S] waiting EKF report...");
            }
            return;
        }

        let ekf_ok = (self.ekf_flags & EKF_ATTITUDE) != 0;

        if ekf_ok != self.fsm.ekf_ok_prev {
            self.fsm.ekf_ok_prev = ekf_ok;
            if ekf_ok {
                self.fsm.ekf_att_stable_ms = now;
                self.fsm.ekf_queued = false;
            }
            io.log(&format!(
                "[EKF] flags=0x{:04X} att={} mag={:.3}",
                self.ekf_flags, ekf_ok as u8, self.mag_test_ratio
            ));
        }
        if ekf_ok && !self.fsm.ekf_queued && now.wrapping_sub(self.fsm.ekf_att_stable_ms) > 3_000 {
            self.fsm.ekf_queued = true;
            self.send_statustext_udp(io, "Bridge: EKF OK");
        }

        if !ekf_ok {
            if now.wrapping_sub(self.state_entry_ms) > 30_000 {
                if now.wrapping_sub(self.fsm.ekf_timeout_log) > 5_000 {
                    self.fsm.ekf_timeout_log = now;
                    io.log(&format!(
                        "[S] EKF_ATT timeout {} s, forcing MAG_OK",
                        now.wrapping_sub(self.state_entry_ms) / 1000
                    ));
                    self.send_statustext_udp(io, "Bridge: EKF att timeout, skip mag check");
                }
                if now.wrapping_sub(self.state_entry_ms) > 35_000 {
                    self.enter_state(io, State::MagOk);
                }
            }
            return;
        }
        if now.wrapping_sub(self.fsm.ekf_att_stable_ms) < 5_000 {
            return;
        }

        let fc_init_done = self.mag_sensor_ready
            && self.fc_hb_first_ms != 0
            && now.wrapping_sub(self.fc_hb_first_ms) > 20_000;
        if !fc_init_done {
            if now.wrapping_sub(self.fsm.ekf_att_stable_ms) > 60_000 {
                io.log("[S] FC init timeout, forcing MAG_OK");
                self.enter_state(io, State::MagOk);
                return;
            }
            if now.wrapping_sub(self.fsm.init_wait_log) > 5_000 {
                self.fsm.init_wait_log = now;
                io.log("[S] waiting FC init (mag/SYS_STATUS)");
                self.queue_statustext("wait FC init");
            }
            return;
        }

        io.log("[S] EKF_ATT OK -> MAG_OK");
        self.enter_state(io, State::MagOk);
    }

    fn step_calibration(&mut self, io: &mut dyn Io, now: u32) {
        if !self.cal_cmd_sent {
            self.send_start_mag_cal(io);
            self.send_statustext_udp(io, "Bridge: calibration started");
            self.cal_cmd_sent = true;
            self.last_cal_pct = 0;
            self.fsm.cal_accept_sent = false;
            self.cal_fc_failed = false;
            self.fsm.cal_pct_ms = self.state_entry_ms;
            io.log("[CAL] command sent, waiting progress...");
        }

        if self.cal_completion_pct != self.last_cal_pct {
            if self.cal_completion_pct == 100
                || self.cal_completion_pct > self.last_cal_pct + 5
                || self.cal_completion_pct < self.last_cal_pct
            {
                let buf = format!("Cal: {}%", self.cal_completion_pct);
                self.queue_statustext(&buf);
                io.log(&format!("[CAL] progress {}%", self.cal_completion_pct));
            }
            self.last_cal_pct = self.cal_completion_pct;
            self.fsm.cal_pct_ms = now;
        }

        if self.cal_success {
            let dia_ok = self.cal_dia_x != 0.0
                && (1.0 - self.cal_dia_x).abs() <= DIA_TOLERANCE
                && (1.0 - self.cal_dia_y).abs() <= DIA_TOLERANCE
                && (1.0 - self.cal_dia_z).abs() <= DIA_TOLERANCE;
            if !dia_ok {
                io.log("[CAL] SUCCESS but DIA bad -> retry");
                self.queue_statustext("DIA bad - auto retry");
                self.cal_success = false;
                self.cal_fc_failed = true;
            } else {
                io.log("[CAL] SUCCESS -> CALIBRATION_END");
                self.enter_state(io, State::CalibrationEnd);
                return;
            }
        }

        let cal_elapsed = now.wrapping_sub(self.state_entry_ms);
        let progress_stalled =
            self.fsm.cal_pct_ms != 0 && now.wrapping_sub(self.fsm.cal_pct_ms) > 8_000;
        if !self.cal_success
            && self.cal_cmd_sent
            && self.cal_completion_pct >= 95
            && !self.fsm.cal_accept_sent
            && progress_stalled
            && cal_elapsed > 15_000
        {
            self.send_accept_mag_cal(io);
            self.fsm.cal_accept_sent = true;
            self.queue_statustext("Cal 95%+, accepting");
            io.log("[CAL] near-done, sending ACCEPT");
        }

        let fc_failed = self.cal_fc_failed;
        self.cal_fc_failed = false;
        let attempt_timeout = (!self.cal_success && self.cal_cmd_sent && cal_elapsed > 60_000)
            || (progress_stalled && cal_elapsed > 10_000);
        if attempt_timeout || (fc_failed && cal_elapsed > 5_000) {
            self.cal_retries += 1;
            if self.cal_retries < CAL_MAX_RETRIES {
                io.log(&format!(
                    "[CAL] timeout, retry {}/{}",
                    self.cal_retries, CAL_MAX_RETRIES
                ));
                self.cal_cmd_sent = false;
                self.cal_success = false;
                self.state_entry_ms = now;
                self.fsm.cal_accept_sent = false;
                self.fsm.cal_pct_ms = now;
                self.last_cal_pct = 0;
                self.send_statustext_udp(io, "Bridge: calibration retry");
            } else {
                io.log("[CAL] max retries, power cycle required");
                self.send_statustext_udp(io, "Bridge: calibration failed, power cycle");
                self.queue_statustext("Cal failed - power cycle FC");
                self.mdfly = MDFLY_STOP;
                self.enter_state(io, State::NoArm);
            }
        }
    }

    fn step_mission(&mut self, io: &mut dyn Io, now: u32) {
        if !self.mission_start_msg {
            self.queue_statustext("START");
            self.mission_start_msg = true;
        }
        if self.mission_base_alt < 0.0 && self.vfr_alt > 0.5 {
            self.mission_base_alt = self.vfr_alt;
        }
        if self.mission_base_alt >= 0.0 && self.vfr_alt - self.mission_base_alt > 1.0 {
            self.flew_above_1m = true;
        }

        let in_land = self.mission_has_land || self.current_custom_mode == MODE_LAND;

        if in_land && self.landed_state == MAV_LANDED_STATE_ON_GROUND {
            if self.fsm.land_stop_ms == 0 {
                self.fsm.land_stop_ms = now;
            }
        } else {
            self.fsm.land_stop_ms = 0;
        }
        let landed_stopped = self.flew_above_1m
            && in_land
            && self.fsm.land_stop_ms != 0
            && now.wrapping_sub(self.fsm.land_stop_ms) > 5_000;

        let tipped = self.roll_deg.abs() > 30.0 || self.pitch_deg.abs() > 30.0;
        let bounce_attempt = tipped && self.vfr_climb > 0.5;
        if bounce_attempt {
            if self.fsm.tip_bounce_ms == 0 {
                self.fsm.tip_bounce_ms = now;
            }
        } else {
            self.fsm.tip_bounce_ms = 0;
        }
        let tipped_bounce = self.flew_above_1m
            && in_land
            && self.fsm.tip_bounce_ms != 0
            && now.wrapping_sub(self.fsm.tip_bounce_ms) > 400;

        let mission_ended = !self.is_armed;

        if mission_ended || landed_stopped || tipped_bounce || self.crash_triggered {
            if self.flew_above_1m {
                self.send_set_relay(io);
                self.send_force_disarm(io);
                self.queue_statustext("Relay");
            } else {
                self.send_force_disarm(io);
                self.queue_statustext("Disarmed, no relay");
            }
            self.enter_state(io, State::RelayControl);
        }
    }
}

impl Default for Bridge {
    fn default() -> Self {
        Bridge::new(Config::default())
    }
}

#[cfg(test)]
impl Bridge {
    /// Test-only hook to force a state transition.
    pub fn enter_state_pub(&mut self, io: &mut dyn Io, s: State) {
        self.enter_state(io, s);
    }

    /// Test-only helper: does the outgoing STATUS queue contain `needle`?
    pub fn status_queue_contains(&self, needle: &str) -> bool {
        self.status_queue.contains(needle)
    }
}

#[cfg(test)]
mod tests;
