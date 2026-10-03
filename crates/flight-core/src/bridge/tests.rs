//! End-to-end tests for the bridge: FSM progression, crash->relay, transport
//! forwarding, spam suppression and the console commands.

use super::*;
use crate::io::MockIo;
use crate::mavlink::{encode_v2, Writer};
use crate::messages::defs;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn frame(msgid: u32, payload: &[u8]) -> Vec<u8> {
    let extra = defs::find(msgid).map(|d| d.crc_extra());
    encode_v2(0, 1, 1, msgid, payload, extra)
}

fn hb_payload(custom_mode: u32, armed: bool) -> Vec<u8> {
    let hb = Heartbeat {
        custom_mode,
        typ: 2,       // MAV_TYPE_QUADROTOR
        autopilot: 3, // MAV_AUTOPILOT_ARDUPILOTMEGA
        base_mode: if armed { MAV_MODE_FLAG_SAFETY_ARMED } else { 0 },
        system_status: 4,
        mavlink_version: 3,
    };
    hb.payload()
}

fn ekf_payload(flags: u16, compass_variance: f32) -> Vec<u8> {
    let mut w = Writer::new();
    w.f32(0.1);
    w.f32(0.1);
    w.f32(0.1);
    w.f32(compass_variance);
    w.f32(0.1);
    w.u16(flags);
    w.into_vec()
}

fn sys_status_payload(mag_present: bool, mag_enabled: bool, mag_healthy: bool) -> Vec<u8> {
    let mut w = Writer::new();
    let mag = MAV_SYS_STATUS_SENSOR_3D_MAG;
    let bit = |on: bool| if on { mag } else { 0 };
    w.u32(bit(mag_present));
    w.u32(bit(mag_enabled));
    w.u32(bit(mag_healthy));
    w.u16(0); // load
    w.u16(12_100); // voltage
    w.i16(-1); // current
    w.u16(0); // drop_rate_comm
    w.u16(0); // errors_comm
    w.u16(0);
    w.u16(0);
    w.u16(0);
    w.u16(0);
    w.i8(100); // battery_remaining
    w.into_vec()
}

fn attitude_payload(roll_deg: f32, pitch_deg: f32) -> Vec<u8> {
    let mut w = Writer::new();
    w.u32(0);
    w.f32(roll_deg / RAD_TO_DEG);
    w.f32(pitch_deg / RAD_TO_DEG);
    w.f32(0.0);
    w.into_vec()
}

fn vfr_payload(alt: f32, groundspeed: f32, throttle: u16, climb: f32) -> Vec<u8> {
    let mut w = Writer::new();
    w.f32(0.0); // airspeed
    w.f32(groundspeed);
    w.f32(alt);
    w.f32(climb);
    w.i16(0); // heading
    w.u16(throttle);
    w.into_vec()
}

fn mag_cal_report_payload(status: u8, diag: f32) -> Vec<u8> {
    let mut w = Writer::new();
    w.f32(1.0); // fitness
    w.f32(0.0); // ofs_x
    w.f32(0.0);
    w.f32(0.0);
    w.f32(diag); // diag_x
    w.f32(diag); // diag_y
    w.f32(diag); // diag_z
    w.f32(0.0);
    w.f32(0.0);
    w.f32(0.0);
    w.u8(0); // compass_id
    w.u8(1); // cal_mask
    w.u8(status); // cal_status
    w.u8(1); // autosaved
    w.into_vec()
}

fn mag_cal_progress_payload(status: u8, pct: u8) -> Vec<u8> {
    let mut w = Writer::new();
    w.f32(0.0);
    w.f32(0.0);
    w.f32(0.0);
    w.u8(0); // compass_id
    w.u8(1); // cal_mask
    w.u8(status); // cal_status
    w.u8(0); // attempt
    w.u8(pct); // completion_pct
    w.bytes(&[0u8; 10]); // completion_mask
    w.into_vec()
}

/// A bridge booted with Wi-Fi up and the GCS connected over TCP.
fn booted() -> (Bridge, MockIo) {
    let mut io = MockIo::new();
    io.now = 1_000;
    io.connected = true;
    io.tcp = true;
    let cfg = Config {
        sta_ssid: "LEO".into(),
        sta_pass: "88888888".into(),
        ..Config::default()
    };
    let mut b = Bridge::new(cfg);
    b.boot(&mut io);
    (b, io)
}

// ---------------------------------------------------------------------------
// Heartbeat / telemetry decoding
// ---------------------------------------------------------------------------

#[test]
fn heartbeat_updates_flags() {
    let (mut b, mut io) = booted();
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::HEARTBEAT, &hb_payload(MODE_AUTO, true)),
    );
    assert!(b.heartbeat_received);
    assert!(b.is_armed);
    assert_eq!(b.current_custom_mode, MODE_AUTO);
    // "Mavlink OK" was queued on the first heartbeat.
    assert_eq!(b.status_queue.len(), 2); // "Bridge Ready" + "Mavlink OK"
}

#[test]
fn warning_statustext_is_queued_info_is_not() {
    let (mut b, mut io) = booted();
    let warn = Statustext {
        severity: MAV_SEVERITY_WARNING,
        text: "EKF variance".into(),
    };
    let info = Statustext {
        severity: MAV_SEVERITY_INFO,
        text: "hello".into(),
    };
    b.feed_fc_bytes(&mut io, &frame(messages::id::STATUSTEXT, &warn.payload()));
    b.feed_fc_bytes(&mut io, &frame(messages::id::STATUSTEXT, &info.payload()));
    assert_eq!(b.reason_queue.len(), 1);
    assert_eq!(b.reason_queue.pop().as_deref(), Some("EKF variance"));
}

#[test]
fn gps_and_sys_status_decode() {
    let (mut b, mut io) = booted();
    b.feed_fc_bytes(
        &mut io,
        &frame(
            messages::id::SYS_STATUS,
            &sys_status_payload(true, true, true),
        ),
    );
    assert!(b.mag_sensor_ready);
    assert!((b.battery_voltage - 12.1).abs() < 1e-6);
    assert_eq!(b.battery_remaining, 100);

    let mut w = Writer::new();
    w.u64(0);
    w.i32(0);
    w.i32(0);
    w.i32(0);
    w.u16(0);
    w.u16(0);
    w.u16(0);
    w.u16(0);
    w.u8(3); // fix_type = 3D
    w.u8(11); // satellites
    b.feed_fc_bytes(&mut io, &frame(messages::id::GPS_RAW_INT, &w.into_vec()));
    assert_eq!(b.gps_fix_type, 3);
    assert_eq!(b.gps_sats, 11);
}

// ---------------------------------------------------------------------------
// Transport / forwarding
// ---------------------------------------------------------------------------

#[test]
fn fc_telemetry_is_forwarded_over_tcp() {
    let (mut b, mut io) = booted();
    io.take_net();
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::HEARTBEAT, &hb_payload(MODE_STABILIZE, false)),
    );
    assert!(
        !io.tcp_out.is_empty(),
        "heartbeat must be relayed to the GCS"
    );
}

#[test]
fn calibration_spam_is_not_forwarded() {
    let (mut b, mut io) = booted();
    io.take_net();
    b.feed_fc_bytes(
        &mut io,
        &frame(
            messages::id::MAG_CAL_REPORT,
            &mag_cal_report_payload(4, 1.0),
        ),
    );
    assert!(io.take_net().is_empty(), "MAG_CAL_REPORT must be swallowed");
    assert!(b.cal_success);
}

#[test]
fn gcs_bytes_are_forwarded_to_fc_and_mark_server() {
    let (mut b, mut io) = booted();
    io.take_fc();
    b.feed_gcs_bytes(&mut io, &[0x01, 0x02, 0x03]);
    assert_eq!(io.take_fc(), vec![1, 2, 3]);
    assert!(b.has_server);
}

// ---------------------------------------------------------------------------
// Parameter download: a list request must not restart a walk already in progress
// ---------------------------------------------------------------------------

/// `PARAM_VALUE` payload: value(f32), count(u16), index(u16), name[16], type(u8).
fn param_value_payload(count: u16, index: u16) -> Vec<u8> {
    let mut w = Writer::new();
    w.f32(1.0);
    w.u16(count);
    w.u16(index);
    w.char_array("TEST_PARAM", 16);
    w.u8(9);
    w.into_vec()
}

#[test]
fn param_request_is_forwarded_when_no_walk_is_running() {
    let (mut b, mut io) = booted();
    io.take_fc();
    let request = frame(21, &[1, 1]);
    b.feed_gcs_bytes(&mut io, &request);
    assert_eq!(io.take_fc(), request);
}

#[test]
fn param_request_is_held_while_the_walk_is_running() {
    let (mut b, mut io) = booted();
    // The flight controller is part-way through a walk of 1129 parameters.
    b.feed_fc_bytes(&mut io, &frame(22, &param_value_payload(1129, 40)));
    io.take_fc();
    io.take_net();

    b.feed_gcs_bytes(&mut io, &frame(21, &[1, 1]));

    assert!(
        io.take_fc().is_empty(),
        "a list request arriving mid-walk would restart the download"
    );
}

#[test]
fn held_param_request_is_replayed_when_the_walk_stalls() {
    let (mut b, mut io) = booted();
    b.feed_fc_bytes(&mut io, &frame(22, &param_value_payload(1129, 40)));
    io.take_fc();

    let request = frame(21, &[1, 1]);
    b.feed_gcs_bytes(&mut io, &request);
    assert!(io.take_fc().is_empty());

    // The walk goes quiet. The held request has to go out unchanged, otherwise a
    // download that died could never be restarted. (`tick` also sends its own
    // heartbeat, so look for the request among what was written.)
    io.now += PARAM_WALK_STALL_MS + 1;
    b.tick(&mut io);
    let written = io.take_fc();
    assert!(
        written.windows(request.len()).any(|w| w == request.as_slice()),
        "held request was not replayed"
    );
}

#[test]
fn param_request_is_forwarded_again_once_the_walk_completed() {
    let (mut b, mut io) = booted();
    b.feed_fc_bytes(&mut io, &frame(22, &param_value_payload(50, 49)));
    io.take_fc();

    let request = frame(21, &[1, 1]);
    b.feed_gcs_bytes(&mut io, &request);
    assert_eq!(io.take_fc(), request);
}

#[test]
fn gcs_stream_is_reassembled_across_reads() {
    let (mut b, mut io) = booted();
    io.take_fc();
    let request = frame(21, &[1, 1]);

    // A frame split over two reads must not be forwarded twice or damaged.
    let (head, tail) = request.split_at(7);
    b.feed_gcs_bytes(&mut io, head);
    assert!(io.take_fc().is_empty(), "half a frame is not forwarded");
    b.feed_gcs_bytes(&mut io, tail);
    assert_eq!(io.take_fc(), request);
}

#[test]
fn mavlink1_frames_are_ignored_by_the_v2_only_parser() {
    let (mut b, mut io) = booted();
    io.take_net();

    // A MAVLink 1 HEARTBEAT frame: 0xFE magic, 6-byte header, 9-byte payload.
    let mut v1 = vec![0xFE, 9, 3, 1, 1, messages::id::HEARTBEAT as u8];
    v1.extend_from_slice(&hb_payload(MODE_STABILIZE, false));
    v1.extend_from_slice(&[0u8, 0u8]); // bogus checksum; never even looked at

    b.feed_fc_bytes(&mut io, &v1);
    assert!(io.take_net().is_empty(), "MAVLink 1 must not be relayed");
    assert_eq!(b.fc_msgs, 0, "MAVLink 1 must not be decoded");

    // A following MAVLink 2 frame is still picked up.
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::HEARTBEAT, &hb_payload(MODE_STABILIZE, false)),
    );
    assert_eq!(b.fc_msgs, 1);
    let net = io.take_net();
    assert_eq!(net[0], 0xFD, "telemetry is always relayed as MAVLink 2");
}

#[test]
fn udp_fallback_sends_twice() {
    let (mut b, mut io) = booted();
    io.tcp = false;
    io.take_net();
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::HEARTBEAT, &hb_payload(MODE_STABILIZE, false)),
    );
    // Two UDP copies of the same datagram.
    let net = io.take_net();
    assert!(!net.is_empty());
    let half = net.len() / 2;
    assert_eq!(&net[..half], &net[half..]);
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

/// Drive the bridge from boot all the way to `MAG_OK`.
fn drive_to_mag_ok(b: &mut Bridge, io: &mut MockIo) {
    // INIT_WIFI -> INIT_MAVLINK after 2 s.
    io.now = 4_000;
    b.tick(io);

    b.feed_fc_bytes(
        io,
        &frame(messages::id::HEARTBEAT, &hb_payload(MODE_STABILIZE, false)),
    );
    b.feed_fc_bytes(
        io,
        &frame(
            messages::id::EKF_STATUS_REPORT,
            &ekf_payload(EKF_ATTITUDE, 0.2),
        ),
    );
    b.feed_fc_bytes(
        io,
        &frame(
            messages::id::SYS_STATUS,
            &sys_status_payload(true, true, true),
        ),
    );

    // Need EKF stable > 5 s AND FC heartbeat older than 20 s.
    io.now = 30_000;
    b.tick(io);
    io.now = 36_000;
    b.tick(io);
}

#[test]
fn fsm_boots_to_mag_ok() {
    let (mut b, mut io) = booted();
    drive_to_mag_ok(&mut b, &mut io);
    assert_eq!(b.state, State::MagOk);
}

#[test]
fn rotation_triggers_calibration_then_success() {
    let (mut b, mut io) = booted();
    drive_to_mag_ok(&mut b, &mut io);

    // Rotate > 45 deg -> blue window detects the flip.
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::ATTITUDE, &attitude_payload(60.0, 0.0)),
    );
    b.tick(&mut io);
    assert_eq!(b.state, State::Calibration);

    // The magenta calibration command is sent on the next iteration.
    b.tick(&mut io);
    assert!(b.cal_cmd_sent);

    // FC reports progress complete and a good report.
    b.feed_fc_bytes(
        &mut io,
        &frame(
            messages::id::MAG_CAL_PROGRESS,
            &mag_cal_progress_payload(4, 100),
        ),
    );
    b.feed_fc_bytes(
        &mut io,
        &frame(
            messages::id::MAG_CAL_REPORT,
            &mag_cal_report_payload(4, 1.02),
        ),
    );
    b.tick(&mut io);
    assert_eq!(b.state, State::CalibrationEnd);

    b.tick(&mut io);
    // Calibration finished -> flight loop disabled until a power cycle.
    assert_eq!(b.mdfly, MDFLY_STOP);
}

#[test]
fn calibration_with_bad_diag_is_retried() {
    let (mut b, mut io) = booted();
    drive_to_mag_ok(&mut b, &mut io);
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::ATTITUDE, &attitude_payload(60.0, 0.0)),
    );
    b.tick(&mut io);
    assert_eq!(b.state, State::Calibration);
    b.tick(&mut io); // send the calibration command
    assert!(b.cal_cmd_sent);

    // diag 1.9 is way outside 0.85..1.15 -> not accepted.
    b.feed_fc_bytes(
        &mut io,
        &frame(
            messages::id::MAG_CAL_PROGRESS,
            &mag_cal_progress_payload(4, 100),
        ),
    );
    b.feed_fc_bytes(
        &mut io,
        &frame(
            messages::id::MAG_CAL_REPORT,
            &mag_cal_report_payload(4, 1.9),
        ),
    );
    assert!(b.cal_success);
    // This tick consumes the bad report and marks the attempt failed.
    b.tick(&mut io);
    assert_eq!(b.state, State::Calibration);
    assert!(!b.cal_success);

    // ~11 s later the stalled attempt times out and is retried.
    io.now += 11_000;
    b.tick(&mut io);
    assert_eq!(b.cal_retries, 1);
    assert!(!b.cal_cmd_sent);
}

#[test]
fn non_stabilize_mode_stops_the_flight_loop() {
    let (mut b, mut io) = booted();
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::HEARTBEAT, &hb_payload(MODE_ACRO, false)),
    );
    b.tick(&mut io);
    assert_eq!(b.mdfly, MDFLY_STOP);
}

#[test]
fn mission_end_after_landing_fires_the_relay() {
    let (mut b, mut io) = booted();
    // Jump straight into MISSION.
    b.enter_state_pub(&mut io, State::Mission);
    b.is_armed = true;
    b.current_custom_mode = MODE_AUTO;

    // Fly above 1 m over the base altitude.
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::VFR_HUD, &vfr_payload(10.0, 5.0, 50, 0.0)),
    );
    b.tick(&mut io);
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::VFR_HUD, &vfr_payload(20.0, 5.0, 50, 0.0)),
    );
    b.tick(&mut io);
    assert!(b.flew_above_1m);

    // NAV_LAND in the mission + ON_GROUND for > 5 s.
    let mut mi = Writer::new();
    mi.f32(0.0);
    mi.f32(0.0);
    mi.f32(0.0);
    mi.f32(0.0);
    mi.i32(0);
    mi.i32(0);
    mi.f32(0.0);
    mi.u16(0); // seq
    mi.u16(MAV_CMD_NAV_LAND);
    mi.u8(1);
    mi.u8(1);
    mi.u8(3);
    mi.u8(0);
    mi.u8(1);
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::MISSION_ITEM_INT, &mi.into_vec()),
    );
    assert!(b.mission_has_land);

    b.feed_fc_bytes(
        &mut io,
        &frame(
            messages::id::EXTENDED_SYS_STATE,
            &[0, MAV_LANDED_STATE_ON_GROUND],
        ),
    );
    b.tick(&mut io); // starts the 5 s on-ground timer
    io.now += 6_000;
    b.tick(&mut io);

    assert_eq!(b.state, State::RelayControl);
    assert!(b.status_queue_contains("Relay"));
}

#[test]
fn crash_triggers_relay_only_if_it_flew() {
    let (mut b, mut io) = booted();
    b.enter_state_pub(&mut io, State::Mission);
    b.is_armed = true;
    b.current_custom_mode = MODE_AUTO;

    // Fly above 1 m first, so the crash gate is armed.
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::VFR_HUD, &vfr_payload(10.0, 5.0, 50, 0.0)),
    );
    b.tick(&mut io); // fixes mission_base_alt = 10
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::VFR_HUD, &vfr_payload(20.0, 5.0, 50, 0.0)),
    );

    // Then a hard descent for > 500 ms.
    io.now += 100;
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::VFR_HUD, &vfr_payload(18.0, 5.0, 50, -8.0)),
    );
    io.now += 700;
    b.feed_fc_bytes(
        &mut io,
        &frame(messages::id::VFR_HUD, &vfr_payload(12.0, 5.0, 50, -8.0)),
    );
    assert!(b.crash_triggered);

    b.tick(&mut io);
    assert_eq!(b.state, State::RelayControl);
    assert!(b.status_queue_contains("Relay"));
}

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

#[test]
fn console_configures_wifi_and_persists() {
    let (mut b, mut io) = booted();
    b.run_command(&mut io, "SSID=MyNet");
    b.run_command(&mut io, "PASS=secret");
    b.run_command(&mut io, "BAUD=57600");
    b.run_command(&mut io, "SYSID=7");
    assert_eq!(b.cfg.sta_ssid, "MyNet");
    assert_eq!(b.cfg.sta_pass, "secret");
    assert_eq!(b.cfg.baud, 57_600);
    assert_eq!(b.cfg.sys_id, 7);

    b.run_command(&mut io, "SAVE");
    assert_eq!(io.restarts, 1);
    assert_eq!(io.saved.as_ref().unwrap().sta_ssid, "MyNet");
}

#[test]
fn console_wifi_off_on_and_relay_disarm() {
    let (mut b, mut io) = booted();
    b.run_command(&mut io, "WIFI OFF");
    assert!(!b.wifi_on);
    assert_eq!(io.deactivated, 1);

    b.run_command(&mut io, "WIFI ON");
    assert!(b.wifi_on);
    assert_eq!(io.activated, 2); // once by boot(), once here

    io.take_fc();
    b.run_command(&mut io, "RELAY");
    assert!(!io.take_fc().is_empty());

    io.take_fc();
    b.run_command(&mut io, "DISARM");
    let cmd = io.take_fc();
    // COMMAND_LONG payload carries param2 = 21196 (force disarm) somewhere.
    assert!(cmd
        .windows(4)
        .any(|w| f32::from_le_bytes([w[0], w[1], w[2], w[3]]) == 21196.0));
}

#[test]
fn console_status_prints_and_rejects_junk() {
    let (mut b, mut io) = booted();
    b.run_command(&mut io, "STATUS");
    assert!(io.logs.iter().any(|l| l == "--- STATUS ---"));
    b.run_command(&mut io, "nonsense");
    assert!(io.logs.iter().any(|l| l.starts_with("CMD: STATUS")));
}

#[test]
fn console_char_echoes_and_prompts() {
    let (mut b, mut io) = booted();
    io.logs.clear();
    for c in "RELAY".chars() {
        b.terminal_char(&mut io, c);
    }
    b.terminal_char(&mut io, '\n');
    assert!(io.logs.iter().any(|l| l == "> "));
    assert_eq!(b.input_buffer, "");
}
