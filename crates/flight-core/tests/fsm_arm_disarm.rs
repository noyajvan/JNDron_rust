//! Contract tests that **lock** the ARM/DISARM logic and the state machine.
//!
//! These are the tests to read if you are about to touch `update_system_state`
//! or the arming paths: they pin down every outgoing command and every state
//! edge so an accidental change fails loudly.
//!
//! Everything is driven through real MAVLink 2 input, exactly like a flight
//! controller would produce it.

use flight_core::consts::*;
use flight_core::io::MockIo;
use flight_core::mavlink::{encode_v2, Parser, Reader};
use flight_core::messages::{defs, Heartbeat};
use flight_core::{Bridge, Config, State};

// ---------------------------------------------------------------------------
// Frame building / decoding helpers
// ---------------------------------------------------------------------------

fn frame(msgid: u32, payload: &[u8]) -> Vec<u8> {
    let extra = defs::find(msgid).map(|d| d.crc_extra());
    encode_v2(0, 1, 1, msgid, payload, extra)
}

fn hb_frame(custom_mode: u32, armed: bool) -> Vec<u8> {
    let hb = Heartbeat {
        custom_mode,
        typ: 2,       // MAV_TYPE_QUADROTOR
        autopilot: 3, // ArduPilot
        base_mode: if armed { MAV_MODE_FLAG_SAFETY_ARMED } else { 0 },
        system_status: 4,
        mavlink_version: 3,
    };
    frame(0, &hb.payload())
}

fn ekf_frame(flags: u16) -> Vec<u8> {
    let mut payload = Vec::new();
    for _ in 0..5 {
        payload.extend_from_slice(&0.1f32.to_le_bytes());
    }
    payload.extend_from_slice(&flags.to_le_bytes());
    frame(193, &payload)
}

fn mag_ok_sys_status_frame() -> Vec<u8> {
    let mag = MAV_SYS_STATUS_SENSOR_3D_MAG;
    let mut p = Vec::new();
    p.extend_from_slice(&mag.to_le_bytes());
    p.extend_from_slice(&mag.to_le_bytes());
    p.extend_from_slice(&mag.to_le_bytes());
    p.extend_from_slice(&0u16.to_le_bytes()); // load
    p.extend_from_slice(&12_100u16.to_le_bytes()); // voltage
    p.extend_from_slice(&(-1i16).to_le_bytes()); // current
    for _ in 0..6 {
        p.extend_from_slice(&0u16.to_le_bytes());
    }
    p.push(100); // battery_remaining
    frame(1, &p)
}

fn gps_frame(fix_type: u8) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&0u64.to_le_bytes()); // time_usec
    p.extend_from_slice(&0i32.to_le_bytes()); // lat
    p.extend_from_slice(&0i32.to_le_bytes()); // lon
    p.extend_from_slice(&0i32.to_le_bytes()); // alt
    for _ in 0..4 {
        p.extend_from_slice(&0u16.to_le_bytes()); // eph/epv/vel/cog
    }
    p.push(fix_type);
    p.push(12); // satellites
    frame(24, &p)
}

fn vfr_frame(alt: f32, groundspeed: f32, throttle: u16, climb: f32) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&0.0f32.to_le_bytes()); // airspeed
    p.extend_from_slice(&groundspeed.to_le_bytes());
    p.extend_from_slice(&alt.to_le_bytes());
    p.extend_from_slice(&climb.to_le_bytes());
    p.extend_from_slice(&0i16.to_le_bytes()); // heading
    p.extend_from_slice(&throttle.to_le_bytes());
    frame(74, &p)
}

/// A decoded COMMAND_LONG.
#[derive(Debug, Clone, PartialEq)]
struct Cmd {
    id: u16,
    params: [f32; 7],
    target_system: u8,
    target_component: u8,
}

/// Extract every COMMAND_LONG the bridge wrote to the FC.
fn commands(bytes: &[u8]) -> Vec<Cmd> {
    let mut parser = Parser::new();
    let mut frames = Vec::new();
    parser.push_slice(bytes, &mut frames);

    frames
        .iter()
        .filter(|f| f.msgid == 76)
        .map(|f| {
            let mut r = Reader::new(&f.payload);
            let mut params = [0f32; 7];
            for p in params.iter_mut() {
                *p = r.f32();
            }
            let id = r.u16();
            let target_system = r.u8();
            let target_component = r.u8();
            Cmd {
                id,
                params,
                target_system,
                target_component,
            }
        })
        .collect()
}

fn only_cmd(bytes: &[u8], id: u16) -> Cmd {
    let all = commands(bytes);
    let mut hits: Vec<Cmd> = all.into_iter().filter(|c| c.id == id).collect();
    assert_eq!(hits.len(), 1, "expected exactly one cmd {id}");
    hits.pop().unwrap()
}

fn no_cmds(bytes: &[u8], ids: &[u16]) {
    for c in commands(bytes) {
        assert!(!ids.contains(&c.id), "unexpected command {} was sent", c.id);
    }
}

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

/// Drive the bridge from boot to `STATE_NO_ARM` using only protocol input.
fn drive_to_no_arm(b: &mut Bridge, io: &mut MockIo) {
    io.now = 4_000; // INIT_WIFI -> INIT_MAVLINK after 2 s
    b.tick(io);

    b.feed_fc_bytes(io, &hb_frame(MODE_STABILIZE, false));
    b.feed_fc_bytes(io, &ekf_frame(EKF_ATTITUDE));
    b.feed_fc_bytes(io, &mag_ok_sys_status_frame());

    io.now = 30_000;
    b.tick(io);
    io.now = 36_000;
    b.tick(io);
    assert_eq!(b.state, State::MagOk, "EKF gate must open MAG_OK");

    // 20 s in the blue window without a >45 deg rotation -> skip calibration.
    io.now += 20_001;
    b.tick(io);
    assert_eq!(b.state, State::NoArm);
}

/// Drive from `STATE_NO_ARM` into `STATE_MISSION` (armed, AUTO, GPS + EKF pos).
fn drive_to_mission(b: &mut Bridge, io: &mut MockIo) {
    b.feed_fc_bytes(io, &gps_frame(3));
    b.feed_fc_bytes(io, &ekf_frame(EKF_ATTITUDE | EKF_POS_HORIZ_ABS));
    b.feed_fc_bytes(io, &hb_frame(MODE_AUTO, true));

    io.now += 5_000;
    b.tick(io); // NO_ARM -> ARMING
    assert_eq!(b.state, State::Arming);
    b.tick(io); // ARMING -> MISSION (sends AUTO first)
    assert_eq!(b.state, State::Mission);
}

// ---------------------------------------------------------------------------
// ARM
// ---------------------------------------------------------------------------

#[test]
fn arm_is_withheld_until_gps_fix_is_3d() {
    let (mut b, mut io) = booted();
    drive_to_no_arm(&mut b, &mut io);

    // Plenty of time in NO_ARM, but no GPS fix.
    io.now += 10_000;
    io.take_fc();
    b.tick(&mut io);
    assert_eq!(b.state, State::NoArm);
    no_cmds(&io.take_fc(), &[MAV_CMD_COMPONENT_ARM_DISARM]);

    // 3D fix arrives -> ARM on the next iteration.
    b.feed_fc_bytes(&mut io, &gps_frame(3));
    io.take_fc();
    b.tick(&mut io);
    let arm = only_cmd(&io.take_fc(), MAV_CMD_COMPONENT_ARM_DISARM);
    assert_eq!(arm.params[0], 1.0, "ARM = param1 1.0");
    assert_eq!(arm.params[1], 0.0, "ARM must not be forced");
    assert_eq!(arm.target_system, 1, "the FC's system id");
    assert_eq!(arm.target_component, MAV_COMP_ID_AUTOPILOT1);
}

#[test]
fn arm_is_retried_every_5_seconds() {
    let (mut b, mut io) = booted();
    drive_to_no_arm(&mut b, &mut io);
    b.feed_fc_bytes(&mut io, &gps_frame(3));

    io.now += 5_000;
    io.take_fc();
    b.tick(&mut io);
    only_cmd(&io.take_fc(), MAV_CMD_COMPONENT_ARM_DISARM);

    // Immediate retry must not happen.
    io.take_fc();
    b.tick(&mut io);
    no_cmds(&io.take_fc(), &[MAV_CMD_COMPONENT_ARM_DISARM]);

    // 5 s later it does.
    io.now += 5_000;
    io.take_fc();
    b.tick(&mut io);
    only_cmd(&io.take_fc(), MAV_CMD_COMPONENT_ARM_DISARM);
}

#[test]
fn arming_never_sends_arm_itself() {
    let (mut b, mut io) = booted();
    drive_to_no_arm(&mut b, &mut io);
    b.feed_fc_bytes(&mut io, &gps_frame(3));
    b.feed_fc_bytes(&mut io, &ekf_frame(EKF_ATTITUDE | EKF_POS_HORIZ_ABS));
    b.feed_fc_bytes(&mut io, &hb_frame(MODE_STABILIZE, true));

    io.now += 5_000;
    b.tick(&mut io);
    assert_eq!(b.state, State::Arming);

    io.take_fc();
    b.tick(&mut io);
    no_cmds(&io.take_fc(), &[MAV_CMD_COMPONENT_ARM_DISARM]);
}

// ---------------------------------------------------------------------------
// AUTO / mode change
// ---------------------------------------------------------------------------

#[test]
fn auto_is_sent_only_with_gps_fix_and_ekf_position() {
    let (mut b, mut io) = booted();
    drive_to_no_arm(&mut b, &mut io);

    // Armed, GPS fix, but no EKF horizontal position -> no AUTO.
    b.feed_fc_bytes(&mut io, &gps_frame(3));
    b.feed_fc_bytes(&mut io, &ekf_frame(EKF_ATTITUDE));
    b.feed_fc_bytes(&mut io, &hb_frame(MODE_STABILIZE, true));
    io.now += 5_000;
    b.tick(&mut io); // -> ARMING
    io.take_fc();
    b.tick(&mut io);
    no_cmds(&io.take_fc(), &[MAV_CMD_DO_SET_MODE]);
    assert_eq!(b.state, State::Arming);

    // EKF position becomes valid -> AUTO is sent next iteration.
    b.feed_fc_bytes(&mut io, &ekf_frame(EKF_ATTITUDE | EKF_POS_HORIZ_ABS));
    io.take_fc();
    b.tick(&mut io);
    let mode = only_cmd(&io.take_fc(), MAV_CMD_DO_SET_MODE);
    assert_eq!(mode.params[0], MAV_MODE_FLAG_CUSTOM_MODE_ENABLED as f32);
    assert_eq!(mode.params[1], MODE_AUTO as f32);
}

#[test]
fn disarming_while_arming_returns_to_no_arm_without_commands() {
    let (mut b, mut io) = booted();
    drive_to_no_arm(&mut b, &mut io);
    b.feed_fc_bytes(&mut io, &gps_frame(3));
    b.feed_fc_bytes(&mut io, &hb_frame(MODE_STABILIZE, true));
    io.now += 5_000;
    b.tick(&mut io);
    assert_eq!(b.state, State::Arming);

    b.feed_fc_bytes(&mut io, &hb_frame(MODE_STABILIZE, false));
    io.take_fc();
    b.tick(&mut io);
    assert_eq!(b.state, State::NoArm);
    no_cmds(
        &io.take_fc(),
        &[
            MAV_CMD_COMPONENT_ARM_DISARM,
            MAV_CMD_DO_SET_RELAY,
            MAV_CMD_DO_SET_MODE,
        ],
    );
}

// ---------------------------------------------------------------------------
// Mission end: force disarm + relay
// ---------------------------------------------------------------------------

#[test]
fn mission_end_after_a_real_flight_forces_disarm_and_fires_the_relay() {
    let (mut b, mut io) = booted();
    drive_to_no_arm(&mut b, &mut io);
    drive_to_mission(&mut b, &mut io);

    // Establish the altitude base, then climb above 1 m over it.
    b.feed_fc_bytes(&mut io, &vfr_frame(10.0, 5.0, 50, 0.0));
    b.tick(&mut io);
    b.feed_fc_bytes(&mut io, &vfr_frame(20.0, 5.0, 50, 0.0));
    b.tick(&mut io);
    assert!(b.flew_above_1m);

    // Flight controller disarms (mission finished).
    b.feed_fc_bytes(&mut io, &hb_frame(MODE_AUTO, false));
    io.take_fc();
    b.tick(&mut io);

    assert_eq!(b.state, State::RelayControl);
    let cmds = io.take_fc();
    let relay = only_cmd(&cmds, MAV_CMD_DO_SET_RELAY);
    assert_eq!(relay.params[1], 1.0, "relay channel 0 on");

    let disarm = only_cmd(&cmds, MAV_CMD_COMPONENT_ARM_DISARM);
    assert_eq!(disarm.params[0], 0.0, "DISARM = param1 0.0");
    assert_eq!(disarm.params[1], 21196.0, "DISARM is forced");
}

#[test]
fn mission_end_without_flying_disarms_but_does_not_fire_the_relay() {
    let (mut b, mut io) = booted();
    drive_to_no_arm(&mut b, &mut io);
    drive_to_mission(&mut b, &mut io);

    // Never flew above the base altitude.
    assert!(!b.flew_above_1m);

    b.feed_fc_bytes(&mut io, &hb_frame(MODE_AUTO, false));
    io.take_fc();
    b.tick(&mut io);

    assert_eq!(b.state, State::RelayControl);
    let cmds = io.take_fc();
    no_cmds(&cmds, &[MAV_CMD_DO_SET_RELAY]);
    let disarm = only_cmd(&cmds, MAV_CMD_COMPONENT_ARM_DISARM);
    assert_eq!(disarm.params[1], 21196.0);
}

// ---------------------------------------------------------------------------
// The state sequence itself
// ---------------------------------------------------------------------------

#[test]
fn state_sequence_boot_to_mission_uses_the_documented_edges() {
    let (mut b, mut io) = booted();
    assert_eq!(b.state, State::InitWifi);

    io.now = 4_000;
    b.tick(&mut io);
    assert_eq!(b.state, State::InitMavlink);

    b.feed_fc_bytes(&mut io, &hb_frame(MODE_STABILIZE, false));
    b.feed_fc_bytes(&mut io, &ekf_frame(EKF_ATTITUDE));
    b.feed_fc_bytes(&mut io, &mag_ok_sys_status_frame());
    io.now = 30_000;
    b.tick(&mut io);
    io.now = 36_000;
    b.tick(&mut io);
    assert_eq!(b.state, State::MagOk);

    io.now += 20_001;
    b.tick(&mut io);
    assert_eq!(b.state, State::NoArm);

    drive_to_mission(&mut b, &mut io);
    assert_eq!(b.state, State::Mission);
}

#[test]
fn non_stabilize_non_auto_mode_stops_the_flight_loop() {
    // `mdfly = 60` freezes the whole FSM - this is the guard the pilot guide
    // relies on, so it must not regress.
    let (mut b, mut io) = booted();
    b.feed_fc_bytes(&mut io, &hb_frame(MODE_ACRO, false));
    io.now = 4_000;
    b.tick(&mut io);
    assert_eq!(b.mdfly, MDFLY_STOP);

    let before = b.state;
    io.now += 60_000;
    b.tick(&mut io);
    assert_eq!(b.state, before, "the FSM must stay frozen while mdfly = 60");
}
