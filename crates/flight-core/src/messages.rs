//! Typed MAVLink message definitions plus pack/unpack helpers for every
//! message the bridge sends or interprets.
//!
//! All field orders below are the **wire order** produced by `mavgen`'s
//! "sort fields by decreasing primitive size, stably" rule. The corresponding
//! schemas live in [`defs`] and are the single source of truth for CRC_EXTRA.

use crate::consts::*;
use crate::mavlink::frame::{Reader, Writer};

/// Message id constants.
pub mod id {
    pub const HEARTBEAT: u32 = 0;
    pub const SYS_STATUS: u32 = 1;
    pub const GPS_RAW_INT: u32 = 24;
    pub const RAW_IMU: u32 = 27;
    pub const ATTITUDE: u32 = 30;
    pub const MISSION_REQUEST_LIST: u32 = 43;
    pub const MISSION_COUNT: u32 = 44;
    pub const MISSION_ACK: u32 = 47;
    pub const MISSION_ITEM_INT: u32 = 73;
    pub const VFR_HUD: u32 = 74;
    pub const COMMAND_LONG: u32 = 76;
    pub const MAG_CAL_PROGRESS: u32 = 191;
    pub const MAG_CAL_REPORT: u32 = 192;
    pub const EKF_STATUS_REPORT: u32 = 193;
    pub const EXTENDED_SYS_STATE: u32 = 245;
    pub const STATUSTEXT: u32 = 253;
}

/// Message schemas (field layouts used for CRC_EXTRA computation).
pub mod defs {
    use crate::mavlink::schema::{array, ext, scalar, MsgDef, Ty::*};

    pub const HEARTBEAT: MsgDef = MsgDef {
        id: 0,
        name: "HEARTBEAT",
        fields: &[
            scalar("type", U8),
            scalar("autopilot", U8),
            scalar("base_mode", U8),
            scalar("custom_mode", U32),
            scalar("system_status", U8),
            scalar("mavlink_version", U8),
        ],
    };

    pub const SYS_STATUS: MsgDef = MsgDef {
        id: 1,
        name: "SYS_STATUS",
        fields: &[
            scalar("onboard_control_sensors_present", U32),
            scalar("onboard_control_sensors_enabled", U32),
            scalar("onboard_control_sensors_health", U32),
            scalar("load", U16),
            scalar("voltage_battery", U16),
            scalar("current_battery", I16),
            scalar("battery_remaining", I8),
            scalar("drop_rate_comm", U16),
            scalar("errors_comm", U16),
            scalar("errors_count1", U16),
            scalar("errors_count2", U16),
            scalar("errors_count3", U16),
            scalar("errors_count4", U16),
            ext("onboard_control_sensors_present_extended", U32),
            ext("onboard_control_sensors_enabled_extended", U32),
            ext("onboard_control_sensors_health_extended", U32),
        ],
    };

    pub const GPS_RAW_INT: MsgDef = MsgDef {
        id: 24,
        name: "GPS_RAW_INT",
        fields: &[
            scalar("time_usec", U64),
            scalar("fix_type", U8),
            scalar("lat", I32),
            scalar("lon", I32),
            scalar("alt", I32),
            scalar("eph", U16),
            scalar("epv", U16),
            scalar("vel", U16),
            scalar("cog", U16),
            scalar("satellites_visible", U8),
            ext("alt_ellipsoid", I32),
            ext("h_acc", U32),
            ext("v_acc", U32),
            ext("vel_acc", U32),
            ext("hdg_acc", U32),
            ext("yaw", U16),
        ],
    };

    pub const RAW_IMU: MsgDef = MsgDef {
        id: 27,
        name: "RAW_IMU",
        fields: &[
            scalar("time_usec", U64),
            scalar("xacc", I16),
            scalar("yacc", I16),
            scalar("zacc", I16),
            scalar("xgyro", I16),
            scalar("ygyro", I16),
            scalar("zgyro", I16),
            scalar("xmag", I16),
            scalar("ymag", I16),
            scalar("zmag", I16),
            ext("id", U8),
            ext("temperature", I16),
        ],
    };

    pub const ATTITUDE: MsgDef = MsgDef {
        id: 30,
        name: "ATTITUDE",
        fields: &[
            scalar("time_boot_ms", U32),
            scalar("roll", F32),
            scalar("pitch", F32),
            scalar("yaw", F32),
            scalar("rollspeed", F32),
            scalar("pitchspeed", F32),
            scalar("yawspeed", F32),
        ],
    };

    pub const MISSION_REQUEST_LIST: MsgDef = MsgDef {
        id: 43,
        name: "MISSION_REQUEST_LIST",
        fields: &[
            scalar("target_system", U8),
            scalar("target_component", U8),
            ext("mission_type", U8),
        ],
    };

    pub const MISSION_COUNT: MsgDef = MsgDef {
        id: 44,
        name: "MISSION_COUNT",
        fields: &[
            scalar("target_system", U8),
            scalar("target_component", U8),
            scalar("count", U16),
            ext("mission_type", U8),
        ],
    };

    pub const MISSION_ACK: MsgDef = MsgDef {
        id: 47,
        name: "MISSION_ACK",
        fields: &[
            scalar("target_system", U8),
            scalar("target_component", U8),
            scalar("type", U8),
            ext("mission_type", U8),
            ext("opaque_id", U32),
        ],
    };

    pub const MISSION_ITEM_INT: MsgDef = MsgDef {
        id: 73,
        name: "MISSION_ITEM_INT",
        fields: &[
            scalar("target_system", U8),
            scalar("target_component", U8),
            scalar("seq", U16),
            scalar("frame", U8),
            scalar("command", U16),
            scalar("current", U8),
            scalar("autocontinue", U8),
            scalar("param1", F32),
            scalar("param2", F32),
            scalar("param3", F32),
            scalar("param4", F32),
            scalar("x", I32),
            scalar("y", I32),
            scalar("z", F32),
            ext("mission_type", U8),
        ],
    };

    pub const VFR_HUD: MsgDef = MsgDef {
        id: 74,
        name: "VFR_HUD",
        fields: &[
            scalar("airspeed", F32),
            scalar("groundspeed", F32),
            scalar("heading", I16),
            scalar("throttle", U16),
            scalar("alt", F32),
            scalar("climb", F32),
        ],
    };

    pub const COMMAND_LONG: MsgDef = MsgDef {
        id: 76,
        name: "COMMAND_LONG",
        fields: &[
            scalar("target_system", U8),
            scalar("target_component", U8),
            scalar("command", U16),
            scalar("confirmation", U8),
            scalar("param1", F32),
            scalar("param2", F32),
            scalar("param3", F32),
            scalar("param4", F32),
            scalar("param5", F32),
            scalar("param6", F32),
            scalar("param7", F32),
        ],
    };

    pub const MAG_CAL_PROGRESS: MsgDef = MsgDef {
        id: 191,
        name: "MAG_CAL_PROGRESS",
        fields: &[
            scalar("compass_id", U8),
            scalar("cal_mask", U8),
            scalar("cal_status", U8),
            scalar("attempt", U8),
            scalar("completion_pct", U8),
            array("completion_mask", U8, 10),
            scalar("direction_x", F32),
            scalar("direction_y", F32),
            scalar("direction_z", F32),
        ],
    };

    pub const MAG_CAL_REPORT: MsgDef = MsgDef {
        id: 192,
        name: "MAG_CAL_REPORT",
        fields: &[
            scalar("compass_id", U8),
            scalar("cal_mask", U8),
            scalar("cal_status", U8),
            scalar("autosaved", U8),
            scalar("fitness", F32),
            scalar("ofs_x", F32),
            scalar("ofs_y", F32),
            scalar("ofs_z", F32),
            scalar("diag_x", F32),
            scalar("diag_y", F32),
            scalar("diag_z", F32),
            scalar("offdiag_x", F32),
            scalar("offdiag_y", F32),
            scalar("offdiag_z", F32),
            ext("orientation_confidence", F32),
            ext("old_orientation", U8),
            ext("new_orientation", U8),
            ext("scale_factor", F32),
        ],
    };

    pub const EKF_STATUS_REPORT: MsgDef = MsgDef {
        id: 193,
        name: "EKF_STATUS_REPORT",
        fields: &[
            scalar("flags", U16),
            scalar("velocity_variance", F32),
            scalar("pos_horiz_variance", F32),
            scalar("pos_vert_variance", F32),
            scalar("compass_variance", F32),
            scalar("terrain_alt_variance", F32),
            ext("airspeed_variance", F32),
        ],
    };

    pub const EXTENDED_SYS_STATE: MsgDef = MsgDef {
        id: 245,
        name: "EXTENDED_SYS_STATE",
        fields: &[scalar("vtol_state", U8), scalar("landed_state", U8)],
    };

    pub const STATUSTEXT: MsgDef = MsgDef {
        id: 253,
        name: "STATUSTEXT",
        fields: &[
            scalar("severity", U8),
            array("text", Char, 50),
            ext("id", U16),
            ext("chunk_seq", U8),
        ],
    };

    /// Every schema the bridge knows about.
    pub const ALL: &[&MsgDef] = &[
        &HEARTBEAT,
        &SYS_STATUS,
        &GPS_RAW_INT,
        &RAW_IMU,
        &ATTITUDE,
        &MISSION_REQUEST_LIST,
        &MISSION_COUNT,
        &MISSION_ACK,
        &MISSION_ITEM_INT,
        &VFR_HUD,
        &COMMAND_LONG,
        &MAG_CAL_PROGRESS,
        &MAG_CAL_REPORT,
        &EKF_STATUS_REPORT,
        &EXTENDED_SYS_STATE,
        &STATUSTEXT,
    ];

    /// Look up a schema by message id.
    pub fn find(msgid: u32) -> Option<&'static MsgDef> {
        ALL.iter().copied().find(|d| d.id == msgid)
    }
}

// ---------------------------------------------------------------------------
// Typed message structs
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Heartbeat {
    pub custom_mode: u32,
    pub typ: u8,
    pub autopilot: u8,
    pub base_mode: u8,
    pub system_status: u8,
    pub mavlink_version: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Statustext {
    pub severity: u8,
    pub text: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    pub time_boot_ms: u32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SysStatus {
    pub onboard_control_sensors_present: u32,
    pub onboard_control_sensors_enabled: u32,
    pub onboard_control_sensors_health: u32,
    pub load: u16,
    pub voltage_battery: u16,
    pub current_battery: i16,
    pub battery_remaining: i8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VfrHud {
    pub airspeed: f32,
    pub groundspeed: f32,
    pub heading: i16,
    pub throttle: u16,
    pub alt: f32,
    pub climb: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtendedSysState {
    pub vtol_state: u8,
    pub landed_state: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawImu {
    pub time_usec: u64,
    pub xacc: i16,
    pub yacc: i16,
    pub zacc: i16,
    pub xgyro: i16,
    pub ygyro: i16,
    pub zgyro: i16,
    pub xmag: i16,
    pub ymag: i16,
    pub zmag: i16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsRawInt {
    pub time_usec: u64,
    pub fix_type: u8,
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub satellites_visible: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MagCalProgress {
    pub compass_id: u8,
    pub cal_mask: u8,
    pub cal_status: u8,
    pub attempt: u8,
    pub completion_pct: u8,
    pub direction_x: f32,
    pub direction_y: f32,
    pub direction_z: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagCalReport {
    pub compass_id: u8,
    pub cal_mask: u8,
    pub cal_status: u8,
    pub autosaved: u8,
    pub fitness: f32,
    pub ofs_x: f32,
    pub ofs_y: f32,
    pub ofs_z: f32,
    pub diag_x: f32,
    pub diag_y: f32,
    pub diag_z: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EkfStatusReport {
    pub flags: u16,
    pub velocity_variance: f32,
    pub pos_horiz_variance: f32,
    pub pos_vert_variance: f32,
    pub compass_variance: f32,
    pub terrain_alt_variance: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionCount {
    pub target_system: u8,
    pub target_component: u8,
    pub count: u16,
    pub mission_type: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionItemInt {
    pub target_system: u8,
    pub target_component: u8,
    pub seq: u16,
    pub frame: u8,
    pub command: u16,
    pub current: u8,
    pub autocontinue: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionAck {
    pub target_system: u8,
    pub target_component: u8,
    pub ack_type: u8,
    pub mission_type: u8,
}

/// A decoded message we care about. Anything else becomes [`Message::Other`].
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    Heartbeat(Heartbeat),
    Statustext(Statustext),
    Attitude(Attitude),
    SysStatus(SysStatus),
    VfrHud(VfrHud),
    ExtendedSysState(ExtendedSysState),
    RawImu(RawImu),
    GpsRawInt(GpsRawInt),
    MagCalProgress(MagCalProgress),
    MagCalReport(MagCalReport),
    EkfStatusReport(EkfStatusReport),
    MissionCount(MissionCount),
    MissionItemInt(MissionItemInt),
    MissionAck(MissionAck),
    Other(u32),
}

// ---------------------------------------------------------------------------
// Decoders
// ---------------------------------------------------------------------------

impl Heartbeat {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        Heartbeat {
            custom_mode: r.u32(),
            typ: r.u8(),
            autopilot: r.u8(),
            base_mode: r.u8(),
            system_status: r.u8(),
            mavlink_version: r.u8(),
        }
    }
    /// Build a MAVLink payload (wire order) - used by `send_heartbeat`.
    pub fn payload(&self) -> Vec<u8> {
        let mut w = Writer::new();
        w.u32(self.custom_mode);
        w.u8(self.typ);
        w.u8(self.autopilot);
        w.u8(self.base_mode);
        w.u8(self.system_status);
        w.u8(self.mavlink_version);
        w.into_vec()
    }
}

impl Statustext {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let severity = r.u8();
        let text = r.char_array(50);
        Statustext { severity, text }
    }
    pub fn payload(&self) -> Vec<u8> {
        let mut w = Writer::new();
        w.u8(self.severity);
        w.char_array(&self.text, 50);
        w.into_vec()
    }
}

impl Attitude {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        Attitude {
            time_boot_ms: r.u32(),
            roll: r.f32(),
            pitch: r.f32(),
            yaw: r.f32(),
        }
    }
}

impl SysStatus {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let present = r.u32();
        let enabled = r.u32();
        let health = r.u32();
        let load = r.u16();
        let voltage_battery = r.u16();
        let current_battery = r.i16();
        let drop_rate_comm = r.u16();
        let errors_comm = r.u16();
        let _c1 = r.u16();
        let _c2 = r.u16();
        let _c3 = r.u16();
        let _c4 = r.u16();
        let battery_remaining = r.i8();
        let _ = drop_rate_comm;
        let _ = errors_comm;
        SysStatus {
            onboard_control_sensors_present: present,
            onboard_control_sensors_enabled: enabled,
            onboard_control_sensors_health: health,
            load,
            voltage_battery,
            current_battery,
            battery_remaining,
        }
    }
}

impl VfrHud {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        VfrHud {
            airspeed: r.f32(),
            groundspeed: r.f32(),
            alt: r.f32(),
            climb: r.f32(),
            heading: r.i16(),
            throttle: r.u16(),
        }
    }
}

impl ExtendedSysState {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        ExtendedSysState {
            vtol_state: r.u8(),
            landed_state: r.u8(),
        }
    }
}

impl RawImu {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        RawImu {
            time_usec: r.u64(),
            xacc: r.i16(),
            yacc: r.i16(),
            zacc: r.i16(),
            xgyro: r.i16(),
            ygyro: r.i16(),
            zgyro: r.i16(),
            xmag: r.i16(),
            ymag: r.i16(),
            zmag: r.i16(),
        }
    }
}

impl GpsRawInt {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let time_usec = r.u64();
        let lat = r.i32();
        let lon = r.i32();
        let alt = r.i32();
        let _eph = r.u16();
        let _epv = r.u16();
        let _vel = r.u16();
        let _cog = r.u16();
        let fix_type = r.u8();
        let satellites_visible = r.u8();
        GpsRawInt {
            time_usec,
            fix_type,
            lat,
            lon,
            alt,
            satellites_visible,
        }
    }
}

impl MagCalProgress {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let direction_x = r.f32();
        let direction_y = r.f32();
        let direction_z = r.f32();
        let compass_id = r.u8();
        let cal_mask = r.u8();
        let cal_status = r.u8();
        let attempt = r.u8();
        let completion_pct = r.u8();
        r.skip(10); // completion_mask
        MagCalProgress {
            compass_id,
            cal_mask,
            cal_status,
            attempt,
            completion_pct,
            direction_x,
            direction_y,
            direction_z,
        }
    }
}

impl MagCalReport {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let fitness = r.f32();
        let ofs_x = r.f32();
        let ofs_y = r.f32();
        let ofs_z = r.f32();
        let diag_x = r.f32();
        let diag_y = r.f32();
        let diag_z = r.f32();
        let _offdiag = (r.f32(), r.f32(), r.f32());
        let compass_id = r.u8();
        let cal_mask = r.u8();
        let cal_status = r.u8();
        let autosaved = r.u8();
        MagCalReport {
            compass_id,
            cal_mask,
            cal_status,
            autosaved,
            fitness,
            ofs_x,
            ofs_y,
            ofs_z,
            diag_x,
            diag_y,
            diag_z,
        }
    }
}

impl EkfStatusReport {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let velocity_variance = r.f32();
        let pos_horiz_variance = r.f32();
        let pos_vert_variance = r.f32();
        let compass_variance = r.f32();
        let terrain_alt_variance = r.f32();
        let flags = r.u16();
        EkfStatusReport {
            flags,
            velocity_variance,
            pos_horiz_variance,
            pos_vert_variance,
            compass_variance,
            terrain_alt_variance,
        }
    }
}

impl MissionCount {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let count = r.u16();
        let target_system = r.u8();
        let target_component = r.u8();
        let mission_type = r.u8();
        MissionCount {
            target_system,
            target_component,
            count,
            mission_type,
        }
    }
}

impl MissionItemInt {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        let _params = (r.f32(), r.f32(), r.f32(), r.f32());
        let _x = r.i32();
        let _y = r.i32();
        let _z = r.f32();
        let seq = r.u16();
        let command = r.u16();
        let target_system = r.u8();
        let target_component = r.u8();
        let frame = r.u8();
        let current = r.u8();
        let autocontinue = r.u8();
        MissionItemInt {
            target_system,
            target_component,
            seq,
            frame,
            command,
            current,
            autocontinue,
        }
    }
}

impl MissionAck {
    pub fn decode(p: &[u8]) -> Self {
        let mut r = Reader::new(p);
        MissionAck {
            target_system: r.u8(),
            target_component: r.u8(),
            ack_type: r.u8(),
            mission_type: r.u8(),
        }
    }
}

impl Message {
    /// Decode a raw payload according to its message id.
    pub fn decode(msgid: u32, payload: &[u8]) -> Message {
        match msgid {
            id::HEARTBEAT => Message::Heartbeat(Heartbeat::decode(payload)),
            id::SYS_STATUS => Message::SysStatus(SysStatus::decode(payload)),
            id::GPS_RAW_INT => Message::GpsRawInt(GpsRawInt::decode(payload)),
            id::RAW_IMU => Message::RawImu(RawImu::decode(payload)),
            id::ATTITUDE => Message::Attitude(Attitude::decode(payload)),
            id::MISSION_COUNT => Message::MissionCount(MissionCount::decode(payload)),
            id::MISSION_ACK => Message::MissionAck(MissionAck::decode(payload)),
            id::MISSION_ITEM_INT => Message::MissionItemInt(MissionItemInt::decode(payload)),
            id::VFR_HUD => Message::VfrHud(VfrHud::decode(payload)),
            id::MAG_CAL_PROGRESS => Message::MagCalProgress(MagCalProgress::decode(payload)),
            id::MAG_CAL_REPORT => Message::MagCalReport(MagCalReport::decode(payload)),
            id::EKF_STATUS_REPORT => Message::EkfStatusReport(EkfStatusReport::decode(payload)),
            id::EXTENDED_SYS_STATE => Message::ExtendedSysState(ExtendedSysState::decode(payload)),
            id::STATUSTEXT => Message::Statustext(Statustext::decode(payload)),
            other => Message::Other(other),
        }
    }

    /// True for the two magnetometer-calibration messages the bridge never
    /// forwards (they would flood the 4G link).
    pub fn is_calibration_spam(&self) -> bool {
        matches!(self, Message::MagCalReport(_) | Message::MagCalProgress(_))
    }
}

// ---------------------------------------------------------------------------
// Outgoing command builders (mirroring mavlink_util.cpp)
// ---------------------------------------------------------------------------

/// `mavlink_msg_command_long_pack(...)`
#[allow(clippy::too_many_arguments)]
pub fn command_long_payload(
    target_system: u8,
    target_component: u8,
    command: u16,
    confirmation: u8,
    p1: f32,
    p2: f32,
    p3: f32,
    p4: f32,
    p5: f32,
    p6: f32,
    p7: f32,
) -> Vec<u8> {
    let mut w = Writer::new();
    // wire order: params (f32), command (u16), targets/confirmation (u8)
    w.f32(p1);
    w.f32(p2);
    w.f32(p3);
    w.f32(p4);
    w.f32(p5);
    w.f32(p6);
    w.f32(p7);
    w.u16(command);
    w.u8(target_system);
    w.u8(target_component);
    w.u8(confirmation);
    w.into_vec()
}

/// `mavlink_msg_mission_request_list_pack(...)`
pub fn mission_request_list_payload(
    target_system: u8,
    target_component: u8,
    _mission_type: u8,
) -> Vec<u8> {
    let mut w = Writer::new();
    w.u8(target_system);
    w.u8(target_component);
    w.into_vec()
}

/// ARM / DISARM helper.
pub fn arm_disarm_payload(arm: bool, force: bool) -> Vec<u8> {
    command_long_payload(
        MAV_COMP_ID_AUTOPILOT1,
        MAV_COMP_ID_AUTOPILOT1,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        if arm { 1.0 } else { 0.0 },
        if force { 21196.0 } else { 0.0 },
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
}

/// `sendMavlinkSetMode(mode)` helper.
pub fn set_mode_payload(mode: u8) -> Vec<u8> {
    command_long_payload(
        1,
        1,
        MAV_CMD_DO_SET_MODE,
        0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED as f32,
        mode as f32,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
}

/// `sendMavlinkSetRelay()` helper.
pub fn set_relay_payload() -> Vec<u8> {
    command_long_payload(
        1,
        1,
        MAV_CMD_DO_SET_RELAY,
        0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
}

/// `sendStartMagCal()` helper.
pub fn start_mag_cal_payload() -> Vec<u8> {
    command_long_payload(
        1,
        1,
        MAV_CMD_DO_START_MAG_CAL,
        0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
}

/// `sendAcceptMagCal()` helper.
pub fn accept_mag_cal_payload() -> Vec<u8> {
    command_long_payload(
        1,
        1,
        MAV_CMD_DO_ACCEPT_MAG_CAL,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
}

/// `sendPreflightStorage()` helper.
pub fn preflight_storage_payload() -> Vec<u8> {
    command_long_payload(
        1,
        1,
        MAV_CMD_PREFLIGHT_STORAGE,
        0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
}

/// Convenience: the base payload length for a message id, if known.
pub fn base_len(id: u32) -> Option<usize> {
    defs::find(id).map(|d| d.base_len())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_pack_decode_roundtrip() {
        let hb = Heartbeat {
            custom_mode: 3,
            typ: MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot: MAV_AUTOPILOT_INVALID,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 0,
        };
        let p = hb.payload();
        assert_eq!(p.len(), defs::HEARTBEAT.base_len());
        assert_eq!(Heartbeat::decode(&p), hb);
    }

    #[test]
    fn command_long_layout_matches_schema() {
        let p = command_long_payload(1, 1, 400, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(p.len(), defs::COMMAND_LONG.base_len());
        // param1 is the first little-endian f32
        assert_eq!(&p[..4], &1.0f32.to_le_bytes());
        // command (u16) sits right after the 7 params
        assert_eq!(u16::from_le_bytes([p[28], p[29]]), 400);
    }

    #[test]
    fn statustext_payload_and_decode() {
        let st = Statustext {
            severity: MAV_SEVERITY_INFO,
            text: "Bridge Ready".to_string(),
        };
        let p = st.payload();
        assert_eq!(p.len(), 51);
        let back = Statustext::decode(&p);
        assert_eq!(back.severity, MAV_SEVERITY_INFO);
        assert_eq!(back.text, "Bridge Ready");
    }

    #[test]
    fn vfr_hud_wire_order() {
        // alt/climb come before heading/throttle in the wire layout.
        let mut w = Writer::new();
        w.f32(1.0); // airspeed
        w.f32(2.0); // groundspeed
        w.f32(10.0); // alt
        w.f32(-1.5); // climb
        w.i16(180); // heading
        w.u16(50); // throttle
        let p = w.into_vec();
        assert_eq!(p.len(), defs::VFR_HUD.base_len());
        let v = VfrHud::decode(&p);
        assert_eq!(v.alt, 10.0);
        assert_eq!(v.climb, -1.5);
        assert_eq!(v.heading, 180);
        assert_eq!(v.throttle, 50);
    }

    #[test]
    fn sys_status_decode_offsets() {
        let mut w = Writer::new();
        w.u32(0x1 | 0x2 | 0x4); // present
        w.u32(0x4); // enabled
        w.u32(0x4); // health
        w.u16(100); // load
        w.u16(12_600); // voltage (12.6 V)
        w.i16(-1); // current
        w.u16(0);
        w.u16(0);
        w.u16(0);
        w.u16(0);
        w.u16(0);
        w.u16(0);
        w.i8(77); // battery_remaining
        let p = w.into_vec();
        let s = SysStatus::decode(&p);
        assert_eq!(s.voltage_battery, 12_600);
        assert_eq!(s.battery_remaining, 77);
        assert_eq!(
            s.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG,
            4
        );
    }

    #[test]
    fn ekf_status_report_layout() {
        let mut w = Writer::new();
        w.f32(0.1);
        w.f32(0.2);
        w.f32(0.3);
        w.f32(0.9); // compass_variance (this is `mag_test_ratio`)
        w.f32(0.5);
        w.u16(EKF_ATTITUDE | EKF_POS_HORIZ_ABS);
        let p = w.into_vec();
        assert_eq!(p.len(), defs::EKF_STATUS_REPORT.base_len());
        let e = EkfStatusReport::decode(&p);
        assert_eq!(e.flags, EKF_ATTITUDE | EKF_POS_HORIZ_ABS);
        assert!((e.compass_variance - 0.9).abs() < 1e-6);
    }

    #[test]
    fn mag_cal_report_layout() {
        let mut w = Writer::new();
        w.f32(1.0); // fitness
        w.f32(1.2); // ofs_x
        w.f32(2.3); // ofs_y
        w.f32(3.4); // ofs_z
        w.f32(1.01); // diag_x
        w.f32(0.99); // diag_y
        w.f32(1.05); // diag_z
        w.f32(0.0);
        w.f32(0.0);
        w.f32(0.0);
        w.u8(0); // compass_id
        w.u8(1); // cal_mask
        w.u8(4); // cal_status = SUCCESS
        w.u8(1); // autosaved
        let p = w.into_vec();
        assert_eq!(p.len(), defs::MAG_CAL_REPORT.base_len());
        let r = MagCalReport::decode(&p);
        assert_eq!(r.cal_status, 4);
        assert!((r.diag_x - 1.01).abs() < 1e-6);
        assert!((r.ofs_z - 3.4).abs() < 1e-6);
    }
}
