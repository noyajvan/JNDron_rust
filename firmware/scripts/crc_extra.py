#!/usr/bin/env python3
"""Print MAVLink CRC_EXTRA values, computed the way mavgen does.

Used to keep the probes' tables honest: a wrong CRC_EXTRA makes the flight
controller silently drop a request, which looks exactly like the FC ignoring a
command. Self-tested against published values first.
"""


def x25(data):
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        tmp &= 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def extra(seed):
    c = x25(seed.encode())
    return (c & 0xFF) ^ (c >> 8)


KNOWN = {
    "HEARTBEAT uint32_t custom_mode uint8_t type uint8_t autopilot uint8_t base_mode "
    "uint8_t system_status uint8_t mavlink_version ": 50,
    "ATTITUDE uint32_t time_boot_ms float roll float pitch float yaw float rollspeed "
    "float pitchspeed float yawspeed ": 39,
    "VFR_HUD float airspeed float groundspeed float alt float climb int16_t heading "
    "uint16_t throttle ": 20,
    "PARAM_VALUE float param_value uint16_t param_count uint16_t param_index char param_id "
    + chr(16) + "uint8_t param_type ": 220,
    "STATUSTEXT uint8_t severity char text " + chr(50): 83,
}
for seed, want in KNOWN.items():
    got = extra(seed)
    mark = "ok" if got == want else f"MISMATCH (expected {want})"
    print(f"  {seed.split()[0]:<14} {got:>3}  {mark}")

print("values needed by the probes:")
for name, seed in {
    "PARAM_REQUEST_LIST": "PARAM_REQUEST_LIST uint8_t target_system uint8_t target_component ",
    "PARAM_REQUEST_READ": "PARAM_REQUEST_READ int16_t param_index uint8_t target_system "
                          "uint8_t target_component char param_id " + chr(16),
    "SET_MESSAGE_INTERVAL": "SET_MESSAGE_INTERVAL uint32_t interval_us uint16_t message_id ",
    "REQUEST_DATA_STREAM": "REQUEST_DATA_STREAM uint8_t target_system uint8_t target_component "
                           "uint8_t req_stream_id uint16_t req_message_rate uint8_t start_stop ",
    "MISSION_REQUEST_LIST": "MISSION_REQUEST_LIST uint8_t target_system uint8_t target_component "
                            "uint8_t mission_type ",
    "COMMAND_LONG": "COMMAND_LONG float param1 float param2 float param3 float param4 float param5 "
                    "float param6 float param7 uint16_t command uint8_t target_system "
                    "uint8_t target_component uint8_t confirmation ",
}.items():
    print(f"  {name:<22} {extra(seed)}")
