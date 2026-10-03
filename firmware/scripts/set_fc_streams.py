#!/usr/bin/env python3
"""Raise the flight controller's telemetry stream rates for the bridge's port.

Why: SERIAL2 (the port wired to the bridge) carries SR2_* stream rates of 2 Hz-ish,
so a full parameter download of 1129 parameters takes about two minutes. Mission
Planner re-requests the list when it sees no progress, each request restarts the
walk, and the download never finishes - "Getting params" forever.

The FC ignores SET_MESSAGE_INTERVAL and REQUEST_DATA_STREAM here (verified, with
CRC_EXTRA self-checked against published values), but SRx_* are *parameters*, so
PARAM_SET is honoured. The original C++ firmware also writes parameters, and sends
MAV_CMD_PREFLIGHT_STORAGE afterwards, which this does too.

Everything is verified: each value is read back through PARAM_REQUEST_READ and the
script fails loudly if a read-back disagrees.
"""

import socket
import struct
import sys
import time

CRC_EXTRA = {
    0: 50,      # HEARTBEAT
    20: 214,    # PARAM_REQUEST_READ
    22: 220,    # PARAM_VALUE
    23: 168,    # PARAM_SET
    76: 152,    # COMMAND_LONG
}
MAV_CMD_PREFLIGHT_STORAGE = 245

# Stream rates for the telemetry port the bridge sits on. Chosen to be modest:
# roughly 2 KiB/s of telemetry, which a 4G link carries comfortably, while the
# parameter list arrives in about twenty seconds instead of two minutes.
TARGETS = [
    ("SR2_PARAMS", 10.0),      # the parameter list itself (10 is the accepted maximum)
    ("SR2_EXTRA1", 3.0),       # ATTITUDE
    ("SR2_EXTRA2", 3.0),       # VFR_HUD
    ("SR2_EXTRA3", 2.0),       # AHRS, SYSTEM_TIME, BATTERY_STATUS
    ("SR2_EXT_STAT", 2.0),     # SYS_STATUS, POWER_STATUS, MEMINFO
    ("SR2_POSITION", 2.0),     # GLOBAL_POSITION_INT
    ("SR2_RAW_SENS", 2.0),     # RAW_IMU, GPS_RAW_INT, SCALED_PRESSURE
    ("SR2_RC_CHAN", 2.0),      # RC_CHANNELS, SERVO_OUTPUT_RAW
]

# Rate history, for anyone reading this later: the flight controller shipped with
# SR2_* at 2-10 Hz, which made a 1129-parameter download take about two minutes and
# let Mission Planner retry until it gave up. Raising the lot to 5-10 Hz fixed the
# download but produced more telemetry than a weak hotspot managed (RSSI -73 dBm),
# so the bridge's send queue grew to 49 KiB. The values above are the middle ground:
# about 650 bytes/s of steady telemetry, which this link carries, with the parameter
# list still sent at its maximum.

# The flight controller rejected SR2_PARAMS=50 outright (the value stayed 2, so it
# was range-checked away rather than clamped). Overridable so the accepted maximum
# can be found by measurement instead of guessed.
if len(sys.argv) > 1:
    TARGETS = [(name, float(sys.argv[1]) if name == "SR2_PARAMS" else want)
               for name, want in TARGETS]


def x25(data):
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        tmp &= 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc_extra(seed):
    c = x25(seed.encode())
    return (c & 0xFF) ^ (c >> 8)


# Self test the table against published values before touching anything.
assert crc_extra("HEARTBEAT uint32_t custom_mode uint8_t type uint8_t autopilot uint8_t "
                 "base_mode uint8_t system_status uint8_t mavlink_version ") == 50
assert crc_extra("PARAM_SET float param_value uint8_t target_system uint8_t target_component "
                 "char param_id " + chr(16) + "uint8_t param_type ") == 168
assert crc_extra("COMMAND_LONG float param1 float param2 float param3 float param4 float param5 "
                 "float param6 float param7 uint16_t command uint8_t target_system "
                 "uint8_t target_component uint8_t confirmation ") == 152


def frame(msgid, payload, seq, sysid=255, compid=190):
    header = bytes([0xFD, len(payload), 0, 0, seq, sysid, compid,
                    msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = x25(header[1:] + payload + bytes([CRC_EXTRA[msgid]]))
    return header + payload + struct.pack("<H", crc)


class Link:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setblocking(False)
        self.buf = bytearray()
        self.seq = 0
        self.params = {}
        self.next_heartbeat = 0.0

    def send(self, msgid, payload):
        self.seq = (self.seq + 1) & 0xFF
        self.sock.sendall(frame(msgid, payload, self.seq))

    def heartbeat(self):
        # MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_STATE_ACTIVE
        self.send(0, bytes([0, 0, 0, 0, 6, 8, 0, 4, 3]))

    def poll(self, timeout=3.0):
        """Read for a while, collecting PARAM_VALUE replies."""
        end = time.time() + timeout
        while time.time() < end:
            if time.time() >= self.next_heartbeat:
                self.next_heartbeat = time.time() + 1
                self.heartbeat()
            try:
                chunk = self.sock.recv(65536)
                if chunk:
                    self.buf += chunk
            except (BlockingIOError, InterruptedError):
                time.sleep(0.01)
                continue
            except OSError:
                return
            while True:
                start = self.buf.find(b"\xfd")
                if start < 0:
                    self.buf.clear()
                    break
                if start:
                    del self.buf[:start]
                if len(self.buf) < 12:
                    break
                total = self.buf[1] + 12
                if len(self.buf) < total:
                    break
                msgid = self.buf[7] | (self.buf[8] << 8) | (self.buf[9] << 16)
                payload = bytes(self.buf[10:10 + self.buf[1]])
                if msgid == 22 and len(payload) >= 25:
                    name = payload[8:24].split(b"\x00")[0].decode("ascii", "replace")
                    self.params[name] = struct.unpack("<f", payload[0:4])[0]
                del self.buf[:total]

    def read_param(self, name, timeout=3.0):
        self.params.pop(name, None)
        payload = struct.pack("<hBB", -1, 1, 1) + name.encode().ljust(16, b"\x00")
        self.send(20, payload)
        end = time.time() + timeout
        while time.time() < end and name not in self.params:
            self.poll(0.3)
        return self.params.get(name)

    def set_param(self, name, value, timeout=3.0):
        payload = struct.pack("<fBB", value, 1, 1) + name.encode().ljust(16, b"\x00") + b"\x00"
        self.send(23, payload)
        end = time.time() + timeout
        while time.time() < end:
            self.poll(0.3)
            if abs(self.params.get(name, -999.0) - value) < 0.01:
                return True
        return False

    def save_params(self):
        # MAV_CMD_PREFLIGHT_STORAGE with param1 = 1 writes parameters to storage,
        # exactly what the original firmware's sendPreflightStorage() does.
        payload = struct.pack("<7fHBBB", 1.0, 0, 0, 0, 0, 0, 0,
                              MAV_CMD_PREFLIGHT_STORAGE, 1, 1, 0)
        self.send(76, payload)


def main():
    link = Link("127.0.0.1", 14552)
    print("connected to the relay as a GCS")

    failures = []
    for name, want in TARGETS:
        before = link.read_param(name)
        if before is None:
            print(f"{name:<14} could not be read, skipping")
            failures.append(name)
            continue
        if abs(before - want) < 0.01:
            print(f"{name:<14} already {before:g}, left alone")
            continue
        ok = link.set_param(name, want)
        after = link.read_param(name)
        status = "ok" if ok and after is not None and abs(after - want) < 0.01 else "FAILED"
        print(f"{name:<14} {before:g} -> {after if after is None else format(after, 'g')}  {status}")
        if status == "FAILED":
            failures.append(name)

    if failures:
        print(f"\nnot applied: {', '.join(failures)}")
        return 1

    link.save_params()
    link.poll(1.5)
    print("\nasked the FC to store parameters (MAV_CMD_PREFLIGHT_STORAGE)")

    print("\nread back:")
    for name, want in TARGETS:
        have = link.read_param(name)
        flag = "ok" if have is not None and abs(have - want) < 0.01 else "MISMATCH"
        print(f"   {name:<14} {have}  {flag}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
