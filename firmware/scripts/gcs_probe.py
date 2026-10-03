#!/usr/bin/env python3
"""Act as a GCS against the local relay and follow one parameter download.

Runs on the VPS so the client is not the bottleneck: the Windows PowerShell
probe parses too slowly to tell a slow flight controller from a slow client.

Prints the order in which parameter indices arrive, which shows directly whether
the FC walks the list once or keeps restarting it.
"""

import socket
import struct
import sys
import time

CRC_EXTRA = {
    0: 50,      # HEARTBEAT
    20: 214,    # PARAM_REQUEST_READ
    21: 159,    # PARAM_REQUEST_LIST
    22: 220,    # PARAM_VALUE
    30: 39,     # ATTITUDE
    43: 132,    # MISSION_REQUEST_LIST (mission_type is an extension, excluded)
    66: 193,    # REQUEST_DATA_STREAM
    74: 20,     # VFR_HUD
    193: 82,    # SET_MESSAGE_INTERVAL
    253: 83,    # STATUSTEXT
}

# Indices asked for individually, to test whether the parameter table itself is
# reachable when the flight controller refuses to walk the whole list.
SONDE_INDEXES = (500, 1000)


def x25(data):
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        tmp &= 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def frame(msgid, payload, seq, sysid=255, compid=190):
    header = bytes([0xFD, len(payload), 0, 0, seq, sysid, compid,
                    msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = x25(header[1:] + payload + bytes([CRC_EXTRA.get(msgid, 0)]))
    return header + payload + struct.pack("<H", crc)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    # Optional: ask the FC for ATTITUDE at 10 Hz, to test whether it applies
    # SET_MESSAGE_INTERVAL at all (Mission Planner keeps re-sending those).
    set_interval = "-set" in sys.argv
    old_way = "-stream" in sys.argv
    mission = "-mission" in sys.argv
    relists = "-relists" in sys.argv
    s = socket.create_connection(("127.0.0.1", 14552), timeout=5)
    s.setblocking(False)
    s.sendall(frame(21, bytes([1, 1]), 0))
    print("sent PARAM_REQUEST_LIST")

    if set_interval:
        # SET_MESSAGE_INTERVAL(193): interval_us(u32) message_id(u16)
        s.sendall(frame(193, struct.pack("<IH", 100_000, 30), 9))
        print("sent SET_MESSAGE_INTERVAL(ATTITUDE, 10 Hz)")

    if old_way:
        # REQUEST_DATA_STREAM(66): req_message_rate(u16) target_system target_component
        # req_stream_id start_stop - stream 0 is MAV_DATA_STREAM_ALL.
        s.sendall(frame(66, struct.pack("<HBBBB", 10, 1, 1, 0, 1), 10))
        print("sent REQUEST_DATA_STREAM(ALL, 10 Hz)")

    # PARAM_REQUEST_READ (20): param_index(i16) target_system target_component
    # param_id[16] - wire order puts param_index first, then the byte fields.
    def read_index(index):
        payload = struct.pack("<hBB", index, 1, 1) + b"\x00" * 16
        return frame(20, payload, index & 0xFF)

    sonde_at = {time.time() + 3 + 3 * i: idx for i, idx in enumerate(SONDE_INDEXES)}
    sonde_sent = set()
    found = set()

    # Mission requests are the other thing Mission Planner sends on connect, and the
    # bridge itself asks for the mission periodically. ArduPilot serves one list at a
    # time, so this tests whether the parameter walk survives them.
    mission_next = time.time() + 3 if mission else None
    mission_seq = 0

    # Mission Planner re-sends PARAM_REQUEST_LIST while its dialog sees no progress.
    # If a fresh request restarts the FC's walk, a slow walk can never finish.
    relist_next = time.time() + 5 if relists else None
    relist_seq = 20

    buf = bytearray()
    order = []
    seen = set()
    counts = {}
    names = {}
    # (param_index, sequence) -> how many times that exact pair arrived. A pair
    # seen more than once means the path duplicated a frame; distinct sequences
    # for one index mean the flight controller really sent it again.
    pairs = {}
    param_count = 0
    hb_seq = 1
    next_hb = time.time()
    deadline = time.time() + seconds
    last_new = time.time()

    while time.time() < deadline:
        if relist_next is not None and time.time() >= relist_next:
            relist_next = time.time() + 5
            relist_seq = (relist_seq + 1) & 0xFF
            s.sendall(frame(21, bytes([1, 1]), relist_seq))
        if mission_next is not None and time.time() >= mission_next:
            mission_next = time.time() + 3
            mission_seq = (mission_seq + 1) & 0xFF
            # MISSION_REQUEST_LIST(43): target_system target_component mission_type
            s.sendall(frame(43, bytes([1, 1, 0]), mission_seq))
        for when, index in list(sonde_at.items()):
            if time.time() >= when:
                s.sendall(read_index(index))
                sonde_sent.add(index)
                print(f"sent PARAM_REQUEST_READ for index {index}")
                del sonde_at[when]
        if time.time() >= next_hb:
            next_hb = time.time() + 1
            try:
                # HEARTBEAT: custom_mode(u32) type autopilot base_mode status version
                s.sendall(frame(0, bytes([0, 0, 0, 0, 6, 8, 0, 4, 3]), hb_seq))
                hb_seq = (hb_seq + 1) & 0xFF
            except OSError:
                pass
        try:
            chunk = s.recv(65536)
            if chunk:
                buf += chunk
        except (BlockingIOError, InterruptedError):
            time.sleep(0.005)
            continue
        except OSError:
            break

        while True:
            start = buf.find(b"\xfd")
            if start < 0:
                buf.clear()
                break
            if start:
                del buf[:start]
            if len(buf) < 12:
                break
            total = buf[1] + 12
            if len(buf) < total:
                break
            msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            payload = bytes(buf[10:10 + buf[1]])
            counts[msgid] = counts.get(msgid, 0) + 1
            if msgid == 22 and len(payload) >= 8:
                idx = payload[6] | (payload[7] << 8)
                key = (idx, buf[4])
                pairs[key] = pairs.get(key, 0) + 1
                param_count = payload[4] | (payload[5] << 8) or param_count
                if idx in sonde_sent:
                    found.add(idx)
                if idx not in seen:
                    seen.add(idx)
                    order.append(idx)
                    names[idx] = (payload[8:24].split(b"\x00")[0].decode("ascii", "replace"),
                                  struct.unpack("<f", payload[0:4])[0])
                    last_new = time.time()
            del buf[:total]

        if order and time.time() - last_new > 10:
            print(f"no new parameter for 10s, stopping at index {order[-1]}")
            break

    print(f"param_count reported : {param_count}")
    print(f"distinct parameters  : {len(seen)}")
    print(f"individually asked for: {sorted(sonde_sent)}")
    print(f"...and answered      : {sorted(found)}")
    print(f"total PARAM_VALUE    : {counts.get(22, 0)}")
    if pairs:
        worst = max(pairs.values())
        print(f"distinct (index,seq) : {len(pairs)}  worst repetition of one pair: {worst}")
        repeats = sorted(((v, k) for k, v in pairs.items() if v > 1), reverse=True)[:5]
        print("most repeated pairs (index, seq, times):",
              ", ".join(f"{k[0]}/{k[1]}x{v}" for v, k in repeats) or "none")
    print("first 60 indices in arrival order:")
    print(" ".join(str(i) for i in order[:60]))
    print("messages seen (id: count):",
          ", ".join(f"{k}:{v}" for k, v in sorted(counts.items(), key=lambda kv: -kv[1])))
    if set_interval or old_way:
        rate = counts.get(30, 0) / max(seconds - 3, 1)
        print(f"ATTITUDE rate after the request: {rate:.1f} frames/s "
              f"(about 1 Hz means the FC did not raise the stream rate)")
    interesting = sorted(
        ((name, value) for name, value in names.values()
         if name.startswith(("SR", "SERIAL", "MAV")) and name),
    )
    print(f"stream/port parameters seen ({len(interesting)}):")
    for name, value in interesting:
        print(f"   {name:<16} {value:g}")
    s.close()


if __name__ == "__main__":
    main()
