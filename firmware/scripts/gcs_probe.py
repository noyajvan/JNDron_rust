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

CRC_EXTRA = {0: 50, 20: 214, 21: 159, 22: 220, 30: 39, 74: 20, 253: 83, 193: 193}

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
    s = socket.create_connection(("127.0.0.1", 14552), timeout=5)
    s.setblocking(False)
    s.sendall(frame(21, bytes([1, 1]), 0))
    print("sent PARAM_REQUEST_LIST")

    # PARAM_REQUEST_READ (20): param_index(i16) target_system target_component
    # param_id[16] - wire order puts param_index first, then the byte fields.
    def read_index(index):
        payload = struct.pack("<hBB", index, 1, 1) + b"\x00" * 16
        return frame(20, payload, index & 0xFF)

    sonde_at = {time.time() + 3 + 3 * i: idx for i, idx in enumerate(SONDE_INDEXES)}
    sonde_sent = set()
    found = set()

    buf = bytearray()
    order = []
    seen = set()
    counts = {}
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
    s.close()


if __name__ == "__main__":
    main()
