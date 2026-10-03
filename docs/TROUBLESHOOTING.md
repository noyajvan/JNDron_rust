# Field notes: what breaks, how it looks, what to do

Everything here was measured on the real chain - ESP32-S3 → ArduCopter on
`SERIAL2` → phone hotspot → 4G → Oracle VPS relay (`<relay-ip>`) → Mission
Planner. Each entry gives the symptom as it appears in a log, the actual cause,
and the fix, with the commit that made it. Written because the same few problems
cost hours twice.

## The chain, and where to look

| Piece | How to look at it |
| --- | --- |
| Board console | USB Serial/JTAG, 115200 8N1. `STATUS` prints state, IP, RSSI, FC counters |
| Relay | `ssh ubuntu@<relay-ip> 'journalctl -u drone-relay -n 40'` |
| Relay sockets | `ss -tlnp` (14552 GCS, 14553 drone) and `ss -tin 'sport = :14553'` for byte counters |
| What actually arrives | `firmware/scripts/gcs_probe.py` (run it on the VPS) |
| Flight controller | `firmware/scripts/set_fc_streams.py` reads and writes `SRx_*` |
| MAVLink constant | `firmware/scripts/crc_extra.py`, with self tests against published values |

## 1. "Getting params" that never finishes

**Symptom.** Mission Planner shows `Getting params` and sits there; it reconnects
after a while and starts again.

**Cause, measured.** `PARAM_REQUEST_LIST` makes ArduPilot restart its parameter
walk from index 0. Mission Planner re-sends that request while its dialog sees no
progress, so with a slow walk the download can never complete:

```
one request            -> 1130 distinct parameters
repeat every 5 seconds ->  52 distinct parameters, then it loops
```

The walk is slow because `SR2_PARAMS` was 2 Hz. The full list is 1129 parameters,
about 42 KiB.

**Fix.** `Bridge::feed_gcs_bytes` (commit `c45d697`) parses the ground station's
stream just enough to hold a `PARAM_REQUEST_LIST` back while a walk is making
progress, and replays it byte for byte (same sysid/compid) if the walk stalls for
10 s. Everything else is still forwarded exactly as it arrived. Verified: 1129 of
1129 with repeats every five seconds.

**Related.** The flight controller **ignores** `SET_MESSAGE_INTERVAL` (CRC_EXTRA
82) and `REQUEST_DATA_STREAM` (CRC_EXTRA 193) on this build; neither raised the
stream rate. `SRx_*` are parameters, and `PARAM_SET` is honoured - so
`set_fc_streams.py` writes those instead, then `MAV_CMD_PREFLIGHT_STORAGE` to
store them, exactly like the original `sendPreflightStorage()`.

`SR2_PARAMS` accepts at most **10** (50 is rejected outright, not clamped).

## 2. The link hangs about fifteen seconds in

**Symptom.** Parameters start downloading, then everything stops. On the relay:

```
Drone TCP silent 15s, dropping
```

The relay drops a drone that says nothing for 15 s - that is the "15 seconds".

**Cause.** The board was *associated but had no address*: driver reported
connected (`w=3`), DHCP had not delivered a lease, so every relay connection
failed with `HostUnreachable` and the uplink went quiet.

**Fix.** `Bridge::tick` (commit `589fe95`) treats "associated with no address for
10 s" as a Wi-Fi failure and restarts the radio. `Io::local_ip` and
`Io::rssi_dbm` are implemented for real in the platform, which is both how the
watchdog can see the state and why `STATUS` stopped printing `0.0.0.0`.

**Do not** solve a full send queue by pausing the flight controller (tried,
`bbb7888`, reverted in `f8613dd`). It stops outgoing traffic altogether and the
relay then drops us - losing frames is something MAVLink recovers from, going
mute is not.

## 3. The hotspot is invisible to the board

**Symptom.**

```
W wifi:Haven't to connect to a suitable AP now!
wifi: 'LEO' not in the scan, keeping WPA2Personal
```

**Cause.** On a Pixel with **"Extend compatibility" off**, the hotspot prefers
5 GHz. The ESP32 is 2.4 GHz only and cannot see it at all. DroneBridge's own
troubleshooting page says the same: the access point must do 802.11b/legacy mode
and 2.4 GHz, and Pixels turn the hotspot off when no device is connected.

**Fix.** Settings → Network & internet → Hotspot & tethering → Wi-Fi hotspot:
turn **Extend compatibility ON**, keep security WPA2-Personal, and stop the
hotspot from switching itself off. The board then logs
`wifi: 'LEO' offers WPA2Personal` and joins.

The security mode is now taken from a scan rather than hard-coded WPA2, because
asking for WPA2 against a WPA3 or transition hotspot fails the handshake even
with the correct password (`init -> auth -> assoc -> init (0x400)` in a loop,
indistinguishable from a bad key).

## 4. Radio settings (heat, current, stability)

The C++ original reduced the CPU to 80 MHz, set the transmitter to a fixed
11 dBm and disabled modem sleep, with the comment that 11 dBm is the verified
working value. This port already ran at 80 MHz
(`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_80=y`) but had the maximum transmit power and
modem sleep on. Commit `f8613dd` sets both: 11 dBm and `WIFI_PS_NONE`. Measured
after: RSSI -34 dBm where the same spot had shown -73 dBm with 20 dBm, so the
hotspot and the board were also simply too far apart / too sleepy before.

## 5. Telemetry loss inside the bridge

**Symptom.** `relay: queued only N/M bytes, telemetry dropped` in a loop.

**Cause.** With the socket non-blocking, `tcp_send` threw away whatever did not
fit the socket buffer. Measured against the FC's own counters: the FC produced
about 24 messages/s while about 9/s reached the ground station.

**Fix.** Commit `bbb7888` queues the bytes (64 KiB) and flushes them as the
socket drains, capped at 8 KiB per call (`f8613dd`) - draining 49 KiB in one pass
produced `Interrupt wdt timeout on CPU1` and a software reset (`rst:0xc
RTC_SW_CPU_RST`), which looks like a power problem but is not.

## 6. Flashing a board that is crash-looping

esptool refuses to work when the application is spamming panics into the port:

```
A fatal error occurred: Guru Meditation Error detected (IllegalInstruction)
```

and issuing its own `--before usb_reset` while the chip is already in the
bootloader gave `Write timeout` three times in a row.

`firmware/scripts/usbreset.py` puts the chip into the ROM bootloader without
reading the port (a copy of esptool's USB-JTAG reset sequence), and `flash.cmd`
then uses `--before no_reset`. Measured: three failures in a row with the
single-step approach, first-try success with this one, 980 KiB in 9.5 s.

Also worth knowing: a failed flash leaves the application region half-erased, and
the board then crash-loops on its own until a good image is written. And the
cable matters more than it should - a replaced cable once made every long write
fail while the original flashed 976 KiB in 10 s.

## 7. Relay plumbing

* Drone links: TCP 14553 (preferred, the ESP32 dials out) and UDP 14550 (legacy).
* Ground station: TCP 14552 (Mission Planner), UDP 14551.
* Only TCP is used towards the drone when a TCP drone is connected - no duplicates.
* `Drone TCP silent 15s, dropping` is the only reason a drone gets disconnected.
* The relay forwards bytes, it does not rewrite MAVLink. Anything that needs to
  understand MAVLink has to live in the bridge.

## What the flight controller looks like

```
SERIAL2_BAUD 921   SERIAL2_PROTOCOL 2   <- the port wired to the bridge
SERIAL3_BAUD 921   SERIAL3_PROTOCOL 2
SR2_* initially 2-10 Hz, param_count 1129, ArduCopter V4.6.2 (1ebd4d99)
```

Mission Planner rewrites `SR2_*` on connect according to its own link-speed
estimate, so those values move by themselves.
