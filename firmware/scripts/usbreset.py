"""Put the ESP32-S3 into the ROM download mode over its USB-Serial/JTAG port.

Why this exists
---------------
esptool performs the same sequence itself for `--before usb_reset`, but it can
only get that far after syncing with the chip, and the sync aborts as soon as
the port carries a panic dump:

    A fatal error occurred: Guru Meditation Error detected (IllegalInstruction)

`detect_panic_handler` in esptool's SLIP reader treats any panic text as fatal,
so a crash-looping application makes the board impossible to flash. This script
runs the reset sequence without reading anything, holds the chip in the ROM
bootloader, and then drains the stale panic output so a following esptool run
sees a clean port.

Sequence copied from esptool/reset.py USBJTAGSerialReset, including the Windows
work-around where a dummy DTR write is needed for an RTS change to propagate.
"""

import sys
import time

import serial


def rts(port, value):
    port.rts = value
    # Windows only propagates DTR together with an RTS write.
    port.dtr = port.dtr


def dtr(port, value):
    port.dtr = value


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    port = serial.Serial()
    port.port = name
    port.baudrate = 115200
    port.timeout = 0.3
    port.open()

    rts(port, False)
    dtr(port, False)  # idle
    time.sleep(0.1)
    dtr(port, True)  # set IO0
    rts(port, False)
    time.sleep(0.1)
    rts(port, True)  # reset
    dtr(port, False)
    rts(port, True)
    time.sleep(0.1)
    dtr(port, False)
    rts(port, False)  # out of reset
    time.sleep(0.3)

    stale = b""
    while True:
        chunk = port.read(65536)
        if not chunk:
            break
        stale += chunk
    port.close()

    print(f"usb reset done, drained {len(stale)} stale bytes")
    if stale:
        head = stale[:100].decode("latin-1").replace("\r", " ").replace("\n", " ")
        print(f"first bytes were: {head}")


if __name__ == "__main__":
    main()
