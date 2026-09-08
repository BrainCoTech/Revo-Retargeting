#!/usr/bin/env python3
"""Read Revo2 unit mode, firmware limits and feedback without commanding motion.

Stop the ROS driver before running. Requires pyserial and fuser (psmisc).
Only Modbus FC03/FC04 are sent; unit mode and protection settings are preserved.
Register reference: https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_touch.html
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import struct
import subprocess
import time

import serial


def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xA001 if value & 1 else 0)
    return struct.pack("<H", value)


def read_registers(port, slave, function, address, count):
    if function not in (3, 4) or not 1 <= count <= 24:
        raise ValueError("Only short read-register requests are allowed")
    request = struct.pack(">BBHH", slave, function, address, count)
    port.reset_input_buffer()
    port.write(request + crc16(request))
    header = port.read(3)
    if len(header) != 3 or header[0] != slave:
        raise RuntimeError(f"No valid reply for register {address}: {header.hex()}")
    tail = port.read(2 if header[1] & 0x80 else header[2] + 2)
    frame = header + tail
    if len(tail) < 2 or crc16(frame[:-2]) != frame[-2:]:
        raise RuntimeError(f"Incomplete reply or bad CRC at register {address}")
    if header[1] == function | 0x80:
        raise RuntimeError(f"Modbus exception {header[2]} at register {address}")
    if header[1] != function or header[2] != 2 * count or len(tail) != 2 * count + 2:
        raise RuntimeError(f"Unexpected reply at register {address}: {frame.hex()}")
    time.sleep(0.02)
    return list(struct.unpack(">" + "H" * count, frame[3:-2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=460800)
    parser.add_argument("--slave-id", type=int, default=126)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.slave_id <= 247:
        parser.error("slave-id must be 1..247; broadcast is not allowed")
    if args.output and args.output.exists():
        parser.error("output already exists; choose a new capture filename")
    # Refuse to share a port even with a driver that does not use flock.
    holders = subprocess.run(["fuser", args.port], capture_output=True, text=True)
    if holders.returncode != 1:
        parser.error(f"port is busy or could not be checked: {holders.stdout} {holders.stderr}")
    result = {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "port": args.port, "slave_id": args.slave_id,
        "finger_order": ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"],
        "register_source": "direct Modbus reads, without SDK unit conversion",
        "note": "Configured limits and a stationary snapshot; not measured motion endpoints.",
    }
    with serial.Serial(args.port, args.baudrate, timeout=0.5, exclusive=True) as port:
        def read(address, count=1, function=3):
            return read_registers(port, args.slave_id, function, address, count)

        mode = read(937)[0]
        result["unit_mode_register_937"] = mode
        result["unit_mode"] = {0: "normalized", 1: "physical"}.get(mode, "unknown")
        for name, address in (("min_position_deg", 946), ("max_position_deg", 952),
                              ("max_speed_deg_s", 958), ("max_current_ma", 964)):
            result[name] = read(address, 6)
        result["thumb_aux_lock_current_ma"] = read(936)[0]
        result["positions_raw"] = read(2000, 6, 4)
        result["speeds_raw"] = [v if v < 32768 else v - 65536 for v in read(2006, 6, 4)]
        result["currents_raw"] = [v if v < 32768 else v - 65536 for v in read(2012, 6, 4)]
        result["motor_states"] = read(2018, 6, 4)
        firmware = read(3000, 10, 4)
        result["firmware"] = struct.pack(">10H", *firmware).rstrip(b"\x00").decode("ascii", errors="replace")
        serial_number = read(3010, 10, 4)
        result["serial_number"] = struct.pack(">10H", *serial_number).rstrip(b"\x00").decode("ascii", errors="replace")
        if read(937)[0] != mode:
            raise RuntimeError("Device unit mode changed during capture; discard this snapshot")
    output = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        with args.output.open("x") as stream:
            stream.write(output)
    print(output, end="")


if __name__ == "__main__":
    main()
