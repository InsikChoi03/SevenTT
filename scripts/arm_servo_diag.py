#!/usr/bin/env python3
"""Diagnostic sender for firmware/arm_servo on a standalone Arduino.

The arm_servo firmware expects six servo targets:
    <ARM,ch0,ch1,ch2,ch3,ch4,ch5>

For the current 2R arm wiring we mostly care about ch0/ch1/ch2
(shoulder/wrist/gripper). The remaining channels are held at 90.
"""
from __future__ import annotations

import argparse
import time


POSES = {
    "neutral": (90, 90, 90, 90, 90, 90),
    "stow": (110, 10, 100, 90, 90, 90),
    "place_closed": (110, 30, 50, 90, 90, 90),
    "place_open": (110, 30, 115, 90, 90, 90),
    "pick_open": (25, 150, 115, 90, 90, 90),
    "pick_closed": (25, 150, 50, 90, 90, 90),
}


def read_for(ser, seconds: float) -> str:
    end = time.time() + seconds
    chunks = []
    while time.time() < end:
        data = ser.read(512)
        if data:
            chunks.append(data.decode("ascii", "ignore"))
    return "".join(chunks)


def send_pose(ser, name: str, hold: float) -> None:
    values = POSES[name]
    line = "<ARM," + ",".join(str(v) for v in values) + ">"
    print(f">>> {line}")
    ser.write((line + "\n").encode("ascii"))
    out = read_for(ser, 0.8)
    interesting = [
        ln for ln in out.splitlines()
        if ln.startswith(("<BANNER", "<ARMACK"))
    ]
    if interesting:
        print("\n".join(interesting))
    print(f"[hold] {name} {hold:.1f}s - gently feel torque / watch motion")
    read_for(ser, hold)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hold", type=float, default=4.0)
    parser.add_argument(
        "--mode",
        choices=["neutral", "gripper", "pick", "cycle"],
        default="gripper",
    )
    args = parser.parse_args()

    import serial

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    try:
        print("[open] waiting for Arduino reset/boot...")
        boot = read_for(ser, 3.5)
        for ln in boot.splitlines():
            if ln.startswith("<BANNER"):
                print(ln)

        if args.mode == "neutral":
            send_pose(ser, "neutral", args.hold)
        elif args.mode == "gripper":
            send_pose(ser, "place_closed", args.hold)
            send_pose(ser, "place_open", args.hold)
            send_pose(ser, "place_closed", args.hold)
        elif args.mode == "pick":
            send_pose(ser, "stow", 1.0)
            send_pose(ser, "pick_open", args.hold)
            send_pose(ser, "pick_closed", args.hold)
            send_pose(ser, "stow", args.hold)
        else:
            send_pose(ser, "neutral", 1.0)
            send_pose(ser, "pick_open", args.hold)
            send_pose(ser, "pick_closed", args.hold)
            send_pose(ser, "place_closed", args.hold)
            send_pose(ser, "place_open", args.hold)
            send_pose(ser, "neutral", args.hold)
        print("[done]")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
