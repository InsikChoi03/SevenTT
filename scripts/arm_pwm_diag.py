#!/usr/bin/env python3
"""2R arm PWM diagnostic over the combined Arduino firmware.

This sends STATUS,RUNNING, then raw <ARM,shoulder,wrist,gripper> targets.
Use it when the serial path ACKs but the arm appears limp, because it holds
each target long enough to feel servo torque and prints MCU responses.
"""
from __future__ import annotations

import argparse
import time


POSES = {
    "stow": (110, 10, 100),
    "place_closed": (110, 30, 50),
    "place_open": (110, 30, 115),
    "pick_open": (25, 150, 115),
    "pick_closed": (25, 150, 50),
}


def read_for(ser, seconds: float) -> str:
    end = time.time() + seconds
    chunks = []
    while time.time() < end:
        data = ser.read(512)
        if data:
            chunks.append(data.decode("ascii", "ignore"))
    return "".join(chunks)


def send_line(ser, line: str, wait: float) -> str:
    print(f">>> {line}")
    ser.write((line.rstrip() + "\n").encode("ascii"))
    out = read_for(ser, wait)
    interesting = [
        ln for ln in out.splitlines()
        if ln.startswith(("<I2C", "<BANNER", "<ARMACK", "<STOPPED"))
    ]
    if interesting:
        print("\n".join(interesting))
    return out


def send_pose(ser, name: str, hold: float) -> None:
    sh, wr, gr = POSES[name]
    send_line(ser, f"<ARM,{sh},{wr},{gr}>", 0.5)
    print(f"[hold] {name} {hold:.1f}s - gently feel whether the servo has torque")
    read_for(ser, hold)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hold", type=float, default=4.0)
    parser.add_argument(
        "--mode",
        choices=["stow", "gripper", "pick", "cycle"],
        default="gripper",
    )
    parser.add_argument(
        "--leave-running",
        action="store_true",
        help="do not send STATUS,STANDBY at the end",
    )
    args = parser.parse_args()

    import serial

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    try:
        print("[open] waiting for Arduino reset/boot...")
        boot = read_for(ser, 2.5)
        for ln in boot.splitlines():
            if ln.startswith(("<I2C", "<BANNER")):
                print(ln)

        send_line(ser, "<STATUS,RUNNING>", 0.3)

        if args.mode == "stow":
            send_pose(ser, "stow", args.hold)
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
            send_pose(ser, "stow", 1.0)
            send_pose(ser, "pick_open", args.hold)
            send_pose(ser, "pick_closed", args.hold)
            send_pose(ser, "place_closed", args.hold)
            send_pose(ser, "place_open", args.hold)
            send_pose(ser, "place_closed", args.hold)

        if not args.leave_running:
            send_line(ser, "<STATUS,STANDBY>", 0.2)
        print("[done]")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
