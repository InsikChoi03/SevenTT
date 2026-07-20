#!/usr/bin/env python3
"""Safely pulse the Shape4 profile/lift driver through Arduino D13.

The combined firmware accepts <LIFT,milliseconds>. D13 must drive an external
MOSFET/relay logic input; it must never power the motor directly.
"""

from __future__ import annotations

import argparse
import time

import serial


def read_lines(ser: serial.Serial, seconds: float) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            lines.append(line)
            print(line)
    return lines


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--duration", type=float, default=1.0, help="ON time in seconds (max 10)")
    args = parser.parse_args()

    duration = max(0.1, min(10.0, float(args.duration)))
    duration_ms = int(round(duration * 1000.0))

    print("WARNING: stop the main ROS run and clear the profile mechanism before this test.")
    print(f"Opening {args.port}; the combined Arduino will reset once.")
    with serial.Serial(args.port, 115200, timeout=0.1) as ser:
        time.sleep(2.2)
        read_lines(ser, 0.6)
        print(f"D13 ON for {duration_ms} ms")
        ser.write(f"<LIFT,{duration_ms}>\n".encode("ascii"))
        ser.flush()
        try:
            lines = read_lines(ser, duration + 1.0)
        finally:
            # Redundant OFF commands leave the output low even if the test is interrupted.
            for _ in range(3):
                ser.write(b"<LIFT,0>\n")
                ser.flush()
                time.sleep(0.05)

    accepted = any(line.startswith(f"<LIFTACK,{duration_ms}>") for line in lines)
    completed = any(line == "<LIFT,done>" for line in lines)
    if accepted and completed:
        print("PASS: firmware accepted the command and automatically switched D13 OFF.")
        return 0
    print("FAIL: expected LIFTACK and LIFT done messages were not both observed.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
