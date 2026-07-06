#!/usr/bin/env python3
"""Serial-only encoder distance / rotation test scaffold for the mecanum base.

This script is intentionally low-level and ROS-free. It is the right place to validate:

- whether the MCU emits real <ODOM,...> packets
- whether commanded forward motion roughly matches measured distance
- whether the robot stops near a requested distance
- whether the robot rotates near a requested angle

Current firmware note:
- `base_arm_combined.ino` emits `<ODOM,...>` after the encoder-enabled sketch is flashed
- `<ENC,...>` raw tick lines are printed so wheel/encoder mapping can be checked
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import serial


def send_base(ser: serial.Serial, fl: float, fr: float, rl: float, rr: float) -> None:
    ser.write(f"<BASE,{fl:.3f},{fr:.3f},{rl:.3f},{rr:.3f}>\n".encode("ascii"))


def stop_base(ser: serial.Serial) -> None:
    send_base(ser, 0.0, 0.0, 0.0, 0.0)


def reset_encoders(ser: serial.Serial) -> None:
    ser.write(b"<ENCZERO>\n")


def brake_base(ser: serial.Serial, wheel_speeds: list[float], ms: int, hz: float) -> None:
    if ms <= 0:
        return
    period = 1.0 / max(hz, 1e-6)
    end = time.monotonic() + (ms / 1000.0)
    while time.monotonic() < end:
        send_base(ser, *wheel_speeds)
        time.sleep(period)
    stop_base(ser)


def parse_packet(line: str) -> tuple[str, list[float]] | None:
    if not (line.startswith("<") and line.endswith(">")):
        return None
    body = line[1:-1]
    parts = body.split(",")
    if not parts:
        return None
    tag = parts[0]
    vals: list[float] = []
    for p in parts[1:]:
        try:
            vals.append(float(p))
        except ValueError:
            return None
    return tag, vals


def parse_index_list(text: str) -> list[int]:
    out: list[int] = []
    for part in text.split(","):
        p = part.strip()
        if not p:
            continue
        idx = int(p)
        if idx < 0 or idx > 3:
            raise ValueError(f"wheel index out of range: {idx}")
        out.append(idx)
    if not out:
        raise ValueError("need at least one wheel index")
    return out


def selected_motion_speed(vals: list[float], wheels: list[int], signs: list[float]) -> float:
    if len(vals) < 4:
        return 0.0
    samples = [float(signs[i]) * float(vals[i]) for i in wheels]
    if len(samples) == 1:
        return samples[0]
    if len(samples) == 2:
        return 0.5 * (samples[0] + samples[1])
    return float(statistics.median(samples))


def motion_profile(direction: str, speed: float) -> tuple[list[float], list[float], list[float]]:
    mag = abs(float(speed))
    if direction == "forward":
        cmd = [mag, mag, mag, mag]
        odom_signs = [1.0, 1.0, 1.0, 1.0]
    elif direction == "backward":
        cmd = [-mag, -mag, -mag, -mag]
        odom_signs = [-1.0, -1.0, -1.0, -1.0]
    elif direction == "right":
        cmd = [mag, -mag, -mag, mag]
        odom_signs = [1.0, -1.0, -1.0, 1.0]
    elif direction == "left":
        cmd = [-mag, mag, mag, -mag]
        odom_signs = [-1.0, 1.0, 1.0, -1.0]
    elif direction == "ccw":
        cmd = [-mag, mag, -mag, mag]
        odom_signs = [-1.0, 1.0, -1.0, 1.0]
    elif direction == "cw":
        cmd = [mag, -mag, mag, -mag]
        odom_signs = [1.0, -1.0, 1.0, -1.0]
    else:
        raise ValueError(f"unsupported direction: {direction}")
    brake = [-0.5 * v for v in cmd]
    return cmd, brake, odom_signs


def main() -> None:
    ap = argparse.ArgumentParser(description="Closed-loop distance test using MCU encoder packets")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--direction", choices=["forward", "backward", "left", "right", "cw", "ccw"], default="forward")
    ap.add_argument("--speed", type=float, default=0.10, help="wheel speed command magnitude in m/s")
    ap.add_argument("--target-m", type=float, default=0.50, help="requested travel distance in meters")
    ap.add_argument("--target-deg", type=float, default=90.0, help="requested rotation in degrees for cw/ccw")
    ap.add_argument("--timeout", type=float, default=10.0, help="max test duration in seconds")
    ap.add_argument("--hz", type=float, default=20.0, help="command resend rate")
    ap.add_argument("--k", type=float, default=0.20, help="mecanum k = lx + ly in meters for rotation integration")
    ap.add_argument(
        "--brake-ms",
        type=int,
        default=0,
        help="reverse brake pulse duration in ms after target reached (0 disables)",
    )
    ap.add_argument(
        "--brake-speed-scale",
        type=float,
        default=0.5,
        help="reverse brake pulse speed as a fraction of commanded speed magnitude",
    )
    ap.add_argument(
        "--odom-wheels",
        default="1,2,3",
        help="comma-separated ODOM wheel indexes to trust; default ignores weak FL encoder",
    )
    args = ap.parse_args()
    trusted_wheels = parse_index_list(args.odom_wheels)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"port open failed: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"opened {args.port} @ {args.baud}")
    print("waiting for MCU reboot...")
    time.sleep(2.0)
    ser.reset_input_buffer()
    stop_base(ser)
    reset_encoders(ser)

    cmd_wheels, brake_wheels, odom_signs = motion_profile(args.direction, args.speed)
    brake_scale = max(0.0, float(args.brake_speed_scale))
    brake_wheels = [v * brake_scale for v in brake_wheels]
    rotational = args.direction in {"cw", "ccw"}
    target = math.radians(abs(float(args.target_deg))) if rotational else abs(float(args.target_m))
    cmd_period = 1.0 / max(args.hz, 1e-6)
    start = time.monotonic()
    last_cmd = 0.0
    last_odom_t = None
    progress = 0.0
    saw_hb = False
    saw_odom = False
    rx_buf = b""
    target_label = f"{math.degrees(target):.1f}deg" if rotational else f"{target:.3f}m"
    print(
        f"test: direction={args.direction} wheel_cmd={cmd_wheels} "
        f"target={target_label} odom_wheels={trusted_wheels} "
        f"brake_ms={args.brake_ms} brake_wheels={brake_wheels}"
    )

    try:
        while True:
            now = time.monotonic()
            if now - start > args.timeout:
                if rotational:
                    print(f"timeout: angle={math.degrees(progress):.1f} deg")
                else:
                    print(f"timeout: distance={progress:.3f} m")
                break

            if now - last_cmd >= cmd_period:
                send_base(ser, *cmd_wheels)
                last_cmd = now

            data = ser.read(128)
            if data:
                rx_buf += data
            while b"\n" in rx_buf:
                raw, rx_buf = rx_buf.split(b"\n", 1)
                line = raw.strip().decode("ascii", errors="ignore")
                pkt = parse_packet(line)
                if pkt is None:
                    continue
                tag, vals = pkt

                if tag == "HB":
                    saw_hb = True
                elif tag == "ENC":
                    if len(vals) >= 4:
                        print(
                            f"raw enc e1={int(vals[0])} e2={int(vals[1])} "
                            f"e3={int(vals[2])} e4={int(vals[3])}"
                        )
                elif tag == "ODOM":
                    saw_odom = True
                    t_now = time.monotonic()
                    signed_speed = selected_motion_speed(vals, trusted_wheels, odom_signs)
                    if last_odom_t is not None:
                        dt = max(0.0, min(0.5, t_now - last_odom_t))
                        if rotational:
                            progress += (signed_speed / max(float(args.k), 1e-6)) * dt
                        else:
                            progress += signed_speed * dt
                    last_odom_t = t_now
                    if rotational:
                        print(
                            f"odom spin={signed_speed:+.3f} angle={math.degrees(progress):+.1f} / "
                            f"{math.degrees(target):.1f} deg"
                        )
                    else:
                        print(f"odom speed={signed_speed:+.3f} distance={progress:+.3f} / {target:.3f} m")
                    if abs(progress) >= target:
                        print("target reached")
                        if args.brake_ms > 0:
                            print(f"applying reverse brake pulse: {brake_wheels} for {args.brake_ms} ms")
                            brake_base(ser, brake_wheels, args.brake_ms, args.hz)
                        stop_base(ser)
                        return

            if saw_hb and not saw_odom and (time.monotonic() - start) > 1.5:
                print("MCU is only sending <HB,...> command echo.")
                print("Add real encoder output <ODOM,fl,fr,rl,rr,t_ms> in firmware first.")
                break
    finally:
        for _ in range(5):
            stop_base(ser)
            time.sleep(0.02)
        ser.flush()
        ser.close()
        print("stopped and closed")


if __name__ == "__main__":
    main()
