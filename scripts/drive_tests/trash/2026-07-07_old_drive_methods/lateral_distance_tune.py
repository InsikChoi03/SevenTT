#!/usr/bin/env python3
"""Continuous lateral distance tuner for the mecanum base.

Use this after the left/right wheel balance is roughly correct.

The script does NOT rely on encoder distance for stopping. It drives laterally
for a fixed continuous duration, stops, asks for the measured travel distance,
and suggests the next duration for the requested distance.

Default wheel scales:
    right: FL=0.7 FR=0.7 RL=0.8 RR=0.7
    left : FL=0.65 FR=0.75 RL=0.75 RR=0.65

Current empirical duration seeds at speed=0.50:
    right 10cm  ~= 414ms
    right 100cm ~= 2950ms
    left 10cm   ~= 400ms
    left 100cm  ~= 3410ms
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

import serial

from encoder_distance_test import send_base, stop_base

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
SIGNS = {
    "right": [1.0, -1.0, -1.0, 1.0],
    "left": [-1.0, 1.0, 1.0, -1.0],
}
DEFAULT_SCALES = {
    "right": [0.7, 0.7, 0.8, 0.7],
    "left": [0.65, 0.75, 0.75, 0.65],
}
DEFAULT_DURATIONS_MS = {
    "right": {10.0: 414, 100.0: 2950},
    "left": {10.0: 400, 100.0: 3410},
}


def wheel_cmd(direction: str, speed: float, scales: list[float]) -> list[float]:
    return [SIGNS[direction][i] * abs(speed) * scales[i] for i in range(4)]


def parse_scales(text: str, direction: str) -> list[float]:
    if not text.strip():
        return list(DEFAULT_SCALES[direction])
    parts = text.replace(",", " ").split()
    if len(parts) != 4:
        raise ValueError("scales must be 4 numbers: FL FR RL RR")
    return [float(v) for v in parts]


def append_csv(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def duration_seed(direction: str, target_cm: float) -> int:
    seeds = DEFAULT_DURATIONS_MS[direction]
    target = abs(float(target_cm))
    if target <= 10.0:
        return seeds[10.0]
    if target >= 100.0:
        return seeds[100.0]

    lo_cm, hi_cm = 10.0, 100.0
    lo_ms, hi_ms = seeds[lo_cm], seeds[hi_cm]
    ratio = (target - lo_cm) / (hi_cm - lo_cm)
    return int(round(lo_ms + ratio * (hi_ms - lo_ms)))


def run_continuous(
    ser: serial.Serial,
    *,
    cmd: list[float],
    duration_ms: int,
    brake_ms: int,
    brake_scale: float,
    hz: float,
) -> None:
    period = 1.0 / max(hz, 1e-6)
    duration_s = max(0.001, duration_ms / 1000.0)
    end_at = time.monotonic() + duration_s
    while time.monotonic() < end_at:
        send_base(ser, *cmd)
        time.sleep(period)
    stop_base(ser)
    if brake_ms > 0 and brake_scale > 0.0:
        brake_cmd = [-float(v) * brake_scale for v in cmd]
        end_brake = time.monotonic() + (brake_ms / 1000.0)
        while time.monotonic() < end_brake:
            send_base(ser, *brake_cmd)
            time.sleep(period)
        stop_base(ser)


def suggest_duration_ms(target_cm: float, actual_cm: float, used_duration_ms: int) -> int:
    if abs(actual_cm) < 1e-6:
        return max(used_duration_ms + 50, 1)
    cm_per_ms = abs(actual_cm) / max(used_duration_ms, 1)
    return max(1, int(round(abs(target_cm) / cm_per_ms)))


def error_report(target_cm: float, actual_cm: float) -> tuple[float, float, bool]:
    target = abs(float(target_cm))
    actual = abs(float(actual_cm))
    error_cm = actual - target
    error_pct = 0.0 if target <= 1e-9 else (error_cm / target) * 100.0
    return error_cm, error_pct, abs(error_pct) <= 5.0


def ask_float(prompt: str, default: float | None = None) -> float | None:
    suffix = f" [{default}]" if default is not None else ""
    text = input(f"{prompt}{suffix}: ").strip()
    if text.lower() in {"q", "quit", "exit"}:
        return None
    if not text and default is not None:
        return float(default)
    return float(text)


def ask_int(prompt: str, default: int) -> int | None:
    text = input(f"{prompt} [{default}]: ").strip()
    if text.lower() in {"q", "quit", "exit"}:
        return None
    if not text:
        return int(default)
    return int(text)


def main() -> None:
    ap = argparse.ArgumentParser(description="Tune lateral distance using continuous drive duration")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--direction", choices=["right", "left"], default="right")
    ap.add_argument("--target-cm", type=float, default=10.0)
    ap.add_argument("--speed", type=float, default=0.50)
    ap.add_argument("--duration-ms", type=int, default=0, help="0 uses empirical seed for direction/target")
    ap.add_argument("--brake-ms", type=int, default=0)
    ap.add_argument("--brake-scale", type=float, default=0.35)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--scales", default="", help="optional 4 numbers: FL FR RL RR")
    ap.add_argument("--csv", default="scripts/drive_tests/lateral_distance_tuning_v2_log.csv")
    args = ap.parse_args()

    try:
        scales = parse_scales(args.scales, args.direction)
    except ValueError as e:
        print(e, file=sys.stderr)
        sys.exit(2)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"port open failed: {e}", file=sys.stderr)
        sys.exit(1)

    params: dict[str, object] = {
        "direction": args.direction,
        "target_cm": args.target_cm,
        "speed": args.speed,
        "duration_ms": args.duration_ms if args.duration_ms > 0 else duration_seed(args.direction, args.target_cm),
        "brake_ms": args.brake_ms,
        "brake_scale": args.brake_scale,
        "scales": scales,
    }
    csv_path = Path(args.csv)

    print(f"opened {args.port} @ {args.baud}")
    print("waiting for MCU reboot...")
    time.sleep(2.0)
    ser.reset_input_buffer()
    stop_base(ser)
    print("Type q at prompts to quit.")

    trial = 1
    try:
        while True:
            direction = str(params["direction"])
            print(f"\n--- distance trial {trial} ---")
            print(
                f"default scales for {direction}: "
                + " ".join(f"{name}={value:.2f}" for name, value in zip(WHEEL_NAMES, DEFAULT_SCALES[direction]))
            )

            text = input(f"direction right/left [{direction}]: ").strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            if text:
                if text not in {"right", "left"}:
                    print("direction must be right or left")
                    continue
                params["direction"] = text
                direction = text
                params["scales"] = list(DEFAULT_SCALES[direction])
                params["duration_ms"] = duration_seed(direction, float(params["target_cm"]))

            text = input(
                "scales FL FR RL RR "
                f"[{' '.join(str(v) for v in params['scales'])}]: "
            ).strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            try:
                if text:
                    params["scales"] = parse_scales(text, direction)
            except ValueError as e:
                print(e)
                continue

            target_cm = ask_float("target_cm", float(params["target_cm"]))
            if target_cm is None:
                break
            if abs(float(target_cm) - float(params["target_cm"])) > 1e-9:
                params["duration_ms"] = duration_seed(direction, target_cm)
            speed = ask_float("speed", float(params["speed"]))
            if speed is None:
                break
            duration_ms = ask_int("duration_ms", int(params["duration_ms"]))
            if duration_ms is None:
                break
            brake_ms = ask_int("brake_ms", int(params["brake_ms"]))
            if brake_ms is None:
                break
            brake_scale = ask_float("brake_scale", float(params["brake_scale"]))
            if brake_scale is None:
                break

            params.update(
                {
                    "target_cm": target_cm,
                    "speed": speed,
                    "duration_ms": duration_ms,
                    "brake_ms": brake_ms,
                    "brake_scale": brake_scale,
                }
            )

            cmd = wheel_cmd(direction, speed, list(params["scales"]))
            print(f"cmd: [{cmd[0]:+.3f}, {cmd[1]:+.3f}, {cmd[2]:+.3f}, {cmd[3]:+.3f}]")
            text = input("Press Enter to run, or q to quit: ").strip().lower()
            if text in {"q", "quit", "exit"}:
                break

            started = time.monotonic()
            run_continuous(
                ser,
                cmd=cmd,
                duration_ms=duration_ms,
                brake_ms=brake_ms,
                brake_scale=brake_scale,
                hz=args.hz,
            )
            elapsed_s = time.monotonic() - started

            actual_cm = ask_float("actual lateral cm", None)
            if actual_cm is None:
                break
            forward_drift_cm = ask_float("forward/back drift cm (+forward, -back)", 0.0)
            if forward_drift_cm is None:
                break
            yaw_deg = ask_float("yaw_deg (+ccw, -cw)", 0.0)
            if yaw_deg is None:
                break
            notes = input("notes: ").strip()

            next_duration_ms = suggest_duration_ms(target_cm, actual_cm, duration_ms)
            error_cm, error_pct, within_5pct = error_report(target_cm, actual_cm)
            print(f"actual rate: {abs(actual_cm) / max(duration_ms, 1):.3f} cm/ms")
            print(
                f"error: {error_cm:+.2f}cm ({error_pct:+.1f}%) "
                f"{'PASS' if within_5pct else 'FAIL'} for 5% target"
            )
            print(f"suggested next duration for {target_cm:.1f} cm: {next_duration_ms} ms")
            if input("Use suggested duration next? [Y/n]: ").strip().lower() not in {"n", "no"}:
                params["duration_ms"] = next_duration_ms

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "trial": trial,
                "direction": direction,
                "target_cm": target_cm,
                "speed": speed,
                "duration_ms": duration_ms,
                "brake_ms": brake_ms,
                "brake_scale": brake_scale,
                "scale_fl": list(params["scales"])[0],
                "scale_fr": list(params["scales"])[1],
                "scale_rl": list(params["scales"])[2],
                "scale_rr": list(params["scales"])[3],
                "elapsed_s": f"{elapsed_s:.3f}",
                "actual_lateral_cm": actual_cm,
                "error_cm": f"{error_cm:.3f}",
                "error_pct": f"{error_pct:.2f}",
                "within_5pct": within_5pct,
                "forward_drift_cm": forward_drift_cm,
                "yaw_deg": yaw_deg,
                "suggested_next_duration_ms": next_duration_ms,
                "notes": notes,
            }
            append_csv(csv_path, row)
            print(f"saved to {csv_path}")
            trial += 1
    finally:
        for _ in range(5):
            stop_base(ser)
            time.sleep(0.02)
        ser.flush()
        ser.close()
        print("stopped and closed")


if __name__ == "__main__":
    main()
