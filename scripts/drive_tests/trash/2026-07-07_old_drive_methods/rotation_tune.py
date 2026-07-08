#!/usr/bin/env python3
"""Continuous in-place rotation tuner for the mecanum base.

Use this before fine angle calibration when the base is heavy enough that one
or more wheels do not start turning at low command values.

This script does not stop from encoder angle. It sends direct wheel commands
for a fixed continuous duration, with an optional high-power start boost, then
asks for the measured rotation and suggests the next duration.

Wheel command signs:
    cw : FL=+ FR=- RL=+ RR=-
    ccw: FL=- FR=+ RL=- RR=+
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
    "cw": [1.0, -1.0, 1.0, -1.0],
    "ccw": [-1.0, 1.0, -1.0, 1.0],
}
DEFAULT_SCALES = {
    "cw": [1.0, 1.0, 1.0, 1.0],
    "ccw": [1.0, 1.0, 1.0, 1.0],
}
DEFAULT_DURATIONS_MS = {
    "cw": {45.0: 100, 90.0: 230, 180.0: 665},
    "ccw": {45.0: 100, 90.0: 230, 180.0: 665},
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


def duration_seed(direction: str, target_deg: float) -> int:
    seeds = DEFAULT_DURATIONS_MS[direction]
    target = abs(float(target_deg))
    points = sorted(seeds.items())
    if target <= points[0][0]:
        return int(round(points[0][1] * target / points[0][0]))
    for (lo_deg, lo_ms), (hi_deg, hi_ms) in zip(points, points[1:]):
        if target <= hi_deg:
            ratio = (target - lo_deg) / (hi_deg - lo_deg)
            return int(round(lo_ms + ratio * (hi_ms - lo_ms)))
    lo_deg, lo_ms = points[-1]
    ratio = target / lo_deg
    return int(round(lo_ms * ratio))


def run_continuous_rotation(
    ser: serial.Serial,
    *,
    steady_cmd: list[float],
    boost_cmd: list[float],
    duration_ms: int,
    boost_ms: int,
    brake_ms: int,
    brake_scale: float,
    hz: float,
) -> None:
    period = 1.0 / max(hz, 1e-6)
    duration_s = max(0.001, duration_ms / 1000.0)
    boost_s = max(0.0, min(boost_ms / 1000.0, duration_s))
    start = time.monotonic()
    end_at = start + duration_s
    boost_until = start + boost_s

    while time.monotonic() < end_at:
        cmd = boost_cmd if time.monotonic() < boost_until else steady_cmd
        send_base(ser, *cmd)
        time.sleep(period)

    stop_base(ser)

    if brake_ms > 0 and brake_scale > 0.0:
        brake_cmd = [-float(v) * brake_scale for v in steady_cmd]
        end_brake = time.monotonic() + (brake_ms / 1000.0)
        while time.monotonic() < end_brake:
            send_base(ser, *brake_cmd)
            time.sleep(period)
        stop_base(ser)


def suggest_duration_ms(target_deg: float, actual_deg: float, used_duration_ms: int) -> int:
    if abs(actual_deg) < 1e-6:
        return max(used_duration_ms + 80, int(used_duration_ms * 1.5), 1)
    deg_per_ms = abs(actual_deg) / max(used_duration_ms, 1)
    return max(1, int(round(abs(target_deg) / deg_per_ms)))


def error_report(target_deg: float, actual_deg: float, tolerance_pct: float) -> tuple[float, float, bool]:
    target = abs(float(target_deg))
    actual = abs(float(actual_deg))
    error_deg = actual - target
    error_pct = 0.0 if target <= 1e-9 else (error_deg / target) * 100.0
    return error_deg, error_pct, abs(error_pct) <= tolerance_pct


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


def ask_yes_no(prompt: str, default: bool) -> bool | None:
    default_text = "Y/n" if default else "y/N"
    text = input(f"{prompt} [{default_text}]: ").strip().lower()
    if text in {"q", "quit", "exit"}:
        return None
    if not text:
        return default
    return text in {"y", "yes"}


def format_cmd(cmd: list[float]) -> str:
    return " ".join(f"{name}={value:+.3f}" for name, value in zip(WHEEL_NAMES, cmd))


def main() -> None:
    ap = argparse.ArgumentParser(description="Tune in-place rotation using continuous direct wheel drive")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--direction", choices=["cw", "ccw"], default="cw")
    ap.add_argument("--real-target-deg", type=float, default=45.0)
    ap.add_argument("--speed", type=float, default=0.50)
    ap.add_argument("--duration-ms", type=int, default=0, help="0 uses conservative seed for direction/target")
    ap.add_argument("--boost-ms", type=int, default=120)
    ap.add_argument("--boost-speed", type=float, default=0.65)
    ap.add_argument("--brake-ms", type=int, default=0)
    ap.add_argument("--brake-scale", type=float, default=0.30)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--scales", default="", help="optional 4 numbers: FL FR RL RR")
    ap.add_argument("--tolerance-pct", type=float, default=10.0)
    ap.add_argument("--csv", default="scripts/drive_tests/rotation_duration_tuning_log.csv")
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
        "real_target_deg": args.real_target_deg,
        "speed": args.speed,
        "duration_ms": args.duration_ms if args.duration_ms > 0 else duration_seed(args.direction, args.real_target_deg),
        "boost_ms": args.boost_ms,
        "boost_speed": args.boost_speed,
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
    print(f"log: {csv_path}")
    print("Type q at prompts to quit.")

    trial = 1
    try:
        while True:
            direction = str(params["direction"])
            print(f"\n--- rotation trial {trial} ---")
            print(
                f"default scales for {direction}: "
                + " ".join(f"{name}={value:.2f}" for name, value in zip(WHEEL_NAMES, DEFAULT_SCALES[direction]))
            )

            text = input(f"direction cw/ccw [{direction}]: ").strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            if text:
                if text not in {"cw", "ccw"}:
                    print("direction must be cw or ccw")
                    continue
                params["direction"] = text
                direction = text
                params["scales"] = list(DEFAULT_SCALES[direction])
                params["duration_ms"] = duration_seed(direction, float(params["real_target_deg"]))

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

            target_deg = ask_float("real_target_deg", float(params["real_target_deg"]))
            if target_deg is None:
                break
            if abs(target_deg - float(params["real_target_deg"])) > 1e-9:
                params["duration_ms"] = duration_seed(direction, target_deg)
            params["real_target_deg"] = target_deg

            speed = ask_float("steady speed", float(params["speed"]))
            if speed is None:
                break
            boost_speed = ask_float("boost speed", float(params["boost_speed"]))
            if boost_speed is None:
                break
            boost_ms = ask_int("boost_ms", int(params["boost_ms"]))
            if boost_ms is None:
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
                    "speed": speed,
                    "boost_speed": boost_speed,
                    "boost_ms": boost_ms,
                    "duration_ms": duration_ms,
                    "brake_ms": brake_ms,
                    "brake_scale": brake_scale,
                }
            )

            scales = list(params["scales"])
            steady_cmd = wheel_cmd(direction, float(params["speed"]), scales)
            boost_cmd = wheel_cmd(direction, float(params["boost_speed"]), scales)
            print(f"steady command: {format_cmd(steady_cmd)}")
            print(f"boost command : {format_cmd(boost_cmd)}")
            print(
                "Check that all four wheels can freely rotate, then keep hands clear "
                "before running."
            )

            text = input("Press Enter to run, or q to quit: ").strip().lower()
            if text in {"q", "quit", "exit"}:
                break

            run_continuous_rotation(
                ser,
                steady_cmd=steady_cmd,
                boost_cmd=boost_cmd,
                duration_ms=int(params["duration_ms"]),
                boost_ms=int(params["boost_ms"]),
                brake_ms=int(params["brake_ms"]),
                brake_scale=float(params["brake_scale"]),
                hz=args.hz,
            )

            all_spun = ask_yes_no("did all 4 wheels spin", True)
            if all_spun is None:
                break
            actual_deg = ask_float("actual rotation deg magnitude", None)
            if actual_deg is None:
                break
            forward_drift_cm = ask_float("center drift cm (+forward / -backward)", 0.0)
            if forward_drift_cm is None:
                break
            right_drift_cm = ask_float("center drift cm (+right / -left)", 0.0)
            if right_drift_cm is None:
                break
            stalled_notes = input("stalled/weak wheel notes: ").strip()
            notes = input("notes: ").strip()

            suggested = suggest_duration_ms(
                float(params["real_target_deg"]),
                actual_deg,
                int(params["duration_ms"]),
            )
            error_deg, error_pct, passed = error_report(
                float(params["real_target_deg"]),
                actual_deg,
                args.tolerance_pct,
            )
            print(
                f"error: {error_deg:+.1f}deg ({error_pct:+.1f}%) "
                f"{'PASS' if passed else 'FAIL'} for {args.tolerance_pct:.0f}% target"
            )
            print(f"suggested next duration_ms: {suggested}")

            used_duration_ms = int(params["duration_ms"])
            if input("Use suggested duration next? [Y/n]: ").strip().lower() not in {"n", "no"}:
                params["duration_ms"] = suggested

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "trial": trial,
                "direction": direction,
                "real_target_deg": params["real_target_deg"],
                "duration_ms": used_duration_ms,
                "speed": params["speed"],
                "boost_ms": params["boost_ms"],
                "boost_speed": params["boost_speed"],
                "brake_ms": params["brake_ms"],
                "brake_scale": params["brake_scale"],
                "scale_fl": scales[0],
                "scale_fr": scales[1],
                "scale_rl": scales[2],
                "scale_rr": scales[3],
                "cmd_fl": f"{steady_cmd[0]:.3f}",
                "cmd_fr": f"{steady_cmd[1]:.3f}",
                "cmd_rl": f"{steady_cmd[2]:.3f}",
                "cmd_rr": f"{steady_cmd[3]:.3f}",
                "boost_fl": f"{boost_cmd[0]:.3f}",
                "boost_fr": f"{boost_cmd[1]:.3f}",
                "boost_rl": f"{boost_cmd[2]:.3f}",
                "boost_rr": f"{boost_cmd[3]:.3f}",
                "all_wheels_spun": all_spun,
                "actual_deg": actual_deg,
                "error_deg": f"{error_deg:.2f}",
                "error_pct": f"{error_pct:.2f}",
                "within_tolerance": passed,
                "forward_drift_cm": forward_drift_cm,
                "right_drift_cm": right_drift_cm,
                "suggested_duration_ms": suggested,
                "stalled_notes": stalled_notes,
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
