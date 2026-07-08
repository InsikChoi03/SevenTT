#!/usr/bin/env python3
"""Unified motion tuner for forward/backward/lateral/rotation moves.

This script is for empirical tuning, not autonomous route execution.
Every motion uses the same low-level pattern:

1. optional start boost
2. fixed-duration continuous wheel command
3. optional reverse brake
4. manual measurement
5. suggested next duration

You can adjust per-motion, per-wheel strength with:
    scales FL FR RL RR
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
    "forward": [1.0, 1.0, 1.0, 1.0],
    "backward": [-1.0, -1.0, -1.0, -1.0],
    "right": [1.0, -1.0, -1.0, 1.0],
    "left": [-1.0, 1.0, 1.0, -1.0],
    "cw": [1.0, -1.0, 1.0, -1.0],
    "ccw": [-1.0, 1.0, -1.0, 1.0],
}

KINDS = {
    "forward": "distance",
    "backward": "distance",
    "right": "distance",
    "left": "distance",
    "cw": "angle",
    "ccw": "angle",
}

DEFAULT_SCALES = {
    "forward": [1.0, 1.0, 1.0, 1.0],
    "backward": [1.0, 1.0, 1.0, 1.0],
    "right": [0.7, 0.7, 0.8, 0.7],
    "left": [0.65, 0.75, 0.75, 0.65],
    "cw": [1.0, 1.0, 1.0, 1.0],
    "ccw": [1.0, 1.0, 1.0, 1.0],
}

# Seeds are empirical and intentionally conservative. Refine them by accepting
# suggested duration values after each trial.
DEFAULT_DURATIONS_MS = {
    "forward": {10.0: 196, 100.0: 1222},
    "backward": {10.0: 176, 100.0: 1313},
    "right": {10.0: 414, 100.0: 3041},
    "left": {10.0: 401, 100.0: 3410},
    "cw": {45.0: 100, 90.0: 230, 180.0: 665},
    "ccw": {45.0: 100, 90.0: 230, 180.0: 665},
}

DEFAULT_PARAMS = {
    "forward": {"speed": 0.50, "boost_speed": 0.65, "boost_ms": 100, "brake_ms": 150, "brake_scale": 0.50},
    "backward": {"speed": 0.50, "boost_speed": 0.65, "boost_ms": 100, "brake_ms": 150, "brake_scale": 0.60},
    "right": {"speed": 0.50, "boost_speed": 0.65, "boost_ms": 0, "brake_ms": 0, "brake_scale": 0.35},
    "left": {"speed": 0.50, "boost_speed": 0.65, "boost_ms": 0, "brake_ms": 0, "brake_scale": 0.35},
    "cw": {"speed": 0.50, "boost_speed": 0.65, "boost_ms": 120, "brake_ms": 0, "brake_scale": 0.30},
    "ccw": {"speed": 0.50, "boost_speed": 0.65, "boost_ms": 120, "brake_ms": 0, "brake_scale": 0.30},
}

DEFAULT_TARGETS = {
    "distance": 10.0,
    "angle": 45.0,
}


def clamp_scale(value: float) -> float:
    return max(0.20, min(1.50, float(value)))


def wheel_cmd(direction: str, speed: float, scales: list[float]) -> list[float]:
    return [SIGNS[direction][i] * abs(float(speed)) * scales[i] for i in range(4)]


def format_cmd(cmd: list[float]) -> str:
    return " ".join(f"{name}={value:+.3f}" for name, value in zip(WHEEL_NAMES, cmd))


def format_scales(scales: list[float]) -> str:
    return " ".join(f"{value:.3f}" for value in scales)


def parse_scales(text: str, direction: str) -> list[float]:
    if not text.strip():
        return list(DEFAULT_SCALES[direction])
    parts = text.replace(",", " ").split()
    if len(parts) != 4:
        raise ValueError("scales must be 4 numbers: FL FR RL RR")
    return [clamp_scale(float(v)) for v in parts]


def append_csv(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def duration_seed(direction: str, target: float) -> int:
    seeds = DEFAULT_DURATIONS_MS[direction]
    magnitude = abs(float(target))
    points = sorted(seeds.items())
    if magnitude <= points[0][0]:
        return max(1, int(round(points[0][1] * magnitude / points[0][0])))
    for (lo_target, lo_ms), (hi_target, hi_ms) in zip(points, points[1:]):
        if magnitude <= hi_target:
            ratio = (magnitude - lo_target) / (hi_target - lo_target)
            return max(1, int(round(lo_ms + ratio * (hi_ms - lo_ms))))
    hi_target, hi_ms = points[-1]
    return max(1, int(round(hi_ms * magnitude / hi_target)))


def suggest_duration_ms(target: float, actual: float, used_duration_ms: int) -> int:
    if abs(actual) < 1e-6:
        return max(used_duration_ms + 80, int(used_duration_ms * 1.5), 1)
    units_per_ms = abs(actual) / max(used_duration_ms, 1)
    return max(1, int(round(abs(target) / units_per_ms)))


def error_report(target: float, actual: float, tolerance_pct: float) -> tuple[float, float, bool]:
    target_mag = abs(float(target))
    actual_mag = abs(float(actual))
    error = actual_mag - target_mag
    error_pct = 0.0 if target_mag <= 1e-9 else (error / target_mag) * 100.0
    return error, error_pct, abs(error_pct) <= tolerance_pct


def run_continuous(
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


def suggest_linear_scales(direction: str, scales: list[float], yaw_deg: float, step: float) -> list[float] | None:
    if direction not in {"forward", "backward"} or abs(yaw_deg) < 1e-6:
        return None

    next_scales = list(scales)
    if direction == "forward":
        if yaw_deg > 0.0:
            weak_indices = (1, 3)  # reduce right side
        else:
            weak_indices = (0, 2)  # reduce left side
    else:
        if yaw_deg > 0.0:
            weak_indices = (0, 2)  # backward yaw correction is mirrored
        else:
            weak_indices = (1, 3)

    for idx in weak_indices:
        next_scales[idx] = clamp_scale(next_scales[idx] - abs(step))
    return next_scales


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


def prompt_direction(current: str) -> str | None:
    text = input(f"direction forward/backward/right/left/cw/ccw [{current}]: ").strip().lower()
    if text in {"q", "quit", "exit"}:
        return None
    if not text:
        return current
    if text not in SIGNS:
        print("direction must be one of: forward backward right left cw ccw")
        return current
    return text


def main() -> None:
    ap = argparse.ArgumentParser(description="Unified mecanum motion tuner")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--direction", choices=list(SIGNS), default="forward")
    ap.add_argument("--target-cm", type=float, default=0.0, help="0 uses default target for distance motions")
    ap.add_argument("--target-deg", type=float, default=0.0, help="0 uses default target for rotation motions")
    ap.add_argument("--duration-ms", type=int, default=0, help="0 uses empirical seed for direction/target")
    ap.add_argument("--speed", type=float, default=-1.0, help="negative uses per-direction default")
    ap.add_argument("--boost-ms", type=int, default=-1, help="negative uses per-direction default")
    ap.add_argument("--boost-speed", type=float, default=-1.0, help="negative uses per-direction default")
    ap.add_argument("--brake-ms", type=int, default=-1, help="negative uses per-direction default")
    ap.add_argument("--brake-scale", type=float, default=-1.0, help="negative uses per-direction default")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--scales", default="", help="optional 4 numbers: FL FR RL RR")
    ap.add_argument("--scale-step", type=float, default=0.02)
    ap.add_argument("--tolerance-pct", type=float, default=10.0)
    ap.add_argument("--csv", default="scripts/drive_tests/motion_tuning_log.csv")
    args = ap.parse_args()

    direction = args.direction
    try:
        scales = parse_scales(args.scales, direction)
    except ValueError as e:
        print(e, file=sys.stderr)
        sys.exit(2)

    kind = KINDS[direction]
    target = args.target_deg if kind == "angle" and args.target_deg > 0.0 else args.target_cm
    if target <= 0.0:
        target = DEFAULT_TARGETS[kind]

    defaults = DEFAULT_PARAMS[direction]
    params: dict[str, object] = {
        "direction": direction,
        "target": target,
        "duration_ms": args.duration_ms if args.duration_ms > 0 else duration_seed(direction, target),
        "speed": defaults["speed"] if args.speed < 0.0 else args.speed,
        "boost_ms": defaults["boost_ms"] if args.boost_ms < 0 else args.boost_ms,
        "boost_speed": defaults["boost_speed"] if args.boost_speed < 0.0 else args.boost_speed,
        "brake_ms": defaults["brake_ms"] if args.brake_ms < 0 else args.brake_ms,
        "brake_scale": defaults["brake_scale"] if args.brake_scale < 0.0 else args.brake_scale,
        "scales": scales,
    }

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"port open failed: {e}", file=sys.stderr)
        sys.exit(1)

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
            kind = KINDS[direction]
            unit = "deg" if kind == "angle" else "cm"
            print(f"\n--- motion trial {trial} ---")
            print(
                f"default scales for {direction}: "
                + " ".join(f"{name}={value:.3f}" for name, value in zip(WHEEL_NAMES, DEFAULT_SCALES[direction]))
            )

            new_direction = prompt_direction(direction)
            if new_direction is None:
                break
            if new_direction != direction:
                direction = new_direction
                kind = KINDS[direction]
                unit = "deg" if kind == "angle" else "cm"
                defaults = DEFAULT_PARAMS[direction]
                params.update(
                    {
                        "direction": direction,
                        "target": DEFAULT_TARGETS[kind],
                        "duration_ms": duration_seed(direction, DEFAULT_TARGETS[kind]),
                        "speed": defaults["speed"],
                        "boost_ms": defaults["boost_ms"],
                        "boost_speed": defaults["boost_speed"],
                        "brake_ms": defaults["brake_ms"],
                        "brake_scale": defaults["brake_scale"],
                        "scales": list(DEFAULT_SCALES[direction]),
                    }
                )

            text = input(
                "scales FL FR RL RR "
                f"[{format_scales(list(params['scales']))}]: "
            ).strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            try:
                if text:
                    params["scales"] = parse_scales(text, direction)
            except ValueError as e:
                print(e)
                continue

            target = ask_float(f"target_{unit}", float(params["target"]))
            if target is None:
                break
            if abs(target - float(params["target"])) > 1e-9:
                params["duration_ms"] = duration_seed(direction, target)
            params["target"] = target

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
            steady_cmd = wheel_cmd(direction, speed, scales)
            boost_cmd = wheel_cmd(direction, boost_speed, scales)
            print(f"steady command: {format_cmd(steady_cmd)}")
            if boost_ms > 0:
                print(f"boost command : {format_cmd(boost_cmd)}")
            print("Check the path is clear, then keep hands clear before running.")

            text = input("Press Enter to run, or q to quit: ").strip().lower()
            if text in {"q", "quit", "exit"}:
                break

            started = time.monotonic()
            run_continuous(
                ser,
                steady_cmd=steady_cmd,
                boost_cmd=boost_cmd,
                duration_ms=duration_ms,
                boost_ms=boost_ms,
                brake_ms=brake_ms,
                brake_scale=brake_scale,
                hz=args.hz,
            )
            elapsed_s = time.monotonic() - started

            all_spun = ask_yes_no("did all 4 wheels spin", True)
            if all_spun is None:
                break

            if kind == "angle":
                actual = ask_float("actual rotation deg magnitude", None)
                if actual is None:
                    break
                forward_drift = ask_float("center drift cm (+forward / -backward)", 0.0)
                if forward_drift is None:
                    break
                right_drift = ask_float("center drift cm (+right / -left)", 0.0)
                if right_drift is None:
                    break
                yaw_deg = 0.0
            else:
                actual = ask_float(f"actual {unit} magnitude", None)
                if actual is None:
                    break
                if direction in {"forward", "backward"}:
                    right_drift = ask_float("side drift cm (+right / -left)", 0.0)
                    if right_drift is None:
                        break
                    forward_drift = 0.0
                else:
                    forward_drift = ask_float("forward/back drift cm (+forward / -backward)", 0.0)
                    if forward_drift is None:
                        break
                    right_drift = 0.0
                yaw_deg = ask_float("yaw_deg (+ccw / -cw)", 0.0)
                if yaw_deg is None:
                    break

            stalled_notes = input("stalled/weak wheel notes: ").strip()
            notes = input("notes: ").strip()

            next_duration_ms = suggest_duration_ms(target, actual, duration_ms)
            error, error_pct, passed = error_report(target, actual, args.tolerance_pct)
            rate_label = "deg/ms" if kind == "angle" else "cm/ms"
            print(f"actual rate: {abs(actual) / max(duration_ms, 1):.3f} {rate_label}")
            print(
                f"error: {error:+.2f}{unit} ({error_pct:+.1f}%) "
                f"{'PASS' if passed else 'FAIL'} for {args.tolerance_pct:.0f}% target"
            )
            print(f"suggested next duration for {target:.1f}{unit}: {next_duration_ms} ms")

            suggested_scales = suggest_linear_scales(direction, scales, yaw_deg, args.scale_step)
            if suggested_scales is not None:
                print(
                    "conservative yaw scale suggestion: "
                    f"{format_scales(suggested_scales)}"
                )

            if input("Use suggested duration next? [Y/n]: ").strip().lower() not in {"n", "no"}:
                params["duration_ms"] = next_duration_ms
            if suggested_scales is not None:
                use_scales = ask_yes_no("Use suggested scales next", False)
                if use_scales is None:
                    break
                if use_scales:
                    params["scales"] = suggested_scales

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "trial": trial,
                "direction": direction,
                "kind": kind,
                "target": target,
                "unit": unit,
                "used_duration_ms": duration_ms,
                "speed": speed,
                "boost_ms": boost_ms,
                "boost_speed": boost_speed,
                "brake_ms": brake_ms,
                "brake_scale": brake_scale,
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
                "elapsed_s": f"{elapsed_s:.3f}",
                "all_wheels_spun": all_spun,
                "actual": actual,
                "error": f"{error:.3f}",
                "error_pct": f"{error_pct:.2f}",
                "within_tolerance": passed,
                "forward_drift_cm": forward_drift,
                "right_drift_cm": right_drift,
                "yaw_deg": yaw_deg,
                "suggested_next_duration_ms": next_duration_ms,
                "suggest_scale_fl": "" if suggested_scales is None else f"{suggested_scales[0]:.3f}",
                "suggest_scale_fr": "" if suggested_scales is None else f"{suggested_scales[1]:.3f}",
                "suggest_scale_rl": "" if suggested_scales is None else f"{suggested_scales[2]:.3f}",
                "suggest_scale_rr": "" if suggested_scales is None else f"{suggested_scales[3]:.3f}",
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
