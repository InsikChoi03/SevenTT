#!/usr/bin/env python3
"""Continuous forward/backward distance tuner for the mecanum base.

This uses the same open-loop mechanism as the current lateral/rotation tuning:
direct wheel commands, optional high-power start boost, fixed duration, then
manual measurement and next-duration suggestion.

Wheel command signs:
    forward : FL=+ FR=+ RL=+ RR=+
    backward: FL=- FR=- RL=- RR=-
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
}
DEFAULT_SCALES = {
    "forward": [1.0, 1.0, 1.0, 1.0],
    "backward": [1.0, 1.0, 1.0, 1.0],
}
DEFAULT_DURATIONS_MS = {
    "forward": {10.0: 280, 60.0: 1450, 100.0: 2350},
    "backward": {10.0: 300, 60.0: 1550, 100.0: 2500},
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
    points = sorted(seeds.items())
    if target <= points[0][0]:
        return int(round(points[0][1] * target / points[0][0]))
    for (lo_cm, lo_ms), (hi_cm, hi_ms) in zip(points, points[1:]):
        if target <= hi_cm:
            ratio = (target - lo_cm) / (hi_cm - lo_cm)
            return int(round(lo_ms + ratio * (hi_ms - lo_ms)))
    lo_cm, lo_ms = points[-1]
    ratio = target / lo_cm
    return int(round(lo_ms * ratio))


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


def suggest_duration_ms(target_cm: float, actual_cm: float, used_duration_ms: int) -> int:
    if abs(actual_cm) < 1e-6:
        return max(used_duration_ms + 80, int(used_duration_ms * 1.5), 1)
    cm_per_ms = abs(actual_cm) / max(used_duration_ms, 1)
    return max(1, int(round(abs(target_cm) / cm_per_ms)))


def error_report(target_cm: float, actual_cm: float, tolerance_pct: float) -> tuple[float, float, bool]:
    target = abs(float(target_cm))
    actual = abs(float(actual_cm))
    error_cm = actual - target
    error_pct = 0.0 if target <= 1e-9 else (error_cm / target) * 100.0
    return error_cm, error_pct, abs(error_pct) <= tolerance_pct


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
    ap = argparse.ArgumentParser(description="Tune forward/backward distance using continuous drive duration")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--direction", choices=["forward", "backward"], default="forward")
    ap.add_argument("--target-cm", type=float, default=10.0)
    ap.add_argument("--speed", type=float, default=0.50)
    ap.add_argument("--duration-ms", type=int, default=0, help="0 uses conservative seed for direction/target")
    ap.add_argument("--boost-ms", type=int, default=100)
    ap.add_argument("--boost-speed", type=float, default=0.65)
    ap.add_argument("--brake-ms", type=int, default=0)
    ap.add_argument("--brake-scale", type=float, default=0.30)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--scales", default="", help="optional 4 numbers: FL FR RL RR")
    ap.add_argument("--tolerance-pct", type=float, default=10.0)
    ap.add_argument("--csv", default="scripts/drive_tests/linear_distance_tuning_log.csv")
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
            print(f"\n--- linear distance trial {trial} ---")
            print(
                f"default scales for {direction}: "
                + " ".join(f"{name}={value:.2f}" for name, value in zip(WHEEL_NAMES, DEFAULT_SCALES[direction]))
            )

            text = input(f"direction forward/backward [{direction}]: ").strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            if text:
                if text not in {"forward", "backward"}:
                    print("direction must be forward or backward")
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
            params["target_cm"] = target_cm

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
            print("Check the path is clear, then keep hands clear before running.")

            text = input("Press Enter to run, or q to quit: ").strip().lower()
            if text in {"q", "quit", "exit"}:
                break

            started = time.monotonic()
            run_continuous(
                ser,
                steady_cmd=steady_cmd,
                boost_cmd=boost_cmd,
                duration_ms=int(params["duration_ms"]),
                boost_ms=int(params["boost_ms"]),
                brake_ms=int(params["brake_ms"]),
                brake_scale=float(params["brake_scale"]),
                hz=args.hz,
            )
            elapsed_s = time.monotonic() - started

            all_spun = ask_yes_no("did all 4 wheels spin", True)
            if all_spun is None:
                break
            actual_cm = ask_float("actual forward/backward cm magnitude", None)
            if actual_cm is None:
                break
            right_drift_cm = ask_float("side drift cm (+right / -left)", 0.0)
            if right_drift_cm is None:
                break
            yaw_deg = ask_float("yaw_deg (+ccw / -cw)", 0.0)
            if yaw_deg is None:
                break
            stalled_notes = input("stalled/weak wheel notes: ").strip()
            notes = input("notes: ").strip()

            used_duration_ms = int(params["duration_ms"])
            next_duration_ms = suggest_duration_ms(target_cm, actual_cm, used_duration_ms)
            error_cm, error_pct, passed = error_report(target_cm, actual_cm, args.tolerance_pct)
            print(f"actual rate: {abs(actual_cm) / max(used_duration_ms, 1):.3f} cm/ms")
            print(
                f"error: {error_cm:+.2f}cm ({error_pct:+.1f}%) "
                f"{'PASS' if passed else 'FAIL'} for {args.tolerance_pct:.0f}% target"
            )
            print(f"suggested next duration for {target_cm:.1f} cm: {next_duration_ms} ms")

            if input("Use suggested duration next? [Y/n]: ").strip().lower() not in {"n", "no"}:
                params["duration_ms"] = next_duration_ms

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "trial": trial,
                "direction": direction,
                "target_cm": target_cm,
                "used_duration_ms": used_duration_ms,
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
                "actual_cm": actual_cm,
                "error_cm": f"{error_cm:.3f}",
                "error_pct": f"{error_pct:.2f}",
                "within_tolerance": passed,
                "right_drift_cm": right_drift_cm,
                "yaw_deg": yaw_deg,
                "suggested_next_duration_ms": next_duration_ms,
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
