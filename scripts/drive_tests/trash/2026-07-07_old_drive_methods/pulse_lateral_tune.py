#!/usr/bin/env python3
"""Interactive motor-balance tuner for pulsed mecanum lateral motion.

Goal:
- Tune right and left lateral motion separately.
- Keep the pulse method that worked on the heavy base.
- After each trial, enter measured forward/back drift and yaw error.
- The script suggests small per-wheel scale changes and can save them.
- It also suggests smaller pulse timing/speed when actual travel overshoots.

Wheel order is always:
    FL, FR, RL, RR

Positive measured drift means:
    forward_drift_cm > 0  -> robot moved forward
    forward_drift_cm < 0  -> robot moved backward
    yaw_deg > 0           -> robot rotated counter-clockwise
    yaw_deg < 0           -> robot rotated clockwise
"""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path
import sys
import time

import serial

from encoder_distance_test import (
    parse_index_list,
    parse_packet,
    reset_encoders,
    selected_motion_speed,
    send_base,
    stop_base,
)

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
LATERAL_SIGNS = {
    "right": [1.0, -1.0, -1.0, 1.0],
    "left": [-1.0, 1.0, 1.0, -1.0],
}
FORWARD_SIGNS = [1.0, 1.0, 1.0, 1.0]
CW_SIGNS = [1.0, -1.0, 1.0, -1.0]


def default_preset() -> dict[str, object]:
    return {
        "right": {"scales": [0.7, 0.7, 0.8, 0.7]},
        "left": {"scales": [1.0, 1.0, 1.0, 1.0]},
    }


def load_preset(path: Path) -> dict[str, object]:
    if not path.exists():
        return default_preset()
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    base = default_preset()
    for direction in ("right", "left"):
        if direction in data and "scales" in data[direction]:
            scales = [float(v) for v in data[direction]["scales"]]
            if len(scales) == 4:
                base[direction]["scales"] = scales
    return base


def save_preset(path: Path, preset: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(preset, f, indent=2, ensure_ascii=False)
        f.write("\n")


def wheel_cmd(direction: str, speed: float, scales: list[float]) -> list[float]:
    signs = LATERAL_SIGNS[direction]
    return [signs[i] * abs(float(speed)) * float(scales[i]) for i in range(4)]


def clamp_scales(scales: list[float], lo: float, hi: float) -> list[float]:
    return [max(lo, min(hi, float(v))) for v in scales]


def normalize_scales(scales: list[float]) -> list[float]:
    mean = sum(scales) / max(len(scales), 1)
    if mean <= 1e-9:
        return scales
    return [v / mean for v in scales]


def suggest_scales(
    direction: str,
    scales: list[float],
    *,
    forward_drift_cm: float,
    yaw_deg: float,
    drift_gain: float,
    yaw_gain: float,
    min_scale: float,
    max_scale: float,
) -> list[float]:
    """Suggest next per-wheel scales using small vector corrections.

    Command vector starts as lateral signs * scales. To cancel measured forward
    drift, add the opposite forward vector. To cancel measured yaw, add the
    opposite yaw by adding a clockwise vector for positive CCW error.
    """
    signs = LATERAL_SIGNS[direction]
    cmd = [signs[i] * scales[i] for i in range(4)]

    forward_correction = -drift_gain * forward_drift_cm
    yaw_correction = yaw_gain * yaw_deg

    for i in range(4):
        cmd[i] += forward_correction * FORWARD_SIGNS[i]
        cmd[i] += yaw_correction * CW_SIGNS[i]

    suggested = []
    for i in range(4):
        suggested.append(abs(cmd[i]) if signs[i] * cmd[i] > 0 else min_scale)
    suggested = normalize_scales(suggested)
    return clamp_scales(suggested, min_scale, max_scale)


def fmt_scales(scales: list[float]) -> str:
    return " ".join(f"{name}={value:.3f}" for name, value in zip(WHEEL_NAMES, scales))


def suggest_pulse_energy(
    *,
    target_m: float,
    actual_lateral_cm: float,
    pulse_on_ms: int,
    speed: float,
    min_on_ms: int,
    min_speed: float,
) -> tuple[int, float]:
    actual_m = abs(actual_lateral_cm) / 100.0
    target = abs(target_m)
    if actual_m <= 1e-6 or target <= 1e-6:
        return pulse_on_ms, speed

    ratio = max(0.05, min(1.5, target / actual_m))
    suggested_on = max(min_on_ms, int(round(pulse_on_ms * ratio)))

    if suggested_on == min_on_ms and ratio < 1.0:
        suggested_speed = max(min_speed, speed * max(0.2, ratio / max(min_on_ms / max(pulse_on_ms, 1), 1e-6)))
    else:
        suggested_speed = speed

    return suggested_on, suggested_speed


def parse_scale_line(text: str, current: list[float]) -> list[float]:
    if not text.strip():
        return current
    parts = text.replace(",", " ").split()
    if len(parts) != 4:
        raise ValueError("enter exactly 4 scale values: FL FR RL RR")
    return [float(v) for v in parts]


def ask_float(prompt: str, default: float | None = None) -> float | None:
    suffix = f" [{default}]" if default is not None else ""
    text = input(f"{prompt}{suffix}: ").strip()
    if text.lower() in {"q", "quit", "exit"}:
        return None
    if not text and default is not None:
        return float(default)
    return float(text)


def append_csv(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def run_trial(
    ser: serial.Serial,
    *,
    direction: str,
    speed: float,
    scales: list[float],
    pulse_on_ms: int,
    pulse_off_ms: int,
    target_m: float,
    timeout: float,
    hz: float,
    trusted_wheels: list[int],
    max_pulses: int,
) -> dict[str, object]:
    cmd = wheel_cmd(direction, speed, scales)
    odom_signs = LATERAL_SIGNS[direction]
    period = 1.0 / max(hz, 1e-6)
    on_ms = max(1, int(pulse_on_ms))
    off_ms = max(0, int(pulse_off_ms))
    cycle_ms = on_ms + off_ms
    start = time.monotonic()
    last_cmd = 0.0
    last_odom_t = None
    estimated_m = 0.0
    pulse_count = 0
    was_on = False
    rx_buf = b""
    reached = False

    reset_encoders(ser)
    print(f"cmd: [{cmd[0]:+.3f}, {cmd[1]:+.3f}, {cmd[2]:+.3f}, {cmd[3]:+.3f}]")

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed > timeout:
                print(f"timeout: estimated_lateral={estimated_m:.3f} m")
                break

            elapsed_ms = elapsed * 1000.0
            if max_pulses > 0 and elapsed_ms >= cycle_ms * max_pulses:
                print(f"max pulses reached: {max_pulses}")
                break
            pulse_on = (elapsed_ms % cycle_ms) < on_ms
            if now - last_cmd >= period:
                if pulse_on:
                    send_base(ser, *cmd)
                    if not was_on:
                        pulse_count += 1
                else:
                    stop_base(ser)
                was_on = pulse_on
                last_cmd = now

            data = ser.read(128)
            if data:
                rx_buf += data
            while b"\n" in rx_buf:
                raw, rx_buf = rx_buf.split(b"\n", 1)
                pkt = parse_packet(raw.strip().decode("ascii", errors="ignore"))
                if pkt is None:
                    continue
                tag, vals = pkt
                if tag != "ODOM":
                    continue
                t_now = time.monotonic()
                signed_speed = selected_motion_speed(vals, trusted_wheels, odom_signs)
                if last_odom_t is not None:
                    dt = max(0.0, min(0.5, t_now - last_odom_t))
                    estimated_m += signed_speed * dt
                last_odom_t = t_now
                print(f"odom lateral_speed={signed_speed:+.3f} estimated={estimated_m:+.3f} / {target_m:.3f} m")
                if abs(estimated_m) >= abs(target_m):
                    reached = True
                    print("target reached")
                    return {
                        "reached": reached,
                        "estimated_m": estimated_m,
                        "elapsed_s": time.monotonic() - start,
                        "pulse_count": pulse_count,
                    }
    finally:
        stop_base(ser)

    return {
        "reached": reached,
        "estimated_m": estimated_m,
        "elapsed_s": time.monotonic() - start,
        "pulse_count": pulse_count,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Tune per-wheel pulse scales for left/right lateral motion")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--direction", choices=["right", "left"], default="right")
    ap.add_argument("--speed", type=float, default=0.50)
    ap.add_argument("--pulse-on-ms", type=int, default=150)
    ap.add_argument("--pulse-off-ms", type=int, default=50)
    ap.add_argument("--max-pulses", type=int, default=1)
    ap.add_argument("--target-m", type=float, default=0.10)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--odom-wheels", default="1,2,3")
    ap.add_argument("--preset", default="scripts/drive_tests/lateral_motor_scales.json")
    ap.add_argument("--csv", default="scripts/drive_tests/lateral_motor_tuning_v2_log.csv")
    ap.add_argument("--drift-gain", type=float, default=0.010, help="scale correction per cm of forward drift")
    ap.add_argument("--yaw-gain", type=float, default=0.012, help="scale correction per degree of CCW yaw")
    ap.add_argument("--min-scale", type=float, default=0.60)
    ap.add_argument("--max-scale", type=float, default=1.40)
    ap.add_argument("--min-pulse-on-ms", type=int, default=30)
    ap.add_argument("--min-speed", type=float, default=0.20)
    args = ap.parse_args()

    preset_path = Path(args.preset)
    csv_path = Path(args.csv)
    preset = load_preset(preset_path)
    trusted_wheels = parse_index_list(args.odom_wheels)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print(f"port open failed: {e}", file=sys.stderr)
        sys.exit(1)

    params: dict[str, object] = {
        "direction": args.direction,
        "speed": args.speed,
        "pulse_on_ms": args.pulse_on_ms,
        "pulse_off_ms": args.pulse_off_ms,
        "max_pulses": args.max_pulses,
        "target_m": args.target_m,
        "timeout": args.timeout,
    }

    print(f"opened {args.port} @ {args.baud}")
    print("waiting for MCU reboot...")
    time.sleep(2.0)
    ser.reset_input_buffer()
    stop_base(ser)
    reset_encoders(ser)
    print(f"preset: {preset_path}")
    print(f"log: {csv_path}")
    print("Type q at most prompts to quit.")

    trial_no = 1
    try:
        while True:
            direction = str(params["direction"])
            scales = list(preset[direction]["scales"])
            print(f"\n--- trial {trial_no} ---")
            print(f"current {direction} scales: {fmt_scales(scales)}")

            text = input(f"direction right/left [{direction}]: ").strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            if text:
                if text not in {"right", "left"}:
                    print("direction must be right or left")
                    continue
                params["direction"] = text
                direction = text
                scales = list(preset[direction]["scales"])

            try:
                text = input(f"wheel scales FL FR RL RR [{fmt_scales(scales)}]: ").strip()
                if text.lower() in {"q", "quit", "exit"}:
                    break
                scales = clamp_scales(parse_scale_line(text, scales), args.min_scale, args.max_scale)
            except ValueError as e:
                print(e)
                continue

            speed = ask_float("speed", float(params["speed"]))
            if speed is None:
                break
            pulse_on_ms = ask_float("pulse_on_ms", float(params["pulse_on_ms"]))
            if pulse_on_ms is None:
                break
            pulse_off_ms = ask_float("pulse_off_ms", float(params["pulse_off_ms"]))
            if pulse_off_ms is None:
                break
            max_pulses = ask_float("max_pulses", float(params["max_pulses"]))
            if max_pulses is None:
                break
            target_m = ask_float("target_m", float(params["target_m"]))
            if target_m is None:
                break
            timeout = ask_float("timeout_s", float(params["timeout"]))
            if timeout is None:
                break

            params.update(
                {
                    "speed": speed,
                    "pulse_on_ms": int(pulse_on_ms),
                    "pulse_off_ms": int(pulse_off_ms),
                    "max_pulses": int(max_pulses),
                    "target_m": target_m,
                    "timeout": timeout,
                }
            )
            trial_speed = float(params["speed"])
            trial_pulse_on_ms = int(params["pulse_on_ms"])
            trial_pulse_off_ms = int(params["pulse_off_ms"])
            trial_max_pulses = int(params["max_pulses"])
            trial_target_m = float(params["target_m"])
            trial_timeout = float(params["timeout"])

            print(f"trial scales: {fmt_scales(scales)}")
            text = input("Press Enter to run, s to save scales only, or q to quit: ").strip().lower()
            if text in {"q", "quit", "exit"}:
                break
            if text == "s":
                preset[direction]["scales"] = scales
                save_preset(preset_path, preset)
                print(f"saved {direction} scales")
                continue

            result = run_trial(
                ser,
                direction=direction,
                speed=trial_speed,
                scales=scales,
                pulse_on_ms=trial_pulse_on_ms,
                pulse_off_ms=trial_pulse_off_ms,
                target_m=trial_target_m,
                timeout=trial_timeout,
                hz=args.hz,
                trusted_wheels=trusted_wheels,
                max_pulses=trial_max_pulses,
            )

            print("Measured feedback:")
            actual_lateral_cm = ask_float("actual lateral cm", None)
            if actual_lateral_cm is None:
                break
            forward_drift_cm = ask_float("forward_drift_cm (+forward, -back)", 0.0)
            if forward_drift_cm is None:
                break
            yaw_deg = ask_float("yaw_deg (+ccw, -cw)", 0.0)
            if yaw_deg is None:
                break
            notes = input("notes: ").strip()

            suggested = suggest_scales(
                direction,
                scales,
                forward_drift_cm=float(forward_drift_cm),
                yaw_deg=float(yaw_deg),
                drift_gain=args.drift_gain,
                yaw_gain=args.yaw_gain,
                min_scale=args.min_scale,
                max_scale=args.max_scale,
            )
            suggested_on_ms, suggested_speed = suggest_pulse_energy(
                target_m=trial_target_m,
                actual_lateral_cm=float(actual_lateral_cm),
                pulse_on_ms=trial_pulse_on_ms,
                speed=trial_speed,
                min_on_ms=args.min_pulse_on_ms,
                min_speed=args.min_speed,
            )
            print(f"suggested next {direction} scales: {fmt_scales(suggested)}")
            print(
                "suggested stop/energy: "
                f"speed={suggested_speed:.3f}, pulse_on_ms={suggested_on_ms}, "
                f"max_pulses={params['max_pulses']}"
            )
            accept = input("Use suggested scales for next trial? [y/N]: ").strip().lower()
            next_scales = suggested if accept in {"y", "yes"} else scales
            preset[direction]["scales"] = next_scales
            save_preset(preset_path, preset)
            accept_energy = input("Use suggested speed/pulse_on for next trial? [y/N]: ").strip().lower()
            if accept_energy in {"y", "yes"}:
                params["speed"] = suggested_speed
                params["pulse_on_ms"] = suggested_on_ms

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "trial": trial_no,
                "direction": direction,
                "speed": trial_speed,
                "pulse_on_ms": trial_pulse_on_ms,
                "pulse_off_ms": trial_pulse_off_ms,
                "max_pulses": trial_max_pulses,
                "target_m": trial_target_m,
                "timeout_s": trial_timeout,
                "scale_fl": f"{scales[0]:.4f}",
                "scale_fr": f"{scales[1]:.4f}",
                "scale_rl": f"{scales[2]:.4f}",
                "scale_rr": f"{scales[3]:.4f}",
                "reached": result["reached"],
                "estimated_m": f"{float(result['estimated_m']):.4f}",
                "elapsed_s": f"{float(result['elapsed_s']):.3f}",
                "pulse_count": result["pulse_count"],
                "actual_lateral_cm": actual_lateral_cm,
                "forward_drift_cm": forward_drift_cm,
                "yaw_deg": yaw_deg,
                "suggest_fl": f"{suggested[0]:.4f}",
                "suggest_fr": f"{suggested[1]:.4f}",
                "suggest_rl": f"{suggested[2]:.4f}",
                "suggest_rr": f"{suggested[3]:.4f}",
                "suggest_speed": f"{suggested_speed:.4f}",
                "suggest_pulse_on_ms": suggested_on_ms,
                "accepted_suggestion": accept in {"y", "yes"},
                "accepted_energy": accept_energy in {"y", "yes"},
                "notes": notes,
            }
            append_csv(csv_path, row)
            print(f"saved trial {trial_no}")
            trial_no += 1
    finally:
        for _ in range(5):
            stop_base(ser)
            time.sleep(0.02)
        ser.flush()
        ser.close()
        print("stopped and closed")


if __name__ == "__main__":
    main()
