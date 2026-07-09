#!/usr/bin/env python3
"""Encoder validation tests for higher-level closed-loop base control.

Run this while the normal ROS base path is alive:
  1. base_controller_node subscribes to /base_command
  2. mcu_bridge_base_node publishes /base/wheel_odom

The script has two practical stages:
  repeat-open-loop: send the same command for a fixed duration across several trials, then
                    compare encoder distance with manually measured distance.
  closed-loop-distance: drive until encoder odometry reaches a target, then measure the real result.

These are intentionally ROS-level tests: commands pass through base_controller_node, so wheel_min,
slew, brake, and wheel_scales match the main driving pipeline.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand
from std_msgs.msg import Float32MultiArray


DIRECTIONS = ("forward", "backward", "left", "right", "cw", "ccw")


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


def direction_to_base_command(direction: str, speed: float) -> tuple[float, float, float]:
    mag = abs(float(speed))
    if direction == "forward":
        return mag, 0.0, 0.0
    if direction == "backward":
        return -mag, 0.0, 0.0
    if direction == "left":
        return 0.0, mag, 0.0
    if direction == "right":
        return 0.0, -mag, 0.0
    if direction == "ccw":
        return 0.0, 0.0, mag
    if direction == "cw":
        return 0.0, 0.0, -mag
    raise ValueError(f"unsupported direction: {direction}")


def direction_odom_signs(direction: str) -> list[float]:
    if direction == "forward":
        return [1.0, 1.0, 1.0, 1.0]
    if direction == "backward":
        return [-1.0, -1.0, -1.0, -1.0]
    if direction == "right":
        return [1.0, -1.0, -1.0, 1.0]
    if direction == "left":
        return [-1.0, 1.0, 1.0, -1.0]
    if direction == "ccw":
        return [-1.0, 1.0, -1.0, 1.0]
    if direction == "cw":
        return [1.0, -1.0, 1.0, -1.0]
    raise ValueError(f"unsupported direction: {direction}")


def selected_motion_speed(vals: list[float], wheels: list[int], signs: list[float]) -> float:
    samples = [float(signs[i]) * float(vals[i]) for i in wheels if abs(float(signs[i])) > 1e-9]
    if not samples:
        return 0.0
    if len(samples) == 1:
        return samples[0]
    if len(samples) == 2:
        return 0.5 * (samples[0] + samples[1])
    return float(statistics.median(samples))


def append_csv(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            writer.writeheader()
            writer.writerow(row)
        return

    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fields = list(reader.fieldnames or [])
    for key in row.keys():
        if key not in fields:
            fields.append(key)
    rows.append({key: row.get(key, "") for key in fields})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def ask_choice(prompt: str, choices: tuple[str, ...], default: str) -> str:
    text = input(f"{prompt} ({'/'.join(choices)}) [{default}]: ").strip()
    if not text:
        return default
    if text not in choices:
        raise ValueError(f"invalid choice {text!r}; expected one of {choices}")
    return text


def ask_int(prompt: str, default: int) -> int:
    text = input(f"{prompt} [{default}]: ").strip()
    return int(text) if text else default


def ask_float(prompt: str, default: float) -> float:
    text = input(f"{prompt} [{default}]: ").strip()
    return float(text) if text else default


def ask_optional_float(prompt: str) -> float | None:
    text = input(f"{prompt}; blank to skip: ").strip()
    return float(text) if text else None


class EncoderControlValidation(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("encoder_control_validation")
        self.args = args
        self.odom_wheels = parse_index_list(args.odom_wheels)
        self.odom_signs = direction_odom_signs(args.direction)
        self.rotational = args.direction in {"cw", "ccw"}
        self.progress = 0.0
        self.last_odom_t: float | None = None
        self.last_speed = 0.0
        self.saw_odom = False

        self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)
        self.create_subscription(Float32MultiArray, "/base/wheel_odom", self.on_odom, 20)

    def reset_trial(self) -> None:
        self.progress = 0.0
        self.last_odom_t = None
        self.last_speed = 0.0
        self.saw_odom = False

    def on_odom(self, msg: Float32MultiArray) -> None:
        vals = [float(v) for v in msg.data[:4]]
        if len(vals) < 4:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        signed_speed = selected_motion_speed(vals, self.odom_wheels, self.odom_signs)
        if self.last_odom_t is not None:
            dt = max(0.0, min(0.5, now - self.last_odom_t))
            if self.rotational:
                self.progress += (signed_speed / max(float(self.args.k), 1e-6)) * dt
            else:
                self.progress += signed_speed * dt
        self.last_odom_t = now
        self.last_speed = signed_speed
        self.saw_odom = True

    def publish_move(self) -> None:
        vx, vy, omega = direction_to_base_command(self.args.direction, self.args.speed)
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.omega = float(omega)
        self.pub_cmd.publish(msg)

    def publish_stop(self) -> None:
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(msg)

    def command_subscriber_count(self) -> int:
        return self.count_subscribers("/base_command")

    def odom_publisher_count(self) -> int:
        return self.count_publishers("/base/wheel_odom")


def wait_for_graph(node: EncoderControlValidation) -> None:
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.command_subscriber_count() > 0 and node.odom_publisher_count() > 0:
            return
    if node.command_subscriber_count() <= 0:
        print("WARNING: /base_command has no subscribers. Start base_controller_node first.")
    if node.odom_publisher_count() <= 0:
        print("WARNING: /base/wheel_odom has no publishers. Start mcu_bridge_base_node first.")


def run_for_duration(node: EncoderControlValidation, duration_ms: int, hz: float) -> None:
    period = 1.0 / max(hz, 1e-6)
    end_at = time.monotonic() + max(0.001, duration_ms / 1000.0)
    while time.monotonic() < end_at:
        node.publish_move()
        rclpy.spin_once(node, timeout_sec=period)
        print_progress(node)
    stop_and_settle(node)


def run_until_encoder_target(
    node: EncoderControlValidation,
    *,
    target: float,
    timeout_sec: float,
    hz: float,
) -> str:
    period = 1.0 / max(hz, 1e-6)
    start = time.monotonic()
    while True:
        if abs(node.progress) >= abs(target):
            stop_and_settle(node)
            return "target"
        if time.monotonic() - start >= timeout_sec:
            stop_and_settle(node)
            return "timeout"
        node.publish_move()
        rclpy.spin_once(node, timeout_sec=period)
        print_progress(node)


def stop_and_settle(node: EncoderControlValidation) -> None:
    for _ in range(10):
        node.publish_stop()
        rclpy.spin_once(node, timeout_sec=0.02)


def print_progress(node: EncoderControlValidation) -> None:
    if node.rotational:
        print(
            f"odom speed={node.last_speed:+.3f} angle={math.degrees(node.progress):+.1f} deg",
            end="\r",
            flush=True,
        )
    else:
        print(
            f"odom speed={node.last_speed:+.3f} distance={node.progress:+.3f} m",
            end="\r",
            flush=True,
        )


def actual_prompt(rotational: bool) -> tuple[float | None, str]:
    if rotational:
        return ask_optional_float("Measured actual rotation (deg)"), "deg"
    return ask_optional_float("Measured actual travel (m)"), "m"


def odom_value(node: EncoderControlValidation) -> tuple[float, str]:
    if node.rotational:
        return math.degrees(node.progress), "deg"
    return node.progress, "m"


def log_trial(
    args: argparse.Namespace,
    *,
    stage: str,
    trial: int,
    stop_reason: str,
    odom: float,
    actual: float | None,
    unit: str,
) -> float | None:
    scale = None
    if actual is not None and abs(odom) > 1e-9:
        scale = actual / odom
    append_csv(
        Path(args.log_csv),
        {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "stage": stage,
            "trial": trial,
            "direction": args.direction,
            "speed": args.speed,
            "duration_ms": args.duration_ms if stage == "repeat-open-loop" else "",
            "target": args.target_deg if unit == "deg" else args.target_m,
            "timeout_sec": args.timeout_sec,
            "odom_wheels": args.odom_wheels,
            "stop_reason": stop_reason,
            "odom_value": odom,
            "actual_value": "" if actual is None else actual,
            "scale_actual_over_odom": "" if scale is None else scale,
            "unit": unit,
        },
    )
    return scale


def summarize_scales(scales: list[float]) -> None:
    if not scales:
        return
    med = statistics.median(scales)
    avg = statistics.mean(scales)
    spread = (max(scales) - min(scales)) / max(abs(med), 1e-9) * 100.0
    print("\nscale summary")
    print(f"  trials: {len(scales)}")
    print(f"  mean:   {avg:.4f}")
    print(f"  median: {med:.4f}")
    print(f"  spread: {spread:.1f}% of median")
    if spread <= 15.0:
        print("  verdict: stable enough to consider odom_scale calibration")
    else:
        print("  verdict: too unstable; fix drive repeatability before applying odom_scale")


def configure_interactive(args: argparse.Namespace) -> None:
    if args.no_prompt:
        return
    print("Enter test values. Press Enter to keep the default.")
    args.stage = ask_choice("Stage", ("repeat-open-loop", "closed-loop-distance"), args.stage)
    args.direction = ask_choice("Direction", DIRECTIONS, args.direction)
    args.speed = ask_float("Command speed", args.speed)
    if args.stage == "repeat-open-loop":
        args.duration_ms = ask_int("Drive duration ms", args.duration_ms)
        args.trials = ask_int("Trials", args.trials)
    else:
        if args.direction in {"cw", "ccw"}:
            args.target_deg = ask_float("Encoder target deg", args.target_deg)
        else:
            args.target_m = ask_float("Encoder target m", args.target_m)
        args.timeout_sec = ask_float("Timeout sec", args.timeout_sec)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", choices=["repeat-open-loop", "closed-loop-distance"], default="repeat-open-loop")
    ap.add_argument("--direction", choices=DIRECTIONS, default="forward")
    ap.add_argument("--speed", type=float, default=0.30)
    ap.add_argument("--duration-ms", type=int, default=1000)
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--target-m", type=float, default=0.30)
    ap.add_argument("--target-deg", type=float, default=90.0)
    ap.add_argument("--timeout-sec", type=float, default=5.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--k", type=float, default=0.20, help="mecanum k = lx + ly for rotation integration")
    ap.add_argument("--odom-wheels", default="1,2,3", help="trusted odom wheels; default ignores FL")
    ap.add_argument("--log-csv", default="scripts/drive_tests/encoder_control_validation_log.csv")
    ap.add_argument("--no-prompt", action="store_true")
    args = ap.parse_args()
    configure_interactive(args)

    rclpy.init()
    node = EncoderControlValidation(args)
    wait_for_graph(node)
    scales: list[float] = []
    try:
        if args.stage == "repeat-open-loop":
            for trial in range(1, max(1, args.trials) + 1):
                input(f"\nTrial {trial}/{args.trials}: place robot at start, then press Enter...")
                node.reset_trial()
                run_for_duration(node, args.duration_ms, args.hz)
                odom, unit = odom_value(node)
                print(f"\nRESULT trial={trial} odom_{unit}={odom:+.4f}")
                actual, _unit = actual_prompt(node.rotational)
                scale = log_trial(
                    args,
                    stage=args.stage,
                    trial=trial,
                    stop_reason="duration",
                    odom=odom,
                    actual=actual,
                    unit=unit,
                )
                if scale is not None:
                    scales.append(scale)
                    print(f"scale_actual_over_odom={scale:.4f}")
            summarize_scales(scales)
        else:
            node.reset_trial()
            target = math.radians(abs(args.target_deg)) if node.rotational else abs(args.target_m)
            print(f"Driving until encoder target: {args.target_deg if node.rotational else args.target_m}")
            stop_reason = run_until_encoder_target(
                node,
                target=target,
                timeout_sec=args.timeout_sec,
                hz=args.hz,
            )
            odom, unit = odom_value(node)
            print(f"\nRESULT stop_reason={stop_reason} odom_{unit}={odom:+.4f}")
            actual, _unit = actual_prompt(node.rotational)
            scale = log_trial(
                args,
                stage=args.stage,
                trial=1,
                stop_reason=stop_reason,
                odom=odom,
                actual=actual,
                unit=unit,
            )
            if scale is not None:
                print(f"scale_actual_over_odom={scale:.4f}")
    finally:
        stop_and_settle(node)
        node.destroy_node()
        rclpy.shutdown()
    print(f"logged: {args.log_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
