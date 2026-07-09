#!/usr/bin/env python3
"""ROS open-loop encoder calibration while the live map is running.

This publishes the same kind of command used by the main ROS drive stack, listens to
/base/wheel_odom, integrates encoder travel, then compares it with a manually measured value.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import re
import statistics
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand
from std_msgs.msg import Float32MultiArray


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
    samples = [float(signs[i]) * float(vals[i]) for i in wheels if abs(float(signs[i])) > 1e-9]
    if not samples:
        return 0.0
    if len(samples) == 1:
        return samples[0]
    if len(samples) == 2:
        return 0.5 * (samples[0] + samples[1])
    return float(statistics.median(samples))


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


def direction_to_wheels(direction: str, speed: float) -> tuple[list[float], list[float]]:
    mag = abs(float(speed))
    if direction == "forward":
        return [mag, mag, mag, mag], [1.0, 1.0, 1.0, 1.0]
    if direction == "backward":
        return [-mag, -mag, -mag, -mag], [-1.0, -1.0, -1.0, -1.0]
    if direction == "right":
        return [mag, -mag, -mag, mag], [1.0, -1.0, -1.0, 1.0]
    if direction == "left":
        return [-mag, mag, mag, -mag], [-1.0, 1.0, 1.0, -1.0]
    if direction == "ccw":
        return [-mag, mag, -mag, mag], [-1.0, 1.0, -1.0, 1.0]
    if direction == "cw":
        return [mag, -mag, mag, -mag], [1.0, -1.0, 1.0, -1.0]
    raise ValueError(f"unsupported direction: {direction}")


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
        old_fields = list(reader.fieldnames or [])
    fields = list(old_fields)
    for key in row.keys():
        if key not in fields:
            fields.append(key)
    rows.append({key: row.get(key, "") for key in fields})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_yaml_odom_scale(path: Path) -> float:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return 1.0
    m = re.search(r"(?m)^\s*odom_scale:\s*([-+0-9.eE]+)", text)
    if m is None:
        return 1.0
    return float(m.group(1))


def update_yaml_odom_scale(path: Path, value: float) -> bool:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"WARNING: cannot read params file {path}: {exc}")
        return False
    pattern = re.compile(r"(?m)^(\s*odom_scale:\s*)([-+0-9.eE]+)(.*)$")
    replacement = rf"\g<1>{value:.6f}\g<3>"
    new_text, n = pattern.subn(replacement, text, count=1)
    if n == 0:
        print(f"WARNING: odom_scale not found in {path}")
        return False
    try:
        path.write_text(new_text, encoding="utf-8")
    except OSError as exc:
        print(f"WARNING: cannot write params file {path}: {exc}")
        return False
    return True


def set_live_odom_scale(node: Node, node_name: str, value: float) -> bool:
    service_name = f"{node_name.rstrip('/')}/set_parameters"
    client = node.create_client(SetParameters, service_name)
    if not client.wait_for_service(timeout_sec=1.0):
        print(f"WARNING: parameter service {service_name} not available; restart bridge to apply YAML value")
        return False
    req = SetParameters.Request()
    req.parameters = [
        Parameter(
            name="odom_scale",
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )
    ]
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done():
        print(f"WARNING: timed out setting live {node_name}.odom_scale")
        return False
    response = future.result()
    results = response.results if response is not None else []
    ok = bool(results and results[0].successful)
    if not ok:
        reason = results[0].reason if results else "unknown"
        print(f"WARNING: live odom_scale update rejected: {reason}")
    return ok


def ask_choice(prompt: str, choices: list[str], default: str) -> str:
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


class RosOpenLoopCalib(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ros_open_loop_calib")
        self.args = args
        self.odom_wheels = parse_index_list(args.odom_wheels)
        self.rotational = args.direction in {"cw", "ccw"}
        _wheels, self.odom_signs = direction_to_wheels(args.direction, args.speed)
        self.progress = 0.0
        self.last_odom_t: float | None = None
        self.saw_odom = False
        self.create_subscription(Float32MultiArray, "/base/wheel_odom", self.on_odom, 20)
        if args.command_mode == "base-command":
            self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)
            self.pub_wheels = None
        else:
            self.pub_cmd = None
            self.pub_wheels = self.create_publisher(Float32MultiArray, "/base/wheel_speeds", 10)

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
        self.saw_odom = True
        if self.rotational:
            print(f"odom spin={signed_speed:+.3f} angle={math.degrees(self.progress):+.1f} deg")
        else:
            print(f"odom speed={signed_speed:+.3f} distance={self.progress:+.3f} m")

    def publish_move(self) -> None:
        if self.args.command_mode == "base-command":
            vx, vy, omega = direction_to_base_command(self.args.direction, self.args.speed)
            msg = BaseCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.vx = float(vx)
            msg.vy = float(vy)
            msg.omega = float(omega)
            self.pub_cmd.publish(msg)
        else:
            wheels, _signs = direction_to_wheels(self.args.direction, self.args.speed)
            msg = Float32MultiArray()
            msg.data = [float(v) for v in wheels]
            self.pub_wheels.publish(msg)

    def publish_stop(self) -> None:
        if self.args.command_mode == "base-command":
            msg = BaseCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(msg)
        else:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub_wheels.publish(msg)

    def command_subscriber_count(self) -> int:
        if self.args.command_mode == "base-command":
            return self.count_subscribers("/base_command")
        return self.count_subscribers("/base/wheel_speeds")

    def odom_publisher_count(self) -> int:
        return self.count_publishers("/base/wheel_odom")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--direction", choices=["forward", "backward", "left", "right", "cw", "ccw"], default="forward")
    ap.add_argument("--speed", type=float, default=0.26, help="base command speed or wheel speed magnitude")
    ap.add_argument("--duration-ms", type=int, default=450)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--k", type=float, default=0.20, help="mecanum k = lx + ly for rotation integration")
    ap.add_argument("--odom-wheels", default="1,2,3", help="trusted odom wheels; default ignores FL")
    ap.add_argument(
        "--command-mode",
        choices=["base-command", "wheel-speeds"],
        default="base-command",
        help="base-command matches the main drive stack through base_controller_node",
    )
    ap.add_argument("--actual-m", type=float, default=None)
    ap.add_argument("--actual-deg", type=float, default=None)
    ap.add_argument("--log-csv", default="scripts/drive_tests/ros_open_loop_calib_log.csv")
    ap.add_argument(
        "--params-file",
        default="ros2_ws/src/robot_bringup/config/test_field.yaml",
        help="YAML file whose mcu_bridge_base_node.odom_scale is updated after each measurement",
    )
    ap.add_argument("--mcu-node", default="/mcu_bridge_base_node")
    ap.add_argument(
        "--no-update-odom-scale",
        action="store_true",
        help="only print/log the scale; do not update YAML or the running bridge",
    )
    ap.add_argument(
        "--no-prompt",
        action="store_true",
        help="use CLI args without asking direction/speed/duration interactively",
    )
    args = ap.parse_args()

    if not args.no_prompt:
        print("Enter test values. Press Enter to keep the default.")
        args.direction = ask_choice(
            "Direction",
            ["forward", "backward", "left", "right", "cw", "ccw"],
            args.direction,
        )
        args.duration_ms = ask_int("Drive duration ms", args.duration_ms)
        args.speed = ask_float("Command speed", args.speed)

    rclpy.init()
    node = RosOpenLoopCalib(args)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.command_subscriber_count() > 0 and node.odom_publisher_count() > 0:
            break
    cmd_subs = node.command_subscriber_count()
    odom_pubs = node.odom_publisher_count()
    if cmd_subs <= 0:
        if args.command_mode == "base-command":
            print("WARNING: /base_command has no subscribers. Is base_controller_node running?")
        else:
            print("WARNING: /base/wheel_speeds has no subscribers. Is mcu_bridge_base_node running?")
    if odom_pubs <= 0:
        print("WARNING: /base/wheel_odom has no publishers. Is mcu_bridge_base_node running?")
    period = 1.0 / max(args.hz, 1e-6)
    end_at = time.monotonic() + max(0.001, args.duration_ms / 1000.0)
    print(
        f"ros-open-loop-calib: mode={args.command_mode} direction={args.direction} "
        f"speed={args.speed} duration={args.duration_ms}ms odom_wheels={args.odom_wheels}"
    )
    try:
        while time.monotonic() < end_at:
            node.publish_move()
            rclpy.spin_once(node, timeout_sec=period)
        for _ in range(10):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.02)

        if args.direction in {"cw", "ccw"}:
            odom_value = math.degrees(node.progress)
            actual_value = args.actual_deg
            unit = "deg"
        else:
            odom_value = node.progress
            actual_value = args.actual_m
            unit = "m"
        print(f"RESULT odom_{unit}={odom_value:+.4f}")
        scale = None
        current_odom_scale = read_yaml_odom_scale(Path(args.params_file))
        new_odom_scale = None
        if actual_value is None:
            text = input(f"Measured actual travel ({unit}); blank to skip: ").strip()
            if text:
                actual_value = float(text)
        if actual_value is not None:
            if abs(odom_value) <= 1e-9:
                print("Cannot compute scale: odom value is ~0")
            else:
                scale = float(actual_value) / float(odom_value)
                new_odom_scale = current_odom_scale * scale
                print(f"RESULT actual_{unit}={actual_value:+.4f}")
                print(f"RESULT residual_scale_actual_over_odom={scale:.6f}")
                print(f"RESULT previous_odom_scale={current_odom_scale:.6f}")
                print(f"RESULT updated_odom_scale={new_odom_scale:.6f}")
                if not args.no_update_odom_scale:
                    params_path = Path(args.params_file)
                    if update_yaml_odom_scale(params_path, new_odom_scale):
                        print(f"updated YAML: {params_path} odom_scale={new_odom_scale:.6f}")
                    if set_live_odom_scale(node, args.mcu_node, new_odom_scale):
                        print(f"updated live parameter: {args.mcu_node}.odom_scale={new_odom_scale:.6f}")
        if args.log_csv:
            append_csv(
                Path(args.log_csv),
                {
                    "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "command_mode": args.command_mode,
                    "direction": args.direction,
                    "speed": args.speed,
                    "duration_ms": args.duration_ms,
                    "odom_wheels": args.odom_wheels,
                    "odom_value": odom_value,
                    "actual_value": "" if actual_value is None else actual_value,
                    "residual_scale_actual_over_odom": "" if scale is None else scale,
                    "previous_odom_scale": current_odom_scale,
                    "updated_odom_scale": "" if new_odom_scale is None else new_odom_scale,
                    "unit": unit,
                },
            )
            print(f"logged: {args.log_csv}")
    finally:
        for _ in range(5):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
