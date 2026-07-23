#!/usr/bin/env python3
"""Follow the centre of two rows of floor objects without strafing or wall fixes.

The wide camera's pose-independent ``/world_model/wide_relative_objects``
stream is the only navigation input.  Objects with positive base_link y form
the left boundary and objects with negative y form the right boundary.  A
robust line is fitted to each side every frame; their midpoint and average
direction create a smooth steering command.

Only ``vx`` and ``omega`` are published.  ``vy`` is always exactly zero.
This is a corridor-following test, not arbitrary obstacle avoidance.  It stops
when either boundary disappears, another base-command publisher appears, a
wall-localizer node is running, or an object blocks the centre ahead.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, WorldModel


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def signed_deadband(value: float, deadband: float) -> float:
    magnitude = max(0.0, abs(float(value)) - max(0.0, float(deadband)))
    return math.copysign(magnitude, value) if magnitude > 0.0 else 0.0


@dataclass(frozen=True)
class Observation:
    x: float
    y: float
    confidence: float
    label: str


@dataclass(frozen=True)
class LineFit:
    slope: float
    intercept: float
    count: int
    residual_m: float


@dataclass(frozen=True)
class CorridorEstimate:
    left: LineFit
    right: LineFit
    center_intercept_m: float
    center_slope: float
    left_lookahead_m: float
    right_lookahead_m: float
    width_m: float


@dataclass
class FilteredCorridor:
    center_intercept_m: float | None = None
    center_slope: float | None = None
    width_m: float | None = None

    def update(self, estimate: CorridorEstimate, alpha: float) -> None:
        alpha = clamp(alpha, 0.0, 1.0)
        values = (
            estimate.center_intercept_m,
            estimate.center_slope,
            estimate.width_m,
        )
        if self.center_intercept_m is None:
            self.center_intercept_m, self.center_slope, self.width_m = values
            return
        self.center_intercept_m += alpha * (values[0] - self.center_intercept_m)
        self.center_slope += alpha * (values[1] - self.center_slope)
        self.width_m += alpha * (values[2] - self.width_m)


@dataclass
class SteeringState:
    omega: float = 0.0


@dataclass(frozen=True)
class SteeringOutput:
    vx: float
    omega: float
    center_error_m: float
    lane_heading_deg: float
    target_bias_deg: float


def robust_line_fit(
    points: list[Observation],
    *,
    min_pair_dx_m: float,
    max_slope: float,
    inlier_residual_m: float,
) -> LineFit | None:
    if not points:
        return None
    if len(points) == 1:
        return LineFit(0.0, points[0].y, 1, 0.0)

    def median_slope(items: list[Observation]) -> float:
        slopes: list[float] = []
        for i, first in enumerate(items):
            for second in items[i + 1 :]:
                dx = second.x - first.x
                if abs(dx) >= min_pair_dx_m:
                    slopes.append((second.y - first.y) / dx)
        return statistics.median(slopes) if slopes else 0.0

    slope = clamp(median_slope(points), -max_slope, max_slope)
    intercept = statistics.median(p.y - slope * p.x for p in points)
    residuals = [abs(p.y - (slope * p.x + intercept)) for p in points]
    inliers = [p for p, residual in zip(points, residuals) if residual <= inlier_residual_m]
    if len(inliers) >= 2 and len(inliers) < len(points):
        slope = clamp(median_slope(inliers), -max_slope, max_slope)
        intercept = statistics.median(p.y - slope * p.x for p in inliers)
        points = inliers
        residuals = [abs(p.y - (slope * p.x + intercept)) for p in points]
    residual = statistics.median(residuals) if residuals else 0.0
    return LineFit(slope, intercept, len(points), residual)


def estimate_corridor(
    observations: list[Observation], args: argparse.Namespace
) -> tuple[CorridorEstimate | None, bool, int, int]:
    usable = [
        obj
        for obj in observations
        if obj.confidence >= args.min_confidence
        and args.min_forward_m <= obj.x <= args.max_forward_m
        and abs(obj.y) <= args.max_lateral_m
    ]
    front_blocked = any(
        args.front_block_min_m <= obj.x <= args.front_block_max_m
        and abs(obj.y) <= args.front_clear_half_width_m
        for obj in usable
    )
    left_points = [obj for obj in usable if obj.y >= args.min_side_m]
    right_points = [obj for obj in usable if obj.y <= -args.min_side_m]
    if len(left_points) < args.min_objects_per_side or len(right_points) < args.min_objects_per_side:
        return None, front_blocked, len(left_points), len(right_points)

    fit_kwargs = dict(
        min_pair_dx_m=args.min_pair_dx_m,
        max_slope=math.tan(math.radians(args.max_boundary_angle_deg)),
        inlier_residual_m=args.line_inlier_residual_m,
    )
    left = robust_line_fit(left_points, **fit_kwargs)
    right = robust_line_fit(right_points, **fit_kwargs)
    if left is None or right is None:
        return None, front_blocked, len(left_points), len(right_points)

    lookahead = args.lookahead_m
    left_y = left.slope * lookahead + left.intercept
    right_y = right.slope * lookahead + right.intercept
    width = left_y - right_y
    if not (args.min_corridor_width_m <= width <= args.max_corridor_width_m):
        return None, front_blocked, left.count, right.count
    if left_y <= 0.0 or right_y >= 0.0:
        return None, front_blocked, left.count, right.count

    return (
        CorridorEstimate(
            left=left,
            right=right,
            center_intercept_m=0.5 * (left.intercept + right.intercept),
            center_slope=0.5 * (left.slope + right.slope),
            left_lookahead_m=left_y,
            right_lookahead_m=right_y,
            width_m=width,
        ),
        front_blocked,
        left.count,
        right.count,
    )


def steering_step(
    corridor: FilteredCorridor,
    state: SteeringState,
    dt: float,
    args: argparse.Namespace,
) -> SteeringOutput:
    assert corridor.center_intercept_m is not None
    assert corridor.center_slope is not None
    assert corridor.width_m is not None

    center_error = corridor.center_intercept_m
    effective_center = signed_deadband(center_error, args.center_deadband_m)
    lane_heading = math.atan(corridor.center_slope)
    centering_angle = math.atan2(
        args.center_gain * effective_center,
        max(0.05, args.lookahead_m),
    )
    target_bias = args.lane_heading_gain * lane_heading + centering_angle
    max_bias = math.radians(args.max_steering_bias_deg)
    target_bias = clamp(target_bias, -max_bias, max_bias)
    omega_target = clamp(
        args.steering_kp * target_bias,
        -args.max_omega,
        args.max_omega,
    )
    max_delta = max(0.0, args.omega_slew_rad_s2) * max(0.0, dt)
    state.omega += clamp(omega_target - state.omega, -max_delta, max_delta)

    severity = max(
        abs(effective_center) / max(0.01, args.slow_center_error_m),
        abs(target_bias) / max(math.radians(1.0), max_bias),
    )
    speed_ratio = clamp(severity, 0.0, 1.0)
    vx = args.speed_mps + speed_ratio * (args.min_speed_mps - args.speed_mps)
    return SteeringOutput(
        vx=vx,
        omega=state.omega,
        center_error_m=center_error,
        lane_heading_deg=math.degrees(lane_heading),
        target_bias_deg=math.degrees(target_bias),
    )


class ObjectCorridorNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("shape4_object_corridor_test_v5_5_0")
        self.args = args
        self.cmd_pub = self.create_publisher(BaseCommand, "/base_command", 10)
        self.create_subscription(
            WorldModel,
            "/world_model/wide_relative_objects",
            self.on_relative_objects,
            10,
        )
        self.observations: list[Observation] = []
        self.frame_time_s = 0.0
        self.valid_time_s = 0.0
        self.latest_estimate: CorridorEstimate | None = None
        self.front_blocked = False
        self.left_count = 0
        self.right_count = 0
        self.valid_streak = 0

    def on_relative_objects(self, msg: WorldModel) -> None:
        now = time.monotonic()
        self.frame_time_s = now
        self.observations = [
            Observation(
                x=float(obj.x),
                y=float(obj.y),
                confidence=float(obj.confidence),
                label=str(obj.class_label),
            )
            for obj in msg.objects
            if math.isfinite(float(obj.x))
            and math.isfinite(float(obj.y))
            and math.isfinite(float(obj.confidence))
        ]
        estimate, blocked, left_count, right_count = estimate_corridor(
            self.observations, self.args
        )
        self.front_blocked = blocked
        self.left_count = left_count
        self.right_count = right_count
        self.latest_estimate = estimate
        if estimate is None:
            self.valid_streak = 0
        else:
            self.valid_time_s = now
            self.valid_streak += 1

    def publish_command(self, vx: float, omega: float) -> None:
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.vx = float(vx)
        msg.vy = 0.0
        msg.omega = float(omega)
        self.cmd_pub.publish(msg)

    def stop(self, duration_sec: float = 0.4) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        period = 1.0 / max(5.0, self.args.control_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(period)

    def other_base_publishers(self) -> list[str]:
        others: list[str] = []
        for info in self.get_publishers_info_by_topic("/base_command"):
            if info.node_name != self.get_name():
                namespace = info.node_namespace.rstrip("/")
                others.append(f"{namespace}/{info.node_name}" if namespace else info.node_name)
        return sorted(set(others))

    def wall_localizer_nodes(self) -> list[str]:
        found = []
        for name, namespace in self.get_node_names_and_namespaces():
            if "wall_localizer" in name:
                prefix = namespace.rstrip("/")
                found.append(f"{prefix}/{name}" if prefix else name)
        return sorted(set(found))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Follow the midpoint of two detected object rows; vx+omega only."
    )
    parser.add_argument("--run-sec", type=float, default=12.0)
    parser.add_argument("--speed-mps", type=float, default=0.085)
    parser.add_argument("--min-speed-mps", type=float, default=0.050)
    parser.add_argument("--min-confidence", type=float, default=0.25)
    parser.add_argument("--min-forward-m", type=float, default=0.12)
    parser.add_argument("--max-forward-m", type=float, default=1.35)
    parser.add_argument("--min-side-m", type=float, default=0.18)
    parser.add_argument("--max-lateral-m", type=float, default=1.10)
    parser.add_argument("--min-objects-per-side", type=int, default=2)
    parser.add_argument("--min-pair-dx-m", type=float, default=0.12)
    parser.add_argument("--max-boundary-angle-deg", type=float, default=25.0)
    parser.add_argument("--line-inlier-residual-m", type=float, default=0.12)
    parser.add_argument("--lookahead-m", type=float, default=0.55)
    parser.add_argument("--min-corridor-width-m", type=float, default=0.48)
    parser.add_argument("--max-corridor-width-m", type=float, default=1.20)
    parser.add_argument("--center-deadband-m", type=float, default=0.025)
    parser.add_argument("--center-gain", type=float, default=1.0)
    parser.add_argument("--lane-heading-gain", type=float, default=0.75)
    parser.add_argument("--steering-kp", type=float, default=1.20)
    parser.add_argument("--max-steering-bias-deg", type=float, default=10.0)
    parser.add_argument("--max-omega", type=float, default=0.085)
    parser.add_argument("--omega-slew-rad-s2", type=float, default=0.18)
    parser.add_argument("--corridor-filter-alpha", type=float, default=0.22)
    parser.add_argument("--slow-center-error-m", type=float, default=0.10)
    parser.add_argument("--front-block-min-m", type=float, default=0.10)
    parser.add_argument("--front-block-max-m", type=float, default=0.60)
    parser.add_argument("--front-clear-half-width-m", type=float, default=0.22)
    parser.add_argument("--observation-timeout-sec", type=float, default=0.70)
    parser.add_argument("--stable-frames", type=int, default=4)
    parser.add_argument("--startup-timeout-sec", type=float, default=15.0)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--log-rate-hz", type=float, default=4.0)
    parser.add_argument("--start-delay-sec", type=float, default=2.0)
    parser.add_argument("--csv", default="/tmp/object_corridor_feedback.csv")
    parser.add_argument("--no-confirm", action="store_true")
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    checks = [
        (1.0 <= args.run_sec <= 120.0, "run-sec must be in [1, 120]"),
        (0.03 <= args.min_speed_mps <= args.speed_mps <= 0.18,
         "require 0.03 <= min speed <= speed <= 0.18"),
        (0.0 <= args.min_confidence <= 1.0, "confidence must be in [0, 1]"),
        (0.05 <= args.min_forward_m < args.max_forward_m,
         "invalid forward observation range"),
        (0.10 <= args.min_side_m < args.max_lateral_m,
         "invalid lateral observation range"),
        (1 <= args.min_objects_per_side <= 6, "min objects per side must be in [1, 6]"),
        (0.25 <= args.lookahead_m <= 1.20, "lookahead must be in [0.25, 1.20]"),
        (0.35 <= args.min_corridor_width_m < args.max_corridor_width_m,
         "invalid corridor width range"),
        (0.0 <= args.center_deadband_m < args.slow_center_error_m,
         "deadband must be smaller than slow error"),
        (0.02 <= args.max_omega <= 0.20, "max omega must be in [0.02, 0.20]"),
        (0.0 < args.corridor_filter_alpha <= 1.0,
         "corridor filter alpha must be in (0, 1]"),
        (5.0 <= args.control_rate_hz <= 50.0, "control rate must be in [5, 50]"),
    ]
    for valid, message in checks:
        if not valid:
            parser.error(message)


def open_csv(path: str):
    if not path:
        return None, None
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    handle = open(path, "w", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    writer.writerow([
        "time_s", "left_count", "right_count", "left_lookahead_m",
        "right_lookahead_m", "corridor_width_m", "center_error_m",
        "lane_heading_deg", "target_bias_deg", "vx", "vy", "omega",
    ])
    return handle, writer


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    rclpy.init()
    node = ObjectCorridorNode(args)
    csv_handle = None
    try:
        node.get_logger().info(
            "object corridor test waiting for both object rows; wall/localization pose unused"
        )
        discovery_deadline = time.monotonic() + 0.8
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        walls = node.wall_localizer_nodes()
        if walls:
            node.get_logger().error(
                f"wall localizer is active: {walls}; relaunch with with_wall_localizer:=false"
            )
            return 2
        others = node.other_base_publishers()
        if others:
            node.get_logger().error(
                f"another /base_command publisher is active: {others}; refusing to move"
            )
            return 3
        if node.count_subscribers("/base_command") == 0:
            node.get_logger().error("no /base_command subscriber; base controller is not running")
            return 4

        startup_deadline = time.monotonic() + args.startup_timeout_sec
        next_wait_log = 0.0
        while rclpy.ok() and time.monotonic() < startup_deadline:
            node.publish_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if node.front_blocked:
                node.get_logger().error("object is inside the forward safety corridor")
                return 5
            if node.latest_estimate is not None and node.valid_streak >= args.stable_frames:
                break
            if now >= next_wait_log:
                age = now - node.frame_time_s if node.frame_time_s else math.inf
                node.get_logger().info(
                    f"waiting: left={node.left_count} right={node.right_count} "
                    f"valid={node.valid_streak}/{args.stable_frames} frame_age={age:.2f}s"
                )
                next_wait_log = now + 1.0
        else:
            node.get_logger().error(
                "both object rows were not detected stably before startup timeout"
            )
            return 6

        estimate = node.latest_estimate
        assert estimate is not None
        print(
            "\n물체 통로 중심 추종 테스트\n"
            f"  감지 개수: 왼쪽 {estimate.left.count}, 오른쪽 {estimate.right.count}\n"
            f"  통로 폭:   {estimate.width_m * 100:.1f}cm\n"
            f"  중심 오차: {estimate.center_intercept_m * 100:+.1f}cm "
            "(+는 통로 중심이 로봇 왼쪽)\n"
            f"  속도:      {args.speed_mps:.3f}m/s, 실행 {args.run_sec:.1f}s\n"
            "  입력: 광각 상대 물체만 사용, 벽/맵 pose 미사용\n"
            "  출력: vx + omega, vy=0 고정\n"
        )
        if not args.no_confirm:
            input("좌우 물체 열과 정면 안전거리를 확인했으면 Enter (Ctrl+C 취소): ")

        delay_deadline = time.monotonic() + args.start_delay_sec
        while rclpy.ok() and time.monotonic() < delay_deadline:
            node.publish_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.03)

        csv_handle, csv_writer = open_csv(args.csv)
        filtered = FilteredCorridor()
        steering = SteeringState()
        start_s = time.monotonic()
        last_tick_s = start_s
        next_log_s = start_s
        period = 1.0 / args.control_rate_hz

        while rclpy.ok():
            tick_s = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.005)
            elapsed = tick_s - start_s
            if elapsed >= args.run_sec:
                node.stop()
                print("\n설정 시간이 끝나 정상 정지했습니다.")
                return 0
            walls = node.wall_localizer_nodes()
            if walls:
                node.get_logger().error(f"wall localizer appeared during test: {walls}")
                return 2
            others = node.other_base_publishers()
            if others:
                node.get_logger().error(
                    f"another /base_command publisher appeared: {others}; stopping"
                )
                return 3
            if node.front_blocked:
                node.get_logger().error("forward corridor blocked by an object; stopping")
                return 5
            if tick_s - node.frame_time_s > args.observation_timeout_sec:
                node.get_logger().error("wide relative-object stream is stale; stopping")
                return 7
            if (
                node.latest_estimate is None
                or tick_s - node.valid_time_s > args.observation_timeout_sec
            ):
                node.get_logger().error(
                    f"corridor lost: left={node.left_count} right={node.right_count}; stopping"
                )
                return 8

            estimate = node.latest_estimate
            filtered.update(estimate, args.corridor_filter_alpha)
            dt = clamp(tick_s - last_tick_s, 0.0, 0.20)
            last_tick_s = tick_s
            output = steering_step(filtered, steering, dt, args)
            node.publish_command(output.vx, output.omega)

            if csv_writer is not None:
                csv_writer.writerow([
                    f"{elapsed:.4f}", estimate.left.count, estimate.right.count,
                    f"{estimate.left_lookahead_m:.5f}",
                    f"{estimate.right_lookahead_m:.5f}",
                    f"{filtered.width_m:.5f}", f"{output.center_error_m:.5f}",
                    f"{output.lane_heading_deg:.3f}", f"{output.target_bias_deg:.3f}",
                    f"{output.vx:.5f}", "0.00000", f"{output.omega:.5f}",
                ])
            if tick_s >= next_log_s:
                message = (
                    f"L/R={estimate.left.count}/{estimate.right.count} "
                    f"width={filtered.width_m * 100:.1f}cm "
                    f"center={output.center_error_m * 100:+.1f}cm "
                    f"lane={output.lane_heading_deg:+.1f}deg "
                    f"bias={output.target_bias_deg:+.1f}deg "
                    f"cmd=({output.vx:.3f},0.000,{output.omega:+.3f})"
                )
                print(message)
                node.get_logger().info(message)
                next_log_s = tick_s + 1.0 / max(0.5, args.log_rate_hz)

            sleep_for = period - (time.monotonic() - tick_s)
            if sleep_for > 0.0:
                time.sleep(sleep_for)
    except (KeyboardInterrupt, EOFError):
        print("\n사용자 중단")
        return 130
    finally:
        try:
            node.stop()
        except Exception:
            pass
        if csv_handle is not None:
            csv_handle.close()
            print(f"CSV 저장: {os.path.abspath(args.csv)}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
