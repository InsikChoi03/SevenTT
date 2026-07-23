#!/usr/bin/env python3
"""Independent straight-lane cross-track feedback driving test.

This test publishes only ``vx`` and ``omega`` to ``/base_command``.  ``vy`` is
always zero, so the robot returns to the planned centreline by steering a
smooth curve instead of strafing.

The floor objects used for this test are passive corridor markers.  This
script does not detect or avoid them; never put an object on the centreline.

Typical use (run the base/localization stack with the mission FSM disabled):

  python3 scripts/shape4_lane_feedback_test_v5.4.0.py --distance-m 1.5

Force an initial 10 cm centreline error without moving the robot physically:

  python3 scripts/shape4_lane_feedback_test_v5.4.0.py \
      --distance-m 1.5 --centerline-offset-m 0.10

Positive ``--centerline-offset-m`` puts the planned centreline to the robot's
left.  Negative values put it to the right.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, WorldModel


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    return 2.0 * math.atan2(float(q.z), float(q.w))


def signed_deadband(value: float, deadband: float) -> float:
    magnitude = max(0.0, abs(float(value)) - max(0.0, float(deadband)))
    return math.copysign(magnitude, value) if magnitude > 0.0 else 0.0


@dataclass(frozen=True)
class LinePath:
    start_x: float
    start_y: float
    goal_x: float
    goal_y: float
    heading: float
    length: float
    ux: float
    uy: float
    nx: float
    ny: float

    @classmethod
    def from_pose(
        cls,
        pose: tuple[float, float, float],
        distance_m: float,
        heading: float,
        centerline_offset_m: float,
    ) -> "LinePath":
        ux, uy = math.cos(heading), math.sin(heading)
        nx, ny = -uy, ux  # field-left normal of the planned path
        sx = float(pose[0]) + float(centerline_offset_m) * nx
        sy = float(pose[1]) + float(centerline_offset_m) * ny
        length = float(distance_m)
        return cls(
            start_x=sx,
            start_y=sy,
            goal_x=sx + length * ux,
            goal_y=sy + length * uy,
            heading=wrap_pi(heading),
            length=length,
            ux=ux,
            uy=uy,
            nx=nx,
            ny=ny,
        )

    def errors(self, x: float, y: float) -> tuple[float, float, float]:
        """Return along-track distance, signed cross-track error, remaining."""
        dx, dy = float(x) - self.start_x, float(y) - self.start_y
        along = dx * self.ux + dy * self.uy
        cross_track = dx * self.nx + dy * self.ny
        return along, cross_track, self.length - along


@dataclass
class FeedbackState:
    filtered_cte: float | None = None
    omega: float = 0.0


@dataclass(frozen=True)
class FeedbackOutput:
    vx: float
    omega: float
    filtered_cte: float
    heading_bias: float
    target_heading: float
    heading_error: float


def feedback_step(
    *,
    raw_cte: float,
    robot_heading: float,
    path_heading: float,
    remaining_m: float,
    dt: float,
    state: FeedbackState,
    args: argparse.Namespace,
) -> FeedbackOutput:
    alpha = clamp(args.cte_filter_alpha, 0.0, 1.0)
    if state.filtered_cte is None:
        state.filtered_cte = float(raw_cte)
    else:
        state.filtered_cte += alpha * (float(raw_cte) - state.filtered_cte)

    effective_cte = signed_deadband(state.filtered_cte, args.cte_deadband_m)
    heading_bias = -math.atan2(
        args.cross_track_gain * effective_cte,
        max(0.05, args.lookahead_m),
    )
    max_bias = math.radians(args.max_heading_bias_deg)
    heading_bias = clamp(heading_bias, -max_bias, max_bias)
    target_heading = wrap_pi(path_heading + heading_bias)
    heading_error = wrap_pi(target_heading - robot_heading)

    omega_target = clamp(
        args.heading_kp * heading_error,
        -args.max_omega,
        args.max_omega,
    )
    max_delta = max(0.0, args.omega_slew_rad_s2) * max(0.0, dt)
    state.omega += clamp(omega_target - state.omega, -max_delta, max_delta)

    vx = args.speed_mps
    abs_cte = abs(state.filtered_cte)
    if abs_cte > args.cte_slow_start_m:
        span = max(0.01, args.abort_cte_m - args.cte_slow_start_m)
        ratio = clamp((abs_cte - args.cte_slow_start_m) / span, 0.0, 1.0)
        vx = args.speed_mps + ratio * (args.min_speed_mps - args.speed_mps)
    if remaining_m < args.goal_slow_distance_m:
        goal_ratio = clamp(
            remaining_m / max(0.01, args.goal_slow_distance_m), 0.0, 1.0
        )
        goal_speed = args.min_speed_mps + goal_ratio * (
            args.speed_mps - args.min_speed_mps
        )
        vx = min(vx, goal_speed)

    return FeedbackOutput(
        vx=max(0.0, vx),
        omega=state.omega,
        filtered_cte=state.filtered_cte,
        heading_bias=heading_bias,
        target_heading=target_heading,
        heading_error=heading_error,
    )


class LaneFeedbackTestNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("shape4_lane_feedback_test_v5_4_0")
        self.args = args
        self.cmd_pub = self.create_publisher(BaseCommand, "/base_command", 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.pose: tuple[float, float, float] | None = None
        self.pose_stamp_s = 0.0

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            yaw_from_pose(msg),
        )
        self.pose_stamp_s = time.monotonic()

    def on_world(self, msg: WorldModel) -> None:
        # /localization/pose is preferred.  WorldModel is a fallback so the
        # independent test still works with the same sources as the main FSM.
        if time.monotonic() - self.pose_stamp_s <= self.args.pose_timeout_sec:
            return
        self.pose = (
            float(msg.robot_x),
            float(msg.robot_y),
            float(msg.robot_theta),
        )
        self.pose_stamp_s = time.monotonic()

    def fresh_pose(self) -> tuple[float, float, float] | None:
        if self.pose is None:
            return None
        if time.monotonic() - self.pose_stamp_s > self.args.pose_timeout_sec:
            return None
        return self.pose

    def publish_command(self, vx: float, omega: float) -> None:
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.vx = float(vx)
        msg.vy = 0.0  # Absolute rule for this test: never strafe.
        msg.omega = float(omega)
        self.cmd_pub.publish(msg)

    def stop(self, duration_sec: float = 0.35) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        period = 1.0 / max(5.0, self.args.control_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(period)

    def wait_for_pose(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.fresh_pose() is not None:
                return True
        return False

    def other_base_publishers(self) -> list[str]:
        others: list[str] = []
        for info in self.get_publishers_info_by_topic("/base_command"):
            if info.node_name != self.get_name():
                namespace = info.node_namespace.rstrip("/")
                others.append(f"{namespace}/{info.node_name}" if namespace else info.node_name)
        return sorted(set(others))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="No-strafe straight-line cross-track feedback driving test."
    )
    parser.add_argument("--distance-m", type=float, default=1.50,
                        help="planned centreline length")
    parser.add_argument("--path-heading-deg", type=float, default=None,
                        help="field heading; default is heading at test start")
    parser.add_argument("--centerline-offset-m", type=float, default=0.0,
                        help="shift planned line left (+) or right (-) of start pose")
    parser.add_argument("--speed-mps", type=float, default=0.10)
    parser.add_argument("--min-speed-mps", type=float, default=0.055)
    parser.add_argument("--cte-deadband-m", type=float, default=0.02,
                        help="quiet half-width around the centreline")
    parser.add_argument("--cte-slow-start-m", type=float, default=0.10,
                        help="start reducing forward speed at this lateral error")
    parser.add_argument("--abort-cte-m", type=float, default=0.20,
                        help="stop immediately at this lateral error")
    parser.add_argument("--arrival-cte-m", type=float, default=0.08,
                        help="maximum lateral error accepted at the goal line")
    parser.add_argument("--lookahead-m", type=float, default=0.60,
                        help="larger is smoother/weaker; smaller is faster/sharper")
    parser.add_argument("--cross-track-gain", type=float, default=1.0)
    parser.add_argument("--heading-kp", type=float, default=1.20)
    parser.add_argument("--max-heading-bias-deg", type=float, default=10.0)
    parser.add_argument("--max-omega", type=float, default=0.08)
    parser.add_argument("--omega-slew-rad-s2", type=float, default=0.20)
    parser.add_argument("--cte-filter-alpha", type=float, default=0.20)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.07)
    parser.add_argument("--goal-slow-distance-m", type=float, default=0.30)
    parser.add_argument("--max-overshoot-m", type=float, default=0.12)
    parser.add_argument("--max-start-heading-error-deg", type=float, default=8.0,
                        help="refuse to start if robot is not manually aligned")
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--log-rate-hz", type=float, default=4.0)
    parser.add_argument("--pose-timeout-sec", type=float, default=0.60)
    parser.add_argument("--timeout-sec", type=float, default=35.0)
    parser.add_argument("--start-delay-sec", type=float, default=2.0)
    parser.add_argument("--csv", default="",
                        help="optional CSV path for pose/error samples")
    parser.add_argument("--no-confirm", action="store_true",
                        help="skip the final Enter prompt")
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    checks = [
        (args.distance_m >= 0.30, "--distance-m must be >= 0.30"),
        (0.03 <= args.min_speed_mps <= args.speed_mps <= 0.20,
         "require 0.03 <= min speed <= speed <= 0.20"),
        (0.0 <= args.cte_deadband_m < args.cte_slow_start_m < args.abort_cte_m,
         "require deadband < slow-start < abort CTE"),
        (0.0 < args.arrival_cte_m < args.abort_cte_m,
         "arrival CTE must be between zero and abort CTE"),
        (0.20 <= args.lookahead_m <= 1.50, "lookahead must be in [0.20, 1.50]"),
        (0.1 <= args.cross_track_gain <= 4.0, "cross-track gain must be in [0.1, 4.0]"),
        (0.1 <= args.heading_kp <= 4.0, "heading Kp must be in [0.1, 4.0]"),
        (1.0 <= args.max_heading_bias_deg <= 25.0,
         "max heading bias must be in [1, 25] degrees"),
        (0.02 <= args.max_omega <= 0.25, "max omega must be in [0.02, 0.25]"),
        (0.0 < args.cte_filter_alpha <= 1.0, "filter alpha must be in (0, 1]"),
        (5.0 <= args.control_rate_hz <= 50.0, "control rate must be in [5, 50]"),
    ]
    for valid, message in checks:
        if not valid:
            parser.error(message)


def open_csv(path: str):
    if not path:
        return None, None
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    handle = open(path, "w", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    writer.writerow([
        "time_s", "x_m", "y_m", "heading_deg", "along_m", "raw_cte_m",
        "filtered_cte_m", "remaining_m", "heading_bias_deg",
        "heading_error_deg", "vx", "vy", "omega",
    ])
    return handle, writer


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    rclpy.init()
    node = LaneFeedbackTestNode(args)
    csv_handle = None
    try:
        node.get_logger().info(
            "lane feedback test ready; use test_field.launch.py with with_fsm:=false"
        )
        if not node.wait_for_pose(8.0):
            node.get_logger().error("fresh /localization/pose or /world_model pose not received")
            return 2

        # Give DDS discovery enough time to reveal competing command publishers.
        discovery_deadline = time.monotonic() + 0.6
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        others = node.other_base_publishers()
        if others:
            node.get_logger().error(
                f"another /base_command publisher is active: {others}; refusing to move"
            )
            return 3
        if node.count_subscribers("/base_command") == 0:
            node.get_logger().error("no /base_command subscriber; base controller is not running")
            return 4

        pose = node.fresh_pose()
        if pose is None:
            node.get_logger().error("pose became stale before path initialization")
            return 2
        path_heading = (
            pose[2] if args.path_heading_deg is None
            else math.radians(args.path_heading_deg)
        )
        start_heading_error = wrap_pi(path_heading - pose[2])
        if abs(start_heading_error) > math.radians(args.max_start_heading_error_deg):
            node.get_logger().error(
                f"start heading error {math.degrees(start_heading_error):+.1f}deg exceeds "
                f"{args.max_start_heading_error_deg:.1f}deg; align the robot manually"
            )
            return 5
        path = LinePath.from_pose(
            pose,
            args.distance_m,
            path_heading,
            args.centerline_offset_m,
        )

        print(
            "\n독립 레인 피드백 테스트\n"
            f"  시작 중심선: ({path.start_x:+.3f}, {path.start_y:+.3f})\n"
            f"  목표:        ({path.goal_x:+.3f}, {path.goal_y:+.3f})\n"
            f"  거리/방향:   {path.length:.2f}m / {math.degrees(path.heading):+.1f}deg\n"
            f"  속도:        {args.speed_mps:.3f}m/s (최저 {args.min_speed_mps:.3f})\n"
            f"  deadband:    +/-{args.cte_deadband_m * 100:.1f}cm\n"
            f"  lookahead:   {args.lookahead_m:.2f}m\n"
            f"  최대 보정:   {args.max_heading_bias_deg:.1f}deg, omega {args.max_omega:.3f}rad/s\n"
            f"  강제 정지:   중심선 오차 {args.abort_cte_m * 100:.1f}cm\n"
            "  vy는 실행 내내 0. 물체 자동회피는 하지 않음.\n"
        )
        if not args.no_confirm:
            input("통로와 비상정지 공간을 확인했으면 Enter (Ctrl+C 취소): ")

        print(f"{args.start_delay_sec:.1f}초 후 출발")
        delay_deadline = time.monotonic() + max(0.0, args.start_delay_sec)
        while rclpy.ok() and time.monotonic() < delay_deadline:
            node.publish_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.03)

        csv_handle, csv_writer = open_csv(args.csv)
        state = FeedbackState()
        test_start = time.monotonic()
        last_tick = test_start
        next_log = test_start
        period = 1.0 / args.control_rate_hz
        max_abs_cte = 0.0
        cte_sq_sum = 0.0
        sample_count = 0
        last_nonzero_sign = 0
        sign_crossings = 0

        while rclpy.ok():
            tick_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.005)
            if tick_start - test_start > args.timeout_sec:
                node.get_logger().error("test timeout; stopping")
                return 6
            others = node.other_base_publishers()
            if others:
                node.get_logger().error(
                    f"another /base_command publisher appeared: {others}; stopping"
                )
                return 3
            pose = node.fresh_pose()
            if pose is None:
                node.get_logger().error("pose stream stale; stopping")
                return 2

            rx, ry, rtheta = pose
            along, raw_cte, remaining = path.errors(rx, ry)
            max_abs_cte = max(max_abs_cte, abs(raw_cte))
            cte_sq_sum += raw_cte * raw_cte
            sample_count += 1
            if abs(raw_cte) > args.cte_deadband_m:
                sign = 1 if raw_cte > 0.0 else -1
                if last_nonzero_sign and sign != last_nonzero_sign:
                    sign_crossings += 1
                last_nonzero_sign = sign

            if abs(raw_cte) >= args.abort_cte_m:
                node.get_logger().error(
                    f"cross-track error {raw_cte:+.3f}m reached abort limit; stopping"
                )
                return 7
            if along >= path.length - args.goal_tolerance_m:
                if abs(raw_cte) <= args.arrival_cte_m:
                    rms = math.sqrt(cte_sq_sum / max(1, sample_count))
                    node.stop()
                    print(
                        "\n테스트 성공\n"
                        f"  최종 중심선 오차: {raw_cte * 100:+.1f}cm\n"
                        f"  최대 중심선 오차: {max_abs_cte * 100:.1f}cm\n"
                        f"  RMS 중심선 오차:  {rms * 100:.1f}cm\n"
                        f"  deadband 밖 좌우 교차: {sign_crossings}회\n"
                    )
                    return 0
                if along >= path.length + args.max_overshoot_m:
                    node.get_logger().error(
                        f"passed goal with CTE {raw_cte:+.3f}m; stopping instead of reversing"
                    )
                    return 8

            dt = clamp(tick_start - last_tick, 0.0, 0.20)
            last_tick = tick_start
            output = feedback_step(
                raw_cte=raw_cte,
                robot_heading=rtheta,
                path_heading=path.heading,
                remaining_m=remaining,
                dt=dt,
                state=state,
                args=args,
            )
            node.publish_command(output.vx, output.omega)

            elapsed = tick_start - test_start
            if csv_writer is not None:
                csv_writer.writerow([
                    f"{elapsed:.4f}", f"{rx:.5f}", f"{ry:.5f}",
                    f"{math.degrees(rtheta):.3f}", f"{along:.5f}",
                    f"{raw_cte:.5f}", f"{output.filtered_cte:.5f}",
                    f"{remaining:.5f}", f"{math.degrees(output.heading_bias):.3f}",
                    f"{math.degrees(output.heading_error):.3f}",
                    f"{output.vx:.5f}", "0.00000", f"{output.omega:.5f}",
                ])
            if tick_start >= next_log:
                print(
                    f"t={elapsed:5.1f}s along={along:5.2f}m remain={remaining:5.2f}m "
                    f"CTE={raw_cte * 100:+6.1f}cm filt={output.filtered_cte * 100:+6.1f}cm "
                    f"bias={math.degrees(output.heading_bias):+5.1f}deg "
                    f"herr={math.degrees(output.heading_error):+5.1f}deg "
                    f"cmd=({output.vx:.3f}, 0.000, {output.omega:+.3f})"
                )
                next_log = tick_start + 1.0 / max(0.5, args.log_rate_hz)

            sleep_for = period - (time.monotonic() - tick_start)
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
            if args.csv:
                print(f"CSV 저장: {os.path.abspath(args.csv)}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
