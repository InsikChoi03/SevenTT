#!/usr/bin/env python3
"""Independent main-pipeline-style A1 -> A2 lane driving test.

Pipeline used by this test:

  /localization/pose + /world_model -> LanePlanner(grid_only)
      -> cardinal lane segment -> +/-5 cm cross-track band -> /base_command

The mission FSM and scripted opening must be disabled.  The robot is physically
placed at A1 facing A2.  Before object mapping is enabled, this node resets the
trusted localization pose to A1, which replaces the opening endpoint reset.
Wall localization is forbidden.  Steering uses vx + omega only; vy is always 0.
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
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import BaseCommand, MissionState, WorldModel
from robot_planning.lane_planner import LanePlanner, cardinal_segment_heading
from robot_planning.nodes.mission_fsm_node import PulsedHeadingController
from std_msgs.msg import Bool, String

try:
    import yaml
except Exception:  # noqa: BLE001
    yaml = None


COMPETITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

DEFAULT_TUNING_FILE = "ros2_ws/src/robot_bringup/config/motion_tuning.yaml"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def signed_deadband(value: float, half_width: float) -> float:
    magnitude = max(0.0, abs(float(value)) - max(0.0, float(half_width)))
    return math.copysign(magnitude, value) if magnitude > 0.0 else 0.0


def yaml_params(path: str, node_name: str) -> dict:
    if yaml is None or not path or not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    node = data.get(node_name, {}) if isinstance(data, dict) else {}
    params = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
    return params if isinstance(params, dict) else {}


def parse_zone_candidates(params: dict) -> dict[int, list[tuple[float, float]]]:
    result: dict[int, list[tuple[float, float]]] = {}
    values = [float(value) for value in params.get("zone_anchor_candidates", [])]
    for index in range(0, len(values) - 2, 3):
        zone = int(round(values[index]))
        result.setdefault(zone, []).append((values[index + 1], values[index + 2]))
    fallback = [float(value) for value in params.get("zone_anchor_xy", [])]
    for zone in range(1, 5):
        if zone not in result and len(fallback) >= zone * 2:
            result[zone] = [(fallback[(zone - 1) * 2], fallback[(zone - 1) * 2 + 1])]
    return result


def resolve_anchors(
    params: dict, args: argparse.Namespace
) -> tuple[tuple[float, float], tuple[float, float]]:
    candidates = parse_zone_candidates(params)
    a1 = (
        (args.anchor1_x, args.anchor1_y)
        if args.anchor1_x is not None and args.anchor1_y is not None
        else (candidates.get(1) or [(-1.25, 0.75)])[0]
    )
    if args.anchor2_x is not None and args.anchor2_y is not None:
        a2 = (args.anchor2_x, args.anchor2_y)
    else:
        zone2 = candidates.get(2) or [(-1.25, -0.75)]
        a2 = min(zone2, key=lambda point: math.hypot(point[0] - a1[0], point[1] - a1[1]))
    return (float(a1[0]), float(a1[1])), (float(a2[0]), float(a2[1]))


@dataclass(frozen=True)
class SegmentStatus:
    along_m: float
    raw_cte_m: float
    remaining_m: float
    distance_m: float
    reached: bool
    passed: bool


def segment_status(
    start: tuple[float, float],
    goal: tuple[float, float],
    robot: tuple[float, float],
    reach_tolerance_m: float,
    pass_lateral_tolerance_m: float,
) -> SegmentStatus:
    sx, sy = start
    gx, gy = goal
    rx, ry = robot
    dx, dy = gx - sx, gy - sy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return SegmentStatus(0.0, 0.0, 0.0, 0.0, True, False)
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    px, py = rx - sx, ry - sy
    along = px * ux + py * uy
    cte = px * nx + py * ny
    distance = math.hypot(gx - rx, gy - ry)
    passed = along >= length
    reached = distance <= reach_tolerance_m or (
        passed and abs(cte) <= pass_lateral_tolerance_m
    )
    return SegmentStatus(along, cte, length - along, distance, reached, passed)


@dataclass
class LaneFeedbackState:
    filtered_cte_m: float | None = None
    omega: float = 0.0

    def reset(self) -> None:
        self.filtered_cte_m = None
        self.omega = 0.0


@dataclass(frozen=True)
class LaneCommand:
    vx: float
    omega: float
    filtered_cte_m: float
    band_excess_m: float
    heading_bias_deg: float
    heading_error_deg: float


def lane_feedback_command(
    *,
    raw_cte_m: float,
    robot_heading: float,
    path_heading: float,
    remaining_m: float,
    dt: float,
    state: LaneFeedbackState,
    args: argparse.Namespace,
) -> LaneCommand:
    alpha = clamp(args.cte_filter_alpha, 0.0, 1.0)
    if state.filtered_cte_m is None:
        state.filtered_cte_m = float(raw_cte_m)
    else:
        state.filtered_cte_m += alpha * (float(raw_cte_m) - state.filtered_cte_m)

    effective_cte = signed_deadband(state.filtered_cte_m, args.center_band_half_width_m)
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
        -args.max_drive_omega,
        args.max_drive_omega,
    )
    if (
        abs(state.filtered_cte_m) <= args.center_band_half_width_m
        and abs(heading_error) <= math.radians(args.heading_deadband_deg)
    ):
        omega_target = 0.0
    max_delta = max(0.0, args.omega_slew_rad_s2) * max(0.0, dt)
    state.omega += clamp(omega_target - state.omega, -max_delta, max_delta)

    vx = args.speed_mps
    abs_cte = abs(state.filtered_cte_m)
    if abs_cte > args.slow_cte_m:
        ratio = clamp(
            (abs_cte - args.slow_cte_m)
            / max(0.01, args.abort_cte_m - args.slow_cte_m),
            0.0,
            1.0,
        )
        vx = args.speed_mps + ratio * (args.min_speed_mps - args.speed_mps)
    if remaining_m < args.goal_slow_distance_m:
        ratio = clamp(remaining_m / max(0.01, args.goal_slow_distance_m), 0.0, 1.0)
        vx = min(vx, args.min_speed_mps + ratio * (args.speed_mps - args.min_speed_mps))

    return LaneCommand(
        vx=max(0.0, vx),
        omega=state.omega,
        filtered_cte_m=state.filtered_cte_m,
        band_excess_m=max(0.0, abs(state.filtered_cte_m) - args.center_band_half_width_m),
        heading_bias_deg=math.degrees(heading_bias),
        heading_error_deg=math.degrees(heading_error),
    )


class AnchorLaneTestNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("shape4_anchor1_to_anchor2_lane_test_v5_6_0")
        self.args = args
        self.world: WorldModel | None = None
        self.world_rx_s = 0.0
        self.direct_pose: tuple[float, float, float] | None = None
        self.direct_pose_rx_s = 0.0
        self.competition_state = "UNKNOWN"

        self.cmd_pub = self.create_publisher(BaseCommand, "/base_command", 10)
        self.obstacles_pub = self.create_publisher(PoseArray, "/planning/obstacles", 10)
        self.reset_pose_pub = self.create_publisher(PoseStamped, "/localization/reset_pose", 10)
        self.mapping_pub = self.create_publisher(Bool, "/world_model/mapping_enabled", 10)
        self.track_birth_pub = self.create_publisher(
            Bool, "/world_model/track_birth_enabled", 10
        )
        self.wall_fast_pub = self.create_publisher(
            Bool, "/localization/wall_fast_correction", 10
        )
        self.wall_mode_pub = self.create_publisher(
            String, "/localization/wall_correction_mode", 10
        )
        self.mission_state_pub = self.create_publisher(MissionState, "/mission_state", 10)

        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(
            String, "/competition/state", self.on_competition, COMPETITION_QOS
        )

    def on_world(self, msg: WorldModel) -> None:
        self.world = msg
        self.world_rx_s = time.monotonic()

    def on_pose(self, msg: PoseStamped) -> None:
        self.direct_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            yaw_from_pose(msg),
        )
        self.direct_pose_rx_s = time.monotonic()

    def on_competition(self, msg: String) -> None:
        self.competition_state = str(msg.data).strip().upper()

    def world_pose(self) -> tuple[float, float, float] | None:
        if self.world is None:
            return None
        if time.monotonic() - self.world_rx_s > self.args.pose_timeout_sec:
            return None
        return (
            float(self.world.robot_x),
            float(self.world.robot_y),
            float(self.world.robot_theta),
        )

    def direct_pose_fresh(self) -> tuple[float, float, float] | None:
        if self.direct_pose is None:
            return None
        if time.monotonic() - self.direct_pose_rx_s > self.args.pose_timeout_sec:
            return None
        return self.direct_pose

    def publish_command(self, vx: float, omega: float) -> None:
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.vx = float(vx)
        msg.vy = 0.0
        msg.omega = float(omega)
        self.cmd_pub.publish(msg)

    def publish_control_state(self, mapping_enabled: bool, track_birth: bool) -> None:
        self.mapping_pub.publish(Bool(data=bool(mapping_enabled)))
        self.track_birth_pub.publish(Bool(data=bool(track_birth)))
        self.wall_fast_pub.publish(Bool(data=False))
        self.wall_mode_pub.publish(String(data="OFF"))
        state = MissionState()
        state.state = "ANCHOR_ROUTE_TEST"
        state.stamp = self.get_clock().now().to_msg()
        self.mission_state_pub.publish(state)

    def publish_reset_pose(self, x: float, y: float, heading: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qz, qw = quat_z_w(heading)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.reset_pose_pub.publish(msg)

    def publish_obstacles(self, obstacles: list[tuple[float, float]]) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        for ox, oy in obstacles:
            pose = Pose()
            pose.position.x = float(ox)
            pose.position.y = float(oy)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.obstacles_pub.publish(msg)

    def stop(self, duration_sec: float = 0.45) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        period = 1.0 / max(5.0, self.args.control_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(0.0, 0.0)
            self.publish_control_state(False, False)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(period)

    def other_base_publishers(self) -> list[str]:
        others: list[str] = []
        for info in self.get_publishers_info_by_topic("/base_command"):
            if info.node_name != self.get_name():
                prefix = info.node_namespace.rstrip("/")
                others.append(f"{prefix}/{info.node_name}" if prefix else info.node_name)
        return sorted(set(others))

    def forbidden_nodes(self) -> list[str]:
        found = []
        for name, namespace in self.get_node_names_and_namespaces():
            if "wall_localizer" in name or "mission_fsm" in name:
                prefix = namespace.rstrip("/")
                found.append(f"{prefix}/{name}" if prefix else name)
        return sorted(set(found))


def obstacle_snapshot(
    world: WorldModel,
    destination: tuple[float, float],
    args: argparse.Namespace,
) -> list[tuple[float, float]]:
    obstacles = []
    for obj in world.objects:
        if int(obj.set_type) == 3:
            continue
        if float(obj.confidence) < args.obstacle_min_conf:
            continue
        if int(obj.n_obs) < args.obstacle_min_nobs:
            continue
        point = (float(obj.x), float(obj.y))
        if math.hypot(point[0] - destination[0], point[1] - destination[1]) < args.exclude_goal_radius_m:
            continue
        obstacles.append(point)
    return obstacles


def build_planner(params: dict, args: argparse.Namespace) -> LanePlanner:
    bounds = [float(value) for value in params.get("field_bounds_m", [-2, 2, -2, 2])]
    origin = [float(value) for value in params.get("grid_origin_xy", [0, 0])]
    return LanePlanner(
        spacing=float(params.get("grid_spacing_m", 0.50)),
        bounds=tuple(bounds[:4]),
        margin=float(params.get("robot_margin_m", 0.22)),
        block_radius=float(params.get("lane_block_radius_m", 0.10)),
        comfort_clear=float(params.get("comfort_clear_m", 0.15)),
        clearance_weight=float(params.get("clearance_weight", 0.5)),
        phase_tol=float(params.get("phase_tol_m", 0.08)),
        origin_mode=str(params.get("grid_origin_mode", "fixed")),
        origin_xy=tuple(origin[:2]),
        start_connect_k=int(params.get("start_connect_k", 4)),
        simplify=bool(params.get("lane_simplify_enabled", False)),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Main-style LanePlanner A1->A2 test with a +/-5 cm centre band."
    )
    parser.add_argument("--tuning-file", default=DEFAULT_TUNING_FILE)
    parser.add_argument("--anchor1-x", type=float, default=None)
    parser.add_argument("--anchor1-y", type=float, default=None)
    parser.add_argument("--anchor2-x", type=float, default=None)
    parser.add_argument("--anchor2-y", type=float, default=None)
    parser.add_argument("--start-anchor-tolerance-m", type=float, default=0.15)
    parser.add_argument("--center-band-half-width-m", type=float, default=0.05)
    parser.add_argument("--lookahead-m", type=float, default=0.55)
    parser.add_argument("--cross-track-gain", type=float, default=1.0)
    parser.add_argument("--heading-kp", type=float, default=1.20)
    parser.add_argument("--heading-deadband-deg", type=float, default=2.5)
    parser.add_argument("--max-heading-bias-deg", type=float, default=10.0)
    parser.add_argument("--max-drive-omega", type=float, default=0.08)
    parser.add_argument("--omega-slew-rad-s2", type=float, default=0.20)
    parser.add_argument("--cte-filter-alpha", type=float, default=0.20)
    parser.add_argument("--speed-mps", type=float, default=0.10)
    parser.add_argument("--min-speed-mps", type=float, default=0.055)
    parser.add_argument("--slow-cte-m", type=float, default=0.10)
    parser.add_argument("--abort-cte-m", type=float, default=0.20)
    parser.add_argument("--goal-slow-distance-m", type=float, default=0.30)
    parser.add_argument("--waypoint-reach-tol-m", type=float, default=0.12)
    parser.add_argument("--pass-lateral-tol-m", type=float, default=0.10)
    parser.add_argument("--align-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--hard-realign-deg", type=float, default=18.0)
    parser.add_argument("--obstacle-min-conf", type=float, default=0.40)
    parser.add_argument("--obstacle-min-nobs", type=int, default=2)
    parser.add_argument("--exclude-goal-radius-m", type=float, default=0.30)
    parser.add_argument("--map-warmup-sec", type=float, default=5.0)
    parser.add_argument("--min-world-objects", type=int, default=4)
    parser.add_argument("--pose-timeout-sec", type=float, default=0.70)
    parser.add_argument("--pose-agreement-m", type=float, default=0.08)
    parser.add_argument("--pose-agreement-deg", type=float, default=5.0)
    parser.add_argument("--motion-proof-sec", type=float, default=2.5)
    parser.add_argument("--motion-proof-m", type=float, default=0.03)
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--log-rate-hz", type=float, default=4.0)
    parser.add_argument("--csv", default="/tmp/anchor1_anchor2_lane_feedback.csv")
    parser.add_argument("--require-running", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--no-confirm", action="store_true")
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    checks = [
        (0.03 <= args.center_band_half_width_m <= 0.10,
         "center band half width must be in [0.03, 0.10]"),
        (0.25 <= args.lookahead_m <= 1.20, "lookahead must be in [0.25, 1.20]"),
        (0.03 <= args.min_speed_mps <= args.speed_mps <= 0.18,
         "require 0.03 <= min speed <= speed <= 0.18"),
        (args.center_band_half_width_m < args.slow_cte_m < args.abort_cte_m,
         "require centre band < slow CTE < abort CTE"),
        (0.02 <= args.max_drive_omega <= 0.20, "max drive omega must be in [0.02, 0.20]"),
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
        "time_s", "segment", "world_x", "world_y", "world_heading_deg",
        "direct_x", "direct_y", "direct_heading_deg", "goal_x", "goal_y",
        "along_m", "remaining_m", "raw_cte_m", "filtered_cte_m",
        "band_excess_m", "heading_bias_deg", "heading_error_deg",
        "vx", "vy", "omega", "obstacle_count",
    ])
    return handle, writer


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    tuning_path = os.path.abspath(args.tuning_file)
    params = yaml_params(tuning_path, "mission_fsm_node")
    anchor1, anchor2 = resolve_anchors(params, args)
    route_heading = math.atan2(anchor2[1] - anchor1[1], anchor2[0] - anchor1[0])
    planner = build_planner(params, args)

    rclpy.init()
    node = AnchorLaneTestNode(args)
    csv_handle = None
    try:
        node.get_logger().info(
            f"A1->A2 lane test: A1={anchor1}, A2={anchor2}, opening=OFF, wall=OFF"
        )
        discovery_deadline = time.monotonic() + 0.8
        while time.monotonic() < discovery_deadline:
            node.publish_command(0.0, 0.0)
            node.publish_control_state(False, False)
            rclpy.spin_once(node, timeout_sec=0.05)
        forbidden = node.forbidden_nodes()
        if forbidden:
            node.get_logger().error(
                f"forbidden main/wall nodes are running: {forbidden}; disable FSM and wall localizer"
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

        # Opening is disabled, so establish its trusted endpoint directly: mapping stays off until
        # localizer and WorldModel both see A1 with the A1->A2 heading.
        reset_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < reset_deadline:
            node.publish_command(0.0, 0.0)
            node.publish_control_state(False, False)
            node.publish_reset_pose(anchor1[0], anchor1[1], route_heading)
            rclpy.spin_once(node, timeout_sec=0.03)
            time.sleep(0.04)

        settle_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            node.publish_command(0.0, 0.0)
            node.publish_control_state(False, False)
            rclpy.spin_once(node, timeout_sec=0.04)
        world_pose = node.world_pose()
        direct_pose = node.direct_pose_fresh()
        if world_pose is None or direct_pose is None:
            node.get_logger().error("WorldModel/localization pose did not become fresh after A1 reset")
            return 5
        if math.hypot(world_pose[0] - anchor1[0], world_pose[1] - anchor1[1]) > args.start_anchor_tolerance_m:
            node.get_logger().error(f"world pose is not at A1 after reset: {world_pose}")
            return 5

        node.get_logger().info("A1 pose established; enabling world mapping with opening skipped")
        warmup_deadline = time.monotonic() + args.map_warmup_sec
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            node.publish_command(0.0, 0.0)
            node.publish_control_state(True, True)
            rclpy.spin_once(node, timeout_sec=0.04)
        if node.world is None or node.world_pose() is None:
            node.get_logger().error("WorldModel is unavailable after mapping warmup")
            return 6
        if len(node.world.objects) < args.min_world_objects:
            node.get_logger().error(
                f"only {len(node.world.objects)} world objects; need {args.min_world_objects} for lane test"
            )
            return 6

        obstacles = obstacle_snapshot(node.world, anchor2, args)
        route = planner.plan(anchor1, anchor2, obstacles, route_mode="grid_only")
        node.publish_obstacles(obstacles)
        if not route:
            node.get_logger().error(
                f"LanePlanner found no grid_only route using {len(obstacles)} obstacles"
            )
            return 7
        starts = [anchor1] + list(route[:-1])
        for start, goal in zip(starts, route):
            if cardinal_segment_heading(start, goal, 0.06) is None:
                node.get_logger().error(f"non-cardinal route segment: {start} -> {goal}")
                return 7

        route_text = " -> ".join(f"({x:+.2f},{y:+.2f})" for x, y in [anchor1] + route)
        print(
            "\nA1 -> A2 메인형 레인 테스트\n"
            f"  경로: {route_text}\n"
            f"  장애물: {len(obstacles)}개 / WorldModel: {len(node.world.objects)}개\n"
            f"  중심 영역: 계획 중심선 양옆 +/-{args.center_band_half_width_m * 100:.1f}cm\n"
            f"  속도: {args.speed_mps:.3f}m/s, vy=0\n"
            "  opening: OFF, wall correction: OFF\n"
        )
        if not args.no_confirm:
            input("로봇이 A1에서 A2를 향하고 비상정지 공간이 확보됐으면 Enter: ")

        if args.require_running:
            wait_deadline = time.monotonic() + 60.0
            next_wait_log = 0.0
            while rclpy.ok() and time.monotonic() < wait_deadline:
                node.publish_command(0.0, 0.0)
                node.publish_control_state(True, True)
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if node.competition_state == "RUNNING":
                    break
                if now >= next_wait_log:
                    node.get_logger().info(
                        f"waiting for competition RUNNING; current={node.competition_state}"
                    )
                    next_wait_log = now + 1.0
            else:
                node.get_logger().error("competition did not reach RUNNING")
                return 8

        csv_handle, csv_writer = open_csv(args.csv)
        test_start_s = time.monotonic()
        last_tick_s = test_start_s
        next_log_s = test_start_s
        period = 1.0 / args.control_rate_hz
        segment_index = 0
        segment_start = anchor1
        feedback = LaneFeedbackState()
        heading_controller = PulsedHeadingController()
        phase = "align"
        settle_start_s = 0.0
        drive_start_pose: tuple[float, float] | None = None
        drive_start_s = 0.0
        max_abs_cte = 0.0
        cte_sq_sum = 0.0
        samples = 0
        outside_band_samples = 0

        while rclpy.ok():
            tick_s = time.monotonic()
            elapsed = tick_s - test_start_s
            if elapsed >= args.timeout_sec:
                node.get_logger().error("A1->A2 test timeout")
                return 9
            rclpy.spin_once(node, timeout_sec=0.005)
            node.publish_control_state(True, True)
            node.publish_obstacles(obstacles)
            forbidden = node.forbidden_nodes()
            if forbidden:
                node.get_logger().error(f"forbidden node appeared: {forbidden}")
                return 2
            others = node.other_base_publishers()
            if others:
                node.get_logger().error(f"another base publisher appeared: {others}")
                return 3
            if args.require_running and node.competition_state != "RUNNING":
                node.get_logger().error(
                    f"competition left RUNNING ({node.competition_state}); stopping"
                )
                return 8

            world_pose = node.world_pose()
            direct_pose = node.direct_pose_fresh()
            if world_pose is None or direct_pose is None:
                node.get_logger().error("WorldModel/direct localization pose is stale")
                return 5
            pose_delta = math.hypot(world_pose[0] - direct_pose[0], world_pose[1] - direct_pose[1])
            heading_delta = abs(wrap_pi(world_pose[2] - direct_pose[2]))
            if pose_delta > args.pose_agreement_m or heading_delta > math.radians(args.pose_agreement_deg):
                node.get_logger().error(
                    f"world/direct pose mismatch: dxy={pose_delta:.3f}m "
                    f"dtheta={math.degrees(heading_delta):.1f}deg"
                )
                return 5

            goal = route[segment_index]
            target_heading = cardinal_segment_heading(segment_start, goal, 0.06)
            assert target_heading is not None
            status = segment_status(
                segment_start,
                goal,
                (world_pose[0], world_pose[1]),
                args.waypoint_reach_tol_m,
                args.pass_lateral_tol_m,
            )
            if status.passed and not status.reached:
                node.get_logger().error(
                    f"segment {segment_index + 1} missed: CTE={status.raw_cte_m:+.3f}m"
                )
                return 10
            if status.reached:
                node.publish_command(0.0, 0.0)
                if segment_index >= len(route) - 1:
                    rms = math.sqrt(cte_sq_sum / max(1, samples))
                    outside_ratio = 100.0 * outside_band_samples / max(1, samples)
                    node.stop()
                    print(
                        "\nA2 도착 성공\n"
                        f"  최종 오차: 거리 {status.distance_m * 100:.1f}cm, "
                        f"CTE {status.raw_cte_m * 100:+.1f}cm\n"
                        f"  최대 |CTE|: {max_abs_cte * 100:.1f}cm\n"
                        f"  RMS CTE: {rms * 100:.1f}cm\n"
                        f"  +/-5cm 영역 밖 샘플: {outside_ratio:.1f}%\n"
                    )
                    return 0
                previous_heading = target_heading
                segment_index += 1
                segment_start = goal
                feedback.reset()
                heading_controller.reset()
                next_heading = cardinal_segment_heading(
                    segment_start, route[segment_index], 0.06
                )
                # Match the main FSM's monotonic waypoint advance: collinear raw grid vias do
                # not introduce a stop/settle cycle.  Only an actual 90-degree corner realigns.
                if (
                    next_heading is not None
                    and abs(wrap_pi(next_heading - previous_heading))
                    <= math.radians(args.align_tolerance_deg)
                ):
                    phase = "drive"
                    drive_start_pose = (world_pose[0], world_pose[1])
                    drive_start_s = tick_s
                else:
                    phase = "align"
                    drive_start_pose = None
                node.get_logger().info(
                    f"waypoint reached; segment {segment_index + 1}/{len(route)} "
                    f"phase={phase}"
                )
                continue

            heading_error = wrap_pi(target_heading - world_pose[2])
            if phase == "align":
                aligned, omega, event = heading_controller.step(
                    now_s=tick_s,
                    current_heading=world_pose[2],
                    target_heading=target_heading,
                    tolerance_rad=math.radians(args.align_tolerance_deg),
                    key=(segment_index, round(target_heading, 4)),
                    omega_limit=None,
                )
                node.publish_command(0.0, omega)
                if event in {"coarse_pulse", "fine_pulse", "timeout"}:
                    node.get_logger().info(
                        f"segment {segment_index + 1} ALIGN {event} "
                        f"error={math.degrees(heading_error):+.1f}deg"
                    )
                if event == "timeout":
                    return 11
                if aligned:
                    phase = "settle"
                    settle_start_s = tick_s
                continue
            if phase == "settle":
                node.publish_command(0.0, 0.0)
                if tick_s - settle_start_s >= 0.30:
                    phase = "drive"
                    drive_start_pose = (world_pose[0], world_pose[1])
                    drive_start_s = tick_s
                    feedback.reset()
                continue
            if abs(heading_error) >= math.radians(args.hard_realign_deg):
                node.publish_command(0.0, 0.0)
                heading_controller.reset()
                phase = "align"
                node.get_logger().warn(
                    f"hard heading error {math.degrees(heading_error):+.1f}deg -> re-align"
                )
                continue

            max_abs_cte = max(max_abs_cte, abs(status.raw_cte_m))
            cte_sq_sum += status.raw_cte_m * status.raw_cte_m
            samples += 1
            if abs(status.raw_cte_m) > args.center_band_half_width_m:
                outside_band_samples += 1
            if abs(status.raw_cte_m) >= args.abort_cte_m:
                node.get_logger().error(
                    f"CTE {status.raw_cte_m:+.3f}m reached abort limit"
                )
                return 12
            if (
                drive_start_pose is not None
                and tick_s - drive_start_s >= args.motion_proof_sec
                and math.hypot(
                    world_pose[0] - drive_start_pose[0],
                    world_pose[1] - drive_start_pose[1],
                ) < args.motion_proof_m
            ):
                node.get_logger().error(
                    "vx command is active but world pose translation is frozen; stopping"
                )
                return 13

            dt = clamp(tick_s - last_tick_s, 0.0, 0.20)
            last_tick_s = tick_s
            command = lane_feedback_command(
                raw_cte_m=status.raw_cte_m,
                robot_heading=world_pose[2],
                path_heading=target_heading,
                remaining_m=status.remaining_m,
                dt=dt,
                state=feedback,
                args=args,
            )
            node.publish_command(command.vx, command.omega)

            if csv_writer is not None:
                csv_writer.writerow([
                    f"{elapsed:.4f}", segment_index + 1,
                    f"{world_pose[0]:.5f}", f"{world_pose[1]:.5f}",
                    f"{math.degrees(world_pose[2]):.3f}",
                    f"{direct_pose[0]:.5f}", f"{direct_pose[1]:.5f}",
                    f"{math.degrees(direct_pose[2]):.3f}",
                    f"{goal[0]:.5f}", f"{goal[1]:.5f}",
                    f"{status.along_m:.5f}", f"{status.remaining_m:.5f}",
                    f"{status.raw_cte_m:.5f}", f"{command.filtered_cte_m:.5f}",
                    f"{command.band_excess_m:.5f}",
                    f"{command.heading_bias_deg:.3f}",
                    f"{command.heading_error_deg:.3f}",
                    f"{command.vx:.5f}", "0.00000", f"{command.omega:.5f}",
                    len(obstacles),
                ])
            if tick_s >= next_log_s:
                message = (
                    f"SEG={segment_index + 1}/{len(route)} "
                    f"pose=({world_pose[0]:+.2f},{world_pose[1]:+.2f},"
                    f"{math.degrees(world_pose[2]):+.1f}deg) "
                    f"remain={status.remaining_m:.2f}m "
                    f"CTE={status.raw_cte_m * 100:+.1f}cm "
                    f"filt={command.filtered_cte_m * 100:+.1f}cm "
                    f"outside={command.band_excess_m * 100:.1f}cm "
                    f"bias={command.heading_bias_deg:+.1f}deg "
                    f"cmd=({command.vx:.3f},0.000,{command.omega:+.3f})"
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
