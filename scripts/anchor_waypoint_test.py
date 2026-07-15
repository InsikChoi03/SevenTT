#!/usr/bin/env python3
"""Drive the base through zone anchors and hold still for mapping checks.

Default test:
  wait 15s for perception/YOLO, then drive near A1 -> brake -> face zone grid
  center -> stabilize -> near A2 -> brake -> face zone grid center -> stabilize.
Checkpoint route test:
  --route checkpoint_sweep drives through a lawnmower-like set of checked points.

The goal heading at each anchor faces the mean center of the grid points that
belong to that zone, using the same grid and zone defaults as motion_tuning.
For the A1/A2 anchor test, A1 faces A2 and A2 faces left by default.
Arrival is position-based by default because this test checks map stability near
an anchor, not centimeter-level docking.
Run this with the mission FSM disabled so this script is the only /base/goal_pose
writer.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, WorldModel

try:
    import yaml
except Exception:  # noqa: BLE001 - fallback to built-in defaults
    yaml = None


DEFAULT_ANCHORS = [-1.25, 1.25, -1.25, -1.75, 1.25, -1.75, 1.25, 1.25]
DEFAULT_CHECKPOINTS = [
    -1.8, 1.6,      # P1: heading up
    -1.8, -1.6,     # P2: heading left
    -0.75, -1.6,    # P3: heading down
    -0.75, 1.6,     # P4: heading left
    0.25, 1.6,      # P5: heading up
    0.25, -1.6,     # P6: heading left
    1.25, -1.6,     # P7: heading down
    1.25, 1.6,      # P8: heading left, route ends
]
DEFAULT_CHECKPOINT_HEADINGS = [
    -math.pi / 2.0,
    0.0,
    math.pi / 2.0,
    0.0,
    -math.pi / 2.0,
    0.0,
    math.pi / 2.0,
    0.0,
]
DEFAULT_ZONE_BOUNDS = [
    -2.0, 0.1, -0.25, 2.0,    # Z1
    -2.0, 0.1, -2.0, -0.25,   # Z2
    -0.1, 2.0, -2.0, -0.25,   # Z3
    -0.1, 2.0, -0.25, 2.0,    # Z4
]
DEFAULT_TUNING_FILE = "ros2_ws/src/robot_bringup/config/motion_tuning.yaml"


@dataclass(frozen=True)
class Waypoint:
    zone: int
    x: float
    y: float
    yaw: float
    center_x: float
    center_y: float
    grid_count: int
    label: str = ""


def parse_floats(text: str) -> list[float]:
    if text is None:
        return []
    vals: list[float] = []
    for part in text.replace(";", ",").split(","):
        part = part.strip()
        if part:
            vals.append(float(part))
    return vals


def _yaml_params(data: dict, node_name: str) -> dict:
    node = data.get(node_name, {}) if isinstance(data, dict) else {}
    params = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
    return params if isinstance(params, dict) else {}


def apply_tuning_file_defaults(args: argparse.Namespace) -> None:
    if not args.tuning_file or yaml is None:
        return
    path = args.tuning_file
    if not os.path.isabs(path):
        path = os.path.abspath(path)
    if not os.path.exists(path):
        return
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    params = _yaml_params(data, "anchor_waypoint_test")
    if args.route == "checkpoint_sweep":
        if not args.checkpoints and "checkpoint_route_xy" in params:
            args.checkpoints = ",".join(str(v) for v in params["checkpoint_route_xy"])
        if not args.checkpoint_headings and "checkpoint_route_heading_rad" in params:
            args.checkpoint_headings = ",".join(str(v) for v in params["checkpoint_route_heading_rad"])


def parse_zones(text: str) -> list[int]:
    zones = [int(v.strip()) for v in text.replace(";", ",").split(",") if v.strip()]
    bad = [z for z in zones if z < 1 or z > 4]
    if bad:
        raise ValueError(f"zones must be in 1..4, got {bad}")
    return zones


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    return 2.0 * math.atan2(float(q.z), float(q.w))


def zone_bounds(bounds: list[float], zone: int) -> tuple[float, float, float, float]:
    i = (zone - 1) * 4
    if len(bounds) < i + 4:
        raise ValueError("zone bounds must contain 16 floats")
    return bounds[i], bounds[i + 1], bounds[i + 2], bounds[i + 3]


def grid_points(args: argparse.Namespace) -> Iterable[tuple[float, float]]:
    for row in range(args.grid_rows):
        y = args.grid_origin_y + row * args.grid_spacing
        for col in range(args.grid_cols):
            x = args.grid_origin_x + col * args.grid_spacing
            yield x, y


def zone_grid_center(args: argparse.Namespace, bounds: list[float], zone: int) -> tuple[float, float, int]:
    xmin, xmax, ymin, ymax = zone_bounds(bounds, zone)
    pts = [(x, y) for (x, y) in grid_points(args) if xmin <= x <= xmax and ymin <= y <= ymax]
    if not pts:
        # Fallback: geometric zone center if a config typo excludes all grid points.
        return (0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0)
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return cx, cy, len(pts)


def make_waypoints(args: argparse.Namespace) -> list[Waypoint]:
    if args.route == "checkpoint_sweep":
        return make_checkpoint_waypoints(args)
    anchors = parse_floats(args.anchors)
    bounds = parse_floats(args.zone_bounds)
    if len(anchors) < 8:
        raise ValueError("anchors must contain 8 floats: x1,y1,x2,y2,x3,y3,x4,y4")
    waypoints: list[Waypoint] = []
    for zone in parse_zones(args.zones):
        ax = anchors[(zone - 1) * 2]
        ay = anchors[(zone - 1) * 2 + 1]
        cx, cy, n = zone_grid_center(args, bounds, zone)
        yaw = anchor_heading(args, anchors, zone, ax, ay, cx, cy)
        waypoints.append(Waypoint(zone=zone, x=ax, y=ay, yaw=yaw,
                                  center_x=cx, center_y=cy, grid_count=n,
                                  label=f"A{zone}"))
    return waypoints


def make_checkpoint_waypoints(args: argparse.Namespace) -> list[Waypoint]:
    vals = parse_floats(args.checkpoints)
    if len(vals) < 4 or len(vals) % 2 != 0:
        raise ValueError("checkpoints must contain x,y pairs")
    pts = [(vals[i], vals[i + 1]) for i in range(0, len(vals), 2)]
    heading_vals = parse_floats(args.checkpoint_headings)
    waypoints: list[Waypoint] = []
    for i, (x, y) in enumerate(pts):
        if i < len(heading_vals):
            yaw = heading_vals[i]
            nx, ny = x + math.cos(yaw), y + math.sin(yaw)
        elif i + 1 < len(pts):
            nx, ny = pts[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        elif i > 0:
            nx, ny = x + (x - pts[i - 1][0]), y + (y - pts[i - 1][1])
            yaw = math.atan2(ny - y, nx - x)
        else:
            nx, ny = x, y - 1.0
            yaw = -math.pi / 2.0
        waypoints.append(
            Waypoint(zone=i + 1, x=x, y=y, yaw=yaw, center_x=nx, center_y=ny,
                     grid_count=0, label=f"P{i + 1}")
        )
    return waypoints


def anchor_heading(
    args: argparse.Namespace,
    anchors: list[float],
    zone: int,
    ax: float,
    ay: float,
    cx: float,
    cy: float,
) -> float:
    if args.heading_mode == "grid":
        return math.atan2(cy - ay, cx - ax)
    if args.heading_mode == "test_a1_a2":
        if zone == 1 and len(anchors) >= 4:
            # A1 faces A2, i.e. upward in the current field coordinates.
            return math.atan2(anchors[3] - ay, anchors[2] - ax)
        if zone == 2:
            # Field-left is +x in the current coordinate convention; theta 0 faces left.
            return 0.0
    return math.atan2(cy - ay, cx - ax)


def waypoint_label(wp: Waypoint) -> str:
    return wp.label or f"A{wp.zone}"


class AnchorWaypointNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("anchor_waypoint_test")
        self.args = args
        self.goal_pub = self.create_publisher(PoseStamped, "/base/goal_pose", 10)
        self.stop_pub = self.create_publisher(BaseCommand, "/base_command", 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.pose: tuple[float, float, float] | None = None
        self.pose_t = 0.0
        self.world_pose: tuple[float, float, float] | None = None
        self.world_t = 0.0
        self.object_count = 0
        self.obstacles: list[tuple[float, float]] = []
        self._last_pose_for_brake: tuple[float, float, float] | None = None
        self._last_motion_dir: tuple[float, float] | None = None

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = (float(msg.pose.position.x), float(msg.pose.position.y), yaw_from_pose(msg))
        self.pose_t = time.monotonic()

    def on_world(self, msg: WorldModel) -> None:
        self.world_pose = (float(msg.robot_x), float(msg.robot_y), float(msg.robot_theta))
        self.world_t = time.monotonic()
        self.object_count = len(msg.objects)
        self.obstacles = [(float(o.x), float(o.y)) for o in msg.objects]

    def robot_pose(self) -> tuple[float, float, float] | None:
        now = time.monotonic()
        if self.pose is not None and (now - self.pose_t) <= self.args.pose_timeout_sec:
            return self.pose
        if self.world_pose is not None and (now - self.world_t) <= self.args.pose_timeout_sec:
            return self.world_pose
        return self.pose or self.world_pose

    def publish_goal_xyyaw(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qz, qw = quat_z_w(yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)

    def publish_goal(self, wp: Waypoint) -> None:
        self.publish_goal_xyyaw(wp.x, wp.y, wp.yaw)

    def publish_target(self, x: float, y: float, final_wp: Waypoint) -> None:
        yaw = math.atan2(final_wp.center_y - y, final_wp.center_x - x)
        self.publish_goal_xyyaw(x, y, yaw)

    def publish_base_cmd(self, vx: float, vy: float, omega: float = 0.0) -> None:
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.omega = float(omega)
        self.stop_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_base_cmd(0.0, 0.0, 0.0)

    def wait_for_pose(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.robot_pose() is not None:
                return True
        return self.robot_pose() is not None

    def startup_wait(self) -> None:
        if self.args.startup_wait_sec <= 0.0:
            return
        self.get_logger().info(
            f"startup wait {self.args.startup_wait_sec:.1f}s for YOLO/world_model warmup"
        )
        deadline = time.monotonic() + self.args.startup_wait_sec
        next_log = 0.0
        period = 1.0 / max(1.0, self.args.stop_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_stop()
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_log:
                remain = max(0.0, deadline - now)
                self.get_logger().info(
                    f"startup wait: remaining={remain:.1f}s objects={self.object_count}"
                )
                next_log = now + 1.0
            time.sleep(period)

    def run_waypoint(self, wp: Waypoint) -> bool:
        label = waypoint_label(wp)
        yaw_deg = math.degrees(wp.yaw)
        self.get_logger().info(
            f"{label}: goal=({wp.x:.2f},{wp.y:.2f}) yaw={wp.yaw:.2f}rad/{yaw_deg:.1f}deg "
            f"grid_center=({wp.center_x:.2f},{wp.center_y:.2f}) grids={wp.grid_count}"
        )
        deadline = time.monotonic() + self.args.timeout_sec
        next_log = 0.0
        period = 1.0 / max(0.5, self.args.goal_rate_hz)
        self._last_pose_for_brake = None
        self._last_motion_dir = None
        detour: tuple[float, float] | None = None
        detour_done = not self.args.path_avoidance
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            pose = self.robot_pose()
            if pose is not None:
                rx, ry, rtheta = pose
                self._update_motion_dir(pose)
                if not detour_done and detour is None:
                    detour = self.plan_detour(pose, wp)
                    if detour is None:
                        detour_done = True
                    else:
                        self.get_logger().info(
                            f"{label} path avoidance: detour=({detour[0]:.2f},{detour[1]:.2f})"
                        )
                tx, ty = (detour if detour is not None and not detour_done else (wp.x, wp.y))
                self.publish_target(tx, ty, wp)
                target_dist = math.hypot(tx - rx, ty - ry)
                if detour is not None and not detour_done and target_dist <= self.args.detour_reach_tol_m:
                    self.get_logger().info(
                        f"{label} detour reached: dist={target_dist:.2f}; resuming anchor goal"
                    )
                    detour_done = True
                    detour = None
                dist = math.hypot(wp.x - rx, wp.y - ry)
                yaw_err = wrap_pi(wp.yaw - rtheta)
                now = time.monotonic()
                if now >= next_log:
                    self.get_logger().info(
                        f"{label} moving: pose=({rx:.2f},{ry:.2f},{rtheta:.2f}) "
                        f"dist={dist:.2f} target_dist={target_dist:.2f} "
                        f"yaw_err={yaw_err:.2f} objects={self.object_count}"
                    )
                    next_log = now + 1.0
                yaw_ok = (not self.args.require_yaw) or abs(yaw_err) <= self.args.yaw_tol_rad
                if dist <= self.args.reach_tol_m and yaw_ok:
                    self.get_logger().info(
                        f"{label} reached: dist={dist:.2f} yaw_err={yaw_err:.2f}; "
                        f"brake, align heading, then stabilize {self.args.stabilize_sec:.1f}s"
                    )
                    self.park_goal_at_pose(pose)
                    self.brake_pulse(pose)
                    if self.args.align_heading_after_arrival:
                        self.align_heading(wp)
                    return self.stabilize(wp)
            time.sleep(period)
        self.get_logger().error(f"{label} timeout before reaching anchor")
        return False

    def plan_detour(self, pose: tuple[float, float, float], wp: Waypoint) -> tuple[float, float] | None:
        if not self.obstacles:
            return None
        rx, ry, _ = pose
        vx = wp.x - rx
        vy = wp.y - ry
        seg_len = math.hypot(vx, vy)
        if seg_len < self.args.path_avoid_min_goal_dist_m:
            return None
        ux, uy = vx / seg_len, vy / seg_len
        px, py = -uy, ux

        blocker: tuple[float, float, float, float] | None = None
        for ox, oy in self.obstacles:
            dx, dy = ox - rx, oy - ry
            along = dx * ux + dy * uy
            if along < self.args.path_avoid_start_m or along > seg_len - self.args.path_avoid_goal_skip_m:
                continue
            lateral = abs(dx * px + dy * py)
            if lateral > self.args.path_avoid_corridor_m:
                continue
            if math.hypot(ox - wp.x, oy - wp.y) < self.args.path_avoid_goal_skip_m:
                continue
            if blocker is None or along < blocker[0]:
                blocker = (along, lateral, ox, oy)
        if blocker is None:
            return None

        _along, _lateral, ox, oy = blocker
        candidates = [
            (ox + px * self.args.path_avoid_detour_m, oy + py * self.args.path_avoid_detour_m),
            (ox - px * self.args.path_avoid_detour_m, oy - py * self.args.path_avoid_detour_m),
        ]
        xmin, xmax, ymin, ymax = self.args.field_bounds
        margin = self.args.field_margin_m
        best: tuple[float, float] | None = None
        best_score = -1e9
        for cx, cy in candidates:
            cx = min(max(cx, xmin + margin), xmax - margin)
            cy = min(max(cy, ymin + margin), ymax - margin)
            clearance = min((math.hypot(cx - x, cy - y) for x, y in self.obstacles), default=9.0)
            travel_cost = 0.25 * (math.hypot(cx - rx, cy - ry) + math.hypot(wp.x - cx, wp.y - cy))
            wall_clearance = min(cx - xmin, xmax - cx, cy - ymin, ymax - cy)
            score = clearance + 0.3 * wall_clearance - travel_cost
            if score > best_score:
                best_score = score
                best = (cx, cy)
        return best

    def _update_motion_dir(self, pose: tuple[float, float, float]) -> None:
        prev = self._last_pose_for_brake
        self._last_pose_for_brake = pose
        if prev is None:
            return
        dx = pose[0] - prev[0]
        dy = pose[1] - prev[1]
        d = math.hypot(dx, dy)
        if d >= self.args.brake_min_motion_m:
            self._last_motion_dir = (dx / d, dy / d)

    def park_goal_at_pose(self, pose: tuple[float, float, float]) -> None:
        rx, ry, rtheta = pose
        deadline = time.monotonic() + self.args.park_goal_sec
        period = 1.0 / max(1.0, self.args.goal_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_goal_xyyaw(rx, ry, rtheta)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(period)

    def brake_pulse(self, pose: tuple[float, float, float]) -> None:
        if self.args.brake_sec <= 0.0 or self.args.brake_speed <= 0.0:
            self.publish_stop()
            return
        if self._last_motion_dir is None:
            self.get_logger().info("brake pulse skipped: no recent motion direction")
            self.publish_stop()
            return
        mx, my = self._last_motion_dir
        # Field-frame reverse direction, converted into base_link command frame.
        fx = -mx * self.args.brake_speed
        fy = -my * self.args.brake_speed
        rtheta = pose[2]
        c, s = math.cos(rtheta), math.sin(rtheta)
        vx = c * fx + s * fy
        vy = -s * fx + c * fy
        self.get_logger().info(
            f"brake pulse: vx={vx:.3f} vy={vy:.3f} duration={self.args.brake_sec:.2f}s"
        )
        deadline = time.monotonic() + self.args.brake_sec
        period = 1.0 / max(1.0, self.args.brake_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_base_cmd(vx, vy, 0.0)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(period)
        for _ in range(3):
            self.publish_stop()
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(period)

    def align_heading(self, wp: Waypoint) -> bool:
        label = waypoint_label(wp)
        self.get_logger().info(
            f"{label} heading align: target yaw={wp.yaw:.2f}rad/"
            f"{math.degrees(wp.yaw):.1f}deg"
        )
        deadline = time.monotonic() + self.args.heading_timeout_sec
        next_log = 0.0
        period = 1.0 / max(1.0, self.args.heading_goal_rate_hz)
        last_pose: tuple[float, float, float] | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            pose = self.robot_pose()
            if pose is not None:
                last_pose = pose
                rx, ry, rtheta = pose
                yaw_err = wrap_pi(wp.yaw - rtheta)
                self.publish_goal_xyyaw(rx, ry, wp.yaw)
                now = time.monotonic()
                if now >= next_log:
                    self.get_logger().info(
                        f"{label} heading align: pose_yaw={rtheta:.2f} yaw_err={yaw_err:.2f}"
                    )
                    next_log = now + 0.5
                if abs(yaw_err) <= self.args.heading_yaw_tol_rad:
                    self.get_logger().info(
                        f"{label} heading aligned: yaw_err={yaw_err:.2f}"
                    )
                    self.publish_stop()
                    return True
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(period)
        if last_pose is not None:
            yaw_err = wrap_pi(wp.yaw - last_pose[2])
            self.get_logger().warn(
                f"{label} heading align timeout: yaw_err={yaw_err:.2f}"
            )
        else:
            self.get_logger().warn(f"{label} heading align timeout: no pose")
        self.publish_stop()
        return False

    def stabilize(self, wp: Waypoint) -> bool:
        label = waypoint_label(wp)
        deadline = time.monotonic() + self.args.stabilize_sec
        next_log = 0.0
        period = 1.0 / max(0.5, self.args.stop_rate_hz)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.args.hold_goal_during_stabilize:
                self.publish_goal(wp)
            else:
                self.publish_stop()
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_log:
                remain = max(0.0, deadline - now)
                pose = self.robot_pose()
                if pose is None:
                    self.get_logger().info(
                        f"{label} stabilize: remaining={remain:.1f}s objects={self.object_count}"
                    )
                else:
                    rx, ry, rtheta = pose
                    self.get_logger().info(
                        f"{label} stabilize: remaining={remain:.1f}s "
                        f"pose=({rx:.2f},{ry:.2f},{rtheta:.2f}) objects={self.object_count}"
                    )
                next_log = now + 1.0
            time.sleep(period)
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move through zone anchors and hold for map stabilization checks."
    )
    parser.add_argument("--zones", default="1,2",
                        help="zone/anchor sequence to visit, default: 1,2")
    parser.add_argument("--route", choices=("anchors", "checkpoint_sweep"), default="anchors",
                        help="anchors: A1/A2 style test; checkpoint_sweep: checked route test")
    parser.add_argument("--anchors", default=",".join(str(v) for v in DEFAULT_ANCHORS),
                        help="x1,y1,x2,y2,x3,y3,x4,y4 anchor list")
    parser.add_argument("--checkpoints", default="",
                        help="x,y checkpoint pairs for --route checkpoint_sweep")
    parser.add_argument("--checkpoint-headings",
                        default="",
                        help="heading yaw list for --route checkpoint_sweep")
    parser.add_argument("--tuning-file", default=DEFAULT_TUNING_FILE,
                        help="motion_tuning.yaml path used for checkpoint defaults")
    parser.add_argument("--zone-bounds", default=",".join(str(v) for v in DEFAULT_ZONE_BOUNDS),
                        help="z1..z4 bounds as xmin,xmax,ymin,ymax repeated 4 times")
    parser.add_argument("--grid-origin-x", type=float, default=-1.50)
    parser.add_argument("--grid-origin-y", type=float, default=-1.50)
    parser.add_argument("--grid-spacing", type=float, default=0.50)
    parser.add_argument("--grid-rows", type=int, default=6)
    parser.add_argument("--grid-cols", type=int, default=7)
    parser.add_argument("--heading-mode", choices=("test_a1_a2", "grid"), default="test_a1_a2",
                        help="test_a1_a2: A1 faces A2 and A2 faces left; grid: face zone grid center")
    parser.add_argument("--stabilize-sec", type=float, default=10.0,
                        help="hold time at each anchor after arrival")
    parser.add_argument("--startup-wait-sec", type=float, default=15.0,
                        help="initial stop time before first movement for YOLO/world_model warmup")
    parser.add_argument("--timeout-sec", type=float, default=60.0,
                        help="max travel time per anchor")
    parser.add_argument("--reach-tol-m", type=float, default=0.30,
                        help="position tolerance for starting stabilization")
    parser.add_argument("--yaw-tol-rad", type=float, default=0.35,
                        help="heading tolerance when --require-yaw is set")
    parser.add_argument("--require-yaw", action="store_true",
                        help="require final heading before starting stabilization")
    parser.add_argument("--goal-rate-hz", type=float, default=5.0,
                        help="goal publish rate while moving")
    parser.add_argument("--park-goal-sec", type=float, default=0.12,
                        help="send the current pose as the goal before braking")
    parser.add_argument("--brake-sec", type=float, default=0.12,
                        help="reverse pulse duration after anchor arrival")
    parser.add_argument("--brake-speed", type=float, default=0.035,
                        help="reverse pulse command magnitude in base command units")
    parser.add_argument("--brake-rate-hz", type=float, default=30.0,
                        help="reverse pulse publish rate")
    parser.add_argument("--brake-min-motion-m", type=float, default=0.015,
                        help="minimum pose delta used to infer travel direction")
    parser.add_argument("--align-heading-after-arrival",
                        action=argparse.BooleanOptionalAction, default=True,
                        help="after braking, rotate in place to face the zone grid center")
    parser.add_argument("--heading-timeout-sec", type=float, default=6.0,
                        help="max time for post-arrival heading alignment")
    parser.add_argument("--heading-yaw-tol-rad", type=float, default=0.18,
                        help="post-arrival heading tolerance")
    parser.add_argument("--heading-goal-rate-hz", type=float, default=8.0,
                        help="goal publish rate during post-arrival heading alignment")
    parser.add_argument("--path-avoidance", action=argparse.BooleanOptionalAction, default=False,
                        help="insert one detour waypoint when objects block the anchor path")
    parser.add_argument("--path-avoid-corridor-m", type=float, default=0.34,
                        help="objects within this distance of the direct path trigger detour")
    parser.add_argument("--path-avoid-detour-m", type=float, default=0.48,
                        help="side offset used for the detour waypoint")
    parser.add_argument("--path-avoid-start-m", type=float, default=0.22,
                        help="ignore obstacles this close to the current pose")
    parser.add_argument("--path-avoid-goal-skip-m", type=float, default=0.20,
                        help="ignore obstacles this close to the anchor goal")
    parser.add_argument("--path-avoid-min-goal-dist-m", type=float, default=0.55,
                        help="skip path detour when already close to the anchor")
    parser.add_argument("--detour-reach-tol-m", type=float, default=0.25,
                        help="distance tolerance for completing a detour waypoint")
    parser.add_argument("--field-bounds", type=float, nargs=4, default=[-2.0, 2.0, -2.0, 2.0],
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                        help="field bounds used to clamp detour points")
    parser.add_argument("--field-margin-m", type=float, default=0.25,
                        help="wall margin used to clamp detour points")
    parser.add_argument("--stop-rate-hz", type=float, default=10.0,
                        help="zero-command publish rate during stabilization")
    parser.add_argument("--hold-goal-during-stabilize", action="store_true",
                        help="keep publishing the anchor goal during stabilization")
    parser.add_argument("--pose-timeout-sec", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true",
                        help="print computed goals without publishing")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    apply_tuning_file_defaults(args)
    if args.route == "checkpoint_sweep":
        if not args.checkpoints:
            args.checkpoints = ",".join(str(v) for v in DEFAULT_CHECKPOINTS)
        if not args.checkpoint_headings:
            args.checkpoint_headings = ",".join(str(v) for v in DEFAULT_CHECKPOINT_HEADINGS)
    try:
        waypoints = make_waypoints(args)
    except Exception as exc:
        parser.error(str(exc))

    for wp in waypoints:
        print(
            f"{waypoint_label(wp)}: anchor=({wp.x:.2f},{wp.y:.2f}) "
            f"heading={wp.yaw:.3f}rad/{math.degrees(wp.yaw):.1f}deg "
            f"grid_center=({wp.center_x:.2f},{wp.center_y:.2f}) grids={wp.grid_count}"
        )
    if args.dry_run:
        return 0

    rclpy.init()
    node = AnchorWaypointNode(args)
    try:
        node.get_logger().info(
            "anchor waypoint test started. Run test_field.launch.py with with_fsm:=false."
        )
        if not node.wait_for_pose(8.0):
            node.get_logger().error("no /localization/pose or /world_model pose received")
            return 2
        node.startup_wait()
        for wp in waypoints:
            if not node.run_waypoint(wp):
                node.publish_stop()
                return 3
        node.publish_stop()
        node.get_logger().info("anchor waypoint test complete")
        return 0
    except KeyboardInterrupt:
        node.publish_stop()
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
