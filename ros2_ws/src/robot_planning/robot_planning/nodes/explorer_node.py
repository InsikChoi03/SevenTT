"""Active exploration during SCAN: roam a bounded pattern so the wide cam sees the whole field.

The mission FSM's SCAN state is passive (it just waits for the world model to hold >=1 object).
Since objects are scattered and not all initially visible, this node roams while state == SCAN
by publishing /base/goal_pose waypoints (reusing the proven go_to_goal drive path). It goes
COMPLETELY SILENT in every other state so it never fights the FSM's APPROACH goals (they are
mutually exclusive by state). Localization is dead-reckoning only outside the arena, so the
pattern is deliberately small and bounded: an in-place yaw survey at the origin plus a short
ring of survey points, looped until SCAN ends.

Subscribes: /mission_state (MissionState), /localization/pose (PoseStamped).
Publishes:  /base/goal_pose (PoseStamped, field frame).
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from robot_interfaces.msg import MissionState


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("explorer_node")

        self.declare_parameter("explore_bounds_m", 1.5)     # half-extent of the roam box
        self.declare_parameter("explore_step_m", 0.5)
        self.declare_parameter("yaw_sweep_step_rad", math.pi / 4.0)
        self.declare_parameter("waypoint_settle_sec", 1.0)  # dwell after arriving (let cams see)
        self.declare_parameter("waypoint_max_dwell_sec", 6.0)  # give up on a waypoint after this
        self.declare_parameter("pos_tol_m", 0.12)
        self.declare_parameter("yaw_tol_rad", 0.15)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("scan_state_name", "SCAN")

        self.bounds = float(self.get_parameter("explore_bounds_m").value)
        self.step = float(self.get_parameter("explore_step_m").value)
        self.yaw_step = float(self.get_parameter("yaw_sweep_step_rad").value)
        self.settle = float(self.get_parameter("waypoint_settle_sec").value)
        self.max_dwell = float(self.get_parameter("waypoint_max_dwell_sec").value)
        self.pos_tol = float(self.get_parameter("pos_tol_m").value)
        self.yaw_tol = float(self.get_parameter("yaw_tol_rad").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.scan_state = str(self.get_parameter("scan_state_name").value)

        self.waypoints = self._build_waypoints()
        self.wp_idx = 0
        self.wp_enter_s = self._now_s()
        self.wp_reached_s: float | None = None

        self.state = ""
        self.pose_xytheta: tuple[float, float, float] | None = None

        self.create_subscription(MissionState, "/mission_state", self.on_mission, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/base/goal_pose", 10)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.tick)

        self.get_logger().info(
            f"explorer: {len(self.waypoints)} waypoints  bounds={self.bounds}m "
            f"yaw_step={self.yaw_step:.2f} settle={self.settle}s (roams only in state '{self.scan_state}')"
        )

    # ------------------------------------------------------------------ setup
    def _build_waypoints(self) -> list[tuple[float, float, float]]:
        wps: list[tuple[float, float, float]] = []
        # 1) in-place yaw survey at the origin (the 150-deg fisheye sees a lot per heading).
        th = 0.0
        while th < 2.0 * math.pi - 1e-3:
            wps.append((0.0, 0.0, th))
            th += self.yaw_step
        # 2) a small ring of survey points, each facing outward with a short sweep.
        ring_r = min(self.bounds, max(self.step, self.bounds * 0.6))
        for cx, cy in ((ring_r, 0.0), (0.0, ring_r), (-ring_r, 0.0), (0.0, -ring_r)):
            face = math.atan2(cy, cx)
            wps.append((cx, cy, face))
            wps.append((cx, cy, wrap_pi(face + self.yaw_step)))
            wps.append((cx, cy, wrap_pi(face - self.yaw_step)))
        return wps or [(0.0, 0.0, 0.0)]

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ callbacks
    def on_mission(self, msg: MissionState) -> None:
        new_state = str(msg.state)
        if new_state != self.state:
            # entering SCAN afresh -> restart the current waypoint's timers
            if new_state == self.scan_state:
                self.wp_enter_s = self._now_s()
                self.wp_reached_s = None
            self.state = new_state

    def on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        self.pose_xytheta = (msg.pose.position.x, msg.pose.position.y, yaw)

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        if self.state != self.scan_state:
            return  # silent outside SCAN so we never fight the FSM's approach goals

        gx, gy, gyaw = self.waypoints[self.wp_idx]
        self._publish_goal(gx, gy, gyaw)

        now = self._now_s()
        reached = False
        if self.pose_xytheta is not None:
            rx, ry, rt = self.pose_xytheta
            if math.hypot(gx - rx, gy - ry) < self.pos_tol and abs(wrap_pi(gyaw - rt)) < self.yaw_tol:
                reached = True
        if reached and self.wp_reached_s is None:
            self.wp_reached_s = now

        held = self.wp_reached_s is not None and (now - self.wp_reached_s) >= self.settle
        timed_out = (now - self.wp_enter_s) >= self.max_dwell
        if held or timed_out:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self.wp_enter_s = now
            self.wp_reached_s = None

    def _publish_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
