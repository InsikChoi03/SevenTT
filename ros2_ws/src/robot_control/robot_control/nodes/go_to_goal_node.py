"""Holonomic go-to-goal controller: drives the mecanum base toward /base/goal_pose.

Closes the gap between the FSM and the base controller:
    mission_fsm_node  --/base/goal_pose (PoseStamped, field frame)-->  [THIS NODE]
    [THIS NODE]       --/base_command  (BaseCommand vx,vy,omega base_link)--> base_controller_node

The base is holonomic (mecanum), so the field-frame position error is rotated into the
base frame and driven with a proportional law; yaw is driven to the goal orientation in
parallel. Robot pose comes from /localization/pose (preferred) with /world_model as a
fallback so the node is usable even while the localizer VO path is still partial.

Safety:
  - goal_timeout_sec: if the FSM stops publishing goals the node commands a single zero and
    then stays quiet, letting the base_controller watchdog hold the wheels at zero.
  - speeds are clamped, and a combined-speed cap keeps |(vx,vy)| <= max_lin_speed.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, WorldModel


def yaw_from_quat(qz: float, qw: float) -> float:
    """Planar yaw from a (qx=qy=0) quaternion."""
    return 2.0 * math.atan2(qz, qw)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class GoToGoalNode(Node):
    def __init__(self) -> None:
        super().__init__("go_to_goal_node")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("kp_lin", 1.2)           # m/s per m of position error
        self.declare_parameter("kp_ang", 1.5)           # rad/s per rad of yaw error
        self.declare_parameter("max_lin_speed", 0.35)   # m/s
        self.declare_parameter("max_ang_speed", 1.2)    # rad/s
        self.declare_parameter("min_lin_speed", 0.0)    # m/s stiction floor (0 = off)
        self.declare_parameter("pos_tol_m", 0.05)
        self.declare_parameter("yaw_tol_rad", 0.05)
        self.declare_parameter("goal_timeout_sec", 1.0)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("use_world_model_fallback", True)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.kp_lin = float(self.get_parameter("kp_lin").value)
        self.kp_ang = float(self.get_parameter("kp_ang").value)
        self.max_lin = float(self.get_parameter("max_lin_speed").value)
        self.max_ang = float(self.get_parameter("max_ang_speed").value)
        self.min_lin = float(self.get_parameter("min_lin_speed").value)
        self.pos_tol = float(self.get_parameter("pos_tol_m").value)
        self.yaw_tol = float(self.get_parameter("yaw_tol_rad").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self.use_wm_fallback = bool(self.get_parameter("use_world_model_fallback").value)

        # latest goal (field frame): (x, y, yaw)
        self.goal: tuple[float, float, float] | None = None
        self.goal_stamp_s: float | None = None
        # latest robot pose (field frame)
        self.pose_xytheta: tuple[float, float, float] | None = None
        self.pose_stamp_s: float | None = None
        self.wm_xytheta: tuple[float, float, float] | None = None
        self._arrived_logged = False
        self._stopped = False

        self.create_subscription(PoseStamped, "/base/goal_pose", self.on_goal, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        if self.use_wm_fallback:
            self.create_subscription(WorldModel, "/world_model", self.on_world, 10)

        self.pub = self.create_publisher(BaseCommand, "/base_command", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"go_to_goal ready: kp(lin={self.kp_lin},ang={self.kp_ang}) "
            f"max(lin={self.max_lin}m/s,ang={self.max_ang}rad/s) "
            f"tol(pos={self.pos_tol}m,yaw={self.yaw_tol}rad) rate={rate}Hz "
            f"wm_fallback={self.use_wm_fallback}"
        )

    # ------------------------------------------------------------------ utils
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_pose(self) -> tuple[float, float, float] | None:
        """Prefer fresh /localization/pose; fall back to world model; else stale pose."""
        if self.pose_xytheta is not None and self.pose_stamp_s is not None:
            if (self._now_s() - self.pose_stamp_s) <= self.pose_timeout:
                return self.pose_xytheta
        if self.use_wm_fallback and self.wm_xytheta is not None:
            return self.wm_xytheta
        return self.pose_xytheta

    def _publish(self, vx: float, vy: float, omega: float) -> None:
        cmd = BaseCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.vx = float(vx)
        cmd.vy = float(vy)
        cmd.omega = float(omega)
        self.pub.publish(cmd)
        self._stopped = (vx == 0.0 and vy == 0.0 and omega == 0.0)

    # -------------------------------------------------------------- callbacks
    def on_goal(self, msg: PoseStamped) -> None:
        yaw = yaw_from_quat(msg.pose.orientation.z, msg.pose.orientation.w)
        new = (msg.pose.position.x, msg.pose.position.y, yaw)
        if self.goal is None or math.hypot(new[0] - self.goal[0], new[1] - self.goal[1]) > 1e-4:
            self._arrived_logged = False
        self.goal = new
        self.goal_stamp_s = self._now_s()

    def on_pose(self, msg: PoseStamped) -> None:
        yaw = yaw_from_quat(msg.pose.orientation.z, msg.pose.orientation.w)
        self.pose_xytheta = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.pose_stamp_s = self._now_s()

    def on_world(self, msg: WorldModel) -> None:
        self.wm_xytheta = (msg.robot_x, msg.robot_y, msg.robot_theta)

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        # No goal, or goal went stale -> command one zero, then go quiet (watchdog holds).
        if self.goal is None or self.goal_stamp_s is None:
            return
        if (self._now_s() - self.goal_stamp_s) > self.goal_timeout:
            if not self._stopped:
                self._publish(0.0, 0.0, 0.0)
            return

        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn("no robot pose yet; holding", throttle_duration_sec=2.0)
            self._publish(0.0, 0.0, 0.0)
            return

        gx, gy, gyaw = self.goal
        rx, ry, rtheta = pose
        ex, ey = gx - rx, gy - ry
        dist = math.hypot(ex, ey)
        yaw_err = wrap_pi(gyaw - rtheta)

        if dist < self.pos_tol and abs(yaw_err) < self.yaw_tol:
            self._publish(0.0, 0.0, 0.0)
            if not self._arrived_logged:
                self.get_logger().info(f"arrived at goal ({gx:.2f},{gy:.2f}) yaw={gyaw:.2f}")
                self._arrived_logged = True
            return

        # rotate field-frame error into base frame (REP-103: x fwd, y left) by -theta
        c, s = math.cos(rtheta), math.sin(rtheta)
        bx = c * ex + s * ey
        by = -s * ex + c * ey

        vx = self.kp_lin * bx
        vy = self.kp_lin * by
        speed = math.hypot(vx, vy)
        if speed > self.max_lin and speed > 1e-9:        # combined-speed cap
            vx *= self.max_lin / speed
            vy *= self.max_lin / speed
            speed = self.max_lin
        if 0.0 < speed < self.min_lin and dist > self.pos_tol:   # stiction floor
            vx *= self.min_lin / speed
            vy *= self.min_lin / speed

        omega = max(-self.max_ang, min(self.max_ang, self.kp_ang * yaw_err))

        if dist < self.pos_tol:          # in position, just finish the heading
            vx = vy = 0.0

        self._publish(vx, vy, omega)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoToGoalNode()
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
