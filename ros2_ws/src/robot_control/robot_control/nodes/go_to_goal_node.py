"""Go-to-goal controller: drives the base toward /base/goal_pose.

Closes the gap between the FSM and the base controller:
    mission_fsm_node  --/base/goal_pose (PoseStamped, field frame)-->  [THIS NODE]
    [THIS NODE]       --/base_command  (BaseCommand vx,vy,omega base_link)--> base_controller_node

The base is mecanum, but competition travel can be forced to car-like motion: rotate
toward the waypoint, then drive forward. Robot pose comes from /localization/pose
(preferred) with /world_model as a fallback so the node is usable even while the
localizer VO path is still partial.

Safety:
  - goal_timeout_sec: if the FSM stops publishing goals the node commands a single zero and
    then stays quiet, letting the base_controller watchdog hold the wheels at zero.
  - speeds are clamped, and a combined-speed cap keeps |(vx,vy)| <= max_lin_speed.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, MissionState, WorldModel


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
        self.declare_parameter("max_ang_speed", 0.27)   # rad/s
        self.declare_parameter("min_lin_speed", 0.0)    # m/s stiction floor (0 = off)
        self.declare_parameter("pos_tol_m", 0.05)
        self.declare_parameter("yaw_tol_rad", 0.05)
        # HYBRID drive: this mecanum base drives forward cleanly. TRAVEL normally stays car-like:
        # rotate to face the goal, then drive forward. When a close obstacle enters the keep-out
        # radius, avoid with lateral-only strafe so the base does not spin while beside objects.
        self.declare_parameter("fine_radius_m", 0.15)   # <= this: holonomic fine mode; > this: car-like
        self.declare_parameter("face_tol_rad", 0.35)    # travel: only drive forward once facing within this
        # Rotation is "fast-or-stopped" too (floor clobbers small omega), so once roughly aligned STOP
        # correcting heading and just drive straight — else the base micro-rotates every tick and weaves.
        self.declare_parameter("travel_yaw_deadband_rad", 0.10)
        self.declare_parameter("goal_timeout_sec", 1.0)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("use_world_model_fallback", True)
        self.declare_parameter("lateral_motion_enabled", True)
        # Wall-collision guard: clamp every goal to the field minus the robot half-size so the base
        # never drives its 40x40 body into a wall. [xmin,xmax,ymin,ymax]; margin = half-robot + slack.
        self.declare_parameter("field_bounds_m", [0.0, 0.0, 0.0, 0.0])   # all-zero = disabled
        self.declare_parameter("robot_margin_m", 0.22)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.kp_lin = float(self.get_parameter("kp_lin").value)
        self.kp_ang = float(self.get_parameter("kp_ang").value)
        self.max_lin = float(self.get_parameter("max_lin_speed").value)
        self.max_ang = float(self.get_parameter("max_ang_speed").value)
        self.min_lin = float(self.get_parameter("min_lin_speed").value)
        self.pos_tol = float(self.get_parameter("pos_tol_m").value)
        self.yaw_tol = float(self.get_parameter("yaw_tol_rad").value)
        self.fine_radius = float(self.get_parameter("fine_radius_m").value)
        self.face_tol = float(self.get_parameter("face_tol_rad").value)
        self.travel_yaw_db = float(self.get_parameter("travel_yaw_deadband_rad").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self.use_wm_fallback = bool(self.get_parameter("use_world_model_fallback").value)
        self.lateral_motion_enabled = bool(self.get_parameter("lateral_motion_enabled").value)
        fb = [float(v) for v in self.get_parameter("field_bounds_m").value]
        self.field_bounds = fb if len(fb) == 4 and any(v != 0.0 for v in fb) else None
        self.robot_margin = float(self.get_parameter("robot_margin_m").value)
        # Obstacle avoidance: repel from mapped objects the 40x40 body would otherwise brush past.
        # avoid_radius = keep-out from an object CENTRE (robot half-width + object half + slack).
        # Objects within avoid_goal_skip of the goal are the target being approached -> not repelled.
        self.declare_parameter("avoid_radius_m", 0.34)
        self.declare_parameter("avoid_gain", 0.14)
        self.declare_parameter("avoid_goal_skip_m", 0.30)
        self.declare_parameter("use_planning_obstacles", True)
        self.declare_parameter("planning_obstacle_timeout_sec", 1.0)
        self.declare_parameter("avoid_strafe_enabled", False)
        self.declare_parameter("avoid_strafe_speed", 0.04)
        self.declare_parameter("avoid_strafe_forward_scale", 0.0)
        self.declare_parameter("avoid_strafe_latch_sec", 0.6)
        self.avoid_radius = float(self.get_parameter("avoid_radius_m").value)
        self.avoid_gain = float(self.get_parameter("avoid_gain").value)
        self.avoid_goal_skip = float(self.get_parameter("avoid_goal_skip_m").value)
        self.use_planning_obstacles = bool(self.get_parameter("use_planning_obstacles").value)
        self.planning_obstacle_timeout = float(
            self.get_parameter("planning_obstacle_timeout_sec").value
        )
        self.avoid_strafe_enabled = bool(self.get_parameter("avoid_strafe_enabled").value)
        self.avoid_strafe_speed = float(self.get_parameter("avoid_strafe_speed").value)
        self.avoid_strafe_forward_scale = float(
            self.get_parameter("avoid_strafe_forward_scale").value
        )
        self.avoid_strafe_latch_sec = float(self.get_parameter("avoid_strafe_latch_sec").value)
        self._avoid_strafe_sign = 1.0
        self._avoid_strafe_sign_t: float | None = None
        self._obstacles: list[tuple[float, float]] = []   # mapped object centres (field xy)
        self._planning_obstacles: list[tuple[float, float]] = []
        self._planning_obstacles_t: float | None = None
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
        # Yield /base_command to the FSM during ALIGN/PICK (it visual-servos the base itself).
        self._mission_state = ""
        self.create_subscription(MissionState, "/mission_state", self.on_mission_state, 10)
        # Always subscribe: world model gives the fallback pose AND the obstacle list for avoidance.
        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(PoseArray, "/planning/obstacles", self.on_planning_obstacles, 10)

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
    def on_mission_state(self, msg: MissionState) -> None:
        self._mission_state = str(msg.state)

    def on_goal(self, msg: PoseStamped) -> None:
        yaw = yaw_from_quat(msg.pose.orientation.z, msg.pose.orientation.w)
        gx, gy = msg.pose.position.x, msg.pose.position.y
        if self.field_bounds is not None:   # keep the 40x40 base off the walls
            xmin, xmax, ymin, ymax = self.field_bounds
            m = self.robot_margin
            gx = min(max(gx, xmin + m), xmax - m)
            gy = min(max(gy, ymin + m), ymax - m)
        new = (gx, gy, yaw)
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
        # Obstacle list = every mapped object centre (even blacklisted ones are physically there).
        self._obstacles = [(o.x, o.y) for o in msg.objects]

    def on_planning_obstacles(self, msg: PoseArray) -> None:
        self._planning_obstacles = [(p.position.x, p.position.y) for p in msg.poses]
        self._planning_obstacles_t = self._now_s()

    def _avoidance_obstacles(self) -> list[tuple[float, float]]:
        if (
            self.use_planning_obstacles
            and self._planning_obstacles_t is not None
            and (self._now_s() - self._planning_obstacles_t) <= self.planning_obstacle_timeout
        ):
            return self._planning_obstacles
        return self._obstacles

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        # Yield only for the FSM's direct-drive states (opening move, ALIGN visual servo, PICK).
        # SCAN (drive to map centre) and APPROACH (drive to target) are HOLONOMIC via go_to_goal so
        # the mecanum base translates without spinning in place.
        if self._mission_state in ("OPENING", "ALIGN", "PICK"):
            if not self._stopped:
                self._publish(0.0, 0.0, 0.0)
            return
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

        c, s = math.cos(rtheta), math.sin(rtheta)

        if not self.lateral_motion_enabled:
            # Competition baseline: never command mecanum lateral travel from this waypoint
            # follower. If the waypoint is beside/behind the robot, rotate first; only drive
            # forward once the body is facing the waypoint. This keeps post-opening SCAN/anchor
            # movement on the same forward/turn axes that have proven reliable on the floor.
            if dist >= self.pos_tol:
                bearing = math.atan2(ey, ex)
                head_err = wrap_pi(bearing - rtheta)
                if abs(head_err) < self.travel_yaw_db:
                    omega = 0.0
                else:
                    omega = max(-self.max_ang, min(self.max_ang, self.kp_ang * head_err))
                if abs(head_err) < self.face_tol:
                    vx = min(self.max_lin, self.kp_lin * dist)
                    vx = max(vx, self.min_lin)
                    vx *= max(0.0, math.cos(head_err))
                else:
                    vx = 0.0
                self._publish(vx, 0.0, omega)
                return
            omega = max(-self.max_ang, min(self.max_ang, self.kp_ang * yaw_err))
            self._publish(0.0, 0.0, omega)
            return

        if dist > self.fine_radius:
            # ---- TRAVEL: usually face the goal, then drive FORWARD. If obstacle repulsion bends the
            # aim too far to drive forward, use a small strafe escape instead of rotating in place. ----
            ux, uy = ex, ey
            avoid_active = False
            avoid_side_hint = 0.0
            avoid_side_hint_dist = float("inf")
            obstacles = self._avoidance_obstacles()
            if self.avoid_radius > 0.0 and obstacles:
                for (ox, oy) in obstacles:
                    if math.hypot(ox - gx, oy - gy) < self.avoid_goal_skip:
                        continue                                 # (near) the target -> don't repel
                    odx, ody = ox - rx, oy - ry
                    do = math.hypot(odx, ody)
                    if 1e-3 < do < self.avoid_radius:
                        avoid_active = True
                        f = self.avoid_gain * (1.0 / do - 1.0 / self.avoid_radius)
                        ux -= f * odx                            # bend the aim away from the obstacle
                        uy -= f * ody
                        oby = -s * odx + c * ody
                        if abs(oby) > 0.02 and do < avoid_side_hint_dist:
                            avoid_side_hint = -math.copysign(1.0, oby)
                            avoid_side_hint_dist = do
            if self.avoid_strafe_enabled and avoid_active:
                bx = c * ux + s * uy
                by = -s * ux + c * uy
                desired_sign = math.copysign(1.0, by) if abs(by) > 1e-3 else avoid_side_hint
                now = self._now_s()
                if desired_sign != 0.0:
                    if (
                        self._avoid_strafe_sign_t is None
                        or (now - self._avoid_strafe_sign_t) >= self.avoid_strafe_latch_sec
                    ):
                        self._avoid_strafe_sign = desired_sign
                        self._avoid_strafe_sign_t = now
                strafe_speed = min(self.max_lin, max(self.min_lin, abs(self.avoid_strafe_speed)))
                vx = 0.0
                vy = self._avoid_strafe_sign * strafe_speed
                omega = 0.0
                if bx > 0.0 and self.avoid_strafe_forward_scale > 0.0:
                    v = min(self.max_lin, max(self.min_lin, self.kp_lin * dist))
                    vx = v * min(1.0, max(0.0, self.avoid_strafe_forward_scale))
            else:
                bearing = math.atan2(uy, ux)
                head_err = wrap_pi(bearing - rtheta)
                if abs(head_err) < self.travel_yaw_db:            # roughly aligned -> stop micro-rotating
                    omega = 0.0
                else:
                    omega = max(-self.max_ang, min(self.max_ang, self.kp_ang * head_err))
                if abs(head_err) < self.face_tol:
                    v = min(self.max_lin, self.kp_lin * dist)    # cruise; eases down near fine_radius
                    v = max(v, self.min_lin)                      # keep above stiction (else it stalls)
                    vx = v * math.cos(head_err)                   # ease off forward while still turning in
                    vy = 0.0
                else:
                    vy = 0.0
                    vx = 0.0                                      # rotate in place to face the goal first
        else:
            # ---- FINE (holonomic): within fine_radius, gentle creep toward the point incl. a small
            # strafe for lateral micro-alignment. cm-precision for a GRAB is ALIGN's body-cam servo,
            # not this (localizer pose is cm-noisy); this just lands the stand-off cleanly. ----
            bx = c * ex + s * ey
            by = -s * ex + c * ey
            mag = math.hypot(bx, by)
            if mag > 1e-9:
                vx = self.min_lin * bx / mag                     # fixed slow creep (base can't fine-taper)
                vy = self.min_lin * by / mag
            else:
                vx = vy = 0.0
            omega = max(-self.max_ang, min(self.max_ang, self.kp_ang * yaw_err))

        # translation speed cap (safety).
        speed = math.hypot(vx, vy)
        if speed > self.max_lin and speed > 1e-9:
            vx *= self.max_lin / speed
            vy *= self.max_lin / speed

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
