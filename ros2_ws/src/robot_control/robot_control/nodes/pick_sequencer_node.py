"""Pick sequencer: turns the FSM's /arm/pick_trigger (Bool) into a timed arm motion.

Closes the gap between the FSM and the arm controller:
    mission_fsm_node --/arm/pick_trigger (Bool)--> [THIS NODE]
    [THIS NODE] --/arm/target_pose (Pose, arm_base m)--> arm_controller_node
                --/arm/gripper      (Float32 servo deg)--> arm_controller_node
                --/arm/servo_correction (Vector3)--> arm_controller_node  (neutral here)

trigger True  -> run an open-loop look→descend→grasp→lift→tray→drop→return sequence.
trigger False -> release (open gripper at stow). The physical tray-dump mechanism is
                 separate (TODO); here we just open the gripper so anything held drops.

Grasp target: a fixed default pose (params, arm_base frame, meters). If `use_aruco` is set
and a fresh /aruco/marker_pose arrives, the grasp xyz is refined by transforming the marker
into arm_base via TF — guarded, and falling back to the default pose if TF is unavailable or
uncalibrated. This matches the planned look-then-grab→ArUco pickup; ArUco refinement stays
OFF until the camera→arm_base static TF is measured (the rig TFs are still placeholders).

Timing: the whole sequence should finish within the FSM's pick_duration_sec (default 3.0s)
or the FSM advances regardless. Tune the per-step dwells (sum) and/or the FSM param together.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from robot_control import arm_ik

# TF is optional: only needed for ArUco-refined grasping. Guarded so the node runs without it.
try:
    import tf2_ros
    from tf2_geometry_msgs import do_transform_pose_stamped  # noqa: F401

    _TF_AVAILABLE = True
except Exception:  # noqa: BLE001
    tf2_ros = None  # type: ignore[assignment]
    _TF_AVAILABLE = False


class PickSequencerNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_sequencer_node")

        # --- default poses (arm_base frame, meters) ---
        self.declare_parameter("grasp_xyz", [0.18, 0.0, 0.04])   # at the object
        self.declare_parameter("pregrasp_dz", 0.08)              # height above grasp for look/lift
        self.declare_parameter("tray_xyz", [0.05, 0.0, 0.20])    # body tray drop point
        self.declare_parameter("stow_xyz", [0.10, 0.0, 0.20])    # safe rest pose
        # --- step dwell times (s) ---
        self.declare_parameter("look_sec", 0.6)
        self.declare_parameter("move_sec", 0.5)
        self.declare_parameter("grasp_sec", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)
        # --- ArUco refinement (off until camera->arm_base TF is calibrated) ---
        self.declare_parameter("use_aruco", False)
        self.declare_parameter("aruco_topic", "/aruco/marker_pose")
        self.declare_parameter("arm_base_frame", "arm_base")
        self.declare_parameter("aruco_timeout_sec", 0.5)
        self.declare_parameter("grasp_z_offset_m", 0.0)          # added to ArUco z

        self.grasp = self._xyz("grasp_xyz")
        self.pregrasp_dz = float(self.get_parameter("pregrasp_dz").value)
        self.tray = self._xyz("tray_xyz")
        self.stow = self._xyz("stow_xyz")
        self.look_sec = float(self.get_parameter("look_sec").value)
        self.move_sec = float(self.get_parameter("move_sec").value)
        self.grasp_sec = float(self.get_parameter("grasp_sec").value)
        rate = float(self.get_parameter("control_rate_hz").value)
        self.use_aruco = bool(self.get_parameter("use_aruco").value)
        self.arm_base_frame = str(self.get_parameter("arm_base_frame").value)
        self.aruco_timeout = float(self.get_parameter("aruco_timeout_sec").value)
        self.grasp_z_offset = float(self.get_parameter("grasp_z_offset_m").value)

        # --- runtime sequencer state ---
        self.running = False
        self.kind = ""
        self.seq: list[tuple[str, float, tuple[float, float, float], float]] = []
        self.cum: list[float] = []
        self.total = 0.0
        self.seq_start_s = 0.0
        self._last_step = ""
        self._last_trigger: bool | None = None

        # latest ArUco marker pose
        self.marker: PoseStamped | None = None
        self.marker_stamp_s: float | None = None

        # TF buffer only if we actually intend to use ArUco
        self.tf_buffer = None
        if self.use_aruco and _TF_AVAILABLE:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        elif self.use_aruco and not _TF_AVAILABLE:
            self.get_logger().warn("use_aruco set but tf2 unavailable; using default grasp pose")
            self.use_aruco = False

        self.create_subscription(Bool, "/arm/pick_trigger", self.on_trigger, 10)
        if self.use_aruco:
            self.create_subscription(
                PoseStamped, str(self.get_parameter("aruco_topic").value), self.on_marker, 10
            )

        self.pub_pose = self.create_publisher(Pose, "/arm/target_pose", 10)
        self.pub_grip = self.create_publisher(Float32, "/arm/gripper", 10)
        self.pub_corr = self.create_publisher(Vector3, "/arm/servo_correction", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"pick_sequencer ready: grasp={self.grasp} tray={self.tray} stow={self.stow} "
            f"dwell(look={self.look_sec},move={self.move_sec},grasp={self.grasp_sec})s "
            f"use_aruco={self.use_aruco}"
        )

    # ------------------------------------------------------------------ utils
    def _xyz(self, name: str) -> tuple[float, float, float]:
        v = [float(x) for x in self.get_parameter(name).value]
        return (v[0], v[1], v[2])

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------- callbacks
    def on_trigger(self, msg: Bool) -> None:
        trig = bool(msg.data)
        # edge / latch: only (re)start when not already running the matching sequence
        if self.running:
            return
        if trig and self._last_trigger is not True:
            self._start_pick()
        elif not trig and self._last_trigger is not False:
            self._start_release()
        self._last_trigger = trig

    def on_marker(self, msg: PoseStamped) -> None:
        self.marker = msg
        self.marker_stamp_s = self._now_s()

    # -------------------------------------------------------------- sequences
    def _grasp_pose(self) -> tuple[float, float, float]:
        """Default grasp pose, optionally refined by a fresh ArUco marker via TF."""
        if not self.use_aruco or self.tf_buffer is None:
            return self.grasp
        if self.marker is None or self.marker_stamp_s is None:
            return self.grasp
        if (self._now_s() - self.marker_stamp_s) > self.aruco_timeout:
            return self.grasp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame, self.marker.header.frame_id, rclpy.time.Time()
            )
            p = do_transform_pose_stamped(self.marker, tf).pose.position
            self.get_logger().info(
                f"ArUco grasp refine -> ({p.x:.3f},{p.y:.3f},{p.z:.3f}) arm_base",
                throttle_duration_sec=1.0,
            )
            return (p.x, p.y, p.z + self.grasp_z_offset)
        except Exception as exc:  # noqa: BLE001 - TF missing/uncalibrated -> default
            self.get_logger().warn(
                f"ArUco TF transform failed ({exc}); using default grasp pose",
                throttle_duration_sec=2.0,
            )
            return self.grasp

    def _start_pick(self) -> None:
        grasp = self._grasp_pose()
        pre = (grasp[0], grasp[1], grasp[2] + self.pregrasp_dz)
        o, c = float(arm_ik.GRIPPER_OPEN), float(arm_ik.GRIPPER_CLOSED)
        self.seq = [
            ("LOOK",    self.look_sec,  pre,        o),
            ("DESCEND", self.move_sec,  grasp,      o),
            ("GRASP",   self.grasp_sec, grasp,      c),
            ("LIFT",    self.move_sec,  pre,        c),
            ("TO_TRAY", self.move_sec,  self.tray,  c),
            ("DROP",    self.grasp_sec, self.tray,  o),
            ("RETURN",  self.move_sec,  self.stow,  o),
        ]
        self._launch("PICK")

    def _start_release(self) -> None:
        self.seq = [("RELEASE", self.grasp_sec, self.stow, float(arm_ik.GRIPPER_OPEN))]
        self._launch("RELEASE")

    def _launch(self, kind: str) -> None:
        self.kind = kind
        self.seq_start_s = self._now_s()
        self.running = True
        self._last_step = ""
        t = 0.0
        self.cum = []
        for (_n, d, _p, _g) in self.seq:
            t += d
            self.cum.append(t)
        self.total = t
        self.get_logger().info(f"{kind} sequence start: {len(self.seq)} steps, {self.total:.1f}s")

    def _publish(self, xyz: tuple[float, float, float], grip: float) -> None:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        self.pub_pose.publish(pose)
        self.pub_grip.publish(Float32(data=float(grip)))
        self.pub_corr.publish(Vector3())   # neutral; visual-servo correction wired later

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        if not self.running:
            return
        el = self._now_s() - self.seq_start_s
        if el >= self.total:
            name, _, xyz, grip = self.seq[-1]
            self._publish(xyz, grip)           # hold final pose, then idle
            self.running = False
            self.get_logger().info(f"{self.kind} sequence complete")
            return

        idx = len(self.seq) - 1
        for i, cend in enumerate(self.cum):
            if el < cend:
                idx = i
                break
        name, _, xyz, grip = self.seq[idx]
        if name != self._last_step:
            self.get_logger().info(f"step {name}: pose={tuple(round(v,3) for v in xyz)} grip={grip}")
            self._last_step = name
        self._publish(xyz, grip)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickSequencerNode()
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
