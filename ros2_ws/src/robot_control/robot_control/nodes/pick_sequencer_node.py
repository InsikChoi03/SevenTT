"""Pick sequencer (2R arm): turns the FSM's /arm/pick_trigger (Bool) into a timed 2R servo
sequence, published as /arm2r/target = [shoulder, wrist, gripper] (degrees). The combined-board
bridge (mcu_bridge_base) forwards it as <ARM,shoulder,wrist,gripper> on ttyUSB0.

Transplanted from the VERIFIED scripts/arm_pick2r.py (fixed-blind 2R grasp; pick confirmed by
pick_verify). Replaces the old arm_ik / 6-DOF path (arm_controller_node + mcu_bridge_arm), which
does not match the physical 2R arm. Arm moves are host-ramped no faster than the firmware can
follow. The pick pose is then held with the gripper open before the close command is allowed.

    trigger True  -> GRASP: open gripper at stow -> reach to PICK pose (open) -> settle (open)
                     -> close -> lift while holding -> move to PLACE while holding
                     -> open at PLACE -> stow closed.
    trigger False -> RELEASE: move to PLACE while holding -> open at PLACE -> stow closed.

The arm boots LIMP and snaps to the first <ARM> it receives, so INIT is published on startup to
absorb that snap safely. Angles are the arm_pick2r verified values (params to tune).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


def required_joint_move_sec(
    start: tuple[float, float, float],
    target: tuple[float, float, float],
    speed_dps: tuple[float, float, float],
) -> float:
    """Return the minimum command-ramp time for per-channel firmware limits."""
    if len(speed_dps) != 3 or any(speed <= 0.0 for speed in speed_dps):
        raise ValueError("servo_speed_dps_by_channel must contain three positive values")
    return max(abs(target[i] - start[i]) / speed_dps[i] for i in range(3))


def step_command_pose(
    name: str,
    start: tuple[float, float, float],
    target: tuple[float, float, float],
    progress: float,
) -> tuple[float, float, float]:
    """Return an immediate GRASP target or a normal host-ramped move target."""
    if name == "GRASP":
        return target
    u = max(0.0, min(1.0, progress))
    return tuple(start[i] + (target[i] - start[i]) * u for i in range(3))


class PickSequencerNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_sequencer_node")

        # 2R verified poses (re-taught 2026-07-08). Servo degrees.
        self.declare_parameter("init_pose", [120.0, 5.0, 100.0])
        self.declare_parameter("pick_shoulder_wrist", [25.0, 150.0])
        self.declare_parameter("place_shoulder_wrist", [110.0, 30.0])
        self.declare_parameter("grip_open", 118.0)
        self.declare_parameter("grip_closed", 40.0)
        self.declare_parameter("move_sec", 1.3)  # accepted for legacy parameter files
        self.declare_parameter("reach_lift_sec", 1.3)
        self.declare_parameter("to_place_sec", 1.25)
        self.declare_parameter("stow_sec", 1.0)
        self.declare_parameter("grasp_sec", 0.75)
        self.declare_parameter("preopen_sec", 0.20)
        self.declare_parameter("pick_settle_sec", 0.0)
        self.declare_parameter("servo_speed_dps", 60.0)  # legacy scalar; kept for old configs
        self.declare_parameter("servo_speed_dps_by_channel", [80.0, 120.0, 120.0])
        self.declare_parameter("rate_hz", 20.0)

        self.init_pose = [float(v) for v in self.get_parameter("init_pose").value]
        psw = [float(v) for v in self.get_parameter("pick_shoulder_wrist").value]
        plsw = [float(v) for v in self.get_parameter("place_shoulder_wrist").value]
        self.pick_sw = (psw[0], psw[1])
        self.place_sw = (plsw[0], plsw[1])
        self.grip_open = float(self.get_parameter("grip_open").value)
        self.grip_closed = float(self.get_parameter("grip_closed").value)
        self.stow_pose = tuple(self.init_pose)
        self.reach_lift_sec = float(self.get_parameter("reach_lift_sec").value)
        self.to_place_sec = float(self.get_parameter("to_place_sec").value)
        self.stow_sec = float(self.get_parameter("stow_sec").value)
        self.grasp_sec = float(self.get_parameter("grasp_sec").value)
        self.preopen_sec = float(self.get_parameter("preopen_sec").value)
        self.pick_settle_sec = float(self.get_parameter("pick_settle_sec").value)
        speeds = [float(v) for v in self.get_parameter("servo_speed_dps_by_channel").value]
        if len(speeds) != 3 or any(speed <= 0.0 for speed in speeds):
            raise ValueError("servo_speed_dps_by_channel must contain three positive values")
        self.servo_speed_dps_by_channel = (speeds[0], speeds[1], speeds[2])
        rate = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(Float32MultiArray, "/arm2r/target", 10)
        # The mission uses this real sequencer phase instead of a blind fixed wait.  Publishing
        # continuously also lets a restarted mission learn whether the arm is currently busy.
        self.pub_phase = self.create_publisher(String, "/arm/pick_sequence_phase", 10)
        self.create_subscription(Bool, "/arm/pick_trigger", self.on_trigger, 10)

        # sequence state
        self.seq: list[tuple[str, float, tuple[float, float, float]]] = []
        self.seq_start_poses: list[tuple[float, float, float]] = []
        self.cum: list[float] = []
        self.total = 0.0
        self.seq_start_s = 0.0
        self.running = False
        self.kind = ""
        self._last_step = ""
        self._last_trigger: bool | None = None
        self.current_pose = self.stow_pose

        # Absorb the boot-limp snap: hold INIT for the first ~1.5 s before accepting triggers.
        self._init_until = self._now_s() + 1.5
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"pick_sequencer(2R) ready: init={self.init_pose} pick={self.pick_sw} "
            f"place={self.place_sw} grip(open={self.grip_open}/closed={self.grip_closed}) "
            f"reach/lift={self.reach_lift_sec}s to_place={self.to_place_sec}s "
            f"stow={self.stow_sec}s grasp={self.grasp_sec}s preopen={self.preopen_sec}s "
            f"pick_settle={self.pick_settle_sec}s "
            f"servo_speed_by_channel={self.servo_speed_dps_by_channel}deg/s "
            "-> /arm2r/target"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, pose: tuple[float, float, float]) -> None:
        self.current_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        m = Float32MultiArray()
        m.data = [self.current_pose[0], self.current_pose[1], self.current_pose[2]]
        self.pub.publish(m)

    def _publish_phase(self, phase: str) -> None:
        self.pub_phase.publish(String(data=str(phase)))

    # -------------------------------------------------------------- callbacks
    def on_trigger(self, msg: Bool) -> None:
        trig = bool(msg.data)
        if self.running or self._now_s() < self._init_until:
            return
        if trig and self._last_trigger is not True:
            self._start_pick()
        elif not trig and self._last_trigger is not False:
            self._start_release()
        self._last_trigger = trig

    # -------------------------------------------------------------- sequences
    def _start_pick(self) -> None:
        # Full pick on one True trigger. Keep the gripper closed while lifting and while moving to
        # the place pose; open only after the place pose is reached.
        psh, pwr = self.pick_sw
        ish, iwr, _ig = self.init_pose
        plsh, plwr = self.place_sw
        self.seq = [
            ("PREOPEN",  self.preopen_sec, (ish, iwr, self.grip_open)),      # open before lowering
            ("REACH",    self.reach_lift_sec, (psh, pwr, self.grip_open)),
        ]
        if self.pick_settle_sec > 0.0:
            self.seq.append(
                ("PICK_SETTLE", self.pick_settle_sec, (psh, pwr, self.grip_open))
            )
        self.seq.extend([
            ("GRASP",    self.grasp_sec, (psh, pwr, self.grip_closed)),    # close (grab)
            ("LIFT",     self.reach_lift_sec, (ish, iwr, self.grip_closed)),
            ("TO_PLACE", self.to_place_sec, (plsh, plwr, self.grip_closed)),
            ("PLACE",    self.grasp_sec, (plsh, plwr, self.grip_open)),    # open at place pose
            ("STOW",     self.stow_sec, self.stow_pose),                   # return to driving pose
        ])
        self._launch("PICK_PLACE")

    def _start_release(self) -> None:
        sh, wr = self.place_sw
        self.seq = [
            ("TO_PLACE", self.to_place_sec, (sh, wr, self.grip_closed)),
            ("RELEASE",  self.grasp_sec, (sh, wr, self.grip_open)),    # open at place pose
            ("STOW",     self.stow_sec, self.stow_pose),               # return to driving pose
        ]
        self._launch("RELEASE")

    def _launch(self, kind: str) -> None:
        self.kind = kind
        self.seq_start_s = self._now_s()
        self.running = True
        self._last_step = ""
        t = 0.0
        self.cum = []
        self.seq_start_poses = []
        step_start_pose = self.current_pose
        timed_seq: list[tuple[str, float, tuple[float, float, float]]] = []
        arm_move_steps = {"REACH", "LIFT", "TO_PLACE", "STOW"}
        for (name, configured_duration, pose) in self.seq:
            self.seq_start_poses.append(step_start_pose)
            duration = configured_duration
            if name in arm_move_steps:
                # The MCU moves each servo by at most MAX_STEP every update. If a YAML override
                # requests a ramp shorter than that physical travel time, the MCU remains behind
                # the host target and GRASP can start while the wrist is still descending.
                required_duration = required_joint_move_sec(
                    step_start_pose,
                    pose,
                    self.servo_speed_dps_by_channel,
                )
                duration = max(configured_duration, required_duration)
            timed_seq.append((name, duration, pose))
            t += duration
            self.cum.append(t)
            step_start_pose = pose
        self.seq = timed_seq
        self.total = t
        self.get_logger().info(f"{kind} sequence start: {len(self.seq)} steps, {self.total:.1f}s")

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        if not self.running:
            self._publish(self.stow_pose)              # hold init_pose throughout match driving
            self._publish_phase("IDLE")
            return
        el = self._now_s() - self.seq_start_s
        if el >= self.total:
            self._publish(self.seq[-1][2])
            self._publish_phase("COMPLETE")
            self.running = False
            # RE-ARM the edge detector: the FSM sends a fresh True for EVERY pick, but on_trigger only
            # fires on a rising edge. Without this reset _last_trigger stays True and the 2nd, 3rd...
            # pick never runs. None re-arms both the pick(True) and release(False) edges.
            self._last_trigger = None
            self.get_logger().info(f"{self.kind} sequence complete")
            return
        idx = len(self.seq) - 1
        for i, cend in enumerate(self.cum):
            if el < cend:
                idx = i
                break
        name, duration, pose = self.seq[idx]
        self._publish_phase(name)
        step_start_t = 0.0 if idx == 0 else self.cum[idx - 1]
        progress = (el - step_start_t) / max(duration, 1e-6)
        # GRASP issues the final closed target immediately; other motion remains host-ramped.
        ramped_pose = step_command_pose(name, self.seq_start_poses[idx], pose, progress)
        if name != self._last_step:
            start = self.seq_start_poses[idx]
            self.get_logger().info(
                f"step {name}: from={tuple(round(v) for v in start)} "
                f"to={tuple(round(v) for v in pose)} dur={duration:.1f}s"
            )
            self._last_step = name
        self._publish(ramped_pose)


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
