"""Pick sequencer (2R arm): turns the FSM's /arm/pick_trigger (Bool) into a timed 2R servo
sequence, published as /arm2r/target = [shoulder, wrist, gripper] (degrees). The combined-board
bridge (mcu_bridge_base) forwards it as <ARM,shoulder,wrist,gripper> on ttyUSB0.

Transplanted from the VERIFIED scripts/arm_pick2r.py (fixed-blind 2R grasp; pick confirmed by
pick_verify). Replaces the old arm_ik / 6-DOF path (arm_controller_node + mcu_bridge_arm), which
does not match the physical 2R arm. Each step is host-ramped so the main field launch shows the
same stage order and does not drop the gripper too quickly.

    trigger True  -> GRASP: reach to PICK pose (open) -> close -> lift while holding
                     -> move to PLACE while holding -> open at PLACE -> stow closed.
    trigger False -> RELEASE: move to PLACE while holding -> open at PLACE -> stow closed.

The arm boots LIMP and snaps to the first <ARM> it receives, so INIT is published on startup to
absorb that snap safely. Angles are the arm_pick2r verified values (params to tune).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class PickSequencerNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_sequencer_node")

        # 2R verified poses (re-taught 2026-07-08). Servo degrees.
        self.declare_parameter("init_pose", [110.0, 30.0, 50.0])    # shoulder, wrist, gripper (stowed)
        self.declare_parameter("pick_shoulder_wrist", [25.0, 150.0])
        self.declare_parameter("place_shoulder_wrist", [110.0, 30.0])
        self.declare_parameter("grip_open", 115.0)
        self.declare_parameter("grip_closed", 50.0)
        self.declare_parameter("move_sec", 2.5)     # dwell for an arm move (>= firmware smooth time)
        self.declare_parameter("grasp_sec", 0.7)    # dwell for a gripper open/close
        self.declare_parameter("reach_grip_open_frac", 0.25)
        self.declare_parameter("rate_hz", 20.0)

        self.init_pose = [float(v) for v in self.get_parameter("init_pose").value]
        psw = [float(v) for v in self.get_parameter("pick_shoulder_wrist").value]
        plsw = [float(v) for v in self.get_parameter("place_shoulder_wrist").value]
        self.pick_sw = (psw[0], psw[1])
        self.place_sw = (plsw[0], plsw[1])
        self.grip_open = float(self.get_parameter("grip_open").value)
        self.grip_closed = float(self.get_parameter("grip_closed").value)
        self.stow_pose = (self.place_sw[0], self.place_sw[1], self.grip_closed)
        self.move_sec = float(self.get_parameter("move_sec").value)
        self.grasp_sec = float(self.get_parameter("grasp_sec").value)
        self.reach_grip_open_frac = float(self.get_parameter("reach_grip_open_frac").value)
        rate = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(Float32MultiArray, "/arm2r/target", 10)
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
            f"move={self.move_sec}s grasp={self.grasp_sec}s "
            f"reach_open_frac={self.reach_grip_open_frac} -> /arm2r/target"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, pose: tuple[float, float, float]) -> None:
        self.current_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        m = Float32MultiArray()
        m.data = [self.current_pose[0], self.current_pose[1], self.current_pose[2]]
        self.pub.publish(m)

    @staticmethod
    def _lerp_pose(
        start: tuple[float, float, float],
        target: tuple[float, float, float],
        progress: float,
    ) -> tuple[float, float, float]:
        u = max(0.0, min(1.0, progress))
        return (
            start[0] + (target[0] - start[0]) * u,
            start[1] + (target[1] - start[1]) * u,
            start[2] + (target[2] - start[2]) * u,
        )

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
            ("REACH",    self.move_sec,  (psh, pwr, self.grip_open)),      # down to pick pose, open
            ("GRASP",    self.grasp_sec, (psh, pwr, self.grip_closed)),    # close (grab)
            ("LIFT",     self.move_sec,  (ish, iwr, self.grip_closed)),    # lift to init, holding
            ("TO_PLACE", self.move_sec,  (plsh, plwr, self.grip_closed)),  # move to place, holding
            ("PLACE",    self.grasp_sec, (plsh, plwr, self.grip_open)),    # open at place pose
            ("STOW",     self.grasp_sec, (plsh, plwr, self.grip_closed)),  # drive with gripper closed
        ]
        self._launch("PICK_PLACE")

    def _start_release(self) -> None:
        sh, wr = self.place_sw
        self.seq = [
            ("TO_PLACE", self.move_sec,  (sh, wr, self.grip_closed)),  # move to place, holding
            ("RELEASE",  self.grasp_sec, (sh, wr, self.grip_open)),    # open at place pose
            ("STOW",     self.grasp_sec, (sh, wr, self.grip_closed)),  # drive with gripper closed
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
        for (_n, d, p) in self.seq:
            self.seq_start_poses.append(step_start_pose)
            t += d
            self.cum.append(t)
            step_start_pose = p
        self.total = t
        self.get_logger().info(f"{kind} sequence start: {len(self.seq)} steps, {self.total:.1f}s")

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        if not self.running:
            if self._now_s() < self._init_until:
                self._publish(tuple(self.init_pose))   # snap-absorb: hold INIT on boot
            else:
                self._publish(self.stow_pose)          # match driving: arm up at tray side, gripper closed
            return
        el = self._now_s() - self.seq_start_s
        if el >= self.total:
            self._publish(self.seq[-1][2])
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
        step_start_t = 0.0 if idx == 0 else self.cum[idx - 1]
        progress = (el - step_start_t) / max(duration, 1e-6)
        ramped_pose = self._lerp_pose(self.seq_start_poses[idx], pose, progress)
        if name == "REACH":
            start = self.seq_start_poses[idx]
            frac = max(0.05, min(1.0, self.reach_grip_open_frac))
            grip_progress = min(1.0, max(0.0, progress / frac))
            ramped_pose = (
                ramped_pose[0],
                ramped_pose[1],
                start[2] + (pose[2] - start[2]) * grip_progress,
            )
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
