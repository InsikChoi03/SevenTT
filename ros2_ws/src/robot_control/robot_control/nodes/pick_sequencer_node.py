"""Pick sequencer (2R arm): turns the FSM's /arm/pick_trigger (Bool) into a timed 2R servo
sequence, published as /arm2r/target = [shoulder, wrist, gripper] (degrees). The combined-board
bridge (mcu_bridge_base) forwards it as <ARM,shoulder,wrist,gripper> on ttyUSB0.

Transplanted from the VERIFIED scripts/arm_pick2r.py (fixed-blind 2R grasp; pick confirmed by
pick_verify). Replaces the old arm_ik / 6-DOF path (arm_controller_node + mcu_bridge_arm), which
does not match the physical 2R arm.

    trigger True  -> GRASP: reach to PICK pose (open) -> close -> lift back to INIT (holding).
    trigger False -> PLACE: move to PLACE pose (holding) -> open -> return to INIT.

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

        # 2R verified poses (re-taught 2026-07-04 after the arm angles shifted). Servo degrees.
        self.declare_parameter("init_pose", [170.0, 10.0, 100.0])   # shoulder, wrist, gripper (idle)
        self.declare_parameter("pick_shoulder_wrist", [95.0, 150.0])
        self.declare_parameter("place_shoulder_wrist", [170.0, 30.0])
        self.declare_parameter("grip_open", 85.0)
        self.declare_parameter("grip_closed", 130.0)
        self.declare_parameter("move_sec", 2.5)     # dwell for an arm move (>= firmware smooth time)
        self.declare_parameter("grasp_sec", 0.7)    # dwell for a gripper open/close
        self.declare_parameter("rate_hz", 20.0)

        self.init_pose = [float(v) for v in self.get_parameter("init_pose").value]
        psw = [float(v) for v in self.get_parameter("pick_shoulder_wrist").value]
        plsw = [float(v) for v in self.get_parameter("place_shoulder_wrist").value]
        self.pick_sw = (psw[0], psw[1])
        self.place_sw = (plsw[0], plsw[1])
        self.grip_open = float(self.get_parameter("grip_open").value)
        self.grip_closed = float(self.get_parameter("grip_closed").value)
        self.move_sec = float(self.get_parameter("move_sec").value)
        self.grasp_sec = float(self.get_parameter("grasp_sec").value)
        rate = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(Float32MultiArray, "/arm2r/target", 10)
        self.create_subscription(Bool, "/arm/pick_trigger", self.on_trigger, 10)

        # sequence state
        self.seq: list[tuple[str, float, tuple[float, float, float]]] = []
        self.cum: list[float] = []
        self.total = 0.0
        self.seq_start_s = 0.0
        self.running = False
        self.kind = ""
        self._last_step = ""
        self._last_trigger: bool | None = None

        # Absorb the boot-limp snap: hold INIT for the first ~1.5 s before accepting triggers.
        self._init_until = self._now_s() + 1.5
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"pick_sequencer(2R) ready: init={self.init_pose} pick={self.pick_sw} "
            f"place={self.place_sw} grip(open={self.grip_open}/closed={self.grip_closed}) "
            f"move={self.move_sec}s grasp={self.grasp_sec}s -> /arm2r/target"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, pose: tuple[float, float, float]) -> None:
        m = Float32MultiArray()
        m.data = [float(pose[0]), float(pose[1]), float(pose[2])]
        self.pub.publish(m)

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
        # FULL pick+place on one True trigger: the 2R has a single gripper, so it can't hold an
        # object while chasing the next target — grab it and immediately place it (behind), then
        # return to idle ready for the next target. (release-only stays available via False.)
        psh, pwr = self.pick_sw
        plsh, plwr = self.place_sw
        ish, iwr, _ig = self.init_pose
        self.seq = [
            ("REACH",    self.move_sec,  (psh, pwr, self.grip_open)),      # down to pick pose, open
            ("GRASP",    self.grasp_sec, (psh, pwr, self.grip_closed)),    # close (grab)
            ("LIFT",     self.move_sec,  (ish, iwr, self.grip_closed)),    # lift to init, holding
            ("TO_PLACE", self.move_sec,  (plsh, plwr, self.grip_closed)),  # move to place pose
            ("PLACE",    self.grasp_sec, (plsh, plwr, self.grip_open)),    # open (place behind)
            ("RETURN",   self.move_sec,  tuple(self.init_pose)),           # back to idle
        ]
        self._launch("PICK_PLACE")

    def _start_release(self) -> None:
        sh, wr = self.place_sw
        self.seq = [
            ("TO_PLACE", self.move_sec,  (sh, wr, self.grip_closed)),  # to place pose, holding
            ("PLACE",    self.grasp_sec, (sh, wr, self.grip_open)),    # open (release)
            ("RETURN",   self.move_sec,  tuple(self.init_pose)),       # back to init
        ]
        self._launch("RELEASE")

    def _launch(self, kind: str) -> None:
        self.kind = kind
        self.seq_start_s = self._now_s()
        self.running = True
        self._last_step = ""
        t = 0.0
        self.cum = []
        for (_n, d, _p) in self.seq:
            t += d
            self.cum.append(t)
        self.total = t
        self.get_logger().info(f"{kind} sequence start: {len(self.seq)} steps, {self.total:.1f}s")

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        if not self.running:
            if self._now_s() < self._init_until:
                self._publish(tuple(self.init_pose))   # snap-absorb: hold INIT on boot
            return
        el = self._now_s() - self.seq_start_s
        if el >= self.total:
            self._publish(self.seq[-1][2])
            self.running = False
            self.get_logger().info(f"{self.kind} sequence complete")
            return
        idx = len(self.seq) - 1
        for i, cend in enumerate(self.cum):
            if el < cend:
                idx = i
                break
        name, _, pose = self.seq[idx]
        if name != self._last_step:
            self.get_logger().info(f"step {name}: pose={tuple(round(v) for v in pose)}")
            self._last_step = name
        self._publish(pose)


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
