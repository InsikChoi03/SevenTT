"""Mecanum base inverse kinematics: (vx, vy, omega) -> 4 wheel speeds.

Wheel convention (top view, X forward / Y left):
    FL --- FR
     |     |
    RL --- RR
Each wheel signed speed is the linear speed at wheel tread (m/s);
the bridge node converts to PWM via wheel radius + max RPM calibration.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, MissionState
from std_msgs.msg import Float32MultiArray


class BaseControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("base_controller_node")

        # Wheelbase half-width (lx) and half-track (ly), meters
        self.declare_parameter("lx", 0.10)
        self.declare_parameter("ly", 0.10)
        # Watchdog: zero wheels if no command in window
        self.declare_parameter("cmd_timeout_sec", 0.5)
        # --- Serial-tuned wheel behaviour (from scripts/drive_tests/motion_tune.py) ---
        # Per-wheel FL,FR,RL,RR trim for yaw/imbalance (1.0 = none; forward tuned to all-1).
        self.declare_parameter("wheel_scales", [1.0, 1.0, 1.0, 1.0])
        # DIRECTION-DEPENDENT strafe trim: a mecanum base does NOT strafe straight with the forward
        # trim — each wheel loads differently sideways. drive_tests/motion_tune tuned separate scales
        # per strafe direction so it tracks straight (and hits distance). Forward uses wheel_scales;
        # strafing blends toward these by the |vx|:|vy| ratio (diagonals interpolate). REP-103 y-left,
        # so vy<0 = rightward strafe -> right scales; vy>0 = leftward -> left scales.
        self.declare_parameter("strafe_right_scales", [0.70, 0.70, 0.80, 0.70])
        self.declare_parameter("strafe_left_scales", [0.65, 0.75, 0.75, 0.65])
        # STICTION: the heavy base only moves above ~this <BASE> magnitude (firmware normalises by
        # MAX_SPEED 0.6, so 0.42 ~ 0.7 duty). The velocity pipeline commands 0.05-0.20 which just
        # buzzed — so any real move is floored to this (direction preserved). 0 disables.
        self.declare_parameter("wheel_min", 0.42)
        # Separate stiction floor for PURE IN-PLACE ROTATION (|vx|,|vy|~0). In-place turns otherwise
        # get floored to wheel_min and spin too fast; set this LOWER for a slow-but-moving CW/CCW turn
        # (the start-boost still breaks static friction, then it relaxes to this). Raise if it stalls.
        self.declare_parameter("wheel_min_rot", 0.07)
        # STATIC friction from REST is higher, so kick to this for wheel_boost_ms on start (the
        # motion_tune boost 0.65 / 100-120 ms). Then relax to wheel_min.
        self.declare_parameter("wheel_boost", 0.60)
        self.declare_parameter("wheel_boost_ms", 120)
        self.declare_parameter("wheel_deadband", 0.02)   # below this = treat as stop
        # BRAKE: mirror of the start boost. The heavy base coasts past target after a command stops,
        # so on EVERY move->stop we emit a brief reverse pulse (opposite the last travel direction,
        # scaled) to kill that coast — the same brake the serial motion tuner applies per segment.
        # This also fires when the ALIGN visual-servo settles between nudges: that's benign because
        # ALIGN re-measures the settled position each cycle (closed loop) and the pulse only trims
        # coast, but if ALIGN regresses lower wheel_brake_scale or set wheel_brake_ms:0. 0 disables.
        self.declare_parameter("wheel_brake_ms", 120)
        self.declare_parameter("wheel_brake_scale", 0.315)

        self.lx = float(self.get_parameter("lx").value)
        self.ly = float(self.get_parameter("ly").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        sc = [float(v) for v in self.get_parameter("wheel_scales").value]
        self.wheel_scales = sc if len(sc) == 4 else [1.0, 1.0, 1.0, 1.0]
        sr = [float(v) for v in self.get_parameter("strafe_right_scales").value]
        self.strafe_right = sr if len(sr) == 4 else [0.70, 0.70, 0.80, 0.70]
        sl = [float(v) for v in self.get_parameter("strafe_left_scales").value]
        self.strafe_left = sl if len(sl) == 4 else [0.65, 0.75, 0.75, 0.65]
        self.wheel_min = float(self.get_parameter("wheel_min").value)
        self.wheel_min_rot = float(self.get_parameter("wheel_min_rot").value)
        self.wheel_boost = float(self.get_parameter("wheel_boost").value)
        self.wheel_boost_ms = float(self.get_parameter("wheel_boost_ms").value)
        self.wheel_deadband = float(self.get_parameter("wheel_deadband").value)
        self.wheel_brake_ms = float(self.get_parameter("wheel_brake_ms").value)
        self.wheel_brake_scale = float(self.get_parameter("wheel_brake_scale").value)
        # ALIGN fine-positioning is GENTLER: the full start-boost (0.65) + stop-brake make each tiny
        # nudge jerk ("휙휙"). During ALIGN use a softer kick and NO brake so the base creeps.
        self.declare_parameter("align_wheel_boost", 0.45)
        self.declare_parameter("align_brake_off", True)
        self.align_wheel_boost = float(self.get_parameter("align_wheel_boost").value)
        self.align_brake_off = bool(self.get_parameter("align_brake_off").value)
        # SLEW-RATE limit: cap how much each wheel output can change per 20 ms tick, so the base ramps
        # up/down smoothly instead of jack-rabbiting (급발진) and slipping — slip is what accumulates
        # odometry error and makes it wander later. 0 disables. 0.04/tick @50Hz = 2.0/s.
        self.declare_parameter("wheel_slew_per_tick", 0.04)
        self.wheel_slew = float(self.get_parameter("wheel_slew_per_tick").value)
        self._prev_out = [0.0, 0.0, 0.0, 0.0]
        self._mstate = ""
        self.create_subscription(MissionState, "/mission_state", self._on_mstate, 10)
        self._moving = False
        self._boost_until = 0.0
        self._brake_until = 0.0
        self._brake_cmd = [0.0, 0.0, 0.0, 0.0]
        self._last_move_wheels = [0.0, 0.0, 0.0, 0.0]

        self.last_cmd_time = self.get_clock().now()
        self.last_cmd = (0.0, 0.0, 0.0)

        self.sub = self.create_subscription(BaseCommand, "/base_command", self.on_cmd, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/base/wheel_speeds", 10)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        self.get_logger().info(f"base IK lx={self.lx} ly={self.ly} watchdog={self.cmd_timeout}s")

    def on_cmd(self, msg: BaseCommand) -> None:
        self.last_cmd = (msg.vx, msg.vy, msg.omega)
        self.last_cmd_time = self.get_clock().now()

    def _on_mstate(self, msg: MissionState) -> None:
        self._mstate = str(msg.state)

    def tick(self) -> None:
        # Watchdog: zero output if no recent command
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            vx, vy, omega = 0.0, 0.0, 0.0
        else:
            vx, vy, omega = self.last_cmd

        # Mecanum inverse kinematics (wheel linear speeds, m/s)
        k = self.lx + self.ly
        v_fl = vx - vy - k * omega
        v_fr = vx + vy + k * omega
        v_rl = vx + vy - k * omega
        v_rr = vx - vy + k * omega

        # Direction-dependent per-wheel trim so BOTH forward and strafe track straight: forward uses
        # wheel_scales, strafing uses the tuned per-direction scales, diagonals blend by |vx|:|vy|.
        fwd = self.wheel_scales
        strafe = self.strafe_right if vy < 0.0 else self.strafe_left
        awx, awy = abs(vx), abs(vy)
        tot = awx + awy
        if tot < 1e-6:
            sc = fwd                                    # pure rotation / stop -> forward trim
        else:
            sc = [(awx * fwd[i] + awy * strafe[i]) / tot for i in range(4)]
        wheels = [v_fl * sc[0], v_fr * sc[1], v_rl * sc[2], v_rr * sc[3]]
        wheels = [max(-1.0, min(1.0, w)) for w in wheels]

        # Stiction floor + start-from-rest boost + stop brake. The whole vector is scaled so the
        # fastest wheel reaches the moving threshold (direction preserved). On move->stop we emit a
        # brief reverse pulse to kill the heavy base's coast; below the deadband otherwise = full stop.
        m = max(abs(w) for w in wheels)
        now = self.get_clock().now().nanoseconds * 1e-9
        aligning = self._mstate == "ALIGN"
        kick = False                                  # boost/brake pulse this tick -> bypass slew
        if m <= self.wheel_deadband:
            if self._moving:                          # transition move -> rest: start the brake pulse
                self._moving = False
                brake = self.wheel_brake_ms > 0.0 and self.wheel_brake_scale > 0.0
                if aligning and self.align_brake_off:
                    brake = False                     # no coast-brake during fine ALIGN (it jerks)
                if brake:
                    self._brake_until = now + self.wheel_brake_ms / 1000.0
                    self._brake_cmd = [max(-1.0, min(1.0, -w * self.wheel_brake_scale))
                                       for w in self._last_move_wheels]
            if now < self._brake_until:
                wheels = list(self._brake_cmd)        # reverse pulse decelerates the coast
                kick = True                           # brake pulse must hit hard -> bypass slew
            else:
                wheels = [0.0, 0.0, 0.0, 0.0]
        else:
            self._moving = True
            self._brake_until = 0.0                   # a fresh move cancels any pending brake
            if aligning:
                kick = True                           # ALIGN unit steps bypass slew -> hit full duty
                                                      # immediately, matching the boost-free calibration
            # NO start-boost (removed per tuning: it jerked and made each pulse's distance unrepeatable).
            # The steady floor is the min duty that keeps the base moving; cruise must sit above the
            # static breakaway to start from rest. Pure in-place rotation gets its own (slower) floor.
            is_rot = abs(vx) < 0.02 and abs(vy) < 0.02 and abs(omega) > 1e-3
            steady_floor = self.wheel_min_rot if is_rot else self.wheel_min
            if 0.0 < m < steady_floor:
                s = steady_floor / m
                wheels = [max(-1.0, min(1.0, w * s)) for w in wheels]
            self._last_move_wheels = list(wheels)     # remember travel direction for the stop brake

        # Slew-limit toward the target so accel/decel is smooth (no jack-rabbit start / no slip).
        # Brake pulses bypass slew (kick) so they hit hard enough to actually cut the coast.
        if self.wheel_slew > 0.0 and not kick:
            lim = []
            for w, pv in zip(wheels, self._prev_out):
                dw = w - pv
                if dw > self.wheel_slew:
                    dw = self.wheel_slew
                elif dw < -self.wheel_slew:
                    dw = -self.wheel_slew
                lim.append(pv + dw)
            wheels = lim
        self._prev_out = list(wheels)

        out = Float32MultiArray()
        out.data = [float(w) for w in wheels]
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseControllerNode()
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
