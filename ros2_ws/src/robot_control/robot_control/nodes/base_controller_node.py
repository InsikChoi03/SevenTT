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
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, MissionState
from std_msgs.msg import Bool, Float32MultiArray


def below_motion_deadband(
    wheel_magnitude: float,
    wheel_deadband: float,
    *,
    pure_rotation: bool,
) -> bool:
    """Keep an intentional low-speed rotation alive for the rotation stiction floor."""
    return wheel_magnitude <= wheel_deadband and not pure_rotation


def _four_scales(values, fallback: list[float]) -> list[float]:
    vals = [float(v) for v in values]
    return vals if len(vals) == 4 else list(fallback)


def select_wheel_scales(
    vx: float,
    vy: float,
    omega: float,
    wheel_scales: list[float],
    strafe_right_scales: list[float],
    strafe_left_scales: list[float],
    rotation_cw_scales: list[float],
    rotation_ccw_scales: list[float],
) -> list[float]:
    pure_rotation = abs(vx) < 0.02 and abs(vy) < 0.02 and abs(omega) > 1e-3
    if pure_rotation:
        return list(rotation_cw_scales if omega < 0.0 else rotation_ccw_scales)

    awx, awy = abs(vx), abs(vy)
    tot = awx + awy
    if tot < 1e-6:
        return list(wheel_scales)

    strafe = strafe_right_scales if vy < 0.0 else strafe_left_scales
    return [(awx * wheel_scales[i] + awy * strafe[i]) / tot for i in range(4)]


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
        # Pure in-place rotation trim.  Keep this separate from wheel_scales so a weak wheel can be
        # helped during scan turns without changing forward/strafe behavior.
        self.declare_parameter("rotation_cw_scales", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("rotation_ccw_scales", [1.0, 1.0, 1.0, 1.0])
        # STICTION: the heavy base only moves above ~this <BASE> magnitude (firmware normalises by
        # MAX_SPEED 0.6, so 0.42 ~ 0.7 duty). The velocity pipeline commands 0.05-0.20 which just
        # buzzed — so any real move is floored to this (direction preserved). 0 disables.
        self.declare_parameter("wheel_min", 0.42)
        self.declare_parameter("wheel_min_strafe", 0.42)
        # Separate stiction floor for PURE IN-PLACE ROTATION (|vx|,|vy|~0). In-place turns otherwise
        # get floored to wheel_min and spin too fast; set this LOWER for a slow-but-moving CW/CCW turn
        # (the start-boost still breaks static friction, then it relaxes to this). Raise if it stalls.
        self.declare_parameter("wheel_min_rot", 0.07)
        # STATIC friction from REST is higher, so strafe/rotation starts get a short breakaway kick,
        # then relax to wheel_min/wheel_min_rot. Forward starts stay unboosted so straight-line tuning
        # is not disturbed. `wheel_boost` is kept as a legacy fallback for older YAML files.
        self.declare_parameter("wheel_boost", 0.60)
        legacy_boost = float(self.get_parameter("wheel_boost").value)
        self.declare_parameter("wheel_boost_strafe", legacy_boost)
        self.declare_parameter("wheel_boost_rot", legacy_boost)
        self.declare_parameter("wheel_boost_ms", 120)
        # OPENING has its own tuning so match-start motion can be made assertive without changing
        # normal SCAN/APPROACH/ALIGN behavior.
        self.declare_parameter("opening_wheel_min", 0.0)
        self.declare_parameter("opening_wheel_min_strafe", 0.0)
        self.declare_parameter("opening_wheel_min_rot", 0.0)
        self.declare_parameter("opening_wheel_boost_strafe", legacy_boost)
        self.declare_parameter("opening_wheel_boost_rot", legacy_boost)
        self.declare_parameter("opening_wheel_boost_ms", 120)
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
        self.wheel_scales = _four_scales(
            self.get_parameter("wheel_scales").value, [1.0, 1.0, 1.0, 1.0]
        )
        self.strafe_right = _four_scales(
            self.get_parameter("strafe_right_scales").value, [0.70, 0.70, 0.80, 0.70]
        )
        self.strafe_left = _four_scales(
            self.get_parameter("strafe_left_scales").value, [0.65, 0.75, 0.75, 0.65]
        )
        self.rotation_cw_scales = _four_scales(
            self.get_parameter("rotation_cw_scales").value, [1.0, 1.0, 1.0, 1.0]
        )
        self.rotation_ccw_scales = _four_scales(
            self.get_parameter("rotation_ccw_scales").value, [1.0, 1.0, 1.0, 1.0]
        )
        self.wheel_min = float(self.get_parameter("wheel_min").value)
        self.wheel_min_strafe = float(self.get_parameter("wheel_min_strafe").value)
        self.wheel_min_rot = float(self.get_parameter("wheel_min_rot").value)
        self.wheel_boost_strafe = float(self.get_parameter("wheel_boost_strafe").value)
        self.wheel_boost_rot = float(self.get_parameter("wheel_boost_rot").value)
        self.wheel_boost_ms = float(self.get_parameter("wheel_boost_ms").value)
        self.opening_wheel_min = float(self.get_parameter("opening_wheel_min").value)
        self.opening_wheel_min_strafe = float(self.get_parameter("opening_wheel_min_strafe").value)
        self.opening_wheel_min_rot = float(self.get_parameter("opening_wheel_min_rot").value)
        self.opening_wheel_boost_strafe = float(
            self.get_parameter("opening_wheel_boost_strafe").value
        )
        self.opening_wheel_boost_rot = float(self.get_parameter("opening_wheel_boost_rot").value)
        self.opening_wheel_boost_ms = float(self.get_parameter("opening_wheel_boost_ms").value)
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
        self._debug_pause = False
        self.create_subscription(Bool, "/debug/pause_motion", self._on_debug_pause, 10)
        self._moving = False
        self._boost_until = 0.0
        self._brake_until = 0.0
        self._brake_cmd = [0.0, 0.0, 0.0, 0.0]
        self._last_move_wheels = [0.0, 0.0, 0.0, 0.0]
        self._last_align_profile_move = False

        self.last_cmd_time = self.get_clock().now()
        self.last_cmd = (0.0, 0.0, 0.0)

        self.sub = self.create_subscription(BaseCommand, "/base_command", self.on_cmd, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/base/wheel_speeds", 10)
        self.add_on_set_parameters_callback(self._on_params)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        self.get_logger().info(f"base IK lx={self.lx} ly={self.ly} watchdog={self.cmd_timeout}s")

    def on_cmd(self, msg: BaseCommand) -> None:
        self.last_cmd = (msg.vx, msg.vy, msg.omega)
        self.last_cmd_time = self.get_clock().now()

    def _on_params(self, params) -> SetParametersResult:
        """Apply runtime tuning changes immediately.

        ROS 2 updates the parameter server value before this callback returns, but this node's control
        loop uses cached attributes for speed. Keep those cached values in sync so live diagnostics like
        `ros2 param set /base_controller_node wheel_min_rot ...` affect the next 50 Hz tick.
        """
        try:
            for p in params:
                name = p.name
                if name == "lx":
                    self.lx = float(p.value)
                elif name == "ly":
                    self.ly = float(p.value)
                elif name == "cmd_timeout_sec":
                    self.cmd_timeout = float(p.value)
                elif name == "wheel_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="wheel_scales must have 4 values",
                        )
                    self.wheel_scales = vals
                elif name == "strafe_right_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="strafe_right_scales must have 4 values",
                        )
                    self.strafe_right = vals
                elif name == "strafe_left_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="strafe_left_scales must have 4 values",
                        )
                    self.strafe_left = vals
                elif name == "rotation_cw_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="rotation_cw_scales must have 4 values",
                        )
                    self.rotation_cw_scales = vals
                elif name == "rotation_ccw_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="rotation_ccw_scales must have 4 values",
                        )
                    self.rotation_ccw_scales = vals
                elif name == "wheel_min":
                    self.wheel_min = float(p.value)
                elif name == "wheel_min_strafe":
                    self.wheel_min_strafe = float(p.value)
                elif name == "wheel_min_rot":
                    self.wheel_min_rot = float(p.value)
                elif name == "wheel_boost_strafe":
                    self.wheel_boost_strafe = float(p.value)
                elif name == "wheel_boost_rot":
                    self.wheel_boost_rot = float(p.value)
                elif name == "wheel_boost_ms":
                    self.wheel_boost_ms = float(p.value)
                elif name == "opening_wheel_min":
                    self.opening_wheel_min = float(p.value)
                elif name == "opening_wheel_min_strafe":
                    self.opening_wheel_min_strafe = float(p.value)
                elif name == "opening_wheel_min_rot":
                    self.opening_wheel_min_rot = float(p.value)
                elif name == "opening_wheel_boost_strafe":
                    self.opening_wheel_boost_strafe = float(p.value)
                elif name == "opening_wheel_boost_rot":
                    self.opening_wheel_boost_rot = float(p.value)
                elif name == "opening_wheel_boost_ms":
                    self.opening_wheel_boost_ms = float(p.value)
                elif name == "wheel_deadband":
                    self.wheel_deadband = float(p.value)
                elif name == "wheel_brake_ms":
                    self.wheel_brake_ms = float(p.value)
                elif name == "wheel_brake_scale":
                    self.wheel_brake_scale = float(p.value)
                elif name == "align_wheel_boost":
                    self.align_wheel_boost = float(p.value)
                elif name == "align_brake_off":
                    self.align_brake_off = bool(p.value)
                elif name == "wheel_slew_per_tick":
                    self.wheel_slew = float(p.value)
        except (TypeError, ValueError) as e:
            return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def _on_mstate(self, msg: MissionState) -> None:
        self._mstate = str(msg.state)

    def _on_debug_pause(self, msg: Bool) -> None:
        paused = bool(msg.data)
        if paused != self._debug_pause:
            self.get_logger().warn(
                f"debug pause {'enabled' if paused else 'disabled'}: wheel output clamped"
            )
        self._debug_pause = paused

    def tick(self) -> None:
        # Watchdog: zero output if no recent command
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if self._debug_pause:
            # Diagnostic pause is applied after all /base_command publishers, inside the single
            # base controller owner, so command publishers do not fight each other on /base_command.
            vx, vy, omega = 0.0, 0.0, 0.0
            self._moving = False
            self._boost_until = 0.0
            self._brake_until = 0.0
        elif elapsed > self.cmd_timeout:
            vx, vy, omega = 0.0, 0.0, 0.0
        else:
            vx, vy, omega = self.last_cmd

        # Mecanum inverse kinematics (wheel linear speeds, m/s)
        k = self.lx + self.ly
        v_fl = vx - vy - k * omega
        v_fr = vx + vy + k * omega
        v_rl = vx + vy - k * omega
        v_rr = vx - vy + k * omega

        # Direction-dependent per-wheel trim so forward/strafe/rotation can be tuned independently:
        # forward uses wheel_scales, strafe blends by |vx|:|vy|, pure rotation uses rotation_*_scales.
        sc = select_wheel_scales(
            vx,
            vy,
            omega,
            self.wheel_scales,
            self.strafe_right,
            self.strafe_left,
            self.rotation_cw_scales,
            self.rotation_ccw_scales,
        )
        wheels = [v_fl * sc[0], v_fr * sc[1], v_rl * sc[2], v_rr * sc[3]]
        wheels = [max(-1.0, min(1.0, w)) for w in wheels]

        # Stiction floor + start-from-rest boost + stop brake. The whole vector is scaled so the
        # fastest wheel reaches the moving threshold (direction preserved). On move->stop we emit a
        # brief reverse pulse to kill the heavy base's coast; below the deadband otherwise = full stop.
        m = max(abs(w) for w in wheels)
        now = self.get_clock().now().nanoseconds * 1e-9
        aligning = self._mstate == "ALIGN"
        is_rot = abs(vx) < 0.02 and abs(vy) < 0.02 and abs(omega) > 1e-3
        is_strafe = abs(vy) >= 0.02 and abs(vy) >= abs(vx)
        opening = self._mstate == "OPENING"
        # The opening's one left-strafe pulse is deliberately calibrated from ALIGN.  Give that
        # exact pure-strafe command the same boost-free, slew-bypassed output profile as ALIGN;
        # every other OPENING command keeps the established opening profile.
        opening_align_strafe = (
            opening
            and vy > 0.02
            and abs(vx) < 0.02
            and abs(omega) <= 1e-3
        )
        align_profile = aligning or opening_align_strafe
        kick = False                                  # boost/brake pulse this tick -> bypass slew
        # For pure rotation, mecanum IK scales omega by (lx + ly).  With k=0.2, an intentional
        # omega=0.07 becomes a 0.014 wheel command, below the generic 0.02 wheel deadband.  Do not
        # erase that command before wheel_min_rot can lift it to the calibrated moving floor.
        if below_motion_deadband(
            m,
            self.wheel_deadband,
            pure_rotation=is_rot,
        ):
            if self._moving:                          # transition move -> rest: start the brake pulse
                self._moving = False
                brake = self.wheel_brake_ms > 0.0 and self.wheel_brake_scale > 0.0
                if (aligning or self._last_align_profile_move) and self.align_brake_off:
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
            was_moving = self._moving
            self._moving = True
            self._brake_until = 0.0                   # a fresh move cancels any pending brake
            if align_profile:
                kick = True                           # ALIGN unit steps bypass slew -> hit full duty
                                                      # immediately, matching the boost-free calibration
            # The steady floor is the min duty that keeps the base moving. Pure in-place rotation gets
            # its own floor, while strafe/rotation starts also get a short breakaway boost.
            boost = 0.0
            boost_ms = self.wheel_boost_ms
            if opening and is_rot:
                boost = self.opening_wheel_boost_rot
                boost_ms = self.opening_wheel_boost_ms
            elif opening and is_strafe:
                boost = self.opening_wheel_boost_strafe
                boost_ms = self.opening_wheel_boost_ms
            elif is_rot:
                boost = self.wheel_boost_rot
            elif is_strafe:
                boost = self.wheel_boost_strafe
            if (not align_profile and not was_moving and (is_rot or is_strafe)
                    and boost > 0.0 and boost_ms > 0.0):
                self._boost_until = now + boost_ms / 1000.0
            if opening_align_strafe:
                steady_floor = self.wheel_min_strafe
            elif opening and is_rot and self.opening_wheel_min_rot > 0.0:
                steady_floor = self.opening_wheel_min_rot
            elif opening and is_strafe and self.opening_wheel_min_strafe > 0.0:
                steady_floor = self.opening_wheel_min_strafe
            elif opening and self.opening_wheel_min > 0.0:
                steady_floor = self.opening_wheel_min
            else:
                if is_rot:
                    steady_floor = self.wheel_min_rot
                elif is_strafe:
                    steady_floor = self.wheel_min_strafe
                else:
                    steady_floor = self.wheel_min
            if 0.0 < m < steady_floor:
                s = steady_floor / m
                wheels = [max(-1.0, min(1.0, w * s)) for w in wheels]
            if (is_rot or is_strafe) and now < self._boost_until:
                bm = max(abs(w) for w in wheels)
                if 0.0 < bm < boost:
                    s = boost / bm
                    wheels = [max(-1.0, min(1.0, w * s)) for w in wheels]
                kick = True                           # boost pulse must hit immediately
            self._last_move_wheels = list(wheels)     # remember travel direction for the stop brake
            self._last_align_profile_move = align_profile

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
