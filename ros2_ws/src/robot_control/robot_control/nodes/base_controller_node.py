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


LOCAL_ANCHOR_MOTION_STATES = frozenset(
    {"LOCAL_ANCHOR_INSPECTION", "LOCAL_ANCHOR_ALIGN"}
)
FORWARD_START_BOOST_STATES = frozenset({"SCAN", "APPROACH"})


def local_anchor_motion_profile_active(mission_state: str) -> bool:
    """Return whether the isolated local-anchor base profile owns this state."""
    return str(mission_state).strip().upper() in LOCAL_ANCHOR_MOTION_STATES


def forward_start_boost_active(mission_state: str, vx: float, vy: float) -> bool:
    """Boost only a forward start used by normal sweep/approach navigation.

    Keep reverse, holonomic strafe, ALIGN, opening, and storage profiles unchanged.  Lane heading
    correction may add omega, so yaw is deliberately not part of this translation-only gate.
    """
    state = str(mission_state).strip().upper()
    return bool(
        state in FORWARD_START_BOOST_STATES
        and float(vx) >= 0.02
        and abs(float(vx)) >= abs(float(vy))
    )


def select_mission_profile_value(mission_state: str, main_value, local_anchor_value):
    """Select a local-anchor tuning value without changing the normal match profile."""
    if local_anchor_motion_profile_active(mission_state):
        return local_anchor_value
    return main_value


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


def mix_calibrated_translation_and_rotation(
    vx: float,
    vy: float,
    omega: float,
    k: float,
    translation_scales: list[float],
    drive_rotation_scales: list[float],
) -> list[float]:
    """Mix calibrated translation with an independently scaled yaw component.

    Straight/strafe trims compensate unequal wheel drive gains.  Applying those unequal trims to
    the yaw term as well can cancel most of a simultaneous steering command, so the yaw vector is
    kept symmetric and added only after translation calibration.
    """
    translation = [vx - vy, vx + vy, vx + vy, vx - vy]
    rotation = [-k * omega, k * omega, -k * omega, k * omega]
    return [
        translation[i] * translation_scales[i]
        + rotation[i] * drive_rotation_scales[i]
        for i in range(4)
    ]


def is_precision_strafe_command(
    vx: float,
    vy: float,
    omega: float,
    duty: float,
    tolerance: float,
) -> bool:
    """Identify the calibrated no-boost lateral unit-step command."""
    return bool(
        duty > 0.0
        and abs(vx) < 0.02
        and abs(omega) <= 1e-3
        and abs(abs(vy) - duty) <= max(0.0, tolerance)
    )


def select_brake_pulse_parameters(
    *,
    align_profile: bool,
    rotation_profile: bool,
    align_brake_off: bool,
    wheel_brake_ms: float,
    wheel_brake_scale: float,
    rotation_wheel_brake_ms: float,
    rotation_wheel_brake_scale: float,
    align_wheel_brake_ms: float,
    align_wheel_brake_scale: float,
) -> tuple[float, float]:
    """Select independent ALIGN, in-place rotation, or translation brake parameters."""
    if align_profile:
        if align_brake_off:
            return 0.0, 0.0
        return (
            max(0.0, float(align_wheel_brake_ms)),
            max(0.0, float(align_wheel_brake_scale)),
        )
    if rotation_profile:
        return (
            max(0.0, float(rotation_wheel_brake_ms)),
            max(0.0, float(rotation_wheel_brake_scale)),
        )
    return max(0.0, float(wheel_brake_ms)), max(0.0, float(wheel_brake_scale))


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
        # Simultaneous translation+yaw needs an independent, symmetric steering vector.  Pure
        # in-place turns keep using rotation_cw/ccw_scales above.
        self.declare_parameter("drive_rotation_cw_scales", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("drive_rotation_ccw_scales", [1.0, 1.0, 1.0, 1.0])
        # The isolated local-anchor test uses a deliberately gentler pulse/stop profile.  Keep its
        # values separate so normal match navigation and ALIGN retain their existing calibration.
        self.declare_parameter(
            "local_anchor_rotation_cw_scales", [1.0, 1.0, 1.0, 1.0]
        )
        self.declare_parameter(
            "local_anchor_rotation_ccw_scales", [1.0, 1.0, 1.0, 1.0]
        )
        self.declare_parameter("local_anchor_wheel_min_rot", 0.07)
        self.declare_parameter("local_anchor_wheel_boost_rot", 0.35)
        self.declare_parameter("local_anchor_wheel_boost_ms", 100)
        self.declare_parameter("local_anchor_rotation_wheel_brake_ms", 120)
        self.declare_parameter("local_anchor_rotation_wheel_brake_scale", 0.315)
        self.declare_parameter("local_anchor_align_wheel_brake_ms", 120)
        self.declare_parameter("local_anchor_align_wheel_brake_scale", 0.315)
        self.declare_parameter("local_anchor_wheel_slew_per_tick", 0.06)
        # STICTION: the heavy base only moves above ~this <BASE> magnitude (firmware normalises by
        # MAX_SPEED 0.6, so 0.42 ~ 0.7 duty). The velocity pipeline commands 0.05-0.20 which just
        # buzzed — so any real move is floored to this (direction preserved). 0 disables.
        self.declare_parameter("wheel_min", 0.42)
        self.declare_parameter("wheel_min_strafe", 0.42)
        # Separate stiction floor for PURE IN-PLACE ROTATION (|vx|,|vy|~0). In-place turns otherwise
        # get floored to wheel_min and spin too fast; set this LOWER for a slow-but-moving CW/CCW turn
        # (the start-boost still breaks static friction, then it relaxes to this). Raise if it stalls.
        self.declare_parameter("wheel_min_rot", 0.07)
        # STATIC friction from REST is higher, so starts get a short breakaway kick, then relax to
        # wheel_min/wheel_min_rot. Forward boost is narrowly scoped to SCAN/APPROACH and uniformly
        # scales the calibrated wheel vector, preserving straight-line trim.
        self.declare_parameter("wheel_boost", 0.60)
        legacy_boost = float(self.get_parameter("wheel_boost").value)
        self.declare_parameter("wheel_boost_forward", 0.16)
        self.declare_parameter("wheel_boost_forward_ms", 120)
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
        # BRAKE: translation, pure in-place rotation, and ALIGN use independent reverse pulses.
        # wheel_brake_* remains the translation/strafe profile for backward-compatible YAML.
        self.declare_parameter("wheel_brake_ms", 120)
        self.declare_parameter("wheel_brake_scale", 0.315)
        self.declare_parameter("rotation_wheel_brake_ms", 120)
        self.declare_parameter("rotation_wheel_brake_scale", 0.315)
        self.declare_parameter("align_wheel_brake_ms", 120)
        self.declare_parameter("align_wheel_brake_scale", 0.315)

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
        self.drive_rotation_cw_scales = _four_scales(
            self.get_parameter("drive_rotation_cw_scales").value,
            [1.0, 1.0, 1.0, 1.0],
        )
        self.drive_rotation_ccw_scales = _four_scales(
            self.get_parameter("drive_rotation_ccw_scales").value,
            [1.0, 1.0, 1.0, 1.0],
        )
        self.local_anchor_rotation_cw_scales = _four_scales(
            self.get_parameter("local_anchor_rotation_cw_scales").value,
            [1.0, 1.0, 1.0, 1.0],
        )
        self.local_anchor_rotation_ccw_scales = _four_scales(
            self.get_parameter("local_anchor_rotation_ccw_scales").value,
            [1.0, 1.0, 1.0, 1.0],
        )
        self.local_anchor_wheel_min_rot = float(
            self.get_parameter("local_anchor_wheel_min_rot").value
        )
        self.local_anchor_wheel_boost_rot = float(
            self.get_parameter("local_anchor_wheel_boost_rot").value
        )
        self.local_anchor_wheel_boost_ms = float(
            self.get_parameter("local_anchor_wheel_boost_ms").value
        )
        self.local_anchor_rotation_wheel_brake_ms = float(
            self.get_parameter("local_anchor_rotation_wheel_brake_ms").value
        )
        self.local_anchor_rotation_wheel_brake_scale = float(
            self.get_parameter("local_anchor_rotation_wheel_brake_scale").value
        )
        self.local_anchor_align_wheel_brake_ms = float(
            self.get_parameter("local_anchor_align_wheel_brake_ms").value
        )
        self.local_anchor_align_wheel_brake_scale = float(
            self.get_parameter("local_anchor_align_wheel_brake_scale").value
        )
        self.local_anchor_wheel_slew = float(
            self.get_parameter("local_anchor_wheel_slew_per_tick").value
        )
        self.wheel_min = float(self.get_parameter("wheel_min").value)
        self.wheel_min_strafe = float(self.get_parameter("wheel_min_strafe").value)
        self.wheel_min_rot = float(self.get_parameter("wheel_min_rot").value)
        self.wheel_boost_forward = float(
            self.get_parameter("wheel_boost_forward").value
        )
        self.wheel_boost_forward_ms = float(
            self.get_parameter("wheel_boost_forward_ms").value
        )
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
        self.rotation_wheel_brake_ms = float(
            self.get_parameter("rotation_wheel_brake_ms").value
        )
        self.rotation_wheel_brake_scale = float(
            self.get_parameter("rotation_wheel_brake_scale").value
        )
        self.align_wheel_brake_ms = float(
            self.get_parameter("align_wheel_brake_ms").value
        )
        self.align_wheel_brake_scale = float(
            self.get_parameter("align_wheel_brake_scale").value
        )
        # ALIGN fine-positioning has its own boost and independently switchable brake profile.
        self.declare_parameter("align_wheel_boost", 0.45)
        self.declare_parameter("align_brake_off", True)
        self.declare_parameter("precision_strafe_duty", 0.315)
        self.declare_parameter("precision_strafe_tolerance", 0.005)
        self.align_wheel_boost = float(self.get_parameter("align_wheel_boost").value)
        self.align_brake_off = bool(self.get_parameter("align_brake_off").value)
        self.precision_strafe_duty = max(
            0.0, float(self.get_parameter("precision_strafe_duty").value)
        )
        self.precision_strafe_tolerance = max(
            0.0, float(self.get_parameter("precision_strafe_tolerance").value)
        )
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
        self._last_rotation_move = False

        self.last_cmd_time = self.get_clock().now()
        self.last_cmd = (0.0, 0.0, 0.0)

        self.sub = self.create_subscription(BaseCommand, "/base_command", self.on_cmd, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/base/wheel_speeds", 10)
        self.add_on_set_parameters_callback(self._on_params)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        self.get_logger().info(
            f"base IK lx={self.lx} ly={self.ly} watchdog={self.cmd_timeout}s "
            f"drive_rotation_cw={self.drive_rotation_cw_scales} "
            f"drive_rotation_ccw={self.drive_rotation_ccw_scales}"
        )

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
                elif name == "drive_rotation_cw_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="drive_rotation_cw_scales must have 4 values",
                        )
                    self.drive_rotation_cw_scales = vals
                elif name == "drive_rotation_ccw_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="drive_rotation_ccw_scales must have 4 values",
                        )
                    self.drive_rotation_ccw_scales = vals
                elif name == "local_anchor_rotation_cw_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="local_anchor_rotation_cw_scales must have 4 values",
                        )
                    self.local_anchor_rotation_cw_scales = vals
                elif name == "local_anchor_rotation_ccw_scales":
                    vals = [float(v) for v in p.value]
                    if len(vals) != 4:
                        return SetParametersResult(
                            successful=False,
                            reason="local_anchor_rotation_ccw_scales must have 4 values",
                        )
                    self.local_anchor_rotation_ccw_scales = vals
                elif name == "local_anchor_wheel_min_rot":
                    self.local_anchor_wheel_min_rot = float(p.value)
                elif name == "local_anchor_wheel_boost_rot":
                    self.local_anchor_wheel_boost_rot = float(p.value)
                elif name == "local_anchor_wheel_boost_ms":
                    self.local_anchor_wheel_boost_ms = float(p.value)
                elif name == "local_anchor_rotation_wheel_brake_ms":
                    self.local_anchor_rotation_wheel_brake_ms = float(p.value)
                elif name == "local_anchor_rotation_wheel_brake_scale":
                    self.local_anchor_rotation_wheel_brake_scale = float(p.value)
                elif name == "local_anchor_align_wheel_brake_ms":
                    self.local_anchor_align_wheel_brake_ms = float(p.value)
                elif name == "local_anchor_align_wheel_brake_scale":
                    self.local_anchor_align_wheel_brake_scale = float(p.value)
                elif name == "local_anchor_wheel_slew_per_tick":
                    self.local_anchor_wheel_slew = float(p.value)
                elif name == "wheel_min":
                    self.wheel_min = float(p.value)
                elif name == "wheel_min_strafe":
                    self.wheel_min_strafe = float(p.value)
                elif name == "wheel_min_rot":
                    self.wheel_min_rot = float(p.value)
                elif name == "wheel_boost_forward":
                    self.wheel_boost_forward = float(p.value)
                elif name == "wheel_boost_forward_ms":
                    self.wheel_boost_forward_ms = float(p.value)
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
                elif name == "rotation_wheel_brake_ms":
                    self.rotation_wheel_brake_ms = float(p.value)
                elif name == "rotation_wheel_brake_scale":
                    self.rotation_wheel_brake_scale = float(p.value)
                elif name == "align_wheel_brake_ms":
                    self.align_wheel_brake_ms = float(p.value)
                elif name == "align_wheel_brake_scale":
                    self.align_wheel_brake_scale = float(p.value)
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

        # Direction-dependent per-wheel trim so forward/strafe/rotation can be tuned independently:
        # forward uses wheel_scales, strafe blends by |vx|:|vy|, pure rotation uses rotation_*_scales.
        # During translation+yaw, do not multiply yaw by the unequal straight trims: add a symmetric
        # drive_rotation vector after the translation calibration so steering authority is retained.
        local_anchor_profile = local_anchor_motion_profile_active(self._mstate)
        rotation_cw_scales = select_mission_profile_value(
            self._mstate,
            self.rotation_cw_scales,
            self.local_anchor_rotation_cw_scales,
        )
        rotation_ccw_scales = select_mission_profile_value(
            self._mstate,
            self.rotation_ccw_scales,
            self.local_anchor_rotation_ccw_scales,
        )
        pure_rotation = abs(vx) < 0.02 and abs(vy) < 0.02 and abs(omega) > 1e-3
        if pure_rotation:
            rotation_scales = rotation_cw_scales if omega < 0.0 else rotation_ccw_scales
            wheels = mix_calibrated_translation_and_rotation(
                vx, vy, omega, k, [1.0, 1.0, 1.0, 1.0], rotation_scales
            )
        else:
            translation_scales = select_wheel_scales(
                vx,
                vy,
                0.0,
                self.wheel_scales,
                self.strafe_right,
                self.strafe_left,
                rotation_cw_scales,
                rotation_ccw_scales,
            )
            drive_rotation_scales = (
                self.drive_rotation_cw_scales
                if omega < 0.0
                else self.drive_rotation_ccw_scales
            )
            wheels = mix_calibrated_translation_and_rotation(
                vx,
                vy,
                omega,
                k,
                translation_scales,
                drive_rotation_scales,
            )
        wheels = [max(-1.0, min(1.0, w)) for w in wheels]

        # Stiction floor + start-from-rest boost + stop brake. The whole vector is scaled so the
        # fastest wheel reaches the moving threshold (direction preserved). On move->stop we emit a
        # brief reverse pulse to kill the heavy base's coast; below the deadband otherwise = full stop.
        m = max(abs(w) for w in wheels)
        now = self.get_clock().now().nanoseconds * 1e-9
        aligning = self._mstate in {"ALIGN", "LOCAL_ANCHOR_ALIGN"}
        is_rot = abs(vx) < 0.02 and abs(vy) < 0.02 and abs(omega) > 1e-3
        is_strafe = abs(vy) >= 0.02 and abs(vy) >= abs(vx)
        forward_start_boost = forward_start_boost_active(self._mstate, vx, vy)
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
        precision_strafe = is_precision_strafe_command(
            vx,
            vy,
            omega,
            self.precision_strafe_duty,
            self.precision_strafe_tolerance,
        )
        # ALIGN translation keeps its calibrated no-boost unit-step profile. Pure heading-search
        # pulses use the normal rotation profile so every stop/start can re-arm rotation torque.
        align_profile = (aligning and not is_rot) or opening_align_strafe or precision_strafe
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
                rotation_brake_ms = select_mission_profile_value(
                    self._mstate,
                    self.rotation_wheel_brake_ms,
                    self.local_anchor_rotation_wheel_brake_ms,
                )
                rotation_brake_scale = select_mission_profile_value(
                    self._mstate,
                    self.rotation_wheel_brake_scale,
                    self.local_anchor_rotation_wheel_brake_scale,
                )
                align_brake_ms = select_mission_profile_value(
                    self._mstate,
                    self.align_wheel_brake_ms,
                    self.local_anchor_align_wheel_brake_ms,
                )
                align_brake_scale = select_mission_profile_value(
                    self._mstate,
                    self.align_wheel_brake_scale,
                    self.local_anchor_align_wheel_brake_scale,
                )
                brake_ms, brake_scale = select_brake_pulse_parameters(
                    align_profile=aligning or self._last_align_profile_move,
                    rotation_profile=self._last_rotation_move,
                    align_brake_off=self.align_brake_off,
                    wheel_brake_ms=self.wheel_brake_ms,
                    wheel_brake_scale=self.wheel_brake_scale,
                    rotation_wheel_brake_ms=rotation_brake_ms,
                    rotation_wheel_brake_scale=rotation_brake_scale,
                    align_wheel_brake_ms=align_brake_ms,
                    align_wheel_brake_scale=align_brake_scale,
                )
                brake = brake_ms > 0.0 and brake_scale > 0.0
                if brake:
                    self._brake_until = now + brake_ms / 1000.0
                    self._brake_cmd = [max(-1.0, min(1.0, -w * brake_scale))
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
            if local_anchor_profile and is_rot:
                boost = self.local_anchor_wheel_boost_rot
                boost_ms = self.local_anchor_wheel_boost_ms
            elif forward_start_boost:
                boost = self.wheel_boost_forward
                boost_ms = self.wheel_boost_forward_ms
            elif opening and is_rot:
                boost = self.opening_wheel_boost_rot
                boost_ms = self.opening_wheel_boost_ms
            elif opening and is_strafe:
                boost = self.opening_wheel_boost_strafe
                boost_ms = self.opening_wheel_boost_ms
            elif is_rot:
                boost = self.wheel_boost_rot
            elif is_strafe:
                boost = self.wheel_boost_strafe
            if (not align_profile and not was_moving
                    and (is_rot or is_strafe or forward_start_boost)
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
                    steady_floor = select_mission_profile_value(
                        self._mstate,
                        self.wheel_min_rot,
                        self.local_anchor_wheel_min_rot,
                    )
                elif is_strafe:
                    steady_floor = self.wheel_min_strafe
                else:
                    steady_floor = self.wheel_min
            if 0.0 < m < steady_floor:
                s = steady_floor / m
                wheels = [max(-1.0, min(1.0, w * s)) for w in wheels]
            if (is_rot or is_strafe or forward_start_boost) and now < self._boost_until:
                bm = max(abs(w) for w in wheels)
                if 0.0 < bm < boost:
                    s = boost / bm
                    wheels = [max(-1.0, min(1.0, w * s)) for w in wheels]
                kick = True                           # boost pulse must hit immediately
            self._last_move_wheels = list(wheels)     # remember travel direction for the stop brake
            self._last_align_profile_move = align_profile
            self._last_rotation_move = is_rot

        # Slew-limit toward the target so accel/decel is smooth (no jack-rabbit start / no slip).
        # Brake pulses bypass slew (kick) so they hit hard enough to actually cut the coast.
        wheel_slew = select_mission_profile_value(
            self._mstate,
            self.wheel_slew,
            self.local_anchor_wheel_slew,
        )
        if wheel_slew > 0.0 and not kick:
            lim = []
            for w, pv in zip(wheels, self._prev_out):
                dw = w - pv
                if dw > wheel_slew:
                    dw = wheel_slew
                elif dw < -wheel_slew:
                    dw = -wheel_slew
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
