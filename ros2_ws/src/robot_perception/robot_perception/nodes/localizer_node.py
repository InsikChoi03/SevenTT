"""Robot localizer: fuse mecanum wheel odometry with monocular visual odometry.

Estimates robot pose (x, y, theta) in the `field` frame. NO IMU.

Sources, in order of trust:
  1. Mecanum wheel odometry (dead reckoning) — primary x/y/theta integration.
       Forward kinematics is the exact inverse of base_controller's IK (k = lx + ly):
         vx = (fl+fr+rl+rr)/4
         vy = (-fl+fr+rl-rr)/4
         w  = (-fl+fr-rl+rr)/(4*(lx+ly))
       Integrated in the field frame using the current heading.
  2. Monocular visual odometry from the top camera — heading-drift correction only
       (translation scale is ambiguous for a single camera). Frame-to-frame yaw is
       estimated with estimateAffinePartial2D over LK-tracked features and blended
       into theta with a small weight via a complementary filter.
  3. Landmark absolute-correction HOOK (_detect_landmarks) — snaps theta to the
       nearest wall-aligned heading (multiple of pi/2) when a dominant straight wall
       edge is detected. APPROXIMATE until proper extrinsic/geometry calibration.

Publishes geometry_msgs/PoseStamped on /localization/pose (frame=`field`) at a fixed
rate, and optionally broadcasts the tf field->base_link. The node runs with no inputs:
with no wheel odometry it simply republishes the initial start-zone pose.

Dry-run: cv2 is required and always available here; the node still publishes the
initial pose with no camera and no wheel odometry connected.
"""
from __future__ import annotations

from collections import deque
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from robot_interfaces.msg import MissionState

# tf2 broadcasting is optional: guard the import so the node runs without tf2_ros.
try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster

    TF2_AVAILABLE = True
except ImportError:  # pragma: no cover - environment without tf2_ros
    TF2_AVAILABLE = False


def yaw_to_quaternion(theta: float) -> Tuple[float, float, float, float]:
    """Return (qx, qy, qz, qw) for a yaw-only rotation about z."""
    return 0.0, 0.0, math.sin(theta * 0.5), math.cos(theta * 0.5)


def wrap_angle(theta: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(theta), math.cos(theta))


_LOCAL_ANCHOR_MISSION_STATES = frozenset(
    {"LOCAL_ANCHOR_INSPECTION", "LOCAL_ANCHOR_ALIGN"}
)


def object_flow_min_conf_for_state(
    mission_state: str,
    normal_min_conf: float,
    local_anchor_min_conf: float,
) -> float:
    """Select the relaxed three-point flow gate only in local-anchor states."""
    state = str(mission_state).strip().upper()
    selected = (
        local_anchor_min_conf
        if state in _LOCAL_ANCHOR_MISSION_STATES
        else normal_min_conf
    )
    return max(0.0, min(1.0, float(selected)))


def limit_planar_delta(
    dx: float, dy: float, max_distance_m: float
) -> Tuple[float, float, bool]:
    """Slew-limit a planar correction without changing its direction."""
    limit = max(0.0, float(max_distance_m))
    distance = math.hypot(float(dx), float(dy))
    if distance <= limit or distance <= 1e-12:
        return float(dx), float(dy), False
    scale = limit / distance
    return float(dx) * scale, float(dy) * scale, True


def blend_flow_velocity_with_imu(
    flow_velocity: np.ndarray,
    expected_velocity: np.ndarray,
    min_gain: float,
    full_trust_error_mps: float,
    soft_error_mps: float,
) -> Tuple[np.ndarray, float, float]:
    """Softly blend a flow velocity toward the IMU-supported velocity prediction."""
    flow = np.asarray(flow_velocity, dtype=np.float64)
    expected = np.asarray(expected_velocity, dtype=np.float64)
    error = float(np.linalg.norm(flow - expected))
    gain_floor = max(0.0, min(1.0, float(min_gain)))
    full_trust = max(0.0, float(full_trust_error_mps))
    soft_error = max(full_trust + 1e-6, float(soft_error_mps))
    if error <= full_trust:
        gain = 1.0
    elif error >= soft_error:
        gain = gain_floor
    else:
        ratio = (error - full_trust) / (soft_error - full_trust)
        smoothstep = ratio * ratio * (3.0 - 2.0 * ratio)
        gain = 1.0 - (1.0 - gain_floor) * smoothstep
    blended = expected + gain * (flow - expected)
    return blended, float(gain), error


class LocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("localizer_node")

        # Mecanum half-dimensions (must match base_controller's IK).
        self.declare_parameter("lx", 0.10)
        self.declare_parameter("ly", 0.10)

        # Start-zone pose, facing into the field (theta ~ pi).
        self.declare_parameter("initial_x", 3.8)
        self.declare_parameter("initial_y", 0.2)
        self.declare_parameter("initial_theta", 3.14159)

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("use_visual_odometry", True)
        self.declare_parameter("broadcast_tf", True)
        # Snap heading to the nearest cardinal from a dominant wall edge. Great inside the
        # arena, but OUTSIDE it (mock test) random room/table edges cause spurious snaps, so
        # allow disabling it (VO yaw stays on and works on any textured floor).
        self.declare_parameter("use_landmark_correction", True)
        # Object-landmark pose correction: consume the world model's rigid drift estimate on
        # /localization/landmark_correction (dx,dy,dtheta) and blend a small, clamped fraction
        # into the pose. This corrects x/y AND heading using re-observed mapped objects.
        self.declare_parameter("use_object_landmarks", False)
        self.declare_parameter("landmark_gain", 0.2)
        self.declare_parameter("landmark_max_step_m", 0.1)
        self.declare_parameter("landmark_max_step_rad", 0.1)
        self.declare_parameter("landmark_correction_max_speed_mps", 0.20)
        self.declare_parameter("landmark_correction_max_dt_sec", 0.10)
        # HEADING authority for a CONFIDENT landmark fix (scaled by the fix's confidence). Higher
        # than landmark_gain because a well-conditioned object-bearing heading is an absolute ref.
        self.declare_parameter("landmark_theta_gain", 0.7)

        # Top-camera intrinsics. If any is 0 -> skip undistort / VO-scale usage.
        self.declare_parameter("top_fx", 0.0)
        self.declare_parameter("top_fy", 0.0)
        self.declare_parameter("top_cx", 0.0)
        self.declare_parameter("top_cy", 0.0)
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
        # Top cam is a ~150 deg FISHEYE. fisheye_model=true -> rectify with cv2.fisheye maps
        # (4-elem dist_coeffs) so wall edges become straight for Hough/LK; else pinhole undistort.
        self.declare_parameter("fisheye_model", False)

        self.lx = float(self.get_parameter("lx").value)
        self.ly = float(self.get_parameter("ly").value)
        self.k = self.lx + self.ly

        # Wheel odometry can run from any 3+ valid mecanum wheel encoders. This keeps localization
        # usable while one encoder is known-bad (currently FL on the base).
        self.declare_parameter("use_wheel_odom", True)
        self.declare_parameter("wheel_odom_enabled_wheels", [True, True, True, True])
        self.declare_parameter("wheel_odom_deadband_mps", 0.005)
        self.declare_parameter("wheel_odom_max_dt_sec", 0.4)
        # Encoder motion constraint is deliberately separate from wheel-odom integration. It uses
        # actual measured wheel motion to classify the current motion, with commands only as a sanity
        # check. It never integrates encoder distance into x/y/theta while use_wheel_odom is false.
        self.declare_parameter("encoder_motion_constraint_enabled", False)
        self.declare_parameter("encoder_motion_constraint_publish_debug", True)
        self.declare_parameter("encoder_constraint_cmd_stale_sec", 0.30)
        self.declare_parameter("encoder_constraint_odom_stale_sec", 0.40)
        self.declare_parameter("encoder_constraint_linear_eps_mps", 0.025)
        self.declare_parameter("encoder_constraint_angular_eps_rps", 0.12)
        self.declare_parameter("encoder_constraint_axis_ratio", 2.0)
        self.declare_parameter("encoder_constraint_stop_translation_gain", 0.0)
        self.declare_parameter("encoder_constraint_stop_yaw_gain", 0.05)
        self.declare_parameter("encoder_constraint_forward_gain", 1.0)
        self.declare_parameter("encoder_constraint_forward_lateral_gain", 0.12)
        self.declare_parameter("encoder_constraint_forward_yaw_gain", 0.50)
        self.declare_parameter("encoder_constraint_lateral_gain", 1.0)
        self.declare_parameter("encoder_constraint_lateral_forward_gain", 0.12)
        self.declare_parameter("encoder_constraint_lateral_yaw_gain", 0.50)
        self.declare_parameter("encoder_constraint_rotate_translation_gain", 0.08)
        self.declare_parameter("encoder_constraint_rotate_yaw_gain", 1.0)
        self.declare_parameter("encoder_constraint_mixed_translation_gain", 0.60)
        self.declare_parameter("encoder_constraint_mixed_yaw_gain", 0.80)
        # Wall corrections remain an absolute drift trim even when the base is stationary. Floors
        # prevent the motion constraint from fully discarding a confident wall-based correction.
        self.declare_parameter("encoder_constraint_wall_translation_floor", 0.35)
        self.declare_parameter("encoder_constraint_wall_yaw_floor", 0.35)
        self.declare_parameter("stationary_wheel_eps_mps", 0.008)
        self.declare_parameter("stationary_cmd_eps", 0.03)
        self.declare_parameter("stationary_cmd_stale_sec", 0.7)
        self.declare_parameter("stationary_required_sec", 0.8)
        self.declare_parameter("suppress_object_flow_when_stationary", True)
        self.declare_parameter("freeze_pose_when_stationary", True)
        self.declare_parameter("wall_anchor_gain", 0.50)
        self.declare_parameter("wall_anchor_max_step_m", 0.03)
        self.declare_parameter("wall_anchor_max_step_rad", 0.03)
        self.declare_parameter("wall_anchor_deadband_m", 0.005)
        self.declare_parameter("wall_field_gain", 0.35)
        self.declare_parameter("wall_field_theta_gain", 0.35)
        self.declare_parameter("wall_field_fast_gain", 1.0)
        self.declare_parameter("wall_field_fast_theta_gain", 1.0)
        self.declare_parameter("wall_field_max_step_m", 0.05)
        self.declare_parameter("wall_field_fast_max_step_m", 0.12)
        self.declare_parameter("wall_field_max_step_rad", 0.04)
        self.declare_parameter("wall_field_fast_max_step_rad", 0.0)
        self.declare_parameter("wall_field_deadband_m", 0.005)
        self.declare_parameter("wall_correction_stale_sec", 1.5)
        self.declare_parameter("wall_translation_heading_gate_rad", 0.05235987756)
        enabled = [bool(v) for v in self.get_parameter("wheel_odom_enabled_wheels").value]
        self.wheel_odom_enabled = enabled if len(enabled) == 4 else [True, True, True, True]
        self.wheel_odom_deadband = float(self.get_parameter("wheel_odom_deadband_mps").value)
        self.wheel_odom_max_dt = float(self.get_parameter("wheel_odom_max_dt_sec").value)
        self.encoder_motion_constraint_enabled = bool(
            self.get_parameter("encoder_motion_constraint_enabled").value
        )
        self.encoder_motion_constraint_publish_debug = bool(
            self.get_parameter("encoder_motion_constraint_publish_debug").value
        )
        self.encoder_constraint_cmd_stale_sec = float(
            self.get_parameter("encoder_constraint_cmd_stale_sec").value
        )
        self.encoder_constraint_odom_stale_sec = float(
            self.get_parameter("encoder_constraint_odom_stale_sec").value
        )
        self.encoder_constraint_linear_eps = float(
            self.get_parameter("encoder_constraint_linear_eps_mps").value
        )
        self.encoder_constraint_angular_eps = float(
            self.get_parameter("encoder_constraint_angular_eps_rps").value
        )
        self.encoder_constraint_axis_ratio = max(
            1.0, float(self.get_parameter("encoder_constraint_axis_ratio").value)
        )
        self.encoder_constraint_stop_translation_gain = float(
            self.get_parameter("encoder_constraint_stop_translation_gain").value
        )
        self.encoder_constraint_stop_yaw_gain = float(
            self.get_parameter("encoder_constraint_stop_yaw_gain").value
        )
        self.encoder_constraint_forward_gain = float(
            self.get_parameter("encoder_constraint_forward_gain").value
        )
        self.encoder_constraint_forward_lateral_gain = float(
            self.get_parameter("encoder_constraint_forward_lateral_gain").value
        )
        self.encoder_constraint_forward_yaw_gain = float(
            self.get_parameter("encoder_constraint_forward_yaw_gain").value
        )
        self.encoder_constraint_lateral_gain = float(
            self.get_parameter("encoder_constraint_lateral_gain").value
        )
        self.encoder_constraint_lateral_forward_gain = float(
            self.get_parameter("encoder_constraint_lateral_forward_gain").value
        )
        self.encoder_constraint_lateral_yaw_gain = float(
            self.get_parameter("encoder_constraint_lateral_yaw_gain").value
        )
        self.encoder_constraint_rotate_translation_gain = float(
            self.get_parameter("encoder_constraint_rotate_translation_gain").value
        )
        self.encoder_constraint_rotate_yaw_gain = float(
            self.get_parameter("encoder_constraint_rotate_yaw_gain").value
        )
        self.encoder_constraint_mixed_translation_gain = float(
            self.get_parameter("encoder_constraint_mixed_translation_gain").value
        )
        self.encoder_constraint_mixed_yaw_gain = float(
            self.get_parameter("encoder_constraint_mixed_yaw_gain").value
        )
        self.encoder_constraint_wall_translation_floor = float(
            self.get_parameter("encoder_constraint_wall_translation_floor").value
        )
        self.encoder_constraint_wall_yaw_floor = float(
            self.get_parameter("encoder_constraint_wall_yaw_floor").value
        )
        self.stationary_wheel_eps = float(self.get_parameter("stationary_wheel_eps_mps").value)
        self.stationary_cmd_eps = float(self.get_parameter("stationary_cmd_eps").value)
        self.stationary_cmd_stale_sec = float(self.get_parameter("stationary_cmd_stale_sec").value)
        self.stationary_required_sec = float(self.get_parameter("stationary_required_sec").value)
        self.suppress_object_flow_when_stationary = bool(
            self.get_parameter("suppress_object_flow_when_stationary").value
        )
        self.freeze_pose_when_stationary = bool(
            self.get_parameter("freeze_pose_when_stationary").value
        )
        self.use_wheel_odom = bool(self.get_parameter("use_wheel_odom").value)
        self.wall_anchor_gain = float(self.get_parameter("wall_anchor_gain").value)
        self.wall_anchor_max_step_m = float(self.get_parameter("wall_anchor_max_step_m").value)
        self.wall_anchor_max_step_rad = float(self.get_parameter("wall_anchor_max_step_rad").value)
        self.wall_anchor_deadband_m = float(self.get_parameter("wall_anchor_deadband_m").value)
        self.wall_field_gain = float(self.get_parameter("wall_field_gain").value)
        self.wall_field_theta_gain = float(self.get_parameter("wall_field_theta_gain").value)
        self.wall_field_fast_gain = float(self.get_parameter("wall_field_fast_gain").value)
        self.wall_field_fast_theta_gain = float(
            self.get_parameter("wall_field_fast_theta_gain").value
        )
        self.wall_field_max_step_m = float(self.get_parameter("wall_field_max_step_m").value)
        self.wall_field_fast_max_step_m = float(
            self.get_parameter("wall_field_fast_max_step_m").value
        )
        self.wall_field_max_step_rad = float(self.get_parameter("wall_field_max_step_rad").value)
        self.wall_field_fast_max_step_rad = float(
            self.get_parameter("wall_field_fast_max_step_rad").value
        )
        self.wall_field_deadband_m = float(self.get_parameter("wall_field_deadband_m").value)
        self.wall_correction_stale_sec = float(
            self.get_parameter("wall_correction_stale_sec").value
        )
        self.wall_translation_heading_gate_rad = max(
            0.0, float(self.get_parameter("wall_translation_heading_gate_rad").value)
        )
        self._wheel_rows_all = np.array(
            [
                [1.0, -1.0, -self.k],  # FL
                [1.0, 1.0, self.k],    # FR
                [1.0, 1.0, -self.k],   # RL
                [1.0, -1.0, self.k],   # RR
            ],
            dtype=np.float64,
        )
        self._wheel_mask = np.array(self.wheel_odom_enabled, dtype=bool)
        self._wheel_rows = self._wheel_rows_all[self._wheel_mask]
        if int(self._wheel_mask.sum()) < 3:
            self.get_logger().error(
                "wheel_odom_enabled_wheels needs at least 3 true values for mecanum odometry"
            )

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.theta = float(self.get_parameter("initial_theta").value)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.use_vo = bool(self.get_parameter("use_visual_odometry").value)
        self.broadcast_tf = bool(self.get_parameter("broadcast_tf").value)
        self.use_landmark_correction = bool(self.get_parameter("use_landmark_correction").value)
        self.use_object_landmarks = bool(self.get_parameter("use_object_landmarks").value)
        self.landmark_gain = float(self.get_parameter("landmark_gain").value)
        self.landmark_theta_gain = float(self.get_parameter("landmark_theta_gain").value)
        self.landmark_max_step_m = float(self.get_parameter("landmark_max_step_m").value)
        self.landmark_max_step_rad = float(self.get_parameter("landmark_max_step_rad").value)
        self.landmark_correction_max_speed_mps = max(
            0.0, float(self.get_parameter("landmark_correction_max_speed_mps").value)
        )
        self.landmark_correction_max_dt_sec = max(
            0.0, float(self.get_parameter("landmark_correction_max_dt_sec").value)
        )
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.landmark_correction_nominal_dt = 1.0 / publish_rate_hz
        self.last_landmark_correction_time: Optional[float] = None
        # VO now OWNS rotation (applies the FULL measured yaw delta). Wheel-odom rotation is only a
        # FALLBACK used when VO has been stale this long, so the two never double-count theta.
        self.declare_parameter("vo_stale_sec", 0.25)
        self.vo_stale_sec = float(self.get_parameter("vo_stale_sec").value)
        self.last_vo_time = None
        # Per-frame VO yaw below this is treated as NOISE and not integrated — otherwise a stationary
        # robot slowly drifts (random-walk of the frame-to-frame estimate). Real turns exceed it.
        self.declare_parameter("vo_deadband_rad", 0.004)
        self.vo_deadband = float(self.get_parameter("vo_deadband_rad").value)
        # OBJECT-FLOW odometry is the PRIMARY rotation source: it comes from real wide-cam object
        # points (world_model /localization/object_odom), so it does NOT hallucinate yaw from motor
        # vibration the way dense LK optical flow does. When object-flow is fresh, the LK VO is
        # demoted to a fallback (few-object frames only) and never fights it.
        self.declare_parameter("use_object_flow", True)
        self.use_object_flow = bool(self.get_parameter("use_object_flow").value)
        self.declare_parameter("object_flow_min_conf", 0.25)   # ignore a low-confidence flow estimate
        self.object_flow_min_conf = float(self.get_parameter("object_flow_min_conf").value)
        self.declare_parameter("local_anchor_object_flow_min_conf", 0.15)
        self.local_anchor_object_flow_min_conf = float(
            self.get_parameter("local_anchor_object_flow_min_conf").value
        )
        self.declare_parameter("object_flow_stale_sec", 0.5)   # LK VO wakes up if flow older than this
        self.object_flow_stale_sec = float(self.get_parameter("object_flow_stale_sec").value)
        self.declare_parameter("object_flow_trans", True)      # also apply flow translation (else yaw only)
        self.object_flow_trans = bool(self.get_parameter("object_flow_trans").value)
        self.declare_parameter("imu_flow_consistency_enabled", False)
        self.imu_flow_consistency_enabled = bool(
            self.get_parameter("imu_flow_consistency_enabled").value
        )
        self.declare_parameter("imu_flow_min_gain", 0.10)
        self.imu_flow_min_gain = self._bounded_gain(
            float(self.get_parameter("imu_flow_min_gain").value)
        )
        self.declare_parameter("imu_flow_full_trust_error_mps", 0.15)
        self.imu_flow_full_trust_error = max(
            0.0, float(self.get_parameter("imu_flow_full_trust_error_mps").value)
        )
        self.declare_parameter("imu_flow_soft_error_mps", 0.40)
        self.imu_flow_soft_error = max(
            self.imu_flow_full_trust_error + 1e-6,
            float(self.get_parameter("imu_flow_soft_error_mps").value),
        )
        self.declare_parameter("imu_flow_accel_deadband_mps2", 0.20)
        self.imu_flow_accel_deadband = max(
            0.0, float(self.get_parameter("imu_flow_accel_deadband_mps2").value)
        )
        self.declare_parameter("imu_flow_accel_clip_mps2", 1.50)
        self.imu_flow_accel_clip = max(
            0.0, float(self.get_parameter("imu_flow_accel_clip_mps2").value)
        )
        self.declare_parameter("imu_flow_accel_bias_alpha", 0.01)
        self.imu_flow_accel_bias_alpha = self._bounded_gain(
            float(self.get_parameter("imu_flow_accel_bias_alpha").value)
        )
        self.declare_parameter("imu_flow_accel_filter_alpha", 0.25)
        self.imu_flow_accel_filter_alpha = self._bounded_gain(
            float(self.get_parameter("imu_flow_accel_filter_alpha").value)
        )
        self.declare_parameter("imu_flow_max_dt_sec", 0.30)
        self.imu_flow_max_dt = max(
            0.0, float(self.get_parameter("imu_flow_max_dt_sec").value)
        )
        self.last_objflow_time = None
        # IMU (MPU6050 gyro) is the TOP-PRIORITY heading source: the Z gyro rate is low-noise and only
        # drifts slowly (bias, removed at startup + corrected absolutely by landmarks), so integrating
        # it gives a far steadier heading than object-flow/LK-VO. When IMU is fresh EVERYTHING else is
        # suppressed for yaw (object-flow keeps only translation). Landmark correction still trims the
        # slow residual bias absolutely.
        self.declare_parameter("use_imu_heading", True)
        self.use_imu = bool(self.get_parameter("use_imu_heading").value)
        self.declare_parameter("imu_stale_sec", 0.3)
        self.imu_stale_sec = float(self.get_parameter("imu_stale_sec").value)
        self.declare_parameter("imu_gyro_deadband_rad", 0.01)   # ~0.6 deg/s: below = 0 (kill bias walk)
        self.imu_gyro_deadband = float(self.get_parameter("imu_gyro_deadband_rad").value)
        # IMU motion smoothing does NOT integrate accel into x/y. It only classifies the short-term
        # body state and attenuates suspicious translation corrections from vision/wall landmarks.
        self.declare_parameter("imu_motion_smoothing_enabled", False)
        self.imu_motion_smoothing_enabled = bool(
            self.get_parameter("imu_motion_smoothing_enabled").value
        )
        self.declare_parameter("imu_motion_publish_debug", True)
        self.imu_motion_publish_debug = bool(
            self.get_parameter("imu_motion_publish_debug").value
        )
        self.declare_parameter("imu_motion_stale_sec", 0.30)
        self.imu_motion_stale_sec = float(self.get_parameter("imu_motion_stale_sec").value)
        self.declare_parameter("imu_motion_accel_window", 12)
        self.imu_motion_accel_window = max(
            3, int(self.get_parameter("imu_motion_accel_window").value)
        )
        self.declare_parameter("imu_accel_norm_ref", 9.80665)
        self.imu_accel_norm_ref = float(self.get_parameter("imu_accel_norm_ref").value)
        self.declare_parameter("imu_stationary_gyro_eps_radps", 0.035)
        self.imu_stationary_gyro_eps = float(
            self.get_parameter("imu_stationary_gyro_eps_radps").value
        )
        self.declare_parameter("imu_stationary_accel_norm_eps", 0.35)
        self.imu_stationary_accel_norm_eps = float(
            self.get_parameter("imu_stationary_accel_norm_eps").value
        )
        self.declare_parameter("imu_stationary_accel_var_eps", 0.10)
        self.imu_stationary_accel_var_eps = float(
            self.get_parameter("imu_stationary_accel_var_eps").value
        )
        self.declare_parameter("imu_rotating_gyro_eps_radps", 0.18)
        self.imu_rotating_gyro_eps = float(
            self.get_parameter("imu_rotating_gyro_eps_radps").value
        )
        self.declare_parameter("imu_impact_accel_norm_eps", 1.50)
        self.imu_impact_accel_norm_eps = float(
            self.get_parameter("imu_impact_accel_norm_eps").value
        )
        self.declare_parameter("imu_impact_accel_delta_eps", 1.20)
        self.imu_impact_accel_delta_eps = float(
            self.get_parameter("imu_impact_accel_delta_eps").value
        )
        self.declare_parameter("imu_object_flow_stationary_gain", 0.10)
        self.imu_object_flow_stationary_gain = float(
            self.get_parameter("imu_object_flow_stationary_gain").value
        )
        self.declare_parameter("imu_object_flow_rotating_trans_gain", 0.30)
        self.imu_object_flow_rotating_trans_gain = float(
            self.get_parameter("imu_object_flow_rotating_trans_gain").value
        )
        self.declare_parameter("imu_object_flow_impact_gain", 0.20)
        self.imu_object_flow_impact_gain = float(
            self.get_parameter("imu_object_flow_impact_gain").value
        )
        self.declare_parameter("imu_landmark_impact_gain", 0.40)
        self.imu_landmark_impact_gain = float(
            self.get_parameter("imu_landmark_impact_gain").value
        )
        self.declare_parameter("imu_wall_rotating_gain", 0.60)
        self.imu_wall_rotating_gain = float(self.get_parameter("imu_wall_rotating_gain").value)
        self.declare_parameter("imu_wall_impact_gain", 0.50)
        self.imu_wall_impact_gain = float(self.get_parameter("imu_wall_impact_gain").value)
        self.last_imu_time = None
        self.imu_motion_state = "UNKNOWN"
        self.imu_motion_state_time = None
        self.imu_accel_norm_var = 0.0
        self._imu_last_accel_norm = None
        self._imu_accel_norm_window = deque(maxlen=self.imu_motion_accel_window)
        self._imu_flow_accel_bias_xy: Optional[np.ndarray] = None
        self._imu_flow_accel_xy: Optional[np.ndarray] = None
        self._imu_flow_velocity_xy: Optional[np.ndarray] = None
        self._imu_flow_last_time: Optional[float] = None
        self._imu_flow_last_heading: Optional[float] = None
        self.last_wall_correction_time = None
        self.wall_fast_correction = False
        # Mission-owned gate. Heading correction is enabled only during the post-opening
        # stationary alignment; normal driving remains translation-only for this rollout.
        self.wall_correction_mode = "TRANSLATION_ONLY"

        self.fx = float(self.get_parameter("top_fx").value)
        self.fy = float(self.get_parameter("top_fy").value)
        self.cx = float(self.get_parameter("top_cx").value)
        self.cy = float(self.get_parameter("top_cy").value)
        self.dist_coeffs = np.array(
            [float(c) for c in self.get_parameter("dist_coeffs").value], dtype=np.float64
        )
        # Intrinsics are only "valid" when all four primary terms are non-zero.
        self.have_intrinsics = all(v != 0.0 for v in (self.fx, self.fy, self.cx, self.cy))
        if self.have_intrinsics:
            self.camera_matrix = np.array(
                [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
        else:
            self.camera_matrix = None

        # Fisheye rectify: needs 4 coeffs; maps are built lazily on the first frame (need size).
        self.use_fisheye = bool(self.get_parameter("fisheye_model").value)
        if self.use_fisheye and self.dist_coeffs.size < 4:
            self.get_logger().error("fisheye_model set but <4 dist_coeffs; disabling fisheye")
            self.use_fisheye = False
        self._fish_D = self.dist_coeffs[:4].reshape(4, 1) if self.use_fisheye else None
        self._fish_map = None  # (map1, map2), built on first frame

        # Wheel-odometry integration state.
        self.last_odom_time: Optional[float] = None  # wall-clock seconds of last wheel msg
        self.last_wheel_cmd_time: Optional[float] = None
        self.last_wheel_cmd = [0.0, 0.0, 0.0, 0.0]
        self.last_wheel_cmd_body: Optional[np.ndarray] = None
        self.last_wheel_odom_body: Optional[np.ndarray] = None
        self.stationary_since: Optional[float] = None
        self.is_stationary = False

        # Visual-odometry state.
        self.bridge = CvBridge()
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_pts: Optional[np.ndarray] = None

        # tf broadcaster (optional).
        self.tf_broadcaster = None
        if self.broadcast_tf and TF2_AVAILABLE:
            self.tf_broadcaster = TransformBroadcaster(self)
        elif self.broadcast_tf and not TF2_AVAILABLE:
            self.get_logger().error("broadcast_tf requested but tf2_ros unavailable; tf disabled")

        if self.use_wheel_odom or self.encoder_motion_constraint_enabled:
            self.create_subscription(Float32MultiArray, "/base/wheel_odom", self.on_wheel_odom, 10)
        self.create_subscription(Float32MultiArray, "/base/wheel_speeds", self.on_wheel_cmd, 10)
        if self.use_imu:
            self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera_top/image_raw", self.on_top_image,
                                 qos_profile_sensor_data)  # match camera BEST_EFFORT
        self.create_subscription(
            Float32MultiArray, "/localization/landmark_correction", self.on_landmark_correction, 10
        )
        self.create_subscription(
            Float32MultiArray, "/localization/wall_anchor_correction", self.on_wall_anchor_correction, 10
        )
        self.create_subscription(
            Float32MultiArray, "/localization/wall_field_correction", self.on_wall_field_correction, 10
        )
        self.create_subscription(
            Bool, "/localization/wall_fast_correction", self.on_wall_fast_correction, 10
        )
        self.create_subscription(
            String, "/localization/wall_correction_mode", self.on_wall_correction_mode, 10
        )
        self.create_subscription(
            PoseStamped, "/localization/reset_pose", self.on_reset_pose, 10
        )
        self.create_subscription(
            Float32MultiArray, "/localization/object_odom", self.on_object_odom, 10
        )
        self._mission_state = ""
        self.create_subscription(
            MissionState, "/mission_state", self.on_mission_state, 10
        )
        self.pub = self.create_publisher(PoseStamped, "/localization/pose", 10)
        self.pub_stationary = self.create_publisher(Bool, "/localization/is_stationary", 10)
        self.pub_motion_mode = self.create_publisher(String, "/localization/motion_mode", 10)
        self.pub_imu_motion_state = self.create_publisher(
            String, "/localization/imu_motion_state", 10
        )
        self.pub_wall_map_transform = self.create_publisher(
            Float32MultiArray, "/localization/wall_map_transform", 10
        )
        self.pub_wall_heading_debug = self.create_publisher(
            Float32MultiArray, "/localization/wall_heading_debug", 10
        )
        self.pub_imu_yaw_delta = self.create_publisher(
            Float32, "/localization/imu_yaw_delta", 10
        )
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        self.get_logger().info(
            f"localizer start=({self.x:.2f},{self.y:.2f},{self.theta:.2f}) "
            f"lx={self.lx} ly={self.ly} vo={self.use_vo} landmark={self.use_landmark_correction} "
            f"obj_landmarks={self.use_object_landmarks} tf={self.tf_broadcaster is not None} "
            f"wheel_odom_enabled={self.wheel_odom_enabled} "
            f"encoder_constraint={self.encoder_motion_constraint_enabled} "
            f"imu_smoothing={self.imu_motion_smoothing_enabled} "
            f"imu_flow_consistency={self.imu_flow_consistency_enabled} "
            f"intrinsics={'set' if self.have_intrinsics else 'unset'} "
            f"fisheye={self.use_fisheye} rate={rate}Hz"
        )

    # ------------------------------------------------------------------ wheel odom
    def on_reset_pose(self, msg: PoseStamped) -> None:
        """Atomically establish a trusted field pose after the scripted opening."""
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        # An all-zero quaternion explicitly requests an x/y-only reset. This lets the opening
        # establish its trusted endpoint without lying about the robot's measured heading.
        theta = self.theta if q_norm_sq < 1e-12 else math.atan2(siny_cosp, cosy_cosp)
        if not all(math.isfinite(value) for value in (x, y, theta)):
            self.get_logger().warn("ignored non-finite localization reset pose")
            return
        self.x = x
        self.y = y
        self.theta = wrap_angle(theta)
        self.last_wall_correction_time = None
        self.last_landmark_correction_time = None
        self._reset_imu_flow_consistency()
        self.get_logger().info(
            f"localization pose reset to ({self.x:.3f},{self.y:.3f},{self.theta:.3f})"
        )

    def on_wheel_cmd(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_wheel_cmd = [float(v) for v in msg.data[:4]]
            self.last_wheel_cmd_body = self._wheel_body_velocity(
                np.asarray(self.last_wheel_cmd, dtype=np.float64), measured=False
            )
            self.last_wheel_cmd_time = now
            odom_stale = (
                self.last_odom_time is None
                or (now - self.last_odom_time) > self.stationary_cmd_stale_sec
            )
            if odom_stale:
                if self._command_active(now):
                    self._mark_not_stationary()
                else:
                    self._update_stationary(now, np.zeros(4, dtype=np.float64))

    def on_wheel_odom(self, msg: Float32MultiArray) -> None:
        """Record encoder motion, optionally integrating it only when wheel odom is enabled."""
        if len(msg.data) < 4:
            self.get_logger().warn(
                f"wheel_odom expected 4 values, got {len(msg.data)}", throttle_duration_sec=5.0
            )
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        wheels = np.array([float(msg.data[i]) for i in range(4)], dtype=np.float64)
        if self.wheel_odom_deadband > 0.0:
            wheels[np.abs(wheels) < self.wheel_odom_deadband] = 0.0
        self._update_stationary(now, wheels)
        if int(self._wheel_mask.sum()) < 3:
            return
        self.last_wheel_odom_body = self._wheel_body_velocity(wheels, measured=True)
        if self.last_odom_time is None:
            # First sample only establishes a timestamp; no integration yet.
            self.last_odom_time = now
            return
        dt = now - self.last_odom_time
        self.last_odom_time = now
        # Clamp dt to a sane window so a stale/jumpy clock cannot teleport the pose.
        dt = max(0.0, min(self.wheel_odom_max_dt, dt))
        if dt <= 0.0:
            return
        # When encoder odometry is disabled the encoder message is still intentionally consumed for
        # the motion constraint above, but it contributes no position integration here.
        if not self.use_wheel_odom:
            return

        # Mecanum forward kinematics. With all 4 wheels this is the exact inverse of
        # base_controller IK; with one disabled encoder it becomes a 3-equation least-squares solve.
        vx, vy, w = self.last_wheel_odom_body

        # Body velocity -> field velocity using the current heading.
        ct, st = math.cos(self.theta), math.sin(self.theta)
        vxw = vx * ct - vy * st
        vyw = vx * st + vy * ct

        # Translation: object-flow OWNS it when fresh (grounded in real objects), so the wheel
        # dead-reckons x/y only as a FALLBACK — otherwise the same displacement is integrated twice
        # (wheel here AND object-flow in on_object_odom) and the pose runs at ~2x speed. This mirrors
        # the yaw dedup below. LK VO gives no translation (monocular scale is ambiguous), so this
        # keys on object-flow freshness specifically (last_objflow_time), and only when object-flow
        # translation is actually enabled (object_flow_trans).
        flow_trans_fresh = (
            self.use_object_flow
            and self.object_flow_trans
            and self.last_objflow_time is not None
            and (now - self.last_objflow_time) <= self.object_flow_stale_sec
        )
        # Stationary protection freezes only translation.  Rotation is still allowed to come
        # from the IMU (or the wheel fallback), so a manually rotated robot changes its map arrow.
        if not flow_trans_fresh and not (self.freeze_pose_when_stationary and self.is_stationary):
            self.x += vxw * dt
            self.y += vyw * dt
        # Rotation: IMU > VO > wheel. Only integrate the (open-loop, drift-prone) wheel yaw when BOTH
        # the IMU and VO are stale/absent — so driving never double-counts rotation.
        if not self._imu_fresh(now) and (self.last_vo_time is None
                                         or (now - self.last_vo_time) > self.vo_stale_sec):
            self.theta = wrap_angle(self.theta + w * dt)

    def _wheel_body_velocity(self, wheels: np.ndarray, *, measured: bool) -> np.ndarray:
        """Solve mecanum body velocity from commanded or measured wheel linear speeds."""
        rows = self._wheel_rows if measured else self._wheel_rows_all
        values = wheels[self._wheel_mask] if measured else wheels
        return np.linalg.lstsq(rows, values, rcond=None)[0]

    def _classify_motion(self, body: np.ndarray) -> str:
        """Classify a body velocity as STOP/FORWARD/LATERAL/ROTATE/MIXED."""
        vx, vy, wz = (float(v) for v in body)
        linear = math.hypot(vx, vy)
        angular_tangent = abs(wz) * self.k
        if linear < self.encoder_constraint_linear_eps and abs(wz) < self.encoder_constraint_angular_eps:
            return "STOP"
        if angular_tangent >= self.encoder_constraint_axis_ratio * max(linear, 1e-6):
            return "ROTATE"
        if abs(vx) >= self.encoder_constraint_axis_ratio * max(abs(vy), 1e-6):
            return "FORWARD"
        if abs(vy) >= self.encoder_constraint_axis_ratio * max(abs(vx), 1e-6):
            return "LATERAL"
        return "MIXED"

    def _motion_vectors_agree(self, command: np.ndarray, measured: np.ndarray) -> bool:
        """Return true only when command and encoder motion point in the same body direction."""
        cmd = np.array([command[0], command[1], command[2] * self.k], dtype=np.float64)
        odom = np.array([measured[0], measured[1], measured[2] * self.k], dtype=np.float64)
        cmd_norm = float(np.linalg.norm(cmd))
        odom_norm = float(np.linalg.norm(odom))
        if cmd_norm < 1e-6 or odom_norm < 1e-6:
            return cmd_norm < 1e-6 and odom_norm < 1e-6
        return float(np.dot(cmd, odom)) > 0.0

    def _motion_mode(self, now: float) -> str:
        """Use fresh encoder motion as truth; commands alone never prove the robot moved."""
        command = None
        if (
            self.last_wheel_cmd_body is not None
            and self.last_wheel_cmd_time is not None
            and (now - self.last_wheel_cmd_time) <= self.encoder_constraint_cmd_stale_sec
        ):
            command = self.last_wheel_cmd_body
        measured = None
        if (
            self.last_wheel_odom_body is not None
            and self.last_odom_time is not None
            and (now - self.last_odom_time) <= self.encoder_constraint_odom_stale_sec
        ):
            measured = self.last_wheel_odom_body

        if command is None and measured is None:
            return "UNKNOWN"
        if command is None:
            return self._classify_motion(measured)
        if measured is None:
            return "UNKNOWN"

        command_mode = self._classify_motion(command)
        measured_mode = self._classify_motion(measured)
        if measured_mode == "STOP":
            return "STOP"
        if (
            command_mode == measured_mode
            and command_mode != "MIXED"
            and self._motion_vectors_agree(command, measured)
        ):
            return measured_mode
        # Command/encoder disagreement usually means slip or a stalled base. Trust the encoder so
        # command-only lateral avoidance cannot drag the map while the chassis is physically stuck.
        return measured_mode

    @staticmethod
    def _bounded_gain(value: float) -> float:
        return max(0.0, min(1.0, value))

    def _motion_constraint_gains(self, source: str) -> Tuple[float, float, float]:
        """Return robot-frame forward/lateral/yaw gains for the present motion mode."""
        if not self.encoder_motion_constraint_enabled:
            return 1.0, 1.0, 1.0

        now = self.get_clock().now().nanoseconds * 1e-9
        mode = self._motion_mode(now)
        if mode == "STOP":
            forward = lateral = self.encoder_constraint_stop_translation_gain
            yaw = self.encoder_constraint_stop_yaw_gain
        elif mode == "FORWARD":
            forward = self.encoder_constraint_forward_gain
            lateral = self.encoder_constraint_forward_lateral_gain
            yaw = self.encoder_constraint_forward_yaw_gain
        elif mode == "LATERAL":
            forward = self.encoder_constraint_lateral_forward_gain
            lateral = self.encoder_constraint_lateral_gain
            yaw = self.encoder_constraint_lateral_yaw_gain
        elif mode == "ROTATE":
            forward = lateral = self.encoder_constraint_rotate_translation_gain
            yaw = self.encoder_constraint_rotate_yaw_gain
        elif mode == "MIXED":
            forward = lateral = self.encoder_constraint_mixed_translation_gain
            yaw = self.encoder_constraint_mixed_yaw_gain
        else:
            # Missing/stale encoder is not the same as a measured stop. If a fresh drive command is
            # active, fail open to the legacy visual/landmark behavior so the map does not freeze
            # while the robot is physically moving. A fresh encoder sample of zero still returns STOP.
            if self._command_active(now):
                forward = lateral = yaw = 1.0
            else:
                forward = lateral = self.encoder_constraint_stop_translation_gain
                yaw = self.encoder_constraint_stop_yaw_gain

        # The wall is an independent absolute reference. Keep a limited correction path open even
        # while the drive mode is STOP, otherwise the pose can never settle against a good wall fix.
        if source.startswith("wall"):
            forward = max(forward, self.encoder_constraint_wall_translation_floor)
            lateral = max(lateral, self.encoder_constraint_wall_translation_floor)
            yaw = max(yaw, self.encoder_constraint_wall_yaw_floor)
        return (
            self._bounded_gain(forward),
            self._bounded_gain(lateral),
            self._bounded_gain(yaw),
        )

    def _constrain_robot_delta(
        self, dfwd: float, dleft: float, dtheta: float, source: str
    ) -> Tuple[float, float, float]:
        """Attenuate a robot-frame pose delta according to encoder-derived motion mode."""
        forward_gain, lateral_gain, yaw_gain = self._motion_constraint_gains(source)
        # Gyro yaw is a direct physical measurement and must not be discarded while the slower
        # encoder state still reports STOP at the start of a short rotation pulse. Encoder-derived
        # attenuation remains active for camera and landmark corrections.
        if source == "imu":
            yaw_gain = 1.0
        return dfwd * forward_gain, dleft * lateral_gain, dtheta * yaw_gain

    def _constrain_world_delta(
        self, dx: float, dy: float, dtheta: float, source: str
    ) -> Tuple[float, float, float]:
        """Apply the same constraint to a field-frame correction via the current robot heading."""
        ct, st = math.cos(self.theta), math.sin(self.theta)
        dfwd = dx * ct + dy * st
        dleft = -dx * st + dy * ct
        dfwd, dleft, dtheta = self._constrain_robot_delta(dfwd, dleft, dtheta, source)
        return dfwd * ct - dleft * st, dfwd * st + dleft * ct, dtheta

    def _update_stationary(self, now: float, wheels: np.ndarray) -> None:
        enabled_wheels = wheels[self._wheel_mask] if int(self._wheel_mask.sum()) else wheels
        wheel_still = bool(
            enabled_wheels.size == 0
            or np.max(np.abs(enabled_wheels)) <= self.stationary_wheel_eps
        )
        # Stationary is based on actual encoder motion. A nonzero command with zero/stale encoder
        # means the base is stalled, so visual pose corrections should still be treated as stationary.
        currently_still = wheel_still
        if currently_still:
            if self.stationary_since is None:
                self.stationary_since = now
            self.is_stationary = (now - self.stationary_since) >= self.stationary_required_sec
        else:
            self.stationary_since = None
            self.is_stationary = False

    def _command_active(self, now: float) -> bool:
        if self.last_wheel_cmd_body is None or self.last_wheel_cmd_time is None:
            return False
        if (now - self.last_wheel_cmd_time) > self.stationary_cmd_stale_sec:
            return False
        vx, vy, wz = (float(v) for v in self.last_wheel_cmd_body)
        return max(math.hypot(vx, vy), abs(wz) * self.k) > self.stationary_cmd_eps

    def _mark_not_stationary(self) -> None:
        self.stationary_since = None
        self.is_stationary = False

    # --------------------------------------------------------------------- IMU heading
    def _imu_fresh(self, now: float) -> bool:
        return (self.use_imu and self.last_imu_time is not None
                and (now - self.last_imu_time) <= self.imu_stale_sec)

    def _reset_imu_flow_consistency(self) -> None:
        self._imu_flow_velocity_xy = None
        self._imu_flow_last_time = None
        self._imu_flow_last_heading = None

    def _update_imu_flow_acceleration(self, msg: Imu) -> None:
        """Remove slow planar bias and retain only bounded, short-term acceleration."""
        raw = np.array(
            [float(msg.linear_acceleration.x), float(msg.linear_acceleration.y)],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(raw)):
            return
        if self._imu_flow_accel_bias_xy is None:
            self._imu_flow_accel_bias_xy = raw.copy()
            self._imu_flow_accel_xy = np.zeros(2, dtype=np.float64)
            return

        bias_alpha = self.imu_flow_accel_bias_alpha
        self._imu_flow_accel_bias_xy += bias_alpha * (
            raw - self._imu_flow_accel_bias_xy
        )
        dynamic = raw - self._imu_flow_accel_bias_xy
        magnitude = float(np.linalg.norm(dynamic))
        if magnitude <= self.imu_flow_accel_deadband:
            dynamic[:] = 0.0
        elif magnitude > 1e-9:
            dynamic *= (magnitude - self.imu_flow_accel_deadband) / magnitude
        clipped_magnitude = float(np.linalg.norm(dynamic))
        if self.imu_flow_accel_clip > 0.0 and clipped_magnitude > self.imu_flow_accel_clip:
            dynamic *= self.imu_flow_accel_clip / clipped_magnitude

        filter_alpha = self.imu_flow_accel_filter_alpha
        if self._imu_flow_accel_xy is None:
            self._imu_flow_accel_xy = dynamic
        else:
            self._imu_flow_accel_xy += filter_alpha * (
                dynamic - self._imu_flow_accel_xy
            )

    def _filter_object_flow_translation(
        self, dfwd: float, dleft: float, now: float
    ) -> Tuple[float, float, float]:
        """Softly attenuate implausible flow speed changes without dropping a frame."""
        if not self.imu_flow_consistency_enabled:
            return float(dfwd), float(dleft), 1.0

        previous_time = self._imu_flow_last_time
        previous_heading = self._imu_flow_last_heading
        self._imu_flow_last_time = float(now)
        self._imu_flow_last_heading = float(self.theta)
        if previous_time is None:
            return float(dfwd), float(dleft), 1.0

        dt = float(now) - float(previous_time)
        if dt <= 0.0 or (self.imu_flow_max_dt > 0.0 and dt > self.imu_flow_max_dt):
            self._imu_flow_velocity_xy = None
            return float(dfwd), float(dleft), 1.0

        flow_velocity = np.array([dfwd / dt, dleft / dt], dtype=np.float64)
        if (
            self._imu_flow_velocity_xy is None
            or self._imu_flow_accel_xy is None
            or not self._imu_fresh(now)
        ):
            self._imu_flow_velocity_xy = flow_velocity
            return float(dfwd), float(dleft), 1.0

        expected = self._imu_flow_velocity_xy.copy()
        if previous_heading is not None:
            heading_delta = wrap_angle(float(self.theta) - float(previous_heading))
            c, s = math.cos(heading_delta), math.sin(heading_delta)
            expected = np.array(
                [c * expected[0] + s * expected[1],
                 -s * expected[0] + c * expected[1]],
                dtype=np.float64,
            )
        expected += self._imu_flow_accel_xy * dt
        blended, gain, error = blend_flow_velocity_with_imu(
            flow_velocity,
            expected,
            self.imu_flow_min_gain,
            self.imu_flow_full_trust_error,
            self.imu_flow_soft_error,
        )
        self._imu_flow_velocity_xy = blended
        filtered_dfwd, filtered_dleft = (float(value * dt) for value in blended)
        if gain < 0.999:
            self.get_logger().info(
                f"IMU-flow consistency gain={gain:.2f} speed_error={error:.2f}m/s "
                f"raw=({dfwd*100:+.1f},{dleft*100:+.1f})cm "
                f"used=({filtered_dfwd*100:+.1f},{filtered_dleft*100:+.1f})cm",
                throttle_duration_sec=0.5,
            )
        return filtered_dfwd, filtered_dleft, gain

    def _imu_motion_fresh(self, now: float) -> bool:
        return (
            self.imu_motion_smoothing_enabled
            and self.imu_motion_state_time is not None
            and (now - self.imu_motion_state_time) <= self.imu_motion_stale_sec
        )

    def _update_imu_motion_state(self, msg: Imu, now: float) -> None:
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        accel_delta = 0.0
        if self._imu_last_accel_norm is not None:
            accel_delta = abs(accel_norm - float(self._imu_last_accel_norm))
        self._imu_last_accel_norm = accel_norm
        self._imu_accel_norm_window.append(accel_norm)
        if len(self._imu_accel_norm_window) >= 3:
            self.imu_accel_norm_var = float(np.var(self._imu_accel_norm_window))
        else:
            self.imu_accel_norm_var = 0.0

        wz = abs(float(msg.angular_velocity.z))
        norm_err = abs(accel_norm - self.imu_accel_norm_ref)
        impact = (
            norm_err >= self.imu_impact_accel_norm_eps
            or accel_delta >= self.imu_impact_accel_delta_eps
        )
        stable_gravity = (
            wz <= self.imu_stationary_gyro_eps
            and norm_err <= self.imu_stationary_accel_norm_eps
            and self.imu_accel_norm_var <= self.imu_stationary_accel_var_eps
        )
        if impact:
            state = "IMPACT_OR_SLIP"
        elif wz >= self.imu_rotating_gyro_eps:
            state = "ROTATING"
        elif stable_gravity and self.is_stationary:
            state = "STOP"
        else:
            state = "MOVING_SMOOTH"

        self.imu_motion_state = state
        self.imu_motion_state_time = now
        if self.imu_motion_publish_debug:
            debug = String()
            debug.data = state
            self.pub_imu_motion_state.publish(debug)

    def _imu_translation_gain(self, source: str) -> float:
        if not self.imu_motion_smoothing_enabled:
            return 1.0
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._imu_motion_fresh(now):
            return 1.0

        state = self.imu_motion_state
        if source == "object_flow":
            if state == "STOP":
                return self._bounded_gain(self.imu_object_flow_stationary_gain)
            if state == "ROTATING":
                return self._bounded_gain(self.imu_object_flow_rotating_trans_gain)
            if state == "IMPACT_OR_SLIP":
                return self._bounded_gain(self.imu_object_flow_impact_gain)
        elif source == "object_landmark":
            if state == "IMPACT_OR_SLIP":
                return self._bounded_gain(self.imu_landmark_impact_gain)
        elif source.startswith("wall"):
            if state == "ROTATING":
                return self._bounded_gain(self.imu_wall_rotating_gain)
            if state == "IMPACT_OR_SLIP":
                return self._bounded_gain(self.imu_wall_impact_gain)
        return 1.0

    def on_imu(self, msg: Imu) -> None:
        """TOP-PRIORITY heading: integrate the bias-removed yaw-rate gyro. Low noise + slow bias, so
        heading stays steady (no object-flow churn / LK-VO hallucination). When this is fresh the
        object-flow / LK-VO / wheel yaw are all suppressed so nothing double-counts the rotation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.imu_flow_consistency_enabled:
            self._update_imu_flow_acceleration(msg)
        if self.imu_motion_smoothing_enabled:
            self._update_imu_motion_state(msg, now)
        wz = float(msg.angular_velocity.z)
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if 0.0 < dt < 0.2 and abs(wz) >= self.imu_gyro_deadband:   # deadband kills stationary walk
                physical_dtheta = wz * dt
                self.pub_imu_yaw_delta.publish(Float32(data=float(physical_dtheta)))
                _, _, dtheta = self._constrain_robot_delta(
                    0.0, 0.0, physical_dtheta, "imu"
                )
                self.theta = wrap_angle(self.theta + dtheta)
        self.last_imu_time = now

    # ------------------------------------------------------------- object-flow odometry
    def on_mission_state(self, msg: MissionState) -> None:
        """Track the mission mode used by the state-scoped flow confidence gate."""
        self._mission_state = str(msg.state).strip().upper()

    def on_object_odom(self, msg: Float32MultiArray) -> None:
        """PRIMARY yaw source: per-frame robot motion from wide-cam object points
        [dtheta, dfwd, dleft, conf] (robot frame). Grounded in real objects, so it stays ~0 when
        the robot is still (even under motor vibration) — unlike LK flow. Integrated in full."""
        if not self.use_object_flow:
            return
        d = msg.data
        if len(d) < 3:
            return
        dtheta, dfwd, dleft = float(d[0]), float(d[1]), float(d[2])
        conf = float(d[3]) if len(d) > 3 else 1.0
        min_conf = object_flow_min_conf_for_state(
            self._mission_state,
            self.object_flow_min_conf,
            self.local_anchor_object_flow_min_conf,
        )
        if conf < min_conf:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            not self.imu_flow_consistency_enabled
            and self.suppress_object_flow_when_stationary
            and self.is_stationary
        ):
            self.last_objflow_time = now
            self.last_vo_time = now
            # Keep a credible rotation measurement even while translation is frozen. IMU has
            # priority when fresh, so this is only a fallback for robots without IMU data.
            if not self._imu_fresh(now) and abs(dtheta) >= self.vo_deadband:
                _, _, dtheta = self._constrain_robot_delta(0.0, 0.0, dtheta, "object_flow")
                self.theta = wrap_angle(self.theta + dtheta)
            return
        self.last_objflow_time = now
        self.last_vo_time = now      # object-flow owns rotation -> keep wheel-yaw AND LK-VO suppressed
        dfwd, dleft, dtheta = self._constrain_robot_delta(dfwd, dleft, dtheta, "object_flow")
        dfwd, dleft, _ = self._filter_object_flow_translation(dfwd, dleft, now)
        trans_gain = (
            1.0
            if self.imu_flow_consistency_enabled
            else self._imu_translation_gain("object_flow")
        )
        dfwd *= trans_gain
        dleft *= trans_gain
        if not self._imu_fresh(now):  # IMU gyro outranks object-flow for yaw (steadier)
            self.theta = wrap_angle(self.theta + dtheta)
        if self.object_flow_trans:
            ct, st = math.cos(self.theta), math.sin(self.theta)
            self.x += dfwd * ct - dleft * st
            self.y += dfwd * st + dleft * ct

    # ------------------------------------------------------------- object landmarks
    def on_landmark_correction(self, msg: Float32MultiArray) -> None:
        """Apply an absolute landmark drift estimate [dx, dy, dtheta, confidence].

        Producers include the world model's locked object anchors and the arena wall/floor localizer.
        Translation is nudged gently. HEADING is treated as an ABSOLUTE reference scaled by the
        fit's confidence, with per-update clamps for stability.
        """
        if not (self.use_object_landmarks or self.use_landmark_correction):
            return
        if self.freeze_pose_when_stationary and self.is_stationary:
            return
        d = msg.data
        if len(d) < 3:
            return
        dx, dy, dth = float(d[0]), float(d[1]), float(d[2])
        conf = float(d[3]) if len(d) > 3 else 1.0
        mx, mr = self.landmark_max_step_m, self.landmark_max_step_rad
        cf = max(0.0, min(1.0, conf))
        # POSITION: the locked anchors ARE the wide cam's (now 1280, cm-accurate) map, so this dx/dy
        # is an ABSOLUTE position fix. Apply most of it (confidence-scaled) to null out odometry
        # drift rather than crawl — the wide map is the trusted ground truth for where we are.
        g_xy = self.landmark_gain * cf
        applied_dx = max(-mx, min(mx, g_xy * dx))
        applied_dy = max(-mx, min(mx, g_xy * dy))
        trans_gain = self._imu_translation_gain("object_landmark")
        applied_dx *= trans_gain
        applied_dy *= trans_gain
        g_th = self.landmark_theta_gain * cf                        # confidence-scaled heading authority
        applied_dth = max(-mr, min(mr, g_th * dth))
        applied_dx, applied_dy, applied_dth = self._constrain_world_delta(
            applied_dx, applied_dy, applied_dth, "object_landmark"
        )
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_landmark_correction_time is None:
            dt = min(
                self.landmark_correction_max_dt_sec,
                self.landmark_correction_nominal_dt,
            )
        else:
            dt = max(
                0.0,
                min(
                    self.landmark_correction_max_dt_sec,
                    now - self.last_landmark_correction_time,
                ),
            )
        self.last_landmark_correction_time = now
        max_translation = self.landmark_correction_max_speed_mps * dt
        requested_translation = math.hypot(applied_dx, applied_dy)
        applied_dx, applied_dy, limited = limit_planar_delta(
            applied_dx, applied_dy, max_translation
        )
        if limited:
            self.get_logger().warn(
                f"landmark correction slew-limited: requested={requested_translation*100:.1f}cm "
                f"allowed={max_translation*100:.1f}cm dt={dt:.3f}s",
                throttle_duration_sec=1.0,
            )
        self.x += applied_dx
        self.y += applied_dy
        self.theta = wrap_angle(self.theta + applied_dth)

    def on_wall_anchor_correction(self, msg: Float32MultiArray) -> None:
        """Pull a stationary pose back toward the initial yellow-wall anchor."""
        if not self.freeze_pose_when_stationary or not self.is_stationary or len(msg.data) < 3:
            return
        dx, dy, dth = (float(msg.data[i]) for i in range(3))
        conf = max(0.0, min(1.0, float(msg.data[3]) if len(msg.data) > 3 else 1.0))
        if math.hypot(dx, dy) < self.wall_anchor_deadband_m and abs(dth) < math.radians(0.3):
            return
        gain = max(0.0, self.wall_anchor_gain) * conf
        trans_gain = self._imu_translation_gain("wall_anchor")
        applied_dx = -max(
            -self.wall_anchor_max_step_m,
            min(self.wall_anchor_max_step_m, gain * trans_gain * dx),
        )
        applied_dy = -max(
            -self.wall_anchor_max_step_m,
            min(self.wall_anchor_max_step_m, gain * trans_gain * dy),
        )
        applied_dth = -max(
            -self.wall_anchor_max_step_rad,
            min(self.wall_anchor_max_step_rad, gain * dth),
        )
        applied_dx, applied_dy, applied_dth = self._constrain_world_delta(
            applied_dx, applied_dy, applied_dth, "wall_anchor"
        )
        self.x += applied_dx
        self.y += applied_dy
        self.theta = wrap_angle(self.theta + applied_dth)

    def on_wall_fast_correction(self, msg: Bool) -> None:
        self.wall_fast_correction = bool(msg.data)

    def on_wall_correction_mode(self, msg: String) -> None:
        mode = str(msg.data).strip().upper()
        if mode in {"OFF", "HEADING_ONLY", "TRANSLATION_ONLY", "FULL"}:
            self.wall_correction_mode = mode

    def _wall_fast_active(self) -> bool:
        return bool(self.wall_fast_correction and self.is_stationary)

    def on_wall_field_correction(self, msg: Float32MultiArray) -> None:
        """Apply a mission-gated wall correction and publish its heading diagnostics."""
        self.last_wall_correction_time = self.get_clock().now().nanoseconds * 1e-9
        if len(msg.data) < 3:
            return
        dx, dy, dth = (float(msg.data[i]) for i in range(3))
        conf = max(0.0, min(1.0, float(msg.data[3]) if len(msg.data) > 3 else 1.0))
        raw_dth = float(msg.data[4]) if len(msg.data) > 4 else dth
        segment_count = float(msg.data[5]) if len(msg.data) > 5 else 0.0
        angle_variance = float(msg.data[6]) if len(msg.data) > 6 else math.nan
        heading_conf = max(
            0.0, min(1.0, float(msg.data[7]) if len(msg.data) > 7 else conf)
        )
        allow_heading = self.wall_correction_mode in {"HEADING_ONLY", "FULL"}
        allow_translation = self.wall_correction_mode in {"TRANSLATION_ONLY", "FULL"}
        # Translation was measured after rotating the observed lines by raw_dth. Never consume it
        # until the wall and field axes are already parallel, otherwise x/y is geometrically wrong.
        heading_converged = abs(raw_dth) <= self.wall_translation_heading_gate_rad
        allow_translation = allow_translation and heading_converged
        fast_active = self._wall_fast_active()
        gain_value = self.wall_field_fast_gain if fast_active else self.wall_field_gain
        theta_gain_value = (
            self.wall_field_fast_theta_gain
            if fast_active
            else self.wall_field_theta_gain
        )
        max_step_m = (
            self.wall_field_fast_max_step_m
            if fast_active
            else self.wall_field_max_step_m
        )
        max_step_rad = (
            self.wall_field_fast_max_step_rad
            if fast_active
            else self.wall_field_max_step_rad
        )
        trans_gain = self._imu_translation_gain("wall_field")
        gain = max(0.0, gain_value) * conf * trans_gain if allow_translation else 0.0
        theta_gain = max(0.0, theta_gain_value) * heading_conf if allow_heading else 0.0
        tx = max(-max_step_m, min(max_step_m, gain * dx)) if math.isfinite(dx) else 0.0
        ty = max(-max_step_m, min(max_step_m, gain * dy)) if math.isfinite(dy) else 0.0
        applied_dth = max(
            -max_step_rad,
            min(max_step_rad, theta_gain * dth),
        )

        debug = Float32MultiArray()
        # [raw dtheta, filtered dtheta, applied dtheta, segment count, variance(rad^2),
        #  heading confidence, heading enabled, translation enabled]
        debug.data = [
            float(raw_dth),
            float(dth),
            float(applied_dth),
            float(segment_count),
            float(angle_variance),
            float(heading_conf),
            1.0 if allow_heading else 0.0,
            1.0 if allow_translation else 0.0,
        ]
        self.pub_wall_heading_debug.publish(debug)

        if self.wall_correction_mode == "OFF":
            return
        if not allow_translation and not allow_heading:
            return
        correction_is_tiny = (
            math.hypot(tx, ty) < self.wall_field_deadband_m
            and abs(applied_dth) < math.radians(0.3)
        )
        if correction_is_tiny:
            return

        if self.wall_correction_mode == "HEADING_ONLY":
            # This is an orientation estimate update, not physical robot motion. Keep x/y and the
            # base_link-relative anchor grid untouched; mapping is disabled throughout this phase.
            self.theta = wrap_angle(self.theta + applied_dth)
            return

        ct, st = math.cos(applied_dth), math.sin(applied_dth)
        old_x, old_y = self.x, self.y
        raw_dx = ct * old_x - st * old_y + tx - old_x
        raw_dy = st * old_x + ct * old_y + ty - old_y
        raw_dx, raw_dy, applied_dth = self._constrain_world_delta(
            raw_dx, raw_dy, applied_dth, "wall_field"
        )
        self.x = old_x + raw_dx
        self.y = old_y + raw_dy
        self.theta = wrap_angle(self.theta + applied_dth)

        applied = Float32MultiArray()
        applied.data = [float(raw_dx), float(raw_dy), float(applied_dth)]
        self.pub_wall_map_transform.publish(applied)

    def _wall_correction_fresh(self, now: float) -> bool:
        return (
            self.last_wall_correction_time is not None
            and (now - self.last_wall_correction_time) <= self.wall_correction_stale_sec
        )

    # ----------------------------------------------------------------- top camera
    def on_top_image(self, msg: Image) -> None:
        """Visual-odometry heading correction + landmark absolute-correction hook."""
        if not self.use_vo:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - never let a bad frame kill the node
            self.get_logger().warn(f"cv_bridge failed: {exc}", throttle_duration_sec=5.0)
            return
        if frame is None or frame.size == 0:
            return

        # Object-flow is the primary yaw source; while it is fresh the LK VO result is discarded, so
        # skip the whole VO pipeline (fisheye remap + gray + LK + RANSAC) — it was running at camera
        # rate on the Orin for nothing and starving the 20 Hz pose timer / serial callbacks. Only skip
        # when the wall-cardinal landmark snap (which also needs gray) is off — which it is outside the
        # arena (test_field). prev_gray is invalidated so LK cleanly re-inits when object-flow goes stale.
        now = self.get_clock().now().nanoseconds * 1e-9
        flow_fresh = (self.use_object_flow and self.last_objflow_time is not None
                      and (now - self.last_objflow_time) <= self.object_flow_stale_sec)
        if flow_fresh and not self.use_landmark_correction:
            self.prev_gray = None
            self.prev_pts = None
            return

        # Optional undistort only when intrinsics are provided. Fisheye rectify for the wide
        # ~150 deg lens (straightens walls for Hough/LK); pinhole undistort otherwise.
        if self.have_intrinsics and np.any(self.dist_coeffs):
            if self.use_fisheye:
                frame = self._fisheye_rectify(frame)
            else:
                frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Frame-to-frame heading from LK-tracked features. DEMOTED to a fallback: object-flow (real
        # object points) is the primary yaw source and does not hallucinate rotation from vibration,
        # so LK VO only runs when object-flow has been stale (handled by the early-skip above; here
        # `now`/`flow_fresh` are already set). With landmark-snap on, we reach here even when fresh.
        theta_visual = self._visual_yaw_delta(gray)
        if theta_visual is not None and not flow_fresh:
            self.last_vo_time = now                          # LK-VO alive -> wheel yaw stays suppressed
            # IMU gyro outranks LK-VO for yaw (LK hallucinates rotation from vibration); apply LK yaw
            # only when the IMU is stale/absent.
            if (abs(theta_visual) >= self.vo_deadband and not self._imu_fresh(now)
                    and not (self.freeze_pose_when_stationary and self.is_stationary)):
                _, _, theta_visual = self._constrain_robot_delta(0.0, 0.0, theta_visual, "visual")
                self.theta = wrap_angle(self.theta + theta_visual)

        # Absolute-correction hook (approximate; see _detect_landmarks docstring). Skipped
        # outside the arena, where random edges would snap the heading to a wrong cardinal.
        if self.use_landmark_correction:
            landmark = self._detect_landmarks(gray)
            if landmark is not None:
                _, _, theta_land = landmark
                # Correct ONLY theta: blend the WRAPPED angular error toward the wall-aligned
                # heading (wrap-safe; linear angle averaging would jump near +/-pi).
                err = wrap_angle(theta_land - self.theta)
                _, _, dtheta = self._constrain_robot_delta(0.0, 0.0, 0.05 * err, "visual")
                self.theta = wrap_angle(self.theta + dtheta)

        self.prev_gray = gray

    def _fisheye_rectify(self, frame: np.ndarray) -> np.ndarray:
        """Rectify the wide ~150 deg fisheye to a virtual pinhole (balance=0 -> no black border).

        Maps are built once on the first frame (image size needed). Straightens wall edges so
        Hough/LK behave like a normal camera; this is for VO only (not metric projection).
        """
        h, w = frame.shape[:2]
        if self._fish_map is None:
            new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.camera_matrix, self._fish_D, (w, h), np.eye(3), balance=0.0
            )
            m1, m2 = cv2.fisheye.initUndistortRectifyMap(
                self.camera_matrix, self._fish_D, np.eye(3), new_k, (w, h), cv2.CV_16SC2
            )
            self._fish_map = (m1, m2)
        return cv2.remap(frame, self._fish_map[0], self._fish_map[1], cv2.INTER_LINEAR)

    def _visual_yaw_delta(self, gray: np.ndarray) -> Optional[float]:
        """Estimate frame-to-frame yaw (rad) about the image center via LK + affine.

        Returns None when there is no previous frame or too few inliers (<8). Sign of
        the yaw assumes a top-down/near-top-down camera looking at the floor; this is
        used only to nudge heading and is robust to the exact magnitude.
        """
        prev_gray = self.prev_gray
        if prev_gray is None or prev_gray.shape != gray.shape:
            self.prev_pts = None
            return None

        # (Re)detect features on the previous frame if we have none to track.
        if self.prev_pts is None or len(self.prev_pts) < 8:
            self.prev_pts = cv2.goodFeaturesToTrack(
                prev_gray, maxCorners=200, qualityLevel=0.01, minDistance=8
            )
        if self.prev_pts is None or len(self.prev_pts) < 8:
            return None

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, self.prev_pts, None)
        if next_pts is None or status is None:
            self.prev_pts = None
            return None

        status = status.reshape(-1)
        good_prev = self.prev_pts.reshape(-1, 2)[status == 1]
        good_next = next_pts.reshape(-1, 2)[status == 1]
        if len(good_prev) < 8:
            self.prev_pts = None
            return None

        # Partial-affine (rotation + uniform scale + translation) with RANSAC.
        affine, inliers = cv2.estimateAffinePartial2D(
            good_prev, good_next, method=cv2.RANSAC, ransacReprojThreshold=3.0
        )
        if affine is None or inliers is None or int(inliers.sum()) < 8:
            # Carry forward tracked points so the next frame can retry.
            self.prev_pts = good_next.reshape(-1, 1, 2)
            return None

        # Rotation angle from the 2x2 block: atan2(a10, a00).
        d_yaw = math.atan2(affine[1, 0], affine[0, 0])
        # Carry tracked points forward for continuity.
        self.prev_pts = good_next.reshape(-1, 1, 2)
        return wrap_angle(d_yaw)

    def _detect_landmarks(self, gray: np.ndarray) -> Optional[Tuple[float, float, float]]:
        """Absolute-correction HOOK from field landmarks (walls/corners/flag).

        Returns (x, y, theta) in the field frame, or None when nothing reliable is
        found. Default behaviour is None.

        Current (APPROXIMATE) implementation: find a dominant near-horizontal or
        near-vertical straight wall edge with Canny + HoughLinesP and, if present,
        snap heading to the nearest wall-aligned cardinal direction (multiple of
        pi/2). x and y are returned as the current estimate (NOT corrected here):
        proper absolute x/y correction needs camera extrinsics + field geometry and
        is intentionally left until calibration. Only theta should be consumed.
        """
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180.0, threshold=80, minLineLength=80, maxLineGap=10
        )
        if lines is None:
            return None

        # Pick the longest detected line as the dominant edge.
        best_len = 0.0
        best_angle: Optional[float] = None
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = math.hypot(x2 - x1, y2 - y1)
            if length > best_len:
                best_len = length
                best_angle = math.atan2(y2 - y1, x2 - x1)
        if best_angle is None:
            return None

        # Only act on edges that are clearly near-horizontal or near-vertical (<=12 deg off).
        tol = math.radians(12.0)
        nearest_axis = round(best_angle / (math.pi / 2.0)) * (math.pi / 2.0)
        if abs(wrap_angle(best_angle - nearest_axis)) > tol:
            return None

        # Snap the robot heading to the nearest wall-aligned cardinal direction.
        snapped_theta = round(self.theta / (math.pi / 2.0)) * (math.pi / 2.0)
        return self.x, self.y, wrap_angle(snapped_theta)

    # --------------------------------------------------------------------- publish
    def publish_pose(self) -> None:
        """Publish the current pose; with no inputs this is the initial start pose."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_odom_time is None or (now - self.last_odom_time) > self.stationary_cmd_stale_sec:
            if self._command_active(now):
                self._mark_not_stationary()
            else:
                self._update_stationary(now, np.zeros(4, dtype=np.float64))
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "field"
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)
        stationary = Bool()
        stationary.data = bool(self.is_stationary)
        self.pub_stationary.publish(stationary)
        if self.encoder_motion_constraint_enabled and self.encoder_motion_constraint_publish_debug:
            motion_mode = String()
            motion_mode.data = self._motion_mode(now)
            self.pub_motion_mode.publish(motion_mode)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = "field"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = float(self.x)
            tf.transform.translation.y = float(self.y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizerNode()
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
