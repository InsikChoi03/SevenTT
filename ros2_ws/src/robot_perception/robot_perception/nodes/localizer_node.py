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
from std_msgs.msg import Bool, Float32MultiArray

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
        self.declare_parameter("wheel_odom_enabled_wheels", [True, True, True, True])
        self.declare_parameter("wheel_odom_deadband_mps", 0.005)
        self.declare_parameter("wheel_odom_max_dt_sec", 0.4)
        self.declare_parameter("stationary_wheel_eps_mps", 0.008)
        self.declare_parameter("stationary_cmd_eps", 0.03)
        self.declare_parameter("stationary_cmd_stale_sec", 0.7)
        self.declare_parameter("stationary_required_sec", 0.8)
        self.declare_parameter("suppress_object_flow_when_stationary", True)
        enabled = [bool(v) for v in self.get_parameter("wheel_odom_enabled_wheels").value]
        self.wheel_odom_enabled = enabled if len(enabled) == 4 else [True, True, True, True]
        self.wheel_odom_deadband = float(self.get_parameter("wheel_odom_deadband_mps").value)
        self.wheel_odom_max_dt = float(self.get_parameter("wheel_odom_max_dt_sec").value)
        self.stationary_wheel_eps = float(self.get_parameter("stationary_wheel_eps_mps").value)
        self.stationary_cmd_eps = float(self.get_parameter("stationary_cmd_eps").value)
        self.stationary_cmd_stale_sec = float(self.get_parameter("stationary_cmd_stale_sec").value)
        self.stationary_required_sec = float(self.get_parameter("stationary_required_sec").value)
        self.suppress_object_flow_when_stationary = bool(
            self.get_parameter("suppress_object_flow_when_stationary").value
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
        self.declare_parameter("object_flow_stale_sec", 0.5)   # LK VO wakes up if flow older than this
        self.object_flow_stale_sec = float(self.get_parameter("object_flow_stale_sec").value)
        self.declare_parameter("object_flow_trans", True)      # also apply flow translation (else yaw only)
        self.object_flow_trans = bool(self.get_parameter("object_flow_trans").value)
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
        self.last_imu_time = None

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
            Float32MultiArray, "/localization/object_odom", self.on_object_odom, 10
        )
        self.pub = self.create_publisher(PoseStamped, "/localization/pose", 10)
        self.pub_stationary = self.create_publisher(Bool, "/localization/is_stationary", 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        self.get_logger().info(
            f"localizer start=({self.x:.2f},{self.y:.2f},{self.theta:.2f}) "
            f"lx={self.lx} ly={self.ly} vo={self.use_vo} landmark={self.use_landmark_correction} "
            f"obj_landmarks={self.use_object_landmarks} tf={self.tf_broadcaster is not None} "
            f"wheel_odom_enabled={self.wheel_odom_enabled} "
            f"intrinsics={'set' if self.have_intrinsics else 'unset'} "
            f"fisheye={self.use_fisheye} rate={rate}Hz"
        )

    # ------------------------------------------------------------------ wheel odom
    def on_wheel_cmd(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_wheel_cmd = [float(v) for v in msg.data[:4]]
            self.last_wheel_cmd_time = now
            odom_stale = (
                self.last_odom_time is None
                or (now - self.last_odom_time) > self.stationary_cmd_stale_sec
            )
            if odom_stale:
                self._update_stationary(now, np.zeros(4, dtype=np.float64))

    def on_wheel_odom(self, msg: Float32MultiArray) -> None:
        """Integrate mecanum dead reckoning in the field frame."""
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

        if int(self._wheel_mask.sum()) < 3:
            return

        # Mecanum forward kinematics. With all 4 wheels this is the exact inverse of
        # base_controller IK; with one disabled encoder it becomes a 3-equation least-squares solve.
        vx, vy, w = np.linalg.lstsq(self._wheel_rows, wheels[self._wheel_mask], rcond=None)[0]

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
        if not flow_trans_fresh:
            self.x += vxw * dt
            self.y += vyw * dt
        # Rotation: IMU > VO > wheel. Only integrate the (open-loop, drift-prone) wheel yaw when BOTH
        # the IMU and VO are stale/absent — so driving never double-counts rotation.
        if not self._imu_fresh(now) and (self.last_vo_time is None
                                         or (now - self.last_vo_time) > self.vo_stale_sec):
            self.theta = wrap_angle(self.theta + w * dt)

    def _update_stationary(self, now: float, wheels: np.ndarray) -> None:
        enabled_wheels = wheels[self._wheel_mask] if int(self._wheel_mask.sum()) else wheels
        wheel_still = bool(
            enabled_wheels.size == 0
            or np.max(np.abs(enabled_wheels)) <= self.stationary_wheel_eps
        )
        if self.last_wheel_cmd_time is None:
            cmd_still = True
        else:
            cmd_stale = (now - self.last_wheel_cmd_time) > self.stationary_cmd_stale_sec
            cmd_still = (
                cmd_stale
                or max(abs(v) for v in self.last_wheel_cmd) <= self.stationary_cmd_eps
            )
        currently_still = wheel_still and cmd_still
        if currently_still:
            if self.stationary_since is None:
                self.stationary_since = now
            self.is_stationary = (now - self.stationary_since) >= self.stationary_required_sec
        else:
            self.stationary_since = None
            self.is_stationary = False

    # --------------------------------------------------------------------- IMU heading
    def _imu_fresh(self, now: float) -> bool:
        return (self.use_imu and self.last_imu_time is not None
                and (now - self.last_imu_time) <= self.imu_stale_sec)

    def on_imu(self, msg: Imu) -> None:
        """TOP-PRIORITY heading: integrate the bias-removed yaw-rate gyro. Low noise + slow bias, so
        heading stays steady (no object-flow churn / LK-VO hallucination). When this is fresh the
        object-flow / LK-VO / wheel yaw are all suppressed so nothing double-counts the rotation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        wz = float(msg.angular_velocity.z)
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if 0.0 < dt < 0.2 and abs(wz) >= self.imu_gyro_deadband:   # deadband kills stationary walk
                self.theta = wrap_angle(self.theta + wz * dt)
        self.last_imu_time = now

    # ------------------------------------------------------------- object-flow odometry
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
        if conf < self.object_flow_min_conf:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.suppress_object_flow_when_stationary and self.is_stationary:
            self.last_objflow_time = now
            self.last_vo_time = now
            return
        self.last_objflow_time = now
        self.last_vo_time = now      # object-flow owns rotation -> keep wheel-yaw AND LK-VO suppressed
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
        self.x += max(-mx, min(mx, g_xy * dx))
        self.y += max(-mx, min(mx, g_xy * dy))
        g_th = self.landmark_theta_gain * cf                        # confidence-scaled heading authority
        self.theta = wrap_angle(self.theta + max(-mr, min(mr, g_th * dth)))

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
            if abs(theta_visual) >= self.vo_deadband and not self._imu_fresh(now):
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
                self.theta = wrap_angle(self.theta + 0.05 * err)

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
