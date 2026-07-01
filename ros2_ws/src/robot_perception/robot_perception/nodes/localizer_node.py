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
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

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

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.theta = float(self.get_parameter("initial_theta").value)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.use_vo = bool(self.get_parameter("use_visual_odometry").value)
        self.broadcast_tf = bool(self.get_parameter("broadcast_tf").value)
        self.use_landmark_correction = bool(self.get_parameter("use_landmark_correction").value)
        self.use_object_landmarks = bool(self.get_parameter("use_object_landmarks").value)
        self.landmark_gain = float(self.get_parameter("landmark_gain").value)
        self.landmark_max_step_m = float(self.get_parameter("landmark_max_step_m").value)
        self.landmark_max_step_rad = float(self.get_parameter("landmark_max_step_rad").value)

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
        self.create_subscription(Image, "/camera_top/image_raw", self.on_top_image, 10)
        self.create_subscription(
            Vector3, "/localization/landmark_correction", self.on_landmark_correction, 10
        )
        self.pub = self.create_publisher(PoseStamped, "/localization/pose", 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        self.get_logger().info(
            f"localizer start=({self.x:.2f},{self.y:.2f},{self.theta:.2f}) "
            f"lx={self.lx} ly={self.ly} vo={self.use_vo} landmark={self.use_landmark_correction} "
            f"obj_landmarks={self.use_object_landmarks} tf={self.tf_broadcaster is not None} "
            f"intrinsics={'set' if self.have_intrinsics else 'unset'} "
            f"fisheye={self.use_fisheye} rate={rate}Hz"
        )

    # ------------------------------------------------------------------ wheel odom
    def on_wheel_odom(self, msg: Float32MultiArray) -> None:
        """Integrate mecanum dead reckoning in the field frame."""
        if len(msg.data) < 4:
            self.get_logger().warn(
                f"wheel_odom expected 4 values, got {len(msg.data)}", throttle_duration_sec=5.0
            )
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_odom_time is None:
            # First sample only establishes a timestamp; no integration yet.
            self.last_odom_time = now
            return
        dt = now - self.last_odom_time
        self.last_odom_time = now
        # Clamp dt to a sane window so a stale/jumpy clock cannot teleport the pose.
        dt = max(0.0, min(0.2, dt))
        if dt <= 0.0:
            return

        fl, fr, rl, rr = (float(msg.data[i]) for i in range(4))

        # Mecanum forward kinematics (inverse of base_controller IK, k = lx + ly).
        vx = (fl + fr + rl + rr) / 4.0
        vy = (-fl + fr + rl - rr) / 4.0
        w = (-fl + fr - rl + rr) / (4.0 * self.k)

        # Body velocity -> field velocity using the current heading.
        ct, st = math.cos(self.theta), math.sin(self.theta)
        vxw = vx * ct - vy * st
        vyw = vx * st + vy * ct

        self.x += vxw * dt
        self.y += vyw * dt
        self.theta = wrap_angle(self.theta + w * dt)

    # ------------------------------------------------------------- object landmarks
    def on_landmark_correction(self, msg: Vector3) -> None:
        """Blend a small, clamped fraction of the world model's rigid drift estimate.

        msg = (dx, dy, dtheta) field-frame correction from re-observed mapped objects. Applied
        gently (gain + per-update clamp) so the pose converges without teleporting; the wheel
        odometry keeps integrating between corrections.
        """
        if not self.use_object_landmarks:
            return
        g, mx, mr = self.landmark_gain, self.landmark_max_step_m, self.landmark_max_step_rad
        self.x += max(-mx, min(mx, g * float(msg.x)))
        self.y += max(-mx, min(mx, g * float(msg.y)))
        self.theta = wrap_angle(self.theta + max(-mr, min(mr, g * float(msg.z))))

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

        # Optional undistort only when intrinsics are provided. Fisheye rectify for the wide
        # ~150 deg lens (straightens walls for Hough/LK); pinhole undistort otherwise.
        if self.have_intrinsics and np.any(self.dist_coeffs):
            if self.use_fisheye:
                frame = self._fisheye_rectify(frame)
            else:
                frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Frame-to-frame heading from LK-tracked features.
        theta_visual = self._visual_yaw_delta(gray)
        if theta_visual is not None:
            # Complementary filter in DELTA space (wrap-safe): nudge heading by a small
            # fraction of the measured frame-to-frame visual yaw. Averaging absolute angles
            # linearly would break at the +/-pi wrap boundary — and the start pose
            # theta=pi sits exactly on it, so this matters from the first frame.
            self.theta = wrap_angle(self.theta + 0.1 * theta_visual)

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
