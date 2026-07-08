"""Arena-wall absolute localization: detect the standing walls in the wide fisheye, project their
FLOOR line to the ground, match to the known 4x4 boundary (x=+/-H, y=+/-H), and publish an absolute
pose correction (dx, dy, dtheta) on the SAME channel the object-landmarks use.

Why it works: the wall's floor line is at z=0, so the wide GROUND homography (the very one the world
model already uses, fisheye.undistortPoints -> rot180 -> perspectiveTransform) projects it exactly.
A detected floor segment, taken to the field frame with the current pose, should lie on x=+/-H or
y=+/-H; the offset is the position error and the segment's tilt is the heading error. The IMU-steadied
heading gives a good prior, so we only accept segments that already fall near an expected wall
(wall_gate_m) and are near axis-aligned — random edges are rejected.

Publishes: /localization/landmark_correction  Float32MultiArray [dx, dy, dtheta, confidence]
(localizer already consumes this and blends a small, clamped fraction — gentle, non-jumpy).
"""
from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

try:
    import cv2
    from cv_bridge import CvBridge
    _CV = True
except Exception:  # noqa: BLE001
    _CV = False


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class WallLocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_localizer_node")
        # wide fisheye intrinsics (MUST match the downscaled stream — same values as world_model)
        self.declare_parameter("top_fx", 669.125)
        self.declare_parameter("top_fy", 670.185)
        self.declare_parameter("top_cx", 616.474)
        self.declare_parameter("top_cy", 422.141)
        self.declare_parameter("dist_coeffs", [-0.085721, 0.053910, -0.034649, 0.007943])
        self.declare_parameter("image_rotated_180", True)
        self.declare_parameter("wide_homography_path",
                               "/home/seventt/seventt/workspace/data/calib/wide_ground.npz")
        self.declare_parameter("field_half_m", 2.0)          # walls at x,y = +/- this
        self.declare_parameter("rate_hz", 3.0)
        # detection / gating
        self.declare_parameter("canny_lo", 50)
        self.declare_parameter("canny_hi", 150)
        self.declare_parameter("hough_thresh", 60)
        self.declare_parameter("hough_min_len", 60)
        self.declare_parameter("hough_max_gap", 15)
        self.declare_parameter("use_lsd", True)                # helps the low-contrast wood wall/floor seam
        self.declare_parameter("clahe_clip", 2.0)
        self.declare_parameter("ignore_robot_rect", [0.34, 0.25, 0.66, 0.88])  # normalized x1,y1,x2,y2
        self.declare_parameter("use_color_mask", True)
        self.declare_parameter("color_lab_radius", 24.0)        # adaptive arena-wood color distance
        self.declare_parameter("color_sat_min", 45)
        self.declare_parameter("color_val_min", 45)
        self.declare_parameter("color_val_max", 235)
        self.declare_parameter("color_morph_kernel", 9)
        self.declare_parameter("color_hough_thresh", 35)
        self.declare_parameter("min_ground_len_m", 0.4)      # projected segment must be this long
        self.declare_parameter("wall_gate_m", 0.35)          # segment must fall within this of a wall
        self.declare_parameter("endpoint_gate_scale", 1.15)   # both projected endpoints must sit near same wall
        self.declare_parameter("axis_tol_deg", 14.0)         # segment must be ~parallel to the wall
        self.declare_parameter("max_range_m", 2.6)           # ignore projections beyond the arena+
        self.declare_parameter("correction_conf", 0.5)       # published confidence (localizer scales it)
        self.declare_parameter("max_pos_resid_m", 0.6)       # reject a match with a huge offset (spurious)
        self.declare_parameter("resid_mad_gate_m", 0.08)     # robust residual trimming floor

        self.fx = float(self.get_parameter("top_fx").value)
        self.fy = float(self.get_parameter("top_fy").value)
        self.cx = float(self.get_parameter("top_cx").value)
        self.cy = float(self.get_parameter("top_cy").value)
        d = [float(v) for v in self.get_parameter("dist_coeffs").value]
        self.rot180 = bool(self.get_parameter("image_rotated_180").value)
        self.H = float(self.get_parameter("field_half_m").value)
        self.canny = (int(self.get_parameter("canny_lo").value), int(self.get_parameter("canny_hi").value))
        self.h_thresh = int(self.get_parameter("hough_thresh").value)
        self.h_len = int(self.get_parameter("hough_min_len").value)
        self.h_gap = int(self.get_parameter("hough_max_gap").value)
        self.use_lsd = bool(self.get_parameter("use_lsd").value)
        self.clahe_clip = float(self.get_parameter("clahe_clip").value)
        self.ignore_robot_rect = [float(v) for v in self.get_parameter("ignore_robot_rect").value]
        self.use_color_mask = bool(self.get_parameter("use_color_mask").value)
        self.color_lab_radius = float(self.get_parameter("color_lab_radius").value)
        self.color_sat_min = int(self.get_parameter("color_sat_min").value)
        self.color_val_min = int(self.get_parameter("color_val_min").value)
        self.color_val_max = int(self.get_parameter("color_val_max").value)
        self.color_morph_kernel = int(self.get_parameter("color_morph_kernel").value)
        self.color_hough_thresh = int(self.get_parameter("color_hough_thresh").value)
        self.min_gl = float(self.get_parameter("min_ground_len_m").value)
        self.gate = float(self.get_parameter("wall_gate_m").value)
        self.endpoint_gate = self.gate * float(self.get_parameter("endpoint_gate_scale").value)
        self.axis_tol = math.radians(float(self.get_parameter("axis_tol_deg").value))
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.corr_conf = float(self.get_parameter("correction_conf").value)
        self.max_resid = float(self.get_parameter("max_pos_resid_m").value)
        self.resid_mad_gate = float(self.get_parameter("resid_mad_gate_m").value)

        self._K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], np.float64)
        self._D = np.array(d[:4], np.float64).reshape(4, 1)
        self._Hmat = None
        p = str(self.get_parameter("wide_homography_path").value)
        if _CV and os.path.exists(p):
            try:
                self._Hmat = np.asarray(np.load(p)["H"], np.float64)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"wide homography load failed ({exc})")
        self._bridge = CvBridge() if _CV else None
        self._pose = None       # (x, y, theta) from /localization/pose
        self._img = None

        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(Image, "/camera_top/image_raw", self.on_img, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, "/localization/landmark_correction", 10)

        ok = _CV and self._Hmat is not None
        self.get_logger().info(
            f"wall_localizer {'ready' if ok else 'IDLE (cv2/homography missing)'} "
            f"field=+/-{self.H}m gate={self.gate}m rate={self.get_parameter('rate_hz').value}Hz"
        )
        if ok:
            self.timer = self.create_timer(1.0 / max(0.5, float(self.get_parameter("rate_hz").value)),
                                           self.tick)

    def on_pose(self, msg: PoseStamped) -> None:
        yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self._pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def on_img(self, msg: Image) -> None:
        self._img = msg

    def _to_base(self, pts: np.ndarray) -> np.ndarray | None:
        """(N,2) pixels -> (N,2) base_link ground metres via fisheye undistort + rot180 + homography."""
        if pts.size == 0:
            return None
        und = cv2.fisheye.undistortPoints(pts.reshape(-1, 1, 2).astype(np.float64), self._K, self._D)
        n = und.reshape(-1, 2)
        if self.rot180:
            n = -n
        out = cv2.perspectiveTransform(n.reshape(-1, 1, 2), self._Hmat).reshape(-1, 2)
        return out

    def _robot_rect_px(self, shape: tuple[int, int]) -> tuple[int, int, int, int]:
        h, w = shape[:2]
        x1n, y1n, x2n, y2n = self.ignore_robot_rect[:4]
        return int(x1n * w), int(y1n * h), int(x2n * w), int(y2n * h)

    def _mask_robot_body(self, gray: np.ndarray) -> np.ndarray:
        """Suppress the center/bottom robot chassis so Hough does not lock onto our own frame."""
        out = gray.copy()
        h, w = out.shape[:2]
        x1, y1, x2, y2 = self._robot_rect_px(out.shape)
        fill = int(np.median(out))
        out[max(0, y1):min(h, y2), max(0, x1):min(w, x2)] = fill
        return out

    def _arena_color_mask(self, frame: np.ndarray) -> np.ndarray | None:
        """Adaptive wood-tone mask for today's arena.

        The camera has a strong reddish tint, so fixed RGB thresholds are brittle. Instead, take the
        median Lab color from plausible wood pixels in the current frame, then keep pixels close to
        that chroma/lightness. The resulting binary boundary is excellent at suppressing white
        objects, black clothes, cables, and the robot body before wall-line fitting.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        h, w = hsv.shape[:2]
        valid = (
            (hsv[:, :, 1] >= self.color_sat_min)
            & (hsv[:, :, 2] >= self.color_val_min)
            & (hsv[:, :, 2] <= self.color_val_max)
        )
        x1, y1, x2, y2 = self._robot_rect_px((h, w))
        valid[max(0, y1):min(h, y2), max(0, x1):min(w, x2)] = False
        # Ignore the extreme fisheye corners; they contain black borders, people, and room clutter.
        yy, xx = np.ogrid[:h, :w]
        cx, cy = w * 0.5, h * 0.5
        valid &= (((xx - cx) / (w * 0.53)) ** 2 + ((yy - cy) / (h * 0.56)) ** 2) <= 1.0
        pts = lab[valid]
        if pts.shape[0] < 2000:
            return None
        med = np.median(pts.astype(np.float32), axis=0)
        diff = lab.astype(np.float32) - med
        # Chroma is weighted higher than L because shadows change brightness more than wood color.
        dist = np.sqrt(0.45 * diff[:, :, 0] ** 2 + diff[:, :, 1] ** 2 + diff[:, :, 2] ** 2)
        mask = ((dist <= self.color_lab_radius) & valid).astype(np.uint8) * 255
        k = max(3, self.color_morph_kernel | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _line_candidates(self, frame: np.ndarray) -> list[tuple[int, int, int, int]]:
        """Find long image-space line candidates from today's arena frames.

        The real arena has wood-tone walls and floor. We combine texture edges with an adaptive
        Lab-color arena mask, then let metric wall gating reject clutter after ground projection.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self._mask_robot_body(gray)
        if self.clahe_clip > 0.0:
            clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        candidates: list[tuple[int, int, int, int]] = []
        edges = cv2.Canny(gray, self.canny[0], self.canny[1], apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=self.h_thresh,
                                minLineLength=self.h_len, maxLineGap=self.h_gap)
        if lines is not None:
            candidates.extend(tuple(int(v) for v in ln) for ln in lines[:, 0, :])

        if self.use_color_mask:
            mask = self._arena_color_mask(frame)
            if mask is not None:
                mask_edges = cv2.morphologyEx(
                    mask, cv2.MORPH_GRADIENT, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                )
                color_lines = cv2.HoughLinesP(
                    mask_edges, 1, np.pi / 180.0, threshold=self.color_hough_thresh,
                    minLineLength=self.h_len, maxLineGap=self.h_gap
                )
                if color_lines is not None:
                    candidates.extend(tuple(int(v) for v in ln) for ln in color_lines[:, 0, :])

        if self.use_lsd and hasattr(cv2, "createLineSegmentDetector"):
            try:
                detector = cv2.createLineSegmentDetector(0)
                found = detector.detect(gray)[0]
                if found is not None:
                    for ln in found.reshape(-1, 4):
                        x0, y0, x1, y1 = (int(round(v)) for v in ln)
                        if math.hypot(x1 - x0, y1 - y0) >= self.h_len:
                            candidates.append((x0, y0, x1, y1))
            except Exception:  # noqa: BLE001 - Hough candidates are still usable
                pass
        return candidates

    def _robust_median(self, values: list[float], gate_floor: float) -> tuple[float, int, float]:
        if not values:
            return 0.0, 0, 0.0
        arr = np.asarray(values, dtype=np.float64)
        med = float(np.median(arr))
        dev = np.abs(arr - med)
        mad = float(np.median(dev))
        gate = max(gate_floor, 2.5 * mad)
        kept = arr[dev <= gate]
        if kept.size == 0:
            return med, 0, mad
        return float(np.median(kept)), int(kept.size), mad

    def tick(self) -> None:
        if self._img is None or self._pose is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(self._img, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001
            return
        lines = self._line_candidates(frame)
        if not lines:
            return
        rx, ry, rth = self._pose
        ct, st = math.cos(rth), math.sin(rth)

        # Accumulate per-axis position residuals + heading residuals from matched wall segments.
        dx_list: list[float] = []
        dy_list: list[float] = []
        dth_list: list[float] = []
        for ln in lines:
            base = self._to_base(np.array([[ln[0], ln[1]], [ln[2], ln[3]]], np.float64))
            if base is None:
                continue
            (bx0, by0), (bx1, by1) = base
            if max(math.hypot(bx0, by0), math.hypot(bx1, by1)) > self.max_range:
                continue
            glen = math.hypot(bx1 - bx0, by1 - by0)
            if glen < self.min_gl:
                continue
            # segment endpoints to FIELD frame
            fx0, fy0 = rx + bx0 * ct - by0 * st, ry + bx0 * st + by0 * ct
            fx1, fy1 = rx + bx1 * ct - by1 * st, ry + bx1 * st + by1 * ct
            seg_ang = math.atan2(fy1 - fy0, fx1 - fx0)     # field-frame direction
            mx, my = (fx0 + fx1) / 2.0, (fy0 + fy1) / 2.0  # midpoint

            # Match to a wall: vertical walls x=+/-H (segment runs along y), horizontal y=+/-H (along x).
            for sgn in (+1.0, -1.0):
                # x = sgn*H  (vertical wall): segment ~vertical (dir ~ +/-90deg), midpoint x near sgn*H
                dang = abs(wrap(seg_ang - math.pi / 2.0))
                dang = min(dang, abs(wrap(seg_ang + math.pi / 2.0)))
                endpoint_resid = max(abs(fx0 - sgn * self.H), abs(fx1 - sgn * self.H))
                if dang < self.axis_tol and abs(mx - sgn * self.H) < self.gate and endpoint_resid < self.endpoint_gate:
                    resid = sgn * self.H - mx           # push robot so wall sits at sgn*H
                    if abs(resid) <= self.max_resid:
                        dx_list.append(resid)
                        # heading: wall should be exactly vertical; tilt = -heading error
                        dth_list.append(-wrap(seg_ang - math.copysign(math.pi / 2.0, seg_ang)))
                # y = sgn*H (horizontal wall): segment ~horizontal, midpoint y near sgn*H
                dang2 = min(abs(wrap(seg_ang)), abs(wrap(seg_ang - math.pi)))
                endpoint_resid = max(abs(fy0 - sgn * self.H), abs(fy1 - sgn * self.H))
                if dang2 < self.axis_tol and abs(my - sgn * self.H) < self.gate and endpoint_resid < self.endpoint_gate:
                    resid = sgn * self.H - my
                    if abs(resid) <= self.max_resid:
                        dy_list.append(resid)
                        dth_list.append(-wrap(seg_ang - (0.0 if abs(wrap(seg_ang)) < math.pi / 2 else math.pi)))

        if not dx_list and not dy_list:
            return
        dx, nx, mad_x = self._robust_median(dx_list, self.resid_mad_gate)
        dy, ny, mad_y = self._robust_median(dy_list, self.resid_mad_gate)
        dth, nth, mad_th = self._robust_median(dth_list, math.radians(3.0))
        # confidence scales with agreement count and drops when residuals are scattered.
        n = nx + ny
        scatter = max(mad_x, mad_y)
        scatter_penalty = max(0.35, 1.0 - scatter / max(0.01, self.gate))
        conf = min(1.0, self.corr_conf * (0.5 + 0.1 * n) * scatter_penalty)
        out = Float32MultiArray()
        out.data = [dx, dy, dth, conf]
        self.pub.publish(out)
        self.get_logger().info(
            f"wall fix dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.1f}deg "
            f"(x-walls={nx}/{len(dx_list)} y-walls={ny}/{len(dy_list)} th={nth} "
            f"mad={scatter:.3f}m/{math.degrees(mad_th):.1f}deg) conf={conf:.2f}",
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallLocalizerNode()
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
