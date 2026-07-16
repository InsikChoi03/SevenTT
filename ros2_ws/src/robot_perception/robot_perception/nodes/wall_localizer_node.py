"""Arena-wall observation from the learned wide-camera segmentation mask.

Each connected mask component is represented by a line through its median pixel in the component's
dominant direction. These mask-derived lines are authoritative wall observations; the old arena-axis
and expected-wall-position acceptance gates are intentionally not used.

Publishes:
  /localization/wall_segments        Float32MultiArray correction-eligible mask lines in field coordinates
  /localization/wall_raw_segments    Float32MultiArray raw projected mask lines in field coordinates
  /localization/wall_mask_segments_image Float32MultiArray authoritative mask lines in image pixels
  /localization/wall_segmentation_mask Image mono8 debug mask from the learned model
"""
from __future__ import annotations

import math
import os
import time
from collections import deque
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

try:
    import cv2
    from cv_bridge import CvBridge
    _CV = True
except Exception:  # noqa: BLE001
    _CV = False

try:
    from ultralytics import YOLO
    _YOLO = True
except Exception:  # noqa: BLE001
    YOLO = None  # type: ignore[assignment, misc]
    _YOLO = False


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
        self.declare_parameter("cam_offset_x", 0.0)
        self.declare_parameter("cam_offset_y", 0.0)
        self.declare_parameter("rate_hz", 3.0)
        self.declare_parameter("ignore_robot_rect", [0.34, 0.25, 0.66, 0.88])  # normalized x1,y1,x2,y2
        self.declare_parameter("max_range_m", 2.6)           # ignore projections beyond the arena+
        # Learned mask is the sole wall-observation source.
        self.declare_parameter("use_segmentation_mask", False)
        self.declare_parameter("segmentation_model_path", "")
        self.declare_parameter("segmentation_input_size", 640)
        self.declare_parameter("segmentation_min_confidence", 0.35)
        self.declare_parameter("segmentation_run_rate_hz", 1.0)
        self.declare_parameter("segmentation_morph_kernel", 5)
        self.declare_parameter("publish_segmentation_mask", True)
        self.declare_parameter("segmentation_min_component_area", 120)
        self.declare_parameter("segmentation_max_components", 6)
        self.declare_parameter("segmentation_min_line_length_px", 60)
        self.declare_parameter("wall_image_edge_reject_ratio", 0.05)
        self.declare_parameter("wall_anchor_enabled", True)
        self.declare_parameter("wall_anchor_min_lines", 2)
        self.declare_parameter("wall_anchor_smoothing", 0.20)
        self.declare_parameter("field_half_extent_m", 2.0)
        self.declare_parameter("wall_field_max_residual_m", 0.80)
        self.declare_parameter("wall_field_smoothing", 0.25)
        self.declare_parameter("wall_field_fast_smoothing", 1.0)
        self.declare_parameter("wall_field_filter_window", 5)

        self.fx = float(self.get_parameter("top_fx").value)
        self.fy = float(self.get_parameter("top_fy").value)
        self.cx = float(self.get_parameter("top_cx").value)
        self.cy = float(self.get_parameter("top_cy").value)
        d = [float(v) for v in self.get_parameter("dist_coeffs").value]
        self.rot180 = bool(self.get_parameter("image_rotated_180").value)
        self.cam_offset_x = float(self.get_parameter("cam_offset_x").value)
        self.cam_offset_y = float(self.get_parameter("cam_offset_y").value)
        self.ignore_robot_rect = [float(v) for v in self.get_parameter("ignore_robot_rect").value]
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.use_segmentation_mask = bool(self.get_parameter("use_segmentation_mask").value)
        self.segmentation_model_path = str(self.get_parameter("segmentation_model_path").value)
        self.segmentation_input_size = int(self.get_parameter("segmentation_input_size").value)
        self.segmentation_min_conf = float(self.get_parameter("segmentation_min_confidence").value)
        self.segmentation_run_interval = 1.0 / max(
            0.1, float(self.get_parameter("segmentation_run_rate_hz").value)
        )
        self.segmentation_morph_kernel = int(self.get_parameter("segmentation_morph_kernel").value)
        self.publish_segmentation_mask = bool(self.get_parameter("publish_segmentation_mask").value)
        self.segmentation_min_component_area = int(
            self.get_parameter("segmentation_min_component_area").value
        )
        self.segmentation_max_components = int(self.get_parameter("segmentation_max_components").value)
        self.h_len = int(self.get_parameter("segmentation_min_line_length_px").value)
        self.wall_image_edge_reject_ratio = max(
            0.0, min(0.49, float(self.get_parameter("wall_image_edge_reject_ratio").value))
        )
        self.wall_anchor_enabled = bool(self.get_parameter("wall_anchor_enabled").value)
        self.wall_anchor_min_lines = int(self.get_parameter("wall_anchor_min_lines").value)
        self.wall_anchor_smoothing = float(self.get_parameter("wall_anchor_smoothing").value)
        self.field_half_extent = float(self.get_parameter("field_half_extent_m").value)
        self.wall_field_max_residual = float(
            self.get_parameter("wall_field_max_residual_m").value
        )
        self.wall_field_smoothing = float(self.get_parameter("wall_field_smoothing").value)
        self.wall_field_fast_smoothing = float(
            self.get_parameter("wall_field_fast_smoothing").value
        )
        self.wall_field_filter_window = max(
            1, int(self.get_parameter("wall_field_filter_window").value)
        )

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
        self._seg_model: Any | None = None
        self._last_seg_time = 0.0
        self._stationary = False
        self._wall_anchor_lines: list[np.ndarray] | None = None
        self._wall_anchor_pose: tuple[float, float, float] | None = None
        self._wall_anchor_ema = np.zeros(3, dtype=np.float64)
        self._wall_field_ema = np.zeros(3, dtype=np.float64)
        self._wall_field_history: deque[np.ndarray] = deque(maxlen=self.wall_field_filter_window)
        self._wall_fast_correction = False
        self._load_segmentation_model()

        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(Bool, "/localization/is_stationary", self.on_stationary, 10)
        self.create_subscription(
            Bool, "/localization/wall_fast_correction", self.on_wall_fast_correction, 10
        )
        self.create_subscription(Image, "/camera_top/image_raw", self.on_img, qos_profile_sensor_data)
        self.pub_segments = self.create_publisher(Float32MultiArray, "/localization/wall_segments", 10)
        self.pub_raw_segments = self.create_publisher(
            Float32MultiArray, "/localization/wall_raw_segments", 10
        )
        self.pub_wall_anchor = self.create_publisher(
            Float32MultiArray, "/localization/wall_anchor_correction", 10
        )
        self.pub_wall_field = self.create_publisher(
            Float32MultiArray, "/localization/wall_field_correction", 10
        )
        self.pub_mask_segments_image = self.create_publisher(
            Float32MultiArray, "/localization/wall_mask_segments_image", 10
        )
        self.pub_segmentation_mask = self.create_publisher(
            Image, "/localization/wall_segmentation_mask", 10
        )

        ok = _CV and self._Hmat is not None
        self.get_logger().info(
            f"wall_localizer {'ready' if ok else 'IDLE (cv2/homography missing)'} "
            f"mask-authoritative rate={self.get_parameter('rate_hz').value}Hz "
            f"seg={'on' if self._seg_model is not None else 'off'}"
        )
        if ok:
            self.timer = self.create_timer(1.0 / max(0.5, float(self.get_parameter("rate_hz").value)),
                                           self.tick)

    def on_pose(self, msg: PoseStamped) -> None:
        yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self._pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def on_stationary(self, msg: Bool) -> None:
        self._stationary = bool(msg.data)

    def on_wall_fast_correction(self, msg: Bool) -> None:
        fast = bool(msg.data)
        if fast and not self._wall_fast_correction:
            self._wall_field_history.clear()
        self._wall_fast_correction = fast

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
        out[:, 0] += self.cam_offset_x
        out[:, 1] += self.cam_offset_y
        return out

    def _robot_rect_px(self, shape: tuple[int, int]) -> tuple[int, int, int, int]:
        h, w = shape[:2]
        x1n, y1n, x2n, y2n = self.ignore_robot_rect[:4]
        return int(x1n * w), int(y1n * h), int(x2n * w), int(y2n * h)

    def _load_segmentation_model(self) -> None:
        if not self.use_segmentation_mask:
            return
        if not _YOLO or YOLO is None:
            self.get_logger().warn(
                "segmentation requested but ultralytics is unavailable; using edge fallback"
            )
            return
        if not self.segmentation_model_path or not os.path.exists(self.segmentation_model_path):
            self.get_logger().warn(
                f"segmentation model missing: '{self.segmentation_model_path}'; using edge fallback"
            )
            return
        try:
            self._seg_model = YOLO(self.segmentation_model_path)
            names = getattr(self._seg_model, "names", None)
            self.get_logger().info(
                f"wall segmentation loaded from '{self.segmentation_model_path}' classes={names}"
            )
        except Exception as exc:  # noqa: BLE001
            self._seg_model = None
            self.get_logger().warn(
                f"failed to load wall segmentation model '{self.segmentation_model_path}': {exc}; "
                "using edge fallback"
            )

    def _segmentation_candidates(self, frame: np.ndarray) -> list[tuple[int, int, int, int]]:
        """Run wall/floor-boundary segmentation and extract image-space line candidates."""
        if self._seg_model is None:
            return []
        now = time.monotonic()
        if now - self._last_seg_time < self.segmentation_run_interval:
            return []
        self._last_seg_time = now

        try:
            results = self._seg_model.predict(
                frame,
                conf=self.segmentation_min_conf,
                imgsz=self.segmentation_input_size,
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"wall segmentation inference failed: {exc}", throttle_duration_sec=5.0)
            self._publish_mask_segments_image([])
            return []

        if not results:
            self._publish_mask_segments_image([])
            return []
        masks = getattr(results[0], "masks", None)
        if masks is None or getattr(masks, "data", None) is None:
            self._publish_mask_segments_image([])
            return []

        h, w = frame.shape[:2]
        try:
            arr = masks.data.detach().cpu().numpy()
        except Exception:  # noqa: BLE001
            arr = masks.data.cpu().numpy()
        if arr.size == 0:
            self._publish_mask_segments_image([])
            return []

        mask = (np.max(arr, axis=0) > 0.5).astype(np.uint8) * 255
        if mask.shape[:2] != (h, w):
            mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

        x1, y1, x2, y2 = self._robot_rect_px((h, w))
        mask[max(0, y1):min(h, y2), max(0, x1):min(w, x2)] = 0

        k = max(3, self.segmentation_morph_kernel | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        if self.publish_segmentation_mask and self._bridge is not None:
            try:
                self.pub_segmentation_mask.publish(self._bridge.cv2_to_imgmsg(mask, encoding="mono8"))
            except Exception:  # noqa: BLE001 - debug overlay must never affect localization
                pass
        candidates = self._mask_centerline_candidates(mask)
        self._publish_mask_segments_image(candidates)
        self.get_logger().debug(
            f"authoritative mask wall lines={len(candidates)} "
            f"mask_px={int(np.count_nonzero(mask))}",
            throttle_duration_sec=1.0,
        )
        return candidates

    def _publish_mask_segments_image(
        self,
        segments: list[tuple[int, int, int, int]],
    ) -> None:
        """Publish raw median/PCA mask centerlines for the yellow camera debug overlay."""
        msg = Float32MultiArray()
        for x0, y0, x1, y1 in segments:
            msg.data.extend([float(x0), float(y0), float(x1), float(y1)])
        self.pub_mask_segments_image.publish(msg)

    def _base_line_points(self, lines: list[tuple[int, int, int, int]]) -> list[np.ndarray]:
        points = []
        for line in lines:
            base = self._to_base(np.array([[line[0], line[1]], [line[2], line[3]]], np.float64))
            if base is None or not np.all(np.isfinite(base)):
                continue
            if max(np.linalg.norm(base[0]), np.linalg.norm(base[1])) <= self.max_range:
                points.append(base)
        return points

    def _update_wall_anchor(self, current: list[np.ndarray]) -> None:
        """Use the initial stationary wall view as a local absolute-origin reference."""
        if not self.wall_anchor_enabled or not self._stationary or self._pose is None:
            return
        if len(current) < self.wall_anchor_min_lines:
            return
        if self._wall_anchor_lines is None:
            self._wall_anchor_lines = current[:]
            self._wall_anchor_pose = self._pose
            self._wall_anchor_ema.fill(0.0)
            self.get_logger().info(
                f"wall anchor captured lines={len(current)} "
                f"pose=({self._pose[0]:+.3f},{self._pose[1]:+.3f},"
                f"{math.degrees(self._pose[2]):+.1f}deg)"
            )
            return

        anchor = self._wall_anchor_lines
        anchor_mid = np.asarray([(p[0] + p[1]) * 0.5 for p in anchor], dtype=np.float64)
        current_mid = np.asarray([(p[0] + p[1]) * 0.5 for p in current], dtype=np.float64)
        matches = []
        used = set()
        for aidx, ap in enumerate(anchor):
            avec = ap[1] - ap[0]
            aang = math.atan2(float(avec[1]), float(avec[0]))
            best = None
            for cidx, cp in enumerate(current):
                if cidx in used:
                    continue
                cvec = cp[1] - cp[0]
                cang = math.atan2(float(cvec[1]), float(cvec[0]))
                dang = abs(math.atan2(math.sin(cang - aang), math.cos(cang - aang)))
                score = dang + 0.25 * float(np.linalg.norm(current_mid[cidx] - anchor_mid[aidx]))
                if best is None or score < best[0]:
                    best = (score, cidx)
            if best is not None:
                used.add(best[1])
                matches.append((anchor_mid[aidx], current_mid[best[1]]))
        if len(matches) < self.wall_anchor_min_lines:
            return

        target = np.asarray([m[0] for m in matches], dtype=np.float64)
        source = np.asarray([m[1] for m in matches], dtype=np.float64)
        target_mean = target.mean(axis=0)
        source_mean = source.mean(axis=0)
        u, _, vt = np.linalg.svd((source - source_mean).T @ (target - target_mean))
        rot = vt.T @ u.T
        if np.linalg.det(rot) < 0.0:
            vt[-1, :] *= -1.0
            rot = vt.T @ u.T
        translation_base = target_mean - rot @ source_mean
        dtheta = math.atan2(float(rot[1, 0]), float(rot[0, 0]))
        anchor_theta = self._wall_anchor_pose[2]
        ct, st = math.cos(anchor_theta), math.sin(anchor_theta)
        correction = np.array([
            ct * translation_base[0] - st * translation_base[1],
            st * translation_base[0] + ct * translation_base[1],
            dtheta,
        ])
        alpha = max(0.01, min(1.0, self.wall_anchor_smoothing))
        self._wall_anchor_ema = (1.0 - alpha) * self._wall_anchor_ema + alpha * correction
        if float(np.linalg.norm(self._wall_anchor_ema[:2])) < 0.003 and abs(float(self._wall_anchor_ema[2])) < math.radians(0.3):
            return
        msg = Float32MultiArray()
        msg.data = [
            float(self._wall_anchor_ema[0]),
            float(self._wall_anchor_ema[1]),
            float(self._wall_anchor_ema[2]),
            min(1.0, 0.2 * len(matches)),
        ]
        self.pub_wall_anchor.publish(msg)

    def _mask_centerline_candidates(self, mask: np.ndarray) -> list[tuple[int, int, int, int]]:
        """Fit centerline segments through connected mask blobs.

        The model is trained to output a thin wall/floor-boundary band. Using the mask gradient
        finds the band edges; fitting the foreground pixels instead gives a segment through the
        middle of the learned boundary, which is what the ground projection should use.
        """
        num, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), 8)
        comps = []
        for idx in range(1, num):
            area = int(stats[idx, cv2.CC_STAT_AREA])
            if area >= self.segmentation_min_component_area:
                comps.append((area, idx))
        comps.sort(reverse=True)

        h, w = mask.shape[:2]
        out: list[tuple[int, int, int, int]] = []
        for _, idx in comps[:max(1, self.segmentation_max_components)]:
            ys, xs = np.nonzero(labels == idx)
            if xs.size < 8:
                continue
            pts = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
            # Use the pixel-wise median as the anchor. A few stray mask pixels then cannot pull
            # the fitted wall segment away from the learned boundary as much as a mean can.
            anchor = np.median(pts, axis=0)
            centered = pts - anchor
            cov = np.cov(centered, rowvar=False)
            try:
                vals, vecs = np.linalg.eigh(cov)
            except np.linalg.LinAlgError:
                continue
            # An L-shaped mask has no single meaningful PCA direction. Its first principal
            # axis points diagonally between the two walls, so never emit that diagonal as a
            # wall. Split such components into line segments directly from mask pixels instead.
            total_variance = float(np.sum(vals))
            linearity = float(vals[-1] / total_variance) if total_variance > 1e-9 else 0.0
            if linearity < 0.88:
                split = self._split_mask_component(labels == idx)
                if split:
                    out.extend(split)
                    continue
            axis = vecs[:, int(np.argmax(vals))]
            axis_norm = float(np.linalg.norm(axis))
            if axis_norm < 1e-9:
                continue
            axis = axis / axis_norm
            normal = np.array([-axis[1], axis[0]], dtype=np.float64)
            normal_dist = np.abs(centered @ normal)
            median_dist = float(np.median(normal_dist))
            mad_dist = float(np.median(np.abs(normal_dist - median_dist)))
            inlier_gate = max(3.0, median_dist + 2.5 * mad_dist)
            inliers = normal_dist <= inlier_gate
            if int(np.count_nonzero(inliers)) < 8:
                continue

            # Re-estimate direction from the inliers while keeping the robust median anchor.
            robust_centered = centered[inliers]
            robust_cov = np.cov(robust_centered, rowvar=False)
            try:
                robust_vals, robust_vecs = np.linalg.eigh(robust_cov)
            except np.linalg.LinAlgError:
                continue
            axis = robust_vecs[:, int(np.argmax(robust_vals))]
            axis_norm = float(np.linalg.norm(axis))
            if axis_norm < 1e-9:
                continue
            axis = axis / axis_norm
            proj = centered[inliers] @ axis
            # Trim ragged ends and isolated pixels from the fit.
            t0, t1 = np.percentile(proj, [4.0, 96.0])
            p0 = anchor + axis * t0
            p1 = anchor + axis * t1
            length = float(np.linalg.norm(p1 - p0))
            if length < self.h_len:
                continue
            x0 = int(round(min(max(p0[0], 0.0), w - 1.0)))
            y0 = int(round(min(max(p0[1], 0.0), h - 1.0)))
            x1 = int(round(min(max(p1[0], 0.0), w - 1.0)))
            y1 = int(round(min(max(p1[1], 0.0), h - 1.0)))
            out.append((x0, y0, x1, y1))
        return out

    def _split_mask_component(self, component: np.ndarray) -> list[tuple[int, int, int, int]]:
        """Extract separate mask-supported lines from a non-linear/L-shaped component."""
        ys, xs = np.nonzero(component)
        if xs.size < 8:
            return []
        corner_split = self._split_at_mask_corner(component)
        if corner_split:
            return corner_split
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        crop = (component[y0:y1 + 1, x0:x1 + 1].astype(np.uint8) * 255)
        min_len = max(28, int(self.h_len * 0.45))
        threshold = max(12, int(min_len * 0.35))
        detected = cv2.HoughLinesP(
            crop,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=threshold,
            minLineLength=min_len,
            maxLineGap=max(8, int(self.segmentation_morph_kernel * 2)),
        )
        if detected is None:
            return []

        candidates: list[tuple[float, tuple[int, int, int, int]]] = []
        for raw in detected[:, 0, :]:
            ax, ay, bx, by = (int(v) for v in raw)
            ax, bx, ay, by = ax + x0, bx + x0, ay + y0, by + y0
            length = math.hypot(bx - ax, by - ay)
            if length < min_len:
                continue
            candidates.append((length, (ax, ay, bx, by)))
        candidates.sort(reverse=True, key=lambda item: item[0])

        # Hough can return several nearby detections for one arm. Keep at most one line for
        # each local direction/position, while allowing the two arms of an L to survive.
        out: list[tuple[int, int, int, int]] = []
        descriptors: list[float] = []
        for length, line in candidates:
            ax, ay, bx, by = line
            angle = math.atan2(by - ay, bx - ax) % math.pi
            if angle > math.pi * 0.5:
                angle -= math.pi
            duplicate = False
            for old_angle in descriptors:
                angle_gap = min(abs(angle - old_angle), math.pi - abs(angle - old_angle))
                if angle_gap < math.radians(15.0):
                    duplicate = True
                    break
            if duplicate:
                continue
            out.append(line)
            descriptors.append(angle)
            if len(out) >= 3:
                break
        return out

    def _split_at_mask_corner(self, component: np.ndarray) -> list[tuple[int, int, int, int]]:
        """Split an L mask at its contour corner where the coordinate direction changes."""
        contours, _ = cv2.findContours(
            component.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return []
        contour = max(contours, key=cv2.contourArea)
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 1.0:
            return []
        approx = cv2.approxPolyDP(contour, max(2.0, 0.015 * perimeter), True)
        vertices = approx.reshape(-1, 2).astype(np.float64)
        if len(vertices) < 3:
            return []

        corner_candidates = []
        for i, corner in enumerate(vertices):
            prev = vertices[(i - 1) % len(vertices)] - corner
            nxt = vertices[(i + 1) % len(vertices)] - corner
            lp, ln = float(np.linalg.norm(prev)), float(np.linalg.norm(nxt))
            if min(lp, ln) < max(20.0, self.h_len * 0.25):
                continue
            cosine = float(np.dot(prev, nxt) / max(lp * ln, 1e-9))
            angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
            if not 55.0 <= angle <= 125.0:
                continue
            edge_in = -prev
            edge_out = nxt
            turn = float(edge_in[0] * edge_out[1] - edge_in[1] * edge_out[0])
            corner_candidates.append((min(lp, ln), corner, prev / lp, nxt / ln, turn))
        if not corner_candidates:
            return []
        # Prefer the reflex (inside) corner of an L over the four outside rectangle corners.
        turns = [c[4] for c in corner_candidates if abs(c[4]) > 1e-6]
        majority_sign = 1.0 if sum(t > 0.0 for t in turns) >= len(turns) / 2.0 else -1.0
        reflex = [c for c in corner_candidates if c[4] * majority_sign < 0.0]
        best = max(reflex or corner_candidates, key=lambda c: c[0])

        _, corner, arm_a, arm_b, _ = best
        ys, xs = np.nonzero(component)
        points = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
        rel = points - corner
        groups = []
        for arm in (arm_a, arm_b):
            along = rel @ arm
            across = np.abs(rel[:, 0] * arm[1] - rel[:, 1] * arm[0])
            # Keep mask points close to the ray from the detected bend. The bend itself may
            # belong to both groups, which is harmless and makes the fitted centers stable.
            keep = (along >= -8.0) & (across <= np.maximum(14.0, 0.14 * np.maximum(along, 0.0)))
            selected = points[keep]
            if selected.shape[0] < 20 or np.ptp(selected @ arm) < self.h_len * 0.35:
                return []
            groups.append(selected)

        out: list[tuple[int, int, int, int]] = []
        h, w = component.shape[:2]
        for selected in groups:
            anchor = np.median(selected, axis=0)
            centered = selected - anchor
            cov = np.cov(centered, rowvar=False)
            try:
                vals, vecs = np.linalg.eigh(cov)
            except np.linalg.LinAlgError:
                return []
            axis = vecs[:, int(np.argmax(vals))]
            if float(np.linalg.norm(axis)) < 1e-9:
                return []
            axis /= np.linalg.norm(axis)
            proj = centered @ axis
            t0, t1 = np.percentile(proj, [5.0, 95.0])
            p0, p1 = anchor + axis * t0, anchor + axis * t1
            if float(np.linalg.norm(p1 - p0)) < self.h_len * 0.45:
                return []
            out.append((
                int(round(min(max(p0[0], 0.0), w - 1.0))),
                int(round(min(max(p0[1], 0.0), h - 1.0))),
                int(round(min(max(p1[0], 0.0), w - 1.0))),
                int(round(min(max(p1[1], 0.0), h - 1.0))),
            ))
        return out

    def _line_candidates(self, frame: np.ndarray) -> list[tuple[int, int, int, int]]:
        """Return only the learned mask's median/dominant-direction wall lines."""
        return self._segmentation_candidates(frame)

    def _line_midpoint_in_image_edge(
        self, line: tuple[int, int, int, int], width: int, height: int
    ) -> bool:
        """Reject pose correction from wall lines whose midpoint is too close to the image edge."""
        ratio = self.wall_image_edge_reject_ratio
        if ratio <= 0.0 or width <= 0 or height <= 0:
            return False
        x0, y0, x1, y1 = line
        mx = 0.5 * (float(x0) + float(x1))
        my = 0.5 * (float(y0) + float(y1))
        return (
            mx <= ratio * width or mx >= (1.0 - ratio) * width
            or my <= ratio * height or my >= (1.0 - ratio) * height
        )

    def _field_wall_correction(self, segments: list[tuple[float, float, float, float]]) -> None:
        """Estimate one rigid field-map transform from raw walls to the fixed rectangle.

        The returned [tx, ty, dtheta] is a global SE(2) transform. The localizer applies it
        to the robot and republishes the exact applied transform so the world model can move
        every stored object by the same amount, preserving all relative geometry.
        """
        if not segments:
            return
        extent = self.field_half_extent
        usable: list[tuple[tuple[float, float, float, float], bool]] = []
        heading_errors: list[float] = []
        for x0, y0, x1, y1 in segments:
            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            if length < 0.12:
                continue
            angle = math.atan2(dy, dx)
            # A line has no direction, so compare its angle modulo pi.
            line_angle = angle % math.pi
            mx, my = (x0 + x1) * 0.5, (y0 + y1) * 0.5
            horizontal = min(line_angle, math.pi - line_angle) <= math.radians(35.0)
            if horizontal:
                target_angle = 0.0
            else:
                target_angle = math.pi * 0.5
            angle_error = (target_angle - line_angle + math.pi * 0.5) % math.pi - math.pi * 0.5
            if abs(angle_error) <= math.radians(35.0):
                heading_errors.append(angle_error)
                usable.append(((x0, y0, x1, y1), horizontal))

        if not usable or not heading_errors:
            return

        dtheta = float(np.median(heading_errors))
        ct, st = math.cos(dtheta), math.sin(dtheta)
        x_residuals: list[float] = []
        y_residuals: list[float] = []
        ts = np.linspace(0.1, 0.9, 5)
        for (x0, y0, x1, y1), horizontal in usable:
            sx = x0 + (x1 - x0) * ts
            sy = y0 + (y1 - y0) * ts
            rx = ct * sx - st * sy
            ry = st * sx + ct * sy
            if horizontal:
                center = float(np.median(ry))
                target = min((-extent, extent), key=lambda wall: abs(wall - center))
                residuals = target - ry
                residual = float(np.median(residuals))
                spread = float(np.ptp(residuals))
                if abs(residual) <= self.wall_field_max_residual and spread <= 0.15:
                    y_residuals.append(residual)
            else:
                center = float(np.median(rx))
                target = min((-extent, extent), key=lambda wall: abs(wall - center))
                residuals = target - rx
                residual = float(np.median(residuals))
                spread = float(np.ptp(residuals))
                if abs(residual) <= self.wall_field_max_residual and spread <= 0.15:
                    x_residuals.append(residual)

        if not x_residuals and not y_residuals:
            return
        correction = np.array([math.nan, math.nan, dtheta], dtype=np.float64)
        if x_residuals:
            correction[0] = float(np.median(x_residuals))
        if y_residuals:
            correction[1] = float(np.median(y_residuals))

        # A wall correction is a measurement, not a command. If an axis is not observed in the
        # current frame, decay that axis toward zero instead of replaying an old correction forever.
        # Also reset history when the measured correction crosses zero so EMA lag cannot keep
        # pushing the pose past the wall.
        sign_flip = any(
            math.isfinite(float(correction[i]))
            and abs(float(correction[i])) > 1e-6
            and abs(float(self._wall_field_ema[i])) > 1e-6
            and float(correction[i]) * float(self._wall_field_ema[i]) < 0.0
            for i in range(3)
        )
        if sign_flip:
            self._wall_field_history.clear()
            self._wall_field_ema[:] = 0.0

        if self._wall_fast_correction:
            filtered = np.array(
                [
                    correction[i] if math.isfinite(float(correction[i])) else 0.0
                    for i in range(3)
                ],
                dtype=np.float64,
            )
        else:
            self._wall_field_history.append(correction)
            hist = np.stack(list(self._wall_field_history), axis=0)
            filtered = np.empty(3, dtype=np.float64)
            for i in range(3):
                if not math.isfinite(float(correction[i])):
                    filtered[i] = 0.0
                    continue
                values = hist[:, i]
                values = values[np.isfinite(values)]
                filtered[i] = float(np.median(values)) if values.size else float(correction[i])
        # EMA prevents a single-frame mask jitter from moving the pose visibly.
        smoothing = self.wall_field_fast_smoothing if self._wall_fast_correction else self.wall_field_smoothing
        a = max(0.0, min(1.0, smoothing))
        self._wall_field_ema = (1.0 - a) * self._wall_field_ema + a * filtered
        confidence = min(1.0, (len(x_residuals) + len(y_residuals)) / 2.0)
        msg = Float32MultiArray()
        msg.data = [*map(float, self._wall_field_ema), float(confidence)]
        self.pub_wall_field.publish(msg)

    def _snap_segments_to_field(
        self, segments: list[tuple[float, float, float, float]]
    ) -> list[tuple[float, float, float, float]]:
        """Render yellow observations directly on the fixed gray-box wall coordinates."""
        snapped: list[tuple[float, float, float, float]] = []
        extent = self.field_half_extent
        for x0, y0, x1, y1 in segments:
            dx, dy = x1 - x0, y1 - y0
            if math.hypot(dx, dy) < 0.12:
                continue
            angle = math.atan2(dy, dx) % math.pi
            mx, my = (x0 + x1) * 0.5, (y0 + y1) * 0.5
            horizontal = min(angle, math.pi - angle) <= math.radians(35.0)
            if horizontal:
                wall_y = extent if my >= 0.0 else -extent
                snapped.append((x0, wall_y, x1, wall_y))
            else:
                wall_x = extent if mx >= 0.0 else -extent
                snapped.append((wall_x, y0, wall_x, y1))
        return snapped

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
        h, w = frame.shape[:2]
        correction_lines = [
            ln for ln in lines
            if not self._line_midpoint_in_image_edge(ln, w, h)
        ]
        if not correction_lines:
            self.pub_raw_segments.publish(Float32MultiArray())
            self.pub_segments.publish(Float32MultiArray())
            self.get_logger().info(
                f"mask wall observations image={len(lines)} correction=0 edge_rejected={len(lines)}",
                throttle_duration_sec=1.0,
            )
            return
        self._update_wall_anchor(self._base_line_points(correction_lines))
        rx, ry, rth = self._pose
        ct, st = math.cos(rth), math.sin(rth)

        projected: list[tuple[float, float, float, float]] = []
        correction_projected: list[tuple[float, float, float, float]] = []
        edge_rejected = len(lines) - len(correction_lines)
        for ln in correction_lines:
            base = self._to_base(np.array([[ln[0], ln[1]], [ln[2], ln[3]]], np.float64))
            if base is None:
                continue
            (bx0, by0), (bx1, by1) = base
            if not np.all(np.isfinite(base)):
                continue
            if max(math.hypot(bx0, by0), math.hypot(bx1, by1)) > self.max_range:
                continue
            fx0, fy0 = rx + bx0 * ct - by0 * st, ry + bx0 * st + by0 * ct
            fx1, fy1 = rx + bx1 * ct - by1 * st, ry + bx1 * st + by1 * ct
            projected.append((fx0, fy0, fx1, fy1))
            correction_projected.append((fx0, fy0, fx1, fy1))
            if len(projected) >= 24:
                break
        # The map should show exactly the observations that are eligible for pose correction.
        # Camera overlay still publishes all mask lines, including edge lines, for debugging.
        snapped = self._snap_segments_to_field(correction_projected)
        raw_msg = Float32MultiArray()
        for x0, y0, x1, y1 in projected:
            raw_msg.data.extend([float(x0), float(y0), float(x1), float(y1)])
        self.pub_raw_segments.publish(raw_msg)
        seg_msg = Float32MultiArray()
        for x0, y0, x1, y1 in snapped:
            seg_msg.data.extend([float(x0), float(y0), float(x1), float(y1)])
        self.pub_segments.publish(seg_msg)
        self._field_wall_correction(correction_projected)
        self.get_logger().info(
            f"mask wall observations image={len(lines)} projected={len(seg_msg.data) // 4} "
            f"correction={len(correction_projected)} edge_rejected={edge_rejected}",
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
