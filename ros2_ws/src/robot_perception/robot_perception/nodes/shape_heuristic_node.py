"""DEPRECATED — superseded by yolo_detector_node (custom YOLOv8n cube.pt).

Kept on disk for reference only; no longer registered in setup.py or any launch file.
The Set1 shape on /classification/shape now comes from yolo_detector_node (best body
detection republished, source='yolo'). Do not add this back to the pipeline.

Classical-CV backup for Set1 shape classification (SigLIP weak on polyhedra).

Counts the edges of the largest visible face on the body-cam detection crop using
cv2.approxPolyDP, then maps that edge count to a polyhedron shape:

  n==4 -> "cube"
  n==5 -> "dodecahedron"
  n==3 -> octahedron / icosahedron, disambiguated by how many small triangular
          sub-faces are visible (many -> icosahedron, few -> octahedron).

Publishes robot_interfaces/Classification on /classification/shape with
source="shape_heuristic", set_type=1, image_face_visible=False. Consumed by
mission_fsm_node alongside the SigLIP gate (rulebook §6/§7). cv2 is always
available, so there is no heavy-model dry-run path; the node simply does not
publish when no detection or no frame is available.
"""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_interfaces.msg import Classification, Detection, DetectionArray


# Edge-count -> shape (largest visible face of the polyhedron).
_EDGES_TO_SHAPE = {4: "cube", 5: "dodecahedron"}


class ShapeHeuristicNode(Node):
    """Edge-count shape classifier over the primary body-cam detection crop."""

    def __init__(self) -> None:
        super().__init__("shape_heuristic_node")

        # Today's announced Set1 shape: cube|octahedron|dodecahedron|icosahedron.
        self.declare_parameter("set1_label", "cube")
        self.declare_parameter("confidence_threshold", 0.6)
        self.declare_parameter("approx_epsilon_ratio", 0.02)
        self.declare_parameter("min_area", 500)

        self.set1_label = str(self.get_parameter("set1_label").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.approx_epsilon_ratio = float(self.get_parameter("approx_epsilon_ratio").value)
        self.min_area = float(self.get_parameter("min_area").value)

        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None

        self.create_subscription(Image, "/camera_body/image_raw", self.on_image, 10)
        self.create_subscription(
            DetectionArray, "/camera_body/detections", self.on_detections, 10
        )
        self.pub = self.create_publisher(Classification, "/classification/shape", 10)

        self.get_logger().info(
            f"set1_label='{self.set1_label}' conf_thresh={self.confidence_threshold} "
            f"eps_ratio={self.approx_epsilon_ratio} min_area={self.min_area}"
        )

    # ------------------------------------------------------------------ callbacks
    def on_image(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - keep node alive on bad frames
            self.get_logger().warn(f"imgmsg_to_cv2 failed: {exc}", throttle_duration_sec=2.0)

    def on_detections(self, msg: DetectionArray) -> None:
        if self.latest_frame is None:
            return  # dry-run: no frame yet, simply don't publish
        if not msg.detections:
            return

        det = self._primary_detection(msg.detections)
        crop = self._crop(self.latest_frame, det)
        if crop is None:
            return  # too small / out of bounds: skip (node stays alive)

        result = self._classify(crop)
        if result is None:
            return  # could not extract a usable contour
        label, confidence = result

        out = Classification()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_body"
        out.label = label
        out.set_type = 1
        out.confidence = float(confidence)
        out.is_target = bool(label == self.set1_label and confidence >= self.confidence_threshold)
        out.image_face_visible = False
        out.source = "shape_heuristic"
        self.pub.publish(out)

    # ------------------------------------------------------------------- helpers
    def _primary_detection(self, detections: list[Detection]) -> Detection:
        """Highest confidence; tie-break by proximity to the image center."""
        if self.latest_frame is None:
            return detections[0]
        h, w = self.latest_frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        def key(d: Detection) -> tuple[float, float]:
            dist = (d.x_center - cx) ** 2 + (d.y_center - cy) ** 2
            return (float(d.confidence), -dist)  # higher conf, then nearer center

        return max(detections, key=key)

    def _crop(self, frame: np.ndarray, det: Detection) -> Optional[np.ndarray]:
        """Clamp the detection bbox to the frame and return the crop, or None."""
        h, w = frame.shape[:2]
        x0 = int(round(det.x_center - det.width / 2.0))
        y0 = int(round(det.y_center - det.height / 2.0))
        x1 = int(round(det.x_center + det.width / 2.0))
        y1 = int(round(det.y_center + det.height / 2.0))
        x0 = max(0, min(x0, w - 1))
        y0 = max(0, min(y0, h - 1))
        x1 = max(0, min(x1, w))
        y1 = max(0, min(y1, h))
        if x1 <= x0 or y1 <= y0:
            return None
        if (x1 - x0) * (y1 - y0) < self.min_area:
            return None
        return frame[y0:y1, x0:x1]

    def _classify(self, crop: np.ndarray) -> Optional[tuple[str, float]]:
        """Edge-count the largest face contour and map to a shape + confidence."""
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        # Dilate to close small gaps so the outer face contour is continuous.
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        face = max(contours, key=cv2.contourArea)
        face_area = cv2.contourArea(face)
        if face_area < self.min_area:
            return None

        peri = cv2.arcLength(face, True)
        if peri <= 0.0:
            return None
        approx = cv2.approxPolyDP(face, self.approx_epsilon_ratio * peri, True)
        n = len(approx)

        # Polygon-fit quality: solidity (face area / convex-hull area) in [0, 1].
        hull = cv2.convexHull(face)
        hull_area = cv2.contourArea(hull)
        solidity = float(face_area / hull_area) if hull_area > 0.0 else 0.0
        solidity = max(0.0, min(1.0, solidity))

        if n == 3:
            label = self._disambiguate_triangular(contours, face_area)
            vertex_fit = 1.0  # the visible face is genuinely a triangle
        elif n in _EDGES_TO_SHAPE:
            label = _EDGES_TO_SHAPE[n]
            vertex_fit = 1.0
        else:
            # Unexpected edge count: snap to the nearest known face and penalize.
            label, ideal = self._nearest_known(n)
            vertex_fit = max(0.0, 1.0 - abs(n - ideal) / float(max(ideal, 1)))

        confidence = max(0.0, min(1.0, 0.5 * solidity + 0.5 * vertex_fit))
        if confidence < 0.25:
            label = ""  # too uncertain to commit to a shape
        return label, confidence

    def _disambiguate_triangular(self, contours: list, face_area: float) -> str:
        """n==3 is octahedron or icosahedron; count small triangular sub-faces.

        An icosahedron shows many small triangular facets per visible face; an
        octahedron shows few. Count child-ish triangular contours that are small
        relative to the dominant face and that approximate triangles.
        """
        small_triangles = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area * 0.25:
                continue  # noise
            if area >= face_area * 0.6:
                continue  # the dominant face itself (or near-duplicates)
            peri = cv2.arcLength(c, True)
            if peri <= 0.0:
                continue
            sub = cv2.approxPolyDP(c, self.approx_epsilon_ratio * peri, True)
            if len(sub) == 3:
                small_triangles += 1
        return "icosahedron" if small_triangles >= 3 else "octahedron"

    @staticmethod
    def _nearest_known(n: int) -> tuple[str, int]:
        """Map an unexpected vertex count to the nearest ideal-face shape."""
        ideals = {3: "octahedron", 4: "cube", 5: "dodecahedron"}
        best = min(ideals, key=lambda k: abs(k - n))
        return ideals[best], best


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShapeHeuristicNode()
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
