"""Custom YOLOv8n detector over two camera streams (replaces yolo_world_node).

One shared YOLOv8n model (models/cube.pt, custom-trained on the 4 white polyhedra:
cube / octahedron / dodecahedron / icosahedron) serves both cameras:
  • top  (wide-angle)  -> /camera_top/detections   (world-model object positions)
  • body (eye-in-hand) -> /camera_body/detections  (visual servo + classification)

Because the model is custom-trained with fixed classes there is no set_classes step (unlike
the deprecated YOLO-World). predict() on the shared model is serialized with a lock so the
two image callbacks do not interleave.

It ALSO replaces shape_heuristic_node: the best body-cam detection is republished as a
robot_interfaces/Classification on /classification/shape (set_type=1, source='yolo'), so the
FSM's existing Set1 shape gate keeps working unchanged. Set2 fruit gating still comes from
siglip_gate_node, which reads /camera_body/detections (Set2 objects are fruit-picture cubes,
so cube.pt detects the box and SigLIP reads the printed fruit face).

Dry-run safe: if ultralytics/torch or the weights are missing, the node still constructs,
advertises all topics, logs one error, and the image callbacks early-return.
"""
from __future__ import annotations

import threading
from typing import Any

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from robot_interfaces.msg import Classification, Detection, DetectionArray

# Heavy deps guarded for dry-run. On ImportError the node still runs (idle).
try:
    import torch
    from ultralytics import YOLO

    _YOLO_AVAILABLE = True
except Exception:  # noqa: BLE001 - any import failure must keep node alive
    torch = None  # type: ignore[assignment]
    YOLO = None  # type: ignore[assignment, misc]
    _YOLO_AVAILABLE = False


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        self.declare_parameter(
            "model_path", "/home/seventt/seventt/workspace/models/cube.pt"
        )
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("top_min_interval_sec", 0.12)    # ~8 Hz
        self.declare_parameter("body_min_interval_sec", 0.06)   # ~16 Hz
        # Republish the best body detection as /classification/shape (replaces shape_heuristic).
        self.declare_parameter("publish_shape_classification", True)
        self.declare_parameter("shape_topic", "/classification/shape")

        self.model_path = str(self.get_parameter("model_path").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.top_min_interval = float(self.get_parameter("top_min_interval_sec").value)
        self.body_min_interval = float(self.get_parameter("body_min_interval_sec").value)
        self.publish_shape = bool(self.get_parameter("publish_shape_classification").value)

        self.device = self._resolve_device(str(self.get_parameter("device").value))

        self.bridge = CvBridge()
        self.model: Any | None = None
        self._model_lock = threading.Lock()   # predict() on shared model must not interleave
        self._last_top = 0.0
        self._last_body = 0.0

        self._load_model()

        self.pub_top = self.create_publisher(DetectionArray, "/camera_top/detections", 10)
        self.pub_body = self.create_publisher(DetectionArray, "/camera_body/detections", 10)
        self.pub_shape = None
        if self.publish_shape:
            self.pub_shape = self.create_publisher(
                Classification, str(self.get_parameter("shape_topic").value), 10
            )

        self.create_subscription(Image, "/camera_top/image_raw", self.on_top_image, 10)
        self.create_subscription(Image, "/camera_body/image_raw", self.on_body_image, 10)

        self.get_logger().info(
            f"model='{self.model_path}' device={self.device} conf={self.conf_threshold} "
            f"imgsz={self.imgsz} shape_pub={self.publish_shape}"
        )

    # ------------------------------------------------------------------ setup
    def _resolve_device(self, requested: str) -> str:
        if not _YOLO_AVAILABLE or torch is None:
            return "cpu"
        if requested == "auto":
            try:
                return "cuda" if torch.cuda.is_available() else "cpu"
            except Exception:  # noqa: BLE001
                return "cpu"
        return requested

    def _load_model(self) -> None:
        if not _YOLO_AVAILABLE:
            self.get_logger().error(
                "ultralytics/torch unavailable; running idle (no inference). "
                "Detection topics will be advertised but stay empty."
            )
            return
        try:
            model = YOLO(self.model_path)
            try:
                model.to(self.device)
            except Exception:  # noqa: BLE001 - device move is best-effort
                self.get_logger().warn(
                    f"could not move model to device='{self.device}'", throttle_duration_sec=10.0
                )
            self.model = model
            names = getattr(model, "names", None)
            self.get_logger().info(f"YOLOv8n loaded from '{self.model_path}' classes={names}")
        except Exception as exc:  # noqa: BLE001 - never raise out of __init__
            self.model = None
            self.get_logger().error(
                f"failed to load weights '{self.model_path}': {exc}; running idle."
            )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- callbacks
    def on_top_image(self, msg: Image) -> None:
        now = self._now_sec()
        if (now - self._last_top) < self.top_min_interval:
            return
        self._last_top = now
        self._process(msg, "camera_top", self.pub_top)

    def on_body_image(self, msg: Image) -> None:
        now = self._now_sec()
        if (now - self._last_body) < self.body_min_interval:
            return
        self._last_body = now
        arr = self._process(msg, "camera_body", self.pub_body)
        if arr is not None and self.pub_shape is not None:
            self._publish_shape(arr)

    def _process(self, msg: Image, frame_id: str, pub) -> DetectionArray | None:
        if self.model is None or not _YOLO_AVAILABLE:
            return None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"cv_bridge failed on {frame_id}: {exc}", throttle_duration_sec=5.0
            )
            return None
        try:
            with self._model_lock:
                results = self.model.predict(
                    frame, conf=self.conf_threshold, imgsz=self.imgsz, verbose=False
                )
        except Exception as exc:  # noqa: BLE001 - inference must not kill the node
            self.get_logger().warn(
                f"inference failed on {frame_id}: {exc}", throttle_duration_sec=5.0
            )
            return None

        arr = self._build_array(results, frame_id)
        pub.publish(arr)
        return arr

    def _build_array(self, results: Any, frame_id: str) -> DetectionArray:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        arr = DetectionArray()
        arr.header = header

        if not results:
            return arr
        res = results[0]
        boxes = getattr(res, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return arr

        names = getattr(res, "names", None)
        try:
            xywh = boxes.xywh.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clses = boxes.cls.cpu().numpy()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"failed to read boxes: {exc}", throttle_duration_sec=5.0)
            return arr

        for i in range(xywh.shape[0]):
            det = Detection()
            det.label = self._label_for(int(clses[i]), names)
            det.confidence = float(confs[i])
            det.x_center = float(xywh[i][0])
            det.y_center = float(xywh[i][1])
            det.width = float(xywh[i][2])
            det.height = float(xywh[i][3])
            det.source_frame = frame_id
            arr.detections.append(det)
        return arr

    def _publish_shape(self, arr: DetectionArray) -> None:
        """Best body detection -> Classification on /classification/shape (replaces shape_heuristic).

        The FSM compares shape.label to today's set1_label itself, so is_target stays False here.
        """
        if not arr.detections:
            return
        best = max(arr.detections, key=lambda d: d.confidence)
        c = Classification()
        c.header = arr.header
        c.label = best.label
        c.set_type = 1                 # Set1 shapes
        c.confidence = best.confidence
        c.is_target = False            # FSM does the today's-target comparison
        c.image_face_visible = False   # SigLIP owns the fruit-face discriminator
        c.source = "yolo"
        self.pub_shape.publish(c)

    @staticmethod
    def _label_for(idx: int, names: Any) -> str:
        if isinstance(names, dict) and idx in names:
            return str(names[idx])
        if isinstance(names, (list, tuple)) and 0 <= idx < len(names):
            return str(names[idx])
        return str(idx)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
