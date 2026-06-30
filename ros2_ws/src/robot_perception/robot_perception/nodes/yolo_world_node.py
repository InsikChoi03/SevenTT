"""DEPRECATED — superseded by yolo_detector_node (custom YOLOv8n cube.pt).

Kept on disk for reference only; no longer registered in setup.py or any launch file.
YOLO-World failed to detect the white polyhedra, so detection moved to a custom-trained
YOLOv8n. Do not add this back to the pipeline.

One shared YOLO-World over two camera streams with per-stream prompts.

A single YOLOWorld model serves both the wide-angle top camera and the body
eye-in-hand camera. Because the two streams use different open-vocabulary
prompt sets, ``set_classes`` is called immediately before each ``predict`` and
the whole set_classes+predict critical section is serialized with a lock so the
two callbacks never interleave on the shared model.

  • Top stream  → position-only prompts (e.g. "object on the floor"),
    published on ``/camera_top/detections``  (source_frame="camera_top").
  • Body stream → today's target classes (precise boxes for visual servo),
    published on ``/camera_body/detections`` (source_frame="camera_body").

Dry-run safe: if ultralytics/torch are missing or the weights fail to load, the
node still constructs, advertises both topics, logs one error, and the image
callbacks early-return (no inference) so the process stays alive without GPU,
weights, or cameras.
"""
from __future__ import annotations

import threading
from typing import Any

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from robot_interfaces.msg import Detection, DetectionArray

# Heavy deps guarded for dry-run. On ImportError the node still runs (idle).
try:
    import torch
    from ultralytics import YOLOWorld

    _YOLO_AVAILABLE = True
except Exception:  # noqa: BLE001 - any import failure must keep node alive
    torch = None  # type: ignore[assignment]
    YOLOWorld = None  # type: ignore[assignment, misc]
    _YOLO_AVAILABLE = False


class YoloWorldNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_world_node")

        self.declare_parameter("model_path", "yolov8s-world.pt")
        self.declare_parameter("top_classes", ["object on the floor"])
        self.declare_parameter(
            "body_classes", ["apple", "orange", "banana", "pineapple", "white cube"]
        )
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("top_min_interval_sec", 0.1)
        self.declare_parameter("body_min_interval_sec", 0.04)

        self.model_path = str(self.get_parameter("model_path").value)
        self.top_classes = [str(c) for c in self.get_parameter("top_classes").value]
        self.body_classes = [str(c) for c in self.get_parameter("body_classes").value]
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.top_min_interval = float(self.get_parameter("top_min_interval_sec").value)
        self.body_min_interval = float(self.get_parameter("body_min_interval_sec").value)

        self.device = self._resolve_device(str(self.get_parameter("device").value))

        self.bridge = CvBridge()
        self.model: Any | None = None
        # Guards the shared model: set_classes + predict must not interleave.
        self._model_lock = threading.Lock()
        # Which prompt list is currently armed on the shared model.
        self._active_classes: list[str] | None = None
        # Re-apply prompts on next predict after a runtime parameter change.
        self._top_dirty = True
        self._body_dirty = True

        # Throttle timestamps (seconds, from rclpy clock).
        self._last_top = 0.0
        self._last_body = 0.0

        self._load_model()

        self.pub_top = self.create_publisher(DetectionArray, "/camera_top/detections", 10)
        self.pub_body = self.create_publisher(DetectionArray, "/camera_body/detections", 10)

        self.create_subscription(Image, "/camera_top/image_raw", self.on_top_image, 10)
        self.create_subscription(Image, "/camera_body/image_raw", self.on_body_image, 10)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"model='{self.model_path}' device={self.device} "
            f"top_classes={self.top_classes} body_classes={self.body_classes} "
            f"conf={self.conf_threshold} imgsz={self.imgsz}"
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
                "Both detection topics will be advertised but stay empty."
            )
            return
        try:
            model = YOLOWorld(self.model_path)
            try:
                model.to(self.device)
            except Exception:  # noqa: BLE001 - device move is best-effort
                self.get_logger().warn(
                    f"could not move model to device='{self.device}'", throttle_duration_sec=10.0
                )
            self.model = model
            self.get_logger().info(f"YOLO-World loaded from '{self.model_path}'")
        except Exception as exc:  # noqa: BLE001 - never raise out of __init__
            self.model = None
            self.get_logger().error(
                f"failed to load YOLO-World weights '{self.model_path}': {exc}; running idle."
            )

    # --------------------------------------------------------------- runtime
    def _on_set_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "top_classes":
                self.top_classes = [str(c) for c in p.value]
                self._top_dirty = True
                self.get_logger().info(f"top_classes updated -> {self.top_classes}")
            elif p.name == "body_classes":
                self.body_classes = [str(c) for c in p.value]
                self._body_dirty = True
                self.get_logger().info(f"body_classes updated -> {self.body_classes}")
            elif p.name == "conf_threshold":
                self.conf_threshold = float(p.value)
            elif p.name == "imgsz":
                self.imgsz = int(p.value)
        return SetParametersResult(successful=True)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- callbacks
    def on_top_image(self, msg: Image) -> None:
        now = self._now_sec()
        if (now - self._last_top) < self.top_min_interval:
            return
        self._last_top = now
        self._process(msg, self.top_classes, "top", "camera_top", self.pub_top)

    def on_body_image(self, msg: Image) -> None:
        now = self._now_sec()
        if (now - self._last_body) < self.body_min_interval:
            return
        self._last_body = now
        self._process(msg, self.body_classes, "body", "camera_body", self.pub_body)

    def _process(self, msg: Image, classes: list[str], stream: str, frame_id: str, pub) -> None:
        if self.model is None or not _YOLO_AVAILABLE:
            return
        if not classes:
            self.get_logger().warn(
                f"{stream} stream has no prompt classes; skipping", throttle_duration_sec=10.0
            )
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"cv_bridge failed on {stream}: {exc}", throttle_duration_sec=5.0
            )
            return

        # Serialize set_classes + predict on the shared model across both streams.
        try:
            with self._model_lock:
                self._arm_classes(classes, stream)
                results = self.model.predict(
                    frame, conf=self.conf_threshold, imgsz=self.imgsz, verbose=False
                )
        except Exception as exc:  # noqa: BLE001 - inference must not kill the node
            self.get_logger().warn(
                f"inference failed on {stream}: {exc}", throttle_duration_sec=5.0
            )
            return

        out = self._build_array(results, classes, frame_id)
        pub.publish(out)

    def _arm_classes(self, classes: list[str], stream: str) -> None:
        """Apply ``classes`` to the shared model if not already armed/dirty.

        Called only while holding ``self._model_lock``.
        """
        dirty = self._top_dirty if stream == "top" else self._body_dirty
        if self._active_classes == classes and not dirty:
            return
        self.model.set_classes(classes)
        self._active_classes = list(classes)
        if stream == "top":
            self._top_dirty = False
        else:
            self._body_dirty = False

    def _build_array(self, results: Any, classes: list[str], frame_id: str) -> DetectionArray:
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
            self.get_logger().warn(
                f"failed to read boxes: {exc}", throttle_duration_sec=5.0
            )
            return arr

        for i in range(xywh.shape[0]):
            det = Detection()
            det.label = self._label_for(int(clses[i]), classes, names)
            det.confidence = float(confs[i])
            det.x_center = float(xywh[i][0])
            det.y_center = float(xywh[i][1])
            det.width = float(xywh[i][2])
            det.height = float(xywh[i][3])
            det.source_frame = frame_id
            arr.detections.append(det)
        return arr

    @staticmethod
    def _label_for(idx: int, classes: list[str], names: Any) -> str:
        if isinstance(names, dict) and idx in names:
            return str(names[idx])
        if isinstance(names, (list, tuple)) and 0 <= idx < len(names):
            return str(names[idx])
        if 0 <= idx < len(classes):
            return classes[idx]
        return str(idx)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloWorldNode()
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
