"""Custom YOLOv8n detector over two camera streams (replaces yolo_world_node).

Per-stream models (classes as of 2026-07-11:
cube / octahedron / dodecahedron / icosahedron / fruit_photo_cube / arrival):
  • top  (wide-angle)  uses top_model_path  (models/wide.pt) -> /camera_top/detections
  • body (base-fixed)  uses body_model_path (models/cube.pt) -> /camera_body/detections
Both models are trained on their own camera's domain, so each stream uses the matching one;
set top/body to the same path (or leave them empty to fall back to model_path) to share a
single instance. predict() across both models is serialized with one lock (single GPU).

It ALSO replaces shape_heuristic_node: the best body-cam SHAPE detection is republished as a
robot_interfaces/Classification on /classification/shape (set_type=1, source='yolo') for the
FSM's Set1 gate. Set2 boxes are the fruit_photo_cube class — YOLO establishes they are fruit
boxes and they are excluded from the shape stream; siglip_gate_node reads the printed fruit
face for the specific fruit type.

Dry-run safe: if ultralytics/torch or the weights are missing, the node still constructs,
advertises all topics, logs one error, and the image callbacks early-return.
"""
from __future__ import annotations

import threading
from typing import Any

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

        # Per-stream models (wide.pt / cube.pt). `model_path` is the
        # legacy single-model fallback: if top_model_path / body_model_path are empty, both
        # streams use model_path (backward compatible with the old single-model config).
        self.declare_parameter(
            "model_path", "/home/seventt/seventt/workspace/models/cube.pt"
        )
        self.declare_parameter("top_model_path", "")     # wide cam; "" -> model_path
        self.declare_parameter("body_model_path", "")    # body cam; "" -> model_path
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("imgsz", 640)
        # Per-stream inference size: MUST match each model's TRAINING imgsz or accuracy drops
        # (wide.pt trained @1280, cube.pt @640). 0 -> fall back to the shared imgsz.
        self.declare_parameter("top_imgsz", 0)
        self.declare_parameter("body_imgsz", 0)
        # FP16 inference: ~1.5-2x faster on the Orin's tensor cores, negligible accuracy loss.
        self.declare_parameter("half", True)
        self.declare_parameter("device", "auto")
        self.declare_parameter("top_min_interval_sec", 0.12)    # ~8 Hz
        self.declare_parameter("body_min_interval_sec", 0.06)   # ~16 Hz
        # Republish the best body detection as /classification/shape (replaces shape_heuristic).
        self.declare_parameter("publish_shape_classification", True)
        self.declare_parameter("shape_topic", "/classification/shape")
        # Which stream(s) this instance runs: "top" | "body" | "both". Running two single-camera
        # instances (top+body) parallelises inference across cores — one node serialises both models
        # on a single GIL thread, capping the slower (top@1280) stream. "both" = legacy single-node.
        self.declare_parameter("camera", "both")

        self.model_path = str(self.get_parameter("model_path").value)
        self.top_model_path = str(self.get_parameter("top_model_path").value) or self.model_path
        self.body_model_path = str(self.get_parameter("body_model_path").value) or self.model_path
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.top_imgsz = int(self.get_parameter("top_imgsz").value) or self.imgsz
        self.body_imgsz = int(self.get_parameter("body_imgsz").value) or self.imgsz
        self.half = bool(self.get_parameter("half").value)
        self.top_min_interval = float(self.get_parameter("top_min_interval_sec").value)
        self.body_min_interval = float(self.get_parameter("body_min_interval_sec").value)
        self.publish_shape = bool(self.get_parameter("publish_shape_classification").value)
        cam = str(self.get_parameter("camera").value).lower()
        self._want_top = cam in ("top", "both")
        self._want_body = cam in ("body", "both")

        self.device = self._resolve_device(str(self.get_parameter("device").value))

        self.bridge = CvBridge()
        # One lock serializes predict() across BOTH models (single GPU).
        self._model_lock = threading.Lock()
        self._last_top = 0.0
        self._last_body = 0.0

        # Load only the model(s) this instance serves.
        self.top_model = self._load_model(self.top_model_path, "top") if self._want_top else None
        if not self._want_body:
            self.body_model = None
        elif self._want_top and self.body_model_path == self.top_model_path:
            self.body_model = self.top_model      # same weights -> share (saves ~1 model in memory)
        else:
            self.body_model = self._load_model(self.body_model_path, "body")

        self.pub_top = self.create_publisher(DetectionArray, "/camera_top/detections", 10) \
            if self._want_top else None
        self.pub_body = self.create_publisher(DetectionArray, "/camera_body/detections", 10) \
            if self._want_body else None
        self.pub_shape = None
        if self.publish_shape and self._want_body:     # shape stream comes from the body detector
            self.pub_shape = self.create_publisher(
                Classification, str(self.get_parameter("shape_topic").value), 10
            )

        # SENSOR_DATA QoS must match the camera publisher (BEST_EFFORT) or no frames arrive.
        if self._want_top:
            self.create_subscription(Image, "/camera_top/image_raw", self.on_top_image,
                                     qos_profile_sensor_data)
        if self._want_body:
            self.create_subscription(Image, "/camera_body/image_raw", self.on_body_image,
                                     qos_profile_sensor_data)

        streams = "+".join([s for s, on in (("top", self._want_top), ("body", self._want_body)) if on])
        self.get_logger().info(
            f"streams={streams} "
            f"top_model='{self.top_model_path if self._want_top else '-'}' "
            f"body_model='{self.body_model_path if self._want_body else '-'}' "
            f"device={self.device} conf={self.conf_threshold} "
            f"imgsz(top={self.top_imgsz},body={self.body_imgsz}) "
            f"shape_pub={self.publish_shape}"
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

    def _load_model(self, path: str, tag: str) -> Any | None:
        if not _YOLO_AVAILABLE:
            self.get_logger().error(
                "ultralytics/torch unavailable; running idle (no inference). "
                "Detection topics will be advertised but stay empty."
            )
            return None
        try:
            model = YOLO(path)
            try:
                model.to(self.device)
            except Exception:  # noqa: BLE001 - device move is best-effort
                self.get_logger().warn(
                    f"could not move {tag} model to device='{self.device}'",
                    throttle_duration_sec=10.0,
                )
            names = getattr(model, "names", None)
            self.get_logger().info(f"YOLOv8n {tag} loaded from '{path}' classes={names}")
            return model
        except Exception as exc:  # noqa: BLE001 - never raise out of __init__
            self.get_logger().error(
                f"failed to load {tag} weights '{path}': {exc}; that stream runs idle."
            )
            return None

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- callbacks
    def on_top_image(self, msg: Image) -> None:
        now = self._now_sec()
        if (now - self._last_top) < self.top_min_interval:
            return
        self._last_top = now
        self._process(msg, "camera_top", self.pub_top, self.top_model, self.top_imgsz)

    def on_body_image(self, msg: Image) -> None:
        now = self._now_sec()
        if (now - self._last_body) < self.body_min_interval:
            return
        self._last_body = now
        arr = self._process(msg, "camera_body", self.pub_body, self.body_model, self.body_imgsz)
        if arr is not None and self.pub_shape is not None:
            self._publish_shape(arr)

    def _process(self, msg: Image, frame_id: str, pub, model: Any | None,
                 imgsz: int) -> DetectionArray | None:
        if model is None or not _YOLO_AVAILABLE:
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
                results = model.predict(
                    frame, conf=self.conf_threshold, imgsz=imgsz, half=self.half, verbose=False
                )
        except Exception as exc:  # noqa: BLE001 - inference must not kill the node
            self.get_logger().warn(
                f"inference failed on {frame_id}: {exc}", throttle_duration_sec=5.0
            )
            return None

        arr = self._build_array(results, frame_id, msg.header.stamp)
        pub.publish(arr)
        return arr

    def _build_array(self, results: Any, frame_id: str, stamp) -> DetectionArray:
        header = Header()
        # Propagate the source image's CAPTURE stamp (not now()): the 1280 fisheye inference takes
        # ~80-150 ms, and world_model uses this stamp to project each detection at the robot pose AT
        # CAPTURE TIME. Stamping now() here would smear the map by the inference delay while rotating.
        header.stamp = stamp
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
        """Best body SHAPE detection -> Classification on /classification/shape (Set1 gate).

        The FSM compares shape.label to today's set1_label itself, so is_target stays False here.
        fruit_photo_cube is a Set2 box (SigLIP owns it), and arrival is a finish landmark,
        so both are excluded from the Set1 shape stream.
        """
        non_shapes = {"fruit_photo_cube", "arrival"}
        shapes = [d for d in arr.detections if d.label not in non_shapes]
        if not shapes:
            return
        best = max(shapes, key=lambda d: d.confidence)
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
