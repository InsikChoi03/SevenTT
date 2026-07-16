"""SigLIP body-cam gate: zero-shot fruit classification + cube image-face discriminator.

Purpose (rulebook §6/§7): a Set2 mispick costs -40, so before the arm commits we run a
conservative fine-grained gate on the body-cam crop of the primary detection. SigLIP scores
zero-shot prompts over the announced fruit classes PLUS a "cube with a fruit picture" vs
"plain white cube" discriminator. The picked fruit must match today's Set2 label, clear the
confidence threshold, AND show a visible image face for is_target to be true.

Subscribes:
  /camera_body/image_raw   sensor_msgs/Image (bgr8)          — keep latest frame
  /camera_body/detections  robot_interfaces/DetectionArray   — triggers classification
Publishes:
  /classification/siglip   robot_interfaces/Classification   (source="siglip")

Dry-run safe: transformers/torch import or model load failure leaves the node alive and
advertising; callbacks early-return and one warning is logged.
"""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from robot_interfaces.msg import Classification, Detection, DetectionArray

try:
    import torch
    from PIL import Image as PILImage
    from transformers import AutoModel, AutoProcessor

    SIGLIP_AVAILABLE = True
    SIGLIP_IMPORT_ERROR: Optional[str] = None
except Exception as exc:  # ImportError or any transitive failure
    torch = None  # type: ignore[assignment]
    PILImage = None  # type: ignore[assignment]
    AutoModel = None  # type: ignore[assignment]
    AutoProcessor = None  # type: ignore[assignment]
    SIGLIP_AVAILABLE = False
    SIGLIP_IMPORT_ERROR = str(exc)


# Discriminator prompts (Set1 cube vs Set2 fruit-printed cube share the same shell).
PROMPT_IMAGE_FACE = "a white cube with a fruit picture on it"
PROMPT_PLAIN_CUBE = "a plain white cube with no image"


class SiglipGateNode(Node):
    def __init__(self) -> None:
        super().__init__("siglip_gate_node")

        self.declare_parameter("model_id", "google/siglip-base-patch16-224")
        self.declare_parameter("set2_label", "apple")
        self.declare_parameter("fruit_labels", ["apple", "orange", "banana", "pineapple"])
        self.declare_parameter(
            "fruit_prompt_templates",
            [
                "a photo of a {fruit}",
                "a printed photo of a {fruit}",
                "a {fruit} printed on paper",
            ],
        )
        self.declare_parameter("fruit_prompt_pooling", "max")  # max | mean
        self.declare_parameter("confidence_threshold", 0.7)
        self.declare_parameter("device", "auto")
        self.declare_parameter("min_box_area", 400)
        # Only fruit_photo_cube boxes go to SigLIP (fruit-type is meaningless on a shape). And the
        # crop must be "croppable": big enough (min_box_area) AND square-ish aspect (a fruit face
        # seen roughly head-on). A far/edge-on box is elongated or tiny -> skip (SigLIP would guess).
        self.declare_parameter("target_label", "fruit_photo_cube")
        self.declare_parameter("aspect_ratio_min", 0.6)   # w/h lower bound
        self.declare_parameter("aspect_ratio_max", 1.7)   # w/h upper bound
        # SigLIP runs on a TIMER at this rate over the latest body frame's fruit-cube crops — NOT
        # synchronously per detection message. Firing on every body detection (up to ~16 Hz) stole
        # the single GPU from YOLO right during the approach; the world model only needs a fruit
        # vote a couple times/sec. All croppable fruit cubes in the frame go in ONE batched forward
        # pass. 0 -> fall back to the old per-detection synchronous path.
        self.declare_parameter("classify_rate_hz", 1.5)
        self.declare_parameter("max_batch", 4)            # cap crops per forward pass (VRAM)

        self.model_id = str(self.get_parameter("model_id").value)
        self.set2_label = str(self.get_parameter("set2_label").value)
        self.fruit_labels = [str(f) for f in self.get_parameter("fruit_labels").value]
        self.fruit_prompt_templates = [
            str(p) for p in self.get_parameter("fruit_prompt_templates").value
        ] or ["a photo of a {fruit}"]
        self.fruit_prompt_pooling = str(self.get_parameter("fruit_prompt_pooling").value).lower()
        if self.fruit_prompt_pooling not in {"max", "mean"}:
            self.get_logger().warn(
                f"unknown fruit_prompt_pooling='{self.fruit_prompt_pooling}', using max"
            )
            self.fruit_prompt_pooling = "max"
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        device_param = str(self.get_parameter("device").value)
        self.min_box_area = int(self.get_parameter("min_box_area").value)
        self.target_label = str(self.get_parameter("target_label").value)
        self.aspect_ratio_min = float(self.get_parameter("aspect_ratio_min").value)
        self.aspect_ratio_max = float(self.get_parameter("aspect_ratio_max").value)
        self.classify_rate_hz = float(self.get_parameter("classify_rate_hz").value)
        self.max_batch = int(self.get_parameter("max_batch").value)

        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None
        # Latest fruit-cube candidates awaiting a (throttled, batched) SigLIP pass: (frame, dets, stamp).
        self._pending: Optional[tuple] = None

        # Text prompts: all fruit prompt variants first, then the two discriminator prompts. Fruit
        # variants are pooled back into one score per fruit before the fruit softmax/margin gate.
        self.fruit_prompt_groups: list[tuple[str, list[int]]] = []
        self.fruit_prompts: list[str] = []
        for fruit in self.fruit_labels:
            indices: list[int] = []
            for tmpl in self.fruit_prompt_templates:
                indices.append(len(self.fruit_prompts))
                self.fruit_prompts.append(tmpl.format(fruit=fruit))
            self.fruit_prompt_groups.append((fruit, indices))
        self.prompts = self.fruit_prompts + [PROMPT_IMAGE_FACE, PROMPT_PLAIN_CUBE]

        self.device = self._resolve_device(device_param)
        self.model = None
        self.processor = None
        self._load_model()

        self.create_subscription(Image, "/camera_body/image_raw", self.on_image,
                                 qos_profile_sensor_data)  # match camera BEST_EFFORT
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_detections, 10)
        self.pub = self.create_publisher(Classification, "/classification/siglip", 10)

        # Throttle: SigLIP fires on a timer over the freshest pending crops, not per detection msg.
        if self.classify_rate_hz > 0:
            self.create_timer(1.0 / self.classify_rate_hz, self._tick_classify)

        self.get_logger().info(
            f"model_id='{self.model_id}' device={self.device} "
            f"set2_label='{self.set2_label}' fruits={self.fruit_labels} "
            f"fruit_prompts={len(self.fruit_prompts)} pooling={self.fruit_prompt_pooling} "
            f"thresh={self.confidence_threshold} min_box_area={self.min_box_area}"
        )

    def _resolve_device(self, device_param: str) -> str:
        if not SIGLIP_AVAILABLE:
            return "cpu"
        if device_param == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if device_param == "cuda" and not torch.cuda.is_available():
            self.get_logger().warn("device='cuda' requested but CUDA unavailable; using CPU")
            return "cpu"
        return device_param

    def _load_model(self) -> None:
        if not SIGLIP_AVAILABLE:
            self.get_logger().error(
                f"transformers/torch unavailable ({SIGLIP_IMPORT_ERROR}); "
                "siglip_gate idle, no classifications will be published"
            )
            return
        try:
            self.processor = AutoProcessor.from_pretrained(self.model_id)
            self.model = AutoModel.from_pretrained(self.model_id)
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info(f"SigLIP model loaded on {self.device}")
        except Exception as exc:
            self.model = None
            self.processor = None
            self.get_logger().warn(
                f"SigLIP model load failed ({exc}); node alive but inference disabled"
            )

    def on_image(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}", throttle_duration_sec=2.0)

    def on_detections(self, msg: DetectionArray) -> None:
        if self.model is None or self.processor is None:
            self.get_logger().warn(
                "detection received but SigLIP unavailable; skipping",
                throttle_duration_sec=10.0,
            )
            return
        if self.latest_frame is None:
            self.get_logger().warn("no body-cam frame yet; skipping", throttle_duration_sec=5.0)
            return
        targets = [d for d in msg.detections if str(d.label) == self.target_label]
        if not targets:
            return
        # Store only — the timer runs the throttled, batched inference. This keeps the high-rate
        # detection stream off the GPU (it was stealing it from YOLO on every message).
        self._pending = (self.latest_frame, targets, msg.header.stamp)
        if self.classify_rate_hz <= 0:                    # throttle disabled -> classify inline
            self._tick_classify()

    def _tick_classify(self) -> None:
        """Timer: crop every pending fruit cube and score them in one batched SigLIP pass."""
        pending, self._pending = self._pending, None
        if pending is None or self.model is None or self.processor is None:
            return
        frame, dets, stamp = pending
        h, w = frame.shape[:2]
        crops = []
        for det in dets:
            c = self._crop(frame, det, w, h)
            if c is not None:
                crops.append(c)
            if len(crops) >= self.max_batch:
                break
        if crops:
            self._classify_and_publish(crops, stamp)

    def _crop(self, frame: np.ndarray, det: Detection, w: int, h: int) -> Optional[np.ndarray]:
        x1 = int(round(det.x_center - det.width / 2.0))
        y1 = int(round(det.y_center - det.height / 2.0))
        x2 = int(round(det.x_center + det.width / 2.0))
        y2 = int(round(det.y_center + det.height / 2.0))

        # Clamp to image bounds.
        x1 = max(0, min(x1, w))
        y1 = max(0, min(y1, h))
        x2 = max(0, min(x2, w))
        y2 = max(0, min(y2, h))

        if x2 <= x1 or y2 <= y1:
            return None
        bw, bh = x2 - x1, y2 - y1
        if bw * bh < self.min_box_area:
            self.get_logger().info(
                f"box area {bw * bh} < min_box_area {self.min_box_area}; skipping (너무 멀다)",
                throttle_duration_sec=5.0,
            )
            return None
        # Aspect gate: a fruit face seen head-on is roughly square; an edge-on / far box is skewed.
        ar = bw / float(bh)
        if not (self.aspect_ratio_min <= ar <= self.aspect_ratio_max):
            self.get_logger().info(
                f"aspect {ar:.2f} out of [{self.aspect_ratio_min},{self.aspect_ratio_max}]; "
                f"skipping (각도 나쁨/얼굴면 아님)",
                throttle_duration_sec=5.0,
            )
            return None
        return frame[y1:y2, x1:x2]

    def _classify_and_publish(self, crops: list[np.ndarray], stamp) -> None:
        """One batched SigLIP forward pass over every fruit-cube crop; publish the best read."""
        try:
            pil_imgs = [PILImage.fromarray(cv2.cvtColor(c, cv2.COLOR_BGR2RGB)) for c in crops]
            inputs = self.processor(
                text=self.prompts, images=pil_imgs, padding="max_length", return_tensors="pt"
            )
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            with torch.no_grad():
                outputs = self.model(**inputs)
            # logits_per_image: [n_crops, n_prompts]. SigLIP uses a pairwise SIGMOID loss, so each
            # prompt's match probability is independent (sigmoid per prompt, NOT softmax over prompts)
            # for the face-vs-plain discriminator; the fruit TYPE is a softmax over the fruit prompts.
            logits_all = outputs.logits_per_image.detach().cpu().numpy()
        except Exception as exc:
            self.get_logger().warn(f"SigLIP inference failed: {exc}", throttle_duration_sec=5.0)
            return

        n_fruit_prompts = len(self.fruit_prompts)
        n_fruits = len(self.fruit_prompt_groups)
        best = None          # (rank_key, label, margin, best_soft, image_face_visible, is_target)
        for logits in logits_all:
            sig = 1.0 / (1.0 + np.exp(-logits))          # sigmoid for the face discriminator
            image_face_visible = float(sig[n_fruit_prompts]) > float(sig[n_fruit_prompts + 1])
            # Fruit TYPE = pool prompt variants per fruit, then SOFTMAX over fruits. Reliability =
            # MARGIN over the runner-up fruit (bigger gap -> more trust), published as confidence.
            # (bigger gap -> more trust), published as `confidence` for the world model's vote weight.
            fruit_scores = []
            for _fruit, indices in self.fruit_prompt_groups:
                vals = logits[indices]
                if self.fruit_prompt_pooling == "mean":
                    fruit_scores.append(float(np.mean(vals)))
                else:
                    fruit_scores.append(float(np.max(vals)))
            fl = np.asarray(fruit_scores, dtype=np.float64)
            e = np.exp(fl - fl.max())
            soft = e / e.sum()
            order = np.argsort(soft)[::-1]
            best_idx = int(order[0])
            label = self.fruit_prompt_groups[best_idx][0]
            best_soft = float(soft[best_idx])
            margin = best_soft - float(soft[order[1]]) if n_fruits > 1 else best_soft
            is_target = (
                label == self.set2_label
                and margin >= self.confidence_threshold
                and image_face_visible
            )
            # Rank crops: prefer a visible fruit face, then the largest margin (clearest read).
            rank_key = (1 if image_face_visible else 0, margin)
            if best is None or rank_key > best[0]:
                best = (rank_key, label, margin, best_soft, image_face_visible, is_target)

        _, label, margin, best_soft, image_face_visible, is_target = best
        out = Classification()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_body"
        out.label = label
        out.set_type = 2
        out.confidence = float(margin)          # reliability = margin over the runner-up fruit
        out.is_target = is_target
        out.image_face_visible = image_face_visible
        out.source = "siglip"
        self.pub.publish(out)

        self.get_logger().info(
            f"siglip: {label} soft={best_soft:.2f} margin={margin:.2f} "
            f"face={image_face_visible} target={is_target} (batch={len(crops)})",
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SiglipGateNode()
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
