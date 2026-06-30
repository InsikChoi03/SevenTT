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
        self.declare_parameter("confidence_threshold", 0.7)
        self.declare_parameter("device", "auto")
        self.declare_parameter("min_box_area", 400)

        self.model_id = str(self.get_parameter("model_id").value)
        self.set2_label = str(self.get_parameter("set2_label").value)
        self.fruit_labels = [str(f) for f in self.get_parameter("fruit_labels").value]
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        device_param = str(self.get_parameter("device").value)
        self.min_box_area = int(self.get_parameter("min_box_area").value)

        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None

        # Text prompts: fruit prompts first, then the two discriminator prompts.
        self.fruit_prompts = [f"a photo of a {fruit}" for fruit in self.fruit_labels]
        self.prompts = self.fruit_prompts + [PROMPT_IMAGE_FACE, PROMPT_PLAIN_CUBE]

        self.device = self._resolve_device(device_param)
        self.model = None
        self.processor = None
        self._load_model()

        self.create_subscription(Image, "/camera_body/image_raw", self.on_image, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_detections, 10)
        self.pub = self.create_publisher(Classification, "/classification/siglip", 10)

        self.get_logger().info(
            f"model_id='{self.model_id}' device={self.device} "
            f"set2_label='{self.set2_label}' fruits={self.fruit_labels} "
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
        if not msg.detections:
            return

        frame = self.latest_frame
        h, w = frame.shape[:2]

        primary = self._primary_detection(msg.detections, w, h)
        if primary is None:
            return

        crop = self._crop(frame, primary, w, h)
        if crop is None:
            return

        self._classify_and_publish(crop, msg.header.stamp)

    def _primary_detection(self, detections, w: int, h: int) -> Optional[Detection]:
        """Highest-confidence box; tie-break = closest to image center."""
        cx_img, cy_img = w / 2.0, h / 2.0
        best: Optional[Detection] = None
        best_conf = -1.0
        best_dist = float("inf")
        for det in detections:
            conf = float(det.confidence)
            dist = (float(det.x_center) - cx_img) ** 2 + (float(det.y_center) - cy_img) ** 2
            if conf > best_conf or (conf == best_conf and dist < best_dist):
                best = det
                best_conf = conf
                best_dist = dist
        return best

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
        if (x2 - x1) * (y2 - y1) < self.min_box_area:
            self.get_logger().info(
                f"box area {(x2 - x1) * (y2 - y1)} < min_box_area {self.min_box_area}; skipping",
                throttle_duration_sec=5.0,
            )
            return None
        return frame[y1:y2, x1:x2]

    def _classify_and_publish(self, crop_bgr: np.ndarray, stamp) -> None:
        try:
            crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(crop_rgb)
            inputs = self.processor(
                text=self.prompts, images=pil_img, padding="max_length", return_tensors="pt"
            )
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            with torch.no_grad():
                outputs = self.model(**inputs)
            # SigLIP is trained with a pairwise SIGMOID loss, so each prompt's match
            # probability is independent — use sigmoid(logit) per prompt, NOT softmax over
            # prompts. This keeps confidence_threshold meaningful ("P(this fruit) >= 0.7")
            # instead of being diluted across the prompt set, and keeps the cube
            # image-vs-plain discriminator independent of the fruit scores.
            probs = torch.sigmoid(outputs.logits_per_image)[0].detach().cpu().numpy()
        except Exception as exc:
            self.get_logger().warn(f"SigLIP inference failed: {exc}", throttle_duration_sec=5.0)
            return

        n_fruits = len(self.fruit_labels)
        fruit_probs = probs[:n_fruits]
        prob_image_face = float(probs[n_fruits])
        prob_plain_cube = float(probs[n_fruits + 1])

        best_idx = int(np.argmax(fruit_probs))
        best_label = self.fruit_labels[best_idx]
        best_conf = float(fruit_probs[best_idx])

        image_face_visible = prob_image_face > prob_plain_cube
        is_target = (
            best_label == self.set2_label
            and best_conf >= self.confidence_threshold
            and image_face_visible
        )

        out = Classification()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_body"
        out.label = best_label
        out.set_type = 2
        out.confidence = best_conf
        out.is_target = is_target
        out.image_face_visible = image_face_visible
        out.source = "siglip"
        self.pub.publish(out)

        self.get_logger().info(
            f"siglip: label={best_label} conf={best_conf:.3f} "
            f"face_visible={image_face_visible} is_target={is_target}"
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
