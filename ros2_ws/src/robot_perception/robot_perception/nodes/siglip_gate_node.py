"""SigLIP body-cam gate: zero-shot fruit classification + cube image-face discriminator.

Purpose (rulebook §6/§7): a Set2 mispick costs -40, so before the arm commits we run a
conservative fine-grained gate on the body-cam crop of the primary detection. SigLIP scores
zero-shot prompts over the announced fruit classes PLUS a "cube with a fruit picture" vs
"plain white cube" discriminator. The picked fruit must match today's Set2 label, clear the
confidence threshold, AND show a visible image face for is_target to be true.

Subscribes:
  /camera_body/image_raw   sensor_msgs/Image (bgr8)          — keep latest frame
  /camera_body/detections  robot_interfaces/DetectionArray   — triggers classification
  /mission_state           robot_interfaces/MissionState     — selects local profile
  (wide_classify_enabled only)
  /camera_top/image_raw    sensor_msgs/Image (bgr8)          — keep latest wide frame
  /camera_top/detections   robot_interfaces/DetectionArray   — wide Set2 hint candidates
  /world_model             robot_interfaces/WorldModel       — stop once all Set2 labeled
Publishes:
  /classification/siglip   robot_interfaces/Classification   (source="siglip")
  /classification/siglip_wide_hint  std_msgs/String (JSON)   — wide ROUTING hints only;
      deliberately a separate topic so the mission FSM's CLASSIFY pick gate (which
      subscribes to /classification/siglip only) can never consume a wide read.

Additional paths (all default-off, the original body pipeline is untouched):
  • TRANSIT (transit_classify_enabled): while driving (SCAN/APPROACH/...) body fruit cubes
    are still classified, but low-rate (transit_classify_min_interval_sec) and with
    stricter crop-size/detection-confidence gates against motion blur.
  • WIDE (wide_classify_enabled): far Set2 cubes seen by the top cam are classified at a
    slow rate and published as pixel-tagged hints for world_model track routing.
  • COLOR (color_filter_enabled): an HSV consistency check (fruit_color_gate) can veto a
    read whose crop colours clearly contradict the label (conservative, veto-only).

Dry-run safe: transformers/torch import or model load failure leaves the node alive and
advertising; callbacks early-return and one warning is logged.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
from typing import Callable, Iterable, Optional, TypeVar

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, String

from robot_interfaces.msg import (
    Classification,
    Detection,
    DetectionArray,
    MissionState,
    WorldModel,
)
from robot_perception.fruit_color_gate import ColorGateConfig, evaluate_color_consistency
from robot_perception.spatial_siglip import build_body_siglip_source
from robot_perception.wide_fruit_hint import WIDE_FRUIT_HINT_TOPIC, build_wide_hint

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

COMPETITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


LOCAL_ANCHOR_MISSION_STATES = frozenset(
    {"LOCAL_ANCHOR_INSPECTION", "LOCAL_ANCHOR_ALIGN"}
)
FINAL_CLASSIFY_MISSION_STATES = frozenset({"CLASSIFY"})
C = TypeVar("C")
D = TypeVar("D")


@dataclass(frozen=True)
class ClassificationProfile:
    """Runtime crop and inference-rate policy for one mission mode."""

    rate_hz: float
    prefer_centered_crop: bool
    crop_center_x_px: float
    max_batch: int
    crop_center_y_px: float | None = None


def classification_profile_for_state(
    mission_state: str,
    normal: ClassificationProfile,
    local_anchor: ClassificationProfile,
    final_classify: ClassificationProfile | None = None,
) -> ClassificationProfile:
    """Select local-anchor or final-pick crop policy for the current state."""
    state = str(mission_state).strip().upper()
    if state in LOCAL_ANCHOR_MISSION_STATES:
        return local_anchor
    if final_classify is not None and state in FINAL_CLASSIFY_MISSION_STATES:
        return final_classify
    return normal


def order_detections_for_profile(
    detections: list[Detection], profile: ClassificationProfile
) -> list[Detection]:
    """Return detections in the order in which crops should be attempted."""
    ordered = list(detections)
    if profile.prefer_centered_crop:
        if profile.crop_center_y_px is None:
            ordered.sort(
                key=lambda det: abs(
                    float(det.x_center) - profile.crop_center_x_px
                )
            )
        else:
            ordered.sort(
                key=lambda det: (
                    (float(det.x_center) - profile.crop_center_x_px) ** 2
                    + (float(det.y_center) - profile.crop_center_y_px) ** 2
                )
            )
    return ordered


def body_candidate_labels_for_state(
    mission_state: str,
    planning_phase: int,
    target_label: str,
    final_additional_labels: Iterable[str],
) -> frozenset[str]:
    """Allow generic close-up cube crops only for final Set2 verification."""
    labels = {str(target_label)}
    if (
        str(mission_state).strip().upper() in FINAL_CLASSIFY_MISSION_STATES
        and int(planning_phase) == 2
    ):
        labels.update(str(label) for label in final_additional_labels if str(label))
    return frozenset(labels)


def collect_crops_for_profile(
    detections: list[Detection],
    profile: ClassificationProfile,
    cropper: Callable[[Detection], Optional[C]],
) -> list[C]:
    """Crop in profile order, counting only valid crops toward the batch cap."""
    crops: list[C] = []
    for detection in order_detections_for_profile(detections, profile):
        crop = cropper(detection)
        if crop is not None:
            crops.append(crop)
        if len(crops) >= max(1, int(profile.max_batch)):
            break
    return crops


def collect_detection_crops_for_profile(
    detections: list[Detection],
    profile: ClassificationProfile,
    cropper: Callable[[Detection], Optional[C]],
) -> list[tuple[Detection, C]]:
    """Return valid ``(detection, crop)`` pairs without losing spatial identity."""
    pairs: list[tuple[Detection, C]] = []
    for detection in order_detections_for_profile(detections, profile):
        crop = cropper(detection)
        if crop is not None:
            pairs.append((detection, crop))
        if len(pairs) >= max(1, int(profile.max_batch)):
            break
    return pairs


def closest_stamped_frame(
    frames: Iterable[tuple[float, C]],
    stamp_sec: float,
    tolerance_sec: float,
) -> Optional[C]:
    """Return the buffered image nearest a detection stamp within the allowed skew."""
    buffered = list(frames)
    if not buffered:
        return None
    stamp = float(stamp_sec)
    if stamp <= 0.0:
        return buffered[-1][1]
    best_stamp, best_frame = min(buffered, key=lambda item: abs(item[0] - stamp))
    if abs(best_stamp - stamp) > max(0.0, float(tolerance_sec)):
        return None
    return best_frame


def drain_waiting_detections(
    waiting: Iterable[tuple[float, D]],
    frame_stamp_sec: float,
    tolerance_sec: float,
    max_age_sec: float,
) -> tuple[list[D], list[tuple[float, D]]]:
    """Release detections when their image arrives; prune only definitively old entries."""
    matched: list[D] = []
    retained: list[tuple[float, D]] = []
    frame_stamp = float(frame_stamp_sec)
    tolerance = max(0.0, float(tolerance_sec))
    max_age = max(tolerance, float(max_age_sec))
    for detection_stamp, payload in waiting:
        stamp = float(detection_stamp)
        if stamp <= 0.0 or abs(stamp - frame_stamp) <= tolerance:
            matched.append(payload)
        elif stamp >= frame_stamp - max_age:
            retained.append((stamp, payload))
    return matched, retained


class FractionalRateGate:
    """
    Down-sample one fast ROS timer to a requested profile rate.

    A timer created at the maximum configured profile rate can service both the
    normal and local-anchor modes.  Fractional credit avoids elapsed-time jitter
    turning a 3 Hz timer into an accidental 1.5 Hz timer when callbacks arrive a
    fraction early.
    """

    def __init__(self, timer_rate_hz: float) -> None:
        self.timer_rate_hz = max(0.0, float(timer_rate_hz))
        self._credit = 0.0

    def reset(self) -> None:
        self._credit = 0.0

    def ready(self, requested_rate_hz: float) -> bool:
        requested = max(0.0, float(requested_rate_hz))
        if requested <= 0.0 or self.timer_rate_hz <= 0.0:
            return False
        self._credit += min(1.0, requested / self.timer_rate_hz)
        if self._credit + 1e-12 < 1.0:
            return False
        self._credit = max(0.0, self._credit - 1.0)
        return True


@dataclass(frozen=True)
class CropScore:
    """Per-crop SigLIP read: fruit label + reliability margin + face discriminator."""

    label: str
    margin: float
    best_soft: float
    image_face_visible: bool


def is_transit_state(mission_state: str, transit_states: frozenset) -> bool:
    """Whether the (normalized) mission state counts as a driving/transit state."""
    return str(mission_state).strip().upper() in transit_states


def transit_gate_ready(last_run_sec: float, now_sec: float, min_interval_sec: float) -> bool:
    """Whether the low-rate transit path may run another SigLIP pass."""
    return (float(now_sec) - float(last_run_sec)) >= max(0.0, float(min_interval_sec))


def detection_passes_motion_gate(
    width: float, height: float, confidence: float, min_side_px: float, min_conf: float
) -> bool:
    """
    Gate a detection for classification while the robot may be MOVING.

    Motion blur shrinks/smears far boxes first, so require BOTH a minimum crop side
    length (bigger crops survive blur) and a minimum detector confidence (YOLO conf
    drops sharply on blurred cubes).
    """
    w = float(width)
    h = float(height)
    conf = float(confidence)
    if not (np.isfinite(w) and np.isfinite(h) and np.isfinite(conf)):
        return False
    return min(w, h) >= max(0.0, float(min_side_px)) and conf >= float(min_conf)


def order_wide_candidates(detections: Iterable, max_batch: int) -> list:
    """Largest-area-first order for wide hint crops, capped at max_batch entries."""
    ordered = sorted(
        detections,
        key=lambda det: float(det.width) * float(det.height),
        reverse=True,
    )
    return ordered[: max(1, int(max_batch))]


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
                "a printed picture of a {fruit} on a white cube",
                "a close-up photo of a {fruit} printed on a cube",
                "an angled photo of a printed {fruit}",
                "a tilted printed {fruit} picture",
                "an oblique view of a {fruit} picture on paper",
                "a wide crop of a printed {fruit}",
                "a tall crop of a printed {fruit}",
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
        self.declare_parameter("square_pad_crops", True)
        # SigLIP runs on a TIMER at this rate over the latest body frame's fruit-cube crops — NOT
        # synchronously per detection message. Firing on every body detection (up to ~16 Hz) stole
        # the single GPU from YOLO right during the approach; the world model only needs a fruit
        # vote a couple times/sec. All croppable fruit cubes in the frame go in ONE batched forward
        # pass. 0 -> fall back to the old per-detection synchronous path.
        self.declare_parameter("classify_rate_hz", 1.5)
        self.declare_parameter("max_batch", 4)            # cap crops per forward pass (VRAM)
        self.declare_parameter("prefer_centered_crop", False)
        self.declare_parameter("crop_center_x_px", 320.0)
        # The local-anchor inspection needs the isolated-test behaviour without changing normal
        # APPROACH/CLASSIFY throughput. A mission-state-selected profile keeps the
        # override local.
        self.declare_parameter("local_anchor_classify_rate_hz", 3.0)
        self.declare_parameter("local_anchor_max_batch", 1)
        self.declare_parameter("local_anchor_prefer_centered_crop", True)
        self.declare_parameter("local_anchor_crop_center_x_px", 320.0)
        # Final pick verification must classify exactly the centred object at the gripper,
        # never the clearest of several fruit cubes elsewhere in the Body frame.
        self.declare_parameter("final_classify_rate_hz", 3.0)
        self.declare_parameter("final_classify_max_batch", 1)
        self.declare_parameter("final_classify_prefer_centered_crop", True)
        self.declare_parameter("final_classify_crop_center_x_px", 320.0)
        self.declare_parameter("final_classify_crop_center_y_px", 345.0)
        # Close to the gripper, the detector often changes a fruit-photo cube to generic `cube`
        # because the printed face fills/tilts out of its dedicated box. SigLIP still sees the
        # fruit in the outer cube crop, so CLASSIFY phase 2 must consider both labels.
        self.declare_parameter("final_classify_additional_labels", ["cube"])
        # YOLO detections carry the source image stamp.  Buffer raw frames and pair by stamp
        # instead of cropping a delayed bbox from whichever newer frame happens to be latest.
        self.declare_parameter("frame_buffer_size", 12)  # legacy shared fallback
        # At 30 fps, 12 frames retain only 0.4 s. The combined Wide+Body YOLO process can
        # return a DetectionArray 1-3 s after capture under GPU contention, so the exact
        # source frame was routinely gone before the bbox arrived. Body is kept longer
        # because its exact crop is the final pick authority; Wide remains a routing hint.
        self.declare_parameter("body_frame_buffer_size", 96)
        self.declare_parameter("wide_frame_buffer_size", 72)
        # At 30 fps adjacent captures are ~33 ms apart. Keep the exact source frame when it is
        # available, but allow one neighbouring frame when sensor-data QoS drops that sample.
        self.declare_parameter("frame_match_tolerance_sec", 0.050)
        self.declare_parameter("detection_frame_wait_max_age_sec", 0.25)
        # --- TRANSIT body classification (default OFF; additional path, normal profiles
        # untouched). While the robot is driving (SCAN/APPROACH/...) body fruit cubes are
        # still worth a background read, but blur makes small/low-conf boxes worthless, so
        # this path is low-rate (min interval) with stricter crop-size/conf gates.
        self.declare_parameter("transit_classify_enabled", False)
        self.declare_parameter("transit_classify_min_interval_sec", 0.5)
        self.declare_parameter("transit_classify_min_crop_px", 48)
        self.declare_parameter("transit_classify_min_det_conf", 0.60)
        self.declare_parameter(
            "transit_classify_states", ["SCAN", "APPROACH", "DRIVE_TO_STORAGE"]
        )
        # --- WIDE background classification (default OFF). Far Set2 cubes seen by the top
        # cam get a slow-rate SigLIP read; results go out ONLY as pixel-tagged JSON hints on
        # WIDE_FRUIT_HINT_TOPIC (never on /classification/siglip -> can never trip the FSM's
        # CLASSIFY pick gate). Stops as soon as every Set2 track already has a fruit_label.
        self.declare_parameter("wide_classify_enabled", False)
        self.declare_parameter("wide_classify_rate_hz", 1.0)
        self.declare_parameter("wide_classify_min_crop_px", 28)
        self.declare_parameter("wide_classify_min_det_conf", 0.55)
        self.declare_parameter("wide_classify_max_batch", 2)
        self.declare_parameter("wide_hint_min_margin", 0.15)
        # --- HSV colour-consistency veto (default OFF; body + wide). Conservative: only
        # rejects a read whose crop colours clearly contradict the label. See
        # robot_perception.fruit_color_gate for the exact rule and threshold semantics.
        self.declare_parameter("color_filter_enabled", False)
        self.declare_parameter("color_filter_min_expected_fraction", 0.10)
        self.declare_parameter("color_filter_dominant_min_fraction", 0.40)
        self.declare_parameter("color_filter_dominance_ratio", 3.0)
        self.declare_parameter("color_filter_min_colored_fraction", 0.05)
        self.declare_parameter("color_filter_sat_min", 60.0)
        self.declare_parameter("color_filter_val_min", 60.0)
        self.declare_parameter("color_filter_veto_scale", 0.25)

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
        self.square_pad_crops = bool(self.get_parameter("square_pad_crops").value)
        self.classify_rate_hz = float(self.get_parameter("classify_rate_hz").value)
        self.max_batch = int(self.get_parameter("max_batch").value)
        self.prefer_centered_crop = bool(
            self.get_parameter("prefer_centered_crop").value
        )
        self.crop_center_x_px = float(self.get_parameter("crop_center_x_px").value)
        self.normal_profile = ClassificationProfile(
            rate_hz=self.classify_rate_hz,
            prefer_centered_crop=self.prefer_centered_crop,
            crop_center_x_px=self.crop_center_x_px,
            max_batch=max(1, self.max_batch),
        )
        self.local_anchor_profile = ClassificationProfile(
            rate_hz=float(self.get_parameter("local_anchor_classify_rate_hz").value),
            prefer_centered_crop=bool(
                self.get_parameter("local_anchor_prefer_centered_crop").value
            ),
            crop_center_x_px=float(
                self.get_parameter("local_anchor_crop_center_x_px").value
            ),
            max_batch=max(1, int(self.get_parameter("local_anchor_max_batch").value)),
        )
        self.final_classify_profile = ClassificationProfile(
            rate_hz=float(self.get_parameter("final_classify_rate_hz").value),
            prefer_centered_crop=bool(
                self.get_parameter("final_classify_prefer_centered_crop").value
            ),
            crop_center_x_px=float(
                self.get_parameter("final_classify_crop_center_x_px").value
            ),
            max_batch=max(1, int(self.get_parameter("final_classify_max_batch").value)),
            crop_center_y_px=float(
                self.get_parameter("final_classify_crop_center_y_px").value
            ),
        )
        self.final_classify_additional_labels = tuple(
            str(label)
            for label in self.get_parameter(
                "final_classify_additional_labels"
            ).value
        )
        positive_rates = [
            profile.rate_hz
            for profile in (
                self.normal_profile,
                self.local_anchor_profile,
                self.final_classify_profile,
            )
            if profile.rate_hz > 0.0
        ]
        self._classification_timer_rate_hz = max(positive_rates, default=0.0)
        self._rate_gate = FractionalRateGate(self._classification_timer_rate_hz)

        self.transit_classify_enabled = bool(
            self.get_parameter("transit_classify_enabled").value
        )
        self.transit_classify_min_interval_sec = float(
            self.get_parameter("transit_classify_min_interval_sec").value
        )
        self.transit_classify_min_crop_px = int(
            self.get_parameter("transit_classify_min_crop_px").value
        )
        self.transit_classify_min_det_conf = float(
            self.get_parameter("transit_classify_min_det_conf").value
        )
        self._transit_states = frozenset(
            str(state).strip().upper()
            for state in self.get_parameter("transit_classify_states").value
            if str(state).strip()
        )
        self._last_transit_classify_sec = float("-inf")

        self.wide_classify_enabled = bool(self.get_parameter("wide_classify_enabled").value)
        self.wide_classify_rate_hz = float(self.get_parameter("wide_classify_rate_hz").value)
        self.wide_classify_min_crop_px = int(
            self.get_parameter("wide_classify_min_crop_px").value
        )
        self.wide_classify_min_det_conf = float(
            self.get_parameter("wide_classify_min_det_conf").value
        )
        self.wide_classify_max_batch = max(
            1, int(self.get_parameter("wide_classify_max_batch").value)
        )
        self.wide_hint_min_margin = float(self.get_parameter("wide_hint_min_margin").value)

        self.color_filter_enabled = bool(self.get_parameter("color_filter_enabled").value)
        self.color_filter_veto_scale = float(
            self.get_parameter("color_filter_veto_scale").value
        )
        self.color_gate_config = ColorGateConfig(
            min_expected_fraction=float(
                self.get_parameter("color_filter_min_expected_fraction").value
            ),
            dominant_min_fraction=float(
                self.get_parameter("color_filter_dominant_min_fraction").value
            ),
            dominance_ratio=float(self.get_parameter("color_filter_dominance_ratio").value),
            min_colored_fraction=float(
                self.get_parameter("color_filter_min_colored_fraction").value
            ),
            saturation_min=float(self.get_parameter("color_filter_sat_min").value),
            value_min=float(self.get_parameter("color_filter_val_min").value),
        )

        self.bridge = CvBridge()
        # Inference is synchronous and can occupy a ROS callback for hundreds of milliseconds.
        # Camera ingestion therefore gets its own callback group on a multithreaded executor, so
        # raw frames keep entering the buffers while Body or Wide SigLIP is running.
        self._image_callback_group = MutuallyExclusiveCallbackGroup()
        self._frame_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.frame_buffer_size = max(2, int(self.get_parameter("frame_buffer_size").value))
        self.body_frame_buffer_size = max(
            self.frame_buffer_size,
            int(self.get_parameter("body_frame_buffer_size").value),
        )
        self.wide_frame_buffer_size = max(
            self.frame_buffer_size,
            int(self.get_parameter("wide_frame_buffer_size").value),
        )
        self.frame_match_tolerance_sec = max(
            0.0, float(self.get_parameter("frame_match_tolerance_sec").value)
        )
        self.detection_frame_wait_max_age_sec = max(
            self.frame_match_tolerance_sec,
            float(self.get_parameter("detection_frame_wait_max_age_sec").value),
        )
        self._body_frames: deque[tuple[float, np.ndarray]] = deque(
            maxlen=self.body_frame_buffer_size
        )
        self._wide_frames: deque[tuple[float, np.ndarray]] = deque(
            maxlen=self.wide_frame_buffer_size
        )
        self._body_detections_waiting: deque[tuple[float, DetectionArray]] = deque(maxlen=16)
        self._wide_detections_waiting: deque[tuple[float, DetectionArray]] = deque(maxlen=16)
        self._competition_state = "STANDBY"
        self._mission_state = ""
        self._planning_phase = 1
        # Latest fruit-cube candidates awaiting a throttled, batched SigLIP pass:
        # (frame, detections, stamp).
        self._pending: Optional[tuple] = None
        # WIDE path state: latest top-cam frame + pending hint candidates + whether the
        # world model still contains a Set2 track without a fruit_label (GPU-saving gate;
        # permissive until the first /world_model message arrives).
        self.latest_wide_frame: Optional[np.ndarray] = None
        self._wide_pending: Optional[tuple] = None
        self._unlabeled_set2_present = True

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

        self.create_subscription(
            Image,
            "/camera_body/image_raw",
            self.on_image,
            qos_profile_sensor_data,
            callback_group=self._image_callback_group,
        )  # match camera BEST_EFFORT
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_detections, 10)
        self.create_subscription(
            String, "/competition/state", self.on_competition_state, COMPETITION_QOS
        )
        self.create_subscription(MissionState, "/mission_state", self.on_mission_state, 10)
        self.create_subscription(Int8, "/planning/phase", self.on_planning_phase, 10)
        self.pub = self.create_publisher(Classification, "/classification/siglip", 10)
        # Wide hints go out on their OWN topic (JSON String): the FSM pick gate subscribes
        # to /classification/siglip only, so a wide read can never be mistaken for the
        # body-cam CLASSIFY verdict. Advertised even while disabled (stable topology).
        self.pub_wide_hint = self.create_publisher(String, WIDE_FRUIT_HINT_TOPIC, 10)
        if self.wide_classify_enabled:
            self.create_subscription(
                Image,
                "/camera_top/image_raw",
                self.on_wide_image,
                qos_profile_sensor_data,
                callback_group=self._image_callback_group,
            )
            self.create_subscription(
                DetectionArray, "/camera_top/detections", self.on_wide_detections, 10
            )
            self.create_subscription(WorldModel, "/world_model", self.on_world_model, 10)
            self.create_timer(
                1.0 / max(0.1, self.wide_classify_rate_hz), self._tick_wide_classify
            )

        # Run one timer at the fastest configured profile. FractionalRateGate keeps the ordinary
        # mission at its original rate while allowing the local-anchor profile to reach 3 Hz.
        if self._classification_timer_rate_hz > 0.0:
            self.create_timer(
                1.0 / self._classification_timer_rate_hz,
                self._tick_classify,
            )

        self.get_logger().info(
            f"model_id='{self.model_id}' device={self.device} "
            f"set2_label='{self.set2_label}' fruits={self.fruit_labels} "
            f"fruit_prompts={len(self.fruit_prompts)} pooling={self.fruit_prompt_pooling} "
            f"thresh={self.confidence_threshold} min_box_area={self.min_box_area} "
            f"rates(normal={self.normal_profile.rate_hz:.2f}Hz, "
            f"local_anchor={self.local_anchor_profile.rate_hz:.2f}Hz) "
            f"buffers(body={self.body_frame_buffer_size},wide={self.wide_frame_buffer_size}) "
            f"transit={self.transit_classify_enabled} "
            f"wide={self.wide_classify_enabled} color={self.color_filter_enabled}"
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
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            stamp_sec = self._stamp_sec(msg.header.stamp)
            with self._frame_lock:
                self.latest_frame = frame
                self._body_frames.append((stamp_sec, frame))
                matched, retained = drain_waiting_detections(
                    self._body_detections_waiting,
                    stamp_sec,
                    self.frame_match_tolerance_sec,
                    self.detection_frame_wait_max_age_sec,
                )
                self._body_detections_waiting = deque(retained, maxlen=16)
            for detection_msg in matched:
                self._store_body_detection(detection_msg, frame)
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}", throttle_duration_sec=2.0)

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_competition_state(self, msg: String) -> None:
        state = str(msg.data).strip().upper()
        if state in {"STANDBY", "READY", "RUNNING", "DONE", "ERROR"}:
            changed = state != self._competition_state
            self._competition_state = state
            if changed:
                self._rate_gate.reset()
            if state != "RUNNING":
                SiglipGateNode._set_pending_value(self, "_pending", None)
                SiglipGateNode._set_pending_value(self, "_wide_pending", None)
                with self._frame_lock:
                    self._body_detections_waiting.clear()
                    self._wide_detections_waiting.clear()

    def on_mission_state(self, msg: MissionState) -> None:
        """Switch crop/rate policy without leaking a stale crop across mission modes."""
        state = str(msg.state).strip().upper()
        previous_profile = self._active_profile()
        self._mission_state = state
        if self._active_profile() is not previous_profile:
            SiglipGateNode._set_pending_value(self, "_pending", None)
            self._rate_gate.reset()

    def on_planning_phase(self, msg: Int8) -> None:
        """Track whether final verification belongs to Set1 or Set2."""
        self._planning_phase = int(msg.data)

    def _active_profile(self) -> ClassificationProfile:
        return classification_profile_for_state(
            self._mission_state,
            self.normal_profile,
            self.local_anchor_profile,
            self.final_classify_profile,
        )

    def _set_pending_value(self, name: str, value) -> None:
        lock = getattr(self, "_pending_lock", None)
        if lock is None:
            setattr(self, name, value)
            return
        with lock:
            setattr(self, name, value)

    def _take_pending_value(self, name: str):
        lock = getattr(self, "_pending_lock", None)
        if lock is None:
            value = getattr(self, name, None)
            setattr(self, name, None)
            return value
        with lock:
            value = getattr(self, name, None)
            setattr(self, name, None)
            return value

    def _store_body_detection(self, msg: DetectionArray, frame: np.ndarray) -> None:
        """Bind an already stamp-matched Body DetectionArray to its source image."""
        if self._competition_state != "RUNNING" or self.model is None or self.processor is None:
            return
        candidate_labels = body_candidate_labels_for_state(
            self._mission_state,
            self._planning_phase,
            self.target_label,
            self.final_classify_additional_labels,
        )
        targets = [
            detection
            for detection in msg.detections
            if str(detection.label) in candidate_labels
        ]
        if not targets:
            return
        SiglipGateNode._set_pending_value(
            self, "_pending", (frame, targets, msg.header.stamp)
        )
        if self._active_profile().rate_hz <= 0.0:
            self._tick_classify(force=True)

    def on_detections(self, msg: DetectionArray) -> None:
        if self._competition_state != "RUNNING":
            return
        if self.model is None or self.processor is None:
            self.get_logger().warn(
                "detection received but SigLIP unavailable; skipping",
                throttle_duration_sec=10.0,
            )
            return
        stamp_sec = self._stamp_sec(msg.header.stamp)
        with self._frame_lock:
            frame = closest_stamped_frame(
                self._body_frames,
                stamp_sec,
                self.frame_match_tolerance_sec,
            )
            if frame is None:
                self._body_detections_waiting.append((stamp_sec, msg))
        if frame is None:
            self.get_logger().warn(
                "body DetectionArray arrived before matching image; queued for stamp match",
                throttle_duration_sec=2.0,
            )
            return
        self._store_body_detection(msg, frame)

    def _transit_active(self) -> bool:
        """Whether the low-rate transit path governs classification right now."""
        return self.transit_classify_enabled and is_transit_state(
            self._mission_state, self._transit_states
        )

    def _tick_classify(self, force: bool = False) -> None:
        """Timer: crop every pending fruit cube and score them in one batched SigLIP pass."""
        if self._competition_state != "RUNNING":
            SiglipGateNode._set_pending_value(self, "_pending", None)
            return
        if getattr(self, "_pending", None) is None or self.model is None or self.processor is None:
            return
        profile = self._active_profile()
        transit = self._transit_active()
        if transit:
            # ADDITIONAL transit path: while driving, replace the profile rate gate with the
            # slower min-interval gate; the stationary-state profiles are untouched.
            now = self.get_clock().now().nanoseconds * 1e-9
            if not force and not transit_gate_ready(
                self._last_transit_classify_sec, now,
                self.transit_classify_min_interval_sec,
            ):
                return
        elif not force and not self._rate_gate.ready(profile.rate_hz):
            return
        pending = SiglipGateNode._take_pending_value(self, "_pending")
        if pending is None:
            return
        frame, dets, stamp = pending
        h, w = frame.shape[:2]
        if transit:
            # Motion-blur defence: while moving, only large, confidently-detected boxes
            # are worth GPU time (small/low-conf boxes are smeared and SigLIP would guess).
            def cropper(det: Detection) -> Optional[np.ndarray]:
                if not detection_passes_motion_gate(
                    det.width, det.height, det.confidence,
                    self.transit_classify_min_crop_px, self.transit_classify_min_det_conf,
                ):
                    return None
                return self._crop(frame, det, w, h)
        else:
            def cropper(det: Detection) -> Optional[np.ndarray]:
                return self._crop(frame, det, w, h)
        pairs = collect_detection_crops_for_profile(dets, profile, cropper)
        if pairs:
            self._classify_and_publish(
                [pair[1] for pair in pairs],
                stamp,
                [pair[0] for pair in pairs],
            )
            if transit:
                self._last_transit_classify_sec = self.get_clock().now().nanoseconds * 1e-9

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
        crop = frame[y1:y2, x1:x2]
        return self._square_pad(crop) if self.square_pad_crops else crop

    @staticmethod
    def _square_pad(crop: np.ndarray) -> np.ndarray:
        h, w = crop.shape[:2]
        if h <= 0 or w <= 0 or h == w:
            return crop
        side = max(h, w)
        pad_y = side - h
        pad_x = side - w
        top = pad_y // 2
        bottom = pad_y - top
        left = pad_x // 2
        right = pad_x - left
        return cv2.copyMakeBorder(
            crop,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )

    def _score_crops(self, crops: list[np.ndarray]) -> Optional[list[CropScore]]:
        """One batched SigLIP forward pass; per-crop fruit label/margin/face scores."""
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
            return None

        n_fruit_prompts = len(self.fruit_prompts)
        n_fruits = len(self.fruit_prompt_groups)
        scores: list[CropScore] = []
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
            scores.append(CropScore(label, margin, best_soft, image_face_visible))
        return scores

    def _color_veto(self, crop: np.ndarray, label: str) -> bool:
        """Conservative HSV veto: True only when crop colours contradict the label."""
        if not self.color_filter_enabled:
            return False
        result = evaluate_color_consistency(crop, label, self.color_gate_config)
        if result.veto:
            self.get_logger().info(
                f"color veto: {label} expected={result.expected_fraction:.2f} "
                f"dominant={result.dominant_label}:{result.dominant_fraction:.2f}",
                throttle_duration_sec=2.0,
            )
        return result.veto

    def _classify_and_publish(
        self,
        crops: list[np.ndarray],
        stamp,
        detections: list[Detection] | None = None,
    ) -> None:
        """Score one batch and publish one spatially tagged result per Body crop."""
        scores = self._score_crops(crops)
        if not scores:
            return
        if detections is not None and len(detections) != len(crops):
            self.get_logger().error("Body SigLIP crop/detection count mismatch; dropping batch")
            return
        published = 0
        for index, (crop, score) in enumerate(zip(crops, scores)):
            margin = score.margin
            vetoed = self._color_veto(crop, score.label)
            if vetoed:
                # Downgrade only (veto-only filter): the read still publishes, but with a
                # scaled-down margin and never as a pick-approving target.
                margin *= self.color_filter_veto_scale
            is_target = (
                score.label == self.set2_label
                and margin >= self.confidence_threshold
                and score.image_face_visible
                and not vetoed
            )
            out = Classification()
            # Preserve the capture stamp and the exact YOLO pixel that produced this crop.
            # World-model/FSM consumers can therefore bind every batched read spatially.
            out.header.stamp = stamp
            out.header.frame_id = "camera_body"
            out.label = score.label
            out.set_type = 2
            out.confidence = float(margin)
            out.is_target = is_target
            out.image_face_visible = score.image_face_visible
            if detections is None:
                out.source = "siglip"
            else:
                det = detections[index]
                out.source = build_body_siglip_source(det.x_center, det.y_center)
            self.pub.publish(out)
            published += 1
            capture_sec = self._stamp_sec(stamp)
            self.get_logger().info(
                f"siglip: {score.label} soft={score.best_soft:.2f} margin={margin:.2f} "
                f"face={score.image_face_visible} target={is_target} "
                f"crop={index + 1}/{len(crops)} source={out.source} "
                f"capture={capture_sec:.3f}",
                throttle_duration_sec=1.0,
            )
        if published > 1:
            self.get_logger().debug(f"published {published} spatial Body SigLIP reads")

    # ------------------------------------------------------------- wide hint path

    def on_wide_image(self, msg: Image) -> None:
        """Keep the latest top-cam frame for wide hint crops."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            stamp_sec = self._stamp_sec(msg.header.stamp)
            with self._frame_lock:
                self.latest_wide_frame = frame
                self._wide_frames.append((stamp_sec, frame))
                matched, retained = drain_waiting_detections(
                    self._wide_detections_waiting,
                    stamp_sec,
                    self.frame_match_tolerance_sec,
                    self.detection_frame_wait_max_age_sec,
                )
                self._wide_detections_waiting = deque(retained, maxlen=16)
            for detection_msg in matched:
                self._store_wide_detection(detection_msg, frame)
        except Exception as exc:
            self.get_logger().warn(
                f"wide image conversion failed: {exc}", throttle_duration_sec=2.0
            )

    def on_world_model(self, msg: WorldModel) -> None:
        """Track whether any Set2 track still lacks a fruit_label (wide-path GPU gate)."""
        self._unlabeled_set2_present = any(
            int(obj.set_type) == 2 and not str(obj.fruit_label) and not obj.blacklisted
            for obj in msg.objects
        )

    def _store_wide_detection(self, msg: DetectionArray, frame: np.ndarray) -> None:
        """Bind an already stamp-matched Wide DetectionArray to its source image."""
        if self._competition_state != "RUNNING" or self.model is None or self.processor is None:
            return
        targets = [
            det
            for det in msg.detections
            if str(det.label) == self.target_label
            and detection_passes_motion_gate(
                det.width,
                det.height,
                det.confidence,
                self.wide_classify_min_crop_px,
                self.wide_classify_min_det_conf,
            )
        ]
        if targets:
            SiglipGateNode._set_pending_value(
                self, "_wide_pending", (frame, targets, msg.header.stamp)
            )

    def on_wide_detections(self, msg: DetectionArray) -> None:
        """Store wide fruit-cube candidates; the slow wide timer runs the inference."""
        if self._competition_state != "RUNNING":
            return
        if self.model is None or self.processor is None:
            return
        stamp_sec = self._stamp_sec(msg.header.stamp)
        with self._frame_lock:
            frame = closest_stamped_frame(
                self._wide_frames,
                stamp_sec,
                self.frame_match_tolerance_sec,
            )
            if frame is None:
                self._wide_detections_waiting.append((stamp_sec, msg))
        if frame is None:
            self.get_logger().warn(
                "wide DetectionArray arrived before matching image; queued for stamp match",
                throttle_duration_sec=2.0,
            )
            return
        self._store_wide_detection(msg, frame)

    def _crop_wide(self, frame: np.ndarray, det: Detection, w: int, h: int
                   ) -> Optional[np.ndarray]:
        """Crop a wide detection: clamp, re-check the min side, keep the aspect gate."""
        x1 = max(0, min(int(round(det.x_center - det.width / 2.0)), w))
        y1 = max(0, min(int(round(det.y_center - det.height / 2.0)), h))
        x2 = max(0, min(int(round(det.x_center + det.width / 2.0)), w))
        y2 = max(0, min(int(round(det.y_center + det.height / 2.0)), h))
        if x2 <= x1 or y2 <= y1:
            return None
        bw, bh = x2 - x1, y2 - y1
        if min(bw, bh) < self.wide_classify_min_crop_px:
            return None
        ar = bw / float(bh)
        if not (self.aspect_ratio_min <= ar <= self.aspect_ratio_max):
            return None
        crop = frame[y1:y2, x1:x2]
        return self._square_pad(crop) if self.square_pad_crops else crop

    def _tick_wide_classify(self) -> None:
        """Slow timer: score pending wide Set2 crops and publish pixel-tagged hints."""
        if self._competition_state != "RUNNING":
            SiglipGateNode._set_pending_value(self, "_wide_pending", None)
            return
        if getattr(self, "_wide_pending", None) is None or self.model is None or self.processor is None:
            return
        if not self._unlabeled_set2_present:
            SiglipGateNode._set_pending_value(self, "_wide_pending", None)
            return
        pending = SiglipGateNode._take_pending_value(self, "_wide_pending")
        if pending is None:
            return
        frame, dets, stamp = pending
        h, w = frame.shape[:2]
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        kept: list[Detection] = []
        crops: list[np.ndarray] = []
        for det in order_wide_candidates(dets, self.wide_classify_max_batch):
            crop = self._crop_wide(frame, det, w, h)
            if crop is not None:
                kept.append(det)
                crops.append(crop)
        if not crops:
            return
        scores = self._score_crops(crops)
        if not scores:
            return
        published = 0
        published_details: list[str] = []
        for det, crop, score in zip(kept, crops, scores):
            if not score.image_face_visible:      # blank/edge face -> no fruit to hint
                continue
            if score.margin < self.wide_hint_min_margin:
                continue
            if self._color_veto(crop, score.label):
                continue
            hint = String()
            hint.data = build_wide_hint(
                score.label, score.margin,
                float(det.x_center), float(det.y_center), stamp_sec,
            )
            self.pub_wide_hint.publish(hint)
            published += 1
            published_details.append(
                f"{score.label}@({det.x_center:.0f},{det.y_center:.0f}) "
                f"m={score.margin:.2f} t={stamp_sec:.3f}"
            )
        if published:
            self.get_logger().info(
                f"wide hint: published {published}/{len(crops)} "
                f"[{'; '.join(published_details)}]",
                throttle_duration_sec=2.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SiglipGateNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
