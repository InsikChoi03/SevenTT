from builtin_interfaces.msg import Time
import pytest
from rclpy.qos import HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from types import SimpleNamespace

from robot_perception.nodes.yolo_detector_node import (
    LATEST_IMAGE_QOS,
    center_inside_normalized_rect,
    collapse_nested_fruit_cube_detections,
    image_capture_age_sec,
    image_is_fresh_for_inference,
    YoloDetectorNode,
)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_kwargs):
        self.warnings.append(str(message))


def test_camera_qos_retains_only_the_latest_unread_frame():
    assert LATEST_IMAGE_QOS.history == HistoryPolicy.KEEP_LAST
    assert LATEST_IMAGE_QOS.depth == 1
    assert LATEST_IMAGE_QOS.reliability == ReliabilityPolicy.BEST_EFFORT


def test_capture_age_uses_the_original_camera_timestamp():
    stamp = Time(sec=10, nanosec=250_000_000)

    age = image_capture_age_sec(10.60, stamp.sec, stamp.nanosec)
    assert age == pytest.approx(0.35)
    assert image_capture_age_sec(10.60, 0, 0) is None


def test_stale_frame_is_rejected_before_inference():
    assert image_is_fresh_for_inference(10.20, 10, 0, 0.25)
    assert image_is_fresh_for_inference(10.25, 10, 0, 0.25)
    assert not image_is_fresh_for_inference(10.251, 10, 0, 0.25)


def test_small_future_clock_skew_is_allowed_but_invalid_stamp_is_not():
    assert image_is_fresh_for_inference(9.80, 10, 0, 0.25)
    assert not image_is_fresh_for_inference(9.70, 10, 0, 0.25)
    assert image_is_fresh_for_inference(10.0, 0, 0, 0.25)


def test_wide_self_mask_uses_normalized_detection_centres():
    rect = [0.34, 0.25, 0.66, 0.88]

    assert center_inside_normalized_rect(672.0, 422.0, 1280, 960, rect)
    assert not center_inside_normalized_rect(420.0, 422.0, 1280, 960, rect)
    assert not center_inside_normalized_rect(672.0, 900.0, 1280, 960, rect)
    assert center_inside_normalized_rect(0.34 * 1280, 0.25 * 960, 1280, 960, rect)


def test_inner_fruit_box_promotes_outer_cube_and_removes_duplicate():
    outer = SimpleNamespace(
        label="cube", confidence=0.93,
        x_center=320.0, y_center=856.0, width=74.0, height=68.0,
    )
    inner = SimpleNamespace(
        label="fruit_photo_cube", confidence=0.77,
        x_center=304.0, y_center=867.0, width=30.0, height=31.0,
    )

    result = collapse_nested_fruit_cube_detections([outer, inner])

    assert result == [outer]
    assert outer.label == "fruit_photo_cube"
    assert outer.confidence == pytest.approx(0.93)


def test_standalone_plain_cube_remains_plain():
    cube = SimpleNamespace(
        label="cube", confidence=0.90,
        x_center=100.0, y_center=100.0, width=50.0, height=50.0,
    )
    distant_fruit = SimpleNamespace(
        label="fruit_photo_cube", confidence=0.80,
        x_center=300.0, y_center=300.0, width=20.0, height=20.0,
    )

    result = collapse_nested_fruit_cube_detections([cube, distant_fruit])

    assert result == [cube, distant_fruit]
    assert cube.label == "cube"


def test_top_callback_never_runs_yolo_for_a_stale_queued_image():
    logger = _Logger()
    calls = []
    node = type("NodeStub", (), {})()
    node._competition_state = "RUNNING"
    node._now_sec = lambda: 10.0
    node._last_top = 0.0
    node.top_min_interval = 0.066
    node.max_input_age_sec = 0.25
    node.pub_top = object()
    node.top_model = object()
    node.top_imgsz = 1280
    node.get_logger = lambda: logger
    node._process = lambda *args: calls.append(args)
    node._image_is_fresh = lambda msg, now, stream: (
        YoloDetectorNode._image_is_fresh(node, msg, now, stream)
    )
    stale = Image()
    stale.header.stamp = Time(sec=9, nanosec=0)

    YoloDetectorNode.on_top_image(node, stale)

    assert calls == []
    assert node._last_top == 0.0
    assert "dropping stale top image" in logger.warnings[0]
