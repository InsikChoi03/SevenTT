"""Tests for the SigLIP transit path, wide-hint gating, and the hint wire format."""

from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

from robot_perception.fruit_color_gate import ColorGateConfig
from robot_perception.nodes.siglip_gate_node import (
    ClassificationProfile,
    CropScore,
    SiglipGateNode,
    closest_stamped_frame,
    drain_waiting_detections,
    detection_passes_motion_gate,
    is_transit_state,
    order_wide_candidates,
    transit_gate_ready,
)
from robot_perception.wide_fruit_hint import build_wide_hint, parse_wide_hint


TRANSIT_STATES = frozenset({"SCAN", "APPROACH", "DRIVE_TO_STORAGE"})
NORMAL = ClassificationProfile(
    rate_hz=1.5, prefer_centered_crop=False, crop_center_x_px=320.0, max_batch=4
)


@dataclass
class DetectionStub:
    x_center: float = 320.0
    y_center: float = 240.0
    width: float = 60.0
    height: float = 60.0
    confidence: float = 0.9
    label: str = "fruit_photo_cube"


def test_transit_state_matching_normalizes_and_excludes_stationary_states():
    assert is_transit_state(" scan ", TRANSIT_STATES)
    assert is_transit_state("APPROACH", TRANSIT_STATES)
    for state in ("", "ALIGN", "CLASSIFY", "PICK", "LOCAL_ANCHOR_INSPECTION"):
        assert not is_transit_state(state, TRANSIT_STATES)


def test_transit_gate_enforces_min_interval():
    assert not transit_gate_ready(last_run_sec=10.0, now_sec=10.4, min_interval_sec=0.5)
    assert transit_gate_ready(last_run_sec=10.0, now_sec=10.5, min_interval_sec=0.5)
    assert transit_gate_ready(last_run_sec=float("-inf"), now_sec=0.0, min_interval_sec=0.5)


def test_detection_stamp_selects_matching_buffered_image_not_latest_image():
    old_frame = object()
    matching_frame = object()
    latest_frame = object()
    frames = [(1.0, old_frame), (2.0, matching_frame), (2.1, latest_frame)]
    assert closest_stamped_frame(frames, 2.0, 0.025) is matching_frame
    assert closest_stamped_frame(frames, 1.5, 0.025) is None


def test_frame_match_can_use_one_adjacent_30fps_sample_after_qos_drop():
    adjacent_frame = object()
    frames = [(2.033, adjacent_frame)]
    assert closest_stamped_frame(frames, 2.0, 0.050) is adjacent_frame
    assert closest_stamped_frame(frames, 2.0, 0.025) is None


def test_detection_waits_until_its_matching_image_arrives():
    early = object()
    future = object()
    matched, retained = drain_waiting_detections(
        [(2.000, early), (2.100, future)],
        frame_stamp_sec=2.000,
        tolerance_sec=0.050,
        max_age_sec=0.25,
    )
    assert matched == [early]
    assert retained == [(2.100, future)]


def test_detection_wait_queue_prunes_only_definitively_old_entries():
    stale = object()
    recent = object()
    matched, retained = drain_waiting_detections(
        [(1.000, stale), (1.900, recent)],
        frame_stamp_sec=2.000,
        tolerance_sec=0.050,
        max_age_sec=0.25,
    )
    assert matched == []
    assert retained == [(1.900, recent)]


def test_motion_gate_rejects_small_and_low_confidence_boxes():
    assert detection_passes_motion_gate(60.0, 60.0, 0.9, 48, 0.6)
    assert not detection_passes_motion_gate(40.0, 60.0, 0.9, 48, 0.6)     # short side small
    assert not detection_passes_motion_gate(60.0, 60.0, 0.5, 48, 0.6)     # blurry = low conf
    assert not detection_passes_motion_gate(float("nan"), 60.0, 0.9, 48, 0.6)


def test_wide_candidates_ordered_largest_first_and_capped():
    small = DetectionStub(width=30.0, height=30.0)
    big = DetectionStub(width=90.0, height=90.0)
    mid = DetectionStub(width=50.0, height=50.0)
    assert order_wide_candidates([small, big, mid], 2) == [big, mid]


def test_wide_hint_round_trip_preserves_fields():
    payload = build_wide_hint("apple", 0.42, 123.5, 456.5, 100.25)
    hint = parse_wide_hint(payload)
    assert hint is not None
    assert hint.label == "apple"
    assert abs(hint.confidence - 0.42) < 1e-9
    assert hint.u_px == 123.5 and hint.v_px == 456.5
    assert hint.stamp_sec == 100.25


def test_wide_hint_parser_rejects_malformed_payloads():
    assert parse_wide_hint("not json") is None
    assert parse_wide_hint("[1, 2, 3]") is None
    assert parse_wide_hint('{"label": "apple"}') is None                 # missing fields
    assert parse_wide_hint(build_wide_hint("", 0.4, 1.0, 2.0, 3.0)) is None
    assert parse_wide_hint(
        '{"label": "apple", "confidence": "x", "u_px": 1, "v_px": 2, "stamp_sec": 3}'
    ) is None
    assert parse_wide_hint(
        '{"label": "apple", "confidence": NaN, "u_px": 1, "v_px": 2, "stamp_sec": 3}'
    ) is None


def test_wide_hint_parser_clamps_confidence_and_normalizes_label():
    hint = parse_wide_hint(build_wide_hint(" Apple ", 1.7, 1.0, 2.0, 3.0))
    assert hint.label == "apple"
    assert hint.confidence == 1.0


# --------------------------------------------------------------- _tick_classify harness

@dataclass
class _ClockStub:
    sec: float = 100.0

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.sec * 1e9))


@dataclass
class _Harness:
    """Bare-attribute stand-in for SiglipGateNode inside _tick_classify."""

    transit_classify_enabled: bool
    _mission_state: str
    _transit_states: frozenset = TRANSIT_STATES
    transit_classify_min_interval_sec: float = 0.5
    transit_classify_min_crop_px: int = 48
    transit_classify_min_det_conf: float = 0.60
    _last_transit_classify_sec: float = float("-inf")
    _competition_state: str = "RUNNING"
    _pending: tuple = None
    model: object = None
    processor: object = None
    clock: _ClockStub = field(default_factory=_ClockStub)
    published: list = field(default_factory=list)
    rate_ready: bool = True

    def get_clock(self):
        return self.clock

    def _active_profile(self):
        return NORMAL

    def _transit_active(self):
        return SiglipGateNode._transit_active(self)

    def _crop(self, frame, det, w, h):
        return np.zeros((8, 8, 3), dtype=np.uint8)

    def _classify_and_publish(self, crops, stamp, detections=None):
        self.published.append((len(crops), stamp))


def _make_harness(**kwargs):
    harness = _Harness(**kwargs)
    harness.model = object()
    harness.processor = object()
    harness._rate_gate = SimpleNamespace(ready=lambda rate: harness.rate_ready)
    return harness


def _pend(harness, *dets):
    harness._pending = (np.zeros((480, 640, 3), dtype=np.uint8), list(dets), "stamp")


def test_transit_disabled_keeps_profile_rate_path_in_transit_states():
    harness = _make_harness(transit_classify_enabled=False, _mission_state="SCAN")
    _pend(harness, DetectionStub())
    SiglipGateNode._tick_classify(harness)
    assert harness.published == [(1, "stamp")]


def test_transit_path_waits_for_min_interval_and_keeps_pending():
    harness = _make_harness(transit_classify_enabled=True, _mission_state="SCAN")
    harness._last_transit_classify_sec = harness.clock.sec - 0.2   # interval not elapsed
    _pend(harness, DetectionStub())
    SiglipGateNode._tick_classify(harness)
    assert harness.published == []
    assert harness._pending is not None      # retried on the next timer tick


def test_transit_path_classifies_after_interval_and_records_run_time():
    harness = _make_harness(transit_classify_enabled=True, _mission_state="APPROACH")
    harness._last_transit_classify_sec = harness.clock.sec - 1.0
    _pend(harness, DetectionStub())
    SiglipGateNode._tick_classify(harness)
    assert harness.published == [(1, "stamp")]
    assert harness._last_transit_classify_sec == harness.clock.sec


def test_transit_path_filters_small_and_low_conf_crops_for_motion_blur():
    harness = _make_harness(transit_classify_enabled=True, _mission_state="SCAN")
    _pend(
        harness,
        DetectionStub(width=30.0, height=30.0),        # too small while moving
        DetectionStub(confidence=0.4),                 # too uncertain while moving
    )
    SiglipGateNode._tick_classify(harness)
    assert harness.published == []


def test_transit_gates_do_not_apply_in_stationary_states():
    harness = _make_harness(transit_classify_enabled=True, _mission_state="CLASSIFY")
    _pend(harness, DetectionStub(width=30.0, height=30.0, confidence=0.4))
    SiglipGateNode._tick_classify(harness)
    assert harness.published == [(1, "stamp")]       # normal path: profile gates only


def test_transit_rate_gate_blocks_when_profile_gate_blocks_in_normal_path():
    harness = _make_harness(transit_classify_enabled=False, _mission_state="SCAN")
    harness.rate_ready = False
    _pend(harness, DetectionStub())
    SiglipGateNode._tick_classify(harness)
    assert harness.published == []


# ----------------------------------------------------------- _tick_wide_classify harness

class _PublisherSpy:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _SilentLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _run_wide(score, unlabeled=True):
    stamp = SimpleNamespace(sec=100, nanosec=250_000_000)
    frame = np.zeros((960, 1280, 3), dtype=np.uint8)
    det = DetectionStub(x_center=640.0, y_center=480.0, width=64.0, height=64.0)
    harness = SimpleNamespace(
        _competition_state="RUNNING",
        _wide_pending=(frame, [det], stamp),
        model=object(),
        processor=object(),
        _unlabeled_set2_present=unlabeled,
        wide_classify_min_crop_px=28,
        wide_classify_max_batch=2,
        wide_hint_min_margin=0.15,
        aspect_ratio_min=0.6,
        aspect_ratio_max=1.7,
        square_pad_crops=False,
        color_filter_enabled=False,
        color_target_override_enabled=False,
        pub_wide_hint=_PublisherSpy(),
        get_logger=lambda: _SilentLogger(),
        _score_crops=lambda crops: [score] * len(crops),
        _color_veto=lambda crop, label: False,
    )
    harness._apply_color_target_override = (
        lambda crop, label, margin: SiglipGateNode._apply_color_target_override(
            harness, crop, label, margin
        )
    )
    harness._crop_wide = (
        lambda frame, det, w, h: SiglipGateNode._crop_wide(harness, frame, det, w, h)
    )
    SiglipGateNode._tick_wide_classify(harness)
    return harness


def test_wide_path_publishes_hint_on_its_own_topic_only():
    score = CropScore("apple", 0.5, 0.9, image_face_visible=True)
    harness = _run_wide(score)
    # No `pub` attribute exists on the harness: _tick_wide_classify structurally CANNOT
    # publish on /classification/siglip (it would raise AttributeError here if it tried).
    assert len(harness.pub_wide_hint.messages) == 1
    hint = parse_wide_hint(harness.pub_wide_hint.messages[0].data)
    assert hint.label == "apple"
    assert abs(hint.confidence - 0.5) < 1e-9
    assert hint.u_px == 640.0 and hint.v_px == 480.0
    assert abs(hint.stamp_sec - 100.25) < 1e-9


def test_wide_path_keeps_live_hint_updates_after_all_tracks_have_a_label():
    score = CropScore("apple", 0.5, 0.9, image_face_visible=True)
    harness = _run_wide(score, unlabeled=False)
    assert len(harness.pub_wide_hint.messages) == 1
    assert harness._wide_pending is None


def test_pineapple_target_never_overrides_a_banana_read_from_yellow_alone():
    harness = SimpleNamespace(
        color_filter_enabled=True,
        color_target_override_enabled=True,
        set2_label="pineapple",
        color_target_override_min_fraction=0.90,
        color_target_override_min_colored_fraction=0.10,
        color_filter_veto_scale=0.25,
        color_gate_config=ColorGateConfig(),
        get_logger=lambda: _SilentLogger(),
    )
    yellow = np.full((40, 40, 3), (0, 255, 255), dtype=np.uint8)
    label, margin, vetoed = SiglipGateNode._apply_color_target_override(
        harness, yellow, "banana", 0.6
    )
    assert (label, margin, vetoed) == ("banana", 0.6, False)


def test_wide_path_suppresses_faceless_and_low_margin_reads():
    faceless = CropScore("apple", 0.5, 0.9, image_face_visible=False)
    assert _run_wide(faceless).pub_wide_hint.messages == []
    weak = CropScore("apple", 0.10, 0.9, image_face_visible=True)
    assert _run_wide(weak).pub_wide_hint.messages == []
