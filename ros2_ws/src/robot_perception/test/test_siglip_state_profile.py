"""Regression tests for state-specific SigLIP scheduling and crop policy."""

from dataclasses import dataclass

from robot_perception.nodes.siglip_gate_node import (
    ClassificationProfile,
    FractionalRateGate,
    body_candidate_labels_for_state,
    classification_profile_for_state,
    collect_crops_for_profile,
    order_detections_for_profile,
)


NORMAL = ClassificationProfile(
    rate_hz=1.5,
    prefer_centered_crop=False,
    crop_center_x_px=320.0,
    max_batch=4,
)
LOCAL = ClassificationProfile(
    rate_hz=3.0,
    prefer_centered_crop=True,
    crop_center_x_px=320.0,
    max_batch=1,
)
FINAL = ClassificationProfile(
    rate_hz=3.0,
    prefer_centered_crop=True,
    crop_center_x_px=320.0,
    max_batch=1,
    crop_center_y_px=345.0,
)


@dataclass
class DetectionStub:
    x_center: float
    name: str


def test_general_mission_states_keep_the_existing_profile():
    for state in ("", "SCAN", "APPROACH", "ALIGN", "PICK"):
        assert classification_profile_for_state(state, NORMAL, LOCAL, FINAL) is NORMAL


def test_final_classify_uses_one_centered_crop():
    selected = classification_profile_for_state(" classify ", NORMAL, LOCAL, FINAL)
    assert selected is FINAL
    assert selected.prefer_centered_crop
    assert selected.max_batch == 1


def test_both_local_anchor_states_select_the_isolated_test_profile():
    for state in ("LOCAL_ANCHOR_INSPECTION", "LOCAL_ANCHOR_ALIGN"):
        selected = classification_profile_for_state(state, NORMAL, LOCAL, FINAL)
        assert selected is LOCAL
        assert selected.rate_hz == 3.0
        assert selected.prefer_centered_crop
        assert selected.crop_center_x_px == 320.0
        assert selected.max_batch == 1


def test_three_hz_timer_preserves_normal_one_point_five_hz_rate():
    gate = FractionalRateGate(timer_rate_hz=3.0)

    assert [gate.ready(NORMAL.rate_hz) for _ in range(8)] == [
        False,
        True,
        False,
        True,
        False,
        True,
        False,
        True,
    ]


def test_local_three_hz_profile_runs_on_every_three_hz_timer_tick():
    gate = FractionalRateGate(timer_rate_hz=3.0)

    assert [gate.ready(LOCAL.rate_hz) for _ in range(5)] == [True] * 5


def test_rate_gate_reset_drops_partial_normal_profile_credit():
    gate = FractionalRateGate(timer_rate_hz=3.0)
    assert not gate.ready(NORMAL.rate_hz)

    gate.reset()

    assert not gate.ready(NORMAL.rate_hz)
    assert gate.ready(NORMAL.rate_hz)


def test_local_profile_orders_by_center_and_keeps_one_valid_crop():
    detections = [
        DetectionStub(80.0, "left"),
        DetectionStub(315.0, "center"),
        DetectionStub(500.0, "right"),
    ]

    ordered = order_detections_for_profile(detections, LOCAL)
    crops = collect_crops_for_profile(
        detections,
        LOCAL,
        lambda detection: detection.name,
    )

    assert [detection.name for detection in ordered] == [
        "center",
        "right",
        "left",
    ]
    assert crops == ["center"]


def test_normal_profile_preserves_detection_order_and_batch_capacity():
    detections = [
        DetectionStub(80.0, "first"),
        DetectionStub(315.0, "second"),
        DetectionStub(500.0, "third"),
    ]

    crops = collect_crops_for_profile(
        detections,
        NORMAL,
        lambda detection: detection.name,
    )

    assert crops == ["first", "second", "third"]


def test_invalid_center_crop_does_not_consume_local_batch_slot():
    detections = [
        DetectionStub(319.0, "invalid-center"),
        DetectionStub(350.0, "valid-near-center"),
        DetectionStub(500.0, "valid-right"),
    ]

    crops = collect_crops_for_profile(
        detections,
        LOCAL,
        lambda detection: (
            None if detection.name == "invalid-center" else detection.name
        ),
    )

    assert crops == ["valid-near-center"]


def test_final_set2_prefers_grab_cube_over_x_centered_far_fruit():
    @dataclass
    class BodyDetectionStub:
        x_center: float
        y_center: float
        name: str

    grab_cube = BodyDetectionStub(254.0, 334.5, "grab-cube")
    far_fruit = BodyDetectionStub(351.5, 27.2, "far-fruit")

    ordered = order_detections_for_profile([far_fruit, grab_cube], FINAL)

    assert ordered[0] is grab_cube


def test_generic_cube_is_siglip_candidate_only_in_final_set2_phase():
    extras = ["cube"]

    assert body_candidate_labels_for_state(
        "CLASSIFY", 2, "fruit_photo_cube", extras
    ) == frozenset({"fruit_photo_cube", "cube"})
    assert body_candidate_labels_for_state(
        "APPROACH", 2, "fruit_photo_cube", extras
    ) == frozenset({"fruit_photo_cube"})
    assert body_candidate_labels_for_state(
        "CLASSIFY", 1, "fruit_photo_cube", extras
    ) == frozenset({"fruit_photo_cube"})
