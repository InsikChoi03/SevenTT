"""Set2 final pick safety: routing hints can approach but never move the arm."""

import math
from types import SimpleNamespace

import pytest

from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    Set2IdentityLatch,
    body_source_matches_target_at_capture,
    closest_pose_sample,
    field_point_to_base_at_pose,
    fresh_set2_siglip_verdict,
    select_body_candidate,
    set2_classification_verdict,
    set2_classification_timing_is_valid,
    set2_latch_target_matches,
    set2_pick_allowed,
)


def _classification(label="banana", confidence=0.9, face=True, is_target=True):
    return SimpleNamespace(
        label=label,
        confidence=confidence,
        image_face_visible=face,
        is_target=is_target,
    )


def _track(
    confidence=0.95,
    fruit_label="banana",
    *,
    track_id=6,
    x=-0.5,
    y=0.5,
    blacklisted=False,
):
    return SimpleNamespace(
        id=track_id,
        set_type=2,
        confidence=confidence,
        fruit_label=fruit_label,
        x=x,
        y=y,
        blacklisted=blacklisted,
    )


def _latch():
    return Set2IdentityLatch(
        target_id=6,
        target_x=-0.5,
        target_y=0.5,
        verdict="target",
        label="banana",
        confidence=0.997,
        capture_s=100.0,
        received_s=100.9,
        association_error_m=0.022,
        visit_serial=3,
    )


def test_high_confidence_track_plus_wide_label_cannot_pick_without_body_read():
    verdict, _, _ = fresh_set2_siglip_verdict(
        None,
        None,
        state_enter_s=10.0,
        now_s=10.2,
        max_age_sec=0.6,
        confidence_threshold=0.7,
        target_label="banana",
        spatially_aligned=False,
    )
    assert verdict == "pending"
    assert not set2_pick_allowed(
        _track(),
        pick_track_confidence=0.7,
        require_fruit_label=True,
        body_verdict=verdict,
    )


def test_fresh_aligned_body_target_allows_pick():
    verdict, label, confidence = fresh_set2_siglip_verdict(
        _classification(),
        10.2,
        state_enter_s=10.0,
        now_s=10.3,
        max_age_sec=0.6,
        confidence_threshold=0.7,
        target_label="banana",
        spatially_aligned=True,
    )
    assert (verdict, label, confidence) == ("target", "banana", 0.9)
    assert set2_pick_allowed(
        _track(),
        pick_track_confidence=0.7,
        require_fruit_label=True,
        body_verdict=verdict,
    )


def test_committed_banana_position_accepts_fresh_blank_cube_at_grab():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.set2_committed_cube_pick_enabled = True
    node._committed_set2_target_can_try_align = lambda: True
    node._body_dets_stamp_s = 10.2
    node.state_enter_s = 10.0
    node.classify_body_max_age_sec = 0.6
    node._now_s = lambda: 10.3
    node.grab_x = 0.21
    node.grab_y = 0.0
    node.align_fwd_tol = 0.03
    node.align_tol = 0.03
    node.pick_track_conf = 0.5
    node._body_target_base = lambda _label: ("cube", 0.215, -0.004, 0.91)

    assert node._fresh_committed_set2_cube_at_grab() == (
        "cube", 0.91, pytest.approx(0.005), pytest.approx(-0.004)
    )


def test_pending_siglip_picks_fresh_cube_at_committed_banana_position():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.phase = 2
    node._opportunistic_set2_active = False
    node.set2_require_fruit_label = True
    node.set2_label = "banana"
    node.current_target = _track(track_id=90)
    node._set2_slot_mode = lambda: False
    node._fresh_set2_body_verdict = lambda: ("pending", "", 0.0)
    node._latched_set2_identity = lambda: None
    node._fresh_committed_set2_cube_at_grab = lambda: (
        "cube", 0.91, 0.005, -0.004
    )
    committed = []
    node._commit_pick = lambda set_type, label, detail: committed.append(
        (set_type, label, detail)
    )

    node._step_classify()

    assert committed == [
        (
            2,
            "banana",
            "committed banana position #90; fresh Body cube "
            "conf=0.91 grab_err=(+0.5,-0.4)cm",
        )
    ]


def test_fresh_gripper_bound_set2_target_survives_transient_yolo_loss():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.phase = 1
    node._opportunistic_set2_active = False
    node.current_target = _track()
    node._fresh_set2_body_verdict = lambda: ("target", "banana", 0.99)

    assert node._fresh_aligned_set2_body_target() == ("banana", 0.99)


def test_gripper_body_target_cannot_override_a_set1_visit():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.phase = 1
    node._opportunistic_set2_active = False
    node.current_target = SimpleNamespace(set_type=1)
    node._fresh_set2_body_verdict = lambda: ("target", "banana", 0.99)

    assert node._fresh_aligned_set2_body_target() is None


def test_stale_preclassify_or_spatially_wrong_body_read_stays_pending():
    common = dict(
        classification=_classification(),
        state_enter_s=10.0,
        now_s=10.3,
        max_age_sec=0.6,
        confidence_threshold=0.7,
        target_label="banana",
    )
    assert fresh_set2_siglip_verdict(
        capture_s=9.9, spatially_aligned=True, **common
    )[0] == "pending"
    assert fresh_set2_siglip_verdict(
        capture_s=10.2, spatially_aligned=False, **common
    )[0] == "pending"


def test_fresh_aligned_other_fruit_is_negative_body_evidence():
    verdict, label, _ = fresh_set2_siglip_verdict(
        _classification(label="apple", is_target=False),
        10.2,
        state_enter_s=10.0,
        now_s=10.3,
        max_age_sec=0.6,
        confidence_threshold=0.7,
        target_label="banana",
        spatially_aligned=True,
    )
    assert (verdict, label) == ("non_target", "apple")


def test_gpu_inference_delay_does_not_make_current_capture_stale():
    verdict, label, _ = fresh_set2_siglip_verdict(
        _classification(),
        10.1,
        received_s=11.0,
        state_enter_s=10.0,
        now_s=11.1,
        max_age_sec=0.6,
        confidence_threshold=0.7,
        target_label="banana",
        spatially_aligned=True,
    )

    assert (verdict, label) == ("target", "banana")


@pytest.mark.parametrize(
    ("phase", "current_target", "track_detail"),
    [
        (2, SimpleNamespace(id=83, set_type=0), "track=#83 set0"),
        (2, None, "track=lost"),
        (1, SimpleNamespace(id=83, set_type=0), "track=#83 set0"),
    ],
)
def test_fresh_gripper_body_target_picks_without_set2_track(
    phase, current_target, track_detail
):
    """A close-range set0 regression must not discard a spatially verified target fruit."""
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.phase = phase
    node._opportunistic_set2_active = False
    node.set2_require_fruit_label = True
    node.current_target = current_target
    node._set2_slot_mode = lambda: False
    node._reject_latched_set2_non_target = lambda: False
    node._latched_set2_identity = lambda: None
    node._fresh_set2_body_verdict = lambda: ("target", "banana", 1.0)
    committed = []
    node._commit_pick = lambda set_type, label, detail: committed.append(
        (set_type, label, detail)
    )

    node._step_classify()

    assert committed == [
        (2, "banana", f"fresh gripper Body SigLIP=1.00 {track_detail}")
    ]


def test_track_loss_cannot_pick_without_fresh_spatial_body_target():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.phase = 2
    node._opportunistic_set2_active = False
    node.set2_require_fruit_label = True
    node.current_target = None
    node._set2_slot_mode = lambda: False
    node._reject_latched_set2_non_target = lambda: False
    node._latched_set2_identity = lambda: None
    node._fresh_set2_body_verdict = lambda: ("pending", "banana", 1.0)
    node._front_target = lambda _set_type: None
    node._drive = lambda *_args: None
    node._time_in_state = lambda: 0.1
    node.classify_timeout_sec = 3.5
    committed = []
    node._commit_pick = lambda *args: committed.append(args)

    node._step_classify()

    assert committed == []


def test_global_body_association_ignores_foreground_cube_at_grab():
    foreground = ("cube", 0.19, 0.01, 0.95)
    selected_fruit = ("fruit_photo_cube", 0.316, 0.129, 0.97)

    chosen = select_body_candidate(
        [foreground, selected_fruit],
        grab_xy=(0.19, 0.0),
        expected_xy=(0.31, 0.13),
        expected_match_radius_m=0.18,
    )

    assert chosen == selected_fruit


def test_set2_classification_verdict_accepts_target_and_rejects_other_fruit():
    target = set2_classification_verdict(
        _classification(label="banana", confidence=0.99, is_target=True),
        confidence_threshold=0.7,
        target_label="banana",
    )
    non_target = set2_classification_verdict(
        _classification(label="apple", confidence=0.95, is_target=False),
        confidence_threshold=0.7,
        target_label="banana",
    )

    assert target == ("target", "banana", 0.99)
    assert non_target == ("non_target", "apple", 0.95)


def test_set2_classification_verdict_keeps_vetoed_or_weak_reads_pending():
    vetoed_target = set2_classification_verdict(
        _classification(label="banana", confidence=0.99, is_target=False),
        confidence_threshold=0.7,
        target_label="banana",
    )
    weak_other = set2_classification_verdict(
        _classification(label="apple", confidence=0.69, is_target=False),
        confidence_threshold=0.7,
        target_label="banana",
    )

    assert vetoed_target == ("pending", "banana", 0.99)
    assert weak_other == ("pending", "apple", 0.69)


def test_closest_pose_sample_selects_capture_time_pose_within_skew():
    capture_s = 100.0
    capture_pose = (-0.310, 0.790, math.radians(-131.2))
    samples = [
        (99.80, -0.20, 0.70, math.radians(-120.0)),
        (100.02, *capture_pose),
        (100.25, -0.45, 0.85, math.radians(-145.0)),
    ]

    assert closest_pose_sample(samples, capture_s, max_skew_sec=0.05) == capture_pose


def test_closest_pose_sample_rejects_timestamp_skew():
    samples = [(100.02, -0.310, 0.790, math.radians(-131.2))]

    assert closest_pose_sample(samples, 100.0, max_skew_sec=0.01) is None


def test_latest_run_field_target_matches_body_source_at_capture_pose():
    capture_pose = (-0.310, 0.790, math.radians(-131.2))
    target_field_xy = (-0.5, 0.5)
    source_base_xy = (0.340, 0.070)

    expected_base = field_point_to_base_at_pose(target_field_xy, capture_pose)

    assert expected_base == pytest.approx((0.34335, 0.04806), abs=1e-4)
    assert body_source_matches_target_at_capture(
        source_base_xy,
        target_field_xy,
        capture_pose,
        match_radius_m=0.18,
    )


def test_far_body_source_does_not_match_selected_target_at_capture():
    capture_pose = (-0.310, 0.790, math.radians(-131.2))

    assert not body_source_matches_target_at_capture(
        (0.80, 0.40),
        (-0.5, 0.5),
        capture_pose,
        match_radius_m=0.18,
    )


def test_classification_timing_rejects_previsit_and_delayed_frames():
    assert set2_classification_timing_is_valid(100.1, 101.0, 100.0, 3.0)
    assert not set2_classification_timing_is_valid(99.9, 100.8, 100.0, 3.0)
    assert not set2_classification_timing_is_valid(100.1, 103.2, 100.0, 3.0)


def test_latch_matches_near_id_churn_but_not_same_id_rebound_or_blacklist():
    latch = _latch()

    assert set2_latch_target_matches(
        _track(track_id=17, x=-0.48, y=0.51), latch, 0.18
    )
    assert not set2_latch_target_matches(
        _track(track_id=6, x=0.0, y=0.5), latch, 0.18
    )
    assert not set2_latch_target_matches(
        _track(blacklisted=True), latch, 0.18
    )


def test_blacklisted_set2_track_cannot_authorize_pick():
    assert not set2_pick_allowed(
        _track(blacklisted=True),
        pick_track_confidence=0.7,
        require_fruit_label=True,
        body_verdict="target",
    )
