"""Tests for pose-free virtual anchor geometry."""
import math

import pytest

from robot_planning.relative_anchor_grid import RelativeAnchorGridTracker, RelativeObservation


def _obs(points):
    return [RelativeObservation(x, y, "cube", 1, 0.9) for x, y in points]


def _acquire(points, *, expected=(1.0, 0.5)):
    tracker = RelativeAnchorGridTracker(stale_timeout_s=0.5, smoothing_alpha=1.0)
    for frame in range(3):
        tracker.observe(_obs(points), now_s=0.1 * frame)
    assert tracker.lock(
        expected_k1_center=expected,
        expected_grid_yaw=0.0,
        min_cluster_hits=2,
        min_points=2,
        now_s=0.3,
    )
    return tracker


def test_four_points_fit_k1_and_project_k2_k3_virtual_centers():
    points = [(0.75, 0.25), (1.25, 0.25), (0.75, 0.75), (1.25, 0.75)]
    tracker = _acquire(list(reversed(points)))

    assert tracker.anchor_point(1) == pytest.approx((1.0, 0.5), abs=1e-6)
    assert tracker.anchor_point(2) == pytest.approx((2.0, 0.5), abs=1e-6)
    assert tracker.anchor_point(3) == pytest.approx((3.0, 0.5), abs=1e-6)
    assert len(tracker.evidence_for_anchor(1, min_hits=1)) == 4


@pytest.mark.parametrize("missing", range(4))
def test_three_points_restore_the_same_virtual_center(missing):
    points = [(0.75, 0.25), (1.25, 0.25), (0.75, 0.75), (1.25, 0.75)]
    tracker = _acquire([point for index, point in enumerate(points) if index != missing])
    estimate = tracker.estimate(1, now_s=0.3)

    assert (estimate.center_x, estimate.center_y) == pytest.approx((1.0, 0.5), abs=1e-6)
    assert len(estimate.slots) == 4
    assert len(tracker.evidence_for_anchor(1)) == 3


def test_two_adjacent_points_use_coarse_center_prior_and_keep_four_virtual_slots():
    tracker = _acquire([(0.75, 0.25), (1.25, 0.25)])
    estimate = tracker.estimate(1, now_s=0.3)

    assert (estimate.center_x, estimate.center_y) == pytest.approx((1.0, 0.5), abs=1e-6)
    assert len(estimate.slots) == 4
    assert len(tracker.evidence_for_anchor(1)) == 2


def test_empty_k1_can_bootstrap_from_visible_k2_objects_and_keep_k1_virtual():
    tracker = _acquire([(1.75, 0.25), (2.25, 0.25)])

    assert tracker.anchor_point(1) == pytest.approx((1.0, 0.5), abs=1e-6)
    assert tracker.anchor_point(2) == pytest.approx((2.0, 0.5), abs=1e-6)
    assert len(tracker.evidence_for_anchor(1)) == 0
    assert len(tracker.evidence_for_anchor(2)) == 2


def test_one_or_zero_points_cannot_bootstrap_a_grid():
    for points in ([], [(1.0, 0.5)]):
        tracker = RelativeAnchorGridTracker()
        for frame in range(3):
            tracker.observe(_obs(points), now_s=0.1 * frame)
        assert not tracker.lock(
            expected_k1_center=(1.0, 0.5), min_cluster_hits=2, min_points=2, now_s=0.3
        )


@pytest.mark.parametrize("points", [[], [(0.75, 0.25)]])
def test_trusted_opening_prior_locks_four_slots_when_objects_are_missing(points):
    tracker = RelativeAnchorGridTracker()
    for frame in range(3):
        tracker.observe(_obs(points), now_s=0.1 * frame)

    assert tracker.lock(
        expected_k1_center=(1.0, 0.5),
        expected_grid_yaw=0.0,
        min_cluster_hits=2,
        min_points=2,
        allow_prior_fallback=True,
        now_s=0.3,
    )
    estimate = tracker.estimate(1, now_s=0.3)
    assert (estimate.center_x, estimate.center_y) == pytest.approx((1.0, 0.5))
    assert len(estimate.slots) == 4
    assert len(tracker.evidence_for_anchor(1)) == len(points)


def test_stationary_release_refreshes_empty_prior_after_long_t18_hold():
    tracker = RelativeAnchorGridTracker(dead_reckon_timeout_s=12.0)
    assert tracker.lock(
        expected_k1_center=(0.55, 0.05),
        expected_grid_yaw=math.pi / 2.0,
        allow_prior_fallback=True,
        now_s=4.0,
    )
    assert not tracker.estimate(1, now_s=18.0).navigable
    assert tracker.refresh_trusted_stationary_reference(now_s=18.0)
    assert tracker.estimate(1, now_s=18.0).navigable


def test_locked_grid_updates_from_remaining_landmarks_and_becomes_stale_safely():
    points = [(0.75, 0.25), (1.25, 0.25), (0.75, 0.75), (1.25, 0.75)]
    tracker = _acquire(points)
    # Robot moved 20 cm forward: every stationary landmark is now 20 cm closer.
    moved = [(x - 0.20, y) for x, y in points[:2]]
    tracker.observe(_obs(moved), now_s=0.4)

    assert tracker.anchor_point(1) == pytest.approx((0.8, 0.5), abs=0.04)
    assert tracker.estimate(1, now_s=0.8).navigable
    stale = tracker.estimate(1, now_s=1.1)
    assert not stale.navigable
    assert stale.reason == "stale_relative_landmarks"


def test_rotation_update_preserves_anchor_center_from_two_landmarks():
    points = [(0.75, 0.25), (1.25, 0.25), (0.75, 0.75), (1.25, 0.75)]
    tracker = _acquire(points)
    yaw = math.radians(-10.0)
    rotated = [
        (math.cos(yaw) * x - math.sin(yaw) * y,
         math.sin(yaw) * x + math.cos(yaw) * y)
        for x, y in points[:3]
    ]
    tracker.observe(_obs(rotated), now_s=0.4)

    expected = (
        math.cos(yaw) * 1.0 - math.sin(yaw) * 0.5,
        math.sin(yaw) * 1.0 + math.cos(yaw) * 0.5,
    )
    assert tracker.anchor_point(1) == pytest.approx(expected, abs=0.04)


def test_command_prediction_keeps_empty_virtual_anchor_navigable_until_timeout():
    points = [(0.75, 0.25), (1.25, 0.25)]
    tracker = _acquire(points)
    assert tracker.predict_robot_motion(
        dx_body=0.20, dy_body=0.0, dtheta=0.0, now_s=0.4
    )

    assert tracker.anchor_point(1) == pytest.approx((0.8, 0.5), abs=1e-6)
    estimate = tracker.estimate(2, now_s=0.9)
    assert estimate.navigable
    assert estimate.reason == "dead_reckoning"
    assert not tracker.estimate(2, now_s=13.0).navigable


def test_imu_yaw_prediction_prevents_one_point_rotation_becoming_translation():
    points = [(0.75, 0.25), (1.25, 0.25), (0.75, 0.75), (1.25, 0.75)]
    tracker = _acquire(points)
    yaw = math.radians(10.0)
    assert tracker.predict_robot_motion(
        dx_body=0.0, dy_body=0.0, dtheta=yaw, now_s=0.35
    )
    visible = (
        math.cos(-yaw) * points[0][0] - math.sin(-yaw) * points[0][1],
        math.sin(-yaw) * points[0][0] + math.cos(-yaw) * points[0][1],
    )
    tracker.observe(_obs([visible]), now_s=0.4)

    expected = (
        math.cos(-yaw) * 1.0 - math.sin(-yaw) * 0.5,
        math.sin(-yaw) * 1.0 + math.cos(-yaw) * 0.5,
    )
    assert tracker.anchor_point(1) == pytest.approx(expected, abs=0.02)


def test_active_slot_is_not_allowed_to_drag_the_virtual_lattice():
    points = [(0.75, 0.25), (1.25, 0.25), (0.75, 0.75), (1.25, 0.75)]
    tracker = _acquire(points)
    tracker.set_excluded_slots({(1, 1)})
    tracker.observe(_obs([(0.95, 0.25)]), now_s=0.4)

    assert tracker.anchor_point(1) == pytest.approx((1.0, 0.5), abs=1e-6)


def test_outlier_and_duplicate_boxes_do_not_move_the_fitted_center():
    points = [(0.75, 0.25), (1.25, 0.25), (0.75, 0.75), (1.25, 0.75)]
    noisy = points + [(0.76, 0.26), (4.0, -3.0)]
    tracker = _acquire(noisy)
    assert tracker.anchor_point(1) == pytest.approx((1.0, 0.5), abs=0.03)
