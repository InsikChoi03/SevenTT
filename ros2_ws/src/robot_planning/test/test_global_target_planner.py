"""Global target mode helpers: min-path tour, FOV coverage, unseen-grid fallback."""
import math

import pytest

from robot_planning.global_target_planner import (
    best_observation_pose,
    grid_nodes,
    next_zigzag_observation_pose,
    next_unseen_node,
    nodes_in_ellipse_fov,
    nodes_in_fov,
    order_targets_min_path,
)


def test_tour_prefers_min_total_path_over_greedy_nearest():
    # Greedy from (0,0) grabs A(1,0) first and pays the full backtrack past start:
    #   greedy A->B->C = 1 + 2 + 4 = 7, optimal C->A->B = 1 + 2 + 2 = 5.
    targets = [("A", 1.0, 0.0), ("B", 3.0, 0.0), ("C", -1.0, 0.0)]
    order = order_targets_min_path((0.0, 0.0), targets)
    assert order == ["C", "A", "B"]


def test_tour_single_and_empty():
    assert order_targets_min_path((0.0, 0.0), []) == []
    assert order_targets_min_path((0.0, 0.0), [("only", 2.0, 2.0)]) == ["only"]


def test_tour_greedy_beyond_brute_force_limit_covers_all():
    targets = [(i, float(i % 4), float(i // 4)) for i in range(10)]
    order = order_targets_min_path((0.0, 0.0), targets, brute_force_limit=7)
    assert sorted(order) == list(range(10))


def test_grid_nodes_matches_field_layout():
    nodes = grid_nodes(6, 7, -1.5, -1.5, 0.5)
    assert len(nodes) == 42
    assert nodes[0] == (-1.5, -1.5)
    assert nodes[6] == pytest.approx((1.5, -1.5))
    assert nodes[-1] == pytest.approx((1.5, 1.0))


def test_fov_wedge_sees_front_only_within_range():
    nodes = [(1.0, 0.0), (-1.0, 0.0), (1.0, 1.5), (5.0, 0.0), (0.05, 0.0)]
    seen = nodes_in_fov(
        (0.0, 0.0, 0.0), nodes, math.radians(140.0), max_range_m=2.5, min_range_m=0.15
    )
    assert 0 in seen          # straight ahead
    assert 1 not in seen      # behind
    assert 2 in seen          # ahead-left, inside 70 deg half-angle
    assert 3 not in seen      # beyond trusted range
    assert 4 not in seen      # under the base (closer than min range)


def test_fov_wedge_rotates_with_heading():
    nodes = [(0.0, 1.0), (0.0, -1.0)]
    seen = nodes_in_fov((0.0, 0.0, math.pi / 2), nodes, math.radians(140.0), 2.5)
    assert seen == {0}


def test_next_unseen_node_nearest_first_then_none():
    nodes = grid_nodes(2, 2, 0.0, 0.0, 1.0)   # (0,0) (1,0) (0,1) (1,1)
    got = next_unseen_node((0.9, 0.1), nodes, set())
    assert got is not None and got[0] == 1     # nearest is (1, 0)
    assert next_unseen_node((0.0, 0.0), nodes, {0, 1, 2, 3}) is None
    got = next_unseen_node((0.0, 0.0), nodes, {0})
    assert got is not None and got[0] in {1, 2}


def test_calibrated_ellipse_footprint_rotates_with_robot_heading():
    nodes = [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0)]
    seen = nodes_in_ellipse_fov(
        (0.0, 0.0, 0.0), nodes, 1.1, 0.6, 0.4, min_range_m=0.1
    )
    assert 0 in seen
    assert 1 not in seen
    assert 2 not in seen

    rotated = nodes_in_ellipse_fov(
        (0.0, 0.0, math.pi / 2), nodes, 1.1, 0.6, 0.4, min_range_m=0.1
    )
    assert 2 in rotated
    assert 0 not in rotated


def test_best_observation_pose_uses_safe_candidate_not_object_slot():
    slots = grid_nodes(3, 3, -0.5, -0.5, 0.5)
    lane_centres = [(-0.25, -0.25), (0.25, 0.25), (0.75, 0.75)]
    selected = best_observation_pose(
        (0.8, 0.8, 0.0),
        slots,
        set(),
        lane_centres,
        1.1,
        1.3,
        0.4,
        0.1,
    )
    assert selected is not None
    assert selected[:2] in lane_centres
    assert selected[:2] not in slots
    assert selected[3]


def test_zigzag_observation_continues_forward_and_skips_seen_waypoints():
    nodes = [(-0.5, 0.0), (0.5, 0.0), (1.5, 0.0)]
    waypoints = [(-0.5, 0.0), (0.5, 0.0), (1.5, 0.0)]
    first = next_zigzag_observation_pose(
        nodes, set(), waypoints, 0, 0.6, 0.3, 0.0, 0.0
    )
    assert first is not None and first[0] == 0

    second = next_zigzag_observation_pose(
        nodes, set(first[4]), waypoints, 0, 0.6, 0.3, 0.0, 0.0
    )
    assert second is not None
    assert second[0] > first[0]


def test_zigzag_observation_returns_none_after_all_nodes_seen():
    assert next_zigzag_observation_pose(
        [(0.0, 0.0)], {0}, [(0.0, 0.0)], 0, 1.0, 1.0, 0.0
    ) is None


def test_fixed_zigzag_keeps_connector_even_when_its_view_is_already_seen():
    route = [(-1.25, -1.25), (-0.25, -1.25), (-0.25, 1.25)]
    selected = next_zigzag_observation_pose(
        [(0.0, 0.0)],
        {0},
        route,
        1,
        1.1,
        1.3,
        0.4,
        0.15,
        skip_fully_seen=False,
    )
    assert selected is not None
    assert selected[0:3] == (1, -0.25, -1.25)
    assert selected[4] == set()


def test_z2_first_route_has_requested_double_turn_sequence():
    # Initial leg is south toward Z2. At each connector/end point the signed 90-degree
    # changes must be left-left, right-right, left-left.
    route = [
        (-1.25, 1.25),
        (-1.25, -1.25),
        (-0.25, -1.25),
        (-0.25, 1.25),
        (0.75, 1.25),
        (0.75, -1.25),
        (1.25, -1.25),
        (1.25, 1.25),
    ]
    headings = [
        math.atan2(by - ay, bx - ax)
        for (ax, ay), (bx, by) in zip(route, route[1:])
    ]
    turns = [
        math.atan2(math.sin(b - a), math.cos(b - a))
        for a, b in zip(headings, headings[1:])
    ]
    assert [math.copysign(1, turn) for turn in turns] == [1, 1, -1, -1, 1, 1]
    assert all(abs(abs(turn) - math.pi / 2) < 1e-9 for turn in turns)
