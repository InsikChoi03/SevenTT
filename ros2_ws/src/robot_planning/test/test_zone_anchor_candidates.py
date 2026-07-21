from robot_planning.nodes.mission_fsm_node import (
    _cardinal_goal_reached_or_passed,
    _nearest_zone_anchor,
    _opening_anchor_strafe_velocity,
    _opening_zone_entry_waypoints,
    _parse_zone_anchor_candidates,
)


def test_parse_zone_anchor_candidates_preserves_multiple_points_per_zone():
    candidates = _parse_zone_anchor_candidates(
        [1.0, 1.25, -0.75, 2.0, 0.75, 0.75, 2.0, 1.25, 0.75],
        [],
    )

    assert candidates == {
        1: [(1.25, -0.75)],
        2: [(0.75, 0.75), (1.25, 0.75)],
    }


def test_nearest_zone_anchor_uses_pose_at_zone_entry():
    index, point = _nearest_zone_anchor(
        [(0.75, 0.75), (1.25, 0.75)],
        (1.10, 0.10),
    )

    assert index == 1
    assert point == (1.25, 0.75)


def test_opening_zone_entry_corrects_x_before_y():
    assert _opening_zone_entry_waypoints(
        (-1.40, 1.58), (-1.25, 0.75), 0.06
    ) == [(-1.25, 1.58), (-1.25, 0.75)]


def test_opening_zone_entry_skips_x_leg_when_already_aligned():
    assert _opening_zone_entry_waypoints(
        (-1.30, 1.58), (-1.25, 0.75), 0.06
    ) == [(-1.25, 0.75)]


def test_opening_zone_entry_stops_after_crossing_goal_line():
    reached, passed = _cardinal_goal_reached_or_passed(
        (-1.25, 1.58), (-1.15, 0.70), (-1.25, 0.75), 0.05
    )

    assert reached
    assert passed


def test_opening_anchor_strafe_moves_field_positive_x_while_facing_negative_y():
    assert _opening_anchor_strafe_velocity(0.15, -1.57079632679, 0.315) == 0.315


def test_opening_anchor_strafe_reverses_when_x_goal_is_crossed():
    assert _opening_anchor_strafe_velocity(-0.04, -1.57079632679, 0.315) == -0.315
