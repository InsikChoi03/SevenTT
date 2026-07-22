from robot_planning.nodes.mission_fsm_node import (
    _axis_strafe_zone_entry_waypoints,
    _cardinal_goal_reached_or_passed,
    _nearest_zone_anchor,
    _opening_anchor_strafe_velocity,
    _opening_zone_entry_waypoints,
    _parse_zone_anchor_candidates,
    _zone_anchor_strafe_pulse_duration,
    _zone_anchor_strafe_velocity,
    _zone_anchor_slowdown_state,
    _zone_transition_strafe_profile,
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


def test_zone3_wall_relocalization_anchor_is_a_single_fixed_point():
    candidates = _parse_zone_anchor_candidates(
        [
            1.0, -1.25, 0.75,
            2.0, -0.75, -0.75,
            2.0, -1.25, -0.75,
            3.0, 1.25, -1.25,
            4.0, 1.25, 0.25,
        ],
        [],
    )

    assert candidates[3] == [(1.25, -1.25)]


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


def test_zone_transitions_use_requested_heading_and_cross_axis():
    assert _zone_transition_strafe_profile(1, 2) == ("x", -1.5707963267948966)
    assert _zone_transition_strafe_profile(2, 3) == ("y", 0.0)
    assert _zone_transition_strafe_profile(3, 4) == ("x", 1.5707963267948966)


def test_zone1_to_zone2_corrects_nearest_anchor_x_before_forward_y():
    assert _axis_strafe_zone_entry_waypoints(
        (-1.25, 0.75), (-0.75, -0.75), "x", 0.06
    ) == [(-0.75, 0.75), (-0.75, -0.75)]


def test_zone2_to_zone3_corrects_anchor_y_before_forward_x():
    assert _axis_strafe_zone_entry_waypoints(
        (-1.25, -0.75), (1.25, -1.25), "y", 0.06
    ) == [(-1.25, -1.25), (1.25, -1.25)]


def test_zone2_to_zone3_strafes_right_for_negative_field_y_error():
    assert _zone_anchor_strafe_velocity(-0.50, 0.0, "y", 0.315) == -0.315


def test_zone_anchor_strafe_pulse_duration_uses_three_distance_bands():
    pulse_args = (0.30, 0.15, 1.0, 0.7, 0.4)

    assert _zone_anchor_strafe_pulse_duration(0.50, *pulse_args) == 1.0
    assert _zone_anchor_strafe_pulse_duration(-0.30, *pulse_args) == 1.0
    assert _zone_anchor_strafe_pulse_duration(0.299, *pulse_args) == 0.7
    assert _zone_anchor_strafe_pulse_duration(-0.15, *pulse_args) == 0.7
    assert _zone_anchor_strafe_pulse_duration(0.149, *pulse_args) == 0.4
    assert _zone_anchor_strafe_pulse_duration(-0.061, *pulse_args) == 0.4


def test_zone3_to_zone4_corrects_nearest_anchor_x_before_forward_y():
    assert _axis_strafe_zone_entry_waypoints(
        (1.05, -1.25), (0.75, 0.25), "x", 0.06
    ) == [(0.75, -1.25), (0.75, 0.25)]


def test_zone3_to_zone4_positive_body_left_reduces_negative_field_x_error():
    assert _zone_anchor_strafe_velocity(
        -0.30, 1.5707963267948966, "x", 0.315
    ) == 0.315


def test_zone_anchor_slowdown_latches_inside_threshold():
    active, speed = _zone_anchor_slowdown_state(0.49, True, 0.50, 0.07, False)

    assert active
    assert speed == 0.07


def test_zone_anchor_slowdown_stays_latched_when_pose_distance_grows():
    active, speed = _zone_anchor_slowdown_state(0.64, True, 0.50, 0.07, True)

    assert active
    assert speed == 0.07


def test_zone_anchor_slowdown_can_be_disabled():
    active, speed = _zone_anchor_slowdown_state(0.20, False, 0.50, 0.07, False)

    assert not active
    assert speed is None
