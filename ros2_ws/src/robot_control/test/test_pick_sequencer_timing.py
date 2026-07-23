"""Tests for per-channel 2R pick timing."""

import pytest

from robot_control.nodes.pick_sequencer_node import required_joint_move_sec, step_command_pose


SPEEDS = (80.0, 120.0, 120.0)


def test_reach_and_lift_fit_inside_verified_one_point_three_seconds():
    """The slowest joint reaches the pick pose within the verified 1.3 seconds."""
    init = (120.0, 5.0, 118.0)
    pick = (25.0, 150.0, 118.0)

    assert required_joint_move_sec(init, pick, SPEEDS) == pytest.approx(145.0 / 120.0)
    assert required_joint_move_sec(pick, init, SPEEDS) < 1.3


def test_gripper_limit_matches_the_automatic_close_window_budget():
    """The 118-to-40 close finishes before the 0.75-second GRASP window ends."""
    opened = (25.0, 150.0, 118.0)
    closed = (25.0, 150.0, 40.0)

    assert required_joint_move_sec(opened, closed, SPEEDS) == pytest.approx(78.0 / 120.0)
    assert required_joint_move_sec(opened, closed, SPEEDS) < 0.75


def test_invalid_channel_speed_is_rejected():
    """Zero or negative channel limits cannot silently produce unsafe timing."""
    with pytest.raises(ValueError):
        required_joint_move_sec((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (80.0, 0.0, 120.0))


def test_grasp_issues_closed_target_immediately_but_reach_still_ramps():
    """GRASP uses MCU interpolation while normal arm moves retain host interpolation."""
    opened = (25.0, 150.0, 118.0)
    closed = (25.0, 150.0, 40.0)

    assert step_command_pose("GRASP", opened, closed, 0.0) == closed
    assert step_command_pose("REACH", opened, closed, 0.5) == (25.0, 150.0, 79.0)
