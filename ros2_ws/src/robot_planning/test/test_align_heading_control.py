import math

from robot_planning.nodes.mission_fsm_node import (
    align_heading_error,
    heading_control_command,
)


def test_heading_control_uses_shortest_wrapped_error():
    aligned, omega = heading_control_command(
        math.radians(179.0),
        math.radians(-171.0),
        math.radians(3.0),
        kp=2.0,
        omega_max=0.10,
    )
    assert not aligned
    assert omega == 0.10


def test_heading_control_stops_inside_actual_heading_tolerance():
    aligned, omega = heading_control_command(
        math.radians(8.0),
        math.radians(10.0),
        math.radians(3.0),
        kp=2.0,
        omega_max=0.10,
    )
    assert aligned
    assert omega == 0.0


def test_align_heading_faces_wide_target_without_blind_scan():
    error = align_heading_error(0.375, 0.10, grab_y=0.0)
    assert math.isclose(error, math.atan2(0.10, 0.375), abs_tol=1e-9)


def test_align_heading_preserves_offset_grab_line_for_forward_motion():
    radius = 0.40
    grab_y = 0.02
    desired_bearing = math.asin(grab_y / radius)
    target_x = radius * math.cos(desired_bearing)
    target_y = radius * math.sin(desired_bearing)
    assert math.isclose(align_heading_error(target_x, target_y, grab_y), 0.0, abs_tol=1e-9)
