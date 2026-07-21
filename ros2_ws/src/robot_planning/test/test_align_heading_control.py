import math

from robot_planning.nodes.mission_fsm_node import (
    heading_control_command,
    yaw_scan_target_heading,
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


def test_yaw_scan_targets_are_plus_minus_then_origin():
    origin = math.radians(175.0)
    scan = math.radians(10.0)
    targets = [yaw_scan_target_heading(origin, step, scan) for step in range(3)]
    expected = [math.radians(-175.0), math.radians(165.0), origin]
    for actual, wanted in zip(targets, expected):
        assert actual is not None
        assert math.isclose(
            math.atan2(math.sin(actual - wanted), math.cos(actual - wanted)),
            0.0,
            abs_tol=1e-9,
        )
    assert yaw_scan_target_heading(origin, 3, scan) is None
