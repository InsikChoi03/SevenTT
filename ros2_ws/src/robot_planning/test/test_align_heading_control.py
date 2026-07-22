import math

from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    align_translation_axis,
    align_heading_error,
    heading_control_command,
    nearest_align_wide_observation,
    nearest_align_wide_point,
)
from robot_planning.relative_anchor_grid import RelativeObservation


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


def test_align_associates_expected_target_but_returns_raw_wide_geometry():
    point = nearest_align_wide_point(
        expected=(0.22, 0.06),
        observations=[(0.21, -0.01), (0.50, 0.30)],
        match_radius_m=0.30,
    )
    assert point == (0.21, -0.01)


def test_align_rejects_raw_wide_observation_outside_match_radius():
    assert nearest_align_wide_point(
        expected=(0.20, 0.00),
        observations=[(0.55, 0.00)],
        match_radius_m=0.30,
    ) is None


def test_align_labelled_wide_match_preserves_identity_and_confidence():
    matched = nearest_align_wide_observation(
        expected=(0.22, 0.00),
        observations=[
            RelativeObservation(0.21, -0.01, "octahedron", 1, 0.97),
            RelativeObservation(0.50, 0.20, "cube", 1, 0.99),
        ],
        match_radius_m=0.30,
    )

    assert matched is not None
    assert matched.label == "octahedron"
    assert matched.confidence == 0.97


def test_align_translation_selects_lateral_when_body_y_is_worst():
    assert align_translation_axis(0.01, 0.08, 0.02, 0.02) == "lateral"


def test_align_translation_selects_forward_when_body_x_is_worst():
    assert align_translation_axis(0.09, 0.03, 0.02, 0.02) == "forward"


def test_align_translation_finishes_only_when_both_body_axes_are_inside_tolerance():
    assert align_translation_axis(0.019, -0.019, 0.02, 0.02) is None


def test_align_body_y_error_commands_lateral_translation_without_rotation():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._time_in_state = lambda: 0.0
    node._now_s = lambda: 1.0
    node.align_timeout_sec = 12.0
    node._align_phase = "measure"
    node._body_label = lambda: "dodecahedron"
    node._body_target_base = lambda _label: ("dodecahedron", 0.19, 0.08, 0.9)
    node.anchor_mission_enabled = False
    node._current_anchor_slot = lambda: None
    node._body_nearest_any = lambda: None
    node._update_align_body_presence = lambda _point: (1, True)
    node.grab_x = 0.19
    node.grab_y = 0.0
    node.align_fwd_tol = 0.02
    node.align_tol = 0.02
    node.align_step_strafe_duty = 0.315
    node.align_step_strafe_sec = 0.30
    node.align_step_strafe_mid_sec = 0.60
    node.align_step_fwd_duty = 0.27
    node.align_step_fwd_sec = 0.15
    node.align_step_fwd_mid_sec = 0.25
    node.align_adaptive_steps_enabled = False
    node.align_mid_error_m = 0.06
    node.grab_min_x = 0.17
    node._reset_align_body_presence = lambda: None
    commands = []
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    MissionFsmNode._step_align(node)

    assert commands[-1] == (0.0, 0.315, 0.0)
    assert node._align_phase == "pulse"


def test_align_body_presence_counts_only_fresh_spatially_continuous_frames():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._body_dets_seq = 0
    node.align_tol = 0.08
    node._reset_align_body_presence()

    node._body_dets_seq = 1
    assert node._update_align_body_presence(("icosahedron", 0.20, 0.02, 0.9)) == (1, True)
    assert node._update_align_body_presence(("icosahedron", 0.20, 0.02, 0.9)) == (1, False)

    node._body_dets_seq = 2
    assert node._update_align_body_presence(("cube", 0.21, 0.02, 0.8)) == (2, True)
    node._body_dets_seq = 3
    assert node._update_align_body_presence(("icosahedron", 0.20, 0.01, 0.9)) == (3, True)


def test_align_body_presence_resets_when_detection_jumps_to_another_object():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._body_dets_seq = 0
    node.align_tol = 0.08
    node._reset_align_body_presence()

    node._body_dets_seq = 1
    assert node._update_align_body_presence(("icosahedron", 0.20, 0.00, 0.9))[0] == 1
    node._body_dets_seq = 2
    assert node._update_align_body_presence(("icosahedron", 0.35, 0.00, 0.9))[0] == 1
