import math
from types import SimpleNamespace

from robot_planning.lane_planner import cardinal_segment_heading
from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    PulsedHeadingController,
    _cardinal_waypoint_status,
    circular_heading_filter,
    lane_cross_track_feedback,
    lane_heading_violation_time_step,
    lane_route_replan_required,
)


def test_cardinal_segment_heading_for_four_grid_directions():
    assert cardinal_segment_heading((0.25, 0.25), (0.75, 0.25), 0.06) == 0.0
    assert cardinal_segment_heading((0.25, 0.25), (-0.25, 0.25), 0.06) == math.pi
    assert cardinal_segment_heading((0.25, 0.25), (0.25, 0.75), 0.06) == math.pi / 2.0
    assert cardinal_segment_heading((0.25, 0.25), (0.25, -0.25), 0.06) == -math.pi / 2.0


def test_cardinal_segment_heading_tolerates_small_pose_offset():
    heading = cardinal_segment_heading((0.24, 0.27), (0.75, 0.25), 0.06)
    assert heading == 0.0


def test_cardinal_segment_heading_rejects_diagonal_connector():
    assert cardinal_segment_heading((0.10, 0.10), (0.25, 0.25), 0.06) is None


def test_cardinal_waypoint_accepts_goal_radius_or_nearby_goal_line_pass():
    assert _cardinal_waypoint_status(
        (0.0, 0.0), (0.91, 0.07), (1.0, 0.0), 0.12, 0.15
    ) == (True, False, 0.0)

    reached, passed, lateral_error = _cardinal_waypoint_status(
        (0.0, 0.0), (1.07, 0.14), (1.0, 0.0), 0.12, 0.15
    )
    assert reached
    assert passed
    assert math.isclose(lateral_error, 0.14)


def test_cardinal_waypoint_rejects_goal_line_pass_far_outside_lane():
    reached, passed, lateral_error = _cardinal_waypoint_status(
        (0.0, 0.0), (1.07, 0.20), (1.0, 0.0), 0.12, 0.15
    )
    assert not reached
    assert passed
    assert math.isclose(lateral_error, 0.20)


def test_cardinal_waypoint_pass_works_for_vertical_lane():
    reached, passed, lateral_error = _cardinal_waypoint_status(
        (0.5, 0.0), (0.37, 1.08), (0.5, 1.0), 0.12, 0.15
    )
    assert reached
    assert passed
    assert math.isclose(lateral_error, 0.13)


def test_failed_lane_route_retries_after_throttle_even_without_periodic_replan():
    assert not lane_route_replan_required(
        [], False, False, False, now_s=10.49, last_replan_s=10.0, throttle_sec=0.5
    )
    assert lane_route_replan_required(
        [], False, False, False, now_s=10.50, last_replan_s=10.0, throttle_sec=0.5
    )


def test_valid_lane_route_stays_latched_without_a_replan_reason():
    route = [(0.25, 0.25), (0.25, -0.25)]
    assert not lane_route_replan_required(
        route, False, False, False, now_s=20.0, last_replan_s=10.0, throttle_sec=0.5
    )
    assert lane_route_replan_required(
        route, False, True, False, now_s=20.0, last_replan_s=10.0, throttle_sec=0.5
    )


def test_unattempted_or_route_mode_changed_plan_replans_immediately():
    assert lane_route_replan_required(
        None, False, False, False, now_s=1.0, last_replan_s=1.0, throttle_sec=0.5
    )
    assert lane_route_replan_required(
        [(0.0, 0.0)], True, False, False,
        now_s=1.0, last_replan_s=1.0, throttle_sec=0.5,
    )


def test_circular_heading_filter_handles_wraparound():
    filtered = circular_heading_filter(math.radians(179.0), math.radians(-179.0), 0.5)
    error = math.atan2(math.sin(filtered - math.pi), math.cos(filtered - math.pi))
    assert math.isclose(error, 0.0, abs_tol=1e-9)


def test_cross_track_feedback_has_five_centimetre_dead_band_and_correct_sign():
    inside = lane_cross_track_feedback(
        (0.0, 0.0), (1.0, 0.0), (0.4, 0.05), 0.05, 0.55, 1.0, math.radians(10.0)
    )
    assert math.isclose(inside[0], 0.05)
    assert inside[1:] == (0.0, 0.0)

    left = lane_cross_track_feedback(
        (0.0, 0.0), (1.0, 0.0), (0.4, 0.10), 0.05, 0.55, 1.0, math.radians(10.0)
    )
    right = lane_cross_track_feedback(
        (0.0, 0.0), (1.0, 0.0), (0.4, -0.10), 0.05, 0.55, 1.0, math.radians(10.0)
    )
    assert left[0] > 0.0 and left[1] > 0.0 and left[2] < 0.0
    assert right[0] < 0.0 and right[1] < 0.0 and right[2] > 0.0


def test_cross_track_feedback_works_in_vertical_lane_and_caps_bias():
    cte, effective, bias = lane_cross_track_feedback(
        (0.0, 1.0), (0.0, 0.0), (-0.30, 0.5), 0.05, 0.20, 2.0, math.radians(10.0)
    )
    assert cte < 0.0
    assert effective < 0.0
    assert math.isclose(bias, math.radians(10.0))


def test_single_ten_degree_spike_does_not_request_realign():
    filtered = circular_heading_filter(0.0, math.radians(10.0), 0.20)
    violation_start_s, realign = lane_heading_violation_time_step(
        filtered, math.radians(3.0), None, now_s=1.0, required_sec=0.45
    )
    assert math.degrees(filtered) == 2.0
    assert violation_start_s is None
    assert not realign


def test_continuous_heading_violation_requests_realign_after_hold_time():
    start_s, realign = lane_heading_violation_time_step(
        math.radians(4.0), math.radians(3.0), None, now_s=1.0, required_sec=0.45
    )
    assert start_s == 1.0
    assert not realign
    start_s, realign = lane_heading_violation_time_step(
        math.radians(4.0), math.radians(3.0), start_s, now_s=1.44, required_sec=0.45
    )
    assert not realign
    start_s, realign = lane_heading_violation_time_step(
        math.radians(4.0), math.radians(3.0), start_s, now_s=1.451, required_sec=0.45
    )
    assert realign


def test_heading_violation_timer_resets_after_good_frame():
    start_s, _ = lane_heading_violation_time_step(
        math.radians(4.0), math.radians(3.0), None, now_s=2.0, required_sec=0.45
    )
    start_s, realign = lane_heading_violation_time_step(
        math.radians(2.0), math.radians(3.0), start_s, now_s=2.3, required_sec=0.45
    )
    assert start_s is None
    assert not realign


def test_pulsed_heading_controller_stops_between_torque_pulses():
    controller = PulsedHeadingController()

    aligned, omega, event = controller.step(
        now_s=0.0,
        current_heading=math.radians(-20.0),
        target_heading=0.0,
        tolerance_rad=math.radians(2.0),
        key=("lane", 1),
    )
    assert not aligned
    assert omega == 0.0
    assert event == "coarse_pulse"

    _, omega, _ = controller.step(
        now_s=0.05,
        current_heading=math.radians(-18.0),
        target_heading=0.0,
        tolerance_rad=math.radians(2.0),
        key=("lane", 1),
    )
    assert omega == 0.10

    _, omega, event = controller.step(
        now_s=0.25,
        current_heading=math.radians(3.0),
        target_heading=0.0,
        tolerance_rad=math.radians(2.0),
        key=("lane", 1),
    )
    assert omega == 0.0
    assert event == "settle"

    _, omega, _ = controller.step(
        now_s=0.42,
        current_heading=math.radians(3.0),
        target_heading=0.0,
        tolerance_rad=math.radians(2.0),
        key=("lane", 1),
    )
    assert omega == 0.0


def test_large_heading_error_turns_continuously_until_slowdown_band():
    controller = PulsedHeadingController(
        continuous_min_error_rad=math.radians(30.0),
        slowdown_rad=math.radians(10.0),
    )

    aligned, omega, event = controller.step(
        now_s=0.0,
        current_heading=math.radians(-90.0),
        target_heading=0.0,
        tolerance_rad=math.radians(2.0),
        key=("large", 1),
    )
    assert not aligned
    assert omega == 0.10
    assert event == "continuous_turn"

    for now, heading in ((0.25, -70.0), (0.50, -45.0), (0.75, -15.0)):
        _, omega, event = controller.step(
            now_s=now,
            current_heading=math.radians(heading),
            target_heading=0.0,
            tolerance_rad=math.radians(2.0),
            key=("large", 1),
        )
        assert omega == 0.10
        assert event is None

    _, omega, event = controller.step(
        now_s=1.0,
        current_heading=math.radians(-8.0),
        target_heading=0.0,
        tolerance_rad=math.radians(2.0),
        key=("large", 1),
    )
    assert omega == 0.0
    assert event == "continuous_settle"


def test_cardinal_follower_drives_directly_when_aligned_and_stops_to_realign():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.lane_heading_lock_enabled = True
    node.lane_heading_initial_align_enabled = True
    node.lane_heading_axis_tolerance_m = 0.06
    node.lane_heading_align_tolerance_rad = math.radians(2.0)
    node.lane_heading_realign_tolerance_rad = math.radians(3.0)
    node.lane_heading_soft_entry_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_speed = 0.07
    node.lane_heading_soft_entry_kp = 0.40
    node.lane_heading_soft_entry_omega_max = 0.04
    node.lane_heading_soft_entry_timeout_sec = 1.0
    node.lane_heading_realign_arm_sec = 0.0
    node.lane_heading_realign_hold_sec = 0.45
    node.lane_heading_drive_omega_max = 0.0
    node.lane_heading_filter_alpha = 0.20
    node.lane_heading_settle_sec = 0.30
    node.lane_heading_reverse_settle_sec = 0.30
    node.lane_heading_kp = 1.2
    node.lane_heading_omega_max = 0.16
    node.lane_heading_deadband_rad = math.radians(1.0)
    node.direct_nav_speed = 0.12
    node.direct_nav_stop_radius_m = 0.08
    node.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0)
    node._lane_heading_segment_key = None
    node._lane_heading_phase = "align"
    node._lane_heading_filtered = None
    node._lane_heading_violation_start_s = None
    node._lane_heading_realign_armed_at_s = 0.0
    node._lane_heading_settle_start_s = 0.0
    node._lane_heading_entry_start_s = 0.0
    node._lane_heading_turn_sign = 0
    node._lane_heading_reverse_start_s = 0.0

    now = [0.0]
    commands = []
    node._now_s = lambda: now[0]
    node._distance_to = lambda _x, _y: 1.0
    node._decide = lambda _text: None
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    args = ((0.0, 0.0), (1.0, 0.0), (1, 0))
    assert node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "drive"
    assert commands[-1] == (0.12, 0.0, 0.0)

    assert node._drive_cardinal_lane_segment(*args, speed_limit_mps=0.07)
    assert commands[-1] == (0.07, 0.0, 0.0)

    node.world.robot_theta = math.radians(10.0)
    for _ in range(20):
        now[0] += 0.05
        node._drive_cardinal_lane_segment(*args)
        if node._lane_heading_phase == "align":
            break
    assert node._lane_heading_phase == "align"
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert all(omega == 0.0 for vx, _vy, omega in commands if vx != 0.0)


def test_post_pick_cardinal_follower_forces_align_and_never_commands_strafe():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.lane_heading_lock_enabled = True
    node.lane_heading_initial_align_enabled = False
    node.lane_heading_axis_tolerance_m = 0.06
    node.lane_heading_align_tolerance_rad = math.radians(2.0)
    node.lane_heading_realign_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_speed = 0.07
    node.lane_heading_soft_entry_kp = 0.40
    node.lane_heading_soft_entry_omega_max = 0.04
    node.lane_heading_soft_entry_timeout_sec = 1.0
    node.lane_heading_realign_arm_sec = 0.0
    node.lane_heading_realign_hold_sec = 0.45
    node.lane_heading_drive_omega_max = 0.08
    node.lane_heading_filter_alpha = 0.20
    node.lane_heading_settle_sec = 0.30
    node.lane_heading_reverse_settle_sec = 0.30
    node.lane_heading_kp = 1.2
    node.lane_heading_omega_max = 0.16
    node.lane_heading_deadband_rad = math.radians(3.0)
    node.direct_nav_speed = 0.12
    node.direct_nav_stop_radius_m = 0.08
    node.world = SimpleNamespace(
        robot_x=0.0, robot_y=0.0, robot_theta=math.radians(30.0)
    )
    node._lane_heading_segment_key = None
    node._lane_heading_phase = "align"
    node._lane_heading_filtered = None
    node._lane_heading_violation_start_s = None
    node._lane_heading_realign_armed_at_s = 0.0
    node._lane_heading_settle_start_s = 0.0
    node._lane_heading_entry_start_s = 0.0
    node._lane_heading_turn_sign = 0
    node._lane_heading_reverse_start_s = 0.0

    now = [0.0]
    commands = []
    node._now_s = lambda: now[0]
    node._distance_to = lambda _x, _y: 1.0
    node._decide = lambda _text: None
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    args = ((0.0, 0.0), (1.0, 0.0), (7, 1))
    assert node._drive_cardinal_lane_segment(
        *args, force_initial_align=True, forward_only=True
    )
    assert node._lane_heading_phase == "align"
    assert commands[-1][0] == 0.0
    assert commands[-1][1] == 0.0

    now[0] = 0.1
    node._drive_cardinal_lane_segment(
        *args, force_initial_align=True, forward_only=True
    )
    assert commands[-1] == (0.0, 0.0, -0.10)

    node.world.robot_theta = 0.0
    now[0] = 0.25
    node._drive_cardinal_lane_segment(
        *args, force_initial_align=True, forward_only=True
    )
    now[0] = 0.44
    node._drive_cardinal_lane_segment(
        *args, force_initial_align=True, forward_only=True
    )
    now[0] = 0.45
    node._drive_cardinal_lane_segment(
        *args, force_initial_align=True, forward_only=True
    )
    assert node._lane_heading_phase == "settle"
    now[0] = 0.76
    node._drive_cardinal_lane_segment(
        *args, force_initial_align=True, forward_only=True
    )
    assert node._lane_heading_phase == "drive"
    now[0] = 0.77
    node._drive_cardinal_lane_segment(
        *args, force_initial_align=True, forward_only=True
    )
    assert commands[-1] == (0.12, 0.0, 0.0)
    assert all(vy == 0.0 for _vx, vy, _omega in commands)


def test_cardinal_follower_uses_stopped_fine_pulses_for_small_entry_error():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.lane_heading_lock_enabled = True
    node.lane_heading_initial_align_enabled = True
    node.lane_heading_axis_tolerance_m = 0.06
    node.lane_heading_align_tolerance_rad = math.radians(2.0)
    node.lane_heading_realign_tolerance_rad = math.radians(3.0)
    node.lane_heading_soft_entry_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_speed = 0.07
    node.lane_heading_soft_entry_kp = 0.40
    node.lane_heading_soft_entry_omega_max = 0.04
    node.lane_heading_soft_entry_timeout_sec = 1.0
    node.lane_heading_realign_arm_sec = 0.0
    node.lane_heading_realign_hold_sec = 0.45
    node.lane_heading_drive_omega_max = 0.0
    node.lane_heading_filter_alpha = 0.20
    node.lane_heading_settle_sec = 0.30
    node.lane_heading_reverse_settle_sec = 0.30
    node.lane_heading_kp = 1.2
    node.lane_heading_omega_max = 0.16
    node.lane_heading_deadband_rad = math.radians(1.0)
    node.direct_nav_speed = 0.12
    node.direct_nav_stop_radius_m = 0.08
    node.world = SimpleNamespace(
        robot_x=0.0, robot_y=0.0, robot_theta=math.radians(-6.0)
    )
    node._lane_heading_segment_key = None
    node._lane_heading_phase = "align"
    node._lane_heading_filtered = None
    node._lane_heading_violation_start_s = None
    node._lane_heading_realign_armed_at_s = 0.0
    node._lane_heading_settle_start_s = 0.0
    node._lane_heading_entry_start_s = 0.0
    node._lane_heading_turn_sign = 0
    node._lane_heading_reverse_start_s = 0.0

    now = [0.0]
    commands = []
    decisions = []
    node._now_s = lambda: now[0]
    node._distance_to = lambda _x, _y: 1.0
    node._decide = decisions.append
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    args = ((0.0, 0.0), (1.0, 0.0), (1, 0))
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "align"
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert decisions[1] == "LANE ENTRY PULSE ALIGN error=+6.0deg"

    now[0] = 0.61
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.62
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1] == (0.0, 0.0, 0.10)
    assert all(vx == 0.0 and vy == 0.0 for vx, vy, _omega in commands)


def test_cardinal_follower_stops_before_reversing_turn_direction():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.lane_heading_lock_enabled = True
    node.lane_heading_initial_align_enabled = True
    node.lane_heading_axis_tolerance_m = 0.06
    node.lane_heading_align_tolerance_rad = math.radians(2.0)
    node.lane_heading_realign_tolerance_rad = math.radians(3.0)
    node.lane_heading_soft_entry_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_speed = 0.07
    node.lane_heading_soft_entry_kp = 0.40
    node.lane_heading_soft_entry_omega_max = 0.04
    node.lane_heading_soft_entry_timeout_sec = 1.0
    node.lane_heading_realign_arm_sec = 0.0
    node.lane_heading_realign_hold_sec = 0.45
    node.lane_heading_drive_omega_max = 0.0
    node.lane_heading_filter_alpha = 0.20
    node.lane_heading_settle_sec = 0.30
    node.lane_heading_reverse_settle_sec = 0.30
    node.lane_heading_kp = 1.2
    node.lane_heading_omega_max = 0.16
    node.lane_heading_deadband_rad = math.radians(1.0)
    node.direct_nav_speed = 0.12
    node.direct_nav_stop_radius_m = 0.08
    node.world = SimpleNamespace(
        robot_x=0.0, robot_y=0.0, robot_theta=math.radians(-20.0)
    )
    node._lane_heading_segment_key = None
    node._lane_heading_phase = "align"
    node._lane_heading_filtered = None
    node._lane_heading_violation_start_s = None
    node._lane_heading_realign_armed_at_s = 0.0
    node._lane_heading_settle_start_s = 0.0
    node._lane_heading_entry_start_s = 0.0
    node._lane_heading_turn_sign = 0
    node._lane_heading_reverse_start_s = 0.0

    now = [0.0]
    commands = []
    decisions = []
    node._now_s = lambda: now[0]
    node._distance_to = lambda _x, _y: 1.0
    node._decide = decisions.append
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    args = ((0.0, 0.0), (1.0, 0.0), (1, 0))
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.05
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1][2] > 0.0

    now[0] = 0.10
    node.world.robot_theta = math.radians(20.0)
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1][2] > 0.0

    now[0] = 0.25
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.44
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "align"
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.45
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.46
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1][2] < 0.0


def test_cardinal_follower_aligns_before_entry_and_never_translates_laterally():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.lane_heading_lock_enabled = True
    node.lane_heading_initial_align_enabled = False
    node.lane_heading_axis_tolerance_m = 0.06
    node.lane_heading_align_tolerance_rad = math.radians(2.0)
    node.lane_heading_realign_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_speed = 0.07
    node.lane_heading_soft_entry_kp = 0.40
    node.lane_heading_soft_entry_omega_max = 0.04
    node.lane_heading_soft_entry_timeout_sec = 1.0
    node.lane_heading_realign_arm_sec = 1.2
    node.lane_heading_realign_hold_sec = 0.8
    node.lane_heading_drive_omega_max = 0.03
    node.lane_heading_filter_alpha = 0.20
    node.lane_heading_settle_sec = 0.30
    node.lane_heading_reverse_settle_sec = 0.30
    node.lane_heading_kp = 1.2
    node.lane_heading_omega_max = 0.16
    node.lane_heading_deadband_rad = math.radians(1.0)
    node.direct_nav_speed = 0.12
    node.direct_nav_stop_radius_m = 0.08
    node.world = SimpleNamespace(
        robot_x=0.0, robot_y=0.0, robot_theta=math.pi / 2.0
    )
    node._lane_heading_segment_key = None
    node._lane_heading_phase = "align"
    node._lane_heading_filtered = None
    node._lane_heading_violation_start_s = None
    node._lane_heading_realign_armed_at_s = 0.0
    node._lane_heading_settle_start_s = 0.0
    node._lane_heading_entry_start_s = 0.0
    node._lane_heading_turn_sign = 0
    node._lane_heading_reverse_start_s = 0.0

    now = [0.0]
    commands = []
    decisions = []
    node._now_s = lambda: now[0]
    node._distance_to = lambda _x, _y: 1.0
    node._decide = decisions.append
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    args = ((0.0, 0.0), (1.0, 0.0), (1, 0))
    node._drive_cardinal_lane_segment(*args)

    assert node._lane_heading_phase == "align"
    assert commands[-1] == (0.0, 0.0, -0.10)
    assert decisions[1] == "LANE ENTRY PULSE ALIGN error=-90.0deg"

    now[0] = 0.05
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1] == (0.0, 0.0, -0.10)

    node.world.robot_theta = 0.0
    now[0] = 0.25
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1] == (0.0, 0.0, 0.0)
    now[0] = 0.84
    node._drive_cardinal_lane_segment(*args)
    now[0] = 0.85
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "settle"

    now[0] = 1.16
    node._drive_cardinal_lane_segment(*args)
    now[0] = 1.17
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "drive"
    assert commands[-1] == (0.07, 0.0, 0.0)
    assert all(vy == 0.0 for _vx, vy, _omega in commands)


def test_cardinal_follower_corrects_small_drift_during_realign_arm_window():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.lane_heading_lock_enabled = True
    node.lane_heading_initial_align_enabled = False
    node.lane_heading_axis_tolerance_m = 0.06
    node.lane_heading_align_tolerance_rad = math.radians(2.0)
    node.lane_heading_realign_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_tolerance_rad = math.radians(10.0)
    node.lane_heading_soft_entry_speed = 0.07
    node.lane_heading_soft_entry_kp = 0.60
    node.lane_heading_soft_entry_omega_max = 0.04
    node.lane_heading_soft_entry_timeout_sec = 1.0
    node.lane_heading_realign_arm_sec = 1.2
    node.lane_heading_realign_hold_sec = 0.8
    node.lane_heading_drive_omega_max = 0.08
    node.lane_heading_filter_alpha = 1.0
    node.lane_heading_settle_sec = 0.30
    node.lane_heading_reverse_settle_sec = 0.30
    node.lane_heading_kp = 1.2
    node.lane_heading_omega_max = 0.16
    node.lane_heading_deadband_rad = math.radians(3.0)
    node.direct_nav_speed = 0.12
    node.direct_nav_stop_radius_m = 0.08
    node.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0)
    node._lane_heading_segment_key = None
    node._lane_heading_phase = "align"
    node._lane_heading_filtered = None
    node._lane_heading_violation_start_s = None
    node._lane_heading_realign_armed_at_s = 0.0
    node._lane_heading_settle_start_s = 0.0
    node._lane_heading_entry_start_s = 0.0
    node._lane_heading_turn_sign = 0
    node._lane_heading_reverse_start_s = 0.0

    now = [0.0]
    commands = []
    node._now_s = lambda: now[0]
    node._distance_to = lambda _x, _y: 1.0
    node._decide = lambda _text: None
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    node._drive_cardinal_lane_segment((0.0, 0.0), (1.0, 0.0), (1, 0))
    assert commands[-1] == (0.07, 0.0, 0.0)

    now[0] = 0.1
    node.world.robot_theta = math.radians(-5.0)
    node._drive_cardinal_lane_segment((0.0, 0.0), (1.0, 0.0), (1, 0))
    assert commands[-1][0] > 0.0
    assert commands[-1][1] == 0.0
    assert commands[-1][2] > 0.0
    assert commands[-1][2] <= 0.08
