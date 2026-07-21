import math
from types import SimpleNamespace

from robot_planning.lane_planner import cardinal_segment_heading
from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    circular_heading_filter,
    lane_heading_violation_time_step,
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


def test_circular_heading_filter_handles_wraparound():
    filtered = circular_heading_filter(math.radians(179.0), math.radians(-179.0), 0.5)
    error = math.atan2(math.sin(filtered - math.pi), math.cos(filtered - math.pi))
    assert math.isclose(error, 0.0, abs_tol=1e-9)


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

    node.world.robot_theta = math.radians(10.0)
    for _ in range(20):
        now[0] += 0.05
        node._drive_cardinal_lane_segment(*args)
        if node._lane_heading_phase == "align":
            break
    assert node._lane_heading_phase == "align"
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert all(omega == 0.0 for vx, _vy, omega in commands if vx != 0.0)


def test_cardinal_follower_uses_soft_entry_and_never_reverses_entry_omega():
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
    assert node._lane_heading_phase == "soft_entry"
    assert commands[-1] == (0.07, 0.0, 0.04)
    assert decisions[-1] == "LANE SOFT ENTRY error=+6.0deg"

    now[0] = 0.1
    node.world.robot_theta = math.radians(-2.0)
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "drive"
    assert commands[-1] == (0.12, 0.0, 0.0)

    now[0] = 0.2
    node.world.robot_theta = math.radians(-6.0)
    crossed_args = ((0.0, 0.0), (1.0, 0.0), (2, 0))
    node._drive_cardinal_lane_segment(*crossed_args)
    assert commands[-1][2] > 0.0

    now[0] = 0.3
    node.world.robot_theta = math.radians(4.0)
    node._drive_cardinal_lane_segment(*crossed_args)
    assert node._lane_heading_phase == "drive"
    assert commands[-1] == (0.12, 0.0, 0.0)
    assert decisions[-1] == "LANE SOFT ENTRY CROSSED error=-4.0deg"


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
    assert commands[-1][2] > 0.0

    now[0] = 0.05
    node.world.robot_theta = math.radians(20.0)
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "reverse_settle"
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert decisions[-1] == "LANE TURN REVERSAL -> SETTLE 0.30s"

    now[0] = 0.34
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "reverse_settle"
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.36
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "align"
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.37
    node._drive_cardinal_lane_segment(*args)
    assert commands[-1][2] < 0.0


def test_cardinal_follower_skips_initial_turn_and_translates_in_base_frame():
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

    assert node._lane_heading_phase == "drive"
    assert math.isclose(commands[-1][0], 0.0, abs_tol=1e-9)
    assert math.isclose(commands[-1][1], -0.07, abs_tol=1e-9)
    assert commands[-1][2] == 0.0
    assert decisions[-1] == "LANE ENTRY TRANSLATE error=-90.0deg"

    now[0] = 1.0
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "drive"
    assert commands[-1][2] == 0.0

    now[0] = 1.25
    node._drive_cardinal_lane_segment(*args)
    assert node._lane_heading_phase == "drive"
    assert math.isclose(commands[-1][1], -0.12, abs_tol=1e-9)
    assert commands[-1][2] == -0.03


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
    node.world = SimpleNamespace(
        robot_x=0.0, robot_y=0.0, robot_theta=math.radians(-5.0)
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

    commands = []
    node._now_s = lambda: 0.0
    node._distance_to = lambda _x, _y: 1.0
    node._decide = lambda _text: None
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    node._drive_cardinal_lane_segment((0.0, 0.0), (1.0, 0.0), (1, 0))

    assert commands[-1][0] > 0.0
    assert commands[-1][2] > 0.0
    assert commands[-1][2] <= 0.08
