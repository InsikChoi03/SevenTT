import math
from types import SimpleNamespace

from robot_planning.lane_planner import LanePlanner
from robot_planning.nodes.mission_fsm_node import MissionFsmNode


def _approach_node():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0)
    node.approach_dist_m = 0.375
    node.wp_reach_tol_m = 0.12
    node.approach_standoff_tol = 0.09
    node.approach_brake_settle_sec = 0.40
    node.approach_heading_filter_alpha = 1.0
    node.approach_heading_stable_frames = 3
    node.direct_nav_kp_ang = 1.5
    node.planner_enabled = True
    node.current_target = SimpleNamespace(id=7)
    node._planner = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        origin_mode="fixed",
        simplify=False,
    )
    node._robot_xy = lambda: (node.world.robot_x, node.world.robot_y)
    node._obstacles_snapshot = lambda _exclude_id, _dest: []
    node._decide = lambda _text: None
    node._reset_approach_motion()
    return node


def test_approach_goal_is_latched_to_lane_centre_around_target():
    node = _approach_node()

    assert node._latch_approach_goal((1.0, 0.0))
    assert node._approach_standoff_xy in {
        (0.75, -0.25),
        (0.75, 0.25),
        (1.25, -0.25),
        (1.25, 0.25),
    }
    sx, sy = node._approach_standoff_xy
    assert math.isclose(
        node._approach_heading_latched,
        math.atan2(-sy, 1.0 - sx),
        abs_tol=1e-9,
    )

    node.world.robot_x = 0.4
    node.world.robot_y = 0.2
    assert node._approach_standoff_xy == (sx, sy)


def test_global_target_approach_uses_collision_checked_fast_route():
    node = _approach_node()
    node.global_target_mode = True
    node.global_fast_safe_routes_enabled = True

    assert node._latch_approach_goal((1.0, 0.0))
    assert node._approach_route_mode == "legacy"


def test_non_global_approach_keeps_strict_lane_route():
    node = _approach_node()
    node.global_target_mode = False
    node.global_fast_safe_routes_enabled = True

    assert node._global_exploration_route_mode() == "grid_only"


def test_near_target_skips_grid_standoff_and_enters_body_align_from_current_pose():
    node = _approach_node()
    decisions = []
    node._decide = decisions.append

    assert node._latch_approach_goal((0.22, 0.0))
    assert node._approach_standoff_xy == (0.0, 0.0)
    assert node._approach_route_mode == "legacy"
    assert any("TARGET ALREADY NEAR" in text for text in decisions)


def test_missing_grid_standoff_falls_back_to_radial_direct_goal():
    node = _approach_node()
    decisions = []
    node._decide = decisions.append
    node._planner.plan_object_standoff = lambda *_args, **_kwargs: None

    assert node._latch_approach_goal((1.0, 0.0))
    assert node._approach_standoff_xy == (0.625, 0.0)
    assert node._approach_route_mode == "legacy"
    assert any("STAND-OFF unavailable -> DIRECT" in text for text in decisions)


def test_approach_arrival_brakes_before_three_stable_heading_frames():
    node = _approach_node()
    now = [0.0]
    commands = []
    node._now_s = lambda: now[0]
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))
    node._latch_approach_goal((1.0, 0.0))
    node.world.robot_theta = node._approach_heading_latched

    handled, ready = node._step_approach_arrival(0.05, math.radians(5.7), 0.405)
    assert handled and not ready
    assert node._approach_motion_phase == "brake_settle"
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] = 0.39
    _, ready = node._step_approach_arrival(0.12, math.radians(5.7), 0.405)
    assert not ready
    assert node._approach_motion_phase == "brake_settle"

    now[0] = 0.40
    _, ready = node._step_approach_arrival(0.12, math.radians(5.7), 0.405)
    assert not ready
    assert node._approach_motion_phase == "heading_align"

    for expected_count in (1, 2):
        now[0] += 0.01
        _, ready = node._step_approach_arrival(0.12, math.radians(5.7), 0.405)
        assert not ready
        assert node._approach_heading_stable_count == expected_count

    now[0] += 0.01
    _, ready = node._step_approach_arrival(0.12, math.radians(5.7), 0.405)
    assert ready
    assert node._approach_motion_phase == "ready"
    assert all(command == (0.0, 0.0, 0.0) for command in commands)


def test_fresh_body_target_skips_map_heading_after_brake_settle():
    node = _approach_node()
    now = [0.0]
    commands = []
    decisions = []
    node._now_s = lambda: now[0]
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))
    node._decide = decisions.append
    node._latch_approach_goal((1.0, 0.0))
    node.world.robot_theta = node._approach_heading_latched + math.radians(120.0)

    handled, ready = node._step_approach_arrival(
        0.05, math.radians(5.7), 0.405, body_target_visible=True
    )
    assert handled and not ready

    now[0] = 0.40
    handled, ready = node._step_approach_arrival(
        0.05, math.radians(5.7), 0.405, body_target_visible=True
    )
    assert handled and ready
    assert node._approach_motion_phase == "ready"
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert decisions[-1] == "APPROACH BODY TARGET VISIBLE -> SKIP MAP HEADING / ALIGN"


def test_body_target_stops_heading_turn_as_soon_as_it_is_acquired():
    node = _approach_node()
    commands = []
    decisions = []
    node._now_s = lambda: 1.0
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))
    node._decide = decisions.append
    node._latch_approach_goal((1.0, 0.0))
    node._approach_motion_phase = "heading_align"
    node._approach_heading_filtered = node.world.robot_theta

    handled, ready = node._step_approach_arrival(
        0.05, math.radians(5.7), 0.405, body_target_visible=True
    )

    assert handled and ready
    assert node._approach_motion_phase == "ready"
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert decisions[-1] == "APPROACH BODY TARGET VISIBLE -> STOP HEADING TURN / ALIGN"


def test_second_body_loss_backoff_is_reserved_for_committed_set2_target():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.align_body_lost_max_backoffs = 2
    node._committed_set2_target_can_try_align = lambda: False

    assert node._align_body_lost_backoff_limit() == 1

    node._committed_set2_target_can_try_align = lambda: True
    assert node._align_body_lost_backoff_limit() == 2


def test_body_lost_lateral_sweep_uses_wide_side_and_only_starts_once():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.align_body_lost_lateral_sweep_enabled = True
    node.global_target_mode = True
    node.world = SimpleNamespace(robot_theta=0.2)
    node.current_target = SimpleNamespace(id=7, set_type=2, x=1.0, y=0.2)
    node._appr_tgt_xy = (1.0, 0.2)
    node._body_lost_lateral_sweep_counts = {}
    node._wide_align_target_base = lambda: (0.35, 0.08)
    node._wide_align_expected_base = lambda: (0.35, 0.08)
    node._reset_align_body_loss_confirmation = lambda: None
    node._reset_align_body_presence = lambda: None
    node._decide = lambda _text: None
    node.get_logger = lambda: SimpleNamespace(info=lambda _text: None)

    assert node._start_align_body_lost_lateral_sweep(4.0)
    assert node._align_phase == "body_lost_lateral_sweep"
    assert node._align_body_lost_sweep_strafe_sign == 1.0
    assert not node._start_align_body_lost_lateral_sweep(5.0)


def test_body_lost_lateral_recovery_never_commands_rotation():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._time_in_state = lambda: 1.0
    node._now_s = lambda: 5.0
    node.align_timeout_sec = 12.0
    node._align_phase = "body_lost_lateral_sweep"
    node._align_phase_start = 4.5
    node.align_body_lost_lateral_sweep_sec = 1.2
    node.align_body_lost_lateral_sweep_duty = 0.315
    node._align_body_lost_sweep_strafe_sign = -1.0
    commands = []
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    MissionFsmNode._step_align(node)

    assert commands == [(0.0, -0.315, 0.0)]


def test_global_body_lost_reapproaches_same_position_once():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.align_body_lost_same_slot_retry_enabled = True
    node.align_body_lost_same_slot_retries = 1
    node.global_target_mode = True
    node.current_target = SimpleNamespace(id=8, set_type=2, x=1.04, y=-0.26)
    node._appr_tgt_xy = (1.0, -0.3)
    node._global_body_lost_retry_counts = {}
    node._current_slot = lambda: None
    node._plan = object()
    node._now_s = lambda: 8.0
    entered = []
    latched = []
    node._enter = entered.append
    node._latch_approach_goal = lambda xy: latched.append(xy) or True
    node._decide = lambda _text: None
    node.get_logger = lambda: SimpleNamespace(info=lambda _text: None)

    assert node._same_slot_reapproach_after_body_lost()
    assert entered == ["APPROACH"]
    assert latched == [(1.0, -0.3)]
    assert node._require_precise_heading_before_align is True
    assert not node._same_slot_reapproach_after_body_lost()


def test_approach_heading_count_resets_until_filtered_heading_is_stable():
    node = _approach_node()
    now = [0.0]
    commands = []
    node._now_s = lambda: now[0]
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))
    node._latch_approach_goal((1.0, 0.0))
    target_heading = node._approach_heading_latched
    node.world.robot_theta = target_heading
    node._approach_motion_phase = "heading_align"
    node._approach_heading_filtered = target_heading

    node._step_approach_arrival(0.05, math.radians(5.7), 0.405)
    assert node._approach_heading_stable_count == 1

    node.world.robot_theta = target_heading + math.radians(20.0)
    _, ready = node._step_approach_arrival(0.05, math.radians(5.7), 0.405)
    assert not ready
    assert node._approach_heading_stable_count == 0
    assert commands[-1] == (0.0, 0.0, 0.0)

    now[0] += 0.01
    _, ready = node._step_approach_arrival(0.05, math.radians(5.7), 0.405)
    assert not ready
    assert commands[-1][2] < 0.0

    now[0] += 0.30
    node.world.robot_theta = target_heading
    results = []
    for _ in range(3):
        _, ready = node._step_approach_arrival(0.05, math.radians(5.7), 0.405)
        results.append(ready)
    assert results == [False, False, True]


def test_only_final_object_connector_requires_waypoint_brake():
    node = _approach_node()
    node._plan_route_mode = "object_approach"
    node._plan = [(0.0, 0.0), (0.5, 0.0), (0.7, 0.2)]
    node.lane_heading_axis_tolerance_m = 0.06

    assert not node._object_connector_brake_required(0)
    assert node._object_connector_brake_required(1)

    node._plan[-1] = (0.7, 0.0)
    assert node._object_connector_brake_required(1)

    node._plan[-1] = (0.7, 0.2)
    node._plan_route_mode = "grid_only"
    assert not node._object_connector_brake_required(1)


def test_post_pick_route_mode_is_consumed_by_one_latched_plan():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._plan = None
    node._plan_route_mode = "grid_only"
    node._post_pick_lane_entry_pending = True

    assert node._next_travel_route_mode() == "post_pick_entry"

    node._post_pick_lane_entry_pending = False
    node._plan = [(-0.75, 0.75), (-0.25, 0.75)]
    node._plan_route_mode = "post_pick_entry"
    assert node._next_travel_route_mode() == "post_pick_entry"

    node._plan = None
    assert node._next_travel_route_mode() == "grid_only"


def test_missing_strict_lane_route_retries_collision_checked_flexible_route():
    node = MissionFsmNode.__new__(MissionFsmNode)
    calls = []

    class Planner:
        def plan(self, start, dest, obstacles, route_mode):
            calls.append(route_mode)
            return None if route_mode == "post_pick_entry" else [dest]

    node._planner = Planner()
    node.direct_fallback_enabled = True

    vias, flexible = node._plan_with_flexible_fallback(
        (0.1, 0.1), (1.0, 1.0), [(0.5, 0.5)], "post_pick_entry"
    )

    assert calls == ["post_pick_entry", "legacy"]
    assert vias == [(1.0, 1.0)]
    assert flexible


def test_missing_lane_route_does_not_retry_when_fallback_is_disabled():
    node = MissionFsmNode.__new__(MissionFsmNode)
    calls = []

    class Planner:
        def plan(self, start, dest, obstacles, route_mode):
            calls.append(route_mode)
            return None

    node._planner = Planner()
    node.direct_fallback_enabled = False

    vias, flexible = node._plan_with_flexible_fallback(
        (0.1, 0.1), (1.0, 1.0), [], "grid_only"
    )

    assert calls == ["grid_only"]
    assert vias is None
    assert not flexible


def test_post_pick_lane_entry_aligns_then_drives_without_omega():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0)
    node.wp_reach_tol_m = 0.12
    node.lane_heading_align_tolerance_rad = math.radians(2.0)
    node.direct_nav_kp_ang = 1.5
    node.direct_nav_omega_max = 0.405
    node.direct_nav_speed = 0.12
    now = [0.0]
    commands = []
    node._now_s = lambda: now[0]
    node._decide = lambda _text: None
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))

    node._drive_to_post_pick_lane_entry(0.0, 1.0)
    assert commands[-1] == (0.0, 0.0, 0.1)

    now[0] = 0.05
    node._drive_to_post_pick_lane_entry(0.0, 1.0)
    assert commands[-1][0:2] == (0.0, 0.0)
    assert commands[-1][2] > 0.0

    node.world.robot_theta = math.pi / 2.0
    now[0] = 0.25
    node._drive_to_post_pick_lane_entry(0.0, 1.0)
    now[0] = 0.84
    node._drive_to_post_pick_lane_entry(0.0, 1.0)
    now[0] = 0.85
    node._drive_to_post_pick_lane_entry(0.0, 1.0)
    assert commands[-1] == (0.12, 0.0, 0.0)

    node.world.robot_y = 0.90
    now[0] = 0.86
    node._drive_to_post_pick_lane_entry(0.0, 1.0)
    assert commands[-1] == (0.0, 0.0, 0.0)


def test_final_object_connector_brake_holds_zero_for_settle_time():
    node = _approach_node()
    node._plan = [(0.5, 0.0), (0.7, 0.2)]
    node._plan_generation = 4
    node._object_connector_brake_key = None
    node._object_connector_brake_start_s = 0.0
    now = [2.0]
    commands = []
    decisions = []
    node._now_s = lambda: now[0]
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))
    node._decide = decisions.append

    assert node._step_object_connector_brake(0)
    assert node._object_connector_brake_key == (4, 0)

    now[0] = 2.39
    assert node._step_object_connector_brake(0)

    now[0] = 2.41
    assert not node._step_object_connector_brake(0)
    assert node._object_connector_brake_key is None
    assert commands == [(0.0, 0.0, 0.0)] * 3
    assert decisions == [
        "OBJECT CONNECTOR BRAKE (0.50,0.00)",
        "OBJECT CONNECTOR BRAKE SETTLED -> FINAL APPROACH",
    ]
