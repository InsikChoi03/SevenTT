import math
from types import SimpleNamespace

from robot_planning.lane_planner import LanePlanner
from robot_planning.nodes.mission_fsm_node import MissionFsmNode


def _approach_node():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0)
    node.approach_dist_m = 0.375
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
    assert commands[-1][2] < 0.0

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
