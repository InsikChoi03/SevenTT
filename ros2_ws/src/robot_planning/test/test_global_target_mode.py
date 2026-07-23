"""Global target mode: whole-map tour selection tiers, exclusions, and quotas."""
from types import SimpleNamespace

from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    global_retarget_is_worthwhile,
    global_target_absence_confirmed,
    global_target_in_route_corridor,
    holonomic_waypoint_velocity,
    keep_latched_global_approach,
    target_is_spatially_deferred,
)


def test_holonomic_waypoint_velocity_preserves_direction_without_rotation():
    vx, vy = holonomic_waypoint_velocity(0.30, -0.40, 0.20, 0.10)

    assert round(vx, 6) == 0.12
    assert round(vy, 6) == -0.16


def test_holonomic_waypoint_velocity_stops_inside_radius():
    assert holonomic_waypoint_velocity(0.03, 0.04, 0.20, 0.10) == (0.0, 0.0)


def _obj(
    oid,
    x,
    y,
    set_type,
    label="",
    fruit="",
    conf=0.9,
    blacklisted=False,
    fruit_confidence=0.0,
    fruit_source="",
):
    return SimpleNamespace(
        id=oid,
        x=x,
        y=y,
        set_type=set_type,
        class_label=label,
        fruit_label=fruit,
        confidence=conf,
        fruit_confidence=fruit_confidence,
        fruit_label_source=fruit_source,
        blacklisted=blacklisted,
    )


def _node(objects, rx=0.0, ry=0.0, tray_shape=0, tray_fruit=0):
    node = SimpleNamespace(
        global_target_mode=True,
        world=SimpleNamespace(robot_x=rx, robot_y=ry, objects=objects),
        zone_mission_enabled=False,
        set1_label="dodecahedron",
        set2_label="banana",
        set2_require_fruit_label=True,
        pick_track_conf=0.5,
        tray_shape=tray_shape,
        tray_fruit=tray_fruit,
        shape_target_total=4,
        fruit_target_total=3,
        global_target_corridor_gate_enabled=False,
        _is_zone_ambiguous_excluded=lambda o: False,
    )
    node._shape_quota_available = lambda: MissionFsmNode._shape_quota_available(node)
    node._fruit_quota_available = lambda: MissionFsmNode._fruit_quota_available(node)
    node._iter_set1_targets = lambda: MissionFsmNode._iter_set1_targets(node)
    node._iter_set2_target_objects = lambda: MissionFsmNode._iter_set2_target_objects(node)
    node._global_target_in_active_corridor = (
        lambda target: MissionFsmNode._global_target_in_active_corridor(node, target)
    )
    return node


def _select(node):
    return MissionFsmNode._select_next_global_target(node)


def test_tour_head_minimises_total_path_across_both_sets():
    # Nearest object is the dodecahedron at (1,0), but starting with the banana
    # at (-1,0) gives the shorter full tour (see planner test for the geometry).
    objects = [
        _obj(1, 1.0, 0.0, 1, label="dodecahedron"),
        _obj(2, 3.0, 0.0, 1, label="dodecahedron"),
        _obj(3, -1.0, 0.0, 2, label="fruit_photo_cube", fruit="banana"),
    ]
    kind, tgt = _select(_node(objects))
    assert (kind, int(tgt.id)) == ("set2_object", 3)


def test_route_corridor_accepts_nearby_target_and_rejects_cross_field_target():
    assert global_target_in_route_corridor(
        (-1.50, 0.50), (-1.75, 1.75), (-1.75, -1.25), 0.45
    )
    assert not global_target_in_route_corridor(
        (-1.00, 0.50), (-1.75, 1.75), (-1.75, -1.25), 0.45
    )


def test_global_selection_only_interrupts_for_target_near_active_zigzag_leg():
    node = _node([
        _obj(1, -1.50, 0.50, 1, label="dodecahedron"),
        _obj(2, -1.00, 0.50, 2, label="fruit_photo_cube", fruit="banana"),
    ], rx=-1.75, ry=1.75)
    node.global_target_corridor_gate_enabled = True
    node.global_target_corridor_radius_m = 0.45
    node.global_zigzag_start = (-1.75, 1.75)
    node._global_zigzag_waypoints = [(-1.75, -1.25), (-0.75, -1.25)]
    node._global_zigzag_idx = 0
    node._global_patrol_goal_zigzag_index = None

    kind, target = _select(node)

    assert (kind, int(target.id)) == ("set1", 1)


def test_corridor_gate_is_removed_after_initial_sweep_completion():
    node = _node([
        _obj(1, 1.25, 0.50, 1, label="dodecahedron"),
    ], rx=-1.25, ry=-1.25)
    node.global_target_corridor_gate_enabled = True
    node.global_target_corridor_radius_m = 0.45
    node._global_zigzag_waypoints = [(-1.25, 1.75), (-1.25, -1.25), (1.25, -1.25)]
    node._global_zigzag_idx = 3
    node._global_initial_sweep_complete = True

    kind, target = _select(node)

    assert (kind, int(target.id)) == ("set1", 1)


def test_strong_target_fruit_positive_wins_after_initial_sweep():
    node = _node([
        _obj(
            1, 0.20, 0.0, 2, label="fruit_photo_cube", fruit="banana",
            fruit_confidence=0.30, fruit_source="wide",
        ),
        _obj(
            2, 1.20, 0.0, 2, label="fruit_photo_cube", fruit="banana",
            fruit_confidence=0.85, fruit_source="wide",
        ),
    ])
    node._global_initial_sweep_complete = True
    node.global_strong_fruit_positive_min_confidence = 0.50

    kind, target = _select(node)

    assert (kind, int(target.id)) == ("set2_object", 2)


def test_weak_non_target_fruit_label_is_revisited_for_body_verification():
    node = _node([
        _obj(
            7, -1.50, 0.50, 2, label="fruit_photo_cube", fruit="pineapple",
            fruit_confidence=0.25, fruit_source="wide",
        ),
    ], rx=-1.25, ry=0.75)
    node.global_non_target_fruit_reject_min_confidence = 0.40

    kind, target = _select(node)

    assert (kind, int(target.id)) == ("set2_object", 7)


def test_strong_non_target_fruit_label_is_not_revisited():
    node = _node([
        _obj(
            7, -1.50, 0.50, 2, label="fruit_photo_cube", fruit="pineapple",
            fruit_confidence=0.80, fruit_source="wide",
        ),
    ], rx=-1.25, ry=0.75)
    node.global_non_target_fruit_reject_min_confidence = 0.40

    assert _select(node) is None


def test_initial_sweep_finishes_without_wrapping_to_first_waypoint():
    decisions = []
    node = SimpleNamespace(
        _global_patrol_goal_zigzag_index=2,
        _global_zigzag_waypoints=[
            (-1.25, 1.75), (-1.25, -1.25), (1.25, -1.25),
        ],
        _global_zigzag_idx=2,
        _global_initial_sweep_complete=False,
        global_initial_sweep_once_enabled=True,
        _decide=decisions.append,
    )

    MissionFsmNode._advance_global_initial_sweep(node)

    assert node._global_zigzag_idx == 3
    assert node._global_initial_sweep_complete
    assert decisions == ["INITIAL SWEEP COMPLETE -> OPTIMIZE MAPPED TARGET TOUR"]


def test_initial_sweep_segment_uses_straight_feedback_without_planner():
    calls = []
    node = SimpleNamespace(
        _global_zigzag_waypoints=[
            (-1.25, 1.25), (-1.25, -1.25), (1.25, -1.25),
        ],
        _global_initial_sweep_start_xy=None,
        global_zigzag_start=(-1.80, 1.80),
        global_patrol_reach_tol_m=0.15,
        _plan=[(9.0, 9.0)],
        _plan_start_xy=(9.0, 9.0),
        _plan_dest=(9.0, 9.0),
        _robot_xy=lambda: (-1.70, 1.75),
        _distance_to=lambda _x, _y: 1.0,
        global_initial_sweep_slowdown_distance_m=0.45,
        global_initial_sweep_slow_speed_mps=0.07,
        _publish_planning_obstacles=lambda obstacles: calls.append(
            ("obstacles", list(obstacles))
        ),
        _drive=lambda vx, vy, omega: calls.append(("drive", vx, vy, omega)),
        _drive_toward_direct=lambda x, y, yaw: calls.append(
            ("direct", x, y, yaw)
        ),
    )

    def straight_feedback(start, dest, segment_key, **kwargs):
        calls.append(("straight", start, dest, segment_key, kwargs))
        return True

    node._drive_cardinal_lane_segment = straight_feedback
    node._initial_sweep_segment_start = (
        lambda goal_index: MissionFsmNode._initial_sweep_segment_start(
            node, goal_index
        )
    )

    MissionFsmNode._drive_initial_sweep_segment(node, -1.25, 1.25, 0)

    assert calls[0] == ("obstacles", [])
    assert calls[1][0] == "straight"
    assert calls[1][1] == (-1.70, 1.75)
    assert calls[1][2] == (-1.25, 1.25)
    assert calls[1][4]["allow_non_cardinal"]
    assert node._plan is None
    assert not any(call[0] == "direct" for call in calls)


def test_initial_sweep_segment_slows_near_waypoint():
    calls = []
    node = SimpleNamespace(
        _global_zigzag_waypoints=[
            (-1.25, 1.25), (-1.25, -1.25), (1.25, -1.25),
        ],
        _global_initial_sweep_start_xy=(-1.25, 1.60),
        global_zigzag_start=(-1.25, 1.60),
        global_patrol_reach_tol_m=0.15,
        global_initial_sweep_slowdown_distance_m=0.45,
        global_initial_sweep_slow_speed_mps=0.07,
        _plan=[(9.0, 9.0)],
        _plan_start_xy=(9.0, 9.0),
        _plan_dest=(9.0, 9.0),
        _distance_to=lambda _x, _y: 0.30,
        _publish_planning_obstacles=lambda obstacles: calls.append(
            ("obstacles", list(obstacles))
        ),
        _drive=lambda vx, vy, omega: calls.append(("drive", vx, vy, omega)),
        _drive_toward_direct=lambda x, y, yaw: calls.append(
            ("direct", x, y, yaw)
        ),
    )

    def straight_feedback(start, dest, segment_key, **kwargs):
        calls.append(("straight", start, dest, segment_key, kwargs))
        return True

    node._drive_cardinal_lane_segment = straight_feedback
    node._initial_sweep_segment_start = (
        lambda goal_index: MissionFsmNode._initial_sweep_segment_start(
            node, goal_index
        )
    )

    MissionFsmNode._drive_initial_sweep_segment(node, -1.25, 1.25, 0)

    assert calls[1][4]["speed_limit_mps"] == 0.07


def test_initial_sweep_accepts_waypoint_goal_line_overrun():
    calls = []
    decisions = []
    node = SimpleNamespace(
        _global_zigzag_waypoints=[
            (-1.25, 1.25), (-1.25, -1.25), (1.25, -1.25),
        ],
        _global_initial_sweep_start_xy=(-1.25, 1.60),
        global_patrol_reach_tol_m=0.15,
        lane_pass_lateral_tol_m=0.15,
        _robot_xy=lambda: (-1.45, -1.32),
        _drive=lambda vx, vy, omega: calls.append((vx, vy, omega)),
        _decide=decisions.append,
    )
    node._initial_sweep_segment_start = (
        lambda goal_index: MissionFsmNode._initial_sweep_segment_start(
            node, goal_index
        )
    )

    arrived = MissionFsmNode._initial_sweep_goal_arrived(node, -1.25, -1.25, 1)

    assert arrived
    assert calls == [(0.0, 0.0, 0.0)]
    assert decisions == [
        "GLOBAL ZIGZAG WAYPOINT OVERRUN lateral=0.20m -> ACCEPT TURN"
    ]


def test_global_target_outside_safe_robot_bounds_is_ignored():
    node = _node([
        _obj(1, 1.90, 0.0, 2, label="fruit_photo_cube", fruit="banana"),
        _obj(2, 1.50, 0.0, 1, label="dodecahedron"),
    ])
    node._planner = SimpleNamespace(interior=(-1.78, 1.78, -1.78, 1.78))

    kind, target = _select(node)

    assert kind == "set1"
    assert target.id == 2


def test_confirmed_targets_beat_untyped_cubes_regardless_of_distance():
    objects = [
        _obj(1, 0.2, 0.0, 2, label="fruit_photo_cube", fruit=""),
        _obj(2, 2.0, 2.0, 2, label="fruit_photo_cube", fruit="banana"),
    ]
    kind, tgt = _select(_node(objects))
    assert int(tgt.id) == 2


def test_local_blacklist_blocks_reselection_before_world_model_echo():
    node = _node([
        _obj(1, 0.2, 0.0, 2, label="fruit_photo_cube", fruit="banana"),
        _obj(2, 0.5, 0.0, 1, label="dodecahedron"),
    ])
    node._local_blacklisted_ids = {1}

    kind, target = _select(node)

    assert (kind, int(target.id)) == ("set1", 2)


def test_untyped_cube_is_the_fallback_when_no_confirmed_target():
    objects = [
        _obj(1, 1.0, 0.0, 2, label="fruit_photo_cube", fruit=""),
        _obj(2, 0.4, 0.0, 2, label="fruit_photo_cube", fruit="apple"),  # wrong fruit
        _obj(3, 0.3, 0.0, 1, label="octahedron"),                       # wrong shape
    ]
    kind, tgt = _select(_node(objects))
    assert (kind, int(tgt.id)) == ("set2_object", 1)


def test_known_wrong_fruit_hint_is_not_approached():
    objects = [
        _obj(1, 0.3, 0.0, 2, label="fruit_photo_cube", fruit="apple"),
    ]
    assert _select(_node(objects)) is None


def test_far_wrong_wide_hint_does_not_trigger_a_dedicated_cross_field_visit():
    objects = [
        _obj(1, 1.2, 0.0, 2, label="fruit_photo_cube", fruit="apple"),
    ]
    assert _select(_node(objects)) is None


def test_confirmed_banana_has_priority_over_nearer_set1_shape():
    objects = [
        _obj(1, 0.15, 0.0, 1, label="dodecahedron"),
        _obj(2, 1.20, 0.0, 2, label="fruit_photo_cube", fruit="banana"),
    ]

    kind, target = _select(_node(objects))

    assert (kind, int(target.id)) == ("set2_object", 2)


def test_untyped_fruit_cube_is_inspected_before_set1_shape():
    objects = [
        _obj(1, 0.15, 0.0, 1, label="dodecahedron"),
        _obj(2, 1.20, 0.0, 2, label="fruit_photo_cube", fruit=""),
    ]

    kind, target = _select(_node(objects))

    assert (kind, int(target.id)) == ("set2_object", 2)


def test_global_retarget_locks_once_current_goal_is_close():
    assert not global_retarget_is_worthwhile(0.79, 0.10, 0.20, 0.80)
    assert global_retarget_is_worthwhile(1.50, 0.80, 0.20, 0.80)
    assert not global_retarget_is_worthwhile(1.50, 1.30, 0.20, 0.80)


def test_wrong_wide_hint_stays_latched_during_global_approach():
    target = _obj(
        7, 0.5, 0.0, 2, label="fruit_photo_cube", fruit="orange"
    )
    node = _node([target])
    node.current_target = target
    node._opportunistic_set2_active = False
    node._set2_slot_mode = lambda: False
    node._lookup_object = lambda oid: target if oid == 7 else None
    node._nearest_phase_object = lambda ref_xy=None: None

    kept = MissionFsmNode._current_object_for_approach(node, ref_xy=(0.5, 0.0))

    assert kept is target


def test_legacy_approach_still_rejects_a_wrong_fruit_label():
    target = _obj(
        7, 0.5, 0.0, 2, label="fruit_photo_cube", fruit="orange"
    )
    node = _node([target])
    node.global_target_mode = False
    node.current_target = target
    node._opportunistic_set2_active = False
    node._set2_slot_mode = lambda: False
    node._lookup_object = lambda oid: target if oid == 7 else None
    node._nearest_phase_object = lambda ref_xy=None: None

    assert MissionFsmNode._current_object_for_approach(node, ref_xy=(0.5, 0.0)) is None


def test_global_set1_label_change_does_not_fallback_to_neighbour():
    target = _obj(9, -1.5, 0.0, 1, label="icosahedron")
    fresh_non_target = _obj(9, -1.5, 0.0, 1, label="dodecahedron")
    neighbour = _obj(11, -0.5, -0.5, 1, label="icosahedron")
    node = _node([fresh_non_target, neighbour])
    node.set1_label = "icosahedron"
    node.current_target = target
    node._opportunistic_set2_active = False
    node._set2_slot_mode = lambda: False
    node._lookup_object = lambda oid: fresh_non_target if oid == 9 else None
    node._nearest_phase_object = lambda ref_xy=None: neighbour

    kept = MissionFsmNode._current_object_for_approach(node, ref_xy=(-1.5, 0.0))

    assert kept is None
    assert node.current_target is fresh_non_target


def test_global_approach_keeps_frozen_goal_across_temporary_track_dropout():
    target = _obj(7, 0.5, 0.0, 2, label="fruit_photo_cube", fruit="orange")
    assert keep_latched_global_approach(
        global_target_mode=True,
        target_xy=(0.3, 0.0),
        current_target=target,
    )
    assert not keep_latched_global_approach(
        global_target_mode=False,
        target_xy=(0.3, 0.0),
        current_target=target,
    )
    target.blacklisted = True
    assert not keep_latched_global_approach(
        global_target_mode=True,
        target_xy=(0.3, 0.0),
        current_target=target,
    )


def test_global_target_absence_requires_both_timeout_and_fresh_body_frames():
    assert not global_target_absence_confirmed(
        elapsed_s=0.59,
        fresh_missing_body_frames=3,
        timeout_sec=0.60,
        confirm_frames=3,
    )
    assert not global_target_absence_confirmed(
        elapsed_s=0.60,
        fresh_missing_body_frames=2,
        timeout_sec=0.60,
        confirm_frames=3,
    )
    assert global_target_absence_confirmed(
        elapsed_s=0.60,
        fresh_missing_body_frames=3,
        timeout_sec=0.60,
        confirm_frames=3,
    )


def test_align_body_loss_uses_distinct_frames_and_elapsed_time():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._body_dets_seq = 10
    node.align_body_lost_confirm_sec = 0.40
    node.align_body_lost_confirm_frames = 3
    node._reset_align_body_loss_confirmation()

    assert not node._align_body_loss_confirmed(0.0)
    for seq, now in ((11, 0.10), (12, 0.20), (13, 0.39)):
        node._body_dets_seq = seq
        assert not node._align_body_loss_confirmed(now)
    assert node._align_body_loss_confirmed(0.40)


def test_spatial_defer_survives_track_id_churn_and_expires():
    replacement = _obj(99, 0.54, -0.48, 1, label="dodecahedron")
    deferred = [(1, 0.50, -0.50, 16.0)]
    assert target_is_spatially_deferred(replacement, deferred, 10.0, 0.18)
    assert not target_is_spatially_deferred(replacement, deferred, 16.0, 0.18)


def _global_presence_node(target, objects):
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.anchor_mission_enabled = False
    node._current_anchor_slot = lambda: None
    node._set2_slot_mode = lambda: False
    node.current_target = target
    node.world = SimpleNamespace(objects=objects)
    node._lookup_object = lambda _oid: None
    node.global_target_mode = True
    node._appr_tgt_xy = (float(target.x), float(target.y))
    node._is_locally_blacklisted = lambda _oid: False
    node.pick_track_conf = 0.5
    node.global_body_target_match_radius_m = 0.18
    return node


def test_global_target_presence_accepts_only_same_position_track_churn():
    stale = _obj(18, -1.5, 0.0, 2, label="fruit_photo_cube", fruit="")
    replacement = _obj(19, -1.42, 0.04, 2, label="fruit_photo_cube", fruit="")
    node = _global_presence_node(stale, [replacement])

    assert MissionFsmNode._align_target_still_in_world(node)
    assert node.current_target is replacement


def test_global_target_presence_rejects_neighbouring_apple_36cm_away():
    stale = _obj(18, -1.5, 0.0, 2, label="fruit_photo_cube", fruit="")
    apple = _obj(16, -1.5, -0.36, 2, label="fruit_photo_cube", fruit="apple")
    node = _global_presence_node(stale, [apple])

    assert not MissionFsmNode._align_target_still_in_world(node)
    assert node.current_target is stale


def test_absent_global_target_guard_reselects_after_short_confirmation():
    class _Logger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

    target = _obj(18, -1.5, 0.0, 2, label="fruit_photo_cube", fruit="")
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.global_target_mode = True
    node.current_target = target
    node._is_locally_blacklisted = lambda _oid: False
    node._current_anchor_slot = lambda: None
    node._set2_slot_mode = lambda: False
    node._align_target_still_in_world = lambda: False
    node._body_dets_seq = 10
    node.global_target_absent_timeout_sec = 0.60
    node.global_target_absent_confirm_frames = 3
    node._appr_tgt_xy = (-1.5, 0.0)
    node._set2_identity_latch = None
    node.set_type = 2
    node._opportunistic_set2_active = False
    commands = []
    blacklisted = []
    transitions = []
    node._drive = lambda vx, vy, omega=0.0: commands.append((vx, vy, omega))
    node._blacklist = lambda oid: blacklisted.append(int(oid))
    node._decide = lambda _text: None
    node._enter = lambda state: transitions.append(state)
    node.get_logger = lambda: _Logger()
    node._reset_global_target_absence()

    assert node._handle_absent_global_target(0.0, None, context="test")
    for seq, now in ((11, 0.20), (12, 0.40), (13, 0.59)):
        node._body_dets_seq = seq
        assert node._handle_absent_global_target(now, None, context="test")
    assert blacklisted == []
    assert transitions == []

    assert node._handle_absent_global_target(0.61, None, context="test")
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert blacklisted == [18]
    assert transitions == ["SELECT_TARGET"]
    assert node.current_target is None


def test_confirmed_banana_track_loss_preserves_position_for_align_reacquisition():
    class _Logger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

    target = _obj(
        90, 0.0, -0.5, 2, label="fruit_photo_cube", fruit="banana"
    )
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.global_target_mode = True
    node.current_target = target
    node.set2_label = "banana"
    node._appr_tgt_xy = (0.0, -0.5)
    node._is_locally_blacklisted = lambda _oid: False
    node._current_anchor_slot = lambda: None
    node._set2_slot_mode = lambda: False
    node._align_target_still_in_world = lambda: False
    node._body_dets_seq = 20
    node._set2_track_loss_align_override_logged = False
    node._global_approach_committed_from_wide = True
    decisions = []
    blacklisted = []
    transitions = []
    node._blacklist = lambda oid: blacklisted.append(int(oid))
    node._decide = lambda text: decisions.append(text)
    node._enter = lambda state: transitions.append(state)
    node.get_logger = lambda: _Logger()
    node._reset_global_target_absence()

    handled = node._handle_absent_global_target(
        1.0, None, context="APPROACH stand-off"
    )

    assert not handled
    assert node.current_target is target
    assert node._appr_tgt_xy == (0.0, -0.5)
    assert blacklisted == []
    assert transitions == []
    assert decisions == [
        "WIDE TARGET COMMITTED #90 -> KEEP POSITION / BODY VERIFY"
    ]


def test_confirmed_set1_track_loss_also_preserves_position_for_body_visit():
    target = _obj(91, 0.4, -0.5, 1, label="dodecahedron")
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.global_target_mode = True
    node.current_target = target
    node._appr_tgt_xy = (0.4, -0.5)
    node._global_approach_committed_from_wide = True

    assert node._committed_global_target_can_try_align()


def test_global_approach_route_blocked_defers_target_and_resumes_scan():
    class _Logger:
        def info(self, _message):
            pass

    target = _obj(14, -1.5, -0.5, 2, label="fruit_photo_cube", fruit="apple")
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.global_target_mode = True
    node.global_approach_route_blocked_defer_enabled = True
    node.global_approach_route_blocked_confirm_sec = 0.0
    node.global_approach_route_blocked_confirm_attempts = 1
    node.align_body_lost_reselect_cooldown_sec = 6.0
    node.state = "APPROACH"
    node.current_target = target
    node._appr_tgt_xy = (-1.5, -0.5)
    node._global_deferred_targets = []
    node._last_drive_route_blocked = True
    node._approach_route_blocked_since_s = None
    node._approach_route_blocked_attempts = 0
    node._standoff_arrived_s = 1.0
    node._opportunistic_set2_active = True
    node._plan = []
    node._plan_dest = (-1.5, -0.5)
    node._now_s = lambda: 10.0
    commands = []
    decisions = []
    transitions = []
    node._drive = lambda vx, vy, wz: commands.append((vx, vy, wz))
    node._decide = lambda text: decisions.append(text)
    node._enter = lambda state: transitions.append(state)
    node.get_logger = lambda: _Logger()

    assert MissionFsmNode._handle_global_approach_route_blocked(node, 10.0)

    assert node._global_deferred_targets == [(2, -1.5, -0.5, 16.0)]
    assert node.current_target is None
    assert node._appr_tgt_xy is None
    assert node._opportunistic_set2_active is False
    assert commands[-1] == (0.0, 0.0, 0.0)
    assert transitions == ["SCAN"]
    assert decisions[-1].endswith("-> DEFER / RESUME LANE")


def test_set1_fresh_world_non_target_keeps_visit_for_final_classify():
    class _Logger:
        def info(self, _message):
            pass

    target = _obj(9, -1.5, 0.0, 1, label="icosahedron")
    fresh_non_target = _obj(9, -1.5, 0.0, 1, label="dodecahedron", conf=0.94)
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.global_target_mode = True
    node.phase = 1
    node.world = SimpleNamespace(objects=[fresh_non_target])
    node.current_target = target
    node._appr_tgt_xy = (-1.5, 0.0)
    node.set1_label = "icosahedron"
    node.pick_track_conf = 0.5
    node.align_body_lost_reselect_cooldown_sec = 6.0
    node._global_deferred_targets = []
    node._standoff_arrived_s = None
    node.set_type = 1
    node._opportunistic_set2_active = False
    node._lookup_object = lambda oid: fresh_non_target if oid == 9 else None
    node._now_s = lambda: 10.0
    commands = []
    decisions = []
    blacklisted = []
    transitions = []
    node._drive = lambda vx, vy, wz: commands.append((vx, vy, wz))
    node._decide = lambda text: decisions.append(text)
    node._blacklist = lambda oid: blacklisted.append(int(oid))
    node._reset_global_target_absence = lambda: None
    node._enter = lambda state: transitions.append(state)
    node.get_logger = lambda: _Logger()

    handled = MissionFsmNode._reject_latched_set1_non_target(node)

    assert not handled
    assert node.current_target is target
    assert node._appr_tgt_xy == (-1.5, 0.0)
    assert node._global_deferred_targets == []
    assert blacklisted == []
    assert commands == []
    assert transitions == []
    assert decisions == []


def test_set1_fresh_body_non_target_keeps_visit_for_final_classify():
    class _Logger:
        def info(self, _message):
            pass

    target = _obj(16, -0.2, -1.25, 1, label="icosahedron")
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.global_target_mode = True
    node.phase = 1
    node.current_target = target
    node._appr_tgt_xy = (-0.2, -1.25)
    node.set1_label = "icosahedron"
    node.align_body_lost_reselect_cooldown_sec = 6.0
    node._global_deferred_targets = []
    node._standoff_arrived_s = None
    node.set_type = 1
    node._opportunistic_set2_active = False
    node._fresh_world_set1_non_target = lambda: None
    node._fresh_body_set1_non_target_at_latched_position = (
        lambda: ("dodecahedron", 0.21, 0.01, 0.90)
    )
    node._now_s = lambda: 20.0
    commands = []
    blacklisted = []
    transitions = []
    node._drive = lambda vx, vy, wz: commands.append((vx, vy, wz))
    node._decide = lambda text: None
    node._blacklist = lambda oid: blacklisted.append(int(oid))
    node._reset_global_target_absence = lambda: None
    node._enter = lambda state: transitions.append(state)
    node.get_logger = lambda: _Logger()

    handled = MissionFsmNode._reject_latched_set1_non_target(node)

    assert not handled
    assert node.current_target is target
    assert node._global_deferred_targets == []
    assert blacklisted == []
    assert commands == []
    assert transitions == []


def test_untyped_exploration_target_is_not_a_wide_commitment():
    target = _obj(92, 0.4, -0.5, 2, label="fruit_photo_cube", fruit="")
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.global_target_mode = True
    node.current_target = target
    node._appr_tgt_xy = (0.4, -0.5)
    node._global_approach_committed_from_wide = False

    assert not node._committed_global_target_can_try_align()


def test_blacklisted_and_low_conf_are_never_selected():
    objects = [
        _obj(2, 0.4, 0.0, 1, label="dodecahedron", blacklisted=True),
        _obj(3, 0.5, 0.0, 1, label="dodecahedron", conf=0.2),
    ]
    assert _select(_node(objects)) is None


def test_quota_exhaustion_drops_that_set_from_the_tour():
    objects = [
        _obj(1, 0.3, 0.0, 1, label="dodecahedron"),
        _obj(2, 2.0, 0.0, 2, label="fruit_photo_cube", fruit="banana"),
    ]
    kind, tgt = _select(_node(objects, tray_shape=4))
    assert (kind, int(tgt.id)) == ("set2_object", 2)
    kind, tgt = _select(_node(objects, tray_fruit=3))
    assert (kind, int(tgt.id)) == ("set1", 1)


def test_opening_or_mapping_disabled_cannot_consume_unseen_coverage():
    node = SimpleNamespace(
        global_target_mode=True,
        world=SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0),
        _world_mapping_enabled=False,
        state="OPENING",
        _wide_relative_seq=1,
        _global_seen_wide_seq=-1,
        _wide_relative_last_frame_s=10.0,
        _world_last_rx_s=10.0,
        global_sensor_max_age_sec=0.75,
        _global_seen_nodes=set(),
        _global_patrol_nodes=[(0.5, 0.0)],
        global_wide_fov_forward_m=1.1,
        global_wide_fov_lateral_m=1.3,
        global_wide_fov_center_x_m=0.4,
        global_fov_min_range_m=0.15,
        _now_s=lambda: 10.1,
    )
    MissionFsmNode._update_global_seen_nodes(node)
    assert node._global_seen_nodes == set()


def test_fresh_wide_frame_marks_coverage_once_after_mapping_is_ready():
    node = SimpleNamespace(
        global_target_mode=True,
        world=SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0),
        _world_mapping_enabled=True,
        state="SCAN",
        _wide_relative_seq=1,
        _global_seen_wide_seq=-1,
        _wide_relative_last_frame_s=10.0,
        _world_last_rx_s=10.0,
        global_sensor_max_age_sec=0.75,
        _global_seen_nodes=set(),
        _global_patrol_nodes=[(0.5, 0.0), (-1.5, 0.0)],
        global_wide_fov_forward_m=1.1,
        global_wide_fov_lateral_m=1.3,
        global_wide_fov_center_x_m=0.4,
        global_fov_min_range_m=0.15,
        _now_s=lambda: 10.1,
    )
    MissionFsmNode._update_global_seen_nodes(node)
    assert node._global_seen_nodes == {0}
    assert node._global_seen_wide_seq == 1

    node.world.robot_theta = 3.14159
    MissionFsmNode._update_global_seen_nodes(node)
    assert node._global_seen_nodes == {0}  # same camera frame cannot be counted twice
