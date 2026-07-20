import math

from robot_planning.local_anchor_fsm import (
    Candidate,
    LocalAnchorConfig,
    LocalAnchorFruitFsm,
    LocalObservation,
    efficient_candidate_order,
    integrate_imu_yaw,
)


def candidate(candidate_id, bearing_deg, radius=0.30, label="fruit_photo_cube"):
    angle = math.radians(bearing_deg)
    return Candidate(
        candidate_id=candidate_id,
        x=radius * math.cos(angle),
        y=radius * math.sin(angle),
        label=label,
        confidence=0.9,
        hits=3,
    )


def test_candidate_order_does_not_zigzag_across_largest_gap():
    items = [candidate(1, -40), candidate(2, -5), candidate(3, 30), candidate(4, 70)]
    route = efficient_candidate_order(items, current_yaw=math.radians(-45))
    assert route == [1, 2, 3, 4]


def test_physical_imu_yaw_delta_is_accumulated_without_half_scale():
    yaw = 0.0
    yaw = integrate_imu_yaw(yaw, math.radians(45.0))
    yaw = integrate_imu_yaw(yaw, math.radians(45.0))
    assert math.degrees(yaw) == 90.0


def test_clockwise_scan_requests_negative_yaw_pulse():
    cfg = LocalAnchorConfig(
        single_lap_inspection=False,
        scan_positions=2,
        scan_step_rad=math.radians(45),
    )
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.start(0.0)
    fsm.tick(0.0, 0.0)
    command = fsm.tick(0.01, 0.0)
    assert command.omega < 0.0


def test_eight_scan_targets_cover_exactly_one_clockwise_lap():
    cfg = LocalAnchorConfig(
        single_lap_inspection=False,
        scan_positions=8,
        scan_step_rad=math.radians(45),
    )
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.start(0.0)
    expected = [-45, -90, -135, -180, 135, 90, 45, 0]
    actual = [round(math.degrees(target)) for target in fsm.scan_targets]
    assert actual == expected


def test_inventory_merges_repeat_observations_and_drops_singletons():
    cfg = LocalAnchorConfig(
        single_lap_inspection=False,
        scan_positions=1,
        candidate_min_hits=2,
    )
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.start(0.0)
    fsm.tick(0.0, math.radians(-45))
    assert fsm.state == "SCAN_OBSERVE"
    repeated = LocalObservation(0.25, 0.05, "fruit_photo_cube", 0.9)
    singleton = LocalObservation(0.20, -0.20, "fruit_photo_cube", 0.8)
    fsm.add_observations([repeated, singleton])
    fsm.add_observations([LocalObservation(0.255, 0.052, "fruit_photo_cube", 0.85)])
    fsm.tick(cfg.scan_observe_sec + 0.01, math.radians(-45))
    assert len(fsm.candidates) == 1
    assert fsm.candidates[0].hits == 2


def test_three_stable_classifications_mark_target_and_complete():
    cfg = LocalAnchorConfig(
        single_lap_inspection=False,
        classify_stable_frames=3,
        target_fruit_label="banana",
    )
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.candidates = [candidate(1, 0)]
    fsm.route = [1]
    fsm.route_index = 0
    fsm._enter("CLASSIFY", 1.0, "test")
    fsm.note_classification("banana", 0.2, True, 1.1)
    fsm.note_classification("banana", 0.2, True, 1.2)
    assert fsm.state == "CLASSIFY"
    fsm.note_classification("banana", 0.2, True, 1.3)
    assert fsm.candidates[0].status == "TARGET_FRUIT"
    assert fsm.state == "COMPLETE"


def test_non_fruit_is_kept_on_map_but_not_added_to_inspection_route():
    cfg = LocalAnchorConfig(inventory_max_candidates=4)
    fsm = LocalAnchorFruitFsm(cfg)
    fruit = candidate(1, -20)
    shape = candidate(2, 80, label="dodecahedron")
    fsm.candidates = [fruit, shape]
    fsm._freeze_inventory(1.0, 0.0)
    assert len(fsm.candidates) == 2
    assert len(fsm.route) == 1
    ignored = next(item for item in fsm.candidates if item.label == "dodecahedron")
    assert ignored.status == "IGNORED_NON_FRUIT"


def test_unresolved_siglip_holds_position_instead_of_advancing():
    cfg = LocalAnchorConfig(
        classify_timeout_sec=2.0,
        classify_hold_on_failure=True,
    )
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.candidates = [candidate(1, 0), candidate(2, 90)]
    fsm.route = [1, 2]
    fsm.route_index = 0
    fsm._enter("CLASSIFY", 1.0, "test")
    command = fsm.tick(3.1, 0.0)
    assert command.vx == command.vy == command.omega == 0.0
    assert fsm.state == "CLASSIFY"
    assert fsm.route_index == 0
    assert fsm.candidates[0].status == "WAITING_CLASSIFICATION"


def test_inventory_is_capped_to_four_strongest_clusters():
    cfg = LocalAnchorConfig(inventory_max_candidates=4, candidate_merge_radius_m=0.05)
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.candidates = [
        candidate(idx, bearing, radius=0.25 + 0.01 * idx)
        for idx, bearing in enumerate([-150, -90, -30, 30, 90, 150], start=1)
    ]
    fsm._freeze_inventory(1.0, 0.0)
    assert len(fsm.candidates) == 4


def test_post_scan_consolidation_merges_duplicate_views_of_four_objects():
    cfg = LocalAnchorConfig(inventory_max_candidates=4)
    fsm = LocalAnchorFruitFsm(cfg)
    objects = []
    candidate_id = 1
    for bearing in (-135, -45, 45, 135):
        primary = candidate(candidate_id, bearing, radius=0.30)
        primary.hits = 20
        duplicate = candidate(candidate_id + 1, bearing + 7, radius=0.32)
        duplicate.hits = 8
        objects.extend([primary, duplicate])
        candidate_id += 2
    consolidated = fsm._consolidate_candidates(objects)
    assert len(consolidated) == 4
    assert all(item.hits == 28 for item in consolidated)


def test_cube_plus_confident_fruit_view_is_forced_to_fruit_candidate():
    cfg = LocalAnchorConfig(
        initial_inventory_observe_sec=2.0,
        fruit_cube_override_confidence=0.60,
    )
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.start(0.0)
    observations = [
        LocalObservation(0.25, 0.02, "cube", 0.98),
        LocalObservation(0.252, 0.021, "fruit_photo_cube", 0.61),
    ]
    fsm.add_observations(observations)
    assert len(fsm.candidates) == 1
    assert fsm.candidates[0].label == "fruit_photo_cube"
    fsm.tick(2.01, 0.0)
    assert fsm.route == [1]
    assert fsm.candidates[0].status == "FACING"


def test_align_uses_one_axis_pulse_after_all_classification():
    cfg = LocalAnchorConfig(single_lap_inspection=False, enable_align=True)
    fsm = LocalAnchorFruitFsm(cfg)
    target = candidate(1, 0)
    target.status = "TARGET_FRUIT"
    fsm.candidates = [target]
    fsm._align_target_id = 1
    fsm._align_start_s = 0.0
    fsm._enter("ALIGN_MEASURE", 0.0, "test")
    fsm.note_align_target(0.30, 0.01, 0.1)
    fsm.tick(0.1, 0.0)
    command = fsm.tick(0.11, 0.0)
    assert command.vx > 0.0
    assert command.vy == 0.0


def test_single_lap_starts_with_stationary_inventory_window():
    cfg = LocalAnchorConfig(initial_inventory_observe_sec=2.0)
    fsm = LocalAnchorFruitFsm(cfg)
    fsm.start(0.0)
    assert fsm.state == "INITIAL_INVENTORY"
    command = fsm.tick(1.9, 0.0)
    assert command.vx == command.vy == command.omega == 0.0


def test_single_lap_orders_fruit_only_clockwise_from_start():
    fsm = LocalAnchorFruitFsm(LocalAnchorConfig(inventory_max_candidates=4))
    fsm.candidates = [
        candidate(1, 20),
        candidate(2, -150),
        candidate(3, 120),
        candidate(4, -30),
    ]
    fsm._freeze_inventory(2.0, 0.0)
    route_bearings = [
        round(math.degrees(fsm._candidate_by_id(candidate_id).bearing))
        for candidate_id in fsm.route
    ]
    assert route_bearings == [-30, -150, 120, 20]


def test_positive_bearing_is_reached_late_without_counterclockwise_shortcut():
    fsm = LocalAnchorFruitFsm(LocalAnchorConfig())
    fsm.candidates = [candidate(1, 90)]
    fsm._freeze_inventory(2.0, 0.0)
    assert fsm.state == "TURN_MEASURE"
    assert fsm.summary()["turn_target_deg"] == -270.0
    command = fsm.tick(2.1, 0.0)
    assert command.omega == 0.0
    command = fsm.tick(2.11, 0.0)
    assert command.omega < 0.0


def test_single_lap_finishes_at_negative_360_degrees():
    fsm = LocalAnchorFruitFsm(LocalAnchorConfig())
    target = candidate(1, -90)
    target.status = "TARGET_FRUIT"
    fsm.candidates = [target]
    fsm._finish_inspection(4.0)
    assert fsm.state == "TURN_MEASURE"
    assert fsm.summary()["turn_target_deg"] == -360.0
    fsm._unwrapped_yaw = -2.0 * math.pi
    fsm.tick(4.1, 0.0)
    assert fsm.state == "LAP_COMPLETE"
    fsm.tick(4.2, 0.0)
    assert fsm.state == "COMPLETE"


def test_single_lap_unwraps_clockwise_motion_across_minus_pi():
    fsm = LocalAnchorFruitFsm(LocalAnchorConfig(initial_inventory_observe_sec=10.0))
    fsm.start(0.0)
    fsm.tick(1.0, math.radians(-170))
    fsm.tick(2.0, math.radians(170))
    assert fsm.summary()["scan_completed_deg"] == 190.0
