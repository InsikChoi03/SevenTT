import math
from types import SimpleNamespace

import pytest

from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    StableParkingWallTracker,
    arm_pick_lift_started,
    clamp_outward_field_velocity,
    directional_wall_distance,
    select_flag_snapshot_target,
    straight_forward_command,
    straight_reverse_command,
    timed_storage_due,
)


def test_field_boundary_guard_blocks_only_outward_translation():
    bounds = (-1.78, 1.78, -1.78, 1.78)
    # Heading zero: +vx is field +x. At the right edge it is removed.
    assert clamp_outward_field_velocity(
        0.2, 0.0, 0.0, 1.76, 0.0, bounds, 0.03
    ) == pytest.approx((0.0, 0.0))
    # Inward travel remains available, including recovery from slightly outside the edge.
    assert clamp_outward_field_velocity(
        -0.2, 0.0, 0.0, 1.85, 0.0, bounds, 0.03
    ) == pytest.approx((-0.2, 0.0))
    # A tangential component is preserved while the outward component is removed.
    assert clamp_outward_field_velocity(
        0.2, 0.1, 0.0, 1.76, 0.0, bounds, 0.03
    ) == pytest.approx((0.0, 0.1))


def test_storage_deadline_is_relative_to_running_start():
    common = dict(
        run_started=True,
        triggered=False,
        completed=False,
        run_start_s=20.0,
        trigger_sec=160.0,
    )

    assert not timed_storage_due(now_s=179.99, **common)
    assert timed_storage_due(now_s=180.0, **common)


def test_dynamic_storage_deadline_uses_button_relative_remaining_time():
    common = dict(
        run_started=True,
        triggered=False,
        completed=False,
        run_start_s=20.0,
        trigger_sec=135.0,
        dynamic_enabled=True,
        match_duration_sec=180.0,
        route_distance_m=3.0,
        effective_speed_mps=0.10,
        fixed_overhead_sec=12.0,
        safety_margin_sec=10.0,
        distance_safety_factor=1.20,
    )
    # Required reserve is 3.0*1.2/0.1 + 12 + 10 = 58 seconds, hence t=122.
    assert not timed_storage_due(now_s=20.0 + 121.99, **common)
    assert timed_storage_due(now_s=20.0 + 122.0, **common)


def test_fixed_latest_storage_trigger_survives_missing_route_estimate():
    common = dict(
        run_started=True,
        triggered=False,
        completed=False,
        run_start_s=30.0,
        trigger_sec=135.0,
        dynamic_enabled=True,
        match_duration_sec=180.0,
        route_distance_m=None,
    )
    assert not timed_storage_due(now_s=30.0 + 134.99, **common)
    assert timed_storage_due(now_s=30.0 + 135.0, **common)


def test_arm_lift_releases_base_only_for_fresh_current_pick_phase():
    common = dict(phase_rx_s=12.0, pick_enter_s=10.0, now_s=12.1, max_age_sec=0.5)

    assert not arm_pick_lift_started("GRASP", **common)
    assert arm_pick_lift_started("LIFT", **common)
    assert arm_pick_lift_started("TO_PLACE", **common)
    assert not arm_pick_lift_started(
        "LIFT", phase_rx_s=9.0, pick_enter_s=10.0, now_s=10.1, max_age_sec=0.5
    )
    assert not arm_pick_lift_started(
        "LIFT", phase_rx_s=12.0, pick_enter_s=10.0, now_s=13.0, max_age_sec=0.5
    )


def test_directional_wall_distance_selects_requested_axis_and_direction():
    segments = [
        (-1.0, 0.30, 1.0, 0.30),
        (0.20, -1.0, 0.20, 1.0),
        (-0.40, -1.0, -0.40, 1.0),
    ]

    assert directional_wall_distance(
        segments, 0.0, 0.0, axis="y", direction=1
    ) == pytest.approx(0.30)
    assert directional_wall_distance(
        segments, 0.0, 0.0, axis="x", direction=1
    ) == pytest.approx(0.20)
    assert directional_wall_distance(
        segments, 0.0, 0.0, axis="x", direction=-1
    ) == pytest.approx(0.40)


def _method4_tracker():
    return StableParkingWallTracker(
        window_frames=5,
        min_hits=4,
        hold_sec=1.70,
        fresh_sec=0.50,
        min_length_m=0.35,
        axis_tolerance_rad=math.radians(12.0),
        boundary_residual_m=0.35,
        coordinate_spread_m=0.10,
    )


def test_method4_wall_tracker_requires_four_consistent_strong_frames():
    tracker = _method4_tracker()
    wall = (-1.0, 1.92, 1.0, 1.92)

    for frame in range(3):
        tracker.observe(
            [wall],
            axis="y",
            direction=1,
            robot_x=0.0,
            robot_y=1.0,
            field_bounds=(-2.0, 2.0, -2.0, 2.0),
            now_s=frame / 3.0,
        )
        assert not tracker.confirmed(frame / 3.0)

    tracker.observe(
        [wall],
        axis="y",
        direction=1,
        robot_x=0.0,
        robot_y=1.0,
        field_bounds=(-2.0, 2.0, -2.0, 2.0),
        now_s=1.0,
    )
    assert tracker.confirmed(1.0)
    assert tracker.fresh(1.0)
    assert tracker.distance(0.0, 1.0, 1.0) == pytest.approx(0.92)


def test_method4_wall_tracker_rejects_floor_lines_and_coasts_without_fresh_fix():
    tracker = _method4_tracker()
    short_floor_mark = (-0.1, 1.90, 0.1, 1.90)
    off_boundary_floor_line = (-1.0, 1.20, 1.0, 1.20)
    for frame in range(5):
        tracker.observe(
            [short_floor_mark, off_boundary_floor_line],
            axis="y",
            direction=1,
            robot_x=0.0,
            robot_y=1.0,
            field_bounds=(-2.0, 2.0, -2.0, 2.0),
            now_s=frame / 3.0,
        )
    assert not tracker.confirmed(4.0 / 3.0)

    wall = (-1.0, 1.92, 1.0, 1.92)
    for frame in range(4):
        tracker.observe(
            [wall],
            axis="y",
            direction=1,
            robot_x=0.0,
            robot_y=1.0,
            field_bounds=(-2.0, 2.0, -2.0, 2.0),
            now_s=2.0 + frame / 3.0,
        )
    tracker.observe(
        [],
        axis="y",
        direction=1,
        robot_x=0.0,
        robot_y=1.0,
        field_bounds=(-2.0, 2.0, -2.0, 2.0),
        now_s=3.8,
    )
    assert tracker.confirmed(3.8)
    assert not tracker.fresh(3.8)
    assert tracker.distance(0.0, 1.1, 3.8) == pytest.approx(0.82)
    assert not tracker.confirmed(4.8)


def test_storage_commands_have_no_lateral_translation():
    forward = straight_forward_command(
        current_heading=math.pi / 2.0,
        target_heading=math.pi / 2.0,
        distance_m=0.5,
        max_speed=0.08,
        heading_kp=1.0,
        omega_max=0.06,
    )
    reverse = straight_reverse_command(
        current_heading=math.pi,
        target_heading=math.pi,
        distance_m=0.5,
        max_speed=0.08,
        heading_kp=1.0,
        omega_max=0.06,
    )

    assert forward == pytest.approx((0.08, 0.0, 0.0))
    assert reverse == pytest.approx((-0.08, 0.0, 0.0))


def test_wall_processing_runs_only_during_initialization_and_storage():
    node = SimpleNamespace(
        storage_wall_guided_enabled=True,
        wall_processing_during_mission_enabled=False,
        state="WALL_INIT",
    )

    assert MissionFsmNode._wall_processing_requested(node)
    node.state = "SCAN"
    assert not MissionFsmNode._wall_processing_requested(node)
    node.state = "DRIVE_TO_STORAGE"
    assert MissionFsmNode._wall_processing_requested(node)


def test_explicit_wall_diagnostics_remain_enabled_without_wall_storage_mode():
    node = SimpleNamespace(
        storage_wall_guided_enabled=False,
        wall_processing_during_mission_enabled=False,
        state="SCAN",
    )

    assert MissionFsmNode._wall_processing_requested(node)


def test_wall_processing_can_run_during_all_mission_states():
    node = SimpleNamespace(
        storage_wall_guided_enabled=True,
        wall_processing_during_mission_enabled=True,
        state="OPENING",
    )

    for state in ("OPENING", "SCAN", "APPROACH", "ALIGN", "PICK"):
        node.state = state
        assert MissionFsmNode._wall_processing_requested(node)


class _Logger:
    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _StorageStub:
    storage_parking_method = 3
    storage_wall_guided_enabled = True
    storage_down_heading_rad = math.pi / 2.0
    storage_right_heading_rad = math.pi
    storage_bottom_wall_stop_m = 0.30
    storage_left_wall_stop_m = 0.20
    storage_wall_approach_speed = 0.08
    storage_reverse_speed = 0.08
    storage_heading_tolerance_rad = 0.06
    storage_reverse_heading_kp = 1.0
    storage_reverse_omega_max = 0.06
    storage_reverse_realign_rad = 0.1745
    storage_wall_confirm_frames = 3
    storage_wall_max_age_sec = 1.50
    storage_wall_detection_required_m = 0.60
    storage_pose_fallback_enabled = False
    storage_pose_fallback_hold_sec = 0.8
    storage_pose_fallback_contact_speed = 0.06
    storage_last_chance_enabled = False
    storage_last_chance_trigger_sec = 15.0
    storage_last_chance_pre_backoff_m = 0.05
    storage_last_chance_pre_backoff_speed = 0.10
    storage_last_chance_turn_left_deg = 95.0
    storage_last_chance_reverse_sec = 5.0
    storage_last_chance_alternate_sec = 2.0
    storage_last_chance_force_dump_after_sec = 12.0
    storage_last_chance_speed = 0.20
    storage_last_chance_arrival_guidance_enabled = False
    storage_last_chance_arrival_min_conf = 0.30
    storage_last_chance_arrival_max_age_sec = 0.75
    storage_contact_hold_enabled = True
    storage_contact_hold_speed = 0.08
    storage_wall_warmup_sec = 0.8
    storage_wall_warmup_timeout_sec = 2.0

    def __init__(self):
        self.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=math.pi / 2.0)
        self._field_bounds = (-2.0, 2.0, -2.0, 2.0)
        self._storage_route_phase = "face_bottom_wall"
        self._storage_route_completed = False
        self._storage_wall_confirm_count = 0
        self._storage_wall_confirm_last_seq = -1
        self._storage_pose_fallback_key = None
        self._storage_pose_fallback_start_s = 0.0
        self._storage_last_chance_active = False
        self._storage_last_chance_phase = "idle"
        self._storage_last_chance_started_s = 0.0
        self._storage_last_chance_phase_started_s = 0.0
        self._storage_last_chance_turn_heading = None
        self._storage_last_chance_guidance_mode = "pattern"
        self._wall_segments = []
        self._wall_segments_s = 10.0
        self._wall_segments_seq = 0
        self._storage_wall_warmup_last_seq = 0
        self._storage_wall_warmup_valid_frames = 0
        self._now = 10.0
        self.state_enter_s = 10.0
        self.commands = []
        self.decisions = []
        self.state = "DRIVE_TO_STORAGE"
        self._run_started = True
        self._node_start_s = 0.0

    def set_segments(self, *segments):
        self._wall_segments = list(segments)
        self._wall_segments_s = self._now
        self._wall_segments_seq += 1

    def _now_s(self):
        return self._now

    def _time_in_state(self):
        return self._now - self.state_enter_s

    def _drive(self, vx, vy, omega=0.0, **_kwargs):
        self.commands.append((vx, vy, omega))

    def _reset_pulsed_heading(self):
        pass

    def _reset_storage_wall_confirmation(self):
        MissionFsmNode._reset_storage_wall_confirmation(self)

    def _storage_wall_distance(self, **kwargs):
        return MissionFsmNode._storage_wall_distance(self, **kwargs)

    def _storage_wall_stop_confirmed(self, *args):
        return MissionFsmNode._storage_wall_stop_confirmed(self, *args)

    def _storage_pose_wall_distance(self, **kwargs):
        return MissionFsmNode._storage_pose_wall_distance(self, **kwargs)

    def _storage_contact_command(self, **kwargs):
        return MissionFsmNode._storage_contact_command(self, **kwargs)

    def _drive_storage_contact_hold(self):
        MissionFsmNode._drive_storage_contact_hold(self)

    def _decide(self, message):
        self.decisions.append(message)

    def _turn_in_place_pulsed(self, target, *_args, **_kwargs):
        self.world.robot_theta = target
        self._drive(0.0, 0.0, 0.0)
        return True

    @staticmethod
    def _wrap_pi(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _finish_storage_reverse(self, reason):
        MissionFsmNode._finish_storage_reverse(self, reason)

    def _enter(self, state):
        self.state = state

    def _mission_elapsed_s(self):
        return MissionFsmNode._mission_elapsed_s(self)

    def _fresh_storage_wall_available(self):
        return MissionFsmNode._fresh_storage_wall_available(self)

    def _maybe_start_storage_last_chance(self):
        return MissionFsmNode._maybe_start_storage_last_chance(self)

    def _step_storage_last_chance(self):
        MissionFsmNode._step_storage_last_chance(self)

    def get_logger(self):
        return _Logger()


def test_storage_route_uses_bottom_then_left_wall_with_three_frame_confirmation():
    node = _StorageStub()

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "approach_bottom_wall"

    node.set_segments((-1.0, 0.80, 1.0, 0.80))
    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1][0] > 0.0
    assert node.commands[-1][1] == 0.0

    for expected_count in range(1, 4):
        node.set_segments((-1.0, 0.25, 1.0, 0.25))
        MissionFsmNode._step_drive_to_storage(node)
        assert node.commands[-1] == (0.0, 0.0, 0.0)
        if expected_count < 3:
            assert node._storage_route_phase == "approach_bottom_wall"
    assert node._storage_route_phase == "face_right"

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "reverse_to_left_wall"

    node.set_segments((0.80, -1.0, 0.80, 1.0))
    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1][0] < 0.0
    assert node.commands[-1][1] == 0.0

    for expected_count in range(1, 4):
        node.set_segments((0.19, -1.0, 0.19, 1.0))
        MissionFsmNode._step_drive_to_storage(node)
        assert node.commands[-1] == (0.0, 0.0, 0.0)
        if expected_count < 3:
            assert node.state == "DRIVE_TO_STORAGE"

    assert node._storage_route_completed is True
    assert node.state == "ALIGN_OVER_BIN"


class _Method4StorageStub(_StorageStub):
    storage_parking_method = 4

    def __init__(self):
        super().__init__()
        self.storage_parking_method = 4
        self._storage_method4_wall_tracker = _method4_tracker()
        self._storage_method4_wall_last_seq = -1


def test_method4_route_waits_for_stable_wall_track_before_distance_stop():
    node = _Method4StorageStub()
    node._storage_route_phase = "approach_bottom_wall"
    node.world.robot_y = 1.65
    wall = (-1.0, 1.92, 1.0, 1.92)

    for frame in range(3):
        node.set_segments(wall)
        MissionFsmNode._step_drive_to_storage(node)
        assert node._storage_route_phase == "approach_bottom_wall"
        assert node.commands[-1] == (0.0, 0.0, 0.0)
        node._now += 1.0 / 3.0

    # The fourth matching frame confirms the wall track; the existing three-frame stop gate
    # still has to complete before the route is allowed to turn.
    for expected_stop_count in range(1, 4):
        node.set_segments(wall)
        MissionFsmNode._step_drive_to_storage(node)
        if expected_stop_count < 3:
            assert node._storage_route_phase == "approach_bottom_wall"
        node._now += 1.0 / 3.0

    assert node._storage_route_phase == "face_right"


def test_method4_enables_wall_pose_correction_only_for_fresh_confirmed_track():
    node = _Method4StorageStub()
    node._wall_translation_unlocked = True
    wall = (-1.0, 1.92, 1.0, 1.92)

    assert MissionFsmNode._wall_correction_mode_requested(node) == "OFF"
    for _ in range(4):
        node.set_segments(wall)
        MissionFsmNode._storage_wall_distance(node, axis="y", direction=1)
        node._now += 1.0 / 3.0

    assert MissionFsmNode._wall_correction_mode_requested(node) == "TRANSLATION_ONLY"
    node._now += node._storage_method4_wall_tracker.fresh_sec + 0.01
    assert MissionFsmNode._wall_correction_mode_requested(node) == "OFF"


def test_storage_wall_route_starts_with_bottom_wall_alignment_without_warmup():
    node = _StorageStub()
    node.storage_wall_guided_enabled = False
    node._storage_route_phase = "face_bottom_wall"

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "approach_bottom_wall"
    assert node.commands[-1] == (0.0, 0.0, 0.0)


def test_storage_holds_when_near_wall_detection_is_temporarily_missing():
    node = _StorageStub()
    node._storage_route_phase = "approach_bottom_wall"
    node.world.robot_y = 1.45
    node._wall_segments_s = 0.0

    MissionFsmNode._step_drive_to_storage(node)

    assert node.commands[-1] == (0.0, 0.0, 0.0)
    assert node._storage_route_phase == "approach_bottom_wall"


def test_storage_pose_fallback_advances_without_fresh_bottom_wall_detection():
    node = _StorageStub()
    node.storage_pose_fallback_enabled = True
    node._storage_route_phase = "approach_bottom_wall"
    node.world.robot_y = 1.45
    node._wall_segments_s = 0.0

    MissionFsmNode._step_drive_to_storage(node)

    assert node.commands[-1][0] > 0.0
    assert node._storage_route_phase == "approach_bottom_wall"
    assert any("STORAGE POSE FALLBACK bottom wall" in d for d in node.decisions)

    node._now += node.storage_pose_fallback_hold_sec
    MissionFsmNode._step_drive_to_storage(node)

    assert node.commands[-1] == (0.0, 0.0, 0.0)
    assert node._storage_route_phase == "face_right"
    assert any("STORAGE BOTTOM WALL POSE FALLBACK" in d for d in node.decisions)


def test_storage_pose_fallback_finishes_reverse_without_fresh_left_wall_detection():
    node = _StorageStub()
    node.storage_pose_fallback_enabled = True
    node._storage_route_phase = "reverse_to_left_wall"
    node.world.robot_x = 1.55
    node.world.robot_theta = math.pi
    node._wall_segments_s = 0.0

    MissionFsmNode._step_drive_to_storage(node)

    assert node.commands[-1][0] < 0.0
    assert node.state == "DRIVE_TO_STORAGE"
    assert any("STORAGE POSE FALLBACK left wall" in d for d in node.decisions)

    node._now += node.storage_pose_fallback_hold_sec
    MissionFsmNode._step_drive_to_storage(node)

    assert node._storage_route_completed is True
    assert node.state == "ALIGN_OVER_BIN"


def test_storage_last_chance_runs_desperation_pattern_and_forces_dump():
    node = _StorageStub()
    node.storage_last_chance_enabled = True
    node._storage_route_phase = "approach_bottom_wall"
    node._now = node.state_enter_s + 15.0
    node._wall_segments_s = 0.0

    MissionFsmNode._step_drive_to_storage(node)

    assert node._storage_last_chance_active is True
    assert node._storage_last_chance_phase == "pre_backoff"
    assert node._storage_last_chance_turn_heading is None
    assert node.commands[-1] == pytest.approx((-0.10, 0.0, 0.0))
    assert any("STORAGE LAST-CHANCE no fresh wall" in d for d in node.decisions)
    assert any("BACKOFF 5cm" in d for d in node.decisions)

    node._now = node._storage_last_chance_phase_started_s + 0.5
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_last_chance_phase == "reverse_charge"
    assert node._storage_last_chance_turn_heading == pytest.approx(
        math.atan2(
            math.sin(math.pi / 2.0 + math.radians(95.0)),
            math.cos(math.pi / 2.0 + math.radians(95.0)),
        )
    )
    assert any("STORAGE LAST-CHANCE left turn 95deg" in d for d in node.decisions)

    node._now += 0.1
    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1] == pytest.approx((-0.20, 0.0, 0.0))

    node._now = node._storage_last_chance_phase_started_s + 5.0
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_last_chance_phase == "strafe_right"
    assert node.commands[-1] == pytest.approx((0.0, -0.20, 0.0))

    node._now = node._storage_last_chance_phase_started_s + 2.0
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_last_chance_phase == "alternate_reverse"
    assert node.commands[-1] == pytest.approx((-0.20, 0.0, 0.0))

    node._now = node._storage_last_chance_started_s + 12.0
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_completed is True
    assert node.state == "ALIGN_OVER_BIN"
    assert any("last-chance force dump" in d for d in node.decisions)


def test_storage_last_chance_waits_fifteen_seconds_after_storage_entry():
    node = _StorageStub()
    node.storage_last_chance_enabled = True
    node._storage_route_phase = "approach_bottom_wall"
    node._wall_segments_s = 0.0
    node._now = node.state_enter_s + 14.9

    assert MissionFsmNode._maybe_start_storage_last_chance(node) is False
    assert node._storage_last_chance_active is False

    node._now = node.state_enter_s + 15.0

    assert MissionFsmNode._maybe_start_storage_last_chance(node) is True
    assert node._storage_last_chance_active is True


def test_storage_last_chance_arrival_guidance_preempts_blind_pattern():
    node = _StorageStub()
    node.storage_last_chance_enabled = True
    node.storage_last_chance_arrival_guidance_enabled = True
    node._storage_last_chance_active = True
    node._storage_last_chance_phase = "reverse_charge"
    node._storage_last_chance_started_s = node._now
    node._storage_last_chance_phase_started_s = node._now
    guided = []
    node._arrival_base_target = lambda **_kwargs: (
        0.40, -0.30, 0.91, "wide_relative"
    )
    node._drive_toward_arrival = lambda *args, **kwargs: guided.append(
        (args, kwargs)
    )

    MissionFsmNode._step_storage_last_chance(node)

    assert len(guided) == 1
    assert guided[0][0] == (0.40, -0.30, 0.91, "wide_relative")
    assert guided[0][1]["speed"] == 0.20
    assert guided[0][1]["decision_prefix"] == "STORAGE LAST-CHANCE"
    assert node.commands == []


def test_storage_flag_disabled_still_uses_v520_wall_route():
    node = _StorageStub()
    node.storage_wall_guided_enabled = False

    MissionFsmNode._step_drive_to_storage(node)

    assert node._storage_route_phase == "approach_bottom_wall"


def test_storage_without_wall_detection_holds_stopped_during_dump():
    node = _StorageStub()
    node.storage_wall_guided_enabled = False

    MissionFsmNode._drive_storage_contact_hold(node)

    assert node.commands[-1] == (0.0, 0.0, 0.0)


class _FlagStorageStub(_StorageStub):
    storage_parking_method = 1
    storage_flag_staging_x = 1.0
    storage_flag_staging_y = 1.0
    storage_flag_staging_tolerance_m = 0.10
    storage_flag_face_x = 0.0
    storage_flag_face_y = 0.0
    storage_flag_min_confidence = 0.30
    storage_flag_max_age_sec = 0.75
    storage_flag_acquire_sec = 0.50
    storage_flag_floor_center_x = 1.8
    storage_flag_floor_center_y = 1.8
    storage_flag_floor_candidate_radius_m = 0.75
    storage_flag_pair_max_spacing_m = 0.80
    storage_flag_stop_distance_m = 0.05
    storage_flag_reverse_speed = 0.15
    storage_flag_heading_kp = 1.0
    storage_flag_omega_max = 0.08
    storage_fast_nav_speed = 0.20

    def __init__(self):
        super().__init__()
        self.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=0.0)
        self._storage_route_phase = "drive_to_flag_staging"
        self._storage_staging_heading = None
        self._storage_flag_acquire_started_s = self._now
        self._storage_flag_acquire_start_seq = 0
        self._storage_flag_target_field = None
        self._storage_flag_guidance_mode = "idle"
        self._wide_relative_seq = 0
        self.field_candidates = []

    def _drive(self, vx, vy, omega=0.0, **_kwargs):
        self.commands.append((vx, vy, omega))

    def _distance_to(self, x, y):
        return math.hypot(x - self.world.robot_x, y - self.world.robot_y)

    def _drive_toward_direct(self, x, y, **_kwargs):
        self.commands.append(("direct", x, y))

    def _wide_arrival_field_candidates(self, **_kwargs):
        return list(self.field_candidates)

    def _to_base(self, x, y):
        dx = x - self.world.robot_x
        dy = y - self.world.robot_y
        c = math.cos(self.world.robot_theta)
        s = math.sin(self.world.robot_theta)
        return c * dx + s * dy, -s * dx + c * dy

    def _step_flag_guided_storage(self):
        MissionFsmNode._step_flag_guided_storage(self)

    def _finish_storage_flag_parking(self, reason):
        MissionFsmNode._finish_storage_flag_parking(self, reason)


def test_method1_drives_to_one_one_then_faces_origin():
    node = _FlagStorageStub()

    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1] == ("direct", 1.0, 1.0)

    node.world.robot_x = 0.95
    node.world.robot_y = 0.95
    MissionFsmNode._step_drive_to_storage(node)

    assert node._storage_route_phase == "face_flag_reference"
    assert node.commands[-1] == (0.0, 0.0, 0.0)
    assert node._storage_staging_heading == pytest.approx(-3.0 * math.pi / 4.0)

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "acquire_wide_flag_snapshot"


def test_method1_snapshot_uses_single_flag_or_pair_midpoint():
    single = select_flag_snapshot_target(
        [(1.72, 1.84, 0.91)],
        expected_x=1.8,
        expected_y=1.8,
        candidate_radius_m=0.75,
        pair_max_spacing_m=0.80,
    )
    pair = select_flag_snapshot_target(
        [(1.70, 1.60, 0.80), (1.90, 2.00, 0.90), (0.2, 0.2, 0.99)],
        expected_x=1.8,
        expected_y=1.8,
        candidate_radius_m=0.75,
        pair_max_spacing_m=0.80,
    )

    assert single == pytest.approx((1.72, 1.84, 1))
    assert pair == pytest.approx((1.80, 1.80, 2))


def test_method1_latches_snapshot_then_ignores_later_detection_loss():
    node = _FlagStorageStub()
    node._storage_route_phase = "acquire_wide_flag_snapshot"
    node.world.robot_theta = -3.0 * math.pi / 4.0
    node._storage_flag_acquire_started_s = node._now - 0.50
    node._storage_flag_acquire_start_seq = 4
    node._wide_relative_seq = 5
    node.field_candidates = [(1.70, 1.60, 0.80), (1.90, 2.00, 0.90)]

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "reverse_to_latched_flag"
    assert node._storage_flag_target_field == pytest.approx((1.80, 1.80))

    node.field_candidates = []
    MissionFsmNode._step_drive_to_storage(node)
    vx, vy, omega = node.commands[-1]
    assert vx < 0.0
    assert vy == 0.0
    assert abs(omega) <= node.storage_flag_omega_max


def test_method1_continues_until_latched_target_is_within_five_centimeters():
    node = _FlagStorageStub()
    node._storage_route_phase = "reverse_to_latched_flag"
    node._storage_flag_target_field = (1.80, 1.80)
    node.world.robot_theta = -3.0 * math.pi / 4.0

    node.world.robot_x = 1.74
    node.world.robot_y = 1.80
    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1][0] < 0.0
    assert node.state == "DRIVE_TO_STORAGE"

    node.world.robot_x = 1.76
    node.world.robot_y = 1.80
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_completed is True
    assert node.state == "ALIGN_OVER_BIN"


class _Method2StorageStub(_StorageStub):
    storage_parking_method = 2
    storage_method2_contact_distance_m = 0.30
    storage_method2_contact_hold_sec = 0.8
    storage_method2_approach_speed = 0.08
    storage_method2_backoff_distance_m = 0.10
    storage_method2_backoff_speed = 0.10
    storage_method2_turn_left_deg = 95.0
    storage_method2_reverse_speed = 0.20
    storage_method2_reverse_before_dump_sec = 8.0

    def __init__(self):
        super().__init__()
        self._storage_route_phase = "method2_face_bottom_wall"
        self._storage_method2_phase_started_s = self._now
        self._storage_method2_turn_heading = None

    def _step_emergency_storage(self):
        MissionFsmNode._step_emergency_storage(self)

    def _finish_storage_method2_parking(self, reason):
        MissionFsmNode._finish_storage_method2_parking(self, reason)


def test_method2_wall_press_backoff_turn_and_continuous_reverse():
    node = _Method2StorageStub()

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "method2_approach_bottom_wall"

    node.world.robot_y = 1.75
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "method2_contact_hold"
    assert node.commands[-1][0] > 0.0

    node._now += node.storage_method2_contact_hold_sec
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "method2_backoff"
    assert node.commands[-1][0] < 0.0

    node._now += (
        node.storage_method2_backoff_distance_m
        / node.storage_method2_backoff_speed
    )
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "method2_turn_left"
    expected_heading = math.atan2(
        math.sin(math.pi / 2.0 + math.radians(95.0)),
        math.cos(math.pi / 2.0 + math.radians(95.0)),
    )
    assert node._storage_method2_turn_heading == pytest.approx(expected_heading)

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "method2_reverse"

    node._now += 0.1
    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1][0] == pytest.approx(-0.20)
    assert node.commands[-1][1] == 0.0

    node._now = (
        node._storage_method2_phase_started_s
        + node.storage_method2_reverse_before_dump_sec
    )
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_completed is True
    assert node.state == "ALIGN_OVER_BIN"


def test_method2_keeps_reverse_pressure_during_dump_sequence():
    node = _Method2StorageStub()
    node._storage_reverse_heading = math.radians(-175.0)
    node.world.robot_theta = node._storage_reverse_heading

    MissionFsmNode._drive_storage_contact_hold(node)

    assert node.commands[-1][0] == pytest.approx(-0.20)
    assert node.commands[-1][1] == 0.0
