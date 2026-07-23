import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from robot_planning.local_anchor_fsm import LocalAnchorConfig, MotionCommand
from robot_planning.nodes.mission_fsm_node import (
    LOCAL_ANCHOR_STATE,
    MissionFsmNode,
    _local_anchor_observations,
)
from robot_planning.relative_anchor_grid import RelativeObservation


class _Logger:
    def __init__(self):
        self.info_messages = []

    def info(self, message):
        self.info_messages.append(str(message))

    def warn(self, _message, **_kwargs):
        pass

    def error(self, _message, **_kwargs):
        pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _ClockNow:
    def to_msg(self):
        return Time()


class _Clock:
    def now(self):
        return _ClockNow()


class _TerminalFsm:
    def __init__(self, state):
        self.state = state
        self.detail = f"terminal {state.lower()}"
        self.candidates = []

    def summary(self):
        return {"detail": self.detail}


class _TimeoutFsm(_TerminalFsm):
    def __init__(self):
        super().__init__("CLASSIFY")
        self.abort_reasons = []

    def abort(self, _now, reason):
        self.abort_reasons.append(str(reason))
        self.state = "ABORTED"
        self.detail = str(reason)

    def tick(self, _now, _yaw):
        return MotionCommand()


class _PickRequestFsm(_TerminalFsm):
    def __init__(self):
        super().__init__("PICK_TRIGGER")
        self.abort_reasons = []

    def abort(self, _now, reason):
        self.abort_reasons.append(str(reason))
        self.state = "ABORTED"
        self.detail = str(reason)

    def tick(self, _now, _yaw):
        self.state = "PICK_WAIT"
        return MotionCommand(request_pick=True)


def _mission_stub():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.state = "SCAN"
    node.zone_order = [1, 2, 3, 4]
    node._zone_idx = 0
    node.zone_mission_enabled = True
    node.zone_anchor_nav_enabled = True
    node.zone_stabilize_enabled = True
    node.local_anchor_enabled = True
    node.local_anchor_run_timeout_sec = 60.0
    node.local_anchor_config = LocalAnchorConfig(
        target_fruit_label="banana",
        enable_align=True,
        enable_pick=True,
    )
    node._local_anchor_visited_zones = set()
    node._local_anchor_active_zone_id = None
    node._local_anchor_relative_yaw = 0.0
    node._local_anchor_start_pose = None
    node._local_anchor_pick_triggered = False
    node._local_anchor_pick_accounted = False
    node._local_anchor_pose_baseline_seq = 0
    node._local_anchor_world_baseline_seq = 0
    node._local_anchor_pose_refresh_required = False
    node._localization_pose_seq = 10
    node._world_seq = 20
    node._localization_pose_last_s = 25.0
    node._world_last_rx_s = 25.0
    node._localization_pose = (1.0, 2.0, 0.25)
    node.local_anchor_pose_confirm_frames = 3
    node.local_anchor_world_confirm_frames = 2
    node.local_anchor_pose_max_age_sec = 0.5
    node.local_anchor_pose_world_xy_tolerance_m = 0.03
    node.local_anchor_pose_world_theta_tolerance_rad = math.radians(3.0)
    node.local_anchor_input_timeout_sec = 0.75
    node._imu_last_s = 25.0
    node._wide_relative_last_frame_s = 25.0
    node._body_H = object()
    node.current_target = None
    node._opportunistic_set2_active = False
    node.set2_label = "banana"
    node.tray_fruit = 0
    node.world = SimpleNamespace(
        robot_x=1.0,
        robot_y=2.0,
        robot_theta=0.25,
        objects=[],
    )
    node._now_s = lambda: 25.0
    node._time_in_state = lambda: 0.0
    node._clear_current_slot = lambda: None
    node._reset_zone_scan_timer = lambda: None
    node._drive = lambda *_args: None
    node._decisions = []
    node._decide = node._decisions.append
    node._blacklist_local_anchor_pick_track = lambda: None
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node._entered_states = []

    def enter(state):
        node.state = state
        node._entered_states.append(state)

    node._enter = enter
    return node


def test_base_observations_are_rotated_into_fixed_anchor_frame():
    observations = [
        RelativeObservation(
            x=0.20,
            y=-0.10,
            label="fruit_photo_cube",
            set_type=2,
            confidence=0.91,
        ),
        RelativeObservation(
            x=0.30,
            y=0.40,
            label="arrival",
            set_type=3,
            confidence=0.99,
        ),
    ]

    converted = _local_anchor_observations(
        observations,
        math.pi / 2.0,
        {"fruit_photo_cube"},
    )

    assert len(converted) == 1
    assert converted[0].x == pytest.approx(0.10)
    assert converted[0].y == pytest.approx(0.20)
    assert converted[0].label == "fruit_photo_cube"
    assert converted[0].confidence == pytest.approx(0.91)


def test_each_zone_starts_local_inspection_exactly_once():
    node = _mission_stub()

    for zone_index, zone_id in enumerate(node.zone_order):
        node._zone_idx = zone_index
        node.state = "SCAN"

        assert MissionFsmNode._start_local_anchor_inspection(node)
        assert node._local_anchor_active_zone_id == zone_id
        assert not MissionFsmNode._start_local_anchor_inspection(node)

    assert node._local_anchor_visited_zones == {1, 2, 3, 4}
    assert node._entered_states.count(LOCAL_ANCHOR_STATE) == 4


def test_complete_local_pick_is_accounted_once_and_resumes_from_live_pose():
    node = _mission_stub()
    node.state = LOCAL_ANCHOR_STATE
    node._local_anchor_active_zone_id = 2
    node._local_anchor_start_pose = (1.0, 2.0, 0.25)
    node._local_anchor_pick_triggered = True
    node._local_anchor_fsm = _TerminalFsm("COMPLETE")

    # The live world pose moved while the embedded FSM rotated and aligned.
    node.world.robot_x = 2.30
    node.world.robot_y = -0.40
    node.world.robot_theta = 1.20
    live_pose = (
        node.world.robot_x,
        node.world.robot_y,
        node.world.robot_theta,
    )

    MissionFsmNode._finish_local_anchor_inspection(node)
    MissionFsmNode._finish_local_anchor_inspection(node)

    assert node.tray_fruit == 1
    assert node._local_anchor_pick_accounted
    assert node.state == "ZONE_STABILIZE"
    assert node._entered_states[-1] == "ZONE_STABILIZE"
    assert (
        node.world.robot_x,
        node.world.robot_y,
        node.world.robot_theta,
    ) == live_pose
    assert any(
        "pose=(2.30,-0.40,+68.8deg)" in msg
        for msg in node._logger.info_messages
    )


def test_local_pick_blacklist_cannot_consume_a_nearby_set1_track():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._local_anchor_start_pose = (1.0, 2.0, 0.0)
    node.local_anchor_picked_track_match_radius_m = 0.25
    node._local_anchor_fsm = SimpleNamespace(
        candidates=[
            SimpleNamespace(status="PICKED", x=0.20, y=0.0),
        ]
    )
    node.world = SimpleNamespace(
        objects=[
            SimpleNamespace(
                id=11,
                set_type=1,
                x=1.20,
                y=2.0,
                blacklisted=False,
            ),
            SimpleNamespace(
                id=22,
                set_type=2,
                x=1.24,
                y=2.0,
                blacklisted=False,
            ),
        ]
    )
    blacklisted = []
    node._blacklist = blacklisted.append
    node._decide = lambda _message: None

    MissionFsmNode._blacklist_local_anchor_pick_track(node)

    assert blacklisted == [22]


def test_local_fault_returns_to_zone_stabilize_without_counting_a_pick():
    node = _mission_stub()
    node.state = LOCAL_ANCHOR_STATE
    node._local_anchor_active_zone_id = 3
    node._local_anchor_fsm = _TerminalFsm("FAULT")
    node._local_anchor_pick_triggered = False

    MissionFsmNode._finish_local_anchor_inspection(node)

    assert node.tray_fruit == 0
    assert node.state == "ZONE_STABILIZE"
    assert node._entered_states[-1] == "ZONE_STABILIZE"


def test_local_run_timeout_aborts_and_returns_to_zone_stabilize():
    node = _mission_stub()
    node.state = LOCAL_ANCHOR_STATE
    node._local_anchor_active_zone_id = 4
    node.local_anchor_run_timeout_sec = 60.0
    node._time_in_state = lambda: 61.0
    node._local_anchor_fsm = _TimeoutFsm()
    node.dry_pick = False
    node._publish_pick = lambda _trigger: pytest.fail(
        "timeout must not trigger a pick"
    )

    MissionFsmNode._step_local_anchor_inspection(node)

    assert node._local_anchor_fsm.abort_reasons == ["local run timeout 60.0s"]
    assert node.tray_fruit == 0
    assert node.state == "ZONE_STABILIZE"
    assert node._entered_states[-1] == "ZONE_STABILIZE"


def test_stale_local_inputs_abort_without_commanding_motion():
    node = _mission_stub()
    node.state = LOCAL_ANCHOR_STATE
    node._local_anchor_active_zone_id = 1
    node._local_anchor_fsm = _TimeoutFsm()
    node._imu_last_s = 20.0
    node._wide_relative_last_frame_s = 25.0
    node.dry_pick = False
    drive_commands = []
    node._drive = lambda *args: drive_commands.append(args)

    MissionFsmNode._step_local_anchor_inspection(node)

    assert node._local_anchor_fsm.abort_reasons == [
        "IMU stream is stale"
    ]
    assert all(command == (0.0, 0.0, 0.0) for command in drive_commands)
    assert node.state == "ZONE_STABILIZE"


def test_live_local_pick_requires_an_arm_subscriber_before_accounting():
    node = _mission_stub()
    node.state = LOCAL_ANCHOR_STATE
    node._local_anchor_active_zone_id = 1
    node._local_anchor_fsm = _PickRequestFsm()
    node.dry_pick = False
    node.get_subscriptions_info_by_topic = lambda _topic: []
    node._publish_pick = lambda _trigger: pytest.fail(
        "missing arm subscriber must suppress the pick trigger"
    )

    MissionFsmNode._step_local_anchor_inspection(node)

    assert node._local_anchor_fsm.abort_reasons == [
        "pick sequencer subscriber is unavailable"
    ]
    assert node.tray_fruit == 0
    assert not node._local_anchor_pick_triggered
    assert node.state == "ZONE_STABILIZE"


def test_post_local_pose_gate_requires_new_pose_and_world_frames():
    node = _mission_stub()
    node._local_anchor_pose_refresh_required = True
    node._local_anchor_pose_baseline_seq = 10
    node._local_anchor_world_baseline_seq = 20

    ready, detail = MissionFsmNode._local_anchor_pose_refresh_status(node)

    assert not ready
    assert "localization pose frames 0/3" in detail

    node._localization_pose_seq = 13
    node._world_seq = 22
    node._localization_pose = (1.20, 2.10, 0.40)
    node.world.robot_x = 1.20
    node.world.robot_y = 2.10
    node.world.robot_theta = 0.40

    ready, detail = MissionFsmNode._local_anchor_pose_refresh_status(node)

    assert ready
    assert "pose confirmed" in detail


@pytest.mark.parametrize("local_state", ["PICK_TRIGGER", "PICK_WAIT"])
def test_timed_storage_is_deferred_while_local_pick_owns_the_arm(local_state):
    node = _mission_stub()
    node.state = LOCAL_ANCHOR_STATE
    node._local_anchor_fsm = _TerminalFsm(local_state)
    node.timed_storage_enabled = True
    node._run_started = True
    node._timed_storage_triggered = False
    node._storage_route_completed = False
    node._node_start_s = 0.0
    node.timed_storage_start_sec = 50.0
    node._now_s = lambda: 75.0
    drive_commands = []
    node._drive = lambda *args: drive_commands.append(args)

    MissionFsmNode._maybe_start_timed_storage(node)

    assert drive_commands == [(0.0, 0.0, 0.0)]
    assert not node._timed_storage_triggered
    assert "DRIVE_TO_STORAGE" not in node._entered_states


def test_timed_storage_is_deferred_until_post_local_pose_is_fresh():
    node = _mission_stub()
    node.state = "ZONE_STABILIZE"
    node._local_anchor_pose_refresh_required = True
    node.timed_storage_enabled = True
    node._run_started = True
    node._timed_storage_triggered = False
    node._storage_route_completed = False
    node._node_start_s = 0.0
    node.timed_storage_start_sec = 50.0
    node._now_s = lambda: 75.0
    drive_commands = []
    node._drive = lambda *args: drive_commands.append(args)

    MissionFsmNode._maybe_start_timed_storage(node)

    assert drive_commands == [(0.0, 0.0, 0.0)]
    assert not node._timed_storage_triggered
    assert "DRIVE_TO_STORAGE" not in node._entered_states


def test_timed_storage_aborts_local_motion_then_waits_for_fresh_pose():
    node = _mission_stub()
    node.state = LOCAL_ANCHOR_STATE
    node._local_anchor_active_zone_id = 2
    node._local_anchor_fsm = _TimeoutFsm()
    node.timed_storage_enabled = True
    node._run_started = True
    node._timed_storage_triggered = False
    node._storage_route_completed = False
    node._node_start_s = 0.0
    node.timed_storage_start_sec = 50.0
    node._now_s = lambda: 75.0

    MissionFsmNode._maybe_start_timed_storage(node)

    assert node._local_anchor_fsm.abort_reasons == [
        "timed storage requested; stop for pose refresh"
    ]
    assert node.state == "ZONE_STABILIZE"
    assert node._local_anchor_pose_refresh_required
    assert not node._timed_storage_triggered
    assert "DRIVE_TO_STORAGE" not in node._entered_states


def test_local_inspection_publishes_closed_track_birth_gate():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.state = LOCAL_ANCHOR_STATE
    node.get_clock = lambda: _Clock()
    node.pub_track_birth_slots = _Publisher()
    node.pub_track_birth_enabled = _Publisher()

    MissionFsmNode._publish_track_birth_gate(node)

    assert len(node.pub_track_birth_slots.messages) == 1
    assert node.pub_track_birth_slots.messages[0].poses == []
    assert len(node.pub_track_birth_enabled.messages) == 1
    assert node.pub_track_birth_enabled.messages[0].data is False
