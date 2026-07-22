import math
from types import SimpleNamespace

import pytest

from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    directional_wall_distance,
    straight_forward_command,
    straight_reverse_command,
    timed_storage_due,
)


def test_storage_deadline_is_relative_to_running_start():
    common = dict(
        run_started=True,
        triggered=False,
        completed=False,
        run_start_s=20.0,
        trigger_sec=150.0,
    )

    assert not timed_storage_due(now_s=169.99, **common)
    assert timed_storage_due(now_s=170.0, **common)


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


class _Logger:
    def warn(self, *_args, **_kwargs):
        pass


class _StorageStub:
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

    def __init__(self):
        self.world = SimpleNamespace(robot_x=0.0, robot_y=0.0, robot_theta=math.pi / 2.0)
        self._field_bounds = (-2.0, 2.0, -2.0, 2.0)
        self._storage_route_phase = "face_bottom_wall"
        self._storage_route_completed = False
        self._storage_wall_confirm_count = 0
        self._storage_wall_confirm_last_seq = -1
        self._wall_segments = []
        self._wall_segments_s = 10.0
        self._wall_segments_seq = 0
        self._now = 10.0
        self.commands = []
        self.decisions = []
        self.state = "DRIVE_TO_STORAGE"

    def set_segments(self, *segments):
        self._wall_segments = list(segments)
        self._wall_segments_s = self._now
        self._wall_segments_seq += 1

    def _now_s(self):
        return self._now

    def _drive(self, vx, vy, omega=0.0):
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


def test_storage_holds_near_wall_until_a_fresh_wall_segment_exists():
    node = _StorageStub()
    node._storage_route_phase = "approach_bottom_wall"
    node.world.robot_y = 1.45
    node._wall_segments_s = 0.0

    MissionFsmNode._step_drive_to_storage(node)

    assert node.commands[-1] == (0.0, 0.0, 0.0)
    assert node._storage_route_phase == "approach_bottom_wall"
