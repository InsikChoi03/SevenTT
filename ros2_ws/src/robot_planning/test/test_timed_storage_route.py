import math
from types import SimpleNamespace

import pytest

from robot_planning.nodes.mission_fsm_node import (
    MissionFsmNode,
    parking_face_heading,
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


def test_staging_faces_origin_and_reverse_has_no_lateral_command():
    heading = parking_face_heading(1.5, 1.5, 0.0, 0.0)
    vx, vy, omega = straight_reverse_command(
        current_heading=heading,
        target_heading=heading,
        distance_m=0.3,
        max_speed=0.08,
        heading_kp=1.0,
        omega_max=0.06,
    )

    assert heading == pytest.approx(-3.0 * math.pi / 4.0)
    assert vx == pytest.approx(-0.08)
    assert vy == 0.0
    assert omega == 0.0


def test_staging_direct_command_moves_forward_without_lateral_translation():
    vx, vy, omega = straight_forward_command(
        current_heading=math.pi / 4.0,
        target_heading=math.pi / 4.0,
        distance_m=1.0,
        max_speed=0.12,
        heading_kp=1.0,
        omega_max=0.06,
    )

    assert vx == pytest.approx(0.12)
    assert vy == 0.0
    assert omega == 0.0


class _StorageStub:
    storage_staging_x = 1.5
    storage_staging_y = 1.5
    storage_face_x = 0.0
    storage_face_y = 0.0
    storage_staging_reach_tol_m = 0.10
    storage_heading_tolerance_rad = 0.06
    storage_x = 1.8
    storage_y = 1.8
    storage_reach_tol_m = 0.08
    storage_reverse_realign_rad = 0.1745
    storage_reverse_speed = 0.08
    storage_reverse_heading_kp = 1.0
    storage_reverse_omega_max = 0.06
    direct_nav_speed = 0.12

    def __init__(self):
        self.world = SimpleNamespace(robot_x=1.5, robot_y=1.5, robot_theta=-3.0 * math.pi / 4.0)
        self._storage_route_phase = "to_staging"
        self._storage_staging_heading = None
        self._storage_reverse_heading = -3.0 * math.pi / 4.0
        self._storage_route_completed = False
        self.commands = []
        self.decisions = []
        self.state = "DRIVE_TO_STORAGE"

    def _distance_to(self, x, y):
        return math.hypot(x - self.world.robot_x, y - self.world.robot_y)

    def _drive(self, vx, vy, omega=0.0):
        self.commands.append((vx, vy, omega))

    def _drive_toward(self, *_args, **_kwargs):
        raise AssertionError("staging is already reached")

    def _reset_pulsed_heading(self):
        pass

    def _decide(self, message):
        self.decisions.append(message)

    def _turn_in_place_pulsed(self, target, *_args, **_kwargs):
        self._drive(0.0, 0.0, 0.0)
        return True

    @staticmethod
    def _wrap_pi(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _to_base(self, x, y):
        dx = x - self.world.robot_x
        dy = y - self.world.robot_y
        ct = math.cos(self.world.robot_theta)
        st = math.sin(self.world.robot_theta)
        return ct * dx + st * dy, -st * dx + ct * dy

    def _finish_storage_reverse(self, reason):
        MissionFsmNode._finish_storage_reverse(self, reason)

    def _enter(self, state):
        self.state = state


def test_storage_route_stops_faces_origin_then_reverses_straight():
    node = _StorageStub()

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "face_origin"
    assert node.commands[-1] == (0.0, 0.0, 0.0)

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_phase == "reverse"

    node.world.robot_x = 1.6
    node.world.robot_y = 1.6
    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1][0] < 0.0
    assert node.commands[-1][1] == 0.0

    node.world.robot_x = 1.8
    node.world.robot_y = 1.8
    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_route_completed is True
    assert node.state == "ALIGN_OVER_BIN"


def test_storage_route_aligns_then_drives_directly_to_staging():
    node = _StorageStub()
    node.world.robot_x = 0.0
    node.world.robot_y = 0.0
    node.world.robot_theta = math.pi / 4.0

    MissionFsmNode._step_drive_to_storage(node)
    assert node._storage_staging_heading == pytest.approx(math.pi / 4.0)
    assert node._storage_route_phase == "straight_to_staging"
    assert node.commands[-1] == (0.0, 0.0, 0.0)

    MissionFsmNode._step_drive_to_storage(node)
    assert node.commands[-1][0] > 0.0
    assert node.commands[-1][1] == 0.0
