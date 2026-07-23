import math

from robot_perception.nodes.localizer_node import (
    LocalizerNode,
    limit_planar_delta,
    wall_pose_update_allowed,
)


def test_limit_planar_delta_caps_distance_and_preserves_direction():
    dx, dy, limited = limit_planar_delta(0.06, 0.08, 0.02)

    assert limited
    assert math.isclose(math.hypot(dx, dy), 0.02, abs_tol=1e-9)
    assert math.isclose(dx / dy, 0.06 / 0.08, abs_tol=1e-9)


def test_limit_planar_delta_keeps_physically_small_correction():
    dx, dy, limited = limit_planar_delta(0.006, -0.008, 0.02)

    assert not limited
    assert (dx, dy) == (0.006, -0.008)


def test_wall_pose_update_waits_for_stationary_when_requested():
    assert not wall_pose_update_allowed(True, True, False)
    assert wall_pose_update_allowed(True, True, True)
    assert wall_pose_update_allowed(True, False, False)
    assert not wall_pose_update_allowed(False, True, True)


class _ConstraintStub:
    @staticmethod
    def _motion_constraint_gains(_source):
        return 0.0, 0.0, 0.05


def test_imu_yaw_bypasses_encoder_stop_attenuation():
    delta = LocalizerNode._constrain_robot_delta(
        _ConstraintStub(), 0.0, 0.0, 0.4, "imu"
    )

    assert delta == (0.0, 0.0, 0.4)


def test_camera_yaw_keeps_encoder_stop_attenuation():
    delta = LocalizerNode._constrain_robot_delta(
        _ConstraintStub(), 0.0, 0.0, 0.4, "object_flow"
    )

    assert delta == (0.0, 0.0, 0.020000000000000004)


class _StoppedLandmarkConstraintStub:
    encoder_motion_constraint_enabled = True
    encoder_constraint_stop_translation_gain = 0.0
    encoder_constraint_stop_yaw_gain = 0.05
    encoder_constraint_forward_gain = 1.0
    encoder_constraint_forward_lateral_gain = 0.12
    encoder_constraint_forward_yaw_gain = 1.0
    encoder_constraint_lateral_forward_gain = 0.12
    encoder_constraint_lateral_gain = 1.0
    encoder_constraint_lateral_yaw_gain = 1.0
    encoder_constraint_rotate_translation_gain = 0.08
    encoder_constraint_rotate_yaw_gain = 1.0
    encoder_constraint_mixed_translation_gain = 0.60
    encoder_constraint_mixed_yaw_gain = 0.80
    encoder_constraint_wall_translation_floor = 1.0
    encoder_constraint_wall_yaw_floor = 1.0
    encoder_constraint_landmark_translation_floor = 1.0
    _bounded_gain = staticmethod(LocalizerNode._bounded_gain)

    @staticmethod
    def _motion_mode(_now):
        return "STOP"

    @staticmethod
    def get_clock():
        class _Now:
            nanoseconds = 0

        class _Clock:
            @staticmethod
            def now():
                return _Now()

        return _Clock()


def test_grid_landmark_translation_bypasses_encoder_stop_freeze():
    gains = LocalizerNode._motion_constraint_gains(
        _StoppedLandmarkConstraintStub(), "object_landmark"
    )

    assert gains == (1.0, 1.0, 0.05)
