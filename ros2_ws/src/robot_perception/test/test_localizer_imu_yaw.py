import math

from robot_perception.nodes.localizer_node import LocalizerNode, limit_planar_delta


def test_limit_planar_delta_caps_distance_and_preserves_direction():
    dx, dy, limited = limit_planar_delta(0.06, 0.08, 0.02)

    assert limited
    assert math.isclose(math.hypot(dx, dy), 0.02, abs_tol=1e-9)
    assert math.isclose(dx / dy, 0.06 / 0.08, abs_tol=1e-9)


def test_limit_planar_delta_keeps_physically_small_correction():
    dx, dy, limited = limit_planar_delta(0.006, -0.008, 0.02)

    assert not limited
    assert (dx, dy) == (0.006, -0.008)


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
