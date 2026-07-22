import math

import numpy as np

from robot_perception.nodes.localizer_node import (
    LocalizerNode,
    blend_flow_velocity_with_imu,
)


class _Logger:
    def info(self, *_args, **_kwargs):
        pass


def _node(enabled=True):
    node = LocalizerNode.__new__(LocalizerNode)
    node.imu_flow_consistency_enabled = enabled
    node.imu_flow_min_gain = 0.10
    node.imu_flow_full_trust_error = 0.15
    node.imu_flow_soft_error = 0.40
    node.imu_flow_max_dt = 0.30
    node.theta = 0.0
    node._imu_flow_accel_xy = np.zeros(2, dtype=np.float64)
    node._imu_flow_velocity_xy = np.array([0.20, 0.0], dtype=np.float64)
    node._imu_flow_last_time = 0.0
    node._imu_flow_last_heading = 0.0
    node._imu_fresh = lambda _now: True
    node.get_logger = lambda: _Logger()
    return node


def test_small_velocity_error_keeps_full_flow_measurement():
    blended, gain, error = blend_flow_velocity_with_imu(
        np.array([0.20, 0.0]),
        np.array([0.10, 0.0]),
        0.10,
        0.15,
        0.40,
    )

    assert gain == 1.0
    assert math.isclose(error, 0.10)
    assert np.allclose(blended, [0.20, 0.0])


def test_large_velocity_jump_is_softened_but_never_discarded():
    blended, gain, error = blend_flow_velocity_with_imu(
        np.array([3.60, 0.0]),
        np.array([0.20, 0.0]),
        0.10,
        0.15,
        0.40,
    )

    assert gain == 0.10
    assert math.isclose(error, 3.40)
    assert np.allclose(blended, [0.54, 0.0])


def test_filter_turns_a_36cm_spike_into_a_continuous_5_4cm_update():
    node = _node()

    dfwd, dleft, gain = node._filter_object_flow_translation(0.36, 0.0, 0.10)

    assert gain == 0.10
    assert math.isclose(dfwd, 0.054, abs_tol=1e-9)
    assert dleft == 0.0


def test_imu_supported_acceleration_keeps_real_motion_unattenuated():
    node = _node()
    node._imu_flow_velocity_xy = np.array([0.10, 0.0], dtype=np.float64)
    node._imu_flow_accel_xy = np.array([1.0, 0.0], dtype=np.float64)

    dfwd, dleft, gain = node._filter_object_flow_translation(0.02, 0.0, 0.10)

    assert gain == 1.0
    assert math.isclose(dfwd, 0.02, abs_tol=1e-9)
    assert dleft == 0.0


def test_toggle_off_returns_the_original_flow_exactly():
    node = _node(enabled=False)

    assert node._filter_object_flow_translation(0.36, -0.12, 0.10) == (
        0.36,
        -0.12,
        1.0,
    )
