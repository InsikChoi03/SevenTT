from robot_control.nodes.base_controller_node import below_motion_deadband, select_wheel_scales


def test_low_speed_pure_rotation_reaches_rotation_floor():
    assert not below_motion_deadband(
        0.014,
        0.02,
        pure_rotation=True,
    )


def test_small_non_rotation_noise_stays_inside_deadband():
    assert below_motion_deadband(
        0.014,
        0.02,
        pure_rotation=False,
    )


def test_pure_clockwise_rotation_uses_rotation_scales():
    scales = select_wheel_scales(
        0.0,
        0.0,
        -0.10,
        [1.0, 1.0, 1.0, 1.0],
        [0.8, 0.8, 0.8, 0.8],
        [0.7, 0.7, 0.7, 0.7],
        [1.15, 1.0, 1.0, 1.0],
        [1.05, 1.0, 1.0, 1.0],
    )

    assert scales == [1.15, 1.0, 1.0, 1.0]


def test_pure_counterclockwise_rotation_uses_rotation_scales():
    scales = select_wheel_scales(
        0.0,
        0.0,
        0.10,
        [1.0, 1.0, 1.0, 1.0],
        [0.8, 0.8, 0.8, 0.8],
        [0.7, 0.7, 0.7, 0.7],
        [1.15, 1.0, 1.0, 1.0],
        [1.05, 1.0, 1.0, 1.0],
    )

    assert scales == [1.05, 1.0, 1.0, 1.0]


def test_forward_motion_keeps_forward_wheel_scales():
    scales = select_wheel_scales(
        0.10,
        0.0,
        0.0,
        [1.0, 0.95, 1.0, 0.95],
        [0.8, 0.8, 0.8, 0.8],
        [0.7, 0.7, 0.7, 0.7],
        [1.15, 1.0, 1.0, 1.0],
        [1.05, 1.0, 1.0, 1.0],
    )

    assert scales == [1.0, 0.95, 1.0, 0.95]
