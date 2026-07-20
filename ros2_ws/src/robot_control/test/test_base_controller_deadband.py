from robot_control.nodes.base_controller_node import below_motion_deadband


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
