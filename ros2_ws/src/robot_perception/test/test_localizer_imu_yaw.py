from robot_perception.nodes.localizer_node import LocalizerNode


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
