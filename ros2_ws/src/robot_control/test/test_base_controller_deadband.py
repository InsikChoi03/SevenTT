import math

from robot_control.nodes.base_controller_node import (
    below_motion_deadband,
    forward_start_boost_active,
    is_precision_strafe_command,
    local_anchor_motion_profile_active,
    mix_calibrated_translation_and_rotation,
    select_brake_pulse_parameters,
    select_mission_profile_value,
    select_wheel_scales,
)


def test_forward_start_boost_is_scoped_to_normal_navigation():
    assert forward_start_boost_active("SCAN", 0.05, 0.0)
    assert forward_start_boost_active("APPROACH", 0.05, 0.0)
    assert not forward_start_boost_active("OPENING", 0.05, 0.0)
    assert not forward_start_boost_active("ALIGN", 0.05, 0.0)
    assert not forward_start_boost_active("DRIVE_TO_STORAGE", 0.05, 0.0)


def test_forward_start_boost_excludes_reverse_and_strafe():
    assert not forward_start_boost_active("SCAN", -0.05, 0.0)
    assert not forward_start_boost_active("SCAN", 0.0, 0.05)


def test_combined_drive_keeps_translation_trim_and_adds_symmetric_yaw():
    wheels = mix_calibrated_translation_and_rotation(
        vx=0.12,
        vy=0.0,
        omega=-0.16,
        k=0.20,
        translation_scales=[0.9, 1.4, 0.7, 1.3],
        drive_rotation_scales=[2.5, 2.5, 2.5, 2.5],
    )

    assert all(
        math.isclose(actual, expected, abs_tol=1e-12)
        for actual, expected in zip(wheels, [0.188, 0.088, 0.164, 0.076])
    )


def test_drive_rotation_scale_does_not_change_straight_output():
    wheels = mix_calibrated_translation_and_rotation(
        vx=0.12,
        vy=0.0,
        omega=0.0,
        k=0.20,
        translation_scales=[0.9, 1.4, 0.7, 1.3],
        drive_rotation_scales=[2.5, 2.5, 2.5, 2.5],
    )

    assert all(
        math.isclose(actual, expected, abs_tol=1e-12)
        for actual, expected in zip(wheels, [0.108, 0.168, 0.084, 0.156])
    )


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


def test_local_anchor_motion_profile_is_scoped_to_its_two_mission_states():
    assert local_anchor_motion_profile_active("LOCAL_ANCHOR_INSPECTION")
    assert local_anchor_motion_profile_active("LOCAL_ANCHOR_ALIGN")
    assert not local_anchor_motion_profile_active("SCAN")
    assert not local_anchor_motion_profile_active("ALIGN")
    assert not local_anchor_motion_profile_active("OPENING")


def test_mission_profile_value_is_scoped_to_local_anchor_states():
    main = {"wheel_min_rot": 0.16, "wheel_slew": 0.02}
    local = {"wheel_min_rot": 0.07, "wheel_slew": 0.06}

    assert select_mission_profile_value("SCAN", main, local) is main
    assert select_mission_profile_value("ALIGN", main, local) is main
    assert (
        select_mission_profile_value("LOCAL_ANCHOR_INSPECTION", main, local)
        is local
    )
    assert (
        select_mission_profile_value("LOCAL_ANCHOR_ALIGN", main, local)
        is local
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


def test_calibrated_lateral_pulse_uses_precision_profile_in_both_directions():
    assert is_precision_strafe_command(0.0, 0.315, 0.0, 0.315, 0.005)
    assert is_precision_strafe_command(0.0, -0.315, 0.0, 0.315, 0.005)


def test_other_strafe_speed_does_not_use_precision_profile():
    assert not is_precision_strafe_command(0.0, 0.20, 0.0, 0.315, 0.005)


def test_normal_stop_uses_normal_brake_profile():
    assert select_brake_pulse_parameters(
        align_profile=False,
        rotation_profile=False,
        align_brake_off=False,
        wheel_brake_ms=180,
        wheel_brake_scale=0.5,
        rotation_wheel_brake_ms=90,
        rotation_wheel_brake_scale=0.2,
        align_wheel_brake_ms=120,
        align_wheel_brake_scale=0.315,
    ) == (180.0, 0.5)


def test_rotation_stop_uses_independent_rotation_brake_profile():
    assert select_brake_pulse_parameters(
        align_profile=False,
        rotation_profile=True,
        align_brake_off=False,
        wheel_brake_ms=180,
        wheel_brake_scale=0.5,
        rotation_wheel_brake_ms=90,
        rotation_wheel_brake_scale=0.2,
        align_wheel_brake_ms=120,
        align_wheel_brake_scale=0.315,
    ) == (90.0, 0.2)


def test_align_stop_uses_independent_align_brake_profile():
    assert select_brake_pulse_parameters(
        align_profile=True,
        rotation_profile=True,
        align_brake_off=False,
        wheel_brake_ms=180,
        wheel_brake_scale=0.5,
        rotation_wheel_brake_ms=90,
        rotation_wheel_brake_scale=0.2,
        align_wheel_brake_ms=120,
        align_wheel_brake_scale=0.315,
    ) == (120.0, 0.315)


def test_align_brake_off_disables_only_align_profile():
    assert select_brake_pulse_parameters(
        align_profile=True,
        rotation_profile=True,
        align_brake_off=True,
        wheel_brake_ms=180,
        wheel_brake_scale=0.5,
        rotation_wheel_brake_ms=90,
        rotation_wheel_brake_scale=0.2,
        align_wheel_brake_ms=120,
        align_wheel_brake_scale=0.315,
    ) == (0.0, 0.0)
