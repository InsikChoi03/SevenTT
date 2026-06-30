"""Static tf2 transforms for the robot rig.

Frames (REP-103, x forward / y left / z up):
    base_link -> camera_top   wide-angle cam on the rear lift, ~0.8 m up, tilted down
    base_link -> arm_base     arm mount at the front
    arm_wrist -> camera_body  eye-in-hand cam just above the gripper wrist

NOTE: camera_body rides the moving arm wrist; in production its pose comes from the arm
tf chain (joint states), not a fixed transform. The arm_wrist->camera_body entry here is the
single static offset (rulebook/hardware: one static_transform from wrist to body cam).
The world_model/localizer nodes project using their OWN extrinsic params (cam_height_m,
cam_pitch_deg, cam_offset_*) rather than these tf frames, so these are mainly for tf
completeness / RViz. Measure and replace the placeholder values.

static_transform_publisher arg order: --x --y --z --yaw --pitch --roll --frame-id --child-frame-id
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, x, y, z, yaw, pitch, roll, parent, child):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x), "--y", str(y), "--z", str(z),
            "--yaw", str(yaw), "--pitch", str(pitch), "--roll", str(roll),
            "--frame-id", parent, "--child-frame-id", child,
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # rear lift, raised, pitched down ~0.5 rad (placeholder — measure)
        _static_tf("tf_base_to_camera_top", -0.15, 0.0, 0.80, 0.0, 0.5, 0.0,
                   "base_link", "camera_top"),
        # arm base at the front of the deck
        _static_tf("tf_base_to_arm_base", 0.15, 0.0, 0.05, 0.0, 0.0, 0.0,
                   "base_link", "arm_base"),
        # body cam offset from the wrist (eye-in-hand)
        _static_tf("tf_wrist_to_camera_body", 0.0, 0.0, 0.04, 0.0, 0.0, 0.0,
                   "arm_wrist", "camera_body"),
    ])
