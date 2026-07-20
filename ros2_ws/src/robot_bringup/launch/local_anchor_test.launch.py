"""Isolated local-anchor scan/classification test; never starts mission_fsm_node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")
    default_params = os.path.join(bringup_share, "config", "test_field.yaml")
    default_motion = os.path.join(bringup_share, "config", "motion_tuning.yaml")
    default_local = os.path.join(bringup_share, "config", "local_anchor_test.yaml")

    params = LaunchConfiguration("params_file")
    motion = LaunchConfiguration("motion_tuning_file")
    local = LaunchConfiguration("local_anchor_params_file")
    drive_enabled = LaunchConfiguration("drive_enabled")
    armed = LaunchConfiguration("armed")
    with_base = LaunchConfiguration("with_base")
    with_siglip = LaunchConfiguration("with_siglip")
    cam_yaw = LaunchConfiguration("cam_yaw")
    rot180 = LaunchConfiguration("rot180")
    web_port = LaunchConfiguration("web_port")

    def node(package, executable, name, files=None, condition=None, extra=None):
        parameters = list(files or [params, motion])
        if extra:
            parameters.append(extra)
        return Node(
            package=package,
            executable=executable,
            name=name,
            parameters=parameters,
            output="screen",
            condition=condition,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("motion_tuning_file", default_value=default_motion),
            DeclareLaunchArgument("local_anchor_params_file", default_value=default_local),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("armed", default_value="false"),
            DeclareLaunchArgument("with_base", default_value="true"),
            DeclareLaunchArgument("with_siglip", default_value="true"),
            DeclareLaunchArgument("cam_yaw", default_value="90"),
            DeclareLaunchArgument("rot180", default_value="true"),
            DeclareLaunchArgument("web_port", default_value="8082"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "cameras.launch.py"))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "static_transforms.launch.py")
                )
            ),
            node("robot_hardware", "imu_mpu6050_node", "imu_mpu6050_node"),
            node("robot_perception", "localizer_node", "localizer_node"),
            node("robot_perception", "yolo_detector_node", "yolo_detector_node"),
            node(
                "robot_perception",
                "world_model_node",
                "world_model_node",
                extra={
                    "cam_yaw_deg": ParameterValue(cam_yaw, value_type=float),
                    "image_rotated_180": ParameterValue(rot180, value_type=bool),
                    "grid_prior_enabled": False,
                    "grid_track_lock_enabled": False,
                },
            ),
            node(
                "robot_perception",
                "siglip_gate_node",
                "siglip_gate_node",
                condition=IfCondition(with_siglip),
            ),
            node(
                "robot_control",
                "base_controller_node",
                "base_controller_node",
                files=[params, motion, local],
                condition=IfCondition(with_base),
            ),
            node(
                "robot_hardware",
                "mcu_bridge_base_node",
                "mcu_bridge_base_node",
                condition=IfCondition(with_base),
            ),
            node(
                "robot_planning",
                "local_anchor_test_node",
                "local_anchor_test_node",
                files=[local],
                extra={
                    "drive_enabled": ParameterValue(drive_enabled, value_type=bool),
                    "armed": ParameterValue(armed, value_type=bool),
                },
            ),
            node(
                "robot_planning",
                "local_anchor_web_node",
                "local_anchor_web_node",
                files=[local],
                extra={"web_port": ParameterValue(web_port, value_type=int)},
            ),
        ]
    )
