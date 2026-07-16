"""Independent real-robot arrival parking test launch.

This launch intentionally excludes mission_fsm, target_selector, explorer, go_to_goal,
and all arm nodes. with_base is false by default, so the robot will not move unless the
operator explicitly enables the base layer and the parking node safety gates.
"""
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
    parking_share = get_package_share_directory("robot_parking_test")
    bringup_share = get_package_share_directory("robot_bringup")
    bringup_launch = os.path.join(bringup_share, "launch")
    installed_perception = os.path.join(bringup_share, "config", "perception.yaml")
    source_perception = os.path.abspath(
        os.path.join(bringup_share, "../../../../src/robot_bringup/config/perception.yaml")
    )
    default_perception_params = source_perception if os.path.exists(source_perception) else installed_perception
    default_params = os.path.join(parking_share, "config", "arrival_parking_test.yaml")
    installed_tuning = os.path.join(bringup_share, "config", "motion_tuning.yaml")
    source_tuning = os.path.abspath(
        os.path.join(bringup_share, "../../../../src/robot_bringup/config/motion_tuning.yaml")
    )
    default_tuning = source_tuning if os.path.exists(source_tuning) else installed_tuning

    perception_params = LaunchConfiguration("perception_params_file")
    params = LaunchConfiguration("params_file")
    motion_tuning = LaunchConfiguration("motion_tuning_file")
    with_cameras = LaunchConfiguration("with_cameras")
    with_wall_localizer = LaunchConfiguration("with_wall_localizer")
    with_base = LaunchConfiguration("with_base")
    drive_enabled = LaunchConfiguration("drive_enabled")
    armed = LaunchConfiguration("armed")
    auto_start = LaunchConfiguration("auto_start")
    require_deadman = LaunchConfiguration("require_deadman")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_theta = LaunchConfiguration("initial_theta")

    def node(package, executable, name, condition=None, extra=None):
        return Node(
            package=package,
            executable=executable,
            name=name,
            parameters=[perception_params, params, motion_tuning] + ([extra] if extra else []),
            output="screen",
            condition=condition,
        )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch, "cameras.launch.py")),
        condition=IfCondition(with_cameras),
    )
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch, "static_transforms.launch.py"))
    )

    return LaunchDescription([
        DeclareLaunchArgument("perception_params_file", default_value=default_perception_params),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("motion_tuning_file", default_value=default_tuning),
        DeclareLaunchArgument("with_cameras", default_value="true"),
        DeclareLaunchArgument("with_wall_localizer", default_value="true"),
        DeclareLaunchArgument("with_base", default_value="false"),
        DeclareLaunchArgument("drive_enabled", default_value="false"),
        DeclareLaunchArgument("armed", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("require_deadman", default_value="true"),
        DeclareLaunchArgument("initial_x", default_value="-1.8"),
        DeclareLaunchArgument("initial_y", default_value="1.8"),
        DeclareLaunchArgument("initial_theta", default_value="0.0"),
        cameras,
        static_tf,
        node("robot_hardware", "imu_mpu6050_node", "imu_mpu6050_node"),
        node(
            "robot_perception",
            "localizer_node",
            "localizer_node",
            extra={
                "initial_x": ParameterValue(initial_x, value_type=float),
                "initial_y": ParameterValue(initial_y, value_type=float),
                "initial_theta": ParameterValue(initial_theta, value_type=float),
                "use_object_flow": True,
                "use_object_landmarks": True,
                "use_landmark_correction": False,
            },
        ),
        node("robot_perception", "yolo_detector_node", "yolo_detector_node"),
        node("robot_perception", "world_model_node", "world_model_node"),
        node(
            "robot_perception",
            "wall_localizer_node",
            "wall_localizer_node",
            condition=IfCondition(with_wall_localizer),
        ),
        node(
            "robot_parking_test",
            "parking_controller_node",
            "parking_controller_node",
            extra={
                "drive_enabled": ParameterValue(drive_enabled, value_type=bool),
                "armed": ParameterValue(armed, value_type=bool),
                "auto_start": ParameterValue(auto_start, value_type=bool),
                "require_deadman": ParameterValue(require_deadman, value_type=bool),
            },
        ),
        node("robot_control", "base_controller_node", "base_controller_node",
             condition=IfCondition(with_base)),
        node("robot_hardware", "mcu_bridge_base_node", "mcu_bridge_base_node",
             condition=IfCondition(with_base)),
    ])
