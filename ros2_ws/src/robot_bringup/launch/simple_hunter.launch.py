"""Simple-hunter bringup: full stack but with simple_hunter_node replacing the
mission FSM (+ target_selector, which the hunter does not use).

Same perception/control layers as bringup.launch.py; only the judgment node differs:
    cameras + static tf + imu
    localizer / yolo_detector / siglip_gate / world_model / wall_localizer / viz
    simple_hunter_node          (robot_planning)  -> /mission_state, /base_command, /arm/pick_trigger
    base_controller + pick_sequencer + mcu_bridge_base   (with_control:=true)

Usage:
    ros2 launch robot_bringup simple_hunter.launch.py
    ros2 launch robot_bringup simple_hunter.launch.py with_control:=false   # bench, no motors
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")
    params = os.path.join(bringup_share, "config", "perception.yaml")
    default_tuning = os.path.join(bringup_share, "config", "motion_tuning.yaml")

    with_control = LaunchConfiguration("with_control")
    motion_tuning = LaunchConfiguration("motion_tuning_file")

    def perc(executable, name):
        return Node(package="robot_perception", executable=executable, name=name,
                    parameters=[params, motion_tuning], output="screen")

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "cameras.launch.py"))
    )
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "static_transforms.launch.py"))
    )
    imu = Node(package="robot_hardware", executable="imu_mpu6050_node",
               name="imu_mpu6050_node", parameters=[params, motion_tuning], output="screen")

    control_layer = GroupAction(
        condition=IfCondition(with_control),
        actions=[
            Node(package="robot_control", executable="base_controller_node",
                 name="base_controller_node", parameters=[params, motion_tuning], output="screen"),
            Node(package="robot_control", executable="pick_sequencer_node",
                 name="pick_sequencer_node", parameters=[params, motion_tuning], output="screen"),
            Node(package="robot_hardware", executable="mcu_bridge_base_node",
                 name="mcu_bridge_base_node", parameters=[params, motion_tuning], output="screen"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("with_control", default_value="true",
                              description="also start base control + 2R pick sequencer + combined MCU bridge"),
        DeclareLaunchArgument("motion_tuning_file", default_value=default_tuning,
                              description="match tuning override YAML loaded after perception.yaml"),
        cameras,
        static_tf,
        imu,
        perc("localizer_node", "localizer_node"),
        perc("yolo_detector_node", "yolo_detector_node"),
        perc("siglip_gate_node", "siglip_gate_node"),
        perc("world_model_node", "world_model_node"),
        perc("wall_localizer_node", "wall_localizer_node"),
        perc("recognition_viz_node", "recognition_viz_node"),
        Node(package="robot_planning", executable="simple_hunter_node",
             name="simple_hunter_node", parameters=[params, motion_tuning], output="screen"),
        control_layer,
    ])
