"""Full system bringup: cameras + static tf + localization/perception/judgment + control + MCU bridges.

Composed of the smaller launch files so each layer can also be run on its own:
    cameras.launch.py            two CSI cameras (top/wide=sensor-id 1, body=sensor-id 0)
    static_transforms.launch.py  rig tf frames
    perception.launch.py         localizer + perception + FSM/target selector
    + control nodes and MCU serial bridge

Toggle the control/hardware-bridge layer with the 'with_control' launch arg (default true).
On a bench with no MCU/cameras every node still starts (dry-run safe) — sensors/serial just
stay idle.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")
    params = os.path.join(bringup_share, "config", "perception.yaml")

    with_control = LaunchConfiguration("with_control")

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "cameras.launch.py"))
    )
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "static_transforms.launch.py"))
    )
    imu = Node(package="robot_hardware", executable="imu_mpu6050_node",
               name="imu_mpu6050_node", parameters=[params], output="screen")
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "perception.launch.py"))
    )

    control_layer = GroupAction(
        condition=IfCondition(with_control),
        actions=[
            Node(package="robot_control", executable="go_to_goal_node",
                 name="go_to_goal_node", parameters=[params], output="screen"),
            Node(package="robot_control", executable="base_controller_node",
                 name="base_controller_node", parameters=[params], output="screen"),
            Node(package="robot_control", executable="pick_sequencer_node",
                 name="pick_sequencer_node", parameters=[params], output="screen"),
            Node(package="robot_hardware", executable="mcu_bridge_base_node",
                 name="mcu_bridge_base_node", parameters=[params], output="screen"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("with_control", default_value="true",
                              description="also start base control + 2R pick sequencer + combined MCU bridge"),
        cameras,
        static_tf,
        imu,
        perception,
        control_layer,
    ])
