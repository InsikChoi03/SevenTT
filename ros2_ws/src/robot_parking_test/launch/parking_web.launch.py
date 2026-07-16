"""Separate web viewer for the arrival parking test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8090"),
        Node(
            package="robot_parking_test",
            executable="parking_web_node",
            name="parking_web_node",
            parameters=[{"port": ParameterValue(port, value_type=int)}],
            output="screen",
        ),
    ])
