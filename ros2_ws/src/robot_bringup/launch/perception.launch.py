"""Localization + perception + judgment stack.

Brings up (all loading robot_bringup/config/perception.yaml):
    localizer_node          (robot_perception)  -> /localization/pose
    yolo_detector_node      (robot_perception)  -> /camera_{top,body}/detections + /classification/shape
    siglip_gate_node        (robot_perception)  -> /classification/siglip
    world_model_node        (robot_perception)  -> /world_model
    wall_localizer_node     (robot_perception)  -> /localization/wall_segments + corrections
    recognition_viz_node    (robot_perception)  -> live map + camera panels
    target_selector_node    (robot_planning)    -> /selected_target
    mission_fsm_node        (robot_planning)    -> /mission_state, /base/goal_pose, /arm/pick_trigger

Cameras are launched separately (cameras.launch.py); this stack subscribes to their topics.
Run cameras + this together via bringup.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    default_params = os.path.join(bringup_share, "config", "perception.yaml")
    installed_tuning = os.path.join(bringup_share, "config", "motion_tuning.yaml")
    source_tuning = os.path.abspath(
        os.path.join(bringup_share, "../../../../src/robot_bringup/config/motion_tuning.yaml")
    )
    default_tuning = source_tuning if os.path.exists(source_tuning) else installed_tuning
    params = LaunchConfiguration("params_file")
    motion_tuning = LaunchConfiguration("motion_tuning_file")

    def perc(executable, name):
        return Node(package="robot_perception", executable=executable, name=name,
                    parameters=[params, motion_tuning], output="screen")

    def plan(executable, name):
        return Node(package="robot_planning", executable=executable, name=name,
                    parameters=[params, motion_tuning], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="base ROS parameter YAML"),
        DeclareLaunchArgument("motion_tuning_file", default_value=default_tuning,
                              description="match tuning override YAML loaded after params_file"),
        perc("localizer_node", "localizer_node"),
        perc("yolo_detector_node", "yolo_detector_node"),
        perc("siglip_gate_node", "siglip_gate_node"),
        perc("world_model_node", "world_model_node"),
        perc("wall_localizer_node", "wall_localizer_node"),
        perc("recognition_viz_node", "recognition_viz_node"),
        plan("target_selector_node", "target_selector_node"),
        plan("mission_fsm_node", "mission_fsm_node"),
    ])
