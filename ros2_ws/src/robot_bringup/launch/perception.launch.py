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
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "perception.yaml"
    )

    def perc(executable, name):
        return Node(package="robot_perception", executable=executable, name=name,
                    parameters=[params], output="screen")

    def plan(executable, name):
        return Node(package="robot_planning", executable=executable, name=name,
                    parameters=[params], output="screen")

    return LaunchDescription([
        perc("localizer_node", "localizer_node"),
        perc("yolo_detector_node", "yolo_detector_node"),
        perc("siglip_gate_node", "siglip_gate_node"),
        perc("world_model_node", "world_model_node"),
        perc("wall_localizer_node", "wall_localizer_node"),
        perc("recognition_viz_node", "recognition_viz_node"),
        plan("target_selector_node", "target_selector_node"),
        plan("mission_fsm_node", "mission_fsm_node"),
    ])
