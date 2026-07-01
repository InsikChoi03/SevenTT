"""Mock FIELD TEST bringup (recognition-performance verification, OUTSIDE the arena).

Runs the perception -> world-model(DUAL-CAM FUSION) -> planning -> base-drive loop with test
parameters (config/test_field.yaml): continuous whole-field mapping from t=0, Set1-then-Set2
pick ordering, DRY picks (logged, no arm motion). Adds the explorer (active SCAN roam) and the
recognition_viz node (2D map + YOLO camera panels + decision feed + event log).

Launch args (for a safe / memory-bounded staged startup):
    with_base   (default true)  -- explorer + go_to_goal + base_controller + mcu_bridge_base.
                                   Set false for a STATIONARY run (robot does not move).
    with_siglip (default true)  -- siglip_gate (fruit TYPE). Set false to save ~1.5 GB VRAM;
                                   YOLO still separates Set1 vs Set2 (fruit_photo_cube class).

    ros2 launch robot_bringup test_field.launch.py                              # full run
    ros2 launch robot_bringup test_field.launch.py with_base:=false with_siglip:=false  # core smoke

The ARM IS NEVER USED: pick_sequencer_node, arm_controller_node, mcu_bridge_arm_node are not
launched, and mission_fsm runs dry_pick:true. Fill set1_label/set2_label/quotas in the yaml.
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
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")
    params = os.path.join(bringup_share, "config", "test_field.yaml")

    with_base = LaunchConfiguration("with_base")
    with_siglip = LaunchConfiguration("with_siglip")
    output_dir = LaunchConfiguration("output_dir")
    cam_yaw = LaunchConfiguration("cam_yaw")      # wide-cam mount rotation about base z (0/90/180/270)
    rot180 = LaunchConfiguration("rot180")        # optical-axis flip for the 180-rotated top image

    def node(package, executable, name, condition=None, extra=None):
        return Node(package=package, executable=executable, name=name,
                    parameters=[params] + ([extra] if extra else []),
                    output="screen", condition=condition)

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "cameras.launch.py"))
    )
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "static_transforms.launch.py"))
    )

    return LaunchDescription([
        DeclareLaunchArgument("with_base", default_value="true",
                              description="start base drive (explorer + go_to_goal + base + bridge)"),
        DeclareLaunchArgument("with_siglip", default_value="true",
                              description="start siglip_gate (fruit type); false saves VRAM"),
        DeclareLaunchArgument(
            "output_dir",
            default_value="/home/seventt/seventt/workspace/data/mock_field_test",
            description="recognition_viz run-output base dir (script points it at the run folder)"),
        DeclareLaunchArgument("cam_yaw", default_value="180",
                              description="wide-cam mount yaw override (try 0/90/180/270 to fix front/left/right)"),
        DeclareLaunchArgument("rot180", default_value="true",
                              description="wide-cam 180-image optical-axis flip (try true/false)"),
        cameras,
        static_tf,
        # perception core (always)
        node("robot_perception", "localizer_node", "localizer_node"),
        node("robot_perception", "yolo_detector_node", "yolo_detector_node"),
        node("robot_perception", "world_model_node", "world_model_node",
             extra={"cam_yaw_deg": ParameterValue(cam_yaw, value_type=float),
                    "image_rotated_180": ParameterValue(rot180, value_type=bool)}),
        node("robot_perception", "recognition_viz_node", "recognition_viz_node",
             extra={"output_dir": output_dir}),
        # planning (always) -- selector + FSM. explorer is under with_base (it drives the base).
        node("robot_planning", "target_selector_node", "target_selector_node"),
        node("robot_planning", "mission_fsm_node", "mission_fsm_node"),
        # SigLIP fruit-type gate (optional, VRAM-heavy)
        node("robot_perception", "siglip_gate_node", "siglip_gate_node",
             condition=IfCondition(with_siglip)),
        # base drive layer (optional; robot moves only with this) -- NO arm nodes ever
        node("robot_planning", "explorer_node", "explorer_node",
             condition=IfCondition(with_base)),
        node("robot_control", "go_to_goal_node", "go_to_goal_node",
             condition=IfCondition(with_base)),
        node("robot_control", "base_controller_node", "base_controller_node",
             condition=IfCondition(with_base)),
        node("robot_hardware", "mcu_bridge_base_node", "mcu_bridge_base_node",
             condition=IfCondition(with_base)),
    ])
