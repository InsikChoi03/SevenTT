"""Mock FIELD TEST bringup (recognition-performance verification, OUTSIDE the arena).

Runs the perception -> world-model(DUAL-CAM FUSION) -> planning -> base-drive loop with test
parameters (config/test_field.yaml): continuous whole-field mapping from t=0, Set1-then-Set2
pick ordering, DRY picks (logged, no arm motion). Adds the explorer (active SCAN roam) and the
recognition_viz node (2D map + YOLO camera panels + decision feed + event log).

Launch args (for a safe / memory-bounded staged startup):
    with_base   (default true)  -- explorer + go_to_goal + base_controller + mcu_bridge_base.
                                   Set false for a STATIONARY run (robot does not move).
    with_fsm    (default true)  -- target selector + mission FSM. Set false when a manual/test
                                   script owns /base/goal_pose.
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

try:
    import yaml
except Exception:  # noqa: BLE001 - route auto-sync is optional
    yaml = None


def _checkpoint_route_overrides(path: str) -> dict:
    """Use anchor_waypoint_test as the single source for the route overlay."""
    if yaml is None or not path or not os.path.exists(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:  # noqa: BLE001 - keep launch robust
        return {}
    node = data.get("anchor_waypoint_test", {})
    params = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
    if not isinstance(params, dict):
        return {}
    out = {}
    if "checkpoint_route_xy" in params:
        out["checkpoint_route_xy"] = [float(v) for v in params["checkpoint_route_xy"]]
    if "checkpoint_route_heading_rad" in params:
        out["checkpoint_route_heading_rad"] = [
            float(v) for v in params["checkpoint_route_heading_rad"]
        ]
    return out


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")
    default_params = os.path.join(bringup_share, "config", "test_field.yaml")
    installed_tuning = os.path.join(bringup_share, "config", "motion_tuning.yaml")
    source_tuning = os.path.abspath(
        os.path.join(bringup_share, "../../../../src/robot_bringup/config/motion_tuning.yaml")
    )
    default_tuning = source_tuning if os.path.exists(source_tuning) else installed_tuning
    route_overlay = _checkpoint_route_overrides(default_tuning)
    params = LaunchConfiguration("params_file")
    motion_tuning = LaunchConfiguration("motion_tuning_file")

    with_base = LaunchConfiguration("with_base")
    with_fsm = LaunchConfiguration("with_fsm")
    with_siglip = LaunchConfiguration("with_siglip")
    with_arm = LaunchConfiguration("with_arm")     # 2R pick_sequencer (real grasp via combined board)
    with_wall_localizer = LaunchConfiguration("with_wall_localizer")
    grid_prior_enabled = LaunchConfiguration("grid_prior_enabled")
    grid_track_lock_enabled = LaunchConfiguration("grid_track_lock_enabled")
    localizer_initial_theta = LaunchConfiguration("localizer_initial_theta")
    goal_kp_lin = LaunchConfiguration("goal_kp_lin")
    goal_max_lin_speed = LaunchConfiguration("goal_max_lin_speed")
    goal_min_lin_speed = LaunchConfiguration("goal_min_lin_speed")
    goal_max_ang_speed = LaunchConfiguration("goal_max_ang_speed")
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec")
    goal_avoid_radius = LaunchConfiguration("goal_avoid_radius")
    goal_avoid_gain = LaunchConfiguration("goal_avoid_gain")
    goal_avoid_goal_skip = LaunchConfiguration("goal_avoid_goal_skip")
    output_dir = LaunchConfiguration("output_dir")
    cam_yaw = LaunchConfiguration("cam_yaw")      # wide-cam mount rotation about base z (0/90/180/270)
    rot180 = LaunchConfiguration("rot180")        # optical-axis flip for the 180-rotated top image

    def node(package, executable, name, condition=None, extra=None):
        return Node(package=package, executable=executable, name=name,
                    parameters=[params, motion_tuning] + ([extra] if extra else []),
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
        DeclareLaunchArgument("with_fsm", default_value="true",
                              description="start target selector + mission FSM (/base/goal_pose owner)"),
        DeclareLaunchArgument("with_siglip", default_value="true",
                              description="start siglip_gate (fruit type); false saves VRAM"),
        DeclareLaunchArgument("with_arm", default_value="false",
                              description="start 2R pick_sequencer (real grasp); needs with_base (shares ttyUSB0)"),
        DeclareLaunchArgument("with_wall_localizer", default_value="true",
                              description="use arena wall/floor lines as absolute pose correction"),
        DeclareLaunchArgument("grid_prior_enabled", default_value="true",
                              description="enable 7x6 object grid soft-prior in world_model"),
        DeclareLaunchArgument("grid_track_lock_enabled", default_value="true",
                              description="lock confirmed game-object tracks to field grid points"),
        DeclareLaunchArgument("localizer_initial_theta", default_value="0.0",
                              description="initial robot heading override for localizer_node"),
        DeclareLaunchArgument("goal_kp_lin", default_value="0.45",
                              description="test-field go_to_goal linear gain override"),
        DeclareLaunchArgument("goal_max_lin_speed", default_value="0.060",
                              description="test-field go_to_goal max linear speed override"),
        DeclareLaunchArgument("goal_min_lin_speed", default_value="0.035",
                              description="test-field go_to_goal minimum moving command override"),
        DeclareLaunchArgument("goal_max_ang_speed", default_value="0.22",
                              description="test-field go_to_goal max angular speed override"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="0.40",
                              description="test-field goal timeout; shorter lets waypoint tests stop cleanly"),
        DeclareLaunchArgument("goal_avoid_radius", default_value="0.38",
                              description="world-model obstacle repel radius for go_to_goal"),
        DeclareLaunchArgument("goal_avoid_gain", default_value="0.16",
                              description="world-model obstacle repel gain for go_to_goal"),
        DeclareLaunchArgument("goal_avoid_goal_skip", default_value="0.12",
                              description="ignore obstacles this close to the goal"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="YAML parameter file for stationary field testing"),
        DeclareLaunchArgument("motion_tuning_file", default_value=default_tuning,
                              description="match tuning override YAML loaded after params_file"),
        DeclareLaunchArgument(
            "output_dir",
            default_value="/home/seventt/seventt/workspace/data/mock_field_test",
            description="recognition_viz run-output base dir (script points it at the run folder)"),
        DeclareLaunchArgument("cam_yaw", default_value="90",
                              description="wide-cam mount yaw (2026-07-02: 90 by physical align; try 0/90/180/270)"),
        DeclareLaunchArgument("rot180", default_value="true",
                              description="wide-cam 180-image optical-axis flip (try true/false)"),
        cameras,
        static_tf,
        # IMU (MPU6050 on i2c-7 0x68): yaw-rate gyro -> localizer heading (drift fix)
        node("robot_hardware", "imu_mpu6050_node", "imu_mpu6050_node"),
        # perception core (always)
        node("robot_perception", "localizer_node", "localizer_node",
             extra={"initial_theta": ParameterValue(localizer_initial_theta, value_type=float)}),
        node("robot_perception", "yolo_detector_node", "yolo_detector_node"),
        node("robot_perception", "world_model_node", "world_model_node",
             extra={"cam_yaw_deg": ParameterValue(cam_yaw, value_type=float),
                    "image_rotated_180": ParameterValue(rot180, value_type=bool),
                    "grid_prior_enabled": ParameterValue(grid_prior_enabled, value_type=bool),
                    "grid_track_lock_enabled": ParameterValue(
                        grid_track_lock_enabled, value_type=bool
                    )}),
        node("robot_perception", "wall_localizer_node", "wall_localizer_node",
             condition=IfCondition(with_wall_localizer),
             extra={"image_rotated_180": ParameterValue(rot180, value_type=bool)}),
        node("robot_perception", "recognition_viz_node", "recognition_viz_node",
             extra={"output_dir": output_dir, **route_overlay}),
        # planning FSM (optional). Disable when a waypoint/test script owns /base/goal_pose.
        node("robot_planning", "target_selector_node", "target_selector_node",
             condition=IfCondition(with_fsm)),
        node("robot_planning", "mission_fsm_node", "mission_fsm_node",
             condition=IfCondition(with_fsm)),
        # SigLIP fruit-type gate (optional, VRAM-heavy)
        node("robot_perception", "siglip_gate_node", "siglip_gate_node",
             condition=IfCondition(with_siglip)),
        # base drive layer (optional; robot moves only with this) -- NO arm nodes ever
        # explorer_node DISABLED: the mission FSM's lane-coverage sweep is now the sole /base/goal_pose
        # writer during SCAN (a second writer would race/fight the planner). Re-enable only if reverting.
        # node("robot_planning", "explorer_node", "explorer_node",
        #      condition=IfCondition(with_base)),
        node("robot_control", "go_to_goal_node", "go_to_goal_node",
             condition=IfCondition(with_base),
             extra={"kp_lin": ParameterValue(goal_kp_lin, value_type=float),
                    "max_lin_speed": ParameterValue(goal_max_lin_speed, value_type=float),
                    "min_lin_speed": ParameterValue(goal_min_lin_speed, value_type=float),
                    "max_ang_speed": ParameterValue(goal_max_ang_speed, value_type=float),
                    "goal_timeout_sec": ParameterValue(goal_timeout_sec, value_type=float),
                    "avoid_radius_m": ParameterValue(goal_avoid_radius, value_type=float),
                    "avoid_gain": ParameterValue(goal_avoid_gain, value_type=float),
                    "avoid_goal_skip_m": ParameterValue(goal_avoid_goal_skip, value_type=float)}),
        node("robot_control", "base_controller_node", "base_controller_node",
             condition=IfCondition(with_base)),
        node("robot_hardware", "mcu_bridge_base_node", "mcu_bridge_base_node",
             condition=IfCondition(with_base)),
        # 2R arm pick sequencer (real grasp). Publishes /arm2r/target -> mcu_bridge_base (ttyUSB0).
        node("robot_control", "pick_sequencer_node", "pick_sequencer_node",
             condition=IfCondition(with_arm)),
    ])
