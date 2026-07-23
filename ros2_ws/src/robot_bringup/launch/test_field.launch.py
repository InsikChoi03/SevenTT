"""Mock FIELD TEST bringup (recognition-performance verification, OUTSIDE the arena).

Runs the perception -> world-model(DUAL-CAM FUSION) -> planning -> base-drive loop with test
parameters (config/test_field.yaml): continuous whole-field mapping from t=0, Set1-then-Set2
pick ordering, DRY picks (logged, no arm motion). Adds the explorer (active SCAN roam) and the
recognition_viz node (2D map + YOLO camera panels + decision feed + event log).

Launch args (for a safe / memory-bounded staged startup):
    with_base   (default true)  -- FSM + base_controller + mcu_bridge_base.
                                   Set false for a STATIONARY run (robot does not move).
    with_fsm    (default true)  -- target selector + mission FSM. Set false when a manual/test
                                   script owns base motion.
    with_siglip (default true)  -- siglip_gate (fruit TYPE). Set false to save ~1.5 GB VRAM;
                                   YOLO still separates Set1 vs Set2 (fruit_photo_cube class).
    with_arm    (default true)  -- real 2R pick sequencer through the shared base MCU bridge.

    ros2 launch robot_bringup test_field.launch.py                              # full run
    ros2 launch robot_bringup test_field.launch.py with_base:=false with_siglip:=false  # core smoke

The legacy 6-DOF arm nodes are not launched. The verified 2R pick sequencer is enabled by default;
the MCU bridge still discards every arm target until competition state RUNNING.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, OrSubstitution
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


def _yaml_bool_parameter(
    path: str,
    node_name: str,
    parameter_name: str,
    default: bool = False,
) -> bool:
    """Read one boolean used to decide which optional nodes the launch must start."""
    if yaml is None or not path or not os.path.exists(path):
        return bool(default)
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        value = data[node_name]["ros__parameters"][parameter_name]
    except (OSError, TypeError, KeyError, yaml.YAMLError):
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
    return bool(default)


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")
    default_params = os.path.join(bringup_share, "config", "test_field.yaml")
    default_tuning = os.path.join(bringup_share, "config", "motion_tuning.yaml")
    route_overlay = _checkpoint_route_overrides(default_tuning)
    storage_wall_guided_default = _yaml_bool_parameter(
        default_tuning,
        "mission_fsm_node",
        "storage_wall_guided_enabled",
        False,
    )
    params = LaunchConfiguration("params_file")
    motion_tuning = LaunchConfiguration("motion_tuning_file")

    with_base = LaunchConfiguration("with_base")
    with_fsm = LaunchConfiguration("with_fsm")
    with_siglip = LaunchConfiguration("with_siglip")
    with_arm = LaunchConfiguration("with_arm")     # 2R pick_sequencer (real grasp via combined board)
    with_wall_localizer = LaunchConfiguration("with_wall_localizer")
    storage_wall_guided_enabled = LaunchConfiguration(
        "storage_wall_guided_enabled"
    )
    grid_prior_enabled = LaunchConfiguration("grid_prior_enabled")
    grid_track_lock_enabled = LaunchConfiguration("grid_track_lock_enabled")
    localizer_initial_theta = LaunchConfiguration("localizer_initial_theta")
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
                              description="start base drive (FSM + base controller + bridge)"),
        DeclareLaunchArgument("with_fsm", default_value="true",
                              description="start target selector + mission FSM (/base_command owner)"),
        DeclareLaunchArgument("with_siglip", default_value="true",
                              description="start siglip_gate (fruit type); false saves VRAM"),
        DeclareLaunchArgument("with_arm", default_value="true",
                              description="start 2R pick_sequencer (real grasp); needs with_base (shares ttyUSB0)"),
        DeclareLaunchArgument("with_wall_localizer", default_value="false",
                              description="use arena wall/floor lines as absolute pose correction"),
        DeclareLaunchArgument(
            "storage_wall_guided_enabled",
            default_value="true" if storage_wall_guided_default else "false",
            description=(
                "single storage-mode switch: start wall perception and use measured wall "
                "distances for the final storage return"
            ),
        ),
        DeclareLaunchArgument("grid_prior_enabled", default_value="true",
                              description="enable 7x6 object grid soft-prior in world_model"),
        DeclareLaunchArgument("grid_track_lock_enabled", default_value="true",
                              description="lock confirmed game-object tracks to field grid points"),
        DeclareLaunchArgument(
            "localizer_initial_theta",
            default_value="0.0",
            description="initial robot heading: 0 rad points toward waypoint 1 along field +x",
        ),
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
             condition=IfCondition(
                 OrSubstitution(with_wall_localizer, storage_wall_guided_enabled)
             ),
             extra={"image_rotated_180": ParameterValue(rot180, value_type=bool)}),
        node("robot_perception", "recognition_viz_node", "recognition_viz_node",
             extra={"output_dir": output_dir, **route_overlay}),
        # planning FSM (optional). Disable when a waypoint/test script owns base motion.
        node("robot_planning", "target_selector_node", "target_selector_node",
             condition=IfCondition(with_fsm)),
        node("robot_planning", "mission_fsm_node", "mission_fsm_node",
             condition=IfCondition(with_fsm),
             extra={
                 "storage_wall_guided_enabled": ParameterValue(
                     storage_wall_guided_enabled, value_type=bool
                 )
             }),
        # SigLIP fruit-type gate (optional, VRAM-heavy)
        node("robot_perception", "siglip_gate_node", "siglip_gate_node",
             condition=IfCondition(with_siglip)),
        # base drive layer (optional; robot moves only with this)
        # The mission FSM is the sole /base_command publisher in the field-test pipeline.
        # node("robot_planning", "explorer_node", "explorer_node",
        #      condition=IfCondition(with_base)),
        node("robot_control", "base_controller_node", "base_controller_node",
             condition=IfCondition(with_base)),
        node("robot_hardware", "mcu_bridge_base_node", "mcu_bridge_base_node",
             condition=IfCondition(with_base)),
        # 2R arm pick sequencer (real grasp). Publishes /arm2r/target -> mcu_bridge_base (ttyUSB0).
        node("robot_control", "pick_sequencer_node", "pick_sequencer_node",
             condition=IfCondition(with_arm)),
    ])
