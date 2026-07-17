#!/usr/bin/env bash
set -euo pipefail

# One-terminal stationary wall projection test.
#
# Starts only the topics needed by wall_boundary_base_debug.py:
#   /camera_top/image_raw
#   /localization/pose            (fixed fake pose)
#   /localization/is_stationary   (true)
#   /localization/wall_raw_segments
#   /localization/wall_mask_segments_image
#
# Stop other camera/perception/bringup launches before running this script.

WORKSPACE=${WORKSPACE:-/home/seventt/seventt/workspace}
ROS_WS=${ROS_WS:-"$WORKSPACE/ros2_ws"}
PARAMS_FILE=${PARAMS_FILE:-"$ROS_WS/src/robot_bringup/config/perception.yaml"}
MOTION_TUNING_FILE=${MOTION_TUNING_FILE:-"$ROS_WS/src/robot_bringup/config/motion_tuning.yaml"}

POSE_X=${POSE_X:--1.3}
POSE_Y=${POSE_Y:-1.3}
POSE_YAW_DEG=${POSE_YAW_DEG:-0.0}
EXPECTED_X=${EXPECTED_X:-0.50}
TOLERANCE=${TOLERANCE:-0.15}
PRINT_PERIOD=${PRINT_PERIOD:-1.0}

PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

cd "$WORKSPACE"
echo "[wall-debug] workspace=$WORKSPACE"
echo "[wall-debug] fixed pose field=($POSE_X,$POSE_Y), yaw=${POSE_YAW_DEG}deg"
echo "[wall-debug] expected wall base x=${EXPECTED_X}m"
echo "[wall-debug] params=$PARAMS_FILE"
echo "[wall-debug] tuning=$MOTION_TUNING_FILE"

python3 - <<'PY' "$POSE_YAW_DEG" > /tmp/wall_debug_pose_quat.txt
import math
import sys

yaw = math.radians(float(sys.argv[1]))
print(f"{math.sin(yaw * 0.5):.12f} {math.cos(yaw * 0.5):.12f}")
PY
read -r POSE_QZ POSE_QW < /tmp/wall_debug_pose_quat.txt

echo "[wall-debug] starting top camera only..."
ros2 run robot_hardware camera_csi_node --ros-args \
  -r __node:=camera_top \
  -p sensor_id:=1 \
  -p frame_id:=camera_top \
  -p topic:=/camera_top/image_raw \
  -p sensor_mode:=3 \
  -p width:=1640 \
  -p height:=1232 \
  -p fps:=30 \
  -p publish_rate:=30.0 \
  -p wbmode:=8 \
  -p flip_method:=2 \
  -p out_width:=1280 \
  -p out_height:=960 &
PIDS+=("$!")

sleep 2

echo "[wall-debug] publishing fixed /localization/pose..."
ros2 topic pub /localization/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'field'}, pose: {position: {x: ${POSE_X}, y: ${POSE_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${POSE_QZ}, w: ${POSE_QW}}}}" \
  -r 10 >/tmp/wall_debug_pose_pub.log 2>&1 &
PIDS+=("$!")

echo "[wall-debug] publishing /localization/is_stationary=true..."
ros2 topic pub /localization/is_stationary std_msgs/msg/Bool "{data: true}" \
  -r 5 >/tmp/wall_debug_stationary_pub.log 2>&1 &
PIDS+=("$!")

sleep 1

echo "[wall-debug] starting wall_localizer_node..."
ros2 run robot_perception wall_localizer_node --ros-args \
  --params-file "$PARAMS_FILE" \
  --params-file "$MOTION_TUNING_FILE" &
PIDS+=("$!")

sleep 4

echo "[wall-debug] running base projection debug. Ctrl+C to stop all test processes."
python3 scripts/wall_boundary_base_debug.py \
  --expected-x "$EXPECTED_X" \
  --tolerance "$TOLERANCE" \
  --print-period "$PRINT_PERIOD" \
  --target-field-x "$POSE_X" \
  --target-field-y "$POSE_Y"
