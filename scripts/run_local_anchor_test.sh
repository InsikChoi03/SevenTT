#!/usr/bin/env bash
# Independent local-anchor test.  This does NOT launch mission_fsm_node or the arm sequencer.
#
# Safe inspection only (no wheel motion):
#   bash scripts/run_local_anchor_test.sh
# One-command real rotation test (auto deadman + queued START):
#   bash scripts/run_local_anchor_test.sh live
# Then press the Arduino button twice; motion begins when every preflight gate is ready.

set -e -o pipefail

TEST_MODE="${1:-safe}"
if [[ "$TEST_MODE" == "live" ]]; then
  shift
  set -- drive_enabled:=true armed:=true with_arm:=true pick_enabled:=true "$@"
else
  TEST_MODE="safe"
fi

TEST_WS=/home/seventt/seventt/workspace
TEST_RUN_ROOT="$TEST_WS/data/local_anchor_test"
TEST_PID_FILE="$TEST_RUN_ROOT/.launcher.pid"
mkdir -p "$TEST_RUN_ROOT"
source /opt/ros/humble/setup.bash
source "$TEST_WS/ros2_ws/install/setup.bash"
# The Humble workspace currently has a mixed install/symlink layout.  Prefer the
# edited local-test sources so a tuning trial never runs an older installed copy.
export PYTHONPATH="$TEST_WS/ros2_ws/src/robot_perception:$TEST_WS/ros2_ws/src/robot_planning:${PYTHONPATH:-}"
set -u

if [[ -f "$TEST_PID_FILE" ]]; then
  read -r OLD_TEST_PID < "$TEST_PID_FILE" || OLD_TEST_PID=""
  if [[ "$OLD_TEST_PID" =~ ^[0-9]+$ ]] && kill -0 "$OLD_TEST_PID" 2>/dev/null; then
    echo "another local-anchor launcher is active: pid=$OLD_TEST_PID"
    exit 2
  fi
fi

TEST_TS=$(date +%Y%m%d_%H%M%S)
TEST_RUN_DIR="$TEST_RUN_ROOT/run_$TEST_TS"
mkdir -p "$TEST_RUN_DIR"
echo $$ > "$TEST_PID_FILE"

echo "== local-anchor isolated test =="
echo "mission_fsm_node and arm sequencer are intentionally not launched."
echo "mode:    $TEST_MODE"
echo "status: ros2 topic echo /local_anchor_test/status"
echo "web:    http://10.42.0.1:8082/  (main 8080 is untouched)"
if [[ "$TEST_MODE" == "live" ]]; then
  echo "action:  Arduino button twice; auto START waits for RUNNING + camera + YOLO."
else
  echo "start:   safe mode only; pass 'live' to enable wheel motion."
fi
echo "abort:   ros2 topic pub --once /local_anchor_test/control std_msgs/msg/String '{data: ABORT}'"
echo "logs:    $TEST_RUN_DIR"

TEST_BAG_PID=""
TEST_DEADMAN_PID=""
TEST_STARTER_PID=""
if [[ -z "${NOBAG:-}" ]]; then
  ros2 bag record -o "$TEST_RUN_DIR/bag" \
    /local_anchor_test/status /competition/state /localization/pose \
    /imu/data /localization/imu_yaw_delta /localization/motion_mode \
    /world_model/wide_relative_objects /camera_body/detections \
    /classification/siglip /base_command /base/wheel_speeds /base/wheel_odom \
    /arm/pick_trigger /arm2r/target \
    > "$TEST_RUN_DIR/bag.log" 2>&1 &
  TEST_BAG_PID=$!
fi

if [[ "$TEST_MODE" == "live" ]]; then
  ros2 topic pub -r 5 /local_anchor_test/deadman std_msgs/msg/Empty '{}' \
    > "$TEST_RUN_DIR/deadman.log" 2>&1 &
  TEST_DEADMAN_PID=$!
  (
    for _attempt in {1..60}; do
      if ros2 topic info /local_anchor_test/control 2>/dev/null \
        | grep -Eq 'Subscription count: [1-9]'; then
        sleep 1
        ros2 topic pub --once /local_anchor_test/control std_msgs/msg/String \
          '{data: START}' > "$TEST_RUN_DIR/auto_start.log" 2>&1
        exit 0
      fi
      sleep 0.5
    done
    echo "local_anchor_test_node did not appear within 30s" \
      > "$TEST_RUN_DIR/auto_start.log"
    exit 1
  ) &
  TEST_STARTER_PID=$!
fi

cleanup() {
  [[ -n "$TEST_BAG_PID" ]] && kill -INT "$TEST_BAG_PID" 2>/dev/null || true
  [[ -n "$TEST_DEADMAN_PID" ]] && kill "$TEST_DEADMAN_PID" 2>/dev/null || true
  [[ -n "$TEST_STARTER_PID" ]] && kill "$TEST_STARTER_PID" 2>/dev/null || true
  if [[ -f "$TEST_PID_FILE" ]] && [[ "$(<"$TEST_PID_FILE")" == "$$" ]]; then
    rm -f "$TEST_PID_FILE"
  fi
}
trap cleanup INT TERM EXIT

ros2 launch robot_bringup local_anchor_test.launch.py "$@" 2>&1 \
  | tee "$TEST_RUN_DIR/console.log"
