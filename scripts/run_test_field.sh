#!/usr/bin/env bash
# Mock field test launcher (경기장 밖 인식 검증) — 터미널 단독 실행 + 디버깅 로그 전체 기록.
#
# 사용:
#   bash scripts/run_test_field.sh                      # 풀 런 (로밍+접근 + SigLIP 과일종류)
#   bash scripts/run_test_field.sh with_siglip:=false   # SET1(도형)만 — 메모리 빠듯할 때
#   bash scripts/run_test_field.sh with_base:=false      # 정지 — 인식/맵만
#   NOBAG=1 bash scripts/run_test_field.sh               # rosbag 녹화 끔
#   WITHIMG=1 bash scripts/run_test_field.sh             # rosbag에 카메라 영상까지(용량 큼)
# 종료: 이 터미널에서 Ctrl-C  (모든 노드 + rosbag 정리됨)
#
# 남는 것 (한 폴더 data/mock_field_test/run_<시각>/):
#   console.log        — 전 노드 콘솔 출력(픽/전이/siglip점수/랜드마크보정/에러 등)
#   bag/               — 전 토픽 rosbag (ros2 bag play 로 리플레이/분석)
#   <시각>/events.csv,jsonl, map_*.png, live.png — 인식 이벤트 + 2D 맵
#
# 타겟: config/test_field.yaml (SET1=icosahedron×3, SET2=orange×2).

# ROS 2 setup scripts probe optional variables before defining them, so nounset must be enabled
# only after both environments have been sourced.
set -e -o pipefail

WS=/home/seventt/seventt/workspace
RUN_ROOT="$WS/data/mock_field_test"
PID_FILE="$RUN_ROOT/.main_run.pid"
mkdir -p "$RUN_ROOT"
source /opt/ros/humble/setup.bash
source "$WS/ros2_ws/install/setup.bash"
set -u

echo "== preflight =="
# Stop the prior detached launch process first. Killing only its camera children lets ros2 launch
# respawn them, which races the next CSI preflight for the Argus CaptureSession.
launch_target_alive() {
  local pid="$1"
  local mode="${2:-auto}"
  local stat=""
  if [[ "$mode" == "group" ]]; then
    ps -eo pgid=,stat= | awk -v pgid="$pid" \
      '$1 == pgid && $2 !~ /^Z/ { alive=1 } END { exit(alive ? 0 : 1) }'
    return
  fi
  stat=$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)
  [[ -n "$stat" && "$stat" != Z* ]]
}

stop_launch_pid() {
  local pid="$1"
  local mode="${2:-auto}"
  local pgid=""
  local target="$pid"
  if [[ "$mode" == "group" ]]; then
    target="-$pid"
  else
    if ! launch_target_alive "$pid"; then
      return 0
    fi
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)
    if [[ "$pgid" == "$pid" ]]; then
      target="-$pgid"
      mode="group"
    fi
  fi
  if ! launch_target_alive "$pid" "$mode"; then
    return 0
  fi
  kill -INT -- "$target" 2>/dev/null || true
  for _ in {1..10}; do
    launch_target_alive "$pid" "$mode" || return 0
    sleep 0.1
  done
  kill -TERM -- "$target" 2>/dev/null || true
  for _ in {1..10}; do
    launch_target_alive "$pid" "$mode" || return 0
    sleep 0.1
  done
  kill -KILL -- "$target" 2>/dev/null || true
  for _ in {1..10}; do
    launch_target_alive "$pid" "$mode" || return 0
    sleep 0.1
  done
  echo "ERROR: launch target still alive after KILL: $target" >&2
  return 1
}

stop_actuator_bridge() {
  local pattern="$WS/ros2_ws/install/robot_hardware/lib/robot_hardware/mcu_bridge_base_node"
  local pids=""
  pids=$(pgrep -f "$pattern" 2>/dev/null || true)
  [[ -n "$pids" ]] || return 0

  # Give the bridge a short graceful window so destroy_node() sends <STOP> before serial closes.
  kill -INT $pids 2>/dev/null || true
  for _ in {1..5}; do
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    [[ -z "$pids" ]] && return 0
    sleep 0.1
  done
  kill -TERM $pids 2>/dev/null || true
  for _ in {1..5}; do
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    [[ -z "$pids" ]] && return 0
    sleep 0.1
  done
  kill -KILL $pids 2>/dev/null || true
}

# Stop the exact prior launcher first when its PID file is valid. New runs store the actual
# ros2-launch PID; the run_test_field.sh check retains compatibility with older PID files.
if [[ -f "$PID_FILE" ]]; then
  OLD_PID=""
  OLD_MODE=""
  read -r OLD_PID OLD_MODE < "$PID_FILE" || OLD_PID=""
  if [[ "$OLD_PID" =~ ^[0-9]+$ ]] && [[ "$OLD_MODE" == "group" ]] \
      && launch_target_alive "$OLD_PID" group; then
    stop_launch_pid "$OLD_PID" group
    echo "  stopped previous launch group pgid=$OLD_PID"
  elif [[ "$OLD_PID" =~ ^[0-9]+$ ]] && kill -0 "$OLD_PID" 2>/dev/null; then
    OLD_CMD=$(tr '\0' ' ' < "/proc/$OLD_PID/cmdline" 2>/dev/null || true)
    if [[ "$OLD_CMD" == *"run_test_field.sh"* || "$OLD_CMD" == *"ros2 launch robot_bringup test_field.launch.py"* ]]; then
      stop_launch_pid "$OLD_PID"
      echo "  stopped previous launcher pid=$OLD_PID"
    fi
  fi
fi

# Older runs wrote the shell PID and can leave a ros2 launch parent orphaned. Stop every
# remaining test-field parent before touching child nodes so it cannot respawn camera_csi_node.
while read -r OLD_LAUNCH_PID; do
  [[ -n "$OLD_LAUNCH_PID" ]] || continue
  stop_launch_pid "$OLD_LAUNCH_PID"
  echo "  stopped stale ros2 launch pid=$OLD_LAUNCH_PID"
done < <(pgrep -f '[r]os2 launch robot_bringup test_field.launch.py' || true)

for p in "install/robot_perception/lib" "install/robot_planning/lib" \
         "install/robot_control/lib" "install/robot_hardware/lib" "camera_csi_node" "ros2 bag record"; do
  if pkill -f "$p" 2>/dev/null; then
    echo "  killed stale: $p"
  fi
done
# Argus releases CSI sessions asynchronously after the last camera process exits.
sleep 3
CAMERA_READY=0
for ATTEMPT in 1 2 3; do
  if python3 "$WS/scripts/competition_preflight.py" \
      --serial /dev/ttyUSB0 --wide-sensor-id 1 --camera-timeout-sec 8; then
    CAMERA_READY=1
    break
  fi
  if [[ "$ATTEMPT" -lt 3 ]]; then
    echo "  wide camera release pending; retrying preflight in 3s ($ATTEMPT/3)"
    sleep 3
  fi
done
if [[ "$CAMERA_READY" -ne 1 ]]; then
  exit 1
fi
sleep 1
free -m | awk '/Mem/{print "  mem available="$7"MB  (SigLIP은 ~1.5GB 여유 필요 — 빠듯하면 크롬/Colab 끄거나 with_siglip:=false)"}'

TS=$(date +%Y%m%d_%H%M%S)
RUN_DIR="$RUN_ROOT/run_$TS"
mkdir -p "$RUN_DIR"
echo "$$ runner" > "$PID_FILE"
echo "== run dir: $RUN_DIR =="

# ---- rosbag (백그라운드), Ctrl-C 시 함께 정리 ----
BAG_PID=""
CLEANUP_DONE=0
if [[ -z "${NOBAG:-}" ]]; then
  TOPICS="/competition/state /world_model /localization/pose /localization/landmark_correction \
/camera_top/detections /camera_body/detections /classification/siglip /classification/shape \
/selected_target /mission_state /planning/phase /planning/decision \
/base/goal_pose /base_command /base/wheel_speeds /base/wheel_odom"
  [[ -n "${WITHIMG:-}" ]] && TOPICS="$TOPICS /camera_top/image_raw /camera_body/image_raw"
  ros2 bag record -o "$RUN_DIR/bag" $TOPICS > "$RUN_DIR/bag.log" 2>&1 &
  BAG_PID=$!
  echo "== rosbag recording (pid $BAG_PID) -> $RUN_DIR/bag =="
fi
cleanup() {
  if [[ "$CLEANUP_DONE" -eq 1 ]]; then
    return 0
  fi
  # A second terminal signal must not interrupt the stop sequence halfway through.
  trap '' INT TERM
  local shutdown_incomplete=0
  local remaining=""
  echo "== shutdown: stopping actuators and test-field processes =="
  stop_actuator_bridge
  [[ -n "$BAG_PID" ]] && kill -INT "$BAG_PID" 2>/dev/null || true

  # Ctrl-C reaches the foreground ros2 launch and all of its children directly. This fallback
  # only handles a launch parent that did not finish its own shutdown within the signal window.
  while read -r remaining; do
    [[ -n "$remaining" ]] || continue
    stop_launch_pid "$remaining" || true
  done < <(pgrep -f '[r]os2 launch robot_bringup test_field.launch.py' || true)

  remaining=$(pgrep -f \
    'test_field.launch.py|install/robot_(perception|planning|control|hardware)/lib|camera_csi_node|ros2 bag record' \
    2>/dev/null || true)
  if [[ -n "$remaining" ]]; then
    kill -TERM $remaining 2>/dev/null || true
    sleep 0.5
    remaining=$(pgrep -f \
      'test_field.launch.py|install/robot_(perception|planning|control|hardware)/lib|camera_csi_node|ros2 bag record' \
      2>/dev/null || true)
  fi
  if [[ -n "$remaining" ]]; then
    kill -KILL $remaining 2>/dev/null || true
    sleep 0.2
    remaining=$(pgrep -f \
      'test_field.launch.py|install/robot_(perception|planning|control|hardware)/lib|camera_csi_node|ros2 bag record' \
      2>/dev/null || true)
  fi
  [[ -z "$remaining" ]] || shutdown_incomplete=1

  if [[ "$shutdown_incomplete" -eq 0 ]] && [[ -f "$PID_FILE" ]] \
      && [[ "$(<"$PID_FILE")" == "$$ runner" ]]; then
    rm -f "$PID_FILE"
  fi
  CLEANUP_DONE=1
  if [[ "$shutdown_incomplete" -ne 0 ]]; then
    echo "ERROR: shutdown incomplete; keeping $PID_FILE for the next preflight" >&2
    return 1
  fi
}
trap cleanup INT TERM EXIT

echo "== main live web: http://127.0.0.1:8080/ (AP: http://10.42.0.1:8080/) =="
echo "== launch (all node logs -> $RUN_DIR/console.log) =="
echo "   fresh process state: STANDBY; second accepted button press establishes competition t=0."
echo "   라이브 창이 뜨거나(디스플레이 있으면), 헤드리스면 $RUN_DIR/<시각>/live.png 가 계속 갱신됨."
# Keep ros2 launch in this terminal's foreground process group. Ctrl-C then reaches launch,
# camera, planning, control, hardware and the log tee at the same time, as it did originally.
set +e
ros2 launch robot_bringup test_field.launch.py output_dir:="$RUN_DIR" "$@" \
  > >(tee "$RUN_DIR/console.log") 2>&1
LAUNCH_RC=$?
set -e
cleanup
exit "$LAUNCH_RC"
