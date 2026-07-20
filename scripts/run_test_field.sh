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
# Stop the exact prior launcher first when its PID file is valid. Guard against PID reuse by
# checking the command line before sending a signal.
if [[ -f "$PID_FILE" ]]; then
  read -r OLD_PID < "$PID_FILE" || OLD_PID=""
  if [[ "$OLD_PID" =~ ^[0-9]+$ ]] && kill -0 "$OLD_PID" 2>/dev/null; then
    OLD_CMD=$(tr '\0' ' ' < "/proc/$OLD_PID/cmdline" 2>/dev/null || true)
    if [[ "$OLD_CMD" == *"run_test_field.sh"* ]]; then
      kill -TERM "$OLD_PID" 2>/dev/null || true
      for _ in {1..25}; do
        kill -0 "$OLD_PID" 2>/dev/null || break
        sleep 0.2
      done
      if kill -0 "$OLD_PID" 2>/dev/null; then
        kill -KILL "$OLD_PID" 2>/dev/null || true
      fi
      echo "  stopped previous launcher pid=$OLD_PID"
    fi
  fi
fi

for p in "install/robot_perception/lib" "install/robot_planning/lib" \
         "install/robot_control/lib" "install/robot_hardware/lib" "camera_csi_node" "ros2 bag record"; do
  if pkill -f "$p" 2>/dev/null; then
    echo "  killed stale: $p"
  fi
done
sleep 1
python3 "$WS/scripts/competition_preflight.py" \
  --serial /dev/ttyUSB0 --wide-sensor-id 1 --camera-timeout-sec 8
sleep 1
free -m | awk '/Mem/{print "  mem available="$7"MB  (SigLIP은 ~1.5GB 여유 필요 — 빠듯하면 크롬/Colab 끄거나 with_siglip:=false)"}'

TS=$(date +%Y%m%d_%H%M%S)
RUN_DIR="$RUN_ROOT/run_$TS"
mkdir -p "$RUN_DIR"
echo $$ > "$PID_FILE"
echo "== run dir: $RUN_DIR =="

# ---- rosbag (백그라운드), Ctrl-C 시 함께 정리 ----
BAG_PID=""
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
  [[ -n "$BAG_PID" ]] && kill -INT "$BAG_PID" 2>/dev/null || true
  if [[ -f "$PID_FILE" ]] && [[ "$(<"$PID_FILE")" == "$$" ]]; then
    rm -f "$PID_FILE"
  fi
}
trap cleanup INT TERM EXIT

echo "== launch (all node logs -> $RUN_DIR/console.log) =="
echo "   fresh process state: STANDBY; second accepted button press establishes competition t=0."
echo "   라이브 창이 뜨거나(디스플레이 있으면), 헤드리스면 $RUN_DIR/<시각>/live.png 가 계속 갱신됨."
ros2 launch robot_bringup test_field.launch.py output_dir:="$RUN_DIR" "$@" 2>&1 | tee "$RUN_DIR/console.log"
