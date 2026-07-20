#!/usr/bin/env bash
# ROS 2 Humble's generated setup scripts read optional variables before defining
# them, so nounset (-u) must not be enabled while sourcing the environment.
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_WS="$TEST_ROOT/ros2_ws"
RUN_ROOT="$TEST_ROOT/data/shape4_test"
ROS_LOG_ROOT="$TEST_ROOT/data/ros_logs"
PID_FILE="/tmp/shape4_v1_test.launch.pid"

# Safe defaults: live map + both cameras, with no physical base or arm motion.
# Override explicitly for a driving test, for example:
#   SHAPE4_WITH_BASE=true bash scripts/run_shape4_v1_test.sh
WITH_BASE="${SHAPE4_WITH_BASE:-false}"
WITH_ARM="${SHAPE4_WITH_ARM:-false}"
WITH_SIGLIP="${SHAPE4_WITH_SIGLIP:-false}"
WITH_WALL="${SHAPE4_WITH_WALL:-false}"
BASE_PORT="${SHAPE4_BASE_PORT:-/dev/ttyUSB0}"

mkdir -p "$RUN_ROOT" "$ROS_LOG_ROOT"

stop_previous_run() {
    if [[ ! -r "$PID_FILE" ]]; then
        return
    fi

    local old_pid old_cmd
    old_pid="$(tr -cd '0-9' < "$PID_FILE")"
    if [[ -z "$old_pid" || ! -r "/proc/$old_pid/cmdline" ]]; then
        printf '0\n' > "$PID_FILE"
        return
    fi

    old_cmd="$(tr '\0' ' ' < "/proc/$old_pid/cmdline")"
    if [[ "$old_cmd" != *"test_field.launch.py"* || "$old_cmd" != *"shape4_v1_test"* ]]; then
        echo "Refusing to stop unrelated PID $old_pid: $old_cmd" >&2
        exit 1
    fi

    echo "Stopping previous Shape4 run (PID $old_pid) ..."
    kill -INT -- "-$old_pid" 2>/dev/null || true
    for _ in {1..40}; do
        if ! kill -0 "$old_pid" 2>/dev/null; then
            printf '0\n' > "$PID_FILE"
            sleep 1
            return
        fi
        sleep 0.25
    done

    echo "Previous run did not stop cleanly; sending SIGTERM."
    kill -TERM -- "-$old_pid" 2>/dev/null || true
    sleep 2
    printf '0\n' > "$PID_FILE"
}

stop_previous_run

if [[ "${1:-}" == "--stop" ]]; then
    echo "Previous Shape4 run stopped; the next start will begin from t=0."
    exit 0
fi

if [[ "$WITH_BASE" == "true" ]]; then
    if [[ ! -c "$BASE_PORT" ]]; then
        echo "ERROR: combined Arduino serial port is missing: $BASE_PORT" >&2
        echo "Reconnect the CH340 board, then check: ls -l $BASE_PORT" >&2
        exit 1
    fi
    if [[ ! -r "$BASE_PORT" || ! -w "$BASE_PORT" ]]; then
        echo "ERROR: no read/write permission for combined Arduino: $BASE_PORT" >&2
        echo "Check dialout membership: groups" >&2
        exit 1
    fi
fi

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"

echo "Shape4 v1.0.0 clean start"
echo "  state/map/tracks/decisions: reset"
echo "  base=$WITH_BASE arm=$WITH_ARM siglip=$WITH_SIGLIP wall=$WITH_WALL"
echo "  panel: STANDBY(red) -> press -> READY(yellow) -> press -> RUNNING(green)"
echo "  MJPEG: http://10.141.160.217:8080/"

export ROS_LOG_DIR="$ROS_LOG_ROOT"
setsid ros2 launch robot_bringup test_field.launch.py \
    with_base:="$WITH_BASE" \
    with_arm:="$WITH_ARM" \
    with_siglip:="$WITH_SIGLIP" \
    with_wall_localizer:="$WITH_WALL" \
    output_dir:="$RUN_ROOT" \
    "$@" &
LAUNCH_PID=$!
printf '%s\n' "$LAUNCH_PID" > "$PID_FILE"

cleanup() {
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    fi
    printf '0\n' > "$PID_FILE"
}
trap cleanup INT TERM EXIT

wait "$LAUNCH_PID"
