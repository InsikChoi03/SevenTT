#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SKETCH_DIR="$TEST_ROOT/firmware/shape4_start_panel_v1_0_0"
BUILD_DIR="/tmp/shape4_panel_build"
PORT="${SHAPE4_PANEL_PORT:-/dev/ttyUSB0}"
FQBN="arduino:avr:uno"

if [[ ! -e "$PORT" ]]; then
  echo "Arduino serial port not found: $PORT" >&2
  echo "Reconnect the Arduino and check: ls -l $PORT" >&2
  exit 1
fi

echo "[1/3] Compiling Shape4 start-panel test ..."
arduino-cli compile \
  --fqbn "$FQBN" \
  --build-path "$BUILD_DIR" \
  "$SKETCH_DIR"

echo "[2/3] Uploading to $PORT ..."
arduino-cli upload \
  --fqbn "$FQBN" \
  --port "$PORT" \
  --input-dir "$BUILD_DIR"

echo "[3/3] Monitoring button events at 115200 baud."
echo "Expected LEDs: READY=yellow, first press=green, second press=red, third press=yellow."
echo "Press Ctrl+C to close only the monitor; the Arduino test keeps running."
exec arduino-cli monitor \
  --port "$PORT" \
  --config baudrate=115200 \
  --timestamp
