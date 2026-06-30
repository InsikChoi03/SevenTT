#!/usr/bin/env bash
# Quick CSI IMX219 capture test — saves one frame as JPEG.
set -eu

SENSOR_ID="${1:-0}"
OUT="${2:-/tmp/camera_test_sensor${SENSOR_ID}.jpg}"

echo "Capturing 1 frame from sensor-id=${SENSOR_ID} → ${OUT}"
gst-launch-1.0 -e \
  nvarguscamerasrc num-buffers=1 sensor-id="${SENSOR_ID}" \
  ! "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1, format=NV12" \
  ! nvvidconv \
  ! "video/x-raw, format=BGRx" \
  ! videoconvert \
  ! "video/x-raw, format=BGR" \
  ! jpegenc \
  ! filesink location="${OUT}"

if [ -s "${OUT}" ]; then
  echo "OK — $(du -h "${OUT}" | awk '{print $1}') saved at ${OUT}"
else
  echo "FAILED — empty file" >&2
  exit 1
fi
