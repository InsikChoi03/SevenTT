#!/usr/bin/env bash
# Environment sanity check for Jetson Orin Nano + ROS2 Humble
set -u

ok() { printf "  \033[32m✓\033[0m %s\n" "$1"; }
fail() { printf "  \033[31m✗\033[0m %s\n" "$1"; }
warn() { printf "  \033[33m!\033[0m %s\n" "$1"; }
section() { printf "\n\033[1m== %s ==\033[0m\n" "$1"; }

section "Hardware / OS"
if [ -f /etc/nv_tegra_release ]; then
  ok "JetPack: $(head -1 /etc/nv_tegra_release | sed 's/.*RELEASE: \([0-9]*\).*REVISION: \([0-9.]*\).*/R\1.\2/')"
else
  fail "Not a Jetson board (no /etc/nv_tegra_release)"
fi

if grep -q "Ubuntu 22.04" /etc/os-release; then
  ok "Ubuntu 22.04 (ROS2 Humble compatible)"
else
  warn "Not Ubuntu 22.04 — ROS2 Humble may not work"
fi

section "CUDA / TensorRT"
if command -v nvcc >/dev/null 2>&1; then
  ok "CUDA: $(nvcc --version | grep release | awk '{print $5,$6}' | tr -d ',')"
elif ls /usr/local/cuda*/bin/nvcc >/dev/null 2>&1; then
  warn "CUDA installed but not in PATH (add /usr/local/cuda/bin to PATH)"
else
  fail "CUDA not found"
fi

if dpkg -l 2>/dev/null | grep -q libnvinfer-bin; then
  TRT_VER=$(dpkg -l | grep '^ii  libnvinfer-bin' | awk '{print $3}')
  ok "TensorRT: $TRT_VER"
else
  fail "TensorRT not installed"
fi

section "Python"
ok "Python: $(python3 --version 2>&1)"
if command -v pip3 >/dev/null 2>&1; then
  ok "pip3 available"
else
  warn "pip3 not installed (sudo apt install python3-pip)"
fi

section "ROS2"
if [ -f /opt/ros/humble/setup.bash ]; then
  ok "ROS2 Humble installed at /opt/ros/humble"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  ok "ros2 CLI: $(ros2 --version 2>&1 || echo 'unknown')"
  if command -v colcon >/dev/null 2>&1; then
    ok "colcon available"
  else
    warn "colcon not installed (sudo apt install python3-colcon-common-extensions)"
  fi
else
  fail "ROS2 Humble NOT installed"
fi

section "Camera (CSI IMX219)"
if lsmod | grep -q nv_imx219; then
  ok "nv_imx219 kernel module loaded"
else
  fail "nv_imx219 module not loaded"
fi
if systemctl is-active --quiet nvargus-daemon; then
  ok "nvargus-daemon running"
else
  fail "nvargus-daemon not running"
fi
# Probe via gstreamer (quick test, no display required)
if timeout 6 gst-launch-1.0 -v nvarguscamerasrc num-buffers=1 sensor-id=0 ! fakesink 2>&1 | grep -q EOS \
   && ! timeout 6 gst-launch-1.0 -v nvarguscamerasrc num-buffers=1 sensor-id=0 ! fakesink 2>&1 | grep -q "No cameras"; then
  ok "Camera @ CAM0 captures successfully"
else
  fail "Camera not capturing (check FFC cable orientation & latch)"
fi

section "Memory / Disk"
MEM_FREE=$(free -h | awk 'NR==2{print $7}')
ok "Memory available: $MEM_FREE"
DISK_FREE=$(df -h / | awk 'NR==2{print $4}')
ok "Disk available: $DISK_FREE"

echo
echo "Done."
