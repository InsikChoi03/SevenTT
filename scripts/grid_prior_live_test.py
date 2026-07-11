#!/usr/bin/env python3
"""Stationary live test for the world-model object-grid prior.

Starts:
  1. robot_bringup test_field.launch.py with_base:=false grid_prior_enabled:=false by default
  2. scripts/live_field_view.py, which streams the newest recognition_viz live.png

The base-drive nodes are not launched, so the robot should stay still.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROS_WS = ROOT / "ros2_ws"
DEFAULT_OUTPUT = ROOT / "data" / "mock_field_test"
SOURCE_LAUNCH = ROS_WS / "src" / "robot_bringup" / "launch" / "test_field.launch.py"
SOURCE_PARAMS = ROS_WS / "src" / "robot_bringup" / "config" / "test_field.yaml"


def _launch_ros(args: argparse.Namespace) -> subprocess.Popen:
    cmd = [
        "ros2",
        "launch",
        str(SOURCE_LAUNCH),
        f"params_file:={args.params_file}",
        "with_base:=false",
        f"with_siglip:={'true' if args.siglip else 'false'}",
        f"with_wall_localizer:={'true' if args.wall_localizer else 'false'}",
        f"grid_prior_enabled:={'true' if args.grid_prior else 'false'}",
        f"output_dir:={args.output_dir}",
        f"cam_yaw:={args.cam_yaw}",
        f"rot180:={'true' if args.rot180 else 'false'}",
    ]
    shell_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        + " ".join(cmd)
    )
    env = os.environ.copy()
    env.setdefault("RCUTILS_COLORIZED_OUTPUT", "1")
    (ROOT / "tmp" / "ros_log").mkdir(parents=True, exist_ok=True)
    env.setdefault("ROS_LOG_DIR", str(ROOT / "tmp" / "ros_log"))
    source_paths = [
        str(ROS_WS / "src" / "robot_perception"),
        str(ROS_WS / "src" / "robot_planning"),
        str(ROS_WS / "src" / "robot_control"),
        str(ROS_WS / "src" / "robot_hardware"),
        str(ROS_WS / "src" / "robot_bringup"),
    ]
    old_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(source_paths + ([old_pythonpath] if old_pythonpath else []))
    return subprocess.Popen(["bash", "-lc", shell_cmd], cwd=ROS_WS, env=env)


def _launch_stream(args: argparse.Namespace) -> subprocess.Popen:
    cmd = [
        sys.executable,
        str(ROOT / "scripts" / "live_field_view.py"),
        "--root",
        args.output_dir,
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--fps",
        str(args.fps),
    ]
    return subprocess.Popen(cmd, cwd=ROOT)


def _terminate(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        proc.terminate()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    ap.add_argument("--params-file", default=str(SOURCE_PARAMS))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--grid-prior", action=argparse.BooleanOptionalAction, default=False)
    ap.add_argument("--siglip", action="store_true", help="also start SigLIP fruit-type gate")
    ap.add_argument("--wall-localizer", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--cam-yaw", default="90")
    ap.add_argument("--rot180", action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args()

    ros_proc = None
    stream_proc = None
    try:
        stream_proc = _launch_stream(args)
        ros_proc = _launch_ros(args)
        print(
            f"\nStationary grid-prior live test is starting.\n"
            f"Browser stream: http://{args.host}:{args.port}/\n"
            f"ROS launch: with_base=false grid_prior_enabled={str(args.grid_prior).lower()}\n"
            "Press Ctrl-C here to stop both processes.\n",
            flush=True,
        )
        while True:
            if ros_proc.poll() is not None:
                return ros_proc.returncode or 0
            if stream_proc.poll() is not None:
                return stream_proc.returncode or 0
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    finally:
        _terminate(ros_proc)
        _terminate(stream_proc)


if __name__ == "__main__":
    raise SystemExit(main())
