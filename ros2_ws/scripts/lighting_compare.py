#!/usr/bin/env python3
"""Run the repository-level lighting comparison tool from ros2_ws/.

This wrapper exists so both of these work:
  python3 scripts/lighting_compare.py
  python3 ../scripts/lighting_compare.py
"""
from __future__ import annotations

import os
import runpy
import sys


ROS2_WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(ROS2_WS)
REAL_SCRIPT = os.path.join(REPO_ROOT, "scripts", "lighting_compare.py")


if __name__ == "__main__":
    os.chdir(REPO_ROOT)
    sys.argv[0] = REAL_SCRIPT
    runpy.run_path(REAL_SCRIPT, run_name="__main__")
