#!/usr/bin/env python3
"""Solve wide-camera intrinsics from captured checkerboard images and (optionally)
write them into robot_bringup/config/perception.yaml.

Uses the standard OpenCV pinhole + radial/tangential model (cv2.calibrateCamera +
cv2.undistort) to MATCH the consumers:
  - localizer_node : cv2.undistort(frame, camera_matrix, dist_coeffs)
  - world_model_node : pinhole ray d=((u-cx)/fx,(v-cy)/fy,1)
So we emit fx, fy, cx, cy and a 5-element dist_coeffs [k1,k2,p1,p2,k3] (8 with --rational).

  python3 scripts/calib_solve.py                 # dry-run, prints result
  python3 scripts/calib_solve.py --write         # also patches perception.yaml
  python3 scripts/calib_solve.py --rational      # 8-coeff model (very wide lens)
"""
from __future__ import annotations

import argparse
import glob
import os
import re

import cv2
import numpy as np


def solve(imgs, cols, rows, square_mm, rational, fix_aspect=True):
    pattern = (cols, rows)
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm / 1000.0  # meters (only scales extrinsics; intrinsics unaffected)

    paths = sorted(glob.glob(os.path.join(imgs, "*.png")) + glob.glob(os.path.join(imgs, "*.jpg")))
    if not paths:
        raise SystemExit(f"no images in {imgs} — run scripts/calib_capture.py first")

    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    objpoints, imgpoints, used = [], [], []
    img_size = None
    for p in paths:
        img = cv2.imread(p)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, pattern, find_flags)
        if not found:
            print(f"  [skip] no {cols}x{rows} board: {os.path.basename(p)}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
        objpoints.append(objp)
        imgpoints.append(corners)
        used.append(p)

    print(f"used {len(used)}/{len(paths)} images")
    if len(used) < 4:
        raise SystemExit("too few usable images (<4); recapture with the board fully visible")
    if len(used) < 10:
        print("WARNING: <10 good views — solution may be unstable; capture more for production")

    calib_flags = cv2.CALIB_RATIONAL_MODEL if rational else 0
    K_init = None
    if fix_aspect:
        # IMX219 has square pixels, so fx must equal fy. Forcing it removes a
        # gauge degeneracy: with all boards at similar oblique poses the free
        # 5-param solve lets fy run away (fx=1823/fy=5037, RMS 1.31) while a
        # square-pixel solve is both physical AND fits better (fx=fy=962, RMS 1.05).
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_USE_INTRINSIC_GUESS
        K_init = np.array([[1000.0, 0, img_size[0] / 2.0],
                           [0, 1000.0, img_size[1] / 2.0],
                           [0, 0, 1.0]], dtype=np.float64)
    rms, K, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, img_size, K_init, None, flags=calib_flags)
    return rms, K, dist.ravel(), img_size, used


def write_yaml(path, fx, fy, cx, cy, dist):
    if not os.path.exists(path):
        raise SystemExit(f"perception.yaml not found at {path}")
    with open(path) as f:
        lines = f.readlines()
    dist_str = "[" + ", ".join(f"{c:.6f}" for c in dist) + "]"
    vals = {"top_fx": fx, "top_fy": fy, "top_cx": cx, "top_cy": cy}
    out = []
    for line in lines:
        m = re.match(r"^(\s*)(top_fx|top_fy|top_cx|top_cy):\s*\S.*$", line)
        if m:
            indent, key = m.group(1), m.group(2)
            out.append(f"{indent}{key}: {vals[key]:.6f}\n")
            continue
        m = re.match(r"^(\s*dist_coeffs:\s*)\[[^\]]*\](.*)$", line)
        if m:
            out.append(f"{m.group(1)}{dist_str}{m.group(2)}\n")
            continue
        out.append(line)
    with open(path, "w") as f:
        f.writelines(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--imgs", default="data/calib/top")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="checker square size; only scales extrinsics, intrinsics unaffected")
    ap.add_argument("--rational", action="store_true", help="8-coeff rational model (very wide lens)")
    ap.add_argument("--free-aspect", action="store_true",
                    help="allow fx != fy (default forces fx=fy for IMX219 square pixels)")
    ap.add_argument("--write", action="store_true", help="patch perception.yaml in place")
    ap.add_argument("--yaml", default="ros2_ws/src/robot_bringup/config/perception.yaml")
    args = ap.parse_args()

    rms, K, dist, img_size, used = solve(args.imgs, args.cols, args.rows, args.square_mm,
                                         args.rational, fix_aspect=not args.free_aspect)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    print("\n=== RESULT ===")
    print(f"image_size = {img_size[0]}x{img_size[1]}")
    quality = "good" if rms < 1.0 else "HIGH — recapture edges, or try --rational / center crop"
    print(f"RMS reprojection error = {rms:.3f} px   ({quality})")
    print(f"fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}")
    print(f"dist_coeffs = {[round(float(c), 6) for c in dist]}")

    name = os.path.basename(args.imgs.rstrip("/")) or "calib"   # top / body 등
    out_dir = os.path.dirname(args.imgs.rstrip("/")) or "."
    npz = os.path.join(out_dir, f"{name}_calib.npz")
    np.savez(npz, K=K, dist=dist, img_size=img_size, rms=rms)
    sample = cv2.imread(used[0])
    und = cv2.undistort(sample, K, dist)
    undpath = os.path.join(out_dir, f"{name}_undistort_sample.png")
    cv2.imwrite(undpath, np.hstack([sample, und]))
    print(f"saved {npz} and {undpath} "
          f"(left=raw, right=undistorted — 직선이 직선이어야)")

    if args.write:
        write_yaml(args.yaml, fx, fy, cx, cy, [float(c) for c in dist])
        print(f"\n>>> wrote top_fx/fy/cx/cy (both blocks) + dist_coeffs into {args.yaml}")
        print("    rebuild config:  colcon build --packages-select robot_bringup")
    else:
        print("\n(dry-run) re-run with --write to patch perception.yaml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
