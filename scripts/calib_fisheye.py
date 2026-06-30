#!/usr/bin/env python3
"""Fisheye intrinsics for the wide (top) camera — the lens is a ~150 deg fisheye, so the
standard pinhole model (calib_solve.py) is the WRONG model and underestimates the FOV.

Uses cv2.fisheye.calibrate (equidistant model, 4 distortion coeffs) to MATCH the consumers
once they are switched to the fisheye path:
  - world_model_node : ray = cv2.fisheye.undistortPoints((u,v), K, D)
  - localizer_node   : cv2.fisheye.initUndistortRectifyMap + remap

  python3 scripts/calib_fisheye.py                 # solve, print K/D/FOV, save npz + sample
  python3 scripts/calib_fisheye.py --imgs data/calib/top

Prints perception.yaml-ready values; paste them into the localizer_node + world_model_node
blocks (top_fx/fy/cx/cy + 4-elem dist_coeffs + fisheye_model: true).
"""
from __future__ import annotations

import argparse
import glob
import math
import os

import cv2
import numpy as np


def solve(imgs, cols, rows, square_mm):
    objp = np.zeros((1, cols * rows, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[0] *= square_mm / 1000.0
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    ff = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    paths = sorted(glob.glob(os.path.join(imgs, "*.png")) + glob.glob(os.path.join(imgs, "*.jpg")))
    if not paths:
        raise SystemExit(f"no images in {imgs}")
    objpoints, imgpoints, used, size = [], [], [], None
    for p in paths:
        g = cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2GRAY)
        size = (g.shape[1], g.shape[0])
        ok, c = cv2.findChessboardCorners(g, (cols, rows), ff)
        if not ok:
            continue
        c = cv2.cornerSubPix(g, c, (11, 11), (-1, -1), crit)
        objpoints.append(objp)
        imgpoints.append(c.reshape(1, -1, 2).astype(np.float64))
        used.append(p)
    print(f"used {len(used)}/{len(paths)} images")
    if len(used) < 8:
        raise SystemExit("too few usable images (<8)")

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        objpoints, imgpoints, size, K, D, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 80, 1e-6),
    )
    return rms, K, D, size, used


def fov(K, D, size):
    W, H = size

    def ang(px, py):
        u = cv2.fisheye.undistortPoints(np.array([[[px, py]]], np.float64), K, D)
        x, y = u[0, 0]
        return math.degrees(math.atan(math.hypot(x, y)))

    cx, cy = K[0, 2], K[1, 2]
    return ang(0, cy) + ang(W - 1, cy), ang(cx, 0) + ang(cx, H - 1), ang(0, 0) + ang(W - 1, H - 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--imgs", default="data/calib/top")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square-mm", type=float, default=25.0)
    args = ap.parse_args()

    rms, K, D, size, used = solve(args.imgs, args.cols, args.rows, args.square_mm)
    fh, fv, fd = fov(K, D, size)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = D.ravel()

    print("\n=== FISHEYE RESULT ===")
    print(f"image_size = {size[0]}x{size[1]}   RMS = {rms:.3f} px")
    print(f"FOV  horiz {fh:.0f}   vert {fv:.0f}   diag {fd:.0f}  deg")
    print("\n--- perception.yaml (paste into BOTH localizer_node and world_model_node) ---")
    print(f"    fisheye_model: true")
    print(f"    top_fx: {fx:.6f}")
    print(f"    top_fy: {fy:.6f}")
    print(f"    top_cx: {cx:.6f}")
    print(f"    top_cy: {cy:.6f}")
    print(f"    dist_coeffs: [{d[0]:.6f}, {d[1]:.6f}, {d[2]:.6f}, {d[3]:.6f}]")

    out_dir = os.path.dirname(args.imgs.rstrip("/")) or "."
    name = os.path.basename(args.imgs.rstrip("/"))
    npz = os.path.join(out_dir, f"{name}_fisheye_calib.npz")
    np.savez(npz, K=K, D=D, img_size=np.array(size), rms=rms, model="fisheye")
    # visual check: rectified sample (straight lines should become straight)
    img = cv2.imread(used[0])
    h, w = img.shape[:2]
    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=0.0)
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, (w, h), cv2.CV_16SC2)
    und = cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
    samp = os.path.join(out_dir, f"{name}_fisheye_undistort_sample.png")
    cv2.imwrite(samp, np.hstack([img, und]))
    print(f"\nsaved {npz} and {samp} (left=raw fisheye, right=rectified — 직선이 직선이어야)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
