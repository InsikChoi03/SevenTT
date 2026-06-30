#!/usr/bin/env python3
"""저장된 프레임에 화이트밸런스 보정을 적용/비교 (재캡처 없이).

밝은 영역(흰 물체)에서 광원색을 추정해 고정 채널 게인을 건다 — 전역 gray-world의
녹색 과보정을 피함. 추정 게인을 출력하므로 그대로 csi_capture에 박아 쓸 수 있다.

  python3 scripts/wb_apply.py data/cam_test/wb_auto_raw.png --out data/cam_test/wb_fixed.png
  python3 scripts/wb_apply.py data/cam_test/wb_auto_raw.png --gains 0.86,1.0,1.18   # 직접 지정
"""
from __future__ import annotations
import argparse
import cv2
import numpy as np


def estimate_gains(img, bright_pct=98.0):
    """밝은 픽셀(luma 상위 %)의 평균색을 흰색으로 만드는 BGR 게인."""
    f = img.astype(np.float32)
    luma = 0.114 * f[..., 0] + 0.587 * f[..., 1] + 0.299 * f[..., 2]
    thr = np.percentile(luma, bright_pct)
    mask = luma >= thr
    ref = np.array([f[..., c][mask].mean() for c in range(3)])  # B,G,R of whites
    g = ref.mean()
    return g / ref  # 게인: 흰 영역 채널평균을 공통값으로


def apply_gains(img, gains):
    f = img.astype(np.float32)
    for c in range(3):
        f[..., c] *= gains[c]
    return np.clip(f, 0, 255).astype(np.uint8)


def means(img):
    return tuple(round(float(img[..., c].mean()), 1) for c in range(3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--out", default="data/cam_test/wb_fixed.png")
    ap.add_argument("--bright-pct", type=float, default=98.0)
    ap.add_argument("--gains", help="B,G,R 직접 지정 (예: 0.86,1.0,1.18)")
    args = ap.parse_args()

    img = cv2.imread(args.src)
    if img is None:
        print(f"읽기 실패: {args.src}"); return 1

    if args.gains:
        gains = np.array([float(x) for x in args.gains.split(",")])
    else:
        gains = estimate_gains(img, args.bright_pct)

    out = apply_gains(img, gains)
    cv2.imwrite(args.out, out)
    print(f"gains(B,G,R) = {gains[0]:.3f}, {gains[1]:.3f}, {gains[2]:.3f}")
    print(f"raw  mean(B,G,R) = {means(img)}")
    print(f"out  mean(B,G,R) = {means(out)}  -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
