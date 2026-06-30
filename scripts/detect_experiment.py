#!/usr/bin/env python3
"""바닥 위 흰 물체 '탐지' 방법 비교 실험 (저장 이미지 대상).

  A. HSV 흰색 마스크 + (solidity/크기/테두리) 필터        — 지금까지 쓰던 것
  B. 채도낮음 + 표면매끄러움(국소표준편차 낮음)            — 나무결(텍스처) 거부, 무광 물체만
  C. ORB 특징점                                          — 텍스처 없는 물체에 키포인트가 잡히나?

각 방법 시각화를 *_A/_B/_orb.png 로 저장하고, 블롭 진단을 출력.
  python3 scripts/detect_experiment.py data/cam_test/closeup.png
"""
from __future__ import annotations
import argparse
import os
import cv2
import numpy as np


def filt_blobs(mask, frame_shape, min_area, max_area_frac, min_solidity, tag):
    h, w = frame_shape[:2]
    max_area = max_area_frac * h * w
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    kept = []
    print(f"  [{tag}] raw contours={len(cnts)}")
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:12]:
        a = cv2.contourArea(c)
        if a < 200:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        border = x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1
        ha = cv2.contourArea(cv2.convexHull(c))
        sol = a / ha if ha > 0 else 0
        ok = (a >= min_area) and (a <= max_area) and (not border) and (sol >= min_solidity)
        print(f"     area={a:8.0f} sol={sol:.2f} border={int(border)} bbox=({x},{y},{bw},{bh}) "
              f"{'KEEP' if ok else 'drop'}")
        if ok:
            kept.append(c)
    return kept


def method_A(frame, args):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s, v = hsv[..., 1], hsv[..., 2]
    mask = (((s < args.s_max) & (v > args.v_min)).astype(np.uint8)) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def method_B(frame, args):
    """채도낮음 + 표면매끄러움. 나무결은 국소표준편차가 큼 → 거부."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s, v = hsv[..., 1], hsv[..., 2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    win = (args.tex_win, args.tex_win)
    mean = cv2.blur(gray, win)
    sq = cv2.blur(gray * gray, win)
    std = cv2.sqrt(cv2.max(sq - mean * mean, 0))
    smooth = (std < args.tex_max).astype(np.uint8)             # 매끄러운 영역
    desat = (s < args.s_max).astype(np.uint8)                  # 흰/무채색
    bright = (v > args.v_lo).astype(np.uint8)                  # 그림자면까지 포함(낮춤)
    mask = (smooth & desat & bright) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=4)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)
    return mask


def draw_boxes(frame, blobs, color=(0, 255, 0)):
    out = frame.copy()
    for c in blobs:
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 3)
        cv2.drawContours(out, [c], -1, color, 2)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--s-max", type=int, default=70)
    ap.add_argument("--v-min", type=int, default=160)
    ap.add_argument("--v-lo", type=int, default=110)
    ap.add_argument("--tex-win", type=int, default=15)
    ap.add_argument("--tex-max", type=float, default=10.0)
    ap.add_argument("--min-area", type=int, default=2500)
    ap.add_argument("--max-area-frac", type=float, default=0.30)
    ap.add_argument("--min-solidity", type=float, default=0.80)
    args = ap.parse_args()

    frame = cv2.imread(args.src)
    if frame is None:
        print(f"읽기 실패: {args.src}"); return 1
    base = os.path.splitext(args.src)[0]

    print("== Method A: HSV 흰색 ==")
    mA = method_A(frame, args)
    bA = filt_blobs(mA, frame.shape, args.min_area, args.max_area_frac, args.min_solidity, "A")
    cv2.imwrite(base + "_A.png", draw_boxes(frame, bA))
    cv2.imwrite(base + "_A_mask.png", mA)

    print("== Method B: 채도+매끄러움 ==")
    mB = method_B(frame, args)
    bB = filt_blobs(mB, frame.shape, args.min_area, args.max_area_frac, args.min_solidity, "B")
    cv2.imwrite(base + "_B.png", draw_boxes(frame, bB, (255, 0, 255)))
    cv2.imwrite(base + "_B_mask.png", mB)

    print("== ORB 특징점 ==")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(nfeatures=1500)
    kps = orb.detect(gray, None)
    vis = cv2.drawKeypoints(frame, kps, None, color=(0, 255, 0),
                            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imwrite(base + "_orb.png", vis)
    print(f"  ORB keypoints={len(kps)}")

    print(f"\nA blobs={len(bA)}  B blobs={len(bB)}  -> {base}_A/_B/_orb.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
