#!/usr/bin/env python3
"""거리 추정 데모 — 본체 cam intrinsics(fx)로 '알려진 크기' 타겟까지 거리 추정.

    거리(m) = fx(px) × 실제크기(m) / 픽셀크기(px)

기본은 체커보드(사각 25mm, 검출 robust) — 보드를 들고 거리를 바꾸면 추정거리가 따라옴.
줄자로 실제 거리를 재서 화면 숫자와 비교하면 정확도 확인.

  python3 distance_demo.py                               # 체커보드 라이브 (q 종료)
  python3 distance_demo.py --color red --real-width 8    # 빨강 객체(실제 폭 8cm)
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csi_capture import CsiCamera  # noqa: E402

HSV = {
    "red":   [(0, 120, 80), (10, 255, 255)],
    "red2":  [(170, 120, 80), (180, 255, 255)],
    "green": [(40, 80, 60), (85, 255, 255)],
    "blue":  [(95, 120, 60), (130, 255, 255)],
    "yellow": [(20, 120, 120), (35, 255, 255)],
}


def board_distance(gray, cols, rows, square_m, fx):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None, None
    c = corners.reshape(-1, 2)
    sp = []
    for r in range(rows):           # 같은 행 인접 코너 = 1칸(square_m)
        for cc in range(cols - 1):
            i = r * cols + cc
            sp.append(float(np.linalg.norm(c[i] - c[i + 1])))
    px = float(np.median(sp))
    return fx * square_m / px, corners


def color_distance(frame, color, real_w_m, fx):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo, hi = HSV[color]
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    if color == "red":
        lo2, hi2 = HSV["red2"]
        mask |= cv2.inRange(hsv, np.array(lo2), np.array(hi2))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None
    cmax = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(cmax) < 300:
        return None, None
    x, y, w, h = cv2.boundingRect(cmax)
    return fx * real_w_m / w, (x, y, w, h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default="data/calib/body_calib.npz")
    ap.add_argument("--sensor-id", type=int, default=1)
    ap.add_argument("--flip", type=int, default=2)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--color", choices=[k for k in HSV if k != "red2"], help="색 객체 모드")
    ap.add_argument("--real-width", type=float, default=8.0, help="색 객체 실제 폭 cm")
    args = ap.parse_args()

    fx = float(np.load(args.calib)["K"][0, 0])
    print(f"fx={fx:.1f}px ({args.calib}).  q=종료")

    cam = CsiCamera(args.sensor_id, flip=args.flip)
    win = "distance demo  [q]=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame = cam.read(1.0)
            if frame is None:
                continue
            disp = frame.copy()
            D, label = None, ""
            if args.color:
                D, box = color_distance(frame, args.color, args.real_width / 100.0, fx)
                if box is not None:
                    x, y, w, h = box
                    cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    label = f"{args.color} w={w}px"
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                D, corners = board_distance(gray, args.cols, args.rows, args.square_mm / 1000.0, fx)
                if corners is not None:
                    cv2.drawChessboardCorners(disp, (args.cols, args.rows), corners, True)
                    label = "board"
            if D is not None:
                cv2.putText(disp, f"distance ~ {D*100:.1f} cm  ({label})", (12, 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 3, cv2.LINE_AA)
            else:
                cv2.putText(disp, "no target", (12, 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (40, 40, 220), 3, cv2.LINE_AA)
            if disp.shape[1] > 960:
                s = 960 / disp.shape[1]
                disp = cv2.resize(disp, (960, int(disp.shape[0] * s)))
            cv2.imshow(win, disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
