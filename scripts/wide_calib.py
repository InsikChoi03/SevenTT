#!/usr/bin/env python3
"""광각(top) 카메라 → 실제 지면좌표(로봇 base 기준 x=앞, y=왼) 호모그래피 캘리브.

pick_calib.py(본체캠) 와 같은 방식의 광각판. 어안이라 **fisheye 왜곡보정한 정규점**을
지면 (x,y)로 보내는 호모그래피 H_wide 를 학습한다. (extrinsic pitch/height/offset 오차까지
데이터가 흡수 → yaw만 대충 맞으면 절대위치가 정확해짐.)

절차:
  1) 물체 하나를 로봇 **회전중심 기준** 알려진 위치에 놓는다. 앞으로 x cm, 왼쪽으로 y cm
     (오른쪽이면 y 음수). 자로 재서.
  2) 그 값을 입력하고 엔터 → 광각 캡처+YOLO로 그 물체 바닥픽셀을 잡아 대응점 저장.
  3) 4점 이상(작업영역 전체에 퍼뜨릴수록 좋음: 가까이/멀리/좌/우) 모으고 q.
  4) H_wide 저장(data/calib/wide_ground.npz) + 재투영오차 리포트.

  python3 scripts/wide_calib.py                 # 수집→학습
  python3 scripts/wide_calib.py --verify         # 저장된 H로 지금 화면 물체들 실좌표 찍어보기
  python3 scripts/wide_calib.py --points p.json  # 저장한 대응점으로 재계산(카메라 안 씀)
"""
from __future__ import annotations
import argparse, json, math, os, sys
import numpy as np
import cv2

sys.path.insert(0, "/home/seventt/seventt/workspace/scripts")
from csi_capture import CsiCamera  # noqa: E402

FX, FY, CX, CY = 857.316, 860.071, 789.858, 541.748
DIST = np.array([-0.085721, 0.053910, -0.034649, 0.007943], np.float64).reshape(4, 1)
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], np.float64)
ROT180 = True                     # camera_top flip=2 (image_rotated_180)
MODEL = "/home/seventt/seventt/workspace/models/wide.pt"
OUT = "/home/seventt/seventt/workspace/data/calib/wide_ground.npz"
PTS = "/home/seventt/seventt/workspace/data/calib/wide_ground_points.json"


def undistort_norm(u, v):
    """어안 픽셀 → 정규(pinhole) 점 (xn,yn). rot180 반영."""
    p = cv2.fisheye.undistortPoints(np.array([[[u, v]]], np.float64), K, DIST)
    xn, yn = float(p[0, 0, 0]), float(p[0, 0, 1])
    if ROT180:
        xn, yn = -xn, -yn
    return xn, yn


def detect(model, cam, conf):
    """한 프레임 캡처+YOLO → [(cls,conf,u,v)]  (u,v=바닥중앙 접점픽셀)."""
    for _ in range(4):
        cam.read(1.0)
    frame = cam.read(1.5)
    if isinstance(frame, tuple):
        frame = frame[0]
    if frame is None:
        return None, []
    r = model.predict(frame, conf=conf, imgsz=640, verbose=False)[0]
    out = []
    if r.boxes is not None and len(r.boxes):
        names = r.names
        xywh = r.boxes.xywh.cpu().numpy(); cl = r.boxes.cls.cpu().numpy(); cf = r.boxes.conf.cpu().numpy()
        for i in range(len(xywh)):
            xc, yc, w, h = xywh[i]
            out.append((str(names[int(cl[i])]), float(cf[i]), float(xc), float(yc) + 0.25 * float(h)))
    return frame, out


def solve(src_norm, dst_xy):
    src = np.array(src_norm, np.float64).reshape(-1, 1, 2)
    dst = np.array(dst_xy, np.float64).reshape(-1, 1, 2)
    H, _ = cv2.findHomography(src, dst, 0)   # 정확대응이므로 최소제곱(RANSAC 불필요)
    if H is None:
        return None, None
    proj = cv2.perspectiveTransform(src, H).reshape(-1, 2)
    err = np.linalg.norm(proj - np.array(dst_xy), axis=1)
    return H, err


def apply_H(H, u, v):
    xn, yn = undistort_norm(u, v)
    p = cv2.perspectiveTransform(np.array([[[xn, yn]]], np.float64), H).reshape(2)
    return float(p[0]), float(p[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--points", default="")
    args = ap.parse_args()

    if args.points:
        d = json.load(open(args.points))
        src = [p["norm"] for p in d]; dst = [p["xy"] for p in d]
        H, err = solve(src, dst)
        print(f"{len(src)}점 재계산: 재투영오차 평균 {err.mean()*100:.1f}cm 최대 {err.max()*100:.1f}cm")
        np.savez(OUT, H=H); print(f"저장 -> {OUT}")
        return 0

    from ultralytics import YOLO
    model = YOLO(MODEL)
    cam = CsiCamera(1, wb_gains=None, flip=2, wbmode=8)   # 학습분포와 동일(wbmode=8)

    if args.verify:
        if not os.path.exists(OUT):
            print("저장된 H 없음 — 먼저 수집하세요"); return 1
        H = np.load(OUT)["H"]
        frame, dets = detect(model, cam, args.conf); cam.release()
        print("== 저장 H로 실좌표(앞x, 왼y) ==")
        for lbl, c, u, v in dets:
            x, y = apply_H(H, u, v)
            print(f"  {lbl:16s} conf={c:.2f}  (x={x*100:+.0f}cm, y={y*100:+.0f}cm)")
        return 0

    print("== 광각 지면 캘리브 수집 ==  (자로 재서 로봇 회전중심 기준 앞x/왼y cm)")
    print("   물체 하나 놓고 'x y'(예: 50 0, 오른쪽이면 y음수) 입력+엔터=캡처 / u=취소 / q=끝\n")
    src_norm, dst_xy, raw = [], [], []
    while True:
        s = input(f"[{len(src_norm)}점] 앞x 왼y (cm): ").strip().lower()
        if s == "q":
            break
        if s == "u":
            if src_norm:
                src_norm.pop(); dst_xy.pop(); raw.pop(); print("  ↩ 마지막 취소")
            continue
        try:
            xc, yc = (float(t) for t in s.split())
        except ValueError:
            print("  ✗ 'x y' 두 값"); continue
        frame, dets = detect(model, cam, args.conf)
        if not dets:
            print("  ✗ 검출 없음 — 물체/조명 확인"); continue
        if len(dets) > 1:
            for i, (lbl, cf, u, v) in enumerate(dets):
                print(f"     [{i}] {lbl} conf={cf:.2f} px=({u:.0f},{v:.0f})")
            k = input("  방금 놓은 물체 번호: ").strip()
            if not k.isdigit() or int(k) >= len(dets):
                print("  건너뜀"); continue
            lbl, cf, u, v = dets[int(k)]
        else:
            lbl, cf, u, v = dets[0]
        xn, yn = undistort_norm(u, v)
        src_norm.append([xn, yn]); dst_xy.append([xc / 100.0, yc / 100.0])
        raw.append({"norm": [xn, yn], "xy": [xc / 100.0, yc / 100.0], "px": [u, v], "label": lbl})
        json.dump(raw, open(PTS, "w"))
        print(f"  ✓ {lbl} px=({u:.0f},{v:.0f}) → 실({xc:.0f},{yc:.0f})cm  [{len(src_norm)}점, 저장됨]")
    cam.release()
    if len(src_norm) < 4:
        print(f"4점 이상 필요(현재 {len(src_norm)})"); return 1
    H, err = solve(src_norm, dst_xy)
    if H is None:
        print("✗ H 실패 — 점이 일직선이거나 부족. 더 퍼뜨려 재수집"); return 1
    print(f"\n재투영오차: 평균 {err.mean()*100:.1f}cm 최대 {err.max()*100:.1f}cm  ({len(src_norm)}점)")
    for i, e in enumerate(err):
        print(f"  점{i} {raw[i]['label']:12s} 실({dst_xy[i][0]*100:+.0f},{dst_xy[i][1]*100:+.0f})cm 오차 {e*100:.1f}cm")
    np.savez(OUT, H=H, points=PTS)
    print(f"\n저장 -> {OUT}   (world_model 에 wide_homography_path 로 물리면 이 매핑 사용)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
