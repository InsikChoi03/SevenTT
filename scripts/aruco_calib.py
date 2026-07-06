#!/usr/bin/env python3
"""ArUco 지면 캘리브 — 바닥 마커로 카메라 픽셀 → 실제 지면좌표(로봇 회전중심 기준 x앞/y왼) 호모그래피.

마커 중심(서브픽셀)을 대응점으로. 마커는 **바닥에 평평하게**(한 평면). 좌표는 **회전중심 기준
앞=+x, 왼=+y, cm**. 광각은 어안이라 **rectify 후 검출**(멀리·가장자리 마커도 잡힘) + world_model
과 동일 정규화(fisheye undistort + rot180). 온-로봇 마커(robot_markers.json의 ID)는 자동 제외.

  python3 scripts/aruco_calib.py --cam wide                # 대화형 수집(입력값 layout 저장)
  python3 scripts/aruco_calib.py --cam wide --layout data/calib/ground_layout.json  # 재측정 없이
  python3 scripts/aruco_calib.py --cam wide --verify        # 저장 H로 마커 실좌표 찍어보기
"""
from __future__ import annotations
import argparse, json, os, sys
import numpy as np
import cv2

sys.path.insert(0, "/home/seventt/seventt/workspace/scripts")
from csi_capture import CsiCamera  # noqa: E402

WK = np.array([[857.316, 0, 789.858], [0, 860.071, 541.748], [0, 0, 1]], np.float64)
WD = np.array([-0.085721, 0.053910, -0.034649, 0.007943], np.float64).reshape(4, 1)
ROT180 = True
DICTS = {"4x4_50": cv2.aruco.DICT_4X4_50, "5x5_50": cv2.aruco.DICT_5X5_50,
         "apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11}
# 둘 다 회전중심 기준 METRES 지면 호모그래피 (world_model 융합용). 본체는 옛 pick H(arm_base cm)
# 를 덮지 않게 별도 파일. (픽업용 arm_base H는 별개로 유지)
OUT = {"wide": "/home/seventt/seventt/workspace/data/calib/wide_ground.npz",
       "body": "/home/seventt/seventt/workspace/data/calib/body_ground.npz"}
LAYOUT_DEF = "/home/seventt/seventt/workspace/data/calib/ground_layout.json"
ROBOT_MARKERS = "/home/seventt/seventt/workspace/data/calib/robot_markers.json"


def make_detector(dict_name):
    p = cv2.aruco.DetectorParameters()
    p.minMarkerPerimeterRate = 0.01
    p.adaptiveThreshWinSizeMax = 45
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTS[dict_name]), p)


def detect_centers(cam, detector, maps, frames=8):
    """N프레임 검출 → id별 중심픽셀(rectify 시 rect픽셀). raw/detect 마지막프레임."""
    acc, cnt, raw_last, det_last = {}, {}, None, None
    for _ in range(frames + 3):
        f = cam.read(1.0)
        if isinstance(f, tuple):
            f = f[0]
        if f is None:
            continue
        raw_last = f
        det = cv2.remap(f, maps[0], maps[1], cv2.INTER_LINEAR) if maps else f
        det_last = det
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(det, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue
        for c, i in zip(corners, ids.flatten()):
            i = int(i)
            acc[i] = acc.get(i, np.zeros((4, 2))) + c.reshape(4, 2)
            cnt[i] = cnt.get(i, 0) + 1
    centers = {i: (acc[i] / cnt[i]).mean(axis=0) for i in acc}
    return centers, cnt, raw_last, det_last


def solve(src, dst_m):
    H, _ = cv2.findHomography(np.array(src, np.float64).reshape(-1, 1, 2),
                              np.array(dst_m, np.float64).reshape(-1, 1, 2), 0)
    if H is None:
        return None, None
    proj = cv2.perspectiveTransform(np.array(src, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - np.array(dst_m), axis=1)
    return H, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", choices=["wide", "body"], default="wide")
    ap.add_argument("--dict", default="4x4_50", choices=list(DICTS))
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--balance", type=float, default=0.5, help="wide rectify FOV (1=넓게)")
    ap.add_argument("--layout", default="")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    out = args.out or OUT[args.cam]

    detector = make_detector(args.dict)
    cam = CsiCamera(1, wb_gains=None, flip=2, wbmode=8) if args.cam == "wide" \
        else CsiCamera(0, wbmode=8)

    # 광각만 rectify (어안). src 정규화도 여기서 결정.
    if args.cam == "wide":
        W, H = 1640, 1232
        newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            WK, WD, (W, H), np.eye(3), balance=args.balance)
        maps = cv2.fisheye.initUndistortRectifyMap(WK, WD, np.eye(3), newK, (W, H), cv2.CV_16SC2)

        def src_of(u, v):   # rect픽셀 -> 정규 + rot180 (world_model _project_wide_H 와 동일)
            nx, ny = (u - newK[0, 2]) / newK[0, 0], (v - newK[1, 2]) / newK[1, 1]
            return (-nx, -ny) if ROT180 else (nx, ny)
    else:
        maps = None
        def src_of(u, v):   # body는 저왜곡 → raw 픽셀 그대로 (기존 pick 호모그래피와 동일)
            return (u, v)

    centers, cnt, raw, det = detect_centers(cam, detector, maps, args.frames)
    cam.release()
    robot_ids = set()
    if os.path.exists(ROBOT_MARKERS):
        robot_ids = {int(k) for k in json.load(open(ROBOT_MARKERS))["markers"]}
    # 안정 검출만 (유령/플리커 무시: 프레임 절반 미만만 본 것 = 오검출로 버림)
    min_seen = max(2, args.frames // 2)
    flicker = sorted(i for i in centers if cnt.get(i, 0) < min_seen)
    stable = {i: c for i, c in centers.items() if cnt.get(i, 0) >= min_seen}
    ground = {i: c for i, c in stable.items() if i not in robot_ids}
    print(f"검출 {sorted(centers)}  안정 {sorted(stable)}  바닥사용 {sorted(ground)}"
          + (f"  플리커무시(유령) {flicker}" if flicker else ""))
    if det is not None:   # 디버그 이미지
        dbg = det.copy()
        for i, c in centers.items():
            col = ((0, 255, 0) if i in ground else (0, 165, 255) if i in robot_ids else (0, 0, 255))
            cv2.circle(dbg, (int(c[0]), int(c[1])), 12, col, 2)
            cv2.putText(dbg, str(i), (int(c[0]) + 8, int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        dp = f"/home/seventt/seventt/workspace/data/calib/aruco_{args.cam}_detect.png"
        cv2.imwrite(dp, dbg); print(f"검출 이미지 -> {dp} (초록=사용, 주황=온로봇, 빨강=유령/미사용)")
    if not ground:
        print("바닥 마커 검출 0 — 조명/크기/평평/거리 확인"); return 1

    if args.verify:
        if not os.path.exists(out):
            print("저장 H 없음"); return 1
        Hm = np.load(out)["H"]
        lay = {int(k): v for k, v in json.load(open(LAYOUT_DEF)).items()} if os.path.exists(LAYOUT_DEF) else {}
        errs = []
        for i in sorted(ground):
            p = cv2.perspectiveTransform(np.array([[list(src_of(*ground[i]))]], np.float64), Hm).reshape(2)
            if i in lay:
                tx, ty = lay[i]
                e = ((p[0] * 100 - tx) ** 2 + (p[1] * 100 - ty) ** 2) ** 0.5
                errs.append(e)
                print(f"  ID {i:3d} → ({p[0]*100:+.0f},{p[1]*100:+.0f})cm  실측({tx:+.0f},{ty:+.0f})  오차 {e:.1f}cm")
            else:
                print(f"  ID {i:3d} → ({p[0]*100:+.0f},{p[1]*100:+.0f})cm  (실측값 없음)")
        if errs:
            print(f"검증오차 평균 {sum(errs)/len(errs):.1f}cm 최대 {max(errs):.1f}cm  ({len(errs)}점)")
        return 0

    layout = {}
    if args.layout and os.path.exists(args.layout):
        layout = {int(k): v for k, v in json.load(open(args.layout)).items()}

    # 파일 layout은 그대로, 나머지는 대화형(잘못 입력 시 'u'로 이전 마커 재입력).
    entered = dict(layout)
    gids = [i for i in sorted(ground) if i not in layout]
    idx = 0
    while idx < len(gids):
        i = gids[idx]
        s = input(f"  [{idx+1}/{len(gids)}] ID {i}  앞x 왼y cm  (u=이전수정, s/엔터=건너뜀): ").strip().lower()
        if s == "u":
            if idx > 0:
                idx -= 1
                entered.pop(gids[idx], None)
                print(f"   ↩ ID {gids[idx]} 다시 입력")
            else:
                print("   (되돌릴 이전 없음)")
            continue
        if s in ("", "s"):
            entered.pop(i, None); idx += 1; continue
        try:
            xc, yc = (float(t) for t in s.split())
        except ValueError:
            print("   ✗ 'x y' 두 값 (다시)"); continue
        entered[i] = [xc, yc]
        print(f"   ✓ ID {i} → ({xc},{yc})cm")
        idx += 1

    layout = entered
    srcs, dst = [], []
    for i in sorted(ground):
        if i in layout:
            srcs.append(list(src_of(*ground[i])))
            dst.append([layout[i][0] / 100.0, layout[i][1] / 100.0])

    if len(srcs) < 4:
        print(f"4점 이상 필요(현재 {len(srcs)})"); return 1
    os.makedirs(os.path.dirname(LAYOUT_DEF), exist_ok=True)
    json.dump({str(k): v for k, v in layout.items()}, open(args.layout or LAYOUT_DEF, "w"))
    Hm, err = solve(srcs, dst)
    if Hm is None:
        print("✗ H 실패 — 마커 일직선? 더 퍼뜨려"); return 1
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez(out, H=Hm)
    print(f"\n재투영오차 평균 {err.mean()*100:.1f}cm 최대 {err.max()*100:.1f}cm  ({len(srcs)}점)")
    for i, e in zip([k for k in sorted(ground) if k in layout], err):
        flag = "  ⚠큰잔차" if e * 100 > 3.0 else ""
        print(f"  ID {i} 오차 {e*100:.1f}cm{flag}")
    if err.max() * 100 > 3.0:
        print("⚠ 최대잔차 3cm↑ — 마커가 비평면(높이 다름)이거나 실측 입력 오류일 가능성 큼.\n"
              "  모든 마커를 같은 평평한 바닥에 놓고, 전방 근/중/원 + 좌/중/우로 퍼뜨려 다시 찍으세요.")
    print(f"저장 -> {out}")
    if args.cam == "wide":
        print("→ test_field.yaml world_model_node 의 wide_homography_path 주석 해제 후 "
              "colcon build --packages-select robot_bringup")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
