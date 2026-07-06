#!/usr/bin/env python3
"""온-로봇 마커로 카메라 자세 → 지면투영 (코너4점 기반, 강건판).

wobble_test 개선: 마커 '중심 3점' PnP(불량조건/다중해)가 아니라, **마커별 코너4점**으로
개별 pose(IPPE_SQUARE, 45mm 정사각형)를 풀어 카메라프레임 중심을 얻고, 로봇프레임 중심
(robot_markers.json)과 **강체정합(Kabsch)**. 검증: 카메라 높이 ≈ 실제 광각 높이(≈88.5cm),
그리고 아는 위치의 바닥 마커 투영이 실측과 맞는지.

  python3 scripts/wide_pose.py                 # 자세 + (검출된 모든 바닥마커) 지면투영
  python3 scripts/wide_pose.py --truth 21:45,-2  # 특정 마커 실측(앞,좌 cm)과 오차 대조
"""
from __future__ import annotations
import argparse, json, sys
import numpy as np
import cv2

sys.path.insert(0, "/home/seventt/seventt/workspace/scripts")
from csi_capture import CsiCamera  # noqa: E402

WK = np.array([[857.316, 0, 789.858], [0, 860.071, 541.748], [0, 0, 1]], np.float64)
WD = np.array([-0.085721, 0.053910, -0.034649, 0.007943], np.float64).reshape(4, 1)
MARKERS = "/home/seventt/seventt/workspace/data/calib/robot_markers.json"
C_REF = np.array([-0.10, 0.06, 0.85])   # 실측 카메라 위치(robot base): 뒤10 왼6 높이85 — 검증용


def rigid(A, B):
    """B = R@A + t 를 만드는 강체 R,t (A,B: Nx3). Kabsch."""
    ca, cb = A.mean(0), B.mean(0)
    Hm = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(Hm)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cb - R @ ca


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--balance", type=float, default=0.5)
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--truth", default="", help="id:앞,좌  (cm) 실측 대조 예 21:45,-2")
    args = ap.parse_args()

    md = json.load(open(MARKERS))
    size = float(md.get("marker_size_m", 0.045))
    known = {int(k): np.array(v["xyz_m"], np.float64) for k, v in md["markers"].items()}
    truth = {}
    if args.truth:
        i, xy = args.truth.split(":")
        truth[int(i)] = tuple(float(t) for t in xy.split(","))

    p = cv2.aruco.DetectorParameters()
    p.minMarkerPerimeterRate = 0.01
    p.adaptiveThreshWinSizeMax = 45
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), p)

    cam = CsiCamera(1, wb_gains=None, flip=2, wbmode=8)
    W, Hh = 1640, 1232
    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(WK, WD, (W, Hh), np.eye(3), balance=args.balance)
    maps = cv2.fisheye.initUndistortRectifyMap(WK, WD, np.eye(3), newK, (W, Hh), cv2.CV_16SC2)

    # 코너 누적(마커별 4코너 평균) — rectified 픽셀
    acc, cnt, rect_last = {}, {}, None
    for _ in range(args.frames + 3):
        f = cam.read(1.0)
        if isinstance(f, tuple):
            f = f[0]
        if f is None:
            continue
        rect = cv2.remap(f, maps[0], maps[1], cv2.INTER_LINEAR)
        rect_last = rect
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue
        for c, i in zip(corners, ids.flatten()):
            i = int(i)
            acc[i] = acc.get(i, np.zeros((4, 2))) + c.reshape(4, 2)
            cnt[i] = cnt.get(i, 0) + 1
    cam.release()
    if rect_last is not None:   # 디버그: 마지막 rectify에 검출마커 표시 + 온로봇 검출횟수
        dbg = rect_last.copy()
        for i in acc:
            c = acc[i] / cnt[i]
            col = (0, 255, 0) if i in known else (0, 165, 255)   # 온로봇=초록, 바닥=주황
            cv2.polylines(dbg, [c.astype(int)], True, col, 2)
            cv2.putText(dbg, f"{i}({cnt[i]})", (int(c[:, 0].mean()) + 6, int(c[:, 1].mean())),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        miss = [i for i in known if i not in acc]
        cv2.putText(dbg, f"on-robot missing {miss}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        dp = "/home/seventt/seventt/workspace/data/calib/wide_pose_debug.png"
        cv2.imwrite(dp, dbg); print(f"디버그(rectify) -> {dp}  (초록=온로봇, 주황=바닥, 괄호=검출프레임수)")
    th_ground = max(2, args.frames // 2)
    # 온-로봇 마커는 결정적이라 문턱 낮춤(2프레임). 바닥은 절반 이상.
    corners_px = {i: acc[i] / cnt[i] for i in acc if cnt[i] >= (2 if i in known else th_ground)}
    onrb = sorted(i for i in corners_px if i in known)
    print(f"안정검출: {sorted(corners_px)}  (온로봇 {onrb} / 검출횟수 "
          + ", ".join(f"{i}:{cnt[i]}/{args.frames+3}" for i in sorted(known) if i in cnt) + ")")

    # IPPE_SQUARE 로컬 정사각(코너순서 = ArUco: TL,TR,BR,BL)
    s = size / 2.0
    objp = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)

    def marker_center_cam(px):
        ok, rvec, tvec = cv2.solvePnP(objp, px.reshape(4, 1, 2), newK, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return tvec.ravel() if ok else None

    seen = [i for i in known if i in corners_px]
    if len(seen) < 3:
        print(f"온-로봇 마커 3개 필요(현재 {len(seen)})"); return 1
    P_robot = np.array([known[i] for i in seen])
    P_cam = np.array([marker_center_cam(corners_px[i]) for i in seen])
    for i, pc in zip(seen, P_cam):
        print(f"  온로봇 ID {i}: 카메라프레임 거리 {np.linalg.norm(pc)*100:.1f}cm")

    R, t = rigid(P_robot, P_cam)          # P_cam = R@P_robot + t  (R: robot->cam)
    C = -R.T @ t                          # 카메라 위치(robot frame)
    print(f"\n카메라 위치(robot frame): 앞{C[0]*100:+.1f} 왼{C[1]*100:+.1f} 높이{C[2]*100:+.1f} cm")
    print(f"  실측기대            : 앞{C_REF[0]*100:+.1f} 왼{C_REF[1]*100:+.1f} 높이{C_REF[2]*100:+.1f} cm")
    d = (C - C_REF) * 100
    dist = float(np.linalg.norm(d))
    print(f"  → 차이 앞{d[0]:+.1f} 왼{d[1]:+.1f} 높이{d[2]:+.1f} cm (총 {dist:.1f}cm)  "
          + ("✅ 정상" if dist < 8 else "⚠ 여전히 이상 — 마커 배치/검출/실측 확인"))

    def project(u, v):   # rectified 픽셀 → z=0 지면 (robot x앞,y좌)
        ray = R.T @ np.array([(u - newK[0, 2]) / newK[0, 0], (v - newK[1, 2]) / newK[1, 1], 1.0])
        if abs(ray[2]) < 1e-6 or (C[2] > 0) == (ray[2] > 0):
            return None
        g = C + (-C[2] / ray[2]) * ray
        return g[0], g[1]

    print("\n== 바닥 마커 지면투영 (robot 앞x/좌y cm) ==")
    for i in sorted(corners_px):
        if i in known:
            continue
        u, v = corners_px[i].mean(0)
        g = project(u, v)
        if g is None:
            print(f"  ID {i}: 투영불가"); continue
        px, py = g[0] * 100, g[1] * 100
        if i in truth:
            tx, ty = truth[i]
            e = ((px - tx) ** 2 + (py - ty) ** 2) ** 0.5
            print(f"  ★ID {i} → (앞{px:+.1f}, 좌{py:+.1f})  실측(앞{tx:+.0f}, 좌{ty:+.0f})  오차 {e:.1f}cm")
        else:
            print(f"    ID {i} → (앞{px:+.1f}, 좌{py:+.1f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
