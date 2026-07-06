#!/usr/bin/env python3
"""본체캠 지면 호모그래피 — 한 장에 인쇄된 격자시트(20~25) + 2점 실측 앵커.

20~25는 한 종이(rigid 격자)에 인쇄돼 바닥에 붙음. 사용자가 그중 2개(24,25) 실좌표만 실측.
종이의 알려진 상대격자에 2점 유사변환(회전+평행+스케일)을 맞춰 6개 전부 base_link(x앞,y좌)
좌표를 계산 → 본체캠 검출 픽셀과 findHomography → data/calib/body_ground.npz 저장.

  python3 scripts/body_calib_sheet.py            # 아래 하드코딩 앵커/템플릿으로 실행
"""
from __future__ import annotations
import os, sys
import numpy as np
import cv2

sys.path.insert(0, "/home/seventt/seventt/workspace/scripts")
from csi_capture import CsiCamera  # noqa: E402

OUT = "/home/seventt/seventt/workspace/data/calib/body_ground.npz"

# 종이의 상대 격자 (24 기준, cm) 앞=+x 왼=+y — 왼열 20/22/24, 오른열 21/23/25, 8cm 간격
TEMPLATE = {24: (0.0, 0.0), 25: (0.0, -8.0),
            22: (8.0, 0.0), 23: (8.0, -8.0),
            20: (16.0, 0.0), 21: (16.0, -8.0)}
# 사용자 실측 앵커 (앞x, 왼y) cm : 25는 '오른5.8'=왼-5.8
ANCHORS = {24: (22.5, 2.5), 25: (23.0, -5.8)}
FRAMES = 12


def fit_similarity(tpl_pts, meas_pts):
    """tpl -> meas 유사변환(scale*R + t). 복소수법(2점 정확, N점 최소제곱)."""
    a = np.array([complex(x, y) for x, y in tpl_pts])
    b = np.array([complex(x, y) for x, y in meas_pts])
    ca, cb = a.mean(), b.mean()
    c = ((b - cb) * (a - ca).conjugate()).sum() / (abs(a - ca) ** 2).sum()  # scale*rot
    t = cb - c * ca
    return c, t


def main():
    tids = [i for i in ANCHORS]
    c, t = fit_similarity([TEMPLATE[i] for i in tids], [ANCHORS[i] for i in tids])
    print(f"유사변환: 스케일 {abs(c):.4f}  회전 {np.degrees(np.angle(c)):+.2f}°")
    base_xy = {}
    for i, (tx, ty) in TEMPLATE.items():
        z = c * complex(tx, ty) + t
        base_xy[i] = (z.real, z.imag)
    print("종이 6마커 계산 base_link (앞x, 좌y) cm:")
    for i in sorted(base_xy):
        an = " (앵커)" if i in ANCHORS else ""
        print(f"  ID {i}: 앞{base_xy[i][0]:+.1f} 좌{base_xy[i][1]:+.1f}{an}")

    # 본체캠 검출
    p = cv2.aruco.DetectorParameters()
    p.minMarkerPerimeterRate = 0.01
    p.adaptiveThreshWinSizeMax = 45
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), p)
    cam = CsiCamera(0, wbmode=8)
    acc, cnt, last = {}, {}, None
    for _ in range(FRAMES + 3):
        f = cam.read(1.0)
        if isinstance(f, tuple):
            f = f[0]
        if f is None:
            continue
        last = f
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue
        for cc, i in zip(corners, ids.flatten()):
            i = int(i)
            acc[i] = acc.get(i, np.zeros((4, 2))) + cc.reshape(4, 2)
            cnt[i] = cnt.get(i, 0) + 1
    cam.release()
    th = max(2, FRAMES // 2)
    px = {i: acc[i] / cnt[i] for i in acc if cnt[i] >= th}
    used = [i for i in px if i in base_xy]
    print(f"\n본체캠 검출: {sorted(px)}  (호모그래피 사용 {sorted(used)})")
    if len(used) < 4:
        print(f"✗ 시트 마커 4개 이상 필요(현재 {len(used)}). 본체캠 시야에 20~25 더 들어오게."); return 1

    src = np.array([px[i].mean(0) for i in used], np.float64).reshape(-1, 1, 2)
    dstm = np.array([[base_xy[i][0] / 100.0, base_xy[i][1] / 100.0] for i in used], np.float64).reshape(-1, 1, 2)
    Hm, _ = cv2.findHomography(src, dstm, 0)
    if Hm is None:
        print("✗ 호모그래피 실패 — 마커 일직선?"); return 1
    proj = cv2.perspectiveTransform(src, Hm).reshape(-1, 2)
    err = np.linalg.norm(proj - dstm.reshape(-1, 2), axis=1) * 100
    print(f"\n재현오차 평균 {err.mean():.2f}cm 최대 {err.max():.2f}cm")
    for i, e in zip(used, err):
        print(f"  ID {i} 오차 {e:.2f}cm")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    np.savez(OUT, H=Hm)
    print(f"저장 -> {OUT}")

    if last is not None:
        dbg = last.copy()
        for i in used:
            ctr = px[i].mean(0)
            cv2.circle(dbg, (int(ctr[0]), int(ctr[1])), 10, (0, 255, 0), 2)
            cv2.putText(dbg, f"{i}({base_xy[i][0]:+.0f},{base_xy[i][1]:+.0f})",
                        (int(ctr[0]) + 8, int(ctr[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        dp = "/home/seventt/seventt/workspace/data/calib/body_ground_debug.png"
        cv2.imwrite(dp, dbg); print(f"디버그 -> {dp}")
    print("\n→ 재현오차 3cm 이하면 OK. test_field.yaml body_ground_homography_path 주석해제 후 빌드.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
