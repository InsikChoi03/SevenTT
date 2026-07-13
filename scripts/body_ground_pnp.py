#!/usr/bin/env python3
"""본체캠 지면 호모그래피 — 마커 '크기'만으로 (손측정 0).

본체캠은 intrinsics(data/calib/body_calib.npz)와 고정위치(앞5.5·왼0·높이15.5cm)를 알므로,
바닥에 ArUco(38mm) 몇 개 깔고 찍으면:
  1) solvePnP(코너4점+크기)로 각 마커 중심을 카메라프레임 미터로 복원(스케일=마커크기)
  2) 마커중심들로 지면평면 fit → 카메라 pitch/roll + 복원높이(15.5 검증)
  3) yaw≈0(정면장착) 가정 → 카메라 자세 완성
  4) 각 마커중심을 base_link(x앞,y좌)로 변환 → (픽셀↔base_link) 호모그래피 fit → 저장
검증: 복원 카메라높이 ≈ 15.5cm, 마커 배치가 상식적인지.

  python3 scripts/body_ground_pnp.py                       # 기본위치(5.5,0,15.5)
  python3 scripts/body_ground_pnp.py --cam-xyz 5.5 0 15.5  # 위치 override (cm)
"""
from __future__ import annotations
import argparse, os, sys
import numpy as np
import cv2

sys.path.insert(0, "/home/seventt/seventt/workspace/scripts")
from csi_capture import CsiCamera  # noqa: E402

CALIB = "/home/seventt/seventt/workspace/data/calib/body_calib.npz"
OUT = "/home/seventt/seventt/workspace/data/calib/body_ground.npz"
MARKER_SIZE = 0.038


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-xyz", type=float, nargs=3, default=[5.5, 0.0, 15.5],
                    help="본체캠 위치 앞 왼 높이 (cm, base_link)")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--size", type=float, default=MARKER_SIZE)
    args = ap.parse_args()
    C = np.array(args.cam_xyz, np.float64) / 100.0     # metres

    cd = np.load(CALIB)
    K, dist = cd["K"].astype(np.float64), cd["dist"].astype(np.float64)
    calW, calH = (int(x) for x in cd["img_size"])

    p = cv2.aruco.DetectorParameters()
    p.minMarkerPerimeterRate = 0.01
    p.adaptiveThreshWinSizeMax = 45
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), p)

    cam = CsiCamera(0, wbmode=8)
    acc, cnt, last = {}, {}, None
    for _ in range(args.frames + 3):
        f = cam.read(1.0)
        if isinstance(f, tuple):
            f = f[0]
        if f is None:
            continue
        last = f
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue
        for c, i in zip(corners, ids.flatten()):
            i = int(i)
            acc[i] = acc.get(i, np.zeros((4, 2))) + c.reshape(4, 2)
            cnt[i] = cnt.get(i, 0) + 1
    cam.release()
    if last is not None and (last.shape[1], last.shape[0]) != (calW, calH):
        print(f"⚠ 프레임 {last.shape[1]}x{last.shape[0]} ≠ 캘리브 {calW}x{calH} — intrinsics 스케일 불일치 위험")
    th = max(2, args.frames // 2)
    corners_px = {i: acc[i] / cnt[i] for i in acc if cnt[i] >= th}
    print(f"안정검출 마커: {sorted(corners_px)} (각 {th}프레임↑)")
    if len(corners_px) < 4:
        print(f"마커 4개 이상 필요(현재 {len(corners_px)}) — 바닥에 더 깔거나 조명 확인"); return 1

    s = args.size / 2.0
    objp = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)  # TL,TR,BR,BL
    ids_sorted = sorted(corners_px)
    tvecs, px_centers = {}, {}
    for i in ids_sorted:
        ok, rvec, tvec = cv2.solvePnP(objp, corners_px[i].reshape(4, 1, 2), K, dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        tvecs[i] = tvec.ravel()                 # 마커중심, 카메라프레임 (m)
        px_centers[i] = corners_px[i].mean(0)   # 마커중심 픽셀(raw)
    P = np.array([tvecs[i] for i in tvecs])
    print("각 마커 카메라프레임 거리: " + ", ".join(f"{i}:{np.linalg.norm(tvecs[i])*100:.0f}cm" for i in tvecs))

    # 지면평면 fit (카메라프레임): 최소 3점, SVD 로 법선
    ctr = P.mean(0)
    _, _, Vt = np.linalg.svd(P - ctr)
    n = Vt[2]                                    # 평면 법선
    if n @ ctr > 0:                              # 카메라(원점) 향하도록: n·(ground점) < 0
        n = -n
    height = float(-(n @ ctr))                   # 카메라→평면 수직거리 = 복원 높이
    dh = abs(height - C[2]) * 100
    print(f"\n복원 카메라높이 {height*100:.1f}cm  (입력 {C[2]*100:.0f}cm, 차이 {dh:.1f}cm) "
          + ("✅" if dh < 5 else "⚠ 마커 비평면/검출 확인"))

    # 자세: up=n, fwd=광축(z_cam)의 지면투영(yaw≈0 가정), left=up×fwd
    up = n / np.linalg.norm(n)
    zc = np.array([0, 0, 1.0])
    fwd = zc - (zc @ up) * up
    fwd /= np.linalg.norm(fwd)
    left = np.cross(up, fwd)
    R_rc = np.column_stack([fwd, left, up])      # robot축 -> camera프레임 (v_cam = R_rc @ v_robot)

    def to_base(pc):                              # 카메라프레임점 -> base_link (x앞,y좌,z위)
        return C + R_rc.T @ pc

    print("\n== 각 마커 base_link 좌표 (크기기반, 손측정 0) ==")
    xy = {}
    for i in tvecs:
        b = to_base(tvecs[i])
        xy[i] = b[:2]
        print(f"  ID {i}: 앞{b[0]*100:+.1f} 좌{b[1]*100:+.1f} (지면 z {b[2]*100:+.1f})")

    # 픽셀(raw) -> base_link 미터 호모그래피
    src = np.array([px_centers[i] for i in tvecs], np.float64).reshape(-1, 1, 2)
    dstm = np.array([xy[i] for i in tvecs], np.float64).reshape(-1, 1, 2)
    Hm, _ = cv2.findHomography(src, dstm, 0)
    if Hm is None:
        print("✗ 호모그래피 실패 — 마커 일직선? 더 퍼뜨려"); return 1
    proj = cv2.perspectiveTransform(src, Hm).reshape(-1, 2)
    err = np.linalg.norm(proj - dstm.reshape(-1, 2), axis=1) * 100
    print(f"\n호모그래피 재현오차 평균 {err.mean():.2f}cm 최대 {err.max():.2f}cm")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    np.savez(OUT, H=Hm, cam_xyz=C, height_recovered=height)
    print(f"저장 -> {OUT}")

    if last is not None:
        dbg = last.copy()
        for i in tvecs:
            c = px_centers[i]
            cv2.circle(dbg, (int(c[0]), int(c[1])), 10, (0, 255, 0), 2)
            cv2.putText(dbg, f"{i}:({xy[i][0]*100:+.0f},{xy[i][1]*100:+.0f})",
                        (int(c[0]) + 8, int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        dp = "/home/seventt/seventt/workspace/data/calib/body_ground_debug.png"
        cv2.imwrite(dp, dbg); print(f"디버그 -> {dp}")
    print("\n→ 검증: 복원높이가 15.5 근처 + 마커 앞좌표가 상식적이면 OK. "
          "test_field.yaml 의 body_ground_homography_path 주석 해제 후 빌드.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
