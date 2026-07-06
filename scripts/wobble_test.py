#!/usr/bin/env python3
"""온-로봇 ArUco 마커로 카메라 자세 추정 → 물체 지면투영(로봇 base 실좌표) 검증.

robot_markers.json(마커 중심 3D, 로봇 base frame) + 광각 fisheye intrinsics 로 매 프레임
solvePnP(마커 중심 3점 SQPNP) → 카메라↔로봇 자세. 그 자세로 YOLO 물체의 바닥픽셀을 z=0 지면에
투영해 (x앞,y왼) 실좌표를 찍는다. **정면 0.5m 등 아는 위치에 물체 두고 값 맞는지 확인용.**

  python3 scripts/wobble_test.py               # 마커+물체 검출 → 물체 실좌표 출력 + 주석이미지
  python3 scripts/wobble_test.py --no-yolo      # 카메라 자세(카메라 위치)만 확인
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
MODEL = "/home/seventt/seventt/workspace/models/wide.pt"


def undist(u, v):
    p = cv2.fisheye.undistortPoints(np.array([[[u, v]]], np.float64), WK, WD)
    return float(p[0, 0, 0]), float(p[0, 0, 1])


def detect_markers(cam, detector, maps, frames=8):
    """어안 rectify 후 ArUco 검출(주변부 마커도 잡히게). 중심은 RECTIFIED 픽셀. raw/rect 마지막프레임."""
    m1, m2 = maps
    acc, cnt, raw_last, rect_last = {}, {}, None, None
    for _ in range(frames + 3):
        f = cam.read(1.0)
        if isinstance(f, tuple):
            f = f[0]
        if f is None:
            continue
        raw_last = f
        rect = cv2.remap(f, m1, m2, cv2.INTER_LINEAR)
        rect_last = rect
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue
        for c, i in zip(corners, ids.flatten()):
            i = int(i)
            acc[i] = acc.get(i, np.zeros((4, 2))) + c.reshape(4, 2)
            cnt[i] = cnt.get(i, 0) + 1
    centers = {i: (acc[i] / cnt[i]).mean(axis=0) for i in acc}
    return centers, raw_last, rect_last


def project_ground(R, C, u, v):
    """카메라↔로봇(R=robot->cam, C=cam center in robot) + 픽셀 → z=0 지면 (x,y) robot."""
    xn, yn = undist(u, v)
    ray = R.T @ np.array([xn, yn, 1.0])     # camera ray -> robot frame
    if abs(ray[2]) < 1e-6 or (C[2] > 0) == (ray[2] > 0):
        return None                          # ray가 지면 방향 아님
    s = -C[2] / ray[2]
    g = C + s * ray
    return float(g[0]), float(g[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-yolo", action="store_true")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--dict", default="4x4_50")
    ap.add_argument("--balance", type=float, default=0.5, help="rectify FOV (1=넓게 유지, 0=크롭)")
    args = ap.parse_args()

    md = json.load(open(MARKERS))
    known = {int(k): np.array(v["xyz_m"], np.float64) for k, v in md["markers"].items()}
    dict_id = getattr(cv2.aruco, "DICT_" + args.dict.upper(), cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.minMarkerPerimeterRate = 0.01          # 작은 마커도 검출(멀거나 45mm)
    params.adaptiveThreshWinSizeMax = 45
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dict_id), params)
    cam = CsiCamera(1, wb_gains=None, flip=2, wbmode=8)
    model = None if args.no_yolo else __import__("ultralytics").YOLO(MODEL)
    W, H = 1640, 1232
    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        WK, WD, (W, H), np.eye(3), balance=args.balance)
    maps = cv2.fisheye.initUndistortRectifyMap(WK, WD, np.eye(3), newK, (W, H), cv2.CV_16SC2)
    centers, frame, rect = detect_markers(cam, detector, maps)

    def nrm(u, v):   # rectified 픽셀 -> 정규 카메라 광선 (object의 fisheye.undistort와 동일 프레임)
        return ((u - newK[0, 2]) / newK[0, 0], (v - newK[1, 2]) / newK[1, 1])

    seen = [i for i in centers if i in known]
    missing = [i for i in known if i not in centers]
    print(f"온-로봇 마커 검출: {sorted(centers)}  (알려진 검출 {sorted(seen)}, 누락 {missing})")
    if rect is not None:   # 디버그: rectify 영상에 검출 마커 표시 (여기서도 안 잡히면 근본적으로 안 보임)
        dbg = rect.copy()
        for i in sorted(centers):
            u, v = centers[i]
            col = (0, 255, 0) if i in known else (0, 165, 255)   # 온로봇=초록, 바닥=주황
            cv2.circle(dbg, (int(u), int(v)), 12, col, 2)
            cv2.putText(dbg, str(i), (int(u) + 10, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.putText(dbg, f"on-robot seen {sorted(seen)}  missing {missing}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        dp = "/home/seventt/seventt/workspace/data/calib/wobble_debug.png"
        cv2.imwrite(dp, dbg)
        print(f"디버그(rectify) -> {dp}  (초록=온로봇, 주황=바닥. 여기서도 누락이면 마커 키우기/각도/조명)")
    if len(seen) < 3:
        print(f"자세추정 최소 3개 필요 (현재 {len(seen)}). 누락 {missing}.")
        cam.release(); return 1

    objp = np.array([known[i] for i in seen], np.float64)
    imgp = np.array([nrm(*centers[i]) for i in seen], np.float64)   # rectified 정규점
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, np.eye(3), None, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        print("solvePnP 실패"); cam.release(); return 1
    R, _ = cv2.Rodrigues(rvec)
    C = (-R.T @ tvec).ravel()               # 카메라 위치 (robot frame)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, np.eye(3), None)
    err = np.linalg.norm(proj.reshape(-1, 2) - imgp, axis=1)
    print(f"카메라 위치(robot frame): 앞{C[0]*100:+.0f} 왼{C[1]*100:+.0f} 높이{C[2]*100:+.0f} cm "
          f"(높이가 실제 광각 높이와 비슷해야 정상)")
    print(f"마커 재투영오차(정규): 평균 {err.mean():.4f} 최대 {err.max():.4f}")

    if model is not None:
        r = model.predict(frame, conf=args.conf, imgsz=640, verbose=False)[0]
        ann = frame.copy()
        print("\n== 물체 지면투영 (로봇 base 앞x/왼y cm) ==")
        if r.boxes is not None and len(r.boxes):
            names = r.names
            for xywh, cl, cf in zip(r.boxes.xywh.cpu().numpy(), r.boxes.cls.cpu().numpy(),
                                    r.boxes.conf.cpu().numpy()):
                xc, yc, w, h = xywh
                u = float(xc); v = float(yc) + 0.5 * float(h)   # 바닥접점
                g = project_ground(R, C, u, v)
                lbl = str(names[int(cl)])
                if g:
                    print(f"  {lbl:16s} conf={cf:.2f}  (x={g[0]*100:+.0f}cm, y={g[1]*100:+.0f}cm)")
                    cv2.circle(ann, (int(u), int(v)), 6, (0, 255, 0), -1)
                    cv2.putText(ann, f"{lbl[:4]} ({g[0]*100:+.0f},{g[1]*100:+.0f})",
                                (int(u) + 6, int(v) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    print(f"  {lbl:16s} conf={cf:.2f}  투영불가")
        p = "/home/seventt/seventt/workspace/data/calib/wobble_test.png"
        cv2.imwrite(p, ann); print(f"\n주석이미지 -> {p}")
    cam.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
