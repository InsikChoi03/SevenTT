#!/usr/bin/env python3
"""광각(top) 카메라 지면투영 정렬 도구 — world_model 과 똑같은 수학으로 base-frame 위치 확인.

로봇 기준 base frame: x=앞(front), y=왼(left)  [REP-103].
물체를 로봇 **정면 ~0.5m**에 놓고 --sweep 하면, yaw(0/90/180/270) × rot180(T/F) 8조합에서
그 물체가 어디로 투영되는지 표로 나옴. 물체가 **(+x, ~0) = 앞**으로 가는 조합이 정답.

  python3 scripts/wide_align.py --sweep                 # 8조합 한 프레임 비교 (정렬 찾기)
  python3 scripts/wide_align.py --yaw 90 --shot /tmp/a.png   # 특정 조합 + 주석이미지 저장
"""
from __future__ import annotations
import argparse, math, sys
import numpy as np
import cv2

sys.path.insert(0, "/home/seventt/seventt/workspace/scripts")
from csi_capture import CsiCamera  # noqa: E402

# --- 캘리브 (test_field.yaml world_model_node, 회전프레임 fisheye) ---
FX, FY, CX, CY = 857.316, 860.071, 789.858, 541.748
DIST = np.array([-0.085721, 0.053910, -0.034649, 0.007943], np.float64).reshape(4, 1)
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], np.float64)
CAM_H, PITCH, OFF_X, OFF_Y, OBJ_H = 0.885, 88.0, 0.204, 0.052, 0.04
MODEL = "/home/seventt/seventt/workspace/models/wide.pt"


def _build_R_base_cam(pitch_deg):
    p = math.radians(pitch_deg); cp, sp = math.cos(p), math.sin(p)
    return np.array([[0.0, -sp, cp], [-1.0, 0.0, 0.0], [0.0, -cp, -sp]], np.float64)


def _rz(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], np.float64)


def project(u, v, yaw_deg, rot180):
    """광각 픽셀 (u,v) → base-frame (x앞, y왼) m. 로봇 원점·theta=0 가정."""
    und = cv2.fisheye.undistortPoints(np.array([[[u, v]]], np.float64), K, DIST)
    xn, yn = und[0, 0]
    if rot180:
        xn, yn = -xn, -yn
    d_cam = np.array([xn, yn, 1.0], np.float64)
    R = _rz(math.radians(yaw_deg)) @ _build_R_base_cam(PITCH)
    ray = R @ d_cam
    pcam = np.array([OFF_X, OFF_Y, CAM_H], np.float64)
    if ray[2] >= -1e-6:
        return None
    t = (OBJ_H - pcam[2]) / ray[2]
    if t <= 0.0:
        return None
    return (pcam[0] + t * ray[0], pcam[1] + t * ray[1])


def fbir(x, y):
    """앞/뒤 · 좌/우 힌트."""
    fb = "front" if x > 0.05 else ("back" if x < -0.05 else "—")
    lr = "LEFT" if y > 0.05 else ("RIGHT" if y < -0.05 else "—")
    return f"{fb:>5} {lr:>5}"


_CLR = [(80, 200, 80), (0, 150, 255), (200, 120, 60), (255, 90, 40), (180, 80, 220),
        (60, 220, 220), (120, 120, 255), (240, 240, 120)]


def render_map(dets, yaw, rot, size=300, half_m=2.0):
    """base-frame 위=앞(+x), 왼=왼(+y). 로봇 중앙, ±half_m."""
    img = np.full((size, size, 3), 28, np.uint8)
    cx = cy = size // 2
    sc = (size / 2 - 16) / half_m
    cv2.line(img, (cx, 0), (cx, size), (60, 60, 60), 1)
    cv2.line(img, (0, cy), (size, cy), (60, 60, 60), 1)
    # 로봇: 중앙에서 위(앞) 향한 삼각형
    cv2.drawMarker(img, (cx, cy), (255, 90, 40), cv2.MARKER_TRIANGLE_UP, 16, 2)
    cv2.putText(img, "FRONT", (cx - 22, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
    cv2.putText(img, "LEFT", (2, cy - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 140), 1)
    for idx, lbl, c, u, v in dets:
        xy = project(u, v, yaw, rot)
        if not xy:
            continue
        x, y = xy
        px = int(cx - y * sc)   # +y(왼) -> 화면 왼쪽
        py = int(cy - x * sc)   # +x(앞) -> 화면 위쪽
        if 0 <= px < size and 0 <= py < size:
            col = _CLR[idx % len(_CLR)]
            cv2.circle(img, (px, py), 6, col, -1)
            cv2.putText(img, str(idx), (px + 6, py + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
    cv2.putText(img, f"yaw={yaw} rot180={rot}", (6, size - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)
    return img


def render_grid(frame, dets, out_path):
    """카메라(박스+번호) + 8조합 2D지도 그리드 를 한 장으로 저장."""
    cam = frame.copy()
    for idx, lbl, c, u, v in dets:
        cv2.circle(cam, (int(u), int(v)), 6, _CLR[idx % len(_CLR)], -1)
        cv2.putText(cam, f"{idx}:{lbl[:4]}", (int(u) + 6, int(v) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, _CLR[idx % len(_CLR)], 2)
    S = 300
    maps = [render_map(dets, yaw, rot, S) for rot in (1, 0) for yaw in (0, 90, 180, 270)]
    row1 = np.hstack(maps[0:4]); row2 = np.hstack(maps[4:8])
    grid = np.vstack([row1, row2])                       # 1200 x 600
    camr = cv2.resize(cam, (grid.shape[1], int(cam.shape[0] * grid.shape[1] / cam.shape[1])))
    comp = np.vstack([camr, grid])
    cv2.imwrite(out_path, comp)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaw", type=float, default=180.0)
    ap.add_argument("--rot180", type=int, default=1)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--shot", default="")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(MODEL)
    cam = CsiCamera(1, wb_gains=None, flip=2)   # WIDE sensor1, 180 flip (camera_top 과 동일)
    for _ in range(6):
        cam.read(1.0)                            # 3A 안정화
    frame = cam.read(1.5)
    cam.release()
    if frame is None:
        print("카메라 프레임 없음"); return 1
    if isinstance(frame, tuple):
        frame = frame[0]

    res = model.predict(frame, conf=args.conf, imgsz=640, verbose=False)[0]
    dets = []
    if res.boxes is not None and len(res.boxes):
        names = res.names
        xywh = res.boxes.xywh.cpu().numpy()
        cls = res.boxes.cls.cpu().numpy()
        cf = res.boxes.conf.cpu().numpy()
        for i in range(len(xywh)):
            xc, yc, w, h = xywh[i]
            u = float(xc); v = float(yc) + 0.25 * float(h)   # world_model 과 동일 접점픽셀
            dets.append((i, str(names[int(cls[i])]), float(cf[i]), u, v))
    print(f"검출 {len(dets)}개")

    out = args.shot or "/tmp/wide_align.png"
    render_grid(frame, dets, out)
    print(f"이미지 -> {out}   (위=카메라+번호, 아래=8조합 2D지도[위=앞,왼=왼])")
    print(f"\n현재 배포값 yaw={args.yaw} rot180={args.rot180} 투영:")
    for idx, lbl, c, u, v in dets:
        xy = project(u, v, args.yaw, bool(args.rot180))
        s = f"({xy[0]:+.2f},{xy[1]:+.2f}) {fbir(*xy)}" if xy else "투영불가"
        print(f"  [{idx}] {lbl:16s} conf={c:.2f}  base={s}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
