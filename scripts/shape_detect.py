#!/usr/bin/env python3
"""본체 카메라로 흰색 다면체를 윤곽/면 기반으로 검출·구분 (YOLO 불필요).

YOLO-World가 흰 추상 도형을 못 잡으므로 고전 CV로 처리한다:
  1) 흰 물체 분할   — HSV 저채도(S)+명도(V) 마스크 + 모폴로지 + (solidity/크기/테두리) 필터
  2) 면 분리        — 물체 내부를 '면 경계 밝기단차(Canny 엣지)'로 쪼개 면 영역들을 얻음
                      (흰색 binary로는 빛받은면/그림자면이 한 덩어리로 합쳐져 못 나눔)
  3) 도형 구분      — 가장 큰 면의 꼭짓점 수 + 보이는 면 개수:
        면=사각형(4) → cube
        면=오각형(5) → dodecahedron
        면=삼각형(3) → 면 많으면 icosahedron, 적으면 octahedron
        (면을 못 나누면 실루엣 변 수로 폴백 — 단 불확실)

입력 3종:
  python3 scripts/shape_detect.py --image data/cam_test/closeup.png
  python3 scripts/shape_detect.py --shot /tmp/shape.jpg
  DISPLAY=:1 python3 scripts/shape_detect.py            # 라이브 (q/ESC, m=마스크, f=면)
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402

_FACE_COLORS = [(0, 255, 255), (255, 0, 255), (255, 255, 0), (0, 165, 255),
                (128, 255, 0), (255, 128, 0), (0, 0, 255), (0, 255, 128)]


# ----------------------------------------------------------------- segmentation
def segment_white(frame, s_max, v_min, min_area, max_area_frac, min_solidity):
    """흰 물체 후보 외곽 컨투어 + 디버그 마스크. 벽/바닥은 (solidity/크기/테두리)로 제거."""
    h_img, w_img = frame.shape[:2]
    max_area = max_area_frac * h_img * w_img
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s, v = hsv[..., 1], hsv[..., 2]
    mask = (((s < s_max) & (v > v_min)).astype(np.uint8)) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)  # 그림자면까지 메움
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    objs = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_area or a > max_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if x <= 1 or y <= 1 or x + w >= w_img - 1 or y + h >= h_img - 1:
            continue                                   # 테두리 접촉 = 벽/바닥
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0 or a / hull_area < min_solidity:
            continue
        objs.append(c)
    objs.sort(key=cv2.contourArea, reverse=True)
    return objs, mask


# ----------------------------------------------------------------- face analysis
def segment_faces(gray, obj_mask, obj_area, canny_lo, canny_hi, k=4):
    """물체 내부를 '밝기 군집(k-means)'으로 면 단위로 쪼갠다.

    면마다 명암이 달라(빛받은 윗면 vs 그림자 옆면) 밝기로 군집하면 면이 갈린다.
    엣지 단차가 약해 연결성분이 안 끊기는 문제를 우회. 각 군집을 공간적으로 다시
    연결성분 분리해 면 영역 컨투어들을 반환.
    """
    ys, xs = np.where(obj_mask > 0)
    if len(xs) < 50:
        return [], None
    vals = gray[ys, xs].astype(np.float32).reshape(-1, 1)
    K = int(min(k, max(2, len(np.unique(vals.astype(np.uint8))))))
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    _, labels, _ = cv2.kmeans(vals, K, None, crit, 3, cv2.KMEANS_PP_CENTERS)
    labels = labels.flatten()

    faces = []
    kern = np.ones((5, 5), np.uint8)
    for ki in range(K):
        comp = np.zeros(gray.shape, np.uint8)
        comp[ys[labels == ki], xs[labels == ki]] = 255
        comp = cv2.morphologyEx(comp, cv2.MORPH_OPEN, kern)   # 군집 잡음 제거
        comp = cv2.morphologyEx(comp, cv2.MORPH_CLOSE, kern)
        nn, lab2, st2, _ = cv2.connectedComponentsWithStats(comp, 8)
        for i in range(1, nn):
            if st2[i, cv2.CC_STAT_AREA] < obj_area * 0.05:
                continue
            c2 = (lab2 == i).astype(np.uint8) * 255
            cs, _ = cv2.findContours(c2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cs:
                faces.append(max(cs, key=cv2.contourArea))
    faces.sort(key=cv2.contourArea, reverse=True)
    return faces, None


def poly_n(contour, eps_ratio):
    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return 0, None
    approx = cv2.approxPolyDP(contour, eps_ratio * peri, True)
    return len(approx), approx


def classify(silhouette, gray, obj_mask, eps_ratio, canny_lo, canny_hi):
    """한 물체 → (label, conf, detail). 면 꼭짓점+면 개수 기반, 폴백=실루엣."""
    obj_area = cv2.contourArea(silhouette)
    peri = cv2.arcLength(silhouette, True)
    n_sil, sil_approx = poly_n(silhouette, eps_ratio)
    circularity = float(4 * np.pi * obj_area / (peri * peri)) if peri > 0 else 0.0
    hull_area = cv2.contourArea(cv2.convexHull(silhouette))
    solidity = max(0.0, min(1.0, obj_area / hull_area)) if hull_area > 0 else 0.0

    faces, edges = segment_faces(gray, obj_mask, obj_area, canny_lo, canny_hi)
    n_faces = len(faces)
    big = faces[0] if faces else None
    n_face, face_approx = poly_n(big, eps_ratio) if big is not None else (0, None)

    # 작은 삼각면 개수 (icosa 판별 보조)
    tri_faces = 0
    for f in faces:
        nf, _ = poly_n(f, eps_ratio)
        if nf == 3:
            tri_faces += 1

    used = "face"
    if n_face == 4:
        label, vfit = "cube", 1.0
    elif n_face == 5:
        label, vfit = "dodecahedron", 1.0
    elif n_face == 3:
        label = "icosahedron" if (tri_faces >= 4 or n_faces >= 6) else "octahedron"
        vfit = 1.0
    else:
        # 면을 못 셈 → 실루엣 폴백 (불확실)
        used = "silhouette"
        ideals = {3: "octahedron", 4: "cube", 5: "dodecahedron"}
        best = min(ideals, key=lambda kk: abs(kk - n_sil))
        label = ideals[best]
        vfit = max(0.0, 1.0 - abs(n_sil - best) / float(max(best, 1))) * 0.5  # 폴백 신뢰 절반

    conf = max(0.0, min(1.0, 0.4 * solidity + 0.6 * vfit))
    detail = dict(n_sil=n_sil, n_face=n_face, n_faces=n_faces, tri_faces=tri_faces,
                  circ=round(circularity, 2), sol=round(solidity, 2), used=used,
                  faces=faces, face_approx=face_approx, sil_approx=sil_approx)
    return label, conf, detail


def analyze(frame, s_max, v_min, min_area, eps_ratio, max_area_frac, min_solidity,
            canny_lo, canny_hi):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    objs, mask = segment_white(frame, s_max, v_min, min_area, max_area_frac, min_solidity)
    results = []
    for c in objs:
        obj_mask = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(obj_mask, [c], -1, 255, -1)
        label, conf, detail = classify(c, gray, obj_mask, eps_ratio, canny_lo, canny_hi)
        results.append((c, label, conf, detail))
    return results, mask


# ----------------------------------------------------------------------- drawing
def draw(frame, results, show_faces=True):
    for c, label, conf, d in results:
        if show_faces:
            for i, f in enumerate(d["faces"]):
                cv2.drawContours(frame, [f], -1, _FACE_COLORS[i % len(_FACE_COLORS)], 2)
        cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)            # 실루엣=초록
        x, y, w, h = cv2.boundingRect(c)
        txt = f"{label} {conf:.2f} | faces={d['n_faces']} bigface={d['n_face']} sil={d['n_sil']} [{d['used']}]"
        yt = max(14, y - 8)
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (x, yt - th - 6), (x + tw + 4, yt), (0, 255, 0), -1)
        cv2.putText(frame, txt, (x + 2, yt - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 2, cv2.LINE_AA)
    return frame


def _report(results):
    for _, label, conf, d in results:
        print(f"  {label:13s} conf={conf:.2f}  faces={d['n_faces']} bigface={d['n_face']} "
              f"tri={d['tri_faces']} sil={d['n_sil']} circ={d['circ']} sol={d['sol']} [{d['used']}]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="저장된 이미지 파일 처리")
    ap.add_argument("--shot", help="카메라 한 장 잡아 처리·저장")
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--s-max", type=int, default=70, help="흰색 최대 채도")
    ap.add_argument("--v-min", type=int, default=120, help="흰색 최소 명도")
    ap.add_argument("--min-area", type=int, default=2500)
    ap.add_argument("--max-area-frac", type=float, default=0.30, help="물체 최대 면적(프레임 대비)")
    ap.add_argument("--min-solidity", type=float, default=0.85)
    ap.add_argument("--eps", type=float, default=0.03, help="approxPolyDP epsilon 비율")
    ap.add_argument("--canny-lo", type=int, default=30)
    ap.add_argument("--canny-hi", type=int, default=90)
    ap.add_argument("--show-width", type=int, default=1000)
    args = ap.parse_args()

    def run(frame):
        return analyze(frame, args.s_max, args.v_min, args.min_area, args.eps,
                       args.max_area_frac, args.min_solidity, args.canny_lo, args.canny_hi)

    # ---- 저장 이미지 ----
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"읽기 실패: {args.image}"); return 1
        results, mask = run(frame)
        draw(frame, results)
        out = os.path.splitext(args.image)[0] + "_shapes.png"
        cv2.imwrite(out, frame)
        cv2.imwrite(os.path.splitext(args.image)[0] + "_mask.png", mask)
        _report(results)
        print(f"objects={len(results)} -> {out}")
        return 0

    from csi_capture import CsiCamera
    cam = CsiCamera(args.sensor_id)
    for _ in range(12):
        cam.read(1.0)

    # ---- 한 장 ----
    if args.shot:
        frame = None
        for _ in range(5):
            f = cam.read(1.0)
            if f is not None:
                frame = f
        cam.release()
        if frame is None:
            print("캡처 실패"); return 1
        results, _ = run(frame)
        draw(frame, results)
        cv2.imwrite(args.shot, frame)
        _report(results)
        print(f"objects={len(results)} -> {args.shot}")
        return 0

    # ---- 라이브 ----
    win = "shape-detect (q/ESC quit, m=mask, f=faces)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    show_mask = False; show_faces = True
    t_prev = time.time(); fps = 0.0
    try:
        while True:
            frame = cam.read(1.0)
            if frame is None:
                continue
            results, mask = run(frame)
            draw(frame, results, show_faces)
            now = time.time(); fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3)); t_prev = now
            hud = f"{fps:4.1f} FPS | {len(results)} obj | s<{args.s_max} v>{args.v_min}"
            cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            disp = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask else frame
            if args.show_width and disp.shape[1] > args.show_width:
                sc = args.show_width / disp.shape[1]
                disp = cv2.resize(disp, (args.show_width, int(disp.shape[0] * sc)))
            cv2.imshow(win, disp)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("m"):
                show_mask = not show_mask
            if k == ord("f"):
                show_faces = not show_faces
    finally:
        cam.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
