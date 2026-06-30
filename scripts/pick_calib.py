#!/usr/bin/env python3
"""호모그래피 캘리브 — 본체캠(sensor-id 0) 픽셀 → 실제 테이블 (x,y)cm 변환행렬 H 저장.

본체캠은 베이스 고정이라 H는 한 번 캘리브하면 영구 유효(팔 자세 무관).
좌표계: arm_base 기준 x=앞, y=왼 (cm). 테이블 위 평면.

수집 방법 3가지:
  ① 큐브 자동수집(권장):  python3 pick_calib.py --collect-cube
     큐브를 측정한 위치에 놓고 Enter → cube.pt가 바닥앵커 픽셀 자동검출(pick_run과 동일)
     → 그 위치의 arm_base x y(cm) 입력. ≥4점(테이블 전역에 퍼뜨릴수록 좋음), 끝나면 q.
     클릭 불필요 + 픽업과 동일 픽셀이라 일관성 최고.
  ② GUI 클릭:  python3 pick_calib.py        (로봇에 모니터 필요)
     창에서 알려진 점 클릭(>=4) → q → 각 점의 실제 x y(cm) 입력
  ③ 파일:  python3 pick_calib.py --points data/pick/points.json
     points.json: [{"px":[u,v], "xy":[x,y]}, ...]   (>=4점)

저장: data/pick/homography.npz (H + 스캔이미지 경로).
⚠️ 캘리브 동안 팔은 화면 밖으로 치워두세요(베이스 고정캠이라 자세는 무관, 가림만 방지).
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csi_capture import CsiCamera  # noqa: E402

OUT = "data/pick"
BODY_SID = 0   # 본체 cam = sensor-id 0 (camera_config.BODY). 광각(1)은 천장.


def _save_points(px, xy):
    """수집한 원시 대응점을 즉시 points.json 에 저장(오타 수정/재계산용)."""
    os.makedirs(OUT, exist_ok=True)
    pts = [{"px": [float(p[0]), float(p[1])], "xy": [float(q[0]), float(q[1])]}
           for p, q in zip(px, xy)]
    with open(f"{OUT}/points.json", "w") as f:
        json.dump(pts, f, indent=2, ensure_ascii=False)


def capture():
    cam = CsiCamera(BODY_SID)
    try:
        for _ in range(5):
            cam.read(1.0)
        return cam.read(1.0)
    finally:
        cam.release()


def collect_cube(model_path, conf, cube_cls=0):
    """큐브 바닥앵커를 자동검출하며 대화형으로 (픽셀, 실측 x,y) 수집. pick_run과 동일 앵커."""
    from ultralytics import YOLO  # noqa: E402  (느린 import는 이 모드에서만)
    model = YOLO(model_path)
    cam = CsiCamera(BODY_SID)
    px, xy = [], []
    print("\n큐브 자동수집 모드. 팔은 화면 밖으로. 큐브를 측정한 위치에 놓고 Enter.")
    print("≥4점, 테이블 전역(가까이/멀리/좌/우)에 퍼뜨릴수록 정확. 끝내려면 q.\n")
    try:
        for _ in range(5):
            cam.read(1.0)  # 워밍업
        while True:
            cmd = input(f"[{len(px)}점 수집됨] 큐브 놓고 Enter (u=마지막취소, q=종료): ").strip().lower()
            if cmd == "q":
                break
            if cmd == "u":
                if px:
                    px.pop(); rq = xy.pop()
                    _save_points(px, xy)
                    print(f"  ↩ 마지막 점 취소: ({rq[0]},{rq[1]})cm  → 남은 {len(px)}점")
                else:
                    print("  취소할 점 없음")
                continue
            frame = cam.read(1.0)
            if frame is None:
                print("  ✗ 캡처 실패, 재시도"); continue
            res = model.predict(frame, conf=conf, imgsz=640, verbose=False)
            boxes = res[0].boxes if res else None
            cubes = []
            if boxes is not None and len(boxes):
                xywh = boxes.xywh.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                clses = boxes.cls.cpu().numpy().astype(int)
                cubes = [(xywh[i], float(confs[i])) for i in range(len(clses)) if clses[i] == cube_cls]
            if not cubes:
                print("  ✗ 큐브 못 찾음(조명/거리/가림 확인), 재시도"); continue
            bb, cf = max(cubes, key=lambda t: t[1])
            cxp, cyp, w, h = (float(v) for v in bb)
            u, v = cxp, cyp + h / 2.0   # 바닥중앙 앵커 = 테이블 접점 (pick_run --anchor bottom 과 동일)
            ann = frame.copy()
            cv2.rectangle(ann, (int(cxp - w / 2), int(cyp - h / 2)), (int(cxp + w / 2), int(cyp + h / 2)), (0, 255, 0), 2)
            cv2.circle(ann, (int(u), int(v)), 8, (0, 0, 255), -1)
            os.makedirs(OUT, exist_ok=True)
            cv2.imwrite(f"{OUT}/calib_pt_{len(px)}.png", ann)
            s = input(f"  검출 픽셀=({u:.0f},{v:.0f}) conf={cf:.2f} → 이 위치 arm_base x y(cm): ").split()
            if len(s) < 2:
                print("  ✗ x y 두 값 필요, 건너뜀"); continue
            try:
                fx, fy = float(s[0]), float(s[1])
            except ValueError:
                print("  ✗ 숫자 아님, 건너뜀"); continue
            px.append([u, v]); xy.append([fx, fy])
            _save_points(px, xy)
            print(f"  ✓ 수집 {len(px)}점  (px {u:.0f},{v:.0f} → {fx},{fy}cm)  [points.json 저장됨]")
    finally:
        cam.release()
    return px, xy


def gui_collect(frame):
    pts = []
    disp = frame.copy()

    def onclick(e, x, y, flags, p):
        if e == cv2.EVENT_LBUTTONDOWN:
            pts.append((x, y))
            cv2.circle(disp, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(disp, str(len(pts)), (x + 8, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            print(f"  점 {len(pts)}: 픽셀 ({x},{y})")

    cv2.namedWindow("calib")
    cv2.setMouseCallback("calib", onclick)
    print("알려진 점들을 클릭(>=4). 다 찍으면 'q'.")
    while True:
        cv2.imshow("calib", disp)
        if cv2.waitKey(20) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()
    px, xy = [], []
    for i, (u, v) in enumerate(pts):
        s = input(f"점{i + 1} 픽셀({u},{v}) 의 실제 x y(cm): ").split()
        px.append([u, v]); xy.append([float(s[0]), float(s[1])])
    return px, xy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--collect-cube", action="store_true",
                    help="cube.pt로 큐브 바닥앵커 자동검출하며 대화형 수집(클릭 불필요, 권장)")
    ap.add_argument("--model", default="models/cube.pt")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--points", help="대응점 json (주면 GUI 안 씀)")
    ap.add_argument("--snap", default=f"{OUT}/scan.png")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    if args.collect_cube:
        px, xy = collect_cube(args.model, args.conf)
    elif args.points:
        px, xy = [], []
        for d in json.load(open(args.points)):
            px.append(d["px"]); xy.append(d["xy"])
    else:
        frame = capture()
        if frame is None:
            print("캡처 실패"); return
        cv2.imwrite(args.snap, frame)
        print(f"스캔 이미지 저장 {args.snap}  ({frame.shape[1]}x{frame.shape[0]})")
        px, xy = gui_collect(frame)

    if len(px) < 4:
        print(f"4점 이상 필요 (현재 {len(px)})"); return
    px = np.array(px, np.float32)
    xy = np.array(xy, np.float32)
    H, _ = cv2.findHomography(px, xy)
    if H is None:
        print("✗ findHomography 실패 — 점이 일직선이거나 너무 적음. 더 퍼뜨려 재수집."); return
    proj = cv2.perspectiveTransform(px.reshape(1, -1, 2), H)[0]
    err = float(np.mean(np.linalg.norm(proj - xy, axis=1)))
    np.savez(f"{OUT}/homography.npz", H=H, snap=args.snap)
    print(f"\nH 저장 {OUT}/homography.npz   ({len(px)}점, 재투영 평균오차 {err:.2f} cm)")
    print("  →", "좋음 (≤1cm)" if err < 1.0 else "점 분포/측정 재확인 권장 (>1cm)")
    print("이제: python3 pick_run.py --yolo --dry  로 테이블좌표까지 나오는지 확인")


if __name__ == "__main__":
    main()
