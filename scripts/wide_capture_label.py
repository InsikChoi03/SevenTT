#!/usr/bin/env python3
"""광각(top, sensor-id 1) 촬영 / 일괄 pre-label 도구.

기본(촬영 모드): 카메라 live 프리뷰 GUI 창. 사진만 찍어 모음(라벨링 안 함).
  SPACE = 촬영(저장)   q/ESC = 종료
  → data/wide_dataset/images/ 에 wide_<ts>_<n>.jpg 저장 (flip=2, 180°).
  카메라는 1회만 열고 한 세션 재사용(매샷 재오픈 안 함 → CaptureSession 에러 방지).

--prelabel : (나중에 한 번에) 모은 사진 전체를 cube_v3로 일괄 pre-label.
  data/wide_dataset/images/*.jpg → labels/*.txt + review/*.png (마스크·크기필터 적용).
  이후 노트북 labelImg로 교정(놓침 추가 + 클래스 수정) → Colab로 깡통 yolov8n 학습.

  python3 scripts/wide_capture_label.py            # 촬영
  python3 scripts/wide_capture_label.py --prelabel # 모은 사진 일괄 라벨
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")
os.environ.setdefault("DISPLAY", ":1")   # GUI 창은 젯슨 화면(:1)에

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402

ROOT = "data/wide_dataset"
IMG_DIR, LBL_DIR, REV_DIR = f"{ROOT}/images", f"{ROOT}/labels", f"{ROOT}/review"
CLASSES = ["cube", "octahedron", "dodecahedron", "icosahedron"]
_COL = [(0, 255, 0), (0, 200, 255), (255, 150, 0), (255, 0, 200)]
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def ensure_dirs():
    for d in (IMG_DIR, LBL_DIR, REV_DIR):
        os.makedirs(d, exist_ok=True)
    open(f"{ROOT}/classes.txt", "w").write("\n".join(CLASSES) + "\n")
    open(f"{LBL_DIR}/classes.txt", "w").write("\n".join(CLASSES) + "\n")
    open(f"{ROOT}/data.yaml", "w").write(
        f"path: {os.path.abspath(ROOT)}\ntrain: images\nval: images\nnc: {len(CLASSES)}\nnames: {CLASSES}\n")


def load_mask():
    p = f"{ROOT}/arena_mask.json"
    if not os.path.exists(p):
        return None
    cfg = json.load(open(p))
    keep = np.array(cfg["keep_polygon"], np.int32) if cfg.get("keep_polygon") else None
    return {"keep": keep, "exclude": [tuple(b) for b in cfg.get("exclude_boxes", [])]}


# ---------------------------------------------------------------- 촬영 모드
def capture_mode():
    from csi_capture import CsiCamera
    cam = None
    for _ in range(3):
        c = None
        try:
            c = CsiCamera(sensor_id=camera_config.WIDE, wb_gains=None, flip=2)
            for _ in range(8):
                c.read(1.0)
            cam = c
            break
        except Exception as e:
            if c is not None:
                try:
                    c.release()
                except Exception:
                    pass
            print(f"  카메라 열기 실패: {e} — 재시도 (안되면 'sudo systemctl restart nvargus-daemon')")
            time.sleep(1.5)
    if cam is None:
        print("카메라 못 엶. nvargus 재시작 후 다시."); return 1

    win = "wide capture (SPACE=촬영, q=종료)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    n = len(glob.glob(f"{IMG_DIR}/wide_*.jpg"))
    print(f"[촬영] 기존 {n}장. 창에서 SPACE=촬영, q/ESC=종료. (라벨링은 나중에 --prelabel)", flush=True)
    flash = 0
    try:
        while True:
            frame = cam.read(0.5)
            if frame is None:
                if cv2.waitKey(10) & 0xFF in (ord("q"), 27):
                    break
                continue
            disp = frame.copy()
            cv2.putText(disp, f"SPACE=capture  q=quit   saved:{n}", (12, 30),
                        _FONT, 0.8, (0, 0, 0), 4)
            cv2.putText(disp, f"SPACE=capture  q=quit   saved:{n}", (12, 30),
                        _FONT, 0.8, (0, 255, 0), 2)
            if flash > 0:
                cv2.rectangle(disp, (0, 0), (disp.shape[1] - 1, disp.shape[0] - 1), (0, 255, 255), 8)
                flash -= 1
            cv2.imshow(win, disp)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == 32:   # SPACE
                base = f"wide_{int(time.time())}_{n:03d}"
                cv2.imwrite(f"{IMG_DIR}/{base}.jpg", frame)
                print(f"  저장 {base}.jpg  (총 {n+1}장)", flush=True)
                n += 1
                flash = 4
    finally:
        cam.release()
        cv2.destroyAllWindows()
    print(f"\n총 {n}장. 다음: python3 scripts/wide_capture_label.py --prelabel")
    return 0


# ---------------------------------------------------------------- 일괄 라벨 모드
def prelabel_mode(args):
    mask = load_mask()
    import torch
    from ultralytics import YOLO
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[prelabel] 모델 로딩 ({dev}): {args.model}", flush=True)
    model = YOLO(args.model)

    def in_arena(d):
        if mask is None:
            return True
        cx, cy = (d[0] + d[2]) / 2.0, (d[1] + d[3]) / 2.0
        if mask["keep"] is not None and cv2.pointPolygonTest(mask["keep"], (cx, cy), False) < 0:
            return False
        return not any(x1 <= cx <= x2 and y1 <= cy <= y2 for (x1, y1, x2, y2) in mask["exclude"])

    imgs = sorted(glob.glob(f"{IMG_DIR}/*.jpg") + glob.glob(f"{IMG_DIR}/*.png"))
    print(f"[prelabel] 대상 {len(imgs)}장, conf={args.conf}, imgsz={args.imgsz}", flush=True)
    done = 0
    for p in imgs:
        name = os.path.splitext(os.path.basename(p))[0]
        lbl = f"{LBL_DIR}/{name}.txt"
        if os.path.exists(lbl) and not args.relabel:
            continue
        frame = cv2.imread(p)
        if frame is None:
            continue
        H, W = frame.shape[:2]
        r = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device=dev, verbose=False)[0]
        dets, vis = [], frame.copy()
        for b in (r.boxes or []):
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            d = (x1, y1, x2, y2, int(b.cls[0]), float(b.conf[0]))
            if args.min_dim <= max(x2 - x1, y2 - y1) <= args.max_dim and in_arena(d):
                dets.append(d)
        with open(lbl, "w") as f:
            f.write("\n".join(
                f"{c} {(x1+x2)/2/W:.6f} {(y1+y2)/2/H:.6f} {(x2-x1)/W:.6f} {(y2-y1)/H:.6f}"
                for (x1, y1, x2, y2, c, _) in dets) + ("\n" if dets else ""))
        for (x1, y1, x2, y2, c, cf) in dets:
            cv2.rectangle(vis, (x1, y1), (x2, y2), _COL[c % 4], 3)
            cv2.putText(vis, f"{CLASSES[c]} {cf:.2f}", (x1, max(16, y1 - 6)), _FONT, 0.6, (0, 0, 0), 4)
            cv2.putText(vis, f"{CLASSES[c]} {cf:.2f}", (x1, max(16, y1 - 6)), _FONT, 0.6, _COL[c % 4], 1)
        cv2.imwrite(f"{REV_DIR}/{name}.png", vis)
        done += 1
        if done % 20 == 0:
            print(f"  ...{done}장", flush=True)
    print(f"[prelabel] 완료: {done}장 라벨. 다음: 노트북 labelImg로 교정(놓침 추가+클래스 수정) -> Colab 학습.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prelabel", action="store_true", help="모은 사진 전체를 cube_v3로 일괄 라벨")
    ap.add_argument("--model", default="models/cube.pt", help="prelabel 모델 (현 cube_v3)")
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--min-dim", type=int, default=30)
    ap.add_argument("--max-dim", type=int, default=220)
    ap.add_argument("--relabel", action="store_true", help="이미 라벨된 것도 다시")
    args = ap.parse_args()
    ensure_dirs()
    return prelabel_mode(args) if args.prelabel else capture_mode()


if __name__ == "__main__":
    raise SystemExit(main())
