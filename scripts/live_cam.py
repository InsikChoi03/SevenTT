#!/usr/bin/env python3
"""CSI 카메라 라이브 뷰 + YOLO 검출 오버레이.

  python3 scripts/live_cam.py --cam wide            # 광각(sensor1) + models/wide.pt, imgsz1280
  python3 scripts/live_cam.py --cam body            # 본체(sensor0) + models/cube.pt, imgsz640
  python3 scripts/live_cam.py --cam wide --no-detect # 검출 없이 raw만(부드러움)
키: q/ESC 종료, d=검출 on/off 토글, s=스냅샷(/tmp/livecam_snap.png).
광각(sensor1)은 marginal — 열기 PAUSE 실패 시 `sudo systemctl restart nvargus-daemon` 후 재실행.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402
from csi_capture import CsiCamera  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", default="wide", choices=["wide", "body"])
    ap.add_argument("--model", default=None, help="기본: wide→models/wide.pt, body→models/cube.pt")
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--imgsz", type=int, default=None, help="기본: wide 1280, body 640")
    ap.add_argument("--show-width", type=int, default=1100)
    ap.add_argument("--no-detect", action="store_true")
    args = ap.parse_args()

    is_wide = args.cam == "wide"
    model_path = args.model or ("models/wide.pt" if is_wide else "models/cube.pt")
    imgsz = args.imgsz or (1280 if is_wide else 640)

    from ultralytics import YOLO
    print(f"[init] 모델 로드 {model_path} (imgsz={imgsz}) ...", flush=True)
    model = YOLO(model_path)

    sid = camera_config.WIDE if is_wide else camera_config.BODY
    kw = dict(flip=2, wb_gains=None) if is_wide else {}
    try:
        cam = CsiCamera(sid, **kw)
    except Exception as exc:
        print(f"[err] {args.cam}캠(sensor {sid}) 열기 실패: {exc}", file=sys.stderr)
        print("[hint] 광각 marginal — sudo systemctl restart nvargus-daemon 후 재시도", file=sys.stderr)
        return 1
    for _ in range(20):
        cam.read(1.0)                     # 자동노출(3A) 수렴

    detect = not args.no_detect
    win = f"live {args.cam} + {os.path.basename(model_path)}  (q/ESC quit, d=detect, s=snap)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print(f"[init] {args.cam} 스트리밍. 창에서 q/ESC=종료, d=검출토글, s=스냅샷", flush=True)
    try:
        while True:
            f = None
            for _ in range(2):
                g = cam.read(1.0)
                if g is not None:
                    f = g
            if f is None:
                print("[warn] frame timeout"); continue
            if detect:
                r = model.predict(f, imgsz=imgsz, conf=args.conf, verbose=False)[0]
                disp = r.plot()
            else:
                disp = f
            if args.show_width and disp.shape[1] > args.show_width:
                s = args.show_width / disp.shape[1]
                disp = cv2.resize(disp, (args.show_width, int(disp.shape[0] * s)))
            cv2.imshow(win, disp)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("d"):
                detect = not detect
            if k == ord("s"):
                cv2.imwrite("/tmp/livecam_snap.png", f); print("snapshot -> /tmp/livecam_snap.png", flush=True)
    finally:
        cam.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
