#!/usr/bin/env python3
"""본체 카메라(sensor-id=1) 캡처 — 스냅샷 저장 / 라이브 미리보기.

  python3 cam_capture.py --snap data/pick/scan.png    # 한 장 저장
  python3 cam_capture.py --preview                     # 라이브 (GUI 필요, q 종료)
  python3 cam_capture.py                               # 프레임 크기만 출력(동작 확인)
"""
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csi_capture import CsiCamera  # noqa: E402
import camera_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--snap", help="한 장 저장할 경로")
    ap.add_argument("--preview", action="store_true", help="라이브 미리보기 (GUI)")
    ap.add_argument("--warmup", type=int, default=5, help="버릴 초기 프레임 수(AE 안정)")
    args = ap.parse_args()

    cam = CsiCamera(args.sensor_id)
    try:
        for _ in range(args.warmup):
            cam.read(1.0)
        if args.snap:
            f = cam.read(1.0)
            if f is None:
                print("캡처 실패"); return
            os.makedirs(os.path.dirname(args.snap) or ".", exist_ok=True)
            cv2.imwrite(args.snap, f)
            print(f"저장 {args.snap}  ({f.shape[1]}x{f.shape[0]})")
        elif args.preview:
            win = "cam (sensor-id=%d) - q 종료" % args.sensor_id
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win, 960, 720)
            print("미리보기 — q 종료", flush=True)
            while True:
                f = cam.read(1.0)
                if f is None:
                    continue
                cv2.imshow(win, f)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            cv2.destroyAllWindows()
        else:
            f = cam.read(1.0)
            print("프레임:", None if f is None else f.shape)
    finally:
        cam.release()


if __name__ == "__main__":
    main()
