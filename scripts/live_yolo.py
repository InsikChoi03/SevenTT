#!/usr/bin/env python3
"""학습한 YOLOv8 모델로 본체캠 실시간 검출 (모니터 창).

  DISPLAY=:1 python3 scripts/live_yolo.py                       # 최신 best.pt 자동
  DISPLAY=:1 python3 scripts/live_yolo.py --model runs/.../best.pt --conf 0.3
  python3 scripts/live_yolo.py --shot /tmp/y.jpg                # 헤드리스 한 장
키: q/ESC 종료.  ⚠️ 학습 끝난 뒤 실행 (학습 중엔 GPU/메모리 충돌).
"""
from __future__ import annotations
import argparse, glob, os, sys, time
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402
import box_utils  # noqa: E402

_COL = [(0, 255, 0), (0, 200, 255), (255, 150, 0), (255, 0, 200),
        (255, 255, 0), (0, 0, 255), (180, 120, 255), (0, 255, 128)]


def find_latest_best():
    cands = glob.glob("runs/**/weights/best.pt", recursive=True)
    if not cands:
        return None
    return max(cands, key=lambda p: os.path.getmtime(p))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None, help="best.pt 경로 (기본: runs 아래 최신)")
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--show-width", type=int, default=1000)
    ap.add_argument("--shot", default="", help="헤드리스: 한 장 검출 저장 후 종료")
    args = ap.parse_args()

    model_path = args.model or find_latest_best()
    if not model_path or not os.path.exists(model_path):
        print(f"모델 못 찾음: {model_path} (학습 완료 후 best.pt 경로 지정)"); return 1
    print(f"[init] model={model_path}", flush=True)

    import torch
    from ultralytics import YOLO
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model = YOLO(model_path)
    names = model.names

    def infer_draw(frame):
        r = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device=dev,
                          verbose=False, agnostic_nms=True, iou=0.6)[0]
        dets = []
        if r.boxes is not None:
            for b in r.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                dets.append((x1, y1, x2, y2, int(b.cls[0]), float(b.conf[0])))
        dets = box_utils.dedupe(dets)           # 같은 물체 중복/박스속박스 제거 (부분가림은 유지)
        for (x1, y1, x2, y2, cls, cf) in dets:
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            col = _COL[cls % len(_COL)]
            lab = f"{names[cls]} {cf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3)
            cv2.putText(frame, lab, (x1, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(frame, lab, (x1, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 1)
        return len(dets)

    from csi_capture import CsiCamera
    cam = CsiCamera(args.sensor_id)
    for _ in range(8):
        cam.read(1.0)
    print(f"[init] body cam(id={args.sensor_id}) streaming", flush=True)

    # 헤드리스 한 장
    if args.shot:
        frame = None
        for _ in range(6):
            g = cam.read(1.0)
            if g is not None: frame = g
        cam.release()
        if frame is None: print("캡처 실패"); return 1
        n = infer_draw(frame); cv2.imwrite(args.shot, frame)
        print(f"[shot] {n} det -> {args.shot}"); return 0

    win = "live YOLO (q/ESC)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    t_prev = time.time(); fps = 0.0
    try:
        while True:
            frame = cam.read(1.0)
            if frame is None:
                continue
            n = infer_draw(frame)
            now = time.time(); fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3)); t_prev = now
            hud = f"{fps:4.1f} FPS | {n} det"
            cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            disp = frame
            if args.show_width and frame.shape[1] > args.show_width:
                s = args.show_width / frame.shape[1]
                disp = cv2.resize(frame, (args.show_width, int(frame.shape[0] * s)))
            cv2.imshow(win, disp)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
    finally:
        cam.release(); cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
