#!/usr/bin/env python3
"""SigLIP 라이브 과일 분류 검증 — 본체캠 중앙 ROI를 4종 과일로 제로샷 분류해 창에 표시.

YOLO 패치검출/월드모델은 아직 안 붙임. 순수하게 "SigLIP이 과일을 제대로 구분하나"만
눈으로 확인하는 용도. 화면 중앙 초록 박스 안에 과일사진(또는 실제 과일)을 대면
라벨별 점수 막대가 실시간 갱신된다. 이 박스 = 나중에 fruit_photo_cube 패치 크롭이 될 자리.

  python3 scripts/siglip_live.py                          # 라이브 창 (q/ESC 종료)
  python3 scripts/siglip_live.py --labels apple,orange,banana,pineapple
  python3 scripts/siglip_live.py --roi 0.45 --every 2     # ROI 크기 / N프레임마다 추론
  python3 scripts/siglip_live.py --shot /tmp/shot.png     # 헤드리스: 1프레임 분류 후 PNG 저장
  python3 scripts/siglip_live.py --raw                    # 화이트밸런스 보정 끄고 비교
  라이브 창에서 [ / ] = ROI 축소/확대, q/ESC = 종료.

본체캠 IMX219는 마젠타 캐스트가 있어 기본으로 csi_capture 고정게인 WB가 적용된다(과일색 정확도↑).
softmax(후보 4종 상대확률)로 best를 고르고, 괄호의 s=sigmoid(절대확률, "이게 진짜 그 과일인가").
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402  역할↔sensor-id·WB 중앙 설정
from csi_capture import CsiCamera  # noqa: E402  gi/appsink + 본체 WB 게인

MODEL_ID = "google/siglip-base-patch16-224"
DEFAULT_LABELS = "apple,orange,banana,pineapple"
PROMPT = "a photo of a {}"  # siglip_gate_node 와 동일 템플릿


class Siglip:
    """SigLIP 제로샷 분류기 (라벨 고정, 크롭 → sigmoid/softmax 점수)."""

    def __init__(self, labels):
        import torch
        from PIL import Image
        from transformers import AutoModel, AutoProcessor

        self.torch = torch
        self.Image = Image
        self.dev = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"[init] loading SigLIP on {self.dev} ... (첫 로드 + CUDA 컨텍스트로 수초 걸림)", flush=True)
        self.proc = AutoProcessor.from_pretrained(MODEL_ID)
        self.model = AutoModel.from_pretrained(MODEL_ID).to(self.dev).eval()
        self.texts = [PROMPT.format(l) for l in labels]
        print(f"[init] SigLIP ready. labels={labels}", flush=True)

    def classify(self, crop_bgr):
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        img = self.Image.fromarray(rgb)
        inp = self.proc(text=self.texts, images=img, return_tensors="pt",
                        padding="max_length").to(self.dev)
        with self.torch.no_grad():
            out = self.model(**inp)
        # SigLIP은 프롬프트별 독립 sigmoid가 본래 확률. softmax는 후보 4종 중 상대 비교용.
        sig = self.torch.sigmoid(out.logits_per_image)[0].detach().cpu().numpy()
        soft = self.torch.softmax(out.logits_per_image, dim=-1)[0].detach().cpu().numpy()
        return sig, soft


def center_roi(frame, frac):
    h, w = frame.shape[:2]
    s = max(16, int(min(h, w) * frac))
    cx, cy = w // 2, h // 2
    x1, y1 = cx - s // 2, cy - s // 2
    return x1, y1, x1 + s, y1 + s


def draw(frame, labels, sig, soft, roi, infer_ms):
    x1, y1, x2, y2 = roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, "PLACE FRUIT HERE", (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    if soft is not None:
        order = np.argsort(-soft)
        best = int(order[0])
        margin = float(soft[order[0]] - soft[order[1]]) if len(order) > 1 else 1.0
        px, py, bw = 12, 64, 240
        for rank, j in enumerate(order):
            col = (0, 230, 0) if j == best else (180, 180, 180)
            y = py + rank * 30
            cv2.rectangle(frame, (px, y - 17), (px + int(bw * float(soft[j])), y), col, -1)
            txt = f"{labels[j]:<10} {soft[j]:.2f} (s{sig[j]:.2f})"
            cv2.putText(frame, txt, (px + 4, y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, txt, (px + 4, y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        hud = f"best={labels[best]}  margin={margin:.2f}  {infer_ms:.0f}ms"
    else:
        hud = "warming up..."
    cv2.putText(frame, hud, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, hud, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    return frame


def open_camera(sensor_id, raw):
    try:
        # raw면 WB 끔, 아니면 미지정 → csi_capture가 sensor-id별 기본게인(본체=고정WB) 자동 적용.
        return CsiCamera(sensor_id=sensor_id, wb_gains=None) if raw else CsiCamera(sensor_id=sensor_id)
    except Exception as exc:
        print(f"[err] 카메라 sensor-id={sensor_id} 열기 실패: {exc}", file=sys.stderr)
        print("[hint] ROS camera_csi_node 가 켜져 있으면 센서를 점유함 → 끄고 재시도.", file=sys.stderr)
        print("[hint] 또는: sudo systemctl restart nvargus-daemon", file=sys.stderr)
        raise


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY,
                    help="camera_config 참조 (BODY=0 본체, WIDE=1 광각)")
    ap.add_argument("--roi", type=float, default=0.5, help="중앙 ROI 한 변 = min(h,w)*roi")
    ap.add_argument("--every", type=int, default=1, help="N 프레임마다 추론 (창은 매 프레임 갱신)")
    ap.add_argument("--show-width", type=int, default=960)
    ap.add_argument("--raw", action="store_true", help="화이트밸런스 보정 끔(비교용)")
    ap.add_argument("--shot", default="", help="헤드리스: 1프레임 분류해 PNG 저장 후 종료")
    args = ap.parse_args()

    labels = [s.strip() for s in args.labels.split(",") if s.strip()]
    if len(labels) < 2:
        print("라벨 2개 이상 필요"); return 1

    clf = Siglip(labels)
    cam = open_camera(args.sensor_id, args.raw)
    print(f"[init] camera sensor-id={args.sensor_id} streaming (wb={'OFF(raw)' if args.raw else 'ON(fixed)'})",
          flush=True)

    # ---- 헤드리스 단발 (창 없이 파이프라인 검증) ----
    if args.shot:
        frame = None
        for _ in range(15):              # 3A(노출/WB) 안정될 때까지 몇 프레임 흘림
            f = cam.read()
            if f is not None:
                frame = f
        if frame is None:
            print("[shot] 카메라 프레임 없음"); cam.release(); return 1
        roi = center_roi(frame, args.roi)
        t = time.time()
        sig, soft = clf.classify(frame[roi[1]:roi[3], roi[0]:roi[2]])
        ms = (time.time() - t) * 1e3
        order = np.argsort(-soft)
        print(f"[shot] best={labels[int(order[0])]}  ({ms:.0f}ms)")
        for j in order:
            print(f"        {labels[j]:<10} softmax={soft[j]:.3f}  sigmoid={sig[j]:.3f}")
        draw(frame, labels, sig, soft, roi, ms)
        cv2.imwrite(args.shot, frame)
        print(f"[shot] -> {args.shot}")
        cam.release()
        return 0

    # ---- 라이브 창 ----
    win = "siglip-fruit (q/ESC quit, [ ] resize ROI)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    sig = soft = None
    ms = 0.0
    i = 0
    roi_frac = args.roi
    try:
        while True:
            frame = cam.read()
            if frame is None:
                print("[warn] frame timeout"); continue
            roi = center_roi(frame, roi_frac)
            if i % max(1, args.every) == 0:
                t = time.time()
                sig, soft = clf.classify(frame[roi[1]:roi[3], roi[0]:roi[2]])
                ms = (time.time() - t) * 1e3
            i += 1
            draw(frame, labels, sig, soft, roi, ms)
            if args.show_width and frame.shape[1] > args.show_width:
                s = args.show_width / frame.shape[1]
                frame = cv2.resize(frame, (args.show_width, int(frame.shape[0] * s)))
            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("["):
                roi_frac = max(0.1, roi_frac - 0.05)
            if k == ord("]"):
                roi_frac = min(0.95, roi_frac + 0.05)
    finally:
        cam.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
