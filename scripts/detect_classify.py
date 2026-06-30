#!/usr/bin/env python3
"""통합 탐지+분류: 느슨한 region proposal(고전CV) → SigLIP 제로샷 분류 + background reject.

고전CV는 정밀할 필요 없이 후보만 과검출 → SigLIP이 각 크롭을 도형/과일 vs background로 판정.
'background'류로 분류되면 버림 → 고전CV 오검출을 SigLIP이 걸러내는 구조.

  python3 scripts/detect_classify.py                       # 카메라 한 장
  python3 scripts/detect_classify.py --image data/cam_test/now.png
"""
from __future__ import annotations
import argparse
import os
import sys
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

MODEL_ID = "google/siglip-base-patch16-224"
SHAPES = ["cube", "octahedron", "dodecahedron", "icosahedron"]
REJECTS = ["background", "wooden floor", "a cardboard box", "an empty surface",
           "a wall", "a hand", "a cable", "a shadow"]


def propose(frame, s_max, v_min, min_area, max_area_frac, min_solidity, pad):
    """느슨한 흰물체 후보 박스 (과검출 허용)."""
    h, w = frame.shape[:2]
    max_area = max_area_frac * h * w
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s, v = hsv[..., 1], hsv[..., 2]
    mask = (((s < s_max) & (v > v_min)).astype(np.uint8)) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:15]:
        a = cv2.contourArea(c)
        if a < min_area or a > max_area:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        ha = cv2.contourArea(cv2.convexHull(c))
        if ha <= 0 or a / ha < min_solidity:           # 느슨(0.6)
            continue
        x0 = max(0, x - pad); y0 = max(0, y - pad)
        x1 = min(w, x + bw + pad); y1 = min(h, y + bh + pad)
        boxes.append((x0, y0, x1, y1))
    return boxes, mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image")
    ap.add_argument("--sensor-id", type=int, default=1)
    ap.add_argument("--s-max", type=int, default=80)
    ap.add_argument("--v-min", type=int, default=150)
    ap.add_argument("--min-area", type=int, default=2000)
    ap.add_argument("--max-area-frac", type=float, default=0.35)
    ap.add_argument("--min-solidity", type=float, default=0.60)   # 느슨: 과검출
    ap.add_argument("--pad", type=int, default=15)
    ap.add_argument("--keep-thresh", type=float, default=0.45, help="도형 상대확률 최소")
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"읽기 실패: {args.image}"); return 1
        base = args.image.rsplit(".", 1)[0]
    else:
        from csi_capture import CsiCamera
        cam = CsiCamera(args.sensor_id)
        frame = None
        for _ in range(15):
            f = cam.read(1.0)
            if f is not None:
                frame = f
        cam.release()
        if frame is None:
            print("캡처 실패"); return 1
        base = "data/cam_test/dc"
        cv2.imwrite(base + "_raw.png", frame)

    boxes, _ = propose(frame, args.s_max, args.v_min, args.min_area,
                       args.max_area_frac, args.min_solidity, args.pad)
    print(f"proposals={len(boxes)}")
    if not boxes:
        print("후보 없음"); return 0

    import torch
    from PIL import Image
    from transformers import AutoProcessor, AutoModel
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[init] SigLIP on {dev}", flush=True)
    proc = AutoProcessor.from_pretrained(MODEL_ID)
    model = AutoModel.from_pretrained(MODEL_ID).to(dev).eval()

    labels = SHAPES + REJECTS
    texts = [f"This is a photo of a {l}." if not l.startswith(("a ", "an "))
             else f"This is a photo of {l}." for l in labels]

    vis = frame.copy()
    kept = 0
    for i, (x0, y0, x1, y1) in enumerate(boxes):
        crop = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        inp = proc(text=texts, images=Image.fromarray(crop),
                   return_tensors="pt", padding="max_length").to(dev)
        with torch.no_grad():
            soft = torch.softmax(model(**inp).logits_per_image, dim=-1)[0].cpu().numpy()
        j = int(np.argmax(soft))
        best, p = labels[j], float(soft[j])
        is_shape = j < len(SHAPES) and p >= args.keep_thresh
        # 도형 후보 중 최고도 같이 표기
        sj = int(np.argmax(soft[:len(SHAPES)]))
        shape_str = f"{SHAPES[sj]}={soft[sj]:.2f}"
        print(f"  box{i} -> {best} {p:.2f}  [{'KEEP' if is_shape else 'reject'}]  (best-shape {shape_str})")
        if is_shape:
            kept += 1
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 3)
            cv2.putText(vis, f"{best} {p:.2f}", (x0, max(14, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 0, 255), 1)  # 거부=얇은 빨강
    out = base + "_dc.png"
    cv2.imwrite(out, vis)
    print(f"kept(shape)={kept}/{len(boxes)} -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
