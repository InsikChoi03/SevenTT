#!/usr/bin/env python3
"""SigLIP 제로샷으로 '도형 크롭' 분류 테스트.

고전CV(Method A: 채도+명도)로 물체 박스를 잡고, 각 크롭을 SigLIP이 후보 라벨 중
무엇으로 보는지 점수화. shape_heuristic 대체 가능성 확인용.

  python3 scripts/siglip_test.py data/cam_test/now.png
  python3 scripts/siglip_test.py data/cam_test/now.png --labels "cube,octahedron,dodecahedron,icosahedron"
"""
from __future__ import annotations
import argparse
import os
import cv2
import numpy as np

MODEL_ID = "google/siglip-base-patch16-224"
DEFAULT_LABELS = "cube,octahedron,dodecahedron,icosahedron"


def segment_boxes(frame, s_max, v_min, min_area, max_area_frac, min_solidity, pad):
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
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        a = cv2.contourArea(c)
        if a < min_area or a > max_area:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue
        ha = cv2.contourArea(cv2.convexHull(c))
        if ha <= 0 or a / ha < min_solidity:
            continue
        x0 = max(0, x - pad); y0 = max(0, y - pad)
        x1 = min(w, x + bw + pad); y1 = min(h, y + bh + pad)
        boxes.append((x0, y0, x1, y1))
    return boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    ap.add_argument("--s-max", type=int, default=70)
    ap.add_argument("--v-min", type=int, default=160)
    ap.add_argument("--min-area", type=int, default=2500)
    ap.add_argument("--max-area-frac", type=float, default=0.30)
    ap.add_argument("--min-solidity", type=float, default=0.85)
    ap.add_argument("--pad", type=int, default=15)
    args = ap.parse_args()

    frame = cv2.imread(args.src)
    if frame is None:
        print(f"읽기 실패: {args.src}"); return 1
    labels = [s.strip() for s in args.labels.split(",") if s.strip()]
    boxes = segment_boxes(frame, args.s_max, args.v_min, args.min_area,
                          args.max_area_frac, args.min_solidity, args.pad)
    print(f"boxes={len(boxes)}  labels={labels}")
    if not boxes:
        print("박스 없음"); return 0

    import torch
    from PIL import Image
    from transformers import AutoProcessor, AutoModel
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[init] loading SigLIP on {dev} ...", flush=True)
    proc = AutoProcessor.from_pretrained(MODEL_ID)
    model = AutoModel.from_pretrained(MODEL_ID).to(dev).eval()
    texts = [f"This is a photo of a {l}." for l in labels]

    vis = frame.copy()
    for i, (x0, y0, x1, y1) in enumerate(boxes):
        crop = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        img = Image.fromarray(crop)
        inp = proc(text=texts, images=img, return_tensors="pt", padding="max_length").to(dev)
        with torch.no_grad():
            out = model(**inp)
        sig = torch.sigmoid(out.logits_per_image)[0].cpu().numpy()      # SigLIP 절대확률
        soft = torch.softmax(out.logits_per_image, dim=-1)[0].cpu().numpy()  # 후보간 상대
        order = np.argsort(-soft)
        best = labels[order[0]]
        rank = "  ".join(f"{labels[j]}={soft[j]:.2f}(s{sig[j]:.2f})" for j in order)
        print(f"  box{i} ({x0},{y0},{x1},{y1}) -> {best}\n        {rank}")
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 3)
        cv2.putText(vis, f"{best} {soft[order[0]]:.2f}", (x0, max(14, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
    out_path = args.src.rsplit(".", 1)[0] + "_siglip.png"
    cv2.imwrite(out_path, vis)
    print(f"-> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
