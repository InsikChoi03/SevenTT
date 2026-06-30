#!/usr/bin/env python3
"""FastSAM(클래스 무관 '아무거나 분할') 로 물체 제안 — 색 임계 대신 학습된 objectness.

먼저 segment-only로 물체를 robust하게 찾는지 검증. (--siglip 주면 각 조각을 SigLIP 분류)
  python3 scripts/fastsam_detect.py data/cam_test/dc_raw.png
  python3 scripts/fastsam_detect.py data/cam_test/dc_raw.png --siglip
"""
from __future__ import annotations
import argparse
import os
import sys
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
FASTSAM_W = "models/FastSAM-s.pt"
SIGLIP_ID = "google/siglip-base-patch16-224"
SHAPES = ["cube", "octahedron", "dodecahedron", "icosahedron"]
REJECTS = ["background", "wooden floor", "a cardboard box", "an empty surface", "a wall", "a shadow"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--siglip", action="store_true", help="각 조각을 SigLIP으로 분류")
    ap.add_argument("--imgsz", type=int, default=1024)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--iou", type=float, default=0.9)
    ap.add_argument("--min-area-frac", type=float, default=0.003)
    ap.add_argument("--max-area-frac", type=float, default=0.30)
    ap.add_argument("--min-solidity", type=float, default=0.80)
    args = ap.parse_args()

    frame = cv2.imread(args.src)
    if frame is None:
        print(f"읽기 실패: {args.src}"); return 1
    H, W = frame.shape[:2]
    base = args.src.rsplit(".", 1)[0]

    import torch
    from ultralytics import FastSAM
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[init] FastSAM on {dev} (가중치 없으면 자동 다운로드)", flush=True)
    model = FastSAM(FASTSAM_W)
    r = model(frame, device=dev, retina_masks=True, imgsz=args.imgsz,
              conf=args.conf, iou=args.iou, verbose=False)[0]

    if r.masks is None:
        print("masks=0"); return 0
    masks = r.masks.data.cpu().numpy()           # [N,h,w]
    print(f"FastSAM raw masks={len(masks)}")

    # objectness 마스크 → 크기/solidity로 '물체스러운' 조각만
    cand = []
    for m in masks:
        mb = (m > 0.5).astype(np.uint8)
        if mb.shape[:2] != (H, W):
            mb = cv2.resize(mb, (W, H), interpolation=cv2.INTER_NEAREST)
        area = int(mb.sum())
        if area < args.min_area_frac * H * W or area > args.max_area_frac * H * W:
            continue
        cnts, _ = cv2.findContours(mb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        ha = cv2.contourArea(cv2.convexHull(c))
        if ha <= 0 or cv2.contourArea(c) / ha < args.min_solidity:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        cand.append((x, y, x + bw, y + bh, mb))
    print(f"filtered object-like proposals={len(cand)}")

    # SigLIP 분류기 (옵션)
    proc = mdl = None
    if args.siglip and cand:
        from PIL import Image
        from transformers import AutoProcessor, AutoModel
        proc = AutoProcessor.from_pretrained(SIGLIP_ID)
        mdl = AutoModel.from_pretrained(SIGLIP_ID).to(dev).eval()
        labels = SHAPES + REJECTS
        texts = [f"This is a photo of a {l}." if not l.startswith(("a ", "an "))
                 else f"This is a photo of {l}." for l in labels]

    vis = frame.copy()
    kept = 0
    for i, (x0, y0, x1, y1, mb) in enumerate(cand):
        if proc is None:
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
            continue
        from PIL import Image
        crop = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        inp = proc(text=texts, images=Image.fromarray(crop),
                   return_tensors="pt", padding="max_length").to(dev)
        with torch.no_grad():
            soft = torch.softmax(mdl(**inp).logits_per_image, dim=-1)[0].cpu().numpy()
        j = int(np.argmax(soft)); best = (SHAPES + REJECTS)[j]; p = float(soft[j])
        is_shape = j < len(SHAPES) and p >= 0.45
        sj = int(np.argmax(soft[:len(SHAPES)]))
        print(f"  prop{i} -> {best} {p:.2f} [{'KEEP' if is_shape else 'reject'}] (best-shape {SHAPES[sj]}={soft[sj]:.2f})")
        if is_shape:
            kept += 1
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 3)
            cv2.putText(vis, f"{best} {p:.2f}", (x0, max(14, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 0, 255), 1)
    out = base + ("_fastsam_siglip.png" if args.siglip else "_fastsam.png")
    cv2.imwrite(out, vis)
    print(f"{'kept=' + str(kept) + '/' + str(len(cand)) if args.siglip else 'proposals=' + str(len(cand))} -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
