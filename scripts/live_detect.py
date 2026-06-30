#!/usr/bin/env python3
"""실시간 라이브: FastSAM(검출) + SigLIP(분류, top-N 배치) — 본체캠, 모니터 창.

  DISPLAY=:1 python3 scripts/live_detect.py
  키: q/ESC 종료, 클래스/임계는 인자로.
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
FASTSAM_W = "models/FastSAM-s.pt"
SIGLIP_ID = "google/siglip-base-patch16-224"
SHAPES = ["cube", "octahedron", "dodecahedron", "icosahedron"]
REJECTS = ["background", "wooden floor", "a cardboard box", "an empty surface", "a wall", "a shadow"]
_COLORS = [(0, 255, 0), (0, 200, 255), (255, 150, 0), (255, 0, 200)]


def iou(a, b):
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix1 - ix0), max(0, iy1 - iy0)
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def nms(boxes, scores, thr=0.5):
    order = sorted(range(len(boxes)), key=lambda i: scores[i], reverse=True)
    keep = []
    while order:
        i = order.pop(0); keep.append(i)
        order = [j for j in order if iou(boxes[i], boxes[j]) < thr]
    return keep


def fastsam_boxes(r, H, W, min_af, max_af, min_sol):
    if r.masks is None:
        return []
    out = []
    for m in r.masks.data.cpu().numpy():
        mb = (m > 0.5).astype(np.uint8)
        if mb.shape[:2] != (H, W):
            mb = cv2.resize(mb, (W, H), interpolation=cv2.INTER_NEAREST)
        area = int(mb.sum())
        if area < min_af*H*W or area > max_af*H*W:
            continue
        cs, _ = cv2.findContours(mb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            continue
        c = max(cs, key=cv2.contourArea)
        ha = cv2.contourArea(cv2.convexHull(c))
        if ha <= 0 or cv2.contourArea(c)/ha < min_sol:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        out.append(((x, y, x+bw, y+bh), area))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY)
    ap.add_argument("--imgsz", type=int, default=768)
    ap.add_argument("--topn", type=int, default=6, help="분류할 최대 후보 수")
    ap.add_argument("--keep-thresh", type=float, default=0.45)
    ap.add_argument("--min-af", type=float, default=0.004)
    ap.add_argument("--max-af", type=float, default=0.30)
    ap.add_argument("--min-sol", type=float, default=0.78)
    ap.add_argument("--show-width", type=int, default=1000)
    args = ap.parse_args()

    import torch
    from PIL import Image
    from ultralytics import FastSAM
    from transformers import AutoProcessor, AutoModel
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[init] loading FastSAM + SigLIP on {dev} ...", flush=True)
    fs = FastSAM(FASTSAM_W)
    proc = AutoProcessor.from_pretrained(SIGLIP_ID)
    sg = AutoModel.from_pretrained(SIGLIP_ID).to(dev).eval()
    labels = SHAPES + REJECTS
    texts = [f"This is a photo of a {l}." if not l.startswith(("a ", "an "))
             else f"This is a photo of {l}." for l in labels]

    from csi_capture import CsiCamera
    cam = CsiCamera(args.sensor_id)
    for _ in range(8):
        cam.read(1.0)
    print("[init] camera streaming", flush=True)

    win = "live FastSAM+SigLIP (q/ESC)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    t_prev = time.time(); fps = 0.0
    try:
        while True:
            frame = cam.read(1.0)
            if frame is None:
                continue
            H, W = frame.shape[:2]
            r = fs(frame, device=dev, imgsz=args.imgsz, conf=0.4, iou=0.9,
                   retina_masks=True, verbose=False)[0]
            cand = fastsam_boxes(r, H, W, args.min_af, args.max_af, args.min_sol)
            boxes = [b for b, _ in cand]; scores = [s for _, s in cand]
            keepi = nms(boxes, scores, 0.5)
            keepi = sorted(keepi, key=lambda i: scores[i], reverse=True)[:args.topn]
            sel = [boxes[i] for i in keepi]

            results = []  # (box, label, p, is_shape)
            if sel:
                crops = []
                for (x0, y0, x1, y1) in sel:
                    crops.append(Image.fromarray(cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)))
                inp = proc(text=texts, images=crops, return_tensors="pt", padding="max_length").to(dev)
                with torch.no_grad():
                    soft = torch.softmax(sg(**inp).logits_per_image, dim=-1).cpu().numpy()
                for bi, box in enumerate(sel):
                    j = int(np.argmax(soft[bi])); p = float(soft[bi][j])
                    is_shape = j < len(SHAPES) and p >= args.keep_thresh
                    results.append((box, labels[j], p, is_shape))

            for (x0, y0, x1, y1), lab, p, is_shape in results:
                if is_shape:
                    col = _COLORS[SHAPES.index(lab) % len(_COLORS)]
                    cv2.rectangle(frame, (x0, y0), (x1, y1), col, 3)
                    t = f"{lab} {p:.2f}"
                    cv2.putText(frame, t, (x0, max(14, y0-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 4)
                    cv2.putText(frame, t, (x0, max(14, y0-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 1)
                else:
                    cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 0, 255), 1)

            now = time.time(); fps = 0.9*fps + 0.1*(1.0/max(now-t_prev, 1e-3)); t_prev = now
            n_shape = sum(1 for _,_,_,k in results if k)
            hud = f"{fps:4.1f} FPS | {n_shape} shapes / {len(sel)} cand"
            cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 4)
            cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1)

            disp = frame
            if args.show_width and W > args.show_width:
                sc = args.show_width / W
                disp = cv2.resize(frame, (args.show_width, int(H*sc)))
            cv2.imshow(win, disp)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
