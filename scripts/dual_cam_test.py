#!/usr/bin/env python3
"""두 CSI 캠(광각0+본체1)을 한 프로세스에서 동시에 열고 FastSAM 공유로 돌려
실제 메모리/FPS 측정. 마지막에 SigLIP까지 올려 전체 스택 메모리 확인.
"""
from __future__ import annotations
import sys, os, time
import cv2, numpy as np, torch
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def rss():
    for l in open("/proc/self/status"):
        if l.startswith("VmRSS"): return int(l.split()[1]) / 1024
def avail():
    for l in open("/proc/meminfo"):
        if l.startswith("MemAvailable"): return int(l.split()[1]) / 1024
def show(tag):
    print(f"{tag:32s} RSS={rss():6.0f}MB cudaResv={torch.cuda.memory_reserved()/1e6:5.0f}MB sysAvail={avail():5.0f}MB", flush=True)


def fastsam_count(r, H, W):
    if r.masks is None: return 0
    n = 0
    for m in r.masks.data.cpu().numpy():
        mb = (m > 0.5)
        if mb.sum() > 0.004 * H * W: n += 1
    return n


def main():
    show("0 start")
    from csi_capture import CsiCamera
    cam0 = CsiCamera(0, wb_gains=None)      # 광각
    cam1 = CsiCamera(1)                      # 본체
    for _ in range(8):
        cam0.read(1.0); cam1.read(1.0)
    show("1 두 캠 동시 오픈+스트리밍")

    from ultralytics import FastSAM
    fs = FastSAM("models/FastSAM-s.pt")
    IMG = 768
    # warmup
    f0, f1 = cam0.read(1.0), cam1.read(1.0)
    if f0 is not None: fs(f0, device="cuda", imgsz=IMG, conf=0.4, iou=0.9, retina_masks=True, verbose=False)
    if f1 is not None: fs(f1, device="cuda", imgsz=IMG, conf=0.4, iou=0.9, retina_masks=True, verbose=False)
    torch.cuda.synchronize()
    show("2 +FastSAM(공유) 추론후")

    # 두 캠 각각 FastSAM 돌리는 한 사이클을 N번
    N = 12; t0 = time.perf_counter(); c0 = c1 = 0
    for _ in range(N):
        f0 = cam0.read(1.0); f1 = cam1.read(1.0)
        if f0 is not None:
            r0 = fs(f0, device="cuda", imgsz=IMG, conf=0.4, iou=0.9, retina_masks=True, verbose=False)[0]
            c0 = fastsam_count(r0, *f0.shape[:2])
        if f1 is not None:
            r1 = fs(f1, device="cuda", imgsz=IMG, conf=0.4, iou=0.9, retina_masks=True, verbose=False)[0]
            c1 = fastsam_count(r1, *f1.shape[:2])
    torch.cuda.synchronize()
    dt = time.perf_counter() - t0
    print(f"\n[FastSAM 두 캠] {N}사이클 {dt:.1f}s → 사이클당 {dt/N*1000:.0f}ms "
          f"= {N/dt:.1f} 사이클/s (=각 캠 {N/dt:.1f} FPS), 광각객체={c0} 본체객체={c1}")
    show("3 두캠+FastSAM 루프후")

    # SigLIP까지 올려 전체 스택 메모리
    from transformers import AutoProcessor, AutoModel
    from PIL import Image
    proc = AutoProcessor.from_pretrained("google/siglip-base-patch16-224")
    sg = AutoModel.from_pretrained("google/siglip-base-patch16-224").to("cuda").eval()
    texts = [f"a {l}" for l in ["cube", "octahedron", "dodecahedron", "icosahedron", "background"]]
    crop = [Image.fromarray(np.zeros((224, 224, 3), np.uint8)) for _ in range(4)]
    inp = proc(text=texts, images=crop, return_tensors="pt", padding="max_length").to("cuda")
    with torch.no_grad(): sg(**inp)
    torch.cuda.synchronize()
    show("4 +SigLIP까지 (전체스택)")

    cam0.release(); cam1.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
