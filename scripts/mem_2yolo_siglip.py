#!/usr/bin/env python3
"""2개 YOLO(cube.pt 본체 + wide.pt 광각) + SigLIP 동시 로드 메모리 실측.

한 프로세스 = CUDA 컨텍스트 1개 공유 (베스트케이스). 각 모델 로드/추론 후
RSS·cudaReserved·cudaAllocated·시스템가용을 찍어 증분을 본다.
⚠️ ROS 노드로 각각 띄우면 프로세스마다 CUDA 컨텍스트(~1.3GB)가 추가됨 — 그건 따로 가산.

  python3 scripts/mem_2yolo_siglip.py
"""
from __future__ import annotations
import numpy as np
import torch


def rss():
    for l in open("/proc/self/status"):
        if l.startswith("VmRSS"):
            return int(l.split()[1]) / 1024
def avail():
    for l in open("/proc/meminfo"):
        if l.startswith("MemAvailable"):
            return int(l.split()[1]) / 1024
def show(tag):
    r = torch.cuda.memory_reserved() / 1e6
    a = torch.cuda.memory_allocated() / 1e6
    print(f"{tag:36s} RSS={rss():6.0f}MB  cudaResv={r:6.0f}MB  cudaAlloc={a:6.0f}MB  sysAvail={avail():6.0f}MB",
          flush=True)


def main():
    blank = np.zeros((1232, 1640, 3), np.uint8)
    show("0 baseline")

    from ultralytics import YOLO
    show("0b ultralytics import")

    m_body = YOLO("models/cube.pt")
    m_body.predict(blank, imgsz=640, verbose=False)
    torch.cuda.synchronize(); show("1 +cube.pt(body) inferred")

    m_wide = YOLO("models/wide.pt")
    m_wide.predict(blank, imgsz=640, verbose=False)
    torch.cuda.synchronize(); show("2 +wide.pt(wide) inferred")

    from PIL import Image
    from transformers import AutoModel, AutoProcessor
    proc = AutoProcessor.from_pretrained("google/siglip-base-patch16-224")
    sg = AutoModel.from_pretrained("google/siglip-base-patch16-224").to("cuda").eval()
    crop = Image.fromarray(np.zeros((224, 224, 3), np.uint8))
    texts = [f"a photo of a {x}" for x in ["apple", "orange", "banana", "pineapple"]]
    inp = proc(text=texts, images=crop, return_tensors="pt", padding="max_length").to("cuda")
    with torch.no_grad():
        sg(**inp)
    torch.cuda.synchronize(); show("3 +SigLIP inferred (전부 로드)")

    for _ in range(8):
        m_body.predict(blank, imgsz=640, verbose=False)
        m_wide.predict(blank, imgsz=640, verbose=False)
        with torch.no_grad():
            sg(**inp)
    torch.cuda.synchronize(); show("4 combined x8 (정상상태)")

    print("\n[해석] 위는 1프로세스(CUDA 컨텍스트 1개) 기준. ROS 노드를 따로 띄우면 "
          "프로세스마다 ~1.3GB 컨텍스트가 추가된다. sysAvail 감소폭 = 실제 점유.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
