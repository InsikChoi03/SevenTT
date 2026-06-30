#!/usr/bin/env python3
"""경량 YOLOv8n 풋프린트 측정 — FastSAM+SigLIP 대비 메모리/FPS.
검출+분류를 한 모델이 하는 미래 구조의 컴퓨트 프록시(stock yolov8n로 비용만 측정).
2 추론/사이클 = 두 캠 동시 추정.
"""
from __future__ import annotations
import sys, os, time
import cv2, numpy as np, torch
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def rss():
    for l in open("/proc/self/status"):
        if l.startswith("VmRSS"): return int(l.split()[1])/1024
def avail():
    for l in open("/proc/meminfo"):
        if l.startswith("MemAvailable"): return int(l.split()[1])/1024
def show(t):
    print(f"{t:30s} RSS={rss():6.0f}MB cudaResv={torch.cuda.memory_reserved()/1e6:5.0f}MB sysAvail={avail():5.0f}MB", flush=True)

def main():
    show("0 start")
    frame = cv2.imread("data/cam_test/dc_raw.png")
    from ultralytics import YOLO
    m = YOLO("yolov8n.pt")
    show("1 +YOLOv8n load")
    for _ in range(3):
        m.predict(frame, device="cuda", imgsz=640, conf=0.3, verbose=False)
    torch.cuda.synchronize()
    show("2 +infer(warmup)")

    # 단일 추론 latency
    N=20; t=time.perf_counter()
    for _ in range(N):
        m.predict(frame, device="cuda", imgsz=640, conf=0.3, verbose=False)
    torch.cuda.synchronize()
    one=(time.perf_counter()-t)/N*1000
    print(f"\nYOLOv8n imgsz640 단일: {one:.0f} ms/frame = {1000/one:.0f} FPS")

    # 2 추론/사이클 = 두 캠 동시 추정
    t=time.perf_counter()
    for _ in range(N):
        m.predict(frame, device="cuda", imgsz=640, conf=0.3, verbose=False)
        m.predict(frame, device="cuda", imgsz=640, conf=0.3, verbose=False)
    torch.cuda.synchronize()
    cyc=(time.perf_counter()-t)/N*1000
    print(f"YOLOv8n 두 캠(2추론/사이클): {cyc:.0f} ms/사이클 = 각 캠 {1000/cyc:.0f} FPS")
    show("3 루프후")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
