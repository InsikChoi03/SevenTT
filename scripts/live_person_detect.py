#!/usr/bin/env python3
"""Live open-vocabulary detection from a CSI IMX219 camera, drawn on screen.

Why this exists / why it does NOT use cv2.VideoCapture:
  The cv2 build on this Jetson (4.11.0, /usr/local/.../cv2) was compiled with
  GStreamer=NO, so cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER) can never open
  the CSI camera. nvargus + the sensors themselves are fine (gst-launch works).
  So we pull frames through the GStreamer python bindings (gi) + appsink, and use
  cv2 only for drawing + the QT5 imshow window (highgui works, GStreamer doesn't).

Usage:
  python3 scripts/live_person_detect.py                  # live window, detects "person"
  python3 scripts/live_person_detect.py --sensor-id 1    # body cam (low distortion)
  python3 scripts/live_person_detect.py --classes "person,bottle,cell phone"
  python3 scripts/live_person_detect.py --shot /tmp/shot.jpg   # headless: grab+annotate 1 frame, exit
  Quit the live window with  q  or  ESC.
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import os as _os  # noqa: E402
import sys as _sys  # noqa: E402
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import camera_config  # noqa: E402


def build_pipeline(sensor_id: int, mode: int, w: int, h: int, fps: int, flip: int,
                   wbmode: int = 1) -> str:
    # Identical capture path to robot_hardware/camera_csi_node, but terminating in a
    # named appsink we pull from in python instead of cv2's (missing) GStreamer backend.
    # wbmode: 1=auto (default), 8=shade — see wb_test.py. IMX219 here has a magenta
    # cast (weak IR-cut → low green); wbmode only balances R/B, so pair with --wb.
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={mode} wbmode={wbmode} "
        f"! video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1, format=NV12 "
        f"! nvvidconv flip-method={flip} ! video/x-raw, format=BGRx "
        f"! videoconvert ! video/x-raw, format=BGR "
        f"! appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
    )


_FIXED_WB_GAINS = (1.16, 1.08, 0.82)  # body cam B,G,R — see scripts/wb_apply.py & csi_capture.py


def gray_world(img):
    """Software white balance: equalize per-channel means. Cancels the residual
    magenta/green cast that wbmode can't fix (green channel deficiency)."""
    f = img.astype(np.float32)
    means = [f[..., c].mean() + 1e-6 for c in range(3)]
    g = sum(means) / 3.0
    for c in range(3):
        f[..., c] *= g / means[c]
    return np.clip(f, 0, 255).astype(np.uint8)


def apply_wb(frame, mode):
    if mode == "grayworld":
        return gray_world(frame)
    if mode == "fixed":                       # scene-stable: neutralize whites only
        f = frame.astype(np.float32)
        for c in range(3):
            f[..., c] *= _FIXED_WB_GAINS[c]
        return np.clip(f, 0, 255).astype(np.uint8)
    return frame


class CsiCamera:
    """Minimal GStreamer appsink reader → BGR numpy frames."""

    def __init__(self, pipeline_str: str) -> None:
        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name("sink")
        if self.sink is None:
            raise RuntimeError("appsink 'sink' not found in pipeline")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to set GStreamer pipeline to PLAYING")
        # Block until the pipeline actually reaches PLAYING (preroll done) so the
        # first try_pull_sample doesn't race ahead of the producer and return None.
        st = self.pipeline.get_state(5 * Gst.SECOND)
        if st[1] != Gst.State.PLAYING:
            raise RuntimeError(f"pipeline did not reach PLAYING (got {st[1]})")

    def read(self, timeout_s: float = 2.0):
        # Use the appsink action signal (works without importing the GstApp typelib,
        # which doesn't bind try_pull_sample as a plain method in gi 1.20).
        sample = self.sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
        if sample is None:
            return None
        buf = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        w = caps.get_value("width")
        h = caps.get_value("height")
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            # rows may be 4-byte-stride padded; reshape via actual row length then crop.
            row = mapinfo.size // h
            frame = np.frombuffer(mapinfo.data, np.uint8, count=row * h).reshape(h, row)
            frame = frame[:, : w * 3].reshape(h, w, 3).copy()
        finally:
            buf.unmap(mapinfo)
        return frame

    def close(self) -> None:
        self.pipeline.set_state(Gst.State.NULL)


# Distinct colors per class index (BGR).
_PALETTE = [
    (0, 255, 0), (0, 165, 255), (255, 128, 0), (0, 0, 255),
    (255, 0, 255), (255, 255, 0), (128, 0, 255), (0, 255, 255),
]


def draw(frame, boxes, classes, fps):
    for (x1, y1, x2, y2, conf, cls) in boxes:
        color = _PALETTE[int(cls) % len(_PALETTE)]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{classes[int(cls)]} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(frame, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
    hud = f"{fps:4.1f} FPS | {len(boxes)} det | classes={','.join(classes)}"
    cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    return frame


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="models/yolov8s-world.pt")
    ap.add_argument("--classes", default="person", help="comma-separated open-vocab prompts")
    ap.add_argument("--sensor-id", type=int, default=camera_config.BODY,
                    help="camera_config 참조 (현재 BODY=0 본체, WIDE=1 광각)")
    ap.add_argument("--mode", type=int, default=3)
    ap.add_argument("--width", type=int, default=1640)
    ap.add_argument("--height", type=int, default=1232)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--flip", type=int, default=0)
    ap.add_argument("--wbmode", type=int, default=1, help="nvargus white balance mode (1=auto, 8=shade)")
    ap.add_argument("--wb", choices=["none", "grayworld", "fixed"], default="none",
                    help="software white balance applied before inference")
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--show-width", type=int, default=960, help="window downscale width")
    ap.add_argument("--shot", default="", help="headless: grab+annotate one frame to this path, then exit")
    args = ap.parse_args()

    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    print(f"[init] loading {args.weights} ...", flush=True)
    from ultralytics import YOLOWorld
    model = YOLOWorld(args.weights)
    model.set_classes(classes)
    print(f"[init] classes set to {classes}", flush=True)

    cam = CsiCamera(build_pipeline(args.sensor_id, args.mode, args.width,
                                   args.height, args.fps, args.flip, args.wbmode))
    print(f"[init] camera sensor-id={args.sensor_id} streaming "
          f"(wbmode={args.wbmode}, sw_wb={args.wb})", flush=True)

    def infer(frame):
        r = model.predict(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        out = []
        if r.boxes is not None:
            for b in r.boxes:
                x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
                out.append((x1, y1, x2, y2, float(b.conf[0]), int(b.cls[0])))
        return out

    # ---- headless single-shot mode (for verification without a display) ----
    if args.shot:
        frame = None
        for _ in range(15):                       # let 3A settle
            f = cam.read()
            if f is not None:
                frame = f
        if frame is None:
            print("[shot] no frame from camera"); cam.close(); return 1
        frame = apply_wb(frame, args.wb)
        boxes = infer(frame)
        draw(frame, boxes, classes, 0.0)
        cv2.imwrite(args.shot, frame)
        print(f"[shot] {len(boxes)} detection(s); annotated frame -> {args.shot}", flush=True)
        cam.close()
        return 0

    # ---- live window ----
    win = "person-detect (q/ESC to quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    t_prev = time.time()
    fps = 0.0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                print("[warn] frame timeout"); continue
            frame = apply_wb(frame, args.wb)
            boxes = infer(frame)
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
            t_prev = now
            draw(frame, boxes, classes, fps)
            if args.show_width and frame.shape[1] > args.show_width:
                s = args.show_width / frame.shape[1]
                frame = cv2.resize(frame, (args.show_width, int(frame.shape[0] * s)))
            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
