#!/usr/bin/env python3
"""Interactive checkerboard capture for the 160-deg wide CSI camera.

Frames come through gi/appsink (this Jetson's cv2 has GStreamer=NO, so
cv2.VideoCapture(CAP_GSTREAMER) cannot open CSI — see memory
reference-jetson-cv2-no-gstreamer). cv2 is used only for corner detection + the
QT5 preview window.

Live preview turns GREEN when the full board is detected. Press SPACE to save
that frame, q/ESC to quit. Aim for ~20 views: tilt the board, move it to every
corner of the frame (especially the distorted edges), vary distance.

  python3 scripts/calib_capture.py                 # wide cam (sensor-id 0), 9x6 board
  python3 scripts/calib_capture.py --cols 9 --rows 6 --out data/calib/top
Then solve:  python3 scripts/calib_solve.py --write
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import sys as _sys
_sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402  광각=camera_config.WIDE


class CsiCamera:
    """gi + appsink BGR frame reader (same proven path as live_person_detect.py)."""

    def __init__(self, sensor_id, mode, w, h, fps, flip):
        Gst.init(None)
        pl = (
            f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={mode} "
            f"! video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1, format=NV12 "
            f"! nvvidconv flip-method={flip} ! video/x-raw, format=BGRx ! videoconvert "
            f"! video/x-raw, format=BGR ! appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
        )
        self.pipeline = Gst.parse_launch(pl)
        self.sink = self.pipeline.get_by_name("sink")
        self.pipeline.set_state(Gst.State.PLAYING)
        _, st, _ = self.pipeline.get_state(5 * Gst.SECOND)
        if st != Gst.State.PLAYING:
            raise RuntimeError(f"camera did not reach PLAYING (got {st})")

    def read(self, timeout_s=2.0):
        s = self.sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
        if s is None:
            return None
        b = s.get_buffer()
        c = s.get_caps().get_structure(0)
        w, h = c.get_value("width"), c.get_value("height")
        ok, mi = b.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            row = mi.size // h
            f = np.frombuffer(mi.data, np.uint8, count=row * h).reshape(h, row)[:, : w * 3].reshape(h, w, 3).copy()
        finally:
            b.unmap(mi)
        return f

    def close(self):
        self.pipeline.set_state(Gst.State.NULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor-id", type=int, default=camera_config.WIDE, help="기본=광각(camera_config.WIDE)")
    ap.add_argument("--cols", type=int, default=9, help="inner corners along the long side")
    ap.add_argument("--rows", type=int, default=6, help="inner corners along the short side")
    ap.add_argument("--out", default="data/calib/top")
    ap.add_argument("--mode", type=int, default=3)
    ap.add_argument("--width", type=int, default=1640)
    ap.add_argument("--height", type=int, default=1232)
    ap.add_argument("--flip", type=int, default=0)
    ap.add_argument("--show-width", type=int, default=960)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    n = len([f for f in os.listdir(args.out) if f.lower().endswith((".png", ".jpg"))])
    pattern = (args.cols, args.rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK

    cam = CsiCamera(args.sensor_id, args.mode, args.width, args.height, 30, args.flip)
    win = "calib capture  [SPACE]=save (when green)  [q]=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print(f"[capture] out={args.out}  board={args.cols}x{args.rows}  already have {n}.  Aim ~20 varied views.")
    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, flags)
            disp = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, pattern, corners, found)
            color = (0, 255, 0) if found else (40, 40, 220)
            txt = f"{'BOARD OK - press SPACE' if found else 'no board'}   saved={n}"
            cv2.putText(disp, txt, (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
            if args.show_width and disp.shape[1] > args.show_width:
                s = args.show_width / disp.shape[1]
                disp = cv2.resize(disp, (args.show_width, int(disp.shape[0] * s)))
            cv2.imshow(win, disp)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == 32:
                if found:
                    p = os.path.join(args.out, f"img_{n:02d}.png")
                    cv2.imwrite(p, frame)
                    n += 1
                    print(f"[capture] saved {p}  (total {n})")
                else:
                    print("[capture] no board detected — not saved")
    finally:
        cam.close()
        cv2.destroyAllWindows()
    print(f"[capture] done. {n} image(s) in {args.out}.  Next: python3 scripts/calib_solve.py --write")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
