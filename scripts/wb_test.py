#!/usr/bin/env python3
"""본체 CSI 카메라 화이트밸런스 테스트.

nvarguscamerasrc 의 hw wbmode 를 골라 한 프레임 잡고, 같은 프레임에서
소프트웨어 보정 2종(gray-world / white-patch)을 함께 만들어 저장한다.
cv2 GStreamer 없는 Jetson용 → gi/appsink 캡처 (csi_capture 동일 방식).

  python3 scripts/wb_test.py --wbmode 1 --out data/cam_test/wb_auto
  python3 scripts/wb_test.py --wbmode 5 --out data/cam_test/wb_daylight
  # → <out>_raw.png, <out>_grayworld.png, <out>_whitepatch.png 3장
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def make_pipeline(sensor_id, wbmode, sensor_mode=3, width=1640, height=1232, fps=30, flip=0,
                  saturation=1.0):
    # wbmode: 0 off, 1 auto, 2 incandescent, 3 fluorescent, 4 warm-fl, 5 daylight,
    #         6 cloudy, 7 twilight, 8 shade, 9 manual
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} "
        f"wbmode={wbmode} saturation={saturation} "
        f"! video/x-raw(memory:NVMM), width={width}, height={height}, framerate={fps}/1, format=NV12 "
        f"! nvvidconv flip-method={flip} ! video/x-raw, format=BGRx ! videoconvert "
        f"! video/x-raw, format=BGR ! appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
    )


def capture(pipeline_str, warmup=15, timeout_s=2.0):
    Gst.init(None)
    pipe = Gst.parse_launch(pipeline_str)
    sink = pipe.get_by_name("sink")
    pipe.set_state(Gst.State.PLAYING)
    if pipe.get_state(5 * Gst.SECOND)[1] != Gst.State.PLAYING:
        pipe.set_state(Gst.State.NULL)
        raise RuntimeError("pipeline PLAYING 실패")
    frame = None
    try:
        for _ in range(warmup):           # AWB/AE 수렴 대기
            s = sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
            if s is None:
                continue
            buf = s.get_buffer()
            caps = s.get_caps().get_structure(0)
            w = caps.get_value("width"); h = caps.get_value("height")
            ok, info = buf.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                row = info.size // h
                arr = np.frombuffer(info.data, np.uint8, count=row * h).reshape(h, row)
                frame = arr[:, : w * 3].reshape(h, w, 3).copy()
            finally:
                buf.unmap(info)
    finally:
        pipe.set_state(Gst.State.NULL)
    return frame


def gray_world(img):
    """전체 평균색을 회색으로 — 장면에 색이 고르게 분포할 때 좋음."""
    f = img.astype(np.float32)
    means = [f[..., c].mean() + 1e-6 for c in range(3)]
    g = sum(means) / 3.0
    for c in range(3):
        f[..., c] *= g / means[c]
    return np.clip(f, 0, 255).astype(np.uint8)


def white_patch(img, pct=97.0):
    """밝은 픽셀(흰 물체)을 흰색으로 — 흰 도형 테스트에 적합."""
    f = img.astype(np.float32)
    for c in range(3):
        ref = np.percentile(f[..., c], pct) + 1e-6
        f[..., c] *= 255.0 / ref
    return np.clip(f, 0, 255).astype(np.uint8)


def channel_means(img):
    b, g, r = (float(img[..., c].mean()) for c in range(3))
    return f"B={b:.1f} G={g:.1f} R={r:.1f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor-id", type=int, default=1)
    ap.add_argument("--wbmode", type=int, default=1)
    ap.add_argument("--saturation", type=float, default=1.0)
    ap.add_argument("--warmup", type=int, default=15)
    ap.add_argument("--out", default="data/cam_test/wb")
    args = ap.parse_args()

    frame = capture(make_pipeline(args.sensor_id, args.wbmode, saturation=args.saturation),
                    warmup=args.warmup)
    if frame is None:
        print("캡처 실패"); return 1

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    outs = {
        "raw": frame,
        "grayworld": gray_world(frame),
        "whitepatch": white_patch(frame),
    }
    for name, img in outs.items():
        p = f"{args.out}_{name}.png"
        cv2.imwrite(p, img)
        print(f"{p:48s} {channel_means(img)}")
    print(f"(wbmode={args.wbmode} saturation={args.saturation})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
