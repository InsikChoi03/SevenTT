#!/usr/bin/env python3
"""Fail-closed hardware preflight for the main competition launcher."""

from __future__ import annotations

import argparse
import os
import stat
import sys


def check_serial_device(path: str) -> None:
    """Require an existing character device with effective read/write access."""
    try:
        mode = os.stat(path).st_mode
    except OSError as exc:
        raise RuntimeError(f"Arduino device unavailable: {path}: {exc}") from exc
    if not stat.S_ISCHR(mode):
        raise RuntimeError(f"Arduino path is not a character device: {path}")
    missing = []
    if not os.access(path, os.R_OK):
        missing.append("read")
    if not os.access(path, os.W_OK):
        missing.append("write")
    if missing:
        raise RuntimeError(
            f"Arduino device permission missing ({'/'.join(missing)}): {path}; "
            "add the user to dialout or fix the udev rule"
        )


def check_wide_frame(sensor_id: int, timeout_sec: float) -> tuple[int, int]:
    """Open the real CSI wide-camera pipeline and require one non-empty frame buffer."""
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"GStreamer Python bindings unavailable: {exc}") from exc

    Gst.init(None)
    pipeline_text = (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode=3 wbmode=8 "
        "! video/x-raw(memory:NVMM), width=1640, height=1232, "
        "framerate=30/1, format=NV12 "
        "! nvvidconv flip-method=2 "
        "! video/x-raw, format=BGRx, width=1280, height=960 "
        "! appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
    )
    pipeline = None
    try:
        pipeline = Gst.parse_launch(pipeline_text)
        sink = pipeline.get_by_name("sink")
        if sink is None:
            raise RuntimeError("wide-camera appsink was not created")
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("wide-camera pipeline could not enter PLAYING")
        sample = sink.emit("try-pull-sample", int(timeout_sec * Gst.SECOND))
        if sample is None:
            raise RuntimeError(
                f"wide camera sensor-id={sensor_id} produced no frame in {timeout_sec:.1f}s"
            )
        buffer = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        width = int(caps.get_value("width"))
        height = int(caps.get_value("height"))
        if buffer is None or buffer.get_size() <= 0 or width <= 0 or height <= 0:
            raise RuntimeError("wide camera returned an empty frame")
        return width, height
    except RuntimeError:
        raise
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"wide-camera frame probe failed: {exc}") from exc
    finally:
        if pipeline is not None:
            pipeline.set_state(Gst.State.NULL)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--wide-sensor-id", type=int, default=1)
    parser.add_argument("--camera-timeout-sec", type=float, default=8.0)
    args = parser.parse_args()

    try:
        check_serial_device(args.serial)
        print(f"  Arduino OK: {args.serial} (read/write)", flush=True)
        width, height = check_wide_frame(args.wide_sensor_id, args.camera_timeout_sec)
        print(
            f"  wide camera OK: sensor-id={args.wide_sensor_id} actual frame={width}x{height}",
            flush=True,
        )
    except RuntimeError as exc:
        print(f"PRECHECK FAILED: {exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
