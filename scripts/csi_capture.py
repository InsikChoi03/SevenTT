"""공용 CSI 캡처 (gi/appsink) — cv2 GStreamer 없는 Jetson용.

camera_csi_node.py 의 GstCsiCapture 와 동일 방식 (reference-jetson-cv2-no-gstreamer).
본체 카메라 = sensor-id=0 (camera_config.BODY). 풀FOV 4:3 = sensor_mode=3 (1640x1232).

화이트밸런스: 본체 IMX219는 마젠타 캐스트(녹색부족, IR-cut 약함)가 있어 nvargus wbmode만으론
못 고친다 (reference-body-cam-whitebalance). 그래서 기본값으로 wbmode=8(shade) + 흰 기준 고정게인
DEFAULT_WB_GAINS 를 적용한다. 보정 끄려면 CsiCamera(..., wb_gains=None).
상단 광각cam(sensor-id=1)은 특성이 달라 미검증 → 그 경우 wb_gains=None 권장.
"""
from __future__ import annotations

import os
import sys

import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_config  # noqa: E402  역할↔id·WB 중앙 설정

# 후방호환: 본체 cam 흰 기준 고정게인 (B, G, R). 역할별 게인은 camera_config.WB_GAINS.
DEFAULT_WB_GAINS = camera_config.WB_GAINS[camera_config.BODY]

_UNSET = object()


def make_pipeline(sensor_id, sensor_mode=3, width=1640, height=1232, fps=30, flip=0, wbmode=8):
    # wbmode 8=shade — 본체 cam에서 R/B가 가장 덜 튀는 모드 (나머지 잔차는 wb_gains가 처리).
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} wbmode={wbmode} "
        f"! video/x-raw(memory:NVMM), width={width}, height={height}, framerate={fps}/1, format=NV12 "
        f"! nvvidconv flip-method={flip} ! video/x-raw, format=BGRx ! videoconvert "
        f"! video/x-raw, format=BGR ! appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
    )


def apply_gains(frame, gains):
    """채널별 고정게인(B,G,R) 곱 — 마젠타 캐스트 중화."""
    f = frame.astype(np.float32)
    for c in range(3):
        f[..., c] *= gains[c]
    return np.clip(f, 0, 255).astype(np.uint8)


class CsiCamera:
    """gi+appsink로 BGR numpy 프레임을 뽑는다. 기본으로 화이트밸런스 고정게인 적용."""

    def __init__(self, sensor_id=camera_config.BODY, wb_gains=_UNSET, **kw):
        # wb_gains 미지정이면 sensor-id로 역할별 게인 자동 선택 (본체=보정, 광각=None).
        if wb_gains is _UNSET:
            wb_gains = camera_config.WB_GAINS.get(sensor_id)
        Gst.init(None)
        self.wb_gains = wb_gains
        self.pipeline = Gst.parse_launch(make_pipeline(sensor_id, **kw))
        try:
            self.sink = self.pipeline.get_by_name("sink")
            if self.sink is None:
                raise RuntimeError("appsink 'sink' 없음")
            if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("GStreamer PLAYING 실패")
            _, state, _ = self.pipeline.get_state(5 * Gst.SECOND)
            if state != Gst.State.PLAYING:
                raise RuntimeError(f"pipeline PLAYING 못 감 ({state})")
        except Exception:
            # 부분 개통된 파이프라인을 NULL로 내려 nvargus 센서를 반납(누수 방지) 후 재-raise.
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
            raise

    def read(self, timeout_s=1.0):
        sample = self.sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
        if sample is None:
            return None
        buf = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        w = caps.get_value("width")
        h = caps.get_value("height")
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            row = info.size // h
            arr = np.frombuffer(info.data, np.uint8, count=row * h).reshape(h, row)
            frame = arr[:, : w * 3].reshape(h, w, 3).copy()
        finally:
            buf.unmap(info)
        if self.wb_gains is not None:
            frame = apply_gains(frame, self.wb_gains)
        return frame

    def release(self):
        self.pipeline.set_state(Gst.State.NULL)
