"""CSI IMX219 camera publisher via nvarguscamerasrc -> sensor_msgs/Image.

NOTE on capture path: this Jetson's cv2 (4.11.0) is built with GStreamer=NO, so
cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER) silently fails to open the CSI camera
(isOpened()==False) even though nvargus and the sensors are fine. We therefore pull
frames through the GStreamer python bindings (gi) + a named appsink, and build the
Image message by hand (no cv2 / cv_bridge needed for BGR). See memory:
reference-jetson-cv2-no-gstreamer. Verified working reference: scripts/live_person_detect.py.
"""
from __future__ import annotations

import array

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def _nvargus_source_props(exposuretimerange: str = "", gainrange: str = "",
                          aelock: bool = False, awblock: bool = False,
                          nvargus_extra: str = "") -> str:
    props = []
    if exposuretimerange:
        props.append(f'exposuretimerange="{exposuretimerange}"')
    if gainrange:
        props.append(f'gainrange="{gainrange}"')
    if aelock:
        props.append("aelock=true")
    if awblock:
        props.append("awblock=true")
    if nvargus_extra:
        props.append(nvargus_extra.strip())
    return (" " + " ".join(props)) if props else ""


def make_gst_pipeline(sensor_id: int, sensor_mode: int, width: int, height: int, fps: int,
                      flip: int, wbmode: int, out_width: int = 0, out_height: int = 0,
                      exposuretimerange: str = "", gainrange: str = "",
                      aelock: bool = False, awblock: bool = False,
                      nvargus_extra: str = "") -> str:
    # nvvidconv (VIC) rescales on-GPU for free: keep the sensor caps at full FOV (width/height) and
    # ask the OUTPUT caps for a smaller frame. Per-frame CPU (all the copies + DDS serialize + every
    # subscriber's deserialize) is O(pixels), so this is the one lever that cuts it enough for 15 Hz.
    # out_*=0 -> passthrough (full sensor resolution, original behaviour).
    out_caps = "video/x-raw, format=BGRx"
    if out_width > 0 and out_height > 0:
        out_caps += f", width={out_width}, height={out_height}"
    source_props = _nvargus_source_props(
        exposuretimerange, gainrange, aelock, awblock, nvargus_extra
    )
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} wbmode={wbmode}"
        f"{source_props} "
        f"! video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"framerate={fps}/1, format=NV12 "
        f"! nvvidconv flip-method={flip} "
        f"! {out_caps} "
        # NO CPU videoconvert: appsink takes BGRx (GPU nvvidconv output) and read() strips the X
        # byte via cv2.cvtColor (SIMD). videoconvert(BGRx->BGR) was ~135% CPU/cam -> the rate cap.
        f"! appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false"
    )


class GstCsiCapture:
    """Pull BGR numpy frames from an nvargus pipeline via gi + appsink."""

    def __init__(self, pipeline_str: str) -> None:
        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name("sink")
        if self.sink is None:
            raise RuntimeError("appsink 'sink' not found in pipeline")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("failed to set GStreamer pipeline to PLAYING")
        # Block until preroll completes so the first pull doesn't race the producer.
        _, state, _ = self.pipeline.get_state(5 * Gst.SECOND)
        if state != Gst.State.PLAYING:
            raise RuntimeError(f"pipeline did not reach PLAYING (got {state})")

    def read(self, timeout_s: float):
        # Action signal (gi 1.20 doesn't bind try_pull_sample as a plain method).
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
            # BGRx = 4 bytes/pixel (B,G,R,X). Reshape by actual (possibly padded) row length, crop to
            # w*4, view as (h,w,4), then cv2.cvtColor(BGRA->BGR) drops the X byte in one SIMD pass —
            # ~3x faster than the numpy strided [:, :, :3].copy() (per-pixel gather) it replaces.
            row = info.size // h
            arr = np.frombuffer(info.data, np.uint8, count=row * h).reshape(h, row)
            frame = cv2.cvtColor(arr[:, : w * 4].reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
        finally:
            buf.unmap(info)
        return frame, w, h

    def release(self) -> None:
        self.pipeline.set_state(Gst.State.NULL)


class CameraCsiNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_csi_node")

        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("sensor_mode", 3)
        self.declare_parameter("width", 1640)
        self.declare_parameter("height", 1232)
        self.declare_parameter("fps", 30)
        self.declare_parameter("flip_method", 0)
        # MUST match the training-capture pipeline (csi_capture.py): wbmode=8 (shade). The old
        # default (no wbmode -> nvargus auto) fed YOLO a different colour cast than it trained on.
        self.declare_parameter("wbmode", 8)
        # Manual exposure/gain hooks. Empty ranges keep nvargus defaults; non-empty values should
        # use the gst-inspect format, e.g. "8000000 8000000" ns and "1 1" gain.
        self.declare_parameter("exposuretimerange", "")
        self.declare_parameter("gainrange", "")
        self.declare_parameter("aelock", False)
        self.declare_parameter("awblock", False)
        self.declare_parameter("nvargus_extra", "")
        # Post-capture per-channel WB gains [B,G,R] (body cam training used 1.16/1.08/0.82 to kill
        # the magenta cast). [1,1,1] = off (wide cam).
        self.declare_parameter("wb_gains", [1.0, 1.0, 1.0])
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("topic", "image_raw")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("pull_timeout", 0.5)
        self.declare_parameter("reconnect_delay_sec", 2.0)
        self.declare_parameter("max_timeouts_before_reconnect", 5)
        # GPU-side downscale (nvvidconv VIC). 0 = passthrough (full sensor res). Setting these cuts
        # per-frame CPU ~linearly in pixel count — the lever for 15 Hz. Any consumer that projects
        # detection pixels (world_model/mission_fsm homographies, wide intrinsics) MUST scale to match.
        self.declare_parameter("out_width", 0)
        self.declare_parameter("out_height", 0)

        self.sensor_id = int(self.get_parameter("sensor_id").value)
        self.sensor_mode = int(self.get_parameter("sensor_mode").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.flip = int(self.get_parameter("flip_method").value)
        self.wbmode = int(self.get_parameter("wbmode").value)
        self.exposuretimerange = str(self.get_parameter("exposuretimerange").value).strip()
        self.gainrange = str(self.get_parameter("gainrange").value).strip()
        self.aelock = bool(self.get_parameter("aelock").value)
        self.awblock = bool(self.get_parameter("awblock").value)
        self.nvargus_extra = str(self.get_parameter("nvargus_extra").value).strip()
        gains = [float(g) for g in self.get_parameter("wb_gains").value]
        self._wb = np.array(gains[:3], dtype=np.float32).reshape(1, 1, 3) if len(gains) >= 3 else None
        self._apply_wb = self._wb is not None and not np.allclose(self._wb, 1.0)
        # Precompute a per-channel uint8 LUT so tick() does one cv2.LUT (SIMD) instead of a
        # float32 multiply over the whole frame (measured 62 ms -> 43 ms full-res). Shape (1,256,3),
        # gains order [B,G,R] matching bgr8. Numerically identical to clip(x*gain, 0, 255).
        self._wb_lut = None
        if self._apply_wb:
            g = self._wb.reshape(3)  # [B,G,R]
            lut = np.clip(np.arange(256, dtype=np.float32)[:, None] * g.reshape(1, 3), 0, 255)
            self._wb_lut = lut.astype(np.uint8).reshape(1, 256, 3)
        self.out_width = int(self.get_parameter("out_width").value)
        self.out_height = int(self.get_parameter("out_height").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.pull_timeout = float(self.get_parameter("pull_timeout").value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay_sec").value)
        self.max_timeouts_before_reconnect = int(
            self.get_parameter("max_timeouts_before_reconnect").value
        )
        self.pipeline = make_gst_pipeline(self.sensor_id, self.sensor_mode, self.width, self.height,
                                          self.fps, self.flip, self.wbmode,
                                          self.out_width, self.out_height,
                                          self.exposuretimerange, self.gainrange,
                                          self.aelock, self.awblock, self.nvargus_extra)
        self.get_logger().info(f"GStreamer pipeline: {self.pipeline}")

        self.cap = None
        self._next_connect_s = 0.0
        self._timeout_count = 0
        # SENSOR_DATA QoS (BEST_EFFORT, keep-last): a 6 MB frame at 30 Hz over RELIABLE to several
        # subscribers collapsed the delivered rate to ~1 Hz (retransmit storm). BEST_EFFORT lets each
        # consumer just take the freshest frame and drop the rest — the camera streams at full rate.
        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"sensor-id={self.sensor_id} mode={self.sensor_mode} "
            f"{self.width}x{self.height}@{self.fps}fps frame={self.frame_id} topic={topic} (gi/appsink)"
        )
        self._connect()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _drop_capture(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception as exc:  # noqa: BLE001 - best-effort cleanup
                self.get_logger().warn(f"failed to release camera pipeline cleanly: {exc}")
        self.cap = None

    def _connect(self) -> bool:
        now = self._now_s()
        if now < self._next_connect_s:
            return False
        try:
            self.cap = GstCsiCapture(self.pipeline)
        except Exception as exc:  # noqa: BLE001 - keep node alive; launch respawn is last resort
            self.cap = None
            self._next_connect_s = now + max(0.5, self.reconnect_delay)
            self.get_logger().error(
                f"camera open failed for sensor-id={self.sensor_id}; "
                f"retrying in {self.reconnect_delay:.1f}s: {exc}",
                throttle_duration_sec=2.0,
            )
            return False
        self._timeout_count = 0
        self.get_logger().info(f"camera stream connected on sensor-id={self.sensor_id}")
        return True

    def tick(self) -> None:
        if self.cap is None and not self._connect():
            return
        try:
            result = self.cap.read(self.pull_timeout)
        except Exception as exc:  # noqa: BLE001 - Argus/GStreamer can fail after startup
            self.get_logger().error(
                f"camera read failed; reconnecting: {exc}",
                throttle_duration_sec=2.0,
            )
            self._drop_capture()
            self._next_connect_s = self._now_s() + max(0.5, self.reconnect_delay)
            return
        if result is None:
            self._timeout_count += 1
            self.get_logger().warn(
                f"frame pull timed out ({self._timeout_count}/"
                f"{self.max_timeouts_before_reconnect})",
                throttle_duration_sec=2.0,
            )
            if (
                self.max_timeouts_before_reconnect > 0
                and self._timeout_count >= self.max_timeouts_before_reconnect
            ):
                self.get_logger().warn("too many camera timeouts; reconnecting pipeline")
                self._drop_capture()
                self._next_connect_s = self._now_s() + max(0.5, self.reconnect_delay)
            return
        self._timeout_count = 0
        frame, w, h = result
        if self._wb_lut is not None:
            frame = cv2.LUT(frame, self._wb_lut)   # per-channel WB, SIMD (was a float32 whole-frame mul)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        # CRITICAL: array.array('B', frame.tobytes()) is the ONLY fast form (~1 ms). Both
        #   msg.data = bytes            (rclpy per-element uint8[] setter) and
        #   array.array('B', memoryview(frame).cast('B'))   (array.array per-element from a buffer)
        # take ~300-1000 ms for a multi-MB frame and re-cap the camera at <1 Hz. tobytes() first
        # (its copy is included in the 1 ms) then array.array over that bytes object.
        msg.data = array.array("B", frame.tobytes())
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        self._drop_capture()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraCsiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
