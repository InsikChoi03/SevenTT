"""CSI IMX219 camera publisher via nvarguscamerasrc -> sensor_msgs/Image.

NOTE on capture path: this Jetson's cv2 (4.11.0) is built with GStreamer=NO, so
cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER) silently fails to open the CSI camera
(isOpened()==False) even though nvargus and the sensors are fine. We therefore pull
frames through the GStreamer python bindings (gi) + a named appsink, and build the
Image message by hand (no cv2 / cv_bridge needed for BGR). See memory:
reference-jetson-cv2-no-gstreamer. Verified working reference: scripts/live_person_detect.py.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def make_gst_pipeline(sensor_id: int, sensor_mode: int, width: int, height: int, fps: int, flip: int) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} "
        f"! video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"framerate={fps}/1, format=NV12 "
        f"! nvvidconv flip-method={flip} "
        f"! video/x-raw, format=BGRx "
        f"! videoconvert "
        f"! video/x-raw, format=BGR "
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
            # Rows can be 4-byte-stride padded; reshape by actual row length then crop.
            row = info.size // h
            arr = np.frombuffer(info.data, np.uint8, count=row * h).reshape(h, row)
            frame = arr[:, : w * 3].reshape(h, w, 3).copy()
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
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("topic", "image_raw")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("pull_timeout", 0.5)

        self.sensor_id = int(self.get_parameter("sensor_id").value)
        self.sensor_mode = int(self.get_parameter("sensor_mode").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.flip = int(self.get_parameter("flip_method").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.pull_timeout = float(self.get_parameter("pull_timeout").value)

        pipeline = make_gst_pipeline(self.sensor_id, self.sensor_mode, self.width, self.height, self.fps, self.flip)
        self.get_logger().info(f"GStreamer pipeline: {pipeline}")

        self.cap = GstCsiCapture(pipeline)
        self.pub = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"sensor-id={self.sensor_id} mode={self.sensor_mode} "
            f"{self.width}x{self.height}@{self.fps}fps frame={self.frame_id} topic={topic} (gi/appsink)"
        )

    def tick(self) -> None:
        result = self.cap.read(self.pull_timeout)
        if result is None:
            self.get_logger().warn("frame pull timed out", throttle_duration_sec=2.0)
            return
        frame, w, h = result
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        if getattr(self, "cap", None) is not None:
            self.cap.release()
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
