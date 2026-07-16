"""Small no-CDN web viewer for the independent arrival parking test."""
from __future__ import annotations

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import DetectionArray
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


class ParkingWebNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_web_node")
        self.declare_parameter("port", 8090)
        self.declare_parameter("wide_image_topic", "/camera_top/image_raw")
        self.declare_parameter("body_image_topic", "/camera_body/image_raw")
        self.declare_parameter("wide_detections_topic", "/camera_top/detections")
        self.declare_parameter("body_detections_topic", "/camera_body/detections")
        self.declare_parameter("state_topic", "/parking_test/state")
        self.declare_parameter("debug_topic", "/parking_test/debug")
        self.declare_parameter("jpeg_quality", 78)
        self.declare_parameter("stream_fps", 12.0)

        self.port = int(self.get_parameter("port").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.stream_period = 1.0 / max(1.0, float(self.get_parameter("stream_fps").value))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames: dict[str, np.ndarray | None] = {"wide": None, "body": None}
        self.detections: dict[str, list[dict]] = {"wide": [], "body": []}
        self.state_text = ""
        self.debug = []
        self.last_image_t = {"wide": 0.0, "body": 0.0}
        self.last_det_t = {"wide": 0.0, "body": 0.0}
        self.last_state_t = 0.0

        self.create_subscription(
            Image,
            str(self.get_parameter("wide_image_topic").value),
            lambda msg: self._on_image("wide", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("body_image_topic").value),
            lambda msg: self._on_image("body", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DetectionArray,
            str(self.get_parameter("wide_detections_topic").value),
            lambda msg: self._on_detections("wide", msg),
            10,
        )
        self.create_subscription(
            DetectionArray,
            str(self.get_parameter("body_detections_topic").value),
            lambda msg: self._on_detections("body", msg),
            10,
        )
        self.create_subscription(String, str(self.get_parameter("state_topic").value), self._on_state, 10)
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("debug_topic").value),
            self._on_debug,
            10,
        )

        self.web_dir = os.path.join(get_package_share_directory("robot_parking_test"), "web")
        handler = self._make_handler()
        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), handler)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f"parking web viewer: http://127.0.0.1:{self.port}/")

    def destroy_node(self) -> bool:
        self.httpd.shutdown()
        self.httpd.server_close()
        return super().destroy_node()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_image(self, key: str, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"{key} image conversion failed: {exc}", throttle_duration_sec=2.0)
            return
        with self.lock:
            self.frames[key] = frame
            self.last_image_t[key] = self._now_s()

    def _on_detections(self, key: str, msg: DetectionArray) -> None:
        dets = []
        for det in msg.detections:
            dets.append(
                {
                    "label": str(det.label),
                    "confidence": float(det.confidence),
                    "x": float(det.x_center),
                    "y": float(det.y_center),
                    "w": float(det.width),
                    "h": float(det.height),
                }
            )
        with self.lock:
            self.detections[key] = dets
            self.last_det_t[key] = self._now_s()

    def _on_state(self, msg: String) -> None:
        with self.lock:
            self.state_text = str(msg.data)
            self.last_state_t = self._now_s()

    def _on_debug(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.debug = [float(v) for v in msg.data]

    def _jpeg(self, key: str) -> bytes:
        with self.lock:
            frame = None if self.frames[key] is None else self.frames[key].copy()
            dets = list(self.detections[key])
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, f"waiting for {key} image", (30, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
        else:
            self._draw_detections(frame, dets)
        ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return enc.tobytes() if ok else b""

    @staticmethod
    def _draw_detections(frame, dets) -> None:
        for det in dets:
            x = int(round(det["x"] - det["w"] * 0.5))
            y = int(round(det["y"] - det["h"] * 0.5))
            w = int(round(det["w"]))
            h = int(round(det["h"]))
            label = det["label"]
            color = (70, 220, 120) if label == "arrival" else (80, 170, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = f"{label} {det['confidence']:.2f}"
            cv2.putText(frame, text, (x, max(20, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def _state_json(self) -> bytes:
        now = self._now_s()
        with self.lock:
            text = self.state_text
            dbg = list(self.debug)
            freshness = {
                "wide_image_age": now - self.last_image_t["wide"] if self.last_image_t["wide"] else None,
                "body_image_age": now - self.last_image_t["body"] if self.last_image_t["body"] else None,
                "wide_detection_age": now - self.last_det_t["wide"] if self.last_det_t["wide"] else None,
                "body_detection_age": now - self.last_det_t["body"] if self.last_det_t["body"] else None,
                "state_age": now - self.last_state_t if self.last_state_t else None,
            }
        fsm = "UNKNOWN"
        for part in text.split():
            if part.startswith("state="):
                fsm = part.split("=", 1)[1]
                break
        fresh_text = _fresh_text(freshness)
        return json.dumps(
            {"state_text": text, "fsm": fsm, "debug": dbg, "freshness": fresh_text},
            ensure_ascii=False,
        ).encode("utf-8")

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: D401
                return

            def parse_request(self):  # noqa: D401
                if self.raw_requestline.startswith(b"\x00"):
                    self.raw_requestline = self.raw_requestline.lstrip(b"\x00")
                return super().parse_request()

            def do_GET(self):  # noqa: N802
                path = urlparse(self.path).path
                if path == "/":
                    self._send_file("index.html", "text/html; charset=utf-8")
                elif path == "/styles.css":
                    self._send_file("styles.css", "text/css; charset=utf-8")
                elif path == "/app.js":
                    self._send_file("app.js", "application/javascript; charset=utf-8")
                elif path == "/state.json":
                    self._send_bytes(node._state_json(), "application/json; charset=utf-8")
                elif path == "/wide.mjpg":
                    self._stream("wide")
                elif path == "/body.mjpg":
                    self._stream("body")
                elif path == "/favicon.ico":
                    self._send_bytes(b"", "image/x-icon")
                else:
                    self.send_error(404)

            def _send_file(self, name: str, ctype: str) -> None:
                full = os.path.join(node.web_dir, name)
                try:
                    with open(full, "rb") as f:
                        data = f.read()
                except OSError:
                    self.send_error(404)
                    return
                self._send_bytes(data, ctype)

            def _send_bytes(self, data: bytes, ctype: str) -> None:
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            def _stream(self, key: str) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while rclpy.ok():
                        jpg = node._jpeg(key)
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        time.sleep(node.stream_period)
                except (BrokenPipeError, ConnectionResetError):
                    return

        return Handler


def _fresh_text(freshness: dict) -> str:
    vals = []
    for key, value in freshness.items():
        if value is None:
            vals.append(f"{key}=none")
        else:
            vals.append(f"{key}={value:.1f}s")
    return " ".join(vals)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
