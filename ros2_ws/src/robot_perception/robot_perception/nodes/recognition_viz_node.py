"""Recognition-verification / 2D-map visualiser for the mock field test.

Pure sink node: it subscribes to the whole perception+planning bus and produces one combined
panel — the live 2D field map (objects fused from both cameras with evidence count + source,
robot pose + trajectory, current target) NEXT TO the two YOLO-annotated camera feeds (wide +
body) — plus a timestamped event log. Runs headless by default: it overwrites live.png every
redraw (open it in the IDE for a near-real-time view) and saves periodic snapshots; set
show_window:=true for a live cv2 window when a display is attached.

Subscribes:
  /world_model              robot_interfaces/WorldModel     — fused tracks + robot pose (authoritative)
  /localization/pose        geometry_msgs/PoseStamped       — higher-rate pose for a smooth trail
  /camera_top/detections    robot_interfaces/DetectionArray — wide-cam raw detections (HUD counts)
  /camera_body/detections   robot_interfaces/DetectionArray — body-cam raw detections (HUD counts)
  /classification/siglip    robot_interfaces/Classification — body fruit type (logged with picks)
  /classification/shape     robot_interfaces/Classification — body shape class (logged with picks)
  /selected_target          robot_interfaces/Object         — highlighted target
  /mission_state            robot_interfaces/MissionState   — state/tray counts/current target
  /planning/phase           std_msgs/Int8                   — Set1/Set2 pick phase (HUD)

Artefacts under output_dir/<YYYYmmdd_HHMMSS>/:
  events.csv / events.jsonl   — one row per first-seen object, per pick, and per state change
  map_<seq>.png               — periodic annotated map snapshots
  map_final.png + summary.txt — written on shutdown
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int8, String
from robot_interfaces.msg import Classification, DetectionArray, MissionState, Object, WorldModel

try:
    import cv2
    import numpy as np

    _CV2_AVAILABLE = True
except Exception:  # noqa: BLE001 - node stays alive (log-only) without cv2
    _CV2_AVAILABLE = False

try:
    from cv_bridge import CvBridge

    _BRIDGE_AVAILABLE = True
except Exception:  # noqa: BLE001 - camera panels disabled without cv_bridge
    _BRIDGE_AVAILABLE = False

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class _MjpegStreamer:
    """Tiny threaded MJPEG-over-HTTP server. Serves the latest map frame as a continuous
    multipart stream so a laptop browser (http://<jetson-ip>:<port>/) shows the live map like a
    video — no file-reload black flicker. Assign the newest JPEG bytes to `.latest` each redraw."""

    def __init__(self, port: int) -> None:
        self.latest: bytes | None = None
        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):   # silence per-request console spam
                pass

            def do_GET(self):
                if self.path == "/favicon.ico":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache, private")
                self.end_headers()
                try:
                    while True:
                        buf = streamer.latest
                        if buf is not None:
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                + str(len(buf)).encode() + b"\r\n\r\n" + buf + b"\r\n"
                            )
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass   # browser tab closed

        self._httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    def start(self) -> None:
        self._thread.start()


# BGR colours by set_type (and states).
_COL_SET1 = (80, 200, 80)      # shapes -> green
_COL_SET2 = (0, 150, 255)      # fruit cubes -> orange
_COL_UNKNOWN = (170, 170, 170)  # unknown -> gray
_COL_BLACKLIST = (90, 90, 90)  # picked/passed -> dark gray
_COL_TARGET = (0, 255, 255)    # current target ring -> yellow
_COL_ROBOT = (255, 90, 40)     # robot -> blue
_COL_TRAIL = (200, 130, 60)    # trajectory
_COL_GRID = (60, 60, 60)
_COL_TEXT = (235, 235, 235)

_SHAPE_LABELS = frozenset({"cube", "octahedron", "dodecahedron", "icosahedron"})
_FRUIT_LABELS = frozenset({"apple", "orange", "banana", "pineapple"})

# ---- intuitive encoding: shapes by FORM, fruits by COLOUR ----
# Shape marker form = (n_sides, rotation): cube=square, octa=diamond, dodeca=pentagon, icosa=triangle.
_SHAPE_FORM = {
    "cube": (4, math.pi / 4), "octahedron": (4, 0.0),
    "dodecahedron": (5, -math.pi / 2), "icosahedron": (3, -math.pi / 2),
}
_COL_SHAPE = (110, 225, 130)       # all shapes share this green; the FORM distinguishes them
# Per-fruit colours (BGR) roughly matching the real fruit.
_FRUIT_COLORS = {
    "apple": (48, 48, 220),        # red
    "orange": (0, 140, 255),       # orange
    "banana": (70, 225, 245),      # yellow
    "pineapple": (60, 200, 225),   # gold
}
_COL_FRUITCUBE = (205, 90, 200)    # generic fruit_photo_cube (fruit type not read yet) -> purple


def _reg_poly(cx: int, cy: int, r: float, n: int, rot: float):
    return np.array(
        [[int(round(cx + r * math.cos(rot + 2 * math.pi * k / n))),
          int(round(cy + r * math.sin(rot + 2 * math.pi * k / n)))] for k in range(n)],
        np.int32,
    )


def _label_color(label: str):
    """Camera-panel box colour: shapes green, specific fruit by colour, generic fruit-cube purple."""
    if label in _SHAPE_FORM:
        return _COL_SHAPE
    if label in _FRUIT_COLORS:
        return _FRUIT_COLORS[label]
    if label == "fruit_photo_cube":
        return _COL_FRUITCUBE
    return _COL_UNKNOWN


class RecognitionVizNode(Node):
    def __init__(self) -> None:
        super().__init__("recognition_viz_node")

        self.declare_parameter("output_dir", "data/mock_field_test")
        self.declare_parameter("field_extent_m", [-2.0, 2.0, -2.0, 2.0])  # [xmin,xmax,ymin,ymax]
        self.declare_parameter("map_rotate_180", False)   # flip the map to match the wide feed's orientation
        # Auto-fit the map to cover all tracked objects + robot + trail (handles a 4x4 field with
        # 20+ objects placed anywhere; falls back to field_extent_m when nothing is tracked yet).
        self.declare_parameter("auto_extent", True)
        self.declare_parameter("canvas_px", 800)
        self.declare_parameter("snapshot_interval_sec", 5.0)
        self.declare_parameter("show_window", False)
        self.declare_parameter("trail_max_points", 2000)
        self.declare_parameter("redraw_rate_hz", 5.0)
        # Show the two YOLO-annotated camera feeds beside the map, and keep overwriting a single
        # live.png every redraw (open it in the IDE for a near-real-time headless view).
        self.declare_parameter("show_camera_panels", True)
        self.declare_parameter("write_live_png", True)
        # Training-data capture: during a real arena run, dump raw wide/body frames + their YOLO
        # detections as pre-labels (YOLO txt) so the run doubles as a labelling session. Only frames
        # that actually carry >=1 detection are saved (empty frames are useless as training data).
        self.declare_parameter("save_training_data", False)
        self.declare_parameter("training_interval_sec", 2.0)
        # MJPEG stream: open http://<jetson-ip>:<port>/ in a laptop browser for a smooth live map
        # (no live.png reload flicker). 0 disables. Reachable over the AP (10.42.0.1) / Tailscale.
        self.declare_parameter("mjpeg_port", 8080)
        # A track is CONFIRMED (the trustworthy map layer) if both cams matched it, or the reliable
        # body cam saw it, or a wide-only track reached this many observations (rules out a flicker).
        # Below that it is PROVISIONAL (drawn hollow) — seen, but not yet map-certain.
        self.declare_parameter("confirm_min_obs", 6)
        # Camera coverage drawn on the map (base_link, follows the robot). Body = forward SECTOR
        # (부채꼴), wide = forward-biased ELLIPSE. Keep in sync with the same-named world_model params.
        self.declare_parameter("body_fov_half_deg", 34.0)
        self.declare_parameter("body_fov_near_m", 0.08)
        self.declare_parameter("body_fov_far_m", 0.6)
        self.declare_parameter("body_fov_apex_x", 0.065)
        self.declare_parameter("wide_fov_forward_m", 1.6)
        self.declare_parameter("wide_fov_lateral_m", 1.3)
        self.declare_parameter("wide_fov_center_x", 0.3)

        ext = [float(v) for v in self.get_parameter("field_extent_m").value]
        self.extent = ext if len(ext) == 4 else [-2.0, 2.0, -2.0, 2.0]
        self.auto_extent = bool(self.get_parameter("auto_extent").value)
        self._extent_now = list(self.extent)   # extent used for the current frame (auto or fixed)
        self.canvas_px = int(self.get_parameter("canvas_px").value)
        self.map_rotate_180 = bool(self.get_parameter("map_rotate_180").value)
        self.snapshot_interval = float(self.get_parameter("snapshot_interval_sec").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.redraw_rate = float(self.get_parameter("redraw_rate_hz").value)
        trail_max = int(self.get_parameter("trail_max_points").value)
        self.show_cams = bool(self.get_parameter("show_camera_panels").value) and _BRIDGE_AVAILABLE
        self.write_live = bool(self.get_parameter("write_live_png").value)
        self.save_training = bool(self.get_parameter("save_training_data").value) and _BRIDGE_AVAILABLE
        self.training_interval = float(self.get_parameter("training_interval_sec").value)
        self._last_training_save = 0.0
        # YOLO class order shared by wide.pt / cube.pt (see project_shape_id_finding memory).
        self._train_classes = ["cube", "octahedron", "dodecahedron", "icosahedron", "fruit_photo_cube"]
        self._train_cls_idx = {c: i for i, c in enumerate(self._train_classes)}
        self.confirm_min_obs = int(self.get_parameter("confirm_min_obs").value)
        self.body_fov_half = math.radians(float(self.get_parameter("body_fov_half_deg").value))
        self.body_fov_near = float(self.get_parameter("body_fov_near_m").value)
        self.body_fov_far = float(self.get_parameter("body_fov_far_m").value)
        self.body_fov_apex_x = float(self.get_parameter("body_fov_apex_x").value)
        self.wide_fov_fwd = float(self.get_parameter("wide_fov_forward_m").value)
        self.wide_fov_lat = float(self.get_parameter("wide_fov_lateral_m").value)
        self.wide_fov_cx = float(self.get_parameter("wide_fov_center_x").value)
        self.bridge = CvBridge() if _BRIDGE_AVAILABLE else None

        # Run directory (wallclock timestamp -> unique per run).
        base = str(self.get_parameter("output_dir").value)
        self.run_dir = os.path.join(base, time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.run_dir, exist_ok=True)

        # Training-data capture dirs (YOLO layout: images/ + labels/ per camera, shared classes.txt).
        self._train_saved = 0
        if self.save_training:
            self._train_root = os.path.join(self.run_dir, "training")
            for cam in ("wide", "body"):
                os.makedirs(os.path.join(self._train_root, cam, "images"), exist_ok=True)
                os.makedirs(os.path.join(self._train_root, cam, "labels"), exist_ok=True)
            try:
                with open(os.path.join(self._train_root, "classes.txt"), "w") as f:
                    f.write("\n".join(self._train_classes) + "\n")
            except OSError:
                pass

        # Optional MJPEG stream (smooth browser view, no live.png flicker).
        self._mjpeg = None
        mjpeg_port = int(self.get_parameter("mjpeg_port").value)
        if mjpeg_port > 0 and _CV2_AVAILABLE:
            try:
                self._mjpeg = _MjpegStreamer(mjpeg_port)
                self._mjpeg.start()
                self.get_logger().info(
                    f"MJPEG 라이브맵: http://<jetson-ip>:{mjpeg_port}/  "
                    f"(AP 10.42.0.1 / Tailscale 100.122.190.95)"
                )
            except Exception as exc:  # noqa: BLE001 - port busy etc.; live.png still works
                self._mjpeg = None
                self.get_logger().warn(f"MJPEG 스트림 시작 실패 ({exc}); live.png 로만 확인")

        # Latest inbound state.
        self.world: WorldModel | None = None
        self.trail: deque[tuple[float, float]] = deque(maxlen=trail_max)
        self.mission_state = "?"
        self.tray_shape = 0
        self.tray_fruit = 0
        self.current_target_id = 0
        self.selected_id = 0
        self.phase = 0
        self.siglip: Classification | None = None
        self.shape: Classification | None = None
        # Latest raw camera image msgs + their detections (converted at draw time, ~redraw rate).
        self.top_img: Image | None = None
        self.body_img: Image | None = None
        self.top_dets: list = []
        self.body_dets: list = []
        self.proj_dets: list = []   # per-camera raw projections (x, y, src) from /world_model/projected_dets

        self.decisions: deque[str] = deque(maxlen=8)   # recent FSM decisions (PICK/PASS/SKIP/PHASE)
        self._seen_ids: set[int] = set()
        self._prev_state = ""
        self._seq = 0
        self._last_snapshot = 0.0
        self._can_show = self.show_window and _CV2_AVAILABLE
        self._t0 = time.time()

        # Event log files.
        self._csv_path = os.path.join(self.run_dir, "events.csv")
        self._jsonl_path = os.path.join(self.run_dir, "events.jsonl")
        self._csv_cols = [
            "t_iso", "elapsed_s", "event", "obj_id", "class_label", "set_type", "x", "y",
            "conf", "blacklisted", "shape_label", "shape_conf", "siglip_label", "siglip_conf",
            "image_face_visible", "phase", "mission_state", "robot_x", "robot_y", "robot_theta",
        ]
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv = csv.DictWriter(self._csv_file, fieldnames=self._csv_cols)
        self._csv.writeheader()
        self._csv_file.flush()
        self._jsonl_file = open(self._jsonl_path, "w")

        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(DetectionArray, "/camera_top/detections", self.on_top_det, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_det, 10)
        if self.show_cams or self.save_training:
            self.create_subscription(Image, "/camera_top/image_raw", self.on_top_img,
                                     qos_profile_sensor_data)  # match camera BEST_EFFORT
            self.create_subscription(Image, "/camera_body/image_raw", self.on_body_img,
                                     qos_profile_sensor_data)
        self.create_subscription(Classification, "/classification/siglip", self.on_siglip, 10)
        self.create_subscription(Classification, "/classification/shape", self.on_shape, 10)
        self.create_subscription(Object, "/selected_target", self.on_selected, 10)
        self.create_subscription(MissionState, "/mission_state", self.on_mission, 10)
        self.create_subscription(Int8, "/planning/phase", self.on_phase, 10)
        self.create_subscription(String, "/planning/decision", self.on_decision, 10)
        self.create_subscription(Float32MultiArray, "/world_model/projected_dets", self.on_proj, 10)

        self.timer = self.create_timer(1.0 / max(0.5, self.redraw_rate), self.tick)

        if not _CV2_AVAILABLE:
            self.get_logger().warn("cv2/numpy unavailable; map PNGs disabled, event log only")
        self.get_logger().info(
            f"recognition_viz -> {self.run_dir}  extent={self.extent} canvas={self.canvas_px}px "
            f"snapshot={self.snapshot_interval}s show_window={self.show_window} "
            f"cam_panels={self.show_cams} live_png={self.write_live} "
            f"save_training={self.save_training}(every {self.training_interval}s)"
        )

    # ------------------------------------------------------------------ callbacks
    def on_world(self, msg: WorldModel) -> None:
        self.world = msg
        for obj in msg.objects:
            if obj.id not in self._seen_ids:
                self._seen_ids.add(obj.id)
                self._log_event("first_seen", obj=obj)

    def on_pose(self, msg: PoseStamped) -> None:
        self.trail.append((float(msg.pose.position.x), float(msg.pose.position.y)))

    def on_top_det(self, msg: DetectionArray) -> None:
        self.top_dets = list(msg.detections)

    def on_body_det(self, msg: DetectionArray) -> None:
        self.body_dets = list(msg.detections)

    def on_top_img(self, msg: Image) -> None:
        self.top_img = msg

    def on_body_img(self, msg: Image) -> None:
        self.body_img = msg

    def on_siglip(self, msg: Classification) -> None:
        self.siglip = msg

    def on_shape(self, msg: Classification) -> None:
        self.shape = msg

    def on_selected(self, msg: Object) -> None:
        self.selected_id = int(msg.id)

    def on_mission(self, msg: MissionState) -> None:
        self.mission_state = str(msg.state)
        self.tray_shape = int(msg.tray_shape_count)
        self.tray_fruit = int(msg.tray_fruit_count)
        self.current_target_id = int(msg.current_target_id)
        if self.mission_state != self._prev_state:
            # PICK entry = a (dry) pick decision for the current target -> log full context.
            if self.mission_state == "PICK":
                self._log_event("pick", obj=self._lookup(self.current_target_id))
            else:
                self._log_event("state", obj=self._lookup(self.current_target_id))
            self._prev_state = self.mission_state

    def on_phase(self, msg: Int8) -> None:
        self.phase = int(msg.data)

    def on_decision(self, msg: String) -> None:
        self.decisions.append(f"[{time.time() - self._t0:5.0f}s] {msg.data}")

    def on_proj(self, msg: Float32MultiArray) -> None:
        d = list(msg.data)
        self.proj_dets = [(d[i], d[i + 1], int(d[i + 2])) for i in range(0, len(d) - 2, 3)]

    # ------------------------------------------------------------------ helpers
    def _lookup(self, obj_id: int) -> Object | None:
        if self.world is None or obj_id == 0:
            return None
        for obj in self.world.objects:
            if obj.id == obj_id:
                return obj
        return None

    def _log_event(self, event: str, obj: Object | None) -> None:
        rx = ry = rt = 0.0
        if self.world is not None:
            rx, ry, rt = self.world.robot_x, self.world.robot_y, self.world.robot_theta
        row = {c: "" for c in self._csv_cols}
        row["t_iso"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        row["elapsed_s"] = f"{time.time() - self._t0:.1f}"
        row["event"] = event
        row["phase"] = self.phase
        row["mission_state"] = self.mission_state
        row["robot_x"] = f"{rx:.3f}"
        row["robot_y"] = f"{ry:.3f}"
        row["robot_theta"] = f"{rt:.3f}"
        if obj is not None:
            row["obj_id"] = obj.id
            row["class_label"] = obj.class_label
            row["set_type"] = obj.set_type
            row["x"] = f"{obj.x:.3f}"
            row["y"] = f"{obj.y:.3f}"
            row["conf"] = f"{obj.confidence:.3f}"
            row["blacklisted"] = int(bool(obj.blacklisted))
        if self.shape is not None:
            row["shape_label"] = self.shape.label
            row["shape_conf"] = f"{self.shape.confidence:.3f}"
        if self.siglip is not None:
            row["siglip_label"] = self.siglip.label
            row["siglip_conf"] = f"{self.siglip.confidence:.3f}"
            row["image_face_visible"] = int(bool(self.siglip.image_face_visible))
        self._csv.writerow(row)
        self._csv_file.flush()
        self._jsonl_file.write(json.dumps(row) + "\n")
        self._jsonl_file.flush()

    def _w2p(self, x: float, y: float) -> tuple[int, int]:
        xmin, xmax, ymin, ymax = self._extent_now
        W = H = self.canvas_px
        px = int((x - xmin) / max(1e-6, xmax - xmin) * (W - 1))
        py = int((ymax - y) / max(1e-6, ymax - ymin) * (H - 1))   # y up -> invert row
        if self.map_rotate_180:   # flip both axes -> map faces the same way as the wide feed
            px, py = (W - 1) - px, (H - 1) - py
        return px, py

    def _current_extent(self):
        """Square extent covering all objects + robot + trail (+margin), or the fixed extent."""
        if not self.auto_extent or self.world is None or not self.world.objects:
            return list(self.extent)
        xs = [o.x for o in self.world.objects] + [self.world.robot_x] + [p[0] for p in self.trail]
        ys = [o.y for o in self.world.objects] + [self.world.robot_y] + [p[1] for p in self.trail]
        m = 0.6
        xmin, xmax, ymin, ymax = min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m
        span = max(xmax - xmin, ymax - ymin, 1.5)   # keep it square, min 1.5 m across
        cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
        return [cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2]

    # --------------------------------------------------------------------- render
    def tick(self) -> None:
        if not _CV2_AVAILABLE:
            return
        canvas = self._compose()
        if self._can_show:
            try:
                cv2.imshow("recognition_viz", canvas)
                cv2.waitKey(1)
            except Exception as exc:  # noqa: BLE001 - headless: fall back to files only
                self._can_show = False
                self.get_logger().warn(f"cv2 window unavailable ({exc}); PNG snapshots only")
        now = time.time()
        if self._mjpeg is not None:   # feed the browser stream (smooth, no flicker)
            ok, jpg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self._mjpeg.latest = jpg.tobytes()
        if self.write_live:
            # atomic write (temp -> rename) so a viewer never catches a half-written frame
            # (that mid-write read is what flashes black on every live.png reload).
            live = os.path.join(self.run_dir, "live.png")
            tmp = os.path.join(self.run_dir, ".live_tmp.png")   # .png ext so imwrite selects PNG
            if cv2.imwrite(tmp, canvas):
                os.replace(tmp, live)
        # Periodic map_<seq>.png snapshots — DISABLED when snapshot_interval<=0 (only live.png is
        # kept, so a test run doesn't accumulate hundreds of PNGs). live.png above is the live view.
        if self.snapshot_interval > 0 and now - self._last_snapshot >= self.snapshot_interval:
            self._last_snapshot = now
            cv2.imwrite(os.path.join(self.run_dir, f"map_{self._seq:04d}.png"), canvas)
            self._seq += 1
        # Training-data capture (raw frames + YOLO pre-labels) for reuse as training data.
        if self.save_training and now - self._last_training_save >= self.training_interval:
            self._last_training_save = now
            self._save_training_frame("wide", self.top_img, self.top_dets)
            self._save_training_frame("body", self.body_img, self.body_dets)

    def _save_training_frame(self, cam: str, img_msg, dets) -> None:
        """Save a raw frame + its YOLO detections as a pre-label (only if >=1 detection)."""
        if img_msg is None or self.bridge is None or not dets:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001
            return
        h, w = frame.shape[:2]
        if w <= 0 or h <= 0:
            return
        lines = []
        for det in dets:
            ci = self._train_cls_idx.get(det.label)
            if ci is None:
                continue
            cx, cy = det.x_center / w, det.y_center / h
            bw, bh = det.width / w, det.height / h
            # clamp to the [0,1] YOLO range (boxes can graze the frame edge)
            cx, cy = min(max(cx, 0.0), 1.0), min(max(cy, 0.0), 1.0)
            bw, bh = min(max(bw, 0.0), 1.0), min(max(bh, 0.0), 1.0)
            lines.append(f"{ci} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        if not lines:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{int((time.time() % 1) * 1000):03d}"
        name = f"{cam}_{stamp}"
        img_path = os.path.join(self._train_root, cam, "images", name + ".jpg")
        lbl_path = os.path.join(self._train_root, cam, "labels", name + ".txt")
        try:
            if cv2.imwrite(img_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                with open(lbl_path, "w") as f:
                    f.write("\n".join(lines) + "\n")
                self._train_saved += 1
        except OSError:
            pass

    def _compose(self):
        """Full panel: 2D map on the left, YOLO-annotated wide + body feeds stacked on the right."""
        map_img = self._render()
        if not self.show_cams:
            return map_img
        cam_h = self.canvas_px // 2
        cam_w = int(cam_h * 1640 / 1232)
        wide = self._render_cam(self.top_img, self.top_dets, "WIDE (top)", cam_w, cam_h)
        body = self._render_cam(self.body_img, self.body_dets, "BODY", cam_w, cam_h)
        col = np.vstack([wide, body])
        if col.shape[0] != map_img.shape[0]:
            col = cv2.resize(col, (col.shape[1], map_img.shape[0]))
        return np.hstack([map_img, col])

    def _render_cam(self, img_msg, dets, title: str, w: int, h: int):
        panel = np.full((h, w, 3), 30, np.uint8)
        frame = None
        if img_msg is not None and self.bridge is not None:
            try:
                frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            except Exception:  # noqa: BLE001
                frame = None
        if frame is None:
            cv2.putText(panel, f"{title}: no frame", (10, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, _COL_TEXT, 1, cv2.LINE_AA)
            return panel
        oh, ow = frame.shape[:2]
        out = cv2.resize(frame, (w, h))
        sx, sy = w / max(1, ow), h / max(1, oh)
        for det in dets:
            bw, bh = det.width * sx, det.height * sy
            cx, cy = det.x_center * sx, det.y_center * sy
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            col = _label_color(det.label)
            cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
            cv2.putText(out, f"{det.label} {det.confidence:.2f}", (x1, max(14, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.rectangle(out, (0, 0), (w, 18), (0, 0, 0), -1)
        cv2.putText(out, f"{title}  dets={len(dets)}", (6, 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, _COL_TEXT, 1, cv2.LINE_AA)
        return out

    def _render(self):
        W = H = self.canvas_px
        canvas = np.full((H, W, 3), 24, np.uint8)
        self._extent_now = self._current_extent()   # auto-fit (or fixed) for this frame
        self._draw_grid(canvas)
        self._draw_fov(canvas)                       # camera coverage under the objects
        self._draw_trail(canvas)
        if self.world is not None:
            for obj in self.world.objects:
                self._draw_object(canvas, obj)
            self._draw_robot(canvas, self.world.robot_x, self.world.robot_y, self.world.robot_theta)
        self._draw_proj(canvas)   # raw per-camera homography projections on top
        self._draw_hud(canvas)
        self._draw_decisions(canvas)
        self._draw_legend(canvas)
        return canvas

    def _draw_proj(self, canvas) -> None:
        """Raw per-camera homography projections of the CURRENT detections: wide = orange x,
        body = cyan +. A wide-x and body-+ sitting together on a fused track = the two cameras'
        homographies agree; separated marks = a projection/calibration mismatch to investigate."""
        for (x, y, src) in self.proj_dets:
            px, py = self._w2p(x, y)
            if src == 1:                                   # body -> cyan plus
                cv2.line(canvas, (px - 4, py), (px + 4, py), (255, 255, 0), 1)
                cv2.line(canvas, (px, py - 4), (px, py + 4), (255, 255, 0), 1)
            else:                                          # wide -> orange x
                cv2.line(canvas, (px - 4, py - 4), (px + 4, py + 4), (0, 165, 255), 1)
                cv2.line(canvas, (px - 4, py + 4), (px + 4, py - 4), (0, 165, 255), 1)

    def _draw_fov(self, canvas) -> None:
        """Overlay each camera's coverage on the map (base_link, follows the robot's heading):
        wide = forward ELLIPSE (orange), body = forward SECTOR / 부채꼴 (cyan). Points are computed
        in base_link then rotated into the field, so the shapes turn with the robot."""
        if self.world is None:
            return
        rx, ry, th = self.world.robot_x, self.world.robot_y, self.world.robot_theta
        ct, st = math.cos(th), math.sin(th)

        def b2p(bx, by):   # base_link -> field -> pixel
            return self._w2p(rx + bx * ct - by * st, ry + bx * st + by * ct)

        # wide FOV — forward-biased ellipse
        a, b, cx = self.wide_fov_fwd, self.wide_fov_lat, self.wide_fov_cx
        ell = np.array([b2p(cx + a * math.cos(t), b * math.sin(t))
                        for t in (i * math.pi / 24 for i in range(48))], np.int32)
        cv2.polylines(canvas, [ell], True, (0, 120, 200), 1)
        cv2.putText(canvas, "wide FOV", b2p(cx, b + 0.05), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (0, 130, 210), 1, cv2.LINE_AA)

        # body FOV — forward sector (annular: near..far arc, +-half angle)
        half, ax = self.body_fov_half, self.body_fov_apex_x
        near, far, M = self.body_fov_near, self.body_fov_far, 16
        arc = [b2p(ax + far * math.cos(-half + 2 * half * i / M), far * math.sin(-half + 2 * half * i / M))
               for i in range(M + 1)]
        arc += [b2p(ax + near * math.cos(half - 2 * half * i / M), near * math.sin(half - 2 * half * i / M))
                for i in range(M + 1)]
        cv2.polylines(canvas, [np.array(arc, np.int32)], True, (200, 200, 0), 1)
        cv2.putText(canvas, "body FOV", b2p(ax + far * 0.55, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    (200, 200, 0), 1, cv2.LINE_AA)

    def _draw_legend(self, canvas) -> None:
        """Top-right key: shape forms (Set1) + fruit colours (Set2)."""
        x0 = self.canvas_px - 148
        y = 16
        cv2.putText(canvas, "shapes (form):", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    _COL_TEXT, 1, cv2.LINE_AA)
        y += 18
        for name, (n, rot) in _SHAPE_FORM.items():
            cv2.fillPoly(canvas, [_reg_poly(x0 + 8, y - 4, 6, n, rot)], _COL_SHAPE)
            cv2.putText(canvas, name, (x0 + 22, y), cv2.FONT_HERSHEY_SIMPLEX, 0.36, _COL_TEXT, 1,
                        cv2.LINE_AA)
            y += 16
        y += 6
        cv2.putText(canvas, "fruits (color):", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    _COL_TEXT, 1, cv2.LINE_AA)
        y += 18
        for name, col in _FRUIT_COLORS.items():
            cv2.circle(canvas, (x0 + 8, y - 4), 6, col, -1)
            cv2.putText(canvas, name, (x0 + 22, y), cv2.FONT_HERSHEY_SIMPLEX, 0.36, _COL_TEXT, 1,
                        cv2.LINE_AA)
            y += 16
        cv2.circle(canvas, (x0 + 8, y - 4), 6, _COL_FRUITCUBE, -1)
        cv2.putText(canvas, "fruit_cube?", (x0 + 22, y), cv2.FONT_HERSHEY_SIMPLEX, 0.36, _COL_TEXT,
                    1, cv2.LINE_AA)
        cv2.circle(canvas, (x0 + 8, y + 12), 6, (120, 120, 120), 1)
        cv2.circle(canvas, (x0 + 8, y + 12), 9, (255, 255, 255), 1)
        cv2.putText(canvas, "()=locked", (x0 + 22, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    _COL_TEXT, 1, cv2.LINE_AA)
        # fusion source ring + presence/identity key
        y3 = y + 34
        for tag, cc, desc in (("WB", (0, 255, 0), "both"), ("B", (255, 255, 0), "body/id"),
                              ("W", (0, 165, 255), "wide/pos")):
            cv2.circle(canvas, (x0 + 8, y3 - 4), 7, cc, 1)
            cv2.putText(canvas, f"{tag}={desc}", (x0 + 22, y3), cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                        _COL_TEXT, 1, cv2.LINE_AA)
            y3 += 15
        cv2.putText(canvas, "solid=confirmed", (x0, y3 + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.33,
                    _COL_TEXT, 1, cv2.LINE_AA)
        cv2.putText(canvas, "hollow=prov  grey?=id unknown", (x0, y3 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, _COL_TEXT, 1, cv2.LINE_AA)
        cv2.putText(canvas, "x=wide proj  +=body proj", (x0, y3 + 29),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, _COL_TEXT, 1, cv2.LINE_AA)

    def _draw_grid(self, canvas) -> None:
        xmin, xmax, ymin, ymax = self._extent_now
        gx = int(math.floor(xmin))
        while gx <= xmax:
            p0 = self._w2p(gx, ymin)
            p1 = self._w2p(gx, ymax)
            cv2.line(canvas, p0, p1, _COL_GRID, 1)
            gx += 1
        gy = int(math.floor(ymin))
        while gy <= ymax:
            p0 = self._w2p(xmin, gy)
            p1 = self._w2p(xmax, gy)
            cv2.line(canvas, p0, p1, _COL_GRID, 1)
            gy += 1
        # origin axes
        ox, oy = self._w2p(0.0, 0.0)
        cv2.line(canvas, (ox, 0), (ox, self.canvas_px - 1), (90, 90, 90), 1)
        cv2.line(canvas, (0, oy), (self.canvas_px - 1, oy), (90, 90, 90), 1)

    def _draw_trail(self, canvas) -> None:
        pts = [self._w2p(x, y) for (x, y) in self.trail]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], _COL_TRAIL, 1)

    def _draw_object(self, canvas, obj: Object) -> None:
        px, py = self._w2p(obj.x, obj.y)
        label = obj.class_label or ""
        r = 10
        # --- fusion source, identity trust, presence tier ---
        n_obs = getattr(obj, "n_obs", 0)
        src = getattr(obj, "source", "") or ""
        seen_body, seen_wide = "body" in src, "wide" in src
        both = seen_body and seen_wide
        # IDENTITY is trusted only from the body cam (close/reliable). The wide cam is good at
        # POSITION but weak at CLASS, so a wide-only object is "something here, identity unknown":
        # drawn grey with the wide guess only hinted in text — never a committed class shape.
        identity_known = seen_body or bool(obj.blacklisted)
        # PRESENCE tier: CONFIRMED (solid) if both matched / body saw it / wide saw it enough times
        # to rule out a flicker; else PROVISIONAL (hollow).
        confirmed = bool(obj.blacklisted) or both or seen_body or n_obs >= self.confirm_min_obs
        if both:
            stag, sring = "WB", (0, 255, 0)        # matched by BOTH cameras (green)
        elif seen_body:
            stag, sring = "B", (255, 255, 0)       # body only (cyan)
        else:
            stag, sring = "W", (0, 165, 255)       # wide only (orange)
        is_shape = identity_known and label in _SHAPE_FORM
        # colour
        if obj.blacklisted:
            col = _COL_BLACKLIST
        elif not identity_known:
            col = _COL_UNKNOWN                      # wide-only -> identity unknown (grey)
        elif is_shape:
            col = _COL_SHAPE
        elif label in _FRUIT_COLORS:
            col = _FRUIT_COLORS[label]
        elif label == "fruit_photo_cube" or obj.set_type == 2:
            col = _COL_FRUITCUBE
        else:
            col = _COL_UNKNOWN
        # marker: identified shape -> form polygon; else circle. CONFIRMED filled, PROVISIONAL hollow.
        if is_shape:
            n, rot = _SHAPE_FORM[label]
            pts = _reg_poly(px, py, r + 1, n, rot)
            if confirmed:
                cv2.fillPoly(canvas, [pts], col)
                cv2.polylines(canvas, [pts], True, (20, 20, 20), 1)
            else:
                cv2.polylines(canvas, [pts], True, col, 1)
        else:
            cv2.circle(canvas, (px, py), r, col, -1 if confirmed else 1)
            if confirmed:
                cv2.circle(canvas, (px, py), r, (20, 20, 20), 1)
        # source ring (which cameras saw it) just outside the marker
        cv2.circle(canvas, (px, py), r + 2, sring, 1)
        # locked landmark -> white ring overlay (doesn't change the class marker)
        if bool(getattr(obj, "locked", False)):
            cv2.circle(canvas, (px, py), r + 5, (255, 255, 255), 1)
        if obj.id == self.current_target_id and self.current_target_id != 0:
            cv2.circle(canvas, (px, py), r + 7, _COL_TARGET, 2)
        elif obj.id == self.selected_id and self.selected_id != 0:
            cv2.circle(canvas, (px, py), r + 6, (0, 200, 200), 1)
        # text: identified -> class name; wide-only -> "?(wideguess)" (position sure, class tentative)
        pres = "" if confirmed else "?"
        name = (label or "?") if identity_known else (f"?({label})" if label else "?")
        txt = f"#{obj.id}{pres} {name} {obj.confidence:.2f} x{n_obs} [{stag}]"
        cv2.putText(canvas, txt, (px + 13, py + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, _COL_TEXT, 1,
                    cv2.LINE_AA)

    def _draw_robot(self, canvas, x: float, y: float, theta: float) -> None:
        px, py = self._w2p(x, y)
        # heading arrow (0.3 m long) in field frame
        hx, hy = self._w2p(x + 0.3 * math.cos(theta), y + 0.3 * math.sin(theta))
        cv2.circle(canvas, (px, py), 7, _COL_ROBOT, -1)
        cv2.arrowedLine(canvas, (px, py), (hx, hy), _COL_ROBOT, 2, tipLength=0.35)

    def _draw_hud(self, canvas) -> None:
        n_obj = len(self.world.objects) if self.world is not None else 0
        n_bl = sum(1 for o in self.world.objects if o.blacklisted) if self.world is not None else 0
        lines = [
            f"state={self.mission_state}  phase={self.phase}  t={time.time() - self._t0:5.0f}s",
            f"objects={n_obj} (picked/bl={n_bl})  tray shape={self.tray_shape} fruit={self.tray_fruit}",
            f"det wide={len(self.top_dets)} body={len(self.body_dets)}",
        ]
        if self.world is not None:
            objs = self.world.objects
            wb = sum(1 for o in objs if "body" in (o.source or "") and "wide" in (o.source or ""))
            bo = sum(1 for o in objs if (o.source or "") == "body")
            wo = sum(1 for o in objs if (o.source or "") == "wide")
            # body CONFIRMS wb+bo tracks (its identity authority); "body-only" alone reads as 0 since
            # the wide cam now sees almost everything too -> show the real contribution.
            lines.append(f"tracks: WB={wb}  wide-only={wo}  |  body confirms {wb + bo} (identity)")
        if self.shape is not None:
            lines.append(f"shape: {self.shape.label} {self.shape.confidence:.2f}")
        if self.siglip is not None:
            lines.append(
                f"siglip: {self.siglip.label} {self.siglip.confidence:.2f} "
                f"face={int(bool(self.siglip.image_face_visible))}"
            )
        y = 18
        for ln in lines:
            cv2.putText(canvas, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _COL_TEXT, 1,
                        cv2.LINE_AA)
            y += 18

    def _draw_decisions(self, canvas) -> None:
        """Bottom overlay: the FSM's recent PICK / PASS / SKIP / PHASE decisions."""
        if not self.decisions:
            return
        lines = list(self.decisions)
        row = 15
        h = row * (len(lines) + 1) + 6
        y0 = self.canvas_px - h
        cv2.rectangle(canvas, (0, y0), (self.canvas_px, self.canvas_px), (0, 0, 0), -1)
        cv2.putText(canvas, "decisions (FSM):", (6, y0 + row), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (0, 220, 220), 1, cv2.LINE_AA)
        y = y0 + row * 2
        for ln in lines:
            cv2.putText(canvas, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, _COL_TEXT, 1,
                        cv2.LINE_AA)
            y += row

    # --------------------------------------------------------------------- teardown
    def write_summary(self) -> None:
        try:
            # map_final.png only when snapshots are enabled — otherwise only live.png is kept.
            if _CV2_AVAILABLE and self.snapshot_interval > 0:
                canvas = self._compose()
                cv2.imwrite(os.path.join(self.run_dir, "map_final.png"), canvas)
            lines = [
                f"mock field test summary  ({time.strftime('%Y-%m-%d %H:%M:%S')})",
                f"duration: {time.time() - self._t0:.0f}s",
                f"final state: {self.mission_state}  phase: {self.phase}",
                f"tray: shape={self.tray_shape} fruit={self.tray_fruit}",
                f"objects tracked: {len(self._seen_ids)}",
            ]
            if self.save_training:
                lines.append(f"training frames saved: {self._train_saved} -> {self._train_root}")
            if self.world is not None:
                lines.append("final tracks:")
                for o in self.world.objects:
                    lines.append(
                        f"  #{o.id} set{o.set_type} '{o.class_label}' "
                        f"({o.x:.2f},{o.y:.2f}) conf={o.confidence:.2f} "
                        f"{'BL' if o.blacklisted else ''}"
                    )
            with open(os.path.join(self.run_dir, "summary.txt"), "w") as f:
                f.write("\n".join(lines) + "\n")
            self.get_logger().info("wrote map_final.png + summary.txt")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"summary write failed: {exc}")
        finally:
            try:
                self._csv_file.close()
                self._jsonl_file.close()
            except Exception:  # noqa: BLE001
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecognitionVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
