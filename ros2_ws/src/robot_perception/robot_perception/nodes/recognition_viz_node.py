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
  /localization/wall_mask_segments_image Float32MultiArray  — authoritative mask-median wall lines
  /localization/wall_segmentation_mask Image                 — learned wall/floor mask overlay for debugging
  /classification/siglip    robot_interfaces/Classification — body fruit type (logged with picks)
  /classification/shape     robot_interfaces/Classification — body shape class (logged with picks)
  /selected_target          robot_interfaces/Object         — highlighted target
  /mission_state            robot_interfaces/MissionState   — state/tray counts/current target
  /planning/phase           std_msgs/Int8                   — Set1/Set2 pick phase (HUD)
  /planning/zone            std_msgs/Int8                   — active 2x2m mission zone (HUD/map)
  /planning/object_slots    std_msgs/String                 — persistent Set2 slot inventory (JSON)

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
from rcl_interfaces.msg import SetParametersResult
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
_COL_FLAG = (60, 210, 255)     # finish/storage flag -> yellow-ish
_COL_UNKNOWN = (170, 170, 170)  # unknown -> gray
_COL_BLACKLIST = (90, 90, 90)  # picked/passed -> dark gray
_COL_TARGET = (0, 255, 255)    # current target ring -> yellow
_COL_ROBOT = (255, 90, 40)     # robot -> blue
_COL_TRAIL = (200, 130, 60)    # trajectory
_COL_GRID = (60, 60, 60)
_COL_OBJECT_GRID = (115, 115, 115)
_COL_ROUTE = (245, 245, 245)
_COL_ROUTE_HEADING = (0, 230, 255)
_COL_TEXT = (235, 235, 235)
_COL_SLOT = (205, 90, 200)
_COL_SLOT_RETRY = (0, 165, 255)
_COL_SLOT_DONE = (100, 100, 100)
_ZONE_COLORS = {
    1: (70, 120, 220),
    2: (60, 170, 120),
    3: (180, 130, 70),
    4: (170, 90, 170),
}

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
    if label == "arrival":
        return _COL_FLAG
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
        self.declare_parameter("map_margin_px", 40)
        self.declare_parameter("snapshot_interval_sec", 5.0)
        self.declare_parameter("show_window", False)
        self.declare_parameter("trail_max_points", 2000)
        self.declare_parameter("trail_min_step_m", 0.01)
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
        self.declare_parameter("simple_object_labels", False)
        self.declare_parameter("show_set2_slots", True)
        # Camera coverage drawn on the map (base_link, follows the robot). Body = forward SECTOR
        # (부채꼴), wide = forward-biased ELLIPSE. Keep in sync with the same-named world_model params.
        self.declare_parameter("body_fov_half_deg", 34.0)
        self.declare_parameter("body_fov_near_m", 0.08)
        self.declare_parameter("body_fov_far_m", 0.6)
        self.declare_parameter("body_fov_apex_x", 0.065)
        self.declare_parameter("wide_fov_forward_m", 1.6)
        self.declare_parameter("wide_fov_lateral_m", 1.3)
        self.declare_parameter("wide_fov_center_x", 0.3)
        # Optional 7x6 arena object-grid overlay. This is a visual aid for the world_model
        # grid prior, not a synthetic object layer.
        self.declare_parameter("show_object_grid_points", True)
        self.declare_parameter("grid_rows", 6)
        self.declare_parameter("grid_cols", 7)
        self.declare_parameter("grid_spacing_m", 0.50)
        self.declare_parameter("grid_origin_x_m", 0.50)
        self.declare_parameter("grid_origin_y_m", 0.50)
        # Visualise the grid-aligned mission zones used by mission_fsm_node.
        # The center x=0 grid column is slightly overlapped; the y split is between grid rows.
        self.declare_parameter("show_zone_regions", True)
        self.declare_parameter(
            "zone_bounds_m",
            [-2.0, 0.1, -0.25, 2.0, -2.0, 0.1, -2.0, -0.25,
             -0.1, 2.0, -2.0, -0.25, -0.1, 2.0, -0.25, 2.0],
        )

        ext = [float(v) for v in self.get_parameter("field_extent_m").value]
        self.extent = ext if len(ext) == 4 else [-2.0, 2.0, -2.0, 2.0]
        self.auto_extent = bool(self.get_parameter("auto_extent").value)
        self._extent_now = list(self.extent)   # extent used for the current frame (auto or fixed)
        self.canvas_px = int(self.get_parameter("canvas_px").value)
        self.map_margin_px = int(self.get_parameter("map_margin_px").value)
        self.map_rotate_180 = bool(self.get_parameter("map_rotate_180").value)
        self.snapshot_interval = float(self.get_parameter("snapshot_interval_sec").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.redraw_rate = float(self.get_parameter("redraw_rate_hz").value)
        trail_max = int(self.get_parameter("trail_max_points").value)
        self.trail_min_step = float(self.get_parameter("trail_min_step_m").value)
        self.show_cams = bool(self.get_parameter("show_camera_panels").value) and _BRIDGE_AVAILABLE
        self.write_live = bool(self.get_parameter("write_live_png").value)
        self.save_training = bool(self.get_parameter("save_training_data").value) and _BRIDGE_AVAILABLE
        self.training_interval = float(self.get_parameter("training_interval_sec").value)
        self._last_training_save = 0.0
        # YOLO class order shared by wide.pt / cube.pt.
        self._train_classes = [
            "cube", "octahedron", "dodecahedron", "icosahedron", "fruit_photo_cube", "arrival"
        ]
        self._train_cls_idx = {c: i for i, c in enumerate(self._train_classes)}
        self.confirm_min_obs = int(self.get_parameter("confirm_min_obs").value)
        self.simple_object_labels = bool(self.get_parameter("simple_object_labels").value)
        self.show_set2_slots = bool(self.get_parameter("show_set2_slots").value)
        self.body_fov_half = math.radians(float(self.get_parameter("body_fov_half_deg").value))
        self.body_fov_near = float(self.get_parameter("body_fov_near_m").value)
        self.body_fov_far = float(self.get_parameter("body_fov_far_m").value)
        self.body_fov_apex_x = float(self.get_parameter("body_fov_apex_x").value)
        self.wide_fov_fwd = float(self.get_parameter("wide_fov_forward_m").value)
        self.wide_fov_lat = float(self.get_parameter("wide_fov_lateral_m").value)
        self.wide_fov_cx = float(self.get_parameter("wide_fov_center_x").value)
        self.show_object_grid_points = bool(self.get_parameter("show_object_grid_points").value)
        self.grid_rows = max(0, int(self.get_parameter("grid_rows").value))
        self.grid_cols = max(0, int(self.get_parameter("grid_cols").value))
        self.grid_spacing = float(self.get_parameter("grid_spacing_m").value)
        self.grid_origin_x = float(self.get_parameter("grid_origin_x_m").value)
        self.grid_origin_y = float(self.get_parameter("grid_origin_y_m").value)
        self.show_zone_regions = bool(self.get_parameter("show_zone_regions").value)
        zb = [float(v) for v in self.get_parameter("zone_bounds_m").value]
        self.zone_bounds: dict[int, tuple[float, float, float, float]] = {}
        for i in range(0, min(len(zb), 16), 4):
            zid = i // 4 + 1
            self.zone_bounds[zid] = (zb[i], zb[i + 1], zb[i + 2], zb[i + 3])
        self.declare_parameter("show_zone_anchors", True)
        self.declare_parameter(
            "zone_anchor_xy",
            [-0.75, 0.75, -0.75, -1.0, 0.75, -1.0, 0.75, 0.75],
        )
        self.show_zone_anchors = bool(self.get_parameter("show_zone_anchors").value)
        za = [float(v) for v in self.get_parameter("zone_anchor_xy").value]
        self.zone_anchors: dict[int, tuple[float, float]] = {}
        for i in range(0, min(len(za), 8), 2):
            self.zone_anchors[i // 2 + 1] = (za[i], za[i + 1])
        self.declare_parameter("show_checkpoint_route", True)
        self.declare_parameter(
            "checkpoint_route_xy",
            [-1.8, 1.6, -1.8, -1.6, -0.75, -1.6, -0.75, 1.6,
             0.25, 1.6, 0.25, -1.6, 1.25, -1.6, 1.25, 1.6],
        )
        self.declare_parameter(
            "checkpoint_route_heading_rad",
            [-math.pi / 2.0, 0.0, math.pi / 2.0, 0.0,
             -math.pi / 2.0, 0.0, math.pi / 2.0, 0.0],
        )
        self.show_checkpoint_route = bool(self.get_parameter("show_checkpoint_route").value)
        rxy = [float(v) for v in self.get_parameter("checkpoint_route_xy").value]
        self.checkpoint_route: list[tuple[float, float]] = [
            (rxy[i], rxy[i + 1]) for i in range(0, len(rxy) - 1, 2)
        ]
        self.checkpoint_headings = [
            float(v) for v in self.get_parameter("checkpoint_route_heading_rad").value
        ]
        self.add_on_set_parameters_callback(self._on_parameters_changed)
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
        self.zone = 0
        self.object_slots: list[dict] = []
        self.current_slot_id = 0
        self.siglip: Classification | None = None
        self.shape: Classification | None = None
        # Latest raw camera image msgs + their detections (converted at draw time, ~redraw rate).
        self.top_img: Image | None = None
        self.body_img: Image | None = None
        self.top_dets: list = []
        self.body_dets: list = []
        self.proj_dets: list = []   # per-camera raw projections (x, y, src) from /world_model/projected_dets
        self.wall_segments: list[tuple[float, float, float, float]] = []
        self.wall_segments_time = 0.0
        self.wall_raw_segments = []
        self.wall_raw_segments_time = 0.0
        self._wall_tf_tx = 0.0
        self._wall_tf_ty = 0.0
        self._wall_tf_th = 0.0
        self.wall_mask_segments: list[tuple[float, float, float, float]] = []
        self.wall_mask_segments_time = 0.0
        self.wall_mask_img: Image | None = None
        self.wall_mask_time = 0.0

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
        self.create_subscription(
            Float32MultiArray, "/localization/wall_map_transform", self.on_wall_map_transform, 10
        )
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
        self.create_subscription(Int8, "/planning/zone", self.on_zone, 10)
        self.create_subscription(String, "/planning/decision", self.on_decision, 10)
        self.create_subscription(String, "/planning/object_slots", self.on_object_slots, 10)
        self.create_subscription(Float32MultiArray, "/world_model/projected_dets", self.on_proj, 10)
        self.create_subscription(Float32MultiArray, "/localization/wall_segments",
                                 self.on_wall_segments, 10)
        self.create_subscription(Float32MultiArray, "/localization/wall_raw_segments",
                                 self.on_wall_raw_segments, 10)
        self.create_subscription(Float32MultiArray, "/localization/wall_mask_segments_image",
                                 self.on_wall_mask_segments, 10)
        self.create_subscription(Image, "/localization/wall_segmentation_mask",
                                 self.on_wall_mask, qos_profile_sensor_data)

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
        point = (float(msg.pose.position.x), float(msg.pose.position.y))
        if not self.trail or math.hypot(point[0] - self.trail[-1][0], point[1] - self.trail[-1][1]) >= self.trail_min_step:
            self.trail.append(point)

    def on_wall_map_transform(self, msg: Float32MultiArray) -> None:
        """Keep debug overlays and the trail rigid with a global wall alignment."""
        if len(msg.data) < 3:
            return
        tx, ty, dth = (float(msg.data[i]) for i in range(3))
        ct, st = math.cos(dth), math.sin(dth)

        def transform(x: float, y: float) -> tuple[float, float]:
            return ct * x - st * y + tx, st * x + ct * y + ty

        old_tx, old_ty, old_th = self._wall_tf_tx, self._wall_tf_ty, self._wall_tf_th
        self._wall_tf_tx = ct * old_tx - st * old_ty + tx
        self._wall_tf_ty = st * old_tx + ct * old_ty + ty
        self._wall_tf_th = math.atan2(math.sin(old_th + dth), math.cos(old_th + dth))

        self.trail = deque((transform(x, y) for x, y in self.trail), maxlen=self.trail.maxlen)
        self.wall_raw_segments = [
            (*transform(x0, y0), *transform(x1, y1))
            for x0, y0, x1, y1 in self.wall_raw_segments
        ]
        self.proj_dets = [(*transform(x, y), src) for x, y, src in self.proj_dets]

    def on_top_det(self, msg: DetectionArray) -> None:
        self.top_dets = list(msg.detections)

    def on_body_det(self, msg: DetectionArray) -> None:
        self.body_dets = list(msg.detections)

    def on_wall_segments(self, msg: Float32MultiArray) -> None:
        vals = [float(v) for v in msg.data]
        segs = []
        for i in range(0, len(vals) - 3, 4):
            segs.append((vals[i], vals[i + 1], vals[i + 2], vals[i + 3]))
        self.wall_segments = segs
        self.wall_segments_time = time.time()

    def on_wall_raw_segments(self, msg: Float32MultiArray) -> None:
        vals = [float(v) for v in msg.data]
        segs = []
        for i in range(0, len(vals) - 3, 4):
            x0, y0 = self._apply_wall_map_transform(vals[i], vals[i + 1])
            x1, y1 = self._apply_wall_map_transform(vals[i + 2], vals[i + 3])
            segs.append((x0, y0, x1, y1))
        self.wall_raw_segments = segs
        self.wall_raw_segments_time = time.time()

    def on_wall_mask_segments(self, msg: Float32MultiArray) -> None:
        vals = [float(v) for v in msg.data]
        segs = []
        for i in range(0, len(vals) - 3, 4):
            segs.append((vals[i], vals[i + 1], vals[i + 2], vals[i + 3]))
        self.wall_mask_segments = segs
        self.wall_mask_segments_time = time.time()

    def on_wall_mask(self, msg: Image) -> None:
        self.wall_mask_img = msg
        self.wall_mask_time = time.time()

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

    def on_zone(self, msg: Int8) -> None:
        self.zone = int(msg.data)

    def on_decision(self, msg: String) -> None:
        self.decisions.append(f"[{time.time() - self._t0:5.0f}s] {msg.data}")

    def on_object_slots(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            slots = payload.get("slots", [])
            if not isinstance(slots, list):
                return
            self.object_slots = [slot for slot in slots if isinstance(slot, dict)]
            self.current_slot_id = int(payload.get("current_slot_id", 0))
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def on_proj(self, msg: Float32MultiArray) -> None:
        d = list(msg.data)
        self.proj_dets = [
            (*self._apply_wall_map_transform(d[i], d[i + 1]), int(d[i + 2]))
            for i in range(0, len(d) - 2, 3)
        ]

    # ------------------------------------------------------------------ helpers
    def _on_parameters_changed(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "simple_object_labels":
                self.simple_object_labels = bool(param.value)
            elif param.name == "show_set2_slots":
                self.show_set2_slots = bool(param.value)
        return SetParametersResult(successful=True)

    def _apply_wall_map_transform(self, x: float, y: float) -> tuple[float, float]:
        ct, st = math.cos(self._wall_tf_th), math.sin(self._wall_tf_th)
        return ct * x - st * y + self._wall_tf_tx, st * x + ct * y + self._wall_tf_ty

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
        margin = min(max(0, self.map_margin_px), max(0, (min(W, H) - 2) // 2))
        inner_w = max(1, W - 1 - 2 * margin)
        inner_h = max(1, H - 1 - 2 * margin)
        px = int(margin + (x - xmin) / max(1e-6, xmax - xmin) * inner_w)
        py = int(margin + (ymax - y) / max(1e-6, ymax - ymin) * inner_h)   # y up -> invert row
        if self.map_rotate_180:   # flip both axes -> map faces the same way as the wide feed
            px, py = (W - 1) - px, (H - 1) - py
        return px, py

    def _current_extent(self):
        """Square extent covering all objects + robot + trail (+margin), or the fixed extent."""
        if not self.auto_extent:
            return list(self.extent)
        xs = [float(slot.get("x", 0.0)) for slot in self.object_slots]
        ys = [float(slot.get("y", 0.0)) for slot in self.object_slots]
        if self.world is not None:
            xs += [o.x for o in self.world.objects] + [self.world.robot_x]
            ys += [o.y for o in self.world.objects] + [self.world.robot_y]
        xs += [p[0] for p in self.trail]
        ys += [p[1] for p in self.trail]
        if not xs or not ys:
            return list(self.extent)
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
        if title.startswith("WIDE"):
            self._draw_wall_mask(out)
            self._draw_wall_mask_segments(out, sx, sy)
        cv2.rectangle(out, (0, 0), (w, 18), (0, 0, 0), -1)
        cv2.putText(out, f"{title}  dets={len(dets)}", (6, 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, _COL_TEXT, 1, cv2.LINE_AA)
        return out

    def _draw_wall_mask(self, frame) -> None:
        """Overlay raw learned wall/floor-boundary mask on the wide camera panel."""
        if (
            self.wall_mask_img is None
            or self.bridge is None
            or time.time() - self.wall_mask_time > 2.0
        ):
            return
        try:
            mask = self.bridge.imgmsg_to_cv2(self.wall_mask_img, desired_encoding="mono8")
        except Exception:  # noqa: BLE001
            return
        h, w = frame.shape[:2]
        if mask.shape[:2] != (h, w):
            mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
        active = mask > 0
        if not np.any(active):
            return
        overlay = frame.copy()
        overlay[active] = (0, 0, 255)
        cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, dst=frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cv2.drawContours(frame, contours, -1, (0, 0, 255), 1, cv2.LINE_AA)

    def _draw_wall_mask_segments(self, frame, sx: float, sy: float) -> None:
        """Overlay authoritative mask-median wall lines in yellow."""
        if not self.wall_mask_segments or time.time() - self.wall_mask_segments_time > 1.5:
            return
        h, w = frame.shape[:2]
        for x0, y0, x1, y1 in self.wall_mask_segments:
            p0 = (int(round(x0 * sx)), int(round(y0 * sy)))
            p1 = (int(round(x1 * sx)), int(round(y1 * sy)))
            if (
                max(p0[0], p1[0]) < 0 or min(p0[0], p1[0]) >= w
                or max(p0[1], p1[1]) < 0 or min(p0[1], p1[1]) >= h
            ):
                continue
            cv2.line(frame, p0, p1, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(frame, p0, 3, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, p1, 3, (0, 255, 255), -1, cv2.LINE_AA)

    def _render(self):
        W = H = self.canvas_px
        canvas = np.full((H, W, 3), 24, np.uint8)
        self._extent_now = self._current_extent()   # auto-fit (or fixed) for this frame
        self._draw_grid(canvas)
        self._draw_zone_regions(canvas)
        self._draw_object_grid_points(canvas)
        self._draw_zone_anchors(canvas)
        self._draw_checkpoint_route(canvas)
        self._draw_wall_raw_segments(canvas)
        self._draw_wall_segments(canvas)
        self._draw_fov(canvas)                       # camera coverage under the objects
        self._draw_trail(canvas)
        if self.world is not None:
            for obj in self.world.objects:
                self._draw_object(canvas, obj)
            self._draw_slots(canvas)
            self._draw_robot(canvas, self.world.robot_x, self.world.robot_y, self.world.robot_theta)
        else:
            self._draw_slots(canvas)
        self._draw_proj(canvas)   # raw per-camera homography projections on top
        self._draw_hud(canvas)
        self._draw_decisions(canvas)
        self._draw_legend(canvas)
        return canvas

    def _draw_wall_raw_segments(self, canvas) -> None:
        """Draw unsnapped wall projections in orange for calibration debugging."""
        if not self.wall_raw_segments or time.time() - self.wall_raw_segments_time > 1.5:
            return
        for x0, y0, x1, y1 in self.wall_raw_segments:
            p0 = self._w2p(x0, y0)
            p1 = self._w2p(x1, y1)
            cv2.line(canvas, p0, p1, (0, 165, 255), 2, cv2.LINE_AA)
            cv2.circle(canvas, p0, 3, (0, 165, 255), -1, cv2.LINE_AA)
            cv2.circle(canvas, p1, 3, (0, 165, 255), -1, cv2.LINE_AA)

    def _draw_wall_segments(self, canvas) -> None:
        """Mask-authoritative wall observations projected into field coordinates."""
        if not self.wall_segments or time.time() - self.wall_segments_time > 1.5:
            return
        for x0, y0, x1, y1 in self.wall_segments:
            p0 = self._w2p(x0, y0)
            p1 = self._w2p(x1, y1)
            cv2.line(canvas, p0, p1, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.circle(canvas, p0, 4, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(canvas, p1, 4, (0, 255, 255), -1, cv2.LINE_AA)

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
        corners = [self._w2p(xmin, ymin), self._w2p(xmin, ymax),
                   self._w2p(xmax, ymax), self._w2p(xmax, ymin)]
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)

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
        cv2.line(canvas, (ox, y0), (ox, y1), (90, 90, 90), 1)
        cv2.line(canvas, (x0, oy), (x1, oy), (90, 90, 90), 1)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (150, 150, 150), 2)

    def _draw_zone_regions(self, canvas) -> None:
        if not self.show_zone_regions or not self.zone_bounds:
            return
        overlay = canvas.copy()
        rects = []
        for zid, (xmin, xmax, ymin, ymax) in sorted(self.zone_bounds.items()):
            pts = [self._w2p(xmin, ymin), self._w2p(xmin, ymax),
                   self._w2p(xmax, ymax), self._w2p(xmax, ymin)]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            x0, x1 = min(xs), max(xs)
            y0, y1 = min(ys), max(ys)
            col = _ZONE_COLORS.get(zid, (120, 120, 120))
            cv2.rectangle(overlay, (x0, y0), (x1, y1), col, -1)
            rects.append((zid, x0, y0, x1, y1, col))
        cv2.addWeighted(overlay, 0.16, canvas, 0.84, 0.0, dst=canvas)
        for zid, x0, y0, x1, y1, col in rects:
            active = self.zone == zid
            thickness = 3 if active else 1
            label_col = (255, 255, 255) if active else (190, 190, 190)
            cv2.rectangle(canvas, (x0, y0), (x1, y1), col, thickness, cv2.LINE_AA)
            cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
            cv2.putText(canvas, f"Z{zid}", (cx - 14, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, label_col, 2, cv2.LINE_AA)

    def _draw_object_grid_points(self, canvas) -> None:
        if not self.show_object_grid_points or self.grid_rows <= 0 or self.grid_cols <= 0:
            return
        xmin, xmax, ymin, ymax = self._extent_now
        r = 3
        for row in range(self.grid_rows):
            gy = self.grid_origin_y + row * self.grid_spacing
            if gy < ymin or gy > ymax:
                continue
            for col in range(self.grid_cols):
                gx = self.grid_origin_x + col * self.grid_spacing
                if gx < xmin or gx > xmax:
                    continue
                px, py = self._w2p(gx, gy)
                cv2.circle(canvas, (px, py), r + 1, (20, 20, 20), 1, cv2.LINE_AA)
                cv2.circle(canvas, (px, py), r, _COL_OBJECT_GRID, -1, cv2.LINE_AA)

    def _draw_zone_anchors(self, canvas) -> None:
        if not self.show_zone_anchors or not self.zone_anchors:
            return
        for zid, (ax, ay) in sorted(self.zone_anchors.items()):
            px, py = self._w2p(ax, ay)
            col = _ZONE_COLORS.get(zid, (230, 230, 230))
            cv2.drawMarker(canvas, (px, py), col, cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
            cv2.circle(canvas, (px, py), 9, col, 1, cv2.LINE_AA)
            cv2.putText(canvas, f"A{zid}", (px + 10, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (245, 245, 245), 1, cv2.LINE_AA)

    def _draw_checkpoint_route(self, canvas) -> None:
        if not self.show_checkpoint_route or not self.checkpoint_route:
            return
        pts = [self._w2p(x, y) for (x, y) in self.checkpoint_route]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], _COL_ROUTE, 2, cv2.LINE_AA)
        for i, ((x, y), (px, py)) in enumerate(zip(self.checkpoint_route, pts), start=1):
            cv2.circle(canvas, (px, py), 10, _COL_ROUTE, 2, cv2.LINE_AA)
            cv2.circle(canvas, (px, py), 3, _COL_ROUTE_HEADING, -1, cv2.LINE_AA)
            if i - 1 < len(self.checkpoint_headings):
                th = self.checkpoint_headings[i - 1]
                hx, hy = self._w2p(x + 0.28 * math.cos(th), y + 0.28 * math.sin(th))
                cv2.arrowedLine(canvas, (px, py), (hx, hy), _COL_ROUTE_HEADING, 2,
                                cv2.LINE_AA, tipLength=0.35)
            cv2.putText(canvas, f"P{i}", (px + 11, py - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, _COL_ROUTE, 1, cv2.LINE_AA)

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
        elif label == "arrival" or obj.set_type == 3:
            col = _COL_FLAG
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
        txt = name if self.simple_object_labels else f"#{obj.id}{pres} {name} {obj.confidence:.2f} x{n_obs} [{stag}]"
        cv2.putText(canvas, txt, (px + 13, py + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, _COL_TEXT, 1,
                    cv2.LINE_AA)

    def _draw_slots(self, canvas) -> None:
        """Draw persistent Set2 inspection IDs as compact rings around their fixed positions."""
        if not self.show_set2_slots:
            return
        for slot in self.object_slots:
            try:
                slot_id = int(slot.get("id", 0))
                px, py = self._w2p(float(slot["x"]), float(slot["y"]))
            except (KeyError, TypeError, ValueError):
                continue
            state = str(slot.get("state", "candidate"))
            if bool(slot.get("picked")) or bool(slot.get("non_target")):
                col = _COL_SLOT_DONE
            elif state == "lost_suspect" or int(slot.get("retries", 0)) > 0:
                col = _COL_SLOT_RETRY
            else:
                col = _COL_SLOT
            active = slot_id != 0 and slot_id == self.current_slot_id
            cv2.circle(canvas, (px, py), 16 if active else 14, _COL_TARGET if active else col,
                       3 if active else 2, cv2.LINE_AA)
            if bool(slot.get("picked")):
                cv2.drawMarker(canvas, (px, py), col, cv2.MARKER_TILTED_CROSS, 15, 2, cv2.LINE_AA)
            cv2.putText(canvas, f"F{slot_id}", (px - 12, py - 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.38, _COL_TARGET if active else col, 1, cv2.LINE_AA)

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
            f"state={self.mission_state}  phase={self.phase} zone={self.zone}  "
            f"t={time.time() - self._t0:5.0f}s",
            f"objects={n_obj} (picked/bl={n_bl})  tray shape={self.tray_shape} fruit={self.tray_fruit}",
            f"det wide={len(self.top_dets)} body={len(self.body_dets)}",
        ]
        if self.show_set2_slots and self.object_slots:
            unresolved = sum(
                1 for slot in self.object_slots
                if not bool(slot.get("picked")) and not bool(slot.get("non_target"))
            )
            lines.append(
                f"set2 slots={len(self.object_slots)} unresolved={unresolved} current=F{self.current_slot_id}"
            )
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
