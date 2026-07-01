"""World model: tracked objects in `field` frame + robot pose (DUAL-CAMERA FUSION).

Maintains the integrated world model for the AI Robot Challenge. Fuses BOTH cameras
into one nearest-neighbour tracker in field-frame metres:
  • WIDE (top) cam `/camera_top/detections` — sees far; pixels projected via the fisheye
    ray + wide-cam extrinsics (node params). Broad coverage, coarse identity.
  • BODY cam `/camera_body/detections` — base-fixed, sees the near workspace; pixels
    projected via the calibrated pick homography H (body px -> arm_base cm) + the
    arm_base->base_link offset. Closer -> MORE reliable, so body observations win on
    class/confidence when both cameras see the same object (they land at ~the same field
    xy and the tracker associates them into one track).
  • `/classification/siglip` — attaches the concrete fruit type to the most recent
    body-observed Set2 (fruit_photo_cube) track (SigLIP does fruit-type only; YOLO already
    established it is a fruit box).
Also honours `/localization/pose` (robot pose) and `/world_model/blacklist_add`.

Publishes `/world_model` (robot_interfaces/WorldModel) at a fixed rate. Uses no heavy ML;
stays alive with no inputs. Wide projection needs top_fx/fy/cx/cy + extrinsics; body
projection needs body_homography_path (data/pick/homography.npz) — either may be absent
and that camera's fusion is simply skipped (a warning is logged once).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import UInt64
from robot_interfaces.msg import Classification, DetectionArray, Object, WorldModel

# cv2/numpy only needed for the fisheye ray (top cam is a ~150 deg fisheye). Guarded so the
# node still runs (pinhole path) if they are missing.
try:
    import cv2
    import numpy as np

    _CV2_AVAILABLE = True
except Exception:  # noqa: BLE001
    _CV2_AVAILABLE = False


# Label -> set_type. The custom YOLOv8n (wide.pt / cube.pt) is 5-class as of 2026-07-01:
# the 4 white polyhedra are Set1 and fruit_photo_cube is the printed Set2 box. BOTH cameras
# emit these labels, so detections carry a concrete set_type straight from YOLO. The specific
# fruit (apple/orange/...) is filled in later by SigLIP (fruit-type only); see on_siglip.
_LABEL_TO_SET_TYPE: dict[str, int] = {
    "cube": 1,
    "octahedron": 1,
    "dodecahedron": 1,
    "icosahedron": 1,
    "fruit_photo_cube": 2,
}

# The set of SigLIP-identified fruit names count as Set2 too (once a track's class_label is
# promoted from fruit_photo_cube to the concrete fruit, keep it typed Set2).
_FRUIT_LABELS = frozenset({"apple", "orange", "banana", "pineapple"})

# A body-cam observation is closer / more reliable, so its class vote counts this much more
# than a wide-cam vote. Identity is the argmax of accumulated confidence-weighted votes
# (aggregated over count AND confidence — NOT a single latest-frame decision).
_BODY_VOTE_WEIGHT = 3.0


@dataclass
class Track:
    """A single tracked object in field-frame metres (fused across both cameras)."""

    id: int
    x: float
    y: float
    confidence: float
    last_seen_sec: float
    class_label: str = ""
    set_type: int = 0
    blacklisted: bool = False
    # Fusion bookkeeping. Identity is decided by confidence-weighted VOTES accumulated over
    # every observation (best estimate over count + confidence), not by the latest frame.
    source: str = ""              # "wide", "body", or "wide+body" once both have seen it
    seen_body: bool = False       # a body-cam (reliable) detection has updated this track
    fruit_label: str = ""         # concrete fruit from SigLIP once a face was read
    fruit_confidence: float = 0.0
    last_body_sec: float = 0.0     # last time a body detection updated this track
    n_obs: int = 0                # total detections fused (evidence count)
    n_body: int = 0               # of which from the body cam
    class_votes: dict = field(default_factory=dict)   # YOLO label -> summed conf*weight
    fruit_votes: dict = field(default_factory=dict)   # SigLIP fruit -> summed conf
    # Landmark anchoring: once a track is stable it LOCKS to a frozen world position, and
    # further re-observations are used to correct the ROBOT pose (not to move the track).
    locked: bool = False
    anchor_x: float = 0.0
    anchor_y: float = 0.0
    outlier_count: int = 0        # consecutive frames this anchor disagreed with the consensus
                                  # drift (i.e. the object itself moved) -> triggers an unlock


class WorldModelNode(Node):
    def __init__(self) -> None:
        super().__init__("world_model_node")

        # Publish / tracker timing.
        self.declare_parameter("publish_rate_hz", 10.0)

        # Top-cam intrinsics (pixels). Any zero -> cannot project (warn, pose only).
        self.declare_parameter("top_fx", 0.0)
        self.declare_parameter("top_fy", 0.0)
        self.declare_parameter("top_cx", 0.0)
        self.declare_parameter("top_cy", 0.0)
        # The wide (top) cam is a ~150 deg FISHEYE. fisheye_model=true -> the pixel->ground
        # ray is computed with cv2.fisheye.undistortPoints(K, D[4]) instead of the pinhole
        # ray ((u-cx)/fx,(v-cy)/fy,1). dist_coeffs is the 4 fisheye coeffs.
        self.declare_parameter("fisheye_model", False)
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0])
        # Top image is published 180-deg rotated (camera flip_method=2, mounted inverted). The
        # rotated K handles the principal-point shift, but the optical x/y axes also flip, so the
        # undistorted ray must be rotated 180 about the optical axis (negate x,y) to stay
        # consistent with the (unrotated-mount) R_base_cam extrinsic. Without this, forward<->back
        # and left<->right are swapped in the ground projection.
        self.declare_parameter("image_rotated_180", False)

        # Top-cam extrinsics (base_link / floor).
        self.declare_parameter("cam_height_m", 0.8)        # camera height above floor
        self.declare_parameter("cam_pitch_deg", 60.0)      # downward tilt from horizontal
        self.declare_parameter("cam_offset_x", -0.15)      # rear lift offset (base_link x)
        self.declare_parameter("cam_offset_y", 0.0)        # base_link y offset
        # Camera mount yaw about base z. 0 = camera "forward" aligned with robot +x; 180 if the
        # rig is mounted reversed (verified by ground truth: 0.5m-front object lands at image top
        # only with yaw=180). Independent of image_rotated_180 (that is the optical-axis flip).
        self.declare_parameter("cam_yaw_deg", 0.0)
        self.declare_parameter("object_center_height_m", 0.04)  # ~8cm tall -> center 0.04

        # --- BODY-cam fusion (base-fixed cam; pick homography H = body px -> arm_base cm) ---
        # H (data/pick/homography.npz) maps the body-cam ground-contact pixel to arm_base
        # (x forward, y left) in CENTIMETRES. arm_base sits at (arm_base_offset_x/y) in
        # base_link (static tf, zero yaw). body_workspace_m = [x_min,x_max,y_min,y_max] in
        # METRES (arm_base frame): only fuse body detections that project inside H's
        # calibrated near-workspace (outside it the homography extrapolates badly).
        self.declare_parameter("body_homography_path", "data/pick/homography.npz")
        self.declare_parameter("arm_base_offset_x", 0.15)
        self.declare_parameter("arm_base_offset_y", 0.0)
        self.declare_parameter("body_workspace_m", [0.0, 0.6, -0.35, 0.35])

        # --- Object-landmark pose correction (lightweight SLAM-style drift fix) ---
        # When enabled, tracks that reach lock_min_obs observations with lock_min_conf freeze to
        # a world anchor; re-observing >=landmark_min_pairs anchors lets us solve the rigid pose
        # drift (2D Umeyama) and publish a correction delta to the localizer. Anchors are frozen
        # (not chasing the drifting pose), which is what makes this non-circular.
        self.declare_parameter("landmark_correction", False)
        self.declare_parameter("lock_min_obs", 5)
        self.declare_parameter("lock_min_conf", 0.4)
        self.declare_parameter("landmark_min_pairs", 3)
        self.declare_parameter("landmark_max_resid_m", 0.30)
        self.declare_parameter("unlock_after", 3)   # consecutive outlier frames -> object moved

        # Tracker tuning.
        self.declare_parameter("assoc_radius_m", 0.18)     # nearest-neighbour gate
        self.declare_parameter("conf_ema", 0.5)            # EMA weight on new sample
        self.declare_parameter("forget_after_sec", 6.0)    # drop unseen non-blacklisted

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.fx = float(self.get_parameter("top_fx").value)
        self.fy = float(self.get_parameter("top_fy").value)
        self.cx = float(self.get_parameter("top_cx").value)
        self.cy = float(self.get_parameter("top_cy").value)
        self.cam_height = float(self.get_parameter("cam_height_m").value)
        self.cam_pitch_deg = float(self.get_parameter("cam_pitch_deg").value)
        self.cam_offset_x = float(self.get_parameter("cam_offset_x").value)
        self.cam_offset_y = float(self.get_parameter("cam_offset_y").value)
        self.cam_yaw_deg = float(self.get_parameter("cam_yaw_deg").value)
        self.object_center_height = float(self.get_parameter("object_center_height_m").value)
        self.assoc_radius = float(self.get_parameter("assoc_radius_m").value)
        self.conf_ema = float(self.get_parameter("conf_ema").value)
        self.forget_after = float(self.get_parameter("forget_after_sec").value)

        self.arm_base_off_x = float(self.get_parameter("arm_base_offset_x").value)
        self.arm_base_off_y = float(self.get_parameter("arm_base_offset_y").value)
        ws = [float(v) for v in self.get_parameter("body_workspace_m").value]
        self.body_ws = ws if len(ws) == 4 else [0.0, 0.6, -0.35, 0.35]

        self.landmark_correction = bool(self.get_parameter("landmark_correction").value)
        self.lock_min_obs = int(self.get_parameter("lock_min_obs").value)
        self.lock_min_conf = float(self.get_parameter("lock_min_conf").value)
        self.landmark_min_pairs = int(self.get_parameter("landmark_min_pairs").value)
        self.landmark_max_resid = float(self.get_parameter("landmark_max_resid_m").value)
        self.unlock_after = int(self.get_parameter("unlock_after").value)
        self._corr_pairs: list[tuple[int, float, float, float, float]] = []  # (tid,obs_x,obs_y,anchor_x,anchor_y)

        self.can_project = self.fx != 0.0 and self.fy != 0.0 and self.cx != 0.0 and self.cy != 0.0

        # Body homography (px -> arm_base cm). Loaded once; body fusion disabled if absent.
        self._body_H = None
        body_h_path = str(self.get_parameter("body_homography_path").value)
        if _CV2_AVAILABLE and body_h_path:
            try:
                data = np.load(body_h_path)
                self._body_H = np.asarray(data["H"], dtype=np.float64)
            except Exception as exc:  # noqa: BLE001 - missing/bad H just disables body fusion
                self.get_logger().warn(
                    f"body homography '{body_h_path}' unavailable ({exc}); "
                    "body-cam fusion disabled (wide-cam only)"
                )
        elif not _CV2_AVAILABLE:
            self.get_logger().warn("cv2/numpy unavailable; body-cam fusion disabled")
        self.can_project_body = self._body_H is not None

        # Fisheye ray setup (top cam ~150 deg fisheye). Falls back to pinhole if cv2 is
        # missing or fisheye_model is false.
        self.use_fisheye = bool(self.get_parameter("fisheye_model").value)
        self.rotated_180 = bool(self.get_parameter("image_rotated_180").value)
        dist = [float(c) for c in self.get_parameter("dist_coeffs").value]
        self._fish_K = None
        self._fish_D = None
        if self.use_fisheye and self.can_project:
            if not _CV2_AVAILABLE:
                self.get_logger().error(
                    "fisheye_model set but cv2/numpy unavailable; falling back to pinhole ray"
                )
                self.use_fisheye = False
            elif len(dist) < 4:
                self.get_logger().error(
                    f"fisheye_model needs 4 dist_coeffs, got {len(dist)}; falling back to pinhole"
                )
                self.use_fisheye = False
            else:
                self._fish_K = np.array(
                    [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]], np.float64
                )
                self._fish_D = np.array(dist[:4], np.float64).reshape(4, 1)

        # Precompute the (constant) base->cam rotation from a downward pitch about base y.
        # REP-103: base x forward, y left, z up; optical axis = camera +z.
        # With zero pitch the optical axis points along base +x (forward); positive pitch
        # tips it down toward the floor.
        self._R_base_cam = self._matmul3(
            self._rz(math.radians(self.cam_yaw_deg)), self._build_R_base_cam(self.cam_pitch_deg)
        )

        # Latest robot pose in field (0,0,0 until first /localization/pose).
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_theta: float = 0.0
        self.have_pose: bool = False

        # Track store.
        self.tracks: dict[int, Track] = {}
        self._next_id: int = 1
        # Last track updated by a body-cam fruit_photo_cube detection — the SigLIP fruit
        # result (which carries no position) is attached to this track.
        self._last_body_fruit_id: int | None = None

        self.create_subscription(DetectionArray, "/camera_top/detections", self.on_detections, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_detections, 10)
        self.create_subscription(Classification, "/classification/siglip", self.on_siglip, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(UInt64, "/world_model/blacklist_add", self.on_blacklist_add, 10)

        self.pub = self.create_publisher(WorldModel, "/world_model", 10)
        # Pose correction delta (dx, dy, dtheta in field frame) for the localizer.
        self.pub_corr = self.create_publisher(Vector3, "/localization/landmark_correction", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        if not self.can_project:
            self.get_logger().warn(
                "top-cam intrinsics unset (top_fx/fy/cx/cy); projection disabled — "
                "publishing robot pose + empty objects until calibrated"
            )
        self.get_logger().info(
            f"rate={rate}Hz project={self.can_project} fisheye={self.use_fisheye} "
            f"rot180={self.rotated_180} body_fusion={self.can_project_body} "
            f"landmark_corr={self.landmark_correction} "
            f"cam_h={self.cam_height}m pitch={self.cam_pitch_deg}deg yaw={self.cam_yaw_deg}deg "
            f"offset=({self.cam_offset_x},{self.cam_offset_y}) "
            f"arm_base_off=({self.arm_base_off_x},{self.arm_base_off_y}) body_ws={self.body_ws} "
            f"obj_h={self.object_center_height}m assoc={self.assoc_radius}m "
            f"ema={self.conf_ema} forget={self.forget_after}s"
        )

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _build_R_base_cam(pitch_deg: float) -> list[list[float]]:
        """Rotation base_link -> camera optical frame for a downward pitch.

        Camera optical axis is +z (REP-103 pinhole). We want it pointing forward and
        down in base_link. Start from optical-axis-along-base-+x and rotate down by the
        pitch about the base y-axis. Returned as a 3x3 row-major matrix mapping a
        vector expressed in the camera optical frame into base_link.
        NOTE: this is an idealised model; the real rig needs extrinsic calibration.
        """
        p = math.radians(pitch_deg)
        cp = math.cos(p)
        sp = math.sin(p)
        # Camera optical axes expressed in base_link:
        #   cam +z (optical/forward): forward & down -> (cos p, 0, -sin p)
        #   cam +x (image right)    : base -y (image x grows to robot's right) -> (0,-1,0)
        #   cam +y (image down)     : down & forward  -> (-sin p, 0, -cos p)
        # Columns are the camera basis vectors in base_link.
        return [
            [0.0, -sp, cp],
            [-1.0, 0.0, 0.0],
            [0.0, -cp, -sp],
        ]

    @staticmethod
    def _rz(theta: float) -> list[list[float]]:
        c = math.cos(theta)
        s = math.sin(theta)
        return [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _matmul3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
        return [
            [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

    @staticmethod
    def _matvec3(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ----------------------------------------------------------------- projection

    def _project_pixel(self, u: float, v: float) -> tuple[float, float] | None:
        """Project a top-cam pixel (u,v) onto the object-center ground plane -> field xy.

        Returns None when intrinsics are unset, no pose yet, or the ray does not point
        down into the plane (parallel / upward). Intrinsics & extrinsics need calibration.
        """
        if not self.can_project or not self.have_pose:
            return None

        # Ray in camera optical frame. Fisheye: undistort the pixel to a normalized pinhole
        # ray (x,y,1); pinhole fallback: d = ((u-cx)/fx,(v-cy)/fy,1). The top cam is a ~150 deg
        # fisheye, so the pinhole formula is badly wrong at the periphery.
        if self.use_fisheye:
            und = cv2.fisheye.undistortPoints(
                np.array([[[u, v]]], dtype=np.float64), self._fish_K, self._fish_D
            )
            xn, yn = und[0, 0]
            if self.rotated_180:
                xn, yn = -xn, -yn   # 180-deg image rotation -> ray rotated 180 about optical axis
            d_cam = (float(xn), float(yn), 1.0)
        else:
            d_cam = ((u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0)

        # R_field_cam = Rz(theta) @ R_base_cam.
        r_field_cam = self._matmul3(self._rz(self.robot_theta), self._R_base_cam)
        ray = self._matvec3(r_field_cam, d_cam)

        # Camera position in field:
        #   pcam = (rx,ry,0) + Rz(theta)*(off_x,off_y,0) + (0,0,cam_height)
        off = self._matvec3(self._rz(self.robot_theta), (self.cam_offset_x, self.cam_offset_y, 0.0))
        pcam = (self.robot_x + off[0], self.robot_y + off[1], self.cam_height + off[2])

        # Intersect pcam + t*ray with plane z = object_center_height.
        # Need ray.z < 0 (pointing down) and t > 0.
        if ray[2] >= -1e-6:
            return None  # ray parallel to / above the plane
        t = (self.object_center_height - pcam[2]) / ray[2]
        if t <= 0.0:
            return None
        return (pcam[0] + t * ray[0], pcam[1] + t * ray[1])

    # ----------------------------------------------------------------- callbacks

    def on_pose(self, msg: PoseStamped) -> None:
        self.robot_x = float(msg.pose.position.x)
        self.robot_y = float(msg.pose.position.y)
        # yaw from quaternion (z-up planar): theta = atan2(2*(wz+xy), 1-2*(y^2+z^2)).
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
        self.have_pose = True

    def on_blacklist_add(self, msg: UInt64) -> None:
        track = self.tracks.get(int(msg.data))
        if track is None:
            self.get_logger().warn(
                f"blacklist_add for unknown id={int(msg.data)} (ignored)",
                throttle_duration_sec=2.0,
            )
            return
        track.blacklisted = True  # keep so it isn't recreated/re-selected; never un-blacklist
        self.get_logger().info(f"blacklisted track id={track.id}")

    def on_detections(self, msg: DetectionArray) -> None:
        if not self.can_project:
            self.get_logger().warn(
                "top-cam detections received but projection disabled (intrinsics unset)",
                throttle_duration_sec=5.0,
            )
            return
        if not self.have_pose:
            self.get_logger().warn(
                "top-cam detections received before any robot pose; skipping",
                throttle_duration_sec=5.0,
            )
            return

        now = self._now_sec()
        self._corr_pairs.clear()
        for det in msg.detections:
            # Ground-contact pixel: box bottom-center, averaged with center for stability.
            u_center = float(det.x_center)
            v_center = float(det.y_center)
            v_bottom = v_center + float(det.height) * 0.5
            v = 0.5 * (v_center + v_bottom)
            xy = self._project_pixel(u_center, v)
            if xy is None:
                continue
            label = str(det.label)
            set_type = _LABEL_TO_SET_TYPE.get(label, 0)
            self._associate(xy[0], xy[1], float(det.confidence), label, set_type, now, "wide")
        self._maybe_publish_correction()   # wide frame sees many objects -> best rigid solve

    def on_body_detections(self, msg: DetectionArray) -> None:
        """Body-cam detections: project via the pick homography and fuse (body wins identity)."""
        if not self.can_project_body:
            self.get_logger().warn(
                "body-cam detections received but homography unavailable; skipping",
                throttle_duration_sec=10.0,
            )
            return
        if not self.have_pose:
            return
        now = self._now_sec()
        self._corr_pairs.clear()
        for det in msg.detections:
            # Ground-contact pixel = box bottom-center (matches pick_calib's anchor).
            u = float(det.x_center)
            v = float(det.y_center) + float(det.height) * 0.5
            xy = self._project_body_pixel(u, v)
            if xy is None:
                continue  # outside the homography's calibrated near-workspace
            label = str(det.label)
            set_type = _LABEL_TO_SET_TYPE.get(label, 0)
            tid = self._associate(xy[0], xy[1], float(det.confidence), label, set_type, now, "body")
            # Remember which track a body fruit box hit so a following SigLIP result attaches here.
            if label == "fruit_photo_cube":
                self._last_body_fruit_id = tid
        self._maybe_publish_correction()

    def on_siglip(self, msg: Classification) -> None:
        """Attach the concrete fruit (SigLIP argmax) to the most-recent body fruit track.

        SigLIP does fruit-TYPE only; YOLO already established the box is a fruit_photo_cube.
        We use the argmax label (not the hard is_target sigmoid gate, which rarely fires) and
        keep the confidence for the map/log.
        """
        fruit = str(msg.label)
        if fruit not in _FRUIT_LABELS:
            return
        tid = self._last_body_fruit_id
        tr = self.tracks.get(tid) if tid is not None else None
        if tr is None:
            return
        # Vote across faces/frames rather than overwriting with the latest reading.
        tr.fruit_votes[fruit] = tr.fruit_votes.get(fruit, 0.0) + max(0.05, float(msg.confidence))
        tr.fruit_confidence = max(tr.fruit_confidence, float(msg.confidence))
        tr.set_type = 2
        self._refresh_identity(tr)

    def _project_body_pixel(self, u: float, v: float) -> tuple[float, float] | None:
        """Body-cam pixel -> field xy via the pick homography H, or None if out of workspace."""
        if not self.can_project_body or not self.have_pose:
            return None
        pt = np.array([[[float(u), float(v)]]], dtype=np.float64)
        out = cv2.perspectiveTransform(pt, self._body_H)[0][0]
        # H yields arm_base (x forward, y left) in CENTIMETRES -> metres.
        x_ab, y_ab = float(out[0]) / 100.0, float(out[1]) / 100.0
        x0, x1, y0, y1 = self.body_ws
        if not (x0 <= x_ab <= x1 and y0 <= y_ab <= y1):
            return None
        # arm_base -> base_link (static offset, zero yaw).
        bx = self.arm_base_off_x + x_ab
        by = self.arm_base_off_y + y_ab
        # base_link -> field (rotate by heading, translate by robot xy).
        ct, st = math.cos(self.robot_theta), math.sin(self.robot_theta)
        return (self.robot_x + bx * ct - by * st, self.robot_y + bx * st + by * ct)

    # ------------------------------------------------------------------- tracker

    def _associate(
        self, x: float, y: float, conf: float, label: str, set_type: int, now: float, source: str
    ) -> int:
        """Associate a projected detection to the nearest track (or create one). Returns its id.

        `source` is "wide" or "body". Body observations are closer/more reliable, so they pull
        position harder and win on class/set_type; a wide detection never overwrites an identity
        the body cam has already established, and neither clobbers a concrete SigLIP fruit name.
        """
        is_body = source == "body"
        best_id: int | None = None
        best_d = self.assoc_radius
        for tid, tr in self.tracks.items():
            d = math.hypot(tr.x - x, tr.y - y)
            if d <= best_d:
                best_d = d
                best_id = tid

        if best_id is None:
            tid = self._next_id
            self._next_id += 1
            tr = Track(
                id=tid, x=x, y=y, confidence=conf, last_seen_sec=now,
                source=source, seen_body=is_body,
                last_body_sec=(now if is_body else 0.0),
                n_obs=1, n_body=(1 if is_body else 0),
            )
            self._vote(tr, label, conf, is_body)
            self._refresh_identity(tr)
            self.tracks[tid] = tr
            return tid

        # Fuse into the matched track.
        tr = self.tracks[best_id]
        if tr.locked and self.landmark_correction:
            # Frozen landmark: don't move it — record (track id, fresh obs, anchor) so the batch
            # solve can recover the robot-pose drift AND spot anchors that moved (object picked up).
            self._corr_pairs.append((best_id, x, y, tr.anchor_x, tr.anchor_y))
        else:
            pos_a = 0.7 if is_body else self.conf_ema     # body pulls position harder
            tr.x = (1.0 - pos_a) * tr.x + pos_a * x
            tr.y = (1.0 - pos_a) * tr.y + pos_a * y
        tr.confidence = (1.0 - self.conf_ema) * tr.confidence + self.conf_ema * conf
        if is_body:
            tr.confidence = max(tr.confidence, conf)  # trust a confident close look
            tr.seen_body = True
            tr.last_body_sec = now
            tr.n_body += 1
        tr.n_obs += 1
        tr.last_seen_sec = now
        if source not in tr.source:
            tr.source = "wide+body" if tr.source else source
        # Accumulate the class vote and re-estimate identity from ALL evidence so far.
        self._vote(tr, label, conf, is_body)
        self._refresh_identity(tr)
        # Lock a stable track into a frozen world anchor once it has enough confident evidence.
        if (
            self.landmark_correction
            and not tr.locked
            and tr.n_obs >= self.lock_min_obs
            and tr.confidence >= self.lock_min_conf
        ):
            tr.locked = True
            tr.anchor_x = tr.x
            tr.anchor_y = tr.y
        # Never un-blacklist (blacklisted flag is left untouched).
        return best_id

    @staticmethod
    def _vote(tr: Track, label: str, conf: float, is_body: bool) -> None:
        """Add one confidence-weighted class vote (body votes weigh more)."""
        if not label:
            return
        w = max(0.05, conf) * (_BODY_VOTE_WEIGHT if is_body else 1.0)
        tr.class_votes[label] = tr.class_votes.get(label, 0.0) + w

    @staticmethod
    def _refresh_identity(tr: Track) -> None:
        """Best-estimate identity = argmax of accumulated votes (fruit votes win for Set2)."""
        if tr.fruit_votes:
            tr.class_label = max(tr.fruit_votes, key=tr.fruit_votes.get)
            tr.fruit_label = tr.class_label
            tr.set_type = 2
        elif tr.class_votes:
            best = max(tr.class_votes, key=tr.class_votes.get)
            tr.class_label = best
            st = _LABEL_TO_SET_TYPE.get(best, 0)
            if st:
                tr.set_type = st

    # -------------------------------------------------------- landmark correction
    def _maybe_publish_correction(self) -> None:
        """Recover the robot-pose drift from re-observed anchors, and unlock anchors that MOVED.

        Pose drift shows up as a rigid transform common to ALL anchors; a moved object (bumped or
        picked up) is the odd one out. So we fit the consensus rigid transform (trimmed least
        squares), treat persistent outliers as movers and UNLOCK them (re-track at the new spot),
        and publish the pose correction from the inliers only.
        """
        pairs = self._corr_pairs   # (tid, ox, oy, lx, ly)
        if not self.landmark_correction or len(pairs) < self.landmark_min_pairs:
            return

        kept = list(pairs)
        theta_c = tcx = tcy = None
        for _ in range(4):   # iteratively trim movers until the consensus is clean
            if len(kept) < self.landmark_min_pairs:
                break
            th, tx, ty = self._umeyama_2d([(p[1], p[2], p[3], p[4]) for p in kept])
            if th is None:
                return
            c, s = math.cos(th), math.sin(th)
            resid = [
                math.hypot((c * p[1] - s * p[2] + tx) - p[3], (s * p[1] + c * p[2] + ty) - p[4])
                for p in kept
            ]
            if max(resid) <= self.landmark_max_resid:
                theta_c, tcx, tcy = th, tx, ty
                break
            trimmed = [p for p, r in zip(kept, resid) if r <= self.landmark_max_resid]
            if len(trimmed) < self.landmark_min_pairs:
                return   # not enough clean anchors for a reliable consensus this frame
            kept = trimmed
        if theta_c is None:
            return   # never converged (too many movers / big jump) -> skip this frame

        # Classify each observed anchor as inlier (drift) or mover, and unlock persistent movers.
        c, s = math.cos(theta_c), math.sin(theta_c)
        for tid, ox, oy, lx, ly in pairs:
            tr = self.tracks.get(tid)
            if tr is None:
                continue
            r = math.hypot((c * ox - s * oy + tcx) - lx, (s * ox + c * oy + tcy) - ly)
            if r <= self.landmark_max_resid:
                tr.outlier_count = 0
            else:
                tr.outlier_count += 1
                if tr.outlier_count >= self.unlock_after:
                    # object moved -> stop using it as a fixed anchor; re-track from the new obs.
                    tr.locked = False
                    tr.outlier_count = 0
                    tr.x, tr.y = ox, oy
                    self.get_logger().info(f"anchor #{tid} moved -> unlocked, re-tracking")

        # Pose correction from the inlier consensus (P' = T o P, delta = P' - P).
        px, py = self.robot_x, self.robot_y
        dx, dy, dth = c * px - s * py + tcx - px, s * px + c * py + tcy - py, theta_c
        if abs(dx) > 2.0 or abs(dy) > 2.0 or abs(dth) > 1.0:
            return   # implausible -> skip
        self.pub_corr.publish(Vector3(x=float(dx), y=float(dy), z=float(dth)))
        n_locked = sum(1 for t in self.tracks.values() if t.locked)
        self.get_logger().info(
            f"landmark correction dx={dx:+.3f} dy={dy:+.3f} dth={dth:+.3f} "
            f"(inliers~{len(kept)}/{len(pairs)}, {n_locked} locked)",
            throttle_duration_sec=2.0,
        )

    @staticmethod
    def _umeyama_2d(pts):
        """Closed-form 2D rotation+translation aligning source O to target L -> (theta, tx, ty)."""
        n = len(pts)
        if n == 0:
            return None, 0.0, 0.0
        oxm = sum(p[0] for p in pts) / n
        oym = sum(p[1] for p in pts) / n
        lxm = sum(p[2] for p in pts) / n
        lym = sum(p[3] for p in pts) / n
        a = b = 0.0
        for ox, oy, lx, ly in pts:
            ocx, ocy = ox - oxm, oy - oym
            lcx, lcy = lx - lxm, ly - lym
            a += ocx * lcx + ocy * lcy   # sum dot(Oc, Lc)
            b += ocx * lcy - ocy * lcx   # sum cross(Oc, Lc)
        if a == 0.0 and b == 0.0:
            return None, 0.0, 0.0
        theta = math.atan2(b, a)
        c, s = math.cos(theta), math.sin(theta)
        tx = lxm - (c * oxm - s * oym)
        ty = lym - (s * oxm + c * oym)
        return theta, tx, ty

    def _forget_stale(self, now: float) -> None:
        stale = [
            tid
            for tid, tr in self.tracks.items()
            if not tr.blacklisted and (now - tr.last_seen_sec) > self.forget_after
        ]
        for tid in stale:
            del self.tracks[tid]
        if stale:
            self.get_logger().info(f"forgot {len(stale)} stale track(s)")

    # ------------------------------------------------------------------- publish

    def tick(self) -> None:
        now = self._now_sec()
        self._forget_stale(now)

        msg = WorldModel()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"

        objects: list[Object] = []
        for tr in self.tracks.values():
            obj = Object()
            obj.id = int(tr.id)
            obj.class_label = tr.class_label
            obj.set_type = int(tr.set_type)
            obj.x = float(tr.x)
            obj.y = float(tr.y)
            obj.confidence = float(tr.confidence)
            obj.last_seen = self._sec_to_time_msg(tr.last_seen_sec)
            obj.blacklisted = bool(tr.blacklisted)
            obj.n_obs = int(tr.n_obs)
            obj.source = tr.source
            obj.fruit_label = tr.fruit_label
            obj.locked = bool(tr.locked)
            objects.append(obj)
        msg.objects = objects

        msg.robot_x = float(self.robot_x)
        msg.robot_y = float(self.robot_y)
        msg.robot_theta = float(self.robot_theta)
        self.pub.publish(msg)

        # periodic map summary (debug): track counts by set + locked + robot pose
        n1 = sum(1 for t in self.tracks.values() if t.set_type == 1)
        n2 = sum(1 for t in self.tracks.values() if t.set_type == 2)
        n0 = sum(1 for t in self.tracks.values() if t.set_type == 0)
        nl = sum(1 for t in self.tracks.values() if t.locked)
        nb = sum(1 for t in self.tracks.values() if t.blacklisted)
        self.get_logger().info(
            f"map: tracks={len(self.tracks)} set1={n1} set2={n2} unk={n0} "
            f"locked={nl} blacklisted={nb} robot=({self.robot_x:.2f},{self.robot_y:.2f},{self.robot_theta:.2f})",
            throttle_duration_sec=5.0,
        )

    @staticmethod
    def _sec_to_time_msg(sec: float):
        from builtin_interfaces.msg import Time

        t = Time()
        t.sec = int(sec)
        t.nanosec = int(round((sec - int(sec)) * 1e9))
        return t


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldModelNode()
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
