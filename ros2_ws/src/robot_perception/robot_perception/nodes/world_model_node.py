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
from collections import deque
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool, Float32MultiArray, UInt64
from robot_interfaces.msg import Classification, DetectionArray, Object, WorldModel

# cv2/numpy only needed for the fisheye ray (top cam is a ~150 deg fisheye). Guarded so the
# node still runs (pinhole path) if they are missing.
try:
    import cv2
    import numpy as np

    _CV2_AVAILABLE = True
except Exception:  # noqa: BLE001
    _CV2_AVAILABLE = False


# Label -> set_type. The custom YOLOv8n models emit Set1 shapes, Set2 fruit-photo cubes,
# and the finish/storage landmark flag. The specific fruit (apple/orange/...) is filled in
# later by SigLIP (fruit-type only); see on_siglip.
_LABEL_TO_SET_TYPE: dict[str, int] = {
    "cube": 1,
    "octahedron": 1,
    "dodecahedron": 1,
    "icosahedron": 1,
    "fruit_photo_cube": 2,
    "arrival": 3,
}

# The set of SigLIP-identified fruit names count as Set2 too (once a track's class_label is
# promoted from fruit_photo_cube to the concrete fruit, keep it typed Set2).
_FRUIT_LABELS = frozenset({"apple", "orange", "banana", "pineapple"})

# Identity is decided PER SOURCE (see Track.body_votes/wide_votes): the body cam is the close,
# reliable camera, so when it has classified an object its confidence-summed vote wins outright;
# the wide cam only names objects the body never saw. This beats a single blended weight, which
# let the high-frame-rate wide cam out-accumulate the body's higher per-vote weight over time.


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
    fruit_cube_seen: bool = False  # sticky Set2 evidence from a confident fruit_photo_cube box
    fruit_cube_confidence: float = 0.0
    fruit_cube_wide_hits: int = 0
    fruit_cube_body_hits: int = 0
    last_body_sec: float = 0.0     # last time a body detection updated this track
    last_wide_sec: float = 0.0     # last time a wide detection updated this track (for miss penalty)
    n_obs: int = 0                # total detections fused (evidence count)
    n_body: int = 0               # of which from the body cam
    # Identity votes kept PER SOURCE. The body cam is the close, reliable camera, so when it
    # has classified an object its vote decides identity — a distant wide misread (white
    # polyhedra look alike) never overrides it. Within a source, votes are confidence-summed.
    wide_votes: dict = field(default_factory=dict)    # wide YOLO label -> summed conf
    body_votes: dict = field(default_factory=dict)    # body YOLO label -> summed conf
    fruit_votes: dict = field(default_factory=dict)   # SigLIP fruit -> summed conf
    # Landmark anchoring: once a track is stable it LOCKS to a frozen world position, and
    # further re-observations are used to correct the ROBOT pose (not to move the track).
    locked: bool = False
    anchor_x: float = 0.0
    anchor_y: float = 0.0
    outlier_count: int = 0        # consecutive frames this anchor disagreed with the consensus
                                  # drift (i.e. the object itself moved) -> triggers an unlock
    # Optional field-grid prior. A grid id is assigned when a confirmed track is close enough to
    # a free grid point. In grid-track-lock mode, later observations keep that track on-grid.
    spawn_grid_id: int = -1
    current_grid_id: int = -1
    grid_state: str = "off_grid"  # "grid_spawned", "moved", or "off_grid"
    grid_snapped: bool = False


class WorldModelNode(Node):
    def __init__(self) -> None:
        super().__init__("world_model_node")

        # Publish / tracker timing.
        self.declare_parameter("publish_rate_hz", 10.0)
        # When false, detections are ignored and no object tracks/slots are born. This lets the
        # opening motion finish and wall correction settle before the first field-grid objects are
        # committed to the map.
        self.declare_parameter("mapping_enabled_initially", True)

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
        # HEIGHT-PARALLAX correction: a ground homography assumes z=0, but the box-centre pixel of a
        # standing object is at object_center_height. Both cams anchor on the box CENTRE and correct
        # to the object's true ground centre: P = G - (h/H)(G - C_nadir). The low body cam (H≈0.145,
        # h/H≈0.28) needs it far more than the top wide cam (H≈0.885, h/H≈0.045). C_nadir/H per cam.
        self.declare_parameter("height_correct", True)
        self.declare_parameter("body_cam_nadir_x", 0.055)   # body cam ground nadir (base_link x), fwd
        self.declare_parameter("body_cam_nadir_y", 0.0)
        self.declare_parameter("body_cam_height_m", 0.155)  # body cam height above floor
        # Residual systematic bias: the oblique body cam's box CENTRE sits on the object's FRONT
        # face, so it reports the object ~half-depth NEARER than its true centroid (measured +3.9cm
        # radial over a full-turn: wide=centroid is farther). Push the body projection this far
        # radially OUT (from body nadir) so both cams land on the true centroid. 0 disables.
        self.declare_parameter("body_radial_trim_m", 0.039)

        # --- BODY-cam fusion (base-fixed cam; pick homography H = body px -> arm_base cm) ---
        # H (data/pick/homography.npz) maps the body-cam ground-contact pixel to arm_base
        # (x forward, y left) in CENTIMETRES. arm_base sits at (arm_base_offset_x/y) in
        # base_link (static tf, zero yaw). body_workspace_m = [x_min,x_max,y_min,y_max] in
        # METRES (arm_base frame): only fuse body detections that project inside H's
        # calibrated near-workspace (outside it the homography extrapolates badly).
        self.declare_parameter("body_homography_path", "data/pick/homography.npz")
        # Body GROUND homography (raw body pixel -> base_link x,y METRES), from aruco_calib.py
        # --cam body. Same rotation-centre frame as the wide ground H (no arm_base offset / cm).
        # When set it replaces the arm_base-cm body path above.
        self.declare_parameter("body_ground_homography_path", "")
        # Wide-cam ground homography (fisheye-undistorted normalised point -> base-frame x,y),
        # from scripts/wide_calib.py. If set, the wide projection uses this measured mapping
        # instead of the extrinsic pitch/height/offset model (more accurate absolute positions).
        self.declare_parameter("wide_homography_path", "")
        self.declare_parameter("arm_base_offset_x", 0.15)
        self.declare_parameter("arm_base_offset_y", 0.0)
        self.declare_parameter("body_workspace_m", [0.0, 0.6, -0.35, 0.35])
        # Body-cam downscale compensation: multiply each body detection pixel by these before the
        # body homography (which is calibrated at full 1640x1232). 1.0 = body published at full res.
        self.declare_parameter("body_px_scale_x", 1.0)
        self.declare_parameter("body_px_scale_y", 1.0)

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
        # Anchor spread (mean radius from their centroid) at which a heading fix is full-confidence.
        # Well-spread anchors constrain rotation tightly; clustered ones don't -> lower confidence.
        self.declare_parameter("landmark_spread_ref_m", 0.4)
        # HEADING is only trusted when the anchor geometry actually constrains rotation: enough
        # inliers AND wide spread. A tight/sparse cluster gives an ill-conditioned Umeyama theta
        # (pure noise) — so below these gates we zero the heading term and keep only translation
        # (which stays robust with few anchors). Prevents the sparse-view false-rotation jitter.
        self.declare_parameter("landmark_heading_min_pairs", 5)
        self.declare_parameter("landmark_heading_min_spread_m", 0.30)
        # Object-flow odometry (frame-to-frame wide-point scan matching).
        self.declare_parameter("object_flow", True)
        self.declare_parameter("object_flow_min_pairs", 4)       # need this many matched points to trust it
        self.declare_parameter("object_flow_assoc_m", 0.30)      # frame-to-frame NN gate (points barely move)
        self.declare_parameter("object_flow_max_dtheta", 0.5)    # reject a per-frame yaw jump beyond this (rad)
        self.declare_parameter("object_flow_trans_deadband_m", 0.008)  # sub-cm per-frame shift = noise -> 0 (no drift)

        # Tracker tuning.
        self.declare_parameter("assoc_radius_m", 0.18)     # tight gate: new-track spacing + duplicate merge
        # RE-ASSOCIATION gate (generous): a detection near an EXISTING track re-associates to it even
        # after pose jitter / a few-frame miss, so a re-seen object stays the SAME track instead of
        # spawning a duplicate. Bigger than assoc_radius but < grid spacing (0.5m) so distinct objects
        # never merge. This is the fix for "barely-moved = same object" + "missed 2-3 frames != new".
        self.declare_parameter("reassoc_radius_m", 0.32)
        # NEW-TRACK CONFIRMATION: a detection at a fresh spot must be re-seen this many times within
        # candidate_ttl before it becomes a track. Stops a lone motion-jitter/blur detection (while the
        # base shakes) from spawning a phantom object — that over-creation (hundreds of ids for ~28
        # real cubes) is what churned targets and caused the back-and-forth.
        self.declare_parameter("new_track_min_hits", 3)
        self.declare_parameter("candidate_ttl_sec", 0.8)
        self.declare_parameter("conf_ema", 0.5)            # EMA weight on new sample
        self.declare_parameter("forget_after_sec", 6.0)    # drop unseen non-blacklisted

        # Optional 7x6, 50-cm field-grid prior. It is applied only when a confirmed NEW track is
        # created, never while updating an existing track. Defaults off until field-frame origin
        # and axes have been checked against the real arena.
        self.declare_parameter("grid_prior_enabled", False)
        self.declare_parameter("grid_prior_debug", False)
        self.declare_parameter("grid_rows", 6)
        self.declare_parameter("grid_cols", 7)
        self.declare_parameter("grid_spacing_m", 0.50)
        self.declare_parameter("grid_origin_x_m", 0.50)
        self.declare_parameter("grid_origin_y_m", 0.50)
        self.declare_parameter("grid_initial_phase_sec", 20.0)
        self.declare_parameter("grid_initial_snap_radius_m", 0.15)
        self.declare_parameter("grid_new_snap_radius_m", 0.12)
        self.declare_parameter("grid_initial_snap_alpha", 0.80)
        self.declare_parameter("grid_new_snap_alpha", 0.45)
        self.declare_parameter("grid_anchor_snap_radius_m", 0.22)
        self.declare_parameter("grid_anchor_snap_alpha", 0.90)
        self.declare_parameter("grid_anchor_pose_radius_m", 0.35)
        self.declare_parameter("grid_track_lock_enabled", False)
        self.declare_parameter("grid_track_lock_radius_m", 0.26)
        self.declare_parameter("grid_track_lock_alpha", 1.0)
        self.declare_parameter(
            "zone_anchor_xy",
            [-0.75, 0.75, -0.75, -1.0, 0.75, -1.0, 0.75, 0.75],
        )
        self.declare_parameter("grid_moved_threshold_m", 0.22)

        # --- Position vs identity: DIFFERENT confidence cut-offs ---
        # The detector's own conf_threshold is the POSITION cut-off (a box above it means SOMETHING
        # is there — the wide cam is good at this). class_conf_threshold is the higher IDENTITY
        # cut-off: only a detection this confident casts a CLASS vote. A 0.5–0.85 wide box updates
        # the object's position/presence but leaves its identity unknown (the body cam decides).
        self.declare_parameter("class_conf_threshold", 0.85)
        # The body cam is the IDENTITY authority (close, reliable), so its class vote is trusted at a
        # LOWER bar than the wide cam — otherwise a real 0.6–0.8 body octa/icosa never establishes an
        # identity, gets no protection, and is swallowed by a neighbouring high-conf cube track.
        self.declare_parameter("class_conf_threshold_body", 0.50)
        self.declare_parameter("fruit_cube_sticky_enabled", True)
        self.declare_parameter("fruit_cube_sticky_conf_wide", 0.75)
        self.declare_parameter("fruit_cube_sticky_conf_body", 0.50)
        self.declare_parameter("fruit_cube_sticky_min_wide_hits", 3)
        # --- Negative evidence: an in-FOV object that is NOT seen loses presence ---
        # Camera coverage SHAPES (base_link, used for BOTH the negative-evidence "should be visible"
        # test and the map drawing): body = forward SECTOR (부채꼴) apex at the body cam; wide =
        # forward-biased ELLIPSE (the down-looking fisheye's ground footprint). A track inside a
        # camera's shape but unseen for see_window_sec is penalised at miss_penalty_per_sec/s; below
        # min_keep_conf it is dropped (clears phantoms and picked/removed objects).
        self.declare_parameter("body_fov_half_deg", 34.0)   # half of the body cam horizontal FOV
        self.declare_parameter("body_fov_near_m", 0.08)
        self.declare_parameter("body_fov_far_m", 0.6)
        self.declare_parameter("body_fov_apex_x", 0.065)    # body cam forward offset (sector apex)
        self.declare_parameter("wide_fov_forward_m", 1.6)   # wide ellipse semi-axis (forward)
        self.declare_parameter("wide_fov_lateral_m", 1.3)   # wide ellipse semi-axis (lateral)
        self.declare_parameter("wide_fov_center_x", 0.3)    # wide ellipse centre, forward of robot
        self.declare_parameter("miss_penalty_per_sec", 0.3)
        self.declare_parameter("see_window_sec", 0.6)
        self.declare_parameter("min_keep_conf", 0.05)

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
        self.height_correct = bool(self.get_parameter("height_correct").value)
        self.body_cam_nadir_x = float(self.get_parameter("body_cam_nadir_x").value)
        self.body_cam_nadir_y = float(self.get_parameter("body_cam_nadir_y").value)
        self.body_cam_height = float(self.get_parameter("body_cam_height_m").value)
        self.body_radial_trim = float(self.get_parameter("body_radial_trim_m").value)
        self.assoc_radius = float(self.get_parameter("assoc_radius_m").value)
        self.reassoc_radius = float(self.get_parameter("reassoc_radius_m").value)
        self.new_track_min_hits = int(self.get_parameter("new_track_min_hits").value)
        self.candidate_ttl = float(self.get_parameter("candidate_ttl_sec").value)
        self._candidates: list[dict] = []   # unconfirmed detections awaiting new_track_min_hits
        self.conf_ema = float(self.get_parameter("conf_ema").value)
        self.forget_after = float(self.get_parameter("forget_after_sec").value)
        self.grid_prior_enabled = bool(self.get_parameter("grid_prior_enabled").value)
        self.grid_prior_debug = bool(self.get_parameter("grid_prior_debug").value)
        self.grid_rows = max(0, int(self.get_parameter("grid_rows").value))
        self.grid_cols = max(0, int(self.get_parameter("grid_cols").value))
        self.grid_spacing = max(0.0, float(self.get_parameter("grid_spacing_m").value))
        self.grid_origin_x = float(self.get_parameter("grid_origin_x_m").value)
        self.grid_origin_y = float(self.get_parameter("grid_origin_y_m").value)
        self.grid_initial_phase = max(0.0, float(self.get_parameter("grid_initial_phase_sec").value))
        self.grid_initial_radius = max(0.0, float(self.get_parameter("grid_initial_snap_radius_m").value))
        self.grid_new_radius = max(0.0, float(self.get_parameter("grid_new_snap_radius_m").value))
        self.grid_initial_alpha = min(1.0, max(0.0, float(self.get_parameter("grid_initial_snap_alpha").value)))
        self.grid_new_alpha = min(1.0, max(0.0, float(self.get_parameter("grid_new_snap_alpha").value)))
        self.grid_anchor_radius = max(0.0, float(self.get_parameter("grid_anchor_snap_radius_m").value))
        self.grid_anchor_alpha = min(1.0, max(0.0, float(self.get_parameter("grid_anchor_snap_alpha").value)))
        self.grid_anchor_pose_radius = max(0.0, float(self.get_parameter("grid_anchor_pose_radius_m").value))
        self.grid_track_lock_enabled = bool(self.get_parameter("grid_track_lock_enabled").value)
        self.grid_track_lock_radius = max(0.0, float(self.get_parameter("grid_track_lock_radius_m").value))
        self.grid_track_lock_alpha = min(
            1.0, max(0.0, float(self.get_parameter("grid_track_lock_alpha").value))
        )
        za = [float(v) for v in self.get_parameter("zone_anchor_xy").value]
        self.zone_anchors = [(za[i], za[i + 1]) for i in range(0, min(len(za), 8), 2)]
        self.grid_moved_threshold = max(0.0, float(self.get_parameter("grid_moved_threshold_m").value))
        self.class_conf_threshold = float(self.get_parameter("class_conf_threshold").value)
        self.class_conf_threshold_body = float(self.get_parameter("class_conf_threshold_body").value)
        self.fruit_cube_sticky_enabled = bool(self.get_parameter("fruit_cube_sticky_enabled").value)
        self.fruit_cube_sticky_conf_wide = float(self.get_parameter("fruit_cube_sticky_conf_wide").value)
        self.fruit_cube_sticky_conf_body = float(self.get_parameter("fruit_cube_sticky_conf_body").value)
        self.fruit_cube_sticky_min_wide_hits = int(
            self.get_parameter("fruit_cube_sticky_min_wide_hits").value
        )
        self.body_fov_half = math.radians(float(self.get_parameter("body_fov_half_deg").value))
        self.body_fov_near = float(self.get_parameter("body_fov_near_m").value)
        self.body_fov_far = float(self.get_parameter("body_fov_far_m").value)
        self.body_fov_apex_x = float(self.get_parameter("body_fov_apex_x").value)
        self.wide_fov_fwd = float(self.get_parameter("wide_fov_forward_m").value)
        self.wide_fov_lat = float(self.get_parameter("wide_fov_lateral_m").value)
        self.wide_fov_cx = float(self.get_parameter("wide_fov_center_x").value)
        self.miss_penalty_per_sec = float(self.get_parameter("miss_penalty_per_sec").value)
        self.see_window = float(self.get_parameter("see_window_sec").value)
        self.min_keep_conf = float(self.get_parameter("min_keep_conf").value)
        self.tick_dt = 1.0 / rate

        self.body_px_scale_x = float(self.get_parameter("body_px_scale_x").value)
        self.body_px_scale_y = float(self.get_parameter("body_px_scale_y").value)
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
        self.landmark_spread_ref = float(self.get_parameter("landmark_spread_ref_m").value)
        self.landmark_heading_min_pairs = int(self.get_parameter("landmark_heading_min_pairs").value)
        self.landmark_heading_min_spread = float(self.get_parameter("landmark_heading_min_spread_m").value)
        self.object_flow = bool(self.get_parameter("object_flow").value)
        self.object_flow_min_pairs = int(self.get_parameter("object_flow_min_pairs").value)
        self.object_flow_assoc = float(self.get_parameter("object_flow_assoc_m").value)
        self.object_flow_max_dtheta = float(self.get_parameter("object_flow_max_dtheta").value)
        self.object_flow_trans_deadband = float(self.get_parameter("object_flow_trans_deadband_m").value)
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

        # Body GROUND homography (raw px -> base_link METRES, rotation-centre frame). Overrides
        # the arm_base-cm path when present (unifies wide+body into one frame).
        self._body_ground_H = None
        bg_path = str(self.get_parameter("body_ground_homography_path").value)
        if _CV2_AVAILABLE and bg_path:
            try:
                self._body_ground_H = np.asarray(np.load(bg_path)["H"], dtype=np.float64)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"body ground homography '{bg_path}' unavailable ({exc})")
        self.can_project_body = self._body_H is not None or self._body_ground_H is not None

        # Wide-cam ground homography (optional, from scripts/wide_calib.py). When present it
        # replaces the extrinsic wide projection with the measured pixel->ground mapping.
        self._wide_H = None
        wide_h_path = str(self.get_parameter("wide_homography_path").value)
        if _CV2_AVAILABLE and wide_h_path:
            try:
                self._wide_H = np.asarray(np.load(wide_h_path)["H"], dtype=np.float64)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"wide homography '{wide_h_path}' unavailable ({exc}); using extrinsic projection"
                )

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
        self.mapping_enabled = bool(self.get_parameter("mapping_enabled_initially").value)
        # Short robot-pose history (t, x, y, theta) so a detection can be projected at the ROBOT POSE
        # AT IMAGE-CAPTURE TIME (see _pose_at) rather than at message-arrival time — the inference
        # delay otherwise smears the map during rotation. ~3 s at the 20 Hz pose rate covers any
        # realistic capture->inference->arrival latency.
        self._pose_hist: "deque[tuple[float, float, float, float]]" = deque(maxlen=64)

        # Track store.
        self.tracks: dict[int, Track] = {}
        self._next_id: int = 1
        self._grid_points: list[tuple[float, float]] = [
            (self.grid_origin_x + c * self.grid_spacing,
             self.grid_origin_y + r * self.grid_spacing)
            for r in range(self.grid_rows)
            for c in range(self.grid_cols)
        ]
        self._grid_start_sec = self._now_sec()
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        # Last track updated by a body-cam fruit_photo_cube detection — the SigLIP fruit
        # result (which carries no position) is attached to this track, but only if that detection
        # was recent (see on_siglip): a stale id would bind the fruit type to the wrong track.
        self._last_body_fruit_id: int | None = None
        self._last_body_fruit_sec: float = 0.0
        self._fruit_attach_window: float = 1.0   # s; SigLIP result older than this after the last
        #                                          fruit-cube sighting is dropped (no fresh box to bind)

        self.create_subscription(DetectionArray, "/camera_top/detections", self.on_detections, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_detections, 10)
        self.create_subscription(Classification, "/classification/siglip", self.on_siglip, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(Bool, "/world_model/mapping_enabled", self.on_mapping_enabled, 10)
        self.create_subscription(
            Float32MultiArray, "/localization/wall_map_transform", self.on_wall_map_transform, 10
        )
        self.create_subscription(UInt64, "/world_model/blacklist_add", self.on_blacklist_add, 10)

        self.pub = self.create_publisher(WorldModel, "/world_model", 10)
        # Pose correction for the localizer: Float32MultiArray [dx, dy, dtheta, confidence]. The
        # confidence lets the localizer trust a well-conditioned landmark heading fix as an ABSOLUTE
        # reference (YOLO nails object bearings) instead of a tiny clamped nudge.
        self.pub_corr = self.create_publisher(Float32MultiArray, "/localization/landmark_correction", 10)
        # Per-camera RAW projected detections (field xy) for the viz overlay: flat [x,y,src,...]
        # with src=0 wide, src=1 body — lets you SEE whether the two homographies land together.
        self.pub_proj = self.create_publisher(Float32MultiArray, "/world_model/projected_dets", 10)
        self._proj_wide: list[tuple[float, float]] = []
        self._proj_body: list[tuple[float, float]] = []
        # OBJECT-FLOW ODOMETRY: the wide cam nails object POSITION, so tracking how the wide points
        # move between consecutive frames (in the robot frame, BEFORE pose) recovers the robot's own
        # rotation/translation — grounded in the real world, so it does NOT hallucinate motion from
        # motor vibration the way dense LK optical flow does. Published as [dtheta, dfwd, dleft, conf]
        # (robot-frame per-frame delta) for the localizer to use as the PRIMARY yaw source.
        self.pub_odom = self.create_publisher(Float32MultiArray, "/localization/object_odom", 10)
        self._prev_wide_base: list[tuple[float, float, str]] = []   # (bx, by, label) previous wide frame
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
            f"grid_prior={self.grid_prior_enabled} grid_points={len(self._grid_points)} "
            f"mapping_enabled={self.mapping_enabled} "
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

    def _clear_mapping_state(self) -> None:
        self.tracks.clear()
        self._candidates.clear()
        self._next_id = 1
        self._last_body_fruit_id = None
        self._last_body_fruit_sec = 0.0
        self._prev_wide_base = []
        self._proj_wide = []
        self._proj_body = []
        self._corr_pairs.clear()

    def _on_parameters_changed(self, params) -> SetParametersResult:
        """Allow the grid prior and its debug logging to be toggled without restarting ROS."""
        for param in params:
            if param.name == "grid_prior_enabled":
                self.grid_prior_enabled = bool(param.value)
            elif param.name == "grid_prior_debug":
                self.grid_prior_debug = bool(param.value)
            elif param.name == "grid_track_lock_enabled":
                self.grid_track_lock_enabled = bool(param.value)
            elif param.name == "grid_track_lock_radius_m":
                self.grid_track_lock_radius = max(0.0, float(param.value))
            elif param.name == "grid_track_lock_alpha":
                self.grid_track_lock_alpha = min(1.0, max(0.0, float(param.value)))
            elif param.name == "fruit_cube_sticky_enabled":
                self.fruit_cube_sticky_enabled = bool(param.value)
            elif param.name == "fruit_cube_sticky_conf_wide":
                self.fruit_cube_sticky_conf_wide = float(param.value)
            elif param.name == "fruit_cube_sticky_conf_body":
                self.fruit_cube_sticky_conf_body = float(param.value)
            elif param.name == "fruit_cube_sticky_min_wide_hits":
                self.fruit_cube_sticky_min_wide_hits = int(param.value)
        return SetParametersResult(successful=True)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        """builtin_interfaces/Time -> float seconds (same clock domain as the camera capture stamp
        and the localizer pose stamp)."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _pose_at(self, stamp) -> tuple[float, float, float]:
        """Robot pose (x, y, theta) interpolated at image-CAPTURE time `stamp`.

        Falls back to the latest pose when the history can't bracket the stamp (startup, a zero/
        missing stamp, or a stamp newer than the last pose). This is what lets projection use the
        pose AT CAPTURE rather than at inference-completion time, so a rotating robot's map does not
        smear by the (80-300 ms) capture->detection latency.
        """
        latest = (self.robot_x, self.robot_y, self.robot_theta)
        hist = list(self._pose_hist)
        if not hist:
            return latest
        t = self._stamp_to_sec(stamp)
        if t <= 0.0 or t >= hist[-1][0]:
            return latest
        if t <= hist[0][0]:
            return (hist[0][1], hist[0][2], hist[0][3])
        for i in range(len(hist) - 1):
            t0, x0, y0, th0 = hist[i]
            t1, x1, y1, th1 = hist[i + 1]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                dth = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))  # shortest-arc interp
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), th0 + a * dth)
        return latest

    # ----------------------------------------------------------------- projection

    def _project_pixel(self, u: float, v: float, pose=None) -> tuple[float, float] | None:
        """Project a top-cam pixel (u,v) onto the object-center ground plane -> field xy.

        `pose` = (x, y, theta) at IMAGE-CAPTURE TIME (from _pose_at); None -> latest pose.
        Returns None when intrinsics are unset, no pose yet, or the ray does not point
        down into the plane (parallel / upward). Intrinsics & extrinsics need calibration.
        """
        if not self.can_project or not self.have_pose:
            return None
        rx, ry, rth = pose if pose is not None else (self.robot_x, self.robot_y, self.robot_theta)

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
        r_field_cam = self._matmul3(self._rz(rth), self._R_base_cam)
        ray = self._matvec3(r_field_cam, d_cam)

        # Camera position in field:
        #   pcam = (rx,ry,0) + Rz(theta)*(off_x,off_y,0) + (0,0,cam_height)
        off = self._matvec3(self._rz(rth), (self.cam_offset_x, self.cam_offset_y, 0.0))
        pcam = (rx + off[0], ry + off[1], self.cam_height + off[2])

        # Intersect pcam + t*ray with plane z = object_center_height.
        # Need ray.z < 0 (pointing down) and t > 0.
        if ray[2] >= -1e-6:
            return None  # ray parallel to / above the plane
        t = (self.object_center_height - pcam[2]) / ray[2]
        if t <= 0.0:
            return None
        return (pcam[0] + t * ray[0], pcam[1] + t * ray[1])

    def _height_correct(self, bx: float, by: float, cnx: float, cny: float, cam_h: float) -> tuple[float, float]:
        """Remove ground-homography parallax for a box-CENTRE anchor at object_center_height.
        A point at height h projects (assuming z=0) to G displaced outward from the cam nadir C;
        the true ground point is P = G - (h/H)(G - C). Exact for a pinhole ray hitting the plane."""
        if not self.height_correct or cam_h <= self.object_center_height:
            return bx, by
        k = self.object_center_height / cam_h
        return bx - k * (bx - cnx), by - k * (by - cny)

    def _wide_pixel_to_base(self, u: float, v: float) -> tuple[float, float] | None:
        """Wide pixel -> base_link xy (metres, x forward / y left) via the ground homography.
        POSE-INDEPENDENT: this is the raw robot-frame measurement used for object-flow odometry."""
        if self._wide_H is None or self._fish_K is None:
            return None
        und = cv2.fisheye.undistortPoints(
            np.array([[[u, v]]], dtype=np.float64), self._fish_K, self._fish_D
        )
        xn, yn = und[0, 0]
        if self.rotated_180:
            xn, yn = -xn, -yn
        p = cv2.perspectiveTransform(
            np.array([[[float(xn), float(yn)]]], dtype=np.float64), self._wide_H
        )[0, 0]
        return (float(p[0]), float(p[1]))   # base-frame x forward, y left

    def _project_wide_H(self, u: float, v: float, pose=None) -> tuple[float, float] | None:
        """Wide pixel -> field xy via the measured ground homography (fisheye-undistorted).
        `pose` = (x, y, theta) at IMAGE-CAPTURE TIME (from _pose_at); None -> latest pose."""
        if not self.have_pose:
            return None
        base = self._wide_pixel_to_base(u, v)
        if base is None:
            return None
        # Height-correct the box-centre anchor to the object's true ground centre (wide nadir).
        bx, by = self._height_correct(base[0], base[1], self.cam_offset_x, self.cam_offset_y, self.cam_height)
        rx, ry, rth = pose if pose is not None else (self.robot_x, self.robot_y, self.robot_theta)
        ct, st = math.cos(rth), math.sin(rth)
        return (rx + bx * ct - by * st, ry + bx * st + by * ct)

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
        # Record this pose in the capture-time history, keyed by the localizer's publish stamp (same
        # clock as the camera capture stamp propagated through yolo_detector). Fall back to node-now
        # only if the stamp is unset. See _pose_at.
        t = self._stamp_to_sec(msg.header.stamp)
        self._pose_hist.append((t if t > 0.0 else self._now_sec(),
                                self.robot_x, self.robot_y, self.robot_theta))

    def on_mapping_enabled(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self.mapping_enabled:
            return
        self.mapping_enabled = enabled
        # Start the grid-prior initial phase from the first trusted mapping frame, not from process
        # startup while the robot is still doing its opening move.
        if enabled:
            self._grid_start_sec = self._now_sec()
            self._clear_mapping_state()
            self.get_logger().info("object mapping enabled after opening wall-settle")
        else:
            self._clear_mapping_state()
            self.get_logger().info("object mapping disabled")

    def on_wall_map_transform(self, msg: Float32MultiArray) -> None:
        """Move robot history, tracks and anchors by the same wall-alignment transform."""
        if len(msg.data) < 3:
            return
        tx, ty, dth = (float(msg.data[i]) for i in range(3))
        ct, st = math.cos(dth), math.sin(dth)

        def transform(x: float, y: float) -> tuple[float, float]:
            return ct * x - st * y + tx, st * x + ct * y + ty

        self.robot_x, self.robot_y = transform(self.robot_x, self.robot_y)
        self.robot_theta = math.atan2(
            math.sin(self.robot_theta + dth), math.cos(self.robot_theta + dth)
        )
        for tr in self.tracks.values():
            tr.x, tr.y = transform(tr.x, tr.y)
            tr.anchor_x, tr.anchor_y = transform(tr.anchor_x, tr.anchor_y)
        for candidate in self._candidates:
            candidate["x"], candidate["y"] = transform(candidate["x"], candidate["y"])
        self._pose_hist = deque(
            [
                (t, *transform(x, y), math.atan2(math.sin(th + dth), math.cos(th + dth)))
                for t, x, y, th in self._pose_hist
            ],
            maxlen=64,
        )
        self._proj_wide = [transform(x, y) for x, y in self._proj_wide]
        self._proj_body = [transform(x, y) for x, y in self._proj_body]
        self._corr_pairs.clear()

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
        if not self.mapping_enabled:
            self._prev_wide_base = []
            self._proj_wide = []
            return
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
        rpose = self._pose_at(msg.header.stamp)   # robot pose at IMAGE-CAPTURE time (anti-smear)
        self._corr_pairs.clear()
        wide_pts: list[tuple[float, float]] = []
        wide_base: list[tuple[float, float, str]] = []   # (bx, by, label) robot-frame, for object-flow
        for det in msg.detections:
            base = None   # base-frame projection; only set on the _wide_H path — init so the body-FOV
            #               skip check below (and the extrinsic fallback) never hits UnboundLocalError.
            u_center = float(det.x_center)
            v_center = float(det.y_center)
            v_bottom = v_center + float(det.height) * 0.5
            label = str(det.label)
            if self._wide_H is not None:
                # The wide cam is ~TOP-DOWN: a standing object leans radially from the image nadir,
                # so the box BOTTOM is NOT the ground contact — the box CENTRE is the best proxy for
                # the object's ground xy. (Body, being oblique/low, correctly uses the box bottom.)
                base = self._wide_pixel_to_base(u_center, v_center)
                if base is not None:
                    wide_base.append((base[0], base[1], label))
                xy = self._project_wide_H(u_center, v_center, rpose)
            else:
                # Extrinsic model intersects the object-CENTRE-height plane, so aim above the
                # bottom (quarter-down from centre) to hit that plane.
                xy = self._project_pixel(u_center, 0.5 * (v_center + v_bottom), rpose)
            if xy is None:
                continue
            wide_pts.append((xy[0], xy[1]))
            # NEAR field (inside the body sector): the BODY cam owns position AND identity — the
            # top-down fisheye is both position-inaccurate and mis-classifies shapes there (measured:
            # wide calls near cubes octa/icosa, lands 13-18cm off), which cross-matches and pulls
            # body tracks off their accurate spot. So skip wide MAP association here; the wide point
            # still feeds object-flow (below) and the viz overlay. Wide owns the FAR field only.
            if base is not None and self._in_body_fov_base(base[0], base[1]):
                continue
            set_type = _LABEL_TO_SET_TYPE.get(label, 0)
            self._associate(xy[0], xy[1], float(det.confidence), label, set_type, now, "wide")
        self._proj_wide = wide_pts
        # Object-flow odometry BEFORE the absolute correction: robot motion from how the raw
        # robot-frame points moved since the last wide frame (the vibration-proof yaw source).
        self._publish_object_flow(wide_base)
        self._maybe_publish_correction()   # wide frame sees many objects -> best rigid solve

    def on_body_detections(self, msg: DetectionArray) -> None:
        """Body-cam detections: project via the pick homography and fuse (body wins identity)."""
        if not self.mapping_enabled:
            self._proj_body = []
            return
        if not self.can_project_body:
            self.get_logger().warn(
                "body-cam detections received but homography unavailable; skipping",
                throttle_duration_sec=10.0,
            )
            return
        if not self.have_pose:
            return
        now = self._now_sec()
        rpose = self._pose_at(msg.header.stamp)   # robot pose at IMAGE-CAPTURE time (anti-smear)
        self._corr_pairs.clear()
        body_pts: list[tuple[float, float]] = []
        best_fruit = (-1.0, None)   # (conf, tid) of the PRIMARY fruit cube this frame
        for det in msg.detections:
            # Box CENTRE anchor (at object_center_height); _project_body_pixel height-corrects it to
            # the true ground centre. (The low body cam has strong height parallax, so anchoring on
            # the bottom gives the FRONT edge and disagrees with the wide's centre — this unifies.)
            u = float(det.x_center)
            v = float(det.y_center)
            xy = self._project_body_pixel(u, v, rpose)
            if xy is None:
                continue  # outside the homography's calibrated near-workspace
            body_pts.append((xy[0], xy[1]))
            label = str(det.label)
            set_type = _LABEL_TO_SET_TYPE.get(label, 0)
            tid = self._associate(xy[0], xy[1], float(det.confidence), label, set_type, now, "body")
            if tid != 0 and label == "fruit_photo_cube" and float(det.confidence) > best_fruit[0]:
                best_fruit = (float(det.confidence), tid)   # tid 0 = still a candidate (no track yet)
        # A following SigLIP result attaches to the HIGHEST-conf fruit cube (== the box SigLIP
        # itself cropped as its primary), so with several fruit cubes in view the fruit type lands
        # on the right one instead of whichever happened to be last in the list.
        if best_fruit[1] is not None:
            self._last_body_fruit_id = best_fruit[1]
            self._last_body_fruit_sec = now
        self._proj_body = body_pts
        self._maybe_publish_correction()

    def on_siglip(self, msg: Classification) -> None:
        """Attach the concrete fruit (SigLIP argmax) to the most-recent body fruit track.

        SigLIP does fruit-TYPE only; YOLO already established the box is a fruit_photo_cube.
        msg.confidence is the softmax MARGIN over the runner-up fruit (bigger = more reliable) and
        is used directly as the vote weight, so a clear winner dominates and a near-tie barely
        counts. Only record when a fruit FACE is actually visible (else it's a blank/edge face).
        """
        fruit = str(msg.label)
        if fruit not in _FRUIT_LABELS:
            return
        if not msg.image_face_visible:      # no printed fruit face in view -> don't guess a type
            return
        # Only attach if a body fruit-cube was detected RECENTLY: SigLIP runs a few frames behind the
        # body detector, but a result arriving long after any fruit box left view would otherwise bind
        # to a minutes-old _last_body_fruit_id (a different, possibly wrong, track). Stale -> drop.
        if (self._last_body_fruit_id is None
                or (self._now_sec() - self._last_body_fruit_sec) > self._fruit_attach_window):
            return
        tid = self._last_body_fruit_id
        tr = self.tracks.get(tid) if tid is not None else None
        if tr is None:
            return
        # Vote across faces/frames, weighted by the margin (reliability). No large floor: a low-
        # margin reading adds almost nothing, so the map stays type-unknown until a face is read.
        tr.fruit_votes[fruit] = tr.fruit_votes.get(fruit, 0.0) + max(0.0, float(msg.confidence))
        tr.fruit_confidence = max(tr.fruit_confidence, float(msg.confidence))
        tr.set_type = 2
        self._refresh_identity(tr)

    def _project_body_pixel(self, u: float, v: float, pose=None) -> tuple[float, float] | None:
        """Body-cam pixel -> field xy. Prefers the base_link-metres ground H (aruco_calib);
        else the arm_base-cm pick H (+offset). None if out of the near workspace.
        `pose` = (x, y, theta) at IMAGE-CAPTURE TIME (from _pose_at); None -> latest pose."""
        if not self.can_project_body or not self.have_pose:
            return None
        # Body cam may be GPU-downscaled; upscale the detection pixel back to the resolution the
        # body homographies were calibrated at (1640x1232) before transforming. Scale=1 if not downscaled.
        pt = np.array([[[float(u) * self.body_px_scale_x, float(v) * self.body_px_scale_y]]],
                      dtype=np.float64)
        if self._body_ground_H is not None:
            out = cv2.perspectiveTransform(pt, self._body_ground_H)[0][0]
            bx, by = float(out[0]), float(out[1])           # base_link METRES directly
        else:
            out = cv2.perspectiveTransform(pt, self._body_H)[0][0]
            x_ab, y_ab = float(out[0]) / 100.0, float(out[1]) / 100.0   # arm_base cm -> m
            bx = self.arm_base_off_x + x_ab
            by = self.arm_base_off_y + y_ab
        # Height-correct the box-centre anchor to the object's true ground centre (body nadir/height).
        bx, by = self._height_correct(bx, by, self.body_cam_nadir_x, self.body_cam_nadir_y, self.body_cam_height)
        # Front-face bias trim: push radially OUT from the body nadir so it matches the wide centroid.
        if self.body_radial_trim != 0.0:
            dx, dy = bx - self.body_cam_nadir_x, by - self.body_cam_nadir_y
            d = math.hypot(dx, dy)
            if d > 0.01:
                bx += self.body_radial_trim * dx / d
                by += self.body_radial_trim * dy / d
        x0, x1, y0, y1 = self.body_ws
        if not (x0 <= bx <= x1 and y0 <= by <= y1):          # gate in base_link frame
            return None
        # base_link -> field (rotate by heading, translate by robot xy) at CAPTURE-time pose.
        rx, ry, rth = pose if pose is not None else (self.robot_x, self.robot_y, self.robot_theta)
        ct, st = math.cos(rth), math.sin(rth)
        return (rx + bx * ct - by * st, ry + bx * st + by * ct)

    # ------------------------------------------------------------- camera FOV (for negative evidence)
    def _to_base(self, fx: float, fy: float) -> tuple[float, float]:
        """Field xy -> base_link xy (inverse of the base->field robot transform)."""
        dx, dy = fx - self.robot_x, fy - self.robot_y
        ct, st = math.cos(self.robot_theta), math.sin(self.robot_theta)
        return (ct * dx + st * dy, -st * dx + ct * dy)

    def _in_body_fov(self, fx: float, fy: float) -> bool:
        """Inside the body cam's forward sector (부채꼴): within +-half angle and [near,far] range."""
        return self._in_body_fov_base(*self._to_base(fx, fy))

    def _in_body_fov_base(self, bx: float, by: float) -> bool:
        """Body-cam forward sector test in the base_link frame (pose-independent)."""
        dx = bx - self.body_fov_apex_x
        d = math.hypot(dx, by)
        if not (self.body_fov_near <= d <= self.body_fov_far):
            return False
        return abs(math.atan2(by, dx)) <= self.body_fov_half

    def _in_wide_fov(self, fx: float, fy: float) -> bool:
        """Inside the wide fisheye's forward-biased ground ellipse."""
        bx, by = self._to_base(fx, fy)
        return ((bx - self.wide_fov_cx) / self.wide_fov_fwd) ** 2 + (by / self.wide_fov_lat) ** 2 <= 1.0

    # ------------------------------------------------------------------- tracker

    def _occupied_grid_ids(self) -> set[int]:
        """Grid points still claimed by live, unmoved tracks."""
        return {
            tr.spawn_grid_id
            for tr in self.tracks.values()
            if (tr.spawn_grid_id >= 0 and tr.grid_state == "grid_spawned"
                and not tr.blacklisted)
        }

    def _near_zone_anchor(self) -> bool:
        return any(
            math.hypot(self.robot_x - ax, self.robot_y - ay) <= self.grid_anchor_pose_radius
            for ax, ay in self.zone_anchors
        )

    def _apply_grid_prior(
        self, x: float, y: float, set_type: int, now: float
    ) -> tuple[float, float, int, bool, float]:
        """Soft-snap a confirmed NEW object to the nearest free field-grid point.

        Set1/Set2 game objects are eligible. Arrival landmarks and unknown classes are kept at
        their observed positions because they are not part of the 28 grid-spawned objects.
        """
        if not self.grid_prior_enabled or set_type not in (1, 2) or not self._grid_points:
            return x, y, -1, False, math.inf

        gid, (gx, gy) = min(
            enumerate(self._grid_points),
            key=lambda item: math.hypot(item[1][0] - x, item[1][1] - y),
        )
        distance = math.hypot(gx - x, gy - y)
        initial = (now - self._grid_start_sec) <= self.grid_initial_phase
        if self._near_zone_anchor():
            radius = max(self.grid_new_radius, self.grid_anchor_radius)
            alpha = max(self.grid_new_alpha, self.grid_anchor_alpha)
        else:
            radius = self.grid_initial_radius if initial else self.grid_new_radius
            alpha = self.grid_initial_alpha if initial else self.grid_new_alpha
        if distance > radius or gid in self._occupied_grid_ids():
            return x, y, -1, False, distance
        return (
            (1.0 - alpha) * x + alpha * gx,
            (1.0 - alpha) * y + alpha * gy,
            gid,
            True,
            distance,
        )

    def _update_grid_state(self, tr: Track, observed_x: float, observed_y: float) -> None:
        """Release a spawn grid after an existing object has physically moved away from it."""
        if tr.spawn_grid_id < 0 or tr.grid_state != "grid_spawned":
            return
        gx, gy = self._grid_points[tr.spawn_grid_id]
        distance = math.hypot(observed_x - gx, observed_y - gy)
        if distance <= self.grid_moved_threshold:
            tr.current_grid_id = tr.spawn_grid_id
            return
        tr.current_grid_id = -1
        tr.grid_state = "moved"
        if self.grid_prior_debug:
            self.get_logger().info(
                f"grid moved: track={tr.id} spawn_grid={tr.spawn_grid_id} distance={distance:.3f}m"
            )

    def _lock_track_to_grid(self, tr: Track, set_type: int) -> bool:
        """Keep a live game-object track at its claimed grid point during sweep tests.

        This stabilizes the object map when robot pose jitter would otherwise drag static objects
        around. It is intentionally separate from landmark correction: the robot pose can still
        move, but the object layer stays grid-centered.
        """
        if not self.grid_prior_enabled or not self.grid_track_lock_enabled:
            return False
        if set_type not in (1, 2) or not self._grid_points:
            return False
        occupied = self._occupied_grid_ids()
        gid = tr.spawn_grid_id if tr.spawn_grid_id >= 0 and tr.grid_state == "grid_spawned" else -1
        if gid < 0:
            gid, (gx, gy) = min(
                enumerate(self._grid_points),
                key=lambda item: math.hypot(item[1][0] - tr.x, item[1][1] - tr.y),
            )
            if math.hypot(gx - tr.x, gy - tr.y) > self.grid_track_lock_radius:
                return False
            if gid in occupied:
                return False
            tr.spawn_grid_id = gid
            tr.grid_state = "grid_spawned"
            tr.grid_snapped = True
        else:
            gx, gy = self._grid_points[gid]
        occupied.discard(tr.spawn_grid_id)
        if gid in occupied:
            return False
        alpha = self.grid_track_lock_alpha
        tr.x = (1.0 - alpha) * tr.x + alpha * gx
        tr.y = (1.0 - alpha) * tr.y + alpha * gy
        tr.current_grid_id = gid
        tr.grid_state = "grid_spawned"
        tr.grid_snapped = True
        if tr.locked:
            tr.anchor_x = tr.x
            tr.anchor_y = tr.y
        return True

    def _candidate_hit(self, x: float, y: float, conf: float, label: str, set_type: int,
                       now: float, is_body: bool, vote_thresh: float) -> int:
        """A detection with no matching track: hold it as a CANDIDATE and only spawn a real track once
        it has been re-observed new_track_min_hits times at ~the same spot. Returns the new track id on
        promotion, else 0 (no track yet). Kills phantom over-creation from motion jitter."""
        self._candidates = [c for c in self._candidates if now - c["t"] <= self.candidate_ttl]
        best = None
        bd = self.assoc_radius
        for c in self._candidates:
            d = math.hypot(c["x"] - x, c["y"] - y)
            if d <= bd:
                bd = d
                best = c
        if best is None:
            fruit_hit = self._is_fruit_cube_evidence(label, conf, is_body)
            fruit_body_hits = 1 if fruit_hit and is_body else 0
            fruit_wide_hits = 1 if fruit_hit and not is_body else 0
            fruit_seen = self._fruit_cube_sticky_from_hits(
                body_hits=fruit_body_hits,
                wide_hits=fruit_wide_hits,
            )
            self._candidates.append({"x": x, "y": y, "n": 1, "t": now, "conf": conf,
                                     "label": label,
                                     "set": (2 if fruit_seen else set_type),
                                     "body": is_body,
                                     "fruit_cube_seen": fruit_seen,
                                     "fruit_cube_conf": conf if fruit_hit else 0.0,
                                     "fruit_cube_wide_hits": fruit_wide_hits,
                                     "fruit_cube_body_hits": fruit_body_hits})
            return 0
        best["x"] = 0.5 * best["x"] + 0.5 * x
        best["y"] = 0.5 * best["y"] + 0.5 * y
        best["n"] += 1
        best["t"] = now
        best["conf"] = max(best["conf"], conf)
        best["body"] = best["body"] or is_body
        if self._is_fruit_cube_evidence(label, conf, is_body):
            if is_body:
                best["fruit_cube_body_hits"] = int(best.get("fruit_cube_body_hits", 0)) + 1
            else:
                best["fruit_cube_wide_hits"] = int(best.get("fruit_cube_wide_hits", 0)) + 1
            best["fruit_cube_conf"] = max(float(best.get("fruit_cube_conf", 0.0)), conf)
            if self._fruit_cube_sticky_from_hits(
                    body_hits=int(best.get("fruit_cube_body_hits", 0)),
                    wide_hits=int(best.get("fruit_cube_wide_hits", 0))):
                best["fruit_cube_seen"] = True
                best["set"] = 2
        if best["n"] < self.new_track_min_hits:
            return 0
        # Confirmed -> promote to a real track.
        tid = self._next_id
        self._next_id += 1
        body = bool(best["body"])
        raw_x, raw_y = float(best["x"]), float(best["y"])
        track_x, track_y, grid_id, snapped, snap_distance = self._apply_grid_prior(
            raw_x, raw_y, int(best["set"]), now
        )
        tr = Track(
            id=tid, x=track_x, y=track_y, confidence=best["conf"], last_seen_sec=now,
            source=("body" if body else "wide"), seen_body=body,
            last_body_sec=(now if is_body else 0.0), last_wide_sec=(0.0 if is_body else now),
            n_obs=best["n"], n_body=(best["n"] if body else 0),
            spawn_grid_id=grid_id, current_grid_id=grid_id,
            grid_state=("grid_spawned" if snapped else "off_grid"), grid_snapped=snapped,
            fruit_cube_seen=bool(best.get("fruit_cube_seen", False)),
            fruit_cube_confidence=float(best.get("fruit_cube_conf", 0.0)),
            fruit_cube_wide_hits=int(best.get("fruit_cube_wide_hits", 0)),
            fruit_cube_body_hits=int(best.get("fruit_cube_body_hits", 0)),
        )
        if tr.fruit_cube_seen:
            tr.set_type = 2
        if conf >= vote_thresh:
            self._vote(tr, label, conf, is_body)
        self._refresh_identity(tr)
        self.tracks[tid] = tr
        self._candidates.remove(best)
        if self.grid_prior_debug:
            nearest = "none" if not math.isfinite(snap_distance) else f"{snap_distance:.3f}m"
            self.get_logger().info(
                f"grid new track={tid} raw=({raw_x:.3f},{raw_y:.3f}) "
                f"map=({track_x:.3f},{track_y:.3f}) grid={grid_id} "
                f"nearest_distance={nearest} snapped={snapped}"
            )
        return tid

    def _associate(
        self, x: float, y: float, conf: float, label: str, set_type: int, now: float, source: str
    ) -> int:
        """Associate a projected detection to the nearest track (or create one). Returns its id.

        `source` is "wide" or "body". Body observations are closer/more reliable, so they pull
        position harder and win on class/set_type; a wide detection never overwrites an identity
        the body cam has already established, and neither clobbers a concrete SigLIP fruit name.
        """
        is_body = source == "body"
        vote_thresh = self.class_conf_threshold_body if is_body else self.class_conf_threshold
        # Pure nearest-neighbour within assoc_radius. Distinct objects are kept apart by DISTANCE
        # (assoc_radius tightened for the cm-accurate 1280 wide) — NOT by label, because the same
        # object often gets a wrong wide class + a correct body class, and those must still merge
        # (body then wins identity). Label-based splitting would duplicate that object on the map.
        # Match to an existing track with the GENEROUS re-association gate so a re-seen object (jittered
        # or missed a few frames) sticks to its track instead of duplicating. New-track creation below
        # still uses the tight assoc_radius (via the candidate mechanism), so distinct objects stay apart.
        best_id: int | None = None
        best_d = self.reassoc_radius
        for tid, tr in self.tracks.items():
            d = math.hypot(tr.x - x, tr.y - y)
            if d <= best_d:
                best_d = d
                best_id = tid

        if best_id is None:
            # No existing track within reassoc_radius -> genuinely new spot. DON'T spawn from a single
            # detection (jitter/blur -> phantoms). Require new_track_min_hits consistent re-obs.
            return self._candidate_hit(x, y, conf, label, set_type, now, is_body, vote_thresh)

        # Fuse into the matched track.
        tr = self.tracks[best_id]
        if self._is_fruit_cube_evidence(label, conf, is_body):
            if is_body:
                tr.fruit_cube_body_hits += 1
            else:
                tr.fruit_cube_wide_hits += 1
            tr.fruit_cube_confidence = max(tr.fruit_cube_confidence, conf)
            if self._fruit_cube_sticky_from_hits(
                    body_hits=tr.fruit_cube_body_hits,
                    wide_hits=tr.fruit_cube_wide_hits):
                tr.fruit_cube_seen = True
                tr.set_type = 2
        self._lock_track_to_grid(tr, set_type)
        if not self.grid_track_lock_enabled:
            self._update_grid_state(tr, x, y)
        if tr.locked and self.landmark_correction:
            # Frozen landmark: don't move it — record (track id, fresh obs, anchor) so the batch
            # solve can recover the robot-pose drift AND spot anchors that moved (object picked up).
            self._corr_pairs.append((best_id, x, y, tr.anchor_x, tr.anchor_y))
        else:
            pos_a = 0.7 if is_body else self.conf_ema     # body pulls position harder
            tr.x = (1.0 - pos_a) * tr.x + pos_a * x
            tr.y = (1.0 - pos_a) * tr.y + pos_a * y
            if self.grid_prior_enabled and self._near_zone_anchor() and set_type in (1, 2) and self._grid_points:
                gid, (gx, gy) = min(
                    enumerate(self._grid_points),
                    key=lambda item: math.hypot(item[1][0] - tr.x, item[1][1] - tr.y),
                )
                dist = math.hypot(gx - tr.x, gy - tr.y)
                occupied = self._occupied_grid_ids()
                occupied.discard(tr.spawn_grid_id)
                if dist <= self.grid_anchor_radius and gid not in occupied:
                    tr.x = (1.0 - self.grid_anchor_alpha) * tr.x + self.grid_anchor_alpha * gx
                    tr.y = (1.0 - self.grid_anchor_alpha) * tr.y + self.grid_anchor_alpha * gy
                    tr.current_grid_id = gid
                    if tr.spawn_grid_id < 0:
                        tr.spawn_grid_id = gid
                        tr.grid_state = "grid_spawned"
                        tr.grid_snapped = True
            self._lock_track_to_grid(tr, set_type)
        new_conf = (1.0 - self.conf_ema) * tr.confidence + self.conf_ema * conf
        if is_body:
            tr.confidence = max(tr.confidence, conf)  # body = quality authority: sticky to its confident look
            tr.seen_body = True
            tr.last_body_sec = now
            tr.n_body += 1
        elif tr.seen_body:
            tr.confidence = max(tr.confidence, new_conf)  # wide may REINFORCE a body track, never erode it
            tr.last_wide_sec = now
        else:
            tr.confidence = new_conf                      # wide-only: plain EMA
            tr.last_wide_sec = now
        tr.n_obs += 1
        tr.last_seen_sec = now
        if source not in tr.source:
            tr.source = "wide+body" if tr.source else source
        # Position/presence updated above for ANY detection; identity only from CONFIDENT ones.
        if conf >= vote_thresh:
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
        """Add one confidence-weighted class vote into this source's pool."""
        if not label:
            return
        pool = tr.body_votes if is_body else tr.wide_votes
        pool[label] = pool.get(label, 0.0) + max(0.05, conf)

    def _is_fruit_cube_evidence(self, label: str, conf: float, is_body: bool) -> bool:
        if not self.fruit_cube_sticky_enabled or label != "fruit_photo_cube":
            return False
        thresh = self.fruit_cube_sticky_conf_body if is_body else self.fruit_cube_sticky_conf_wide
        return conf >= thresh

    def _fruit_cube_sticky_from_hits(self, *, body_hits: int, wide_hits: int) -> bool:
        return bool(
            self.fruit_cube_sticky_enabled
            and (int(body_hits) > 0
                 or int(wide_hits) >= max(1, self.fruit_cube_sticky_min_wide_hits))
        )

    @staticmethod
    def _refresh_identity(tr: Track) -> None:
        """Best-estimate identity. SigLIP fruit wins for Set2; otherwise the BODY cam's vote
        decides whenever it has classified the object (close/reliable), falling back to the wide
        cam only for objects the body never saw. Within the chosen source, argmax of the
        confidence-summed votes (so YOLO confidence drives the pick)."""
        if tr.fruit_votes:
            tr.class_label = max(tr.fruit_votes, key=tr.fruit_votes.get)
            tr.fruit_label = tr.class_label
            tr.set_type = 2
            return
        arrival = tr.body_votes.get("arrival", 0.0) + tr.wide_votes.get("arrival", 0.0)
        if arrival > 0.0:
            tr.class_label = "arrival"
            tr.set_type = 3
            return
        # Fruit-photo evidence is sticky: once a track has one sufficiently confident
        # fruit_photo_cube box, later plain cube votes must not demote it back to Set1.
        if tr.fruit_cube_seen:
            tr.class_label = "fruit_photo_cube"
            tr.set_type = 2
            return
        pool = tr.body_votes if tr.body_votes else tr.wide_votes
        if pool:
            # fruit_photo_cube handled above; pick the best SHAPE vote for a Set1 object.
            shapes = {k: v for k, v in pool.items() if k not in ("fruit_photo_cube", "arrival")}
            if shapes:
                best = max(shapes, key=shapes.get)
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

        # Confidence of this heading fix: more inliers + well-spread anchors + low residual => this
        # is a trustworthy ABSOLUTE heading (YOLO bearings are sharp), so the localizer can apply
        # most of it rather than a tiny nudge. Clustered / few / high-residual anchors -> low conf.
        resid_k = [math.hypot((c * p[1] - s * p[2] + tcx) - p[3], (s * p[1] + c * p[2] + tcy) - p[4])
                   for p in kept]
        mean_resid = sum(resid_k) / len(resid_k)
        lxc = sum(p[3] for p in kept) / len(kept)
        lyc = sum(p[4] for p in kept) / len(kept)
        spread = sum(math.hypot(p[3] - lxc, p[4] - lyc) for p in kept) / len(kept)
        n_factor = min(1.0, (len(kept) - 2) / 4.0)                     # 3 anchors -> .25, 6 -> 1
        resid_factor = max(0.0, 1.0 - mean_resid / max(1e-6, self.landmark_max_resid))
        spread_factor = min(1.0, spread / max(1e-6, self.landmark_spread_ref))
        conf = max(0.0, n_factor * resid_factor * spread_factor)

        # Heading gate: only trust the rotation term when the anchor geometry constrains it
        # (enough inliers AND wide spread). Otherwise keep translation, zero the heading — the
        # sparse/tight case gives a noisy Umeyama theta that was jittering the robot's yaw.
        heading_ok = (len(kept) >= self.landmark_heading_min_pairs
                      and spread >= self.landmark_heading_min_spread)
        if heading_ok:
            dth_out, dx_out, dy_out = dth, dx, dy
        else:
            # theta_c is ill-conditioned here, so dx,dy (which fold in (R-I)(p-centroid)) carry its
            # noise amplified by the robot->anchor lever arm. Recompute the correction as a PURE mean
            # displacement (R = I): dx,dy = centroid(anchors) - centroid(observations). This stays
            # robust with only 3 clustered anchors and matches the "translation is trustworthy even
            # when rotation is not" intent, instead of leaking a fake rotation into position.
            oxc = sum(p[1] for p in kept) / len(kept)
            oyc = sum(p[2] for p in kept) / len(kept)
            dth_out, dx_out, dy_out = 0.0, lxc - oxc, lyc - oyc

        m = Float32MultiArray()
        m.data = [float(dx_out), float(dy_out), float(dth_out), float(conf)]
        self.pub_corr.publish(m)
        n_locked = sum(1 for t in self.tracks.values() if t.locked)
        self.get_logger().info(
            f"landmark correction dx={dx_out:+.3f} dy={dy_out:+.3f} dth={dth_out:+.3f}"
            f"{'' if heading_ok else '(heading gated)'} conf={conf:.2f} "
            f"(inliers~{len(kept)}/{len(pairs)} spread={spread:.2f}m, {n_locked} locked)",
            throttle_duration_sec=2.0,
        )

    def _publish_object_flow(self, curr_base: list[tuple[float, float, str]]) -> None:
        """Frame-to-frame odometry from the wide cam's robot-frame object points.

        Match this frame's points to the previous frame's (nearest-neighbour; points barely move
        between frames, and a same-label match is preferred to break ties), then fit the rigid
        transform. A static world seen from a rotating/translating robot moves rigidly in the robot
        frame, so this recovers the robot's OWN per-frame motion — from real objects, so it reports
        ~0 when the robot is still even while the motors buzz (unlike dense LK flow, which sees the
        vibration as rotation). Published [dtheta, dfwd, dleft, conf] in the robot frame.
        """
        prev = self._prev_wide_base
        self._prev_wide_base = curr_base
        if not self.object_flow or len(prev) < self.object_flow_min_pairs or len(curr_base) < self.object_flow_min_pairs:
            return
        gate = self.object_flow_assoc
        pairs = []   # (px, py, qx, qy)
        used = [False] * len(prev)
        for qx, qy, qlbl in curr_base:
            best = -1
            best_d = gate
            for i, (px, py, plbl) in enumerate(prev):
                if used[i]:
                    continue
                d = math.hypot(qx - px, qy - py)
                if plbl == qlbl:
                    d *= 0.6   # same shape -> prefer this pairing (body/wide labels aid association)
                if d < best_d:
                    best_d = d
                    best = i
            if best >= 0:
                used[best] = True
                px, py, _ = prev[best]
                pairs.append((px, py, qx, qy))
        if len(pairs) < self.object_flow_min_pairs:
            return
        # Solve prev->curr, trim mismatches once by residual, re-solve.
        phi, tx, ty = self._umeyama_2d(pairs)
        if phi is None:
            return
        for _ in range(2):
            c, s = math.cos(phi), math.sin(phi)
            resid = [math.hypot((c * p[0] - s * p[1] + tx) - p[2], (s * p[0] + c * p[1] + ty) - p[3]) for p in pairs]
            kept = [p for p, r in zip(pairs, resid) if r <= gate * 0.5]
            if len(kept) < self.object_flow_min_pairs or len(kept) == len(pairs):
                break
            pairs = kept
            phi, tx, ty = self._umeyama_2d(pairs)
            if phi is None:
                return
        c, s = math.cos(phi), math.sin(phi)
        resid = [math.hypot((c * p[0] - s * p[1] + tx) - p[2], (s * p[0] + c * p[1] + ty) - p[3]) for p in pairs]
        mean_resid = sum(resid) / len(resid)
        # Robot per-frame delta (robot frame): world rotates by phi in the robot frame => robot
        # yawed by -phi; translation likewise inverts (first order for small per-frame motion).
        dtheta = -phi
        dfwd, dleft = -tx, -ty
        if abs(dtheta) > self.object_flow_max_dtheta:
            return   # implausible per-frame jump -> bad association, drop this frame
        # Sub-cm per-frame translation is match noise, not motion — zero it so a stationary robot
        # does NOT random-walk away (drift). Real driving exceeds this each frame.
        if math.hypot(dfwd, dleft) < self.object_flow_trans_deadband:
            dfwd = dleft = 0.0
        n_factor = min(1.0, (len(pairs) - 2) / 5.0)                 # 4 pts -> .4, 7 -> 1
        resid_factor = max(0.0, 1.0 - mean_resid / max(1e-6, gate * 0.5))
        conf = max(0.0, n_factor * resid_factor)
        m = Float32MultiArray()
        m.data = [float(dtheta), float(dfwd), float(dleft), float(conf)]
        self.pub_odom.publish(m)
        self.get_logger().info(
            f"object-flow dθ={math.degrees(dtheta):+5.1f}° dfwd={dfwd*100:+.0f} dleft={dleft*100:+.0f}cm "
            f"conf={conf:.2f} ({len(pairs)} pts, resid={mean_resid*100:.1f}cm)",
            throttle_duration_sec=1.0,
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

    def _apply_negative_evidence(self, now: float) -> None:
        """Penalise tracks inside a camera's FOV that were NOT seen recently. If an object should be
        visible there and isn't, its presence belief decays -> eventually dropped (clears phantoms
        and objects picked up / moved away). Only an ACTIVE camera penalises (a disabled one never
        does), and the wide cam's disk models its all-around fisheye view."""
        if not self.have_pose:
            return
        for tr in self.tracks.values():
            if tr.blacklisted:
                continue
            miss = 0
            in_body = self._in_body_fov(tr.x, tr.y)
            if (self.can_project_body and in_body
                    and (now - tr.last_body_sec) > self.see_window):
                miss += 1
            # Wide miss only counts OUTSIDE the body sector: on_detections deliberately drops wide
            # associations inside that sector (body owns the near field), so last_wide_sec is never
            # refreshed there even though the wide cam IS seeing the object — counting a wide miss
            # then would double-penalise a fully-visible near track and delete it. Body owns presence
            # in its sector.
            if (self.can_project and not in_body and self._in_wide_fov(tr.x, tr.y)
                    and (now - tr.last_wide_sec) > self.see_window):
                miss += 1
            if miss:
                tr.confidence = max(
                    0.0, tr.confidence - self.miss_penalty_per_sec * miss * self.tick_dt
                )

    def _forget_stale(self, now: float) -> None:
        # A LOCKED anchor is the field's absolute position reference (in test_field there are no
        # walls, so anchors are the ONLY absolute fix). Exempt it from the TIME-based forget so the
        # whole-field map persists while the robot is elsewhere >forget_after — but still drop it on
        # confidence collapse (miss-penalty below min_keep_conf), which is how a picked/removed anchor
        # is retired. Unlock-on-move (unlock_after) already handles anchors that physically shifted.
        stale = [
            tid
            for tid, tr in self.tracks.items()
            if not tr.blacklisted and (
                (not tr.locked and (now - tr.last_seen_sec) > self.forget_after)
                or tr.confidence < self.min_keep_conf
            )
        ]
        for tid in stale:
            del self.tracks[tid]
        if stale:
            self.get_logger().info(f"forgot {len(stale)} stale/low-presence track(s)")

    def _merge_duplicates(self) -> None:
        """Collapse tracks that sit within assoc_radius of each other into ONE.

        _associate is a pure per-detection greedy NN with no frame-level 1:1 constraint, so
        projection jitter can split a single object across two ids (and a duplicate never re-merges
        on its own). A stray duplicate/phantom is a top scoring risk (duplicate/phantom penalty) and
        would let the FSM re-approach and re-count the same object after it was picked+blacklisted.
        Merge the NEWER track into the OLDER (kept) id: fuse vote pools, OR the flags, max confidence,
        sum evidence counts. Two CONFIRMED (locked) anchors with different concrete classes are kept
        apart (they are genuinely distinct objects, not a jitter split)."""
        ids = sorted(self.tracks.keys())   # ascending id = oldest first (kept)
        removed: set[int] = set()
        for i, a in enumerate(ids):
            if a in removed:
                continue
            ta = self.tracks.get(a)
            if ta is None:
                continue
            for b in ids[i + 1:]:
                if b in removed:
                    continue
                tb = self.tracks.get(b)
                if tb is None:
                    continue
                if math.hypot(ta.x - tb.x, ta.y - tb.y) > self.reassoc_radius:
                    continue   # generous gate: collapse jitter-split duplicates (still < grid spacing)
                if (ta.locked and tb.locked and ta.class_label and tb.class_label
                        and ta.class_label != tb.class_label
                        and not (ta.fruit_cube_seen or tb.fruit_cube_seen)):
                    continue   # two confirmed, differently-classified anchors -> keep distinct
                for pa, pb in ((ta.wide_votes, tb.wide_votes),
                               (ta.body_votes, tb.body_votes),
                               (ta.fruit_votes, tb.fruit_votes)):
                    for k, v in pb.items():
                        pa[k] = pa.get(k, 0.0) + v
                ta.confidence = max(ta.confidence, tb.confidence)
                ta.fruit_confidence = max(ta.fruit_confidence, tb.fruit_confidence)
                ta.fruit_cube_wide_hits += tb.fruit_cube_wide_hits
                ta.fruit_cube_body_hits += tb.fruit_cube_body_hits
                ta.fruit_cube_seen = ta.fruit_cube_seen or tb.fruit_cube_seen
                if self._fruit_cube_sticky_from_hits(
                        body_hits=ta.fruit_cube_body_hits,
                        wide_hits=ta.fruit_cube_wide_hits):
                    ta.fruit_cube_seen = True
                ta.fruit_cube_confidence = max(ta.fruit_cube_confidence, tb.fruit_cube_confidence)
                ta.n_obs += tb.n_obs
                ta.n_body += tb.n_body
                ta.seen_body = ta.seen_body or tb.seen_body
                ta.blacklisted = ta.blacklisted or tb.blacklisted
                ta.last_seen_sec = max(ta.last_seen_sec, tb.last_seen_sec)
                ta.last_body_sec = max(ta.last_body_sec, tb.last_body_sec)
                ta.last_wide_sec = max(ta.last_wide_sec, tb.last_wide_sec)
                if ta.spawn_grid_id < 0 and tb.spawn_grid_id >= 0:
                    ta.spawn_grid_id = tb.spawn_grid_id
                    ta.current_grid_id = tb.current_grid_id
                    ta.grid_state = tb.grid_state
                    ta.grid_snapped = tb.grid_snapped
                if tb.locked and not ta.locked:   # inherit the confirmed anchor position
                    ta.locked = True
                    ta.anchor_x, ta.anchor_y, ta.x, ta.y = tb.anchor_x, tb.anchor_y, tb.x, tb.y
                if self._last_body_fruit_id == b:
                    self._last_body_fruit_id = a
                self._refresh_identity(ta)
                removed.add(b)
                del self.tracks[b]
        if removed:
            self.get_logger().info(f"merged {len(removed)} duplicate track(s)")

    # ------------------------------------------------------------------- publish

    def tick(self) -> None:
        now = self._now_sec()
        if self.mapping_enabled:
            self._merge_duplicates()
            self._apply_negative_evidence(now)
            self._forget_stale(now)
        else:
            self._clear_mapping_state()

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

        # Per-camera RAW projections (field xy) for the viz overlay: flat [x,y,src,...] (src 0=wide,1=body).
        proj = Float32MultiArray()
        pdata: list[float] = []
        for (x, y) in self._proj_wide:
            pdata += [float(x), float(y), 0.0]
        for (x, y) in self._proj_body:
            pdata += [float(x), float(y), 1.0]
        proj.data = pdata
        self.pub_proj.publish(proj)

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
