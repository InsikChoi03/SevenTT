"""Mission FSM for the AI Robot Challenge.

States:
    SCAN              - top cam world model build/update
    ZONE_STABILIZE    - stop at the active zone anchor and trust stationary observations
    SELECT_TARGET     - pick next object via /selected_target
    APPROACH          - drive base toward target
    ALIGN             - body cam visual servo
    CLASSIFY          - SigLIP gate (+ shape heuristic for set1)
    PICK              - arm pickup motion
    STORE_IN_TRAY     - place in body tray, increment counters
    DRIVE_TO_STORAGE  - move to storage zone
    ALIGN_OVER_BIN    - align over storage box
    DUMP_ALL          - tilt/release tray
    END               - terminal

Each periodic tick evaluates condition-driven transitions using the latest
world model / selected target / classification messages, and drives the base
(/base_command) and arm (/arm/pick_trigger). A manual /state_advance trigger
still force-advances one transition for debug/dry-run exercising.

Pick-gate (rulebook §6/§7, mispick on Set2 = -40, so be conservative):
  - Set2 fruit: siglip == today's fruit + image_face_visible + conf >= thresh -> PICK.
  - Set1 cube : shape == today's cube  + NOT image_face_visible + conf >= thresh -> PICK.
  - otherwise (or classify timeout) -> PASS (blacklist, reselect).
"""
from __future__ import annotations

import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from robot_interfaces.msg import BaseCommand, Classification, DetectionArray, MissionState, Object, WorldModel
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int8, String, UInt64

from robot_planning.lane_planner import LanePlanner
from robot_planning.object_slot_inventory import ObjectSlot, SlotInventory

# Body detection label -> set_type (mirror of world_model), for the ALIGN visual-servo filter.
_LABEL_ST = {"cube": 1, "octahedron": 1, "dodecahedron": 1, "icosahedron": 1, "fruit_photo_cube": 2}


STATES = [
    "OPENING", "SCAN", "ZONE_STABILIZE", "SELECT_TARGET", "APPROACH", "ALIGN", "CLASSIFY", "PICK",
    "STORE_IN_TRAY", "DRIVE_TO_STORAGE", "ALIGN_OVER_BIN", "DUMP_ALL", "END",
]


class MissionFsmNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_fsm_node")

        # Today's announced targets (rulebook §6.1, §7.3).
        self.declare_parameter("set1_label", "")      # e.g. "icosahedron"
        self.declare_parameter("set2_label", "")      # e.g. "apple"
        self.declare_parameter("start_phase", 1)       # 1=Set1 first, 2=Set2/fruit cube test first
        self.declare_parameter("mixed_target_mode", False)
        self.declare_parameter("conf_threshold", 0.7)
        self.declare_parameter("pick_track_conf", 0.5)   # min world-model track confidence to pick the latched target
        self.declare_parameter("approach_dist_m", 0.525)
        self.declare_parameter("align_settle_sec", 1.0)
        self.declare_parameter("classify_timeout_sec", 2.0)
        # ALIGN visual servo: drive the mecanum base so the target lands on the arm's fixed grab
        # point (base_link m, measured 2026-07-04: 26.4cm fwd / 0.7cm left). SAFETY: never advance
        # the target past grab_min_x — the body cam blind-limit (~25cm) is only ~1cm nearer, so an
        # overshoot loses sight of the object right before the grab.
        self.declare_parameter("grab_x", 0.264)
        self.declare_parameter("grab_y", 0.007)
        self.declare_parameter("grab_min_x", 0.255)     # never push the target closer than this
        self.declare_parameter("align_tol_m", 0.04)     # aligned within this -> grab (gripper absorbs)
        self.declare_parameter("align_fwd_tol_m", 0.08)  # forward/back tolerance; lateral still uses align_tol_m
        self.declare_parameter("align_kp", 0.6)         # m/s per m of error
        self.declare_parameter("align_vmax", 0.16)      # cap
        # STICTION: the heavy base won't move below ~this speed (motor just buzzes), so any nonzero
        # servo command is boosted to at least this. Bigger tol above absorbs the coarser steps.
        self.declare_parameter("align_vmin", 0.13)
        self.declare_parameter("align_timeout_sec", 12.0)
        self.declare_parameter("max_align_fails", 3)   # consecutive ALIGN timeouts on an object -> blacklist it
        # Pulse+settle: the heavy base coasts after a command, so instead of a continuous servo we
        # nudge briefly, let it FULLY STOP (inertia dissipates), then measure the settled position
        # and grab if within tolerance (gripper opening absorbs the residual). Repeat otherwise.
        self.declare_parameter("align_pulse_sec", 0.15)     # (legacy) burst length per nudge
        self.declare_parameter("align_settle_pulse_sec", 0.8)  # wait for the base to fully stop
        # UNIT-STEP servo: each nudge is a CALIBRATED fixed pulse that moves a near-constant ~1.5-2.5 cm
        # (measured 2026-07-08, no boost, brake on). We step ONE axis (the worst) per cycle toward the
        # body-cam target, re-measure, repeat -> repeatable & no proportional hunting on pose noise.
        # duty/sec per direction (strafe needs ~1.7x duty & 1.5x time = higher lateral stiction):
        self.declare_parameter("align_step_fwd_duty", 0.18)     # fwd/back: 0.18/0.20s -> ~1.5-2 cm
        self.declare_parameter("align_step_fwd_sec", 0.15)
        self.declare_parameter("align_step_strafe_duty", 0.30)  # strafe: 0.30/0.30s -> ~2.5 cm (yaw ~0)
        self.declare_parameter("align_step_strafe_sec", 0.1575)
        self.declare_parameter("align_adaptive_steps_enabled", False)
        self.declare_parameter("align_mid_error_m", 0.06)
        self.declare_parameter("align_step_fwd_mid_sec", 0.24)
        self.declare_parameter("align_step_strafe_mid_sec", 0.28)
        # If the target remains in the wide/world map but drops out of the body cam at ALIGN,
        # back up once so the body cam can reacquire it instead of waiting stationary for timeout.
        self.declare_parameter("align_body_lost_backoff_enabled", True)
        self.declare_parameter("align_body_lost_backoff_speed", 0.189)
        self.declare_parameter("align_body_lost_backoff_sec", 0.54)
        self.declare_parameter("align_body_lost_backoff_settle_sec", 1.0)
        self.declare_parameter("align_body_lost_same_slot_retry_enabled", True)
        self.declare_parameter("align_body_lost_same_slot_retries", 1)
        self.declare_parameter("align_retry_heading_tol_rad", 0.10)
        self.declare_parameter("align_retry_heading_timeout_sec", 2.0)
        self.declare_parameter("align_body_lost_yaw_scan_enabled", False)
        self.declare_parameter("align_body_lost_yaw_scan_deg", 10.0)
        self.declare_parameter("align_body_lost_yaw_scan_omega", 0.10)
        self.declare_parameter("align_body_lost_yaw_scan_settle_sec", 0.35)
        self.declare_parameter("align_body_lost_yaw_scan_max_attempts", 1)
        self.declare_parameter("pick_duration_sec", 3.0)
        self.declare_parameter("storage_x", 0.2)
        self.declare_parameter("storage_y", 0.2)
        self.declare_parameter("shape_target_total", 4)   # set1 shape * 4
        self.declare_parameter("fruit_target_total", 0)   # set2 disabled by default: shapes only
        self.declare_parameter("publish_rate_hz", 5.0)
        # --- mock-field-test knobs (all default to competition behaviour) ---
        # dry_pick: log the pick instead of firing /arm/pick_trigger (arm not driven in the test).
        # end_after_quota: END once both quotas are met (skip STORE/DRIVE/DUMP storage phase).
        # select_timeout_sec>0: if SELECT_TARGET finds nothing for this long, advance phase (1->2)
        #   or END (phase 2) — graceful termination when fewer objects are present than the quota.
        self.declare_parameter("dry_pick", False)
        self.declare_parameter("end_after_quota", False)
        self.declare_parameter("select_timeout_sec", 0.0)
        # --- hardcoded opening move (very first thing at match start), BODY frame ---
        # Wait briefly, drive forward, strafe right, rotate in place by opening_turn_deg, then hold
        # until the configured match-elapsed release time before handing over to SCAN.
        # Timed through /base_command so the base_controller's start-boost + stop-brake apply.
        self.declare_parameter("opening_enabled", True)
        # Hold briefly at startup, then run the opening while perception is still warming up.
        self.declare_parameter("startup_warmup_sec", 3.0)
        self.declare_parameter("opening_speed", 0.35)        # >= wheel_min so it's the actual speed
        self.declare_parameter("opening_forward_sec", 1.0)
        self.declare_parameter("opening_strafe_speed", 0.35)
        self.declare_parameter("opening_strafe_right_sec", 0.0)
        self.declare_parameter("opening_turn_deg", 45.0)
        self.declare_parameter("opening_turn_omega", -0.17)   # negative = CW, positive = CCW
        self.declare_parameter("opening_wait_after_turn_sec", 1.0)
        self.declare_parameter("opening_release_at_sec", 0.0)  # if >0, do not start SCAN before this match time
        # SCAN search: the start pose likely sees nothing, so after the opening the robot turns slowly
        # CLOCKWISE in place to sweep for objects. Negative omega = CW (REP-103 +z is up). The actual
        # turn speed is set by base_controller wheel_min_rot (this just needs to be non-zero CW).
        self.declare_parameter("scan_search_omega", -0.17)
        # Direct travel uses the same /base_command ownership as OPENING for every waypoint.
        self.declare_parameter("direct_nav_speed", 0.12)
        self.declare_parameter("direct_nav_kp_ang", 1.5)
        self.declare_parameter("direct_nav_omega_max", 0.405)
        self.declare_parameter("direct_nav_face_tol", 0.35)
        self.declare_parameter("direct_nav_stop_radius_m", 0.08)
        # When nothing is visible, DRIVE to the map centre for a better view instead of spinning in
        # place (the wide fisheye already sees all around; a central vantage just helps).
        self.declare_parameter("map_center_x", 0.0)
        self.declare_parameter("map_center_y", 0.0)
        # If the phase target isn't in view, PATROL these field waypoints (looking with the wide cam)
        # until it appears — never give up. Flattened [x0,y0,x1,y1,...]. Kept inside the wall margin.
        self.declare_parameter("patrol_waypoints",
                               [0.0, 0.0, 1.2, 1.2, 1.2, -1.2, -1.2, -1.2, -1.2, 1.2])
        self.declare_parameter("patrol_reach_tol", 0.3)   # advance to next waypoint within this
        # Zone mission: grid-aligned, non-overlapping zones. Z1/Z2 own the right 3 columns,
        # and Z3/Z4 own the left 4 columns; upper/lower split is between grid rows at y=-0.25.
        self.declare_parameter("zone_mission_enabled", False)
        self.declare_parameter("zone_order", [1, 2, 3, 4])
        self.declare_parameter(
            "zone_bounds_m",
            [-2.0, -0.25, -0.25, 2.0, -2.0, -0.25, -2.0, -0.25,
             -0.25, 2.0, -2.0, -0.25, -0.25, 2.0, -0.25, 2.0],
        )
        self.declare_parameter(
            "zone_anchor_xy",
            [-1.0, 0.5, -1.0, -1.0, 0.75, -1.0, 0.75, 0.5],
        )
        self.declare_parameter("zone_no_target_advance_sec", 6.0)
        self.declare_parameter("zone_center_reach_tol_m", 0.25)
        self.declare_parameter("zone_anchor_nav_enabled", True)
        self.declare_parameter("zone_stabilize_enabled", True)
        self.declare_parameter("zone_stabilize_sec", 3.0)
        self.declare_parameter("zone_stabilize_require_fresh_seen", True)
        # Step-wise search: turn a little, STOP to let the cameras identify (clean, blur-free frames),
        # turn again. Continuous spinning motion-blurs the wide cam and churns tracks.
        self.declare_parameter("search_turn_sec", 0.5)    # rotate this long per step (~small angle)
        self.declare_parameter("search_look_sec", 1.6)    # then hold still this long to identify
        # APPROACH uses the direct-navigation parameters above. Keep the target latched through a
        # momentary dropout instead of thrashing back to SELECT.
        self.declare_parameter("approach_lost_grace_sec", 2.5)  # keep target this long if it drops out
        self.declare_parameter("approach_standoff_tol", 0.09)   # reached the stand-off within this -> ALIGN
        # phase 2: at the stand-off, hold up to this long for SigLIP to type the fruit BEFORE aligning,
        # so we don't waste a full align on an apple/banana. Orange -> align now; typed non-orange ->
        # dropped by _nearest_phase_object; still untyped after this -> align closer for a better view.
        self.declare_parameter("classify_standoff_sec", 2.0)
        self.declare_parameter("set2_require_fruit_label", True)
        # A Set2 slot is a fixed field inspection target above short-lived world-model track IDs.
        # The inventory implementation is generic, but this first rollout accepts set_type=2 only.
        self.declare_parameter("set2_slot_enabled", False)
        self.declare_parameter("set2_slot_merge_radius_m", 0.22)
        self.declare_parameter("set2_slot_attach_radius_m", 0.12)
        self.declare_parameter("set2_slot_confirm_min_obs", 1)
        self.declare_parameter("set2_slot_retry_enabled", True)
        self.declare_parameter("set2_slot_retry_limit", 2)
        self.declare_parameter("set2_slot_retry_cooldown_sec", 2.0)
        self.declare_parameter("set2_slot_body_max_age_sec", 1.0)
        self.declare_parameter("set2_slot_require_current_track_before_approach", True)
        self.declare_parameter("set2_slot_missing_track_recheck_attempts", 1)
        self.declare_parameter("set2_slot_distractor_front_x_m", 0.60)
        self.declare_parameter("set2_slot_distractor_side_y_m", 0.30)
        self.declare_parameter("set2_slot_birth_min_confidence", 0.75)
        self.declare_parameter("set2_slot_birth_min_obs_wide", 3)
        self.declare_parameter("set2_slot_birth_min_obs_body", 1)
        self.declare_parameter("set2_slot_birth_current_zone_only", True)
        self.declare_parameter("set2_slot_birth_after_stabilize_only", True)
        self.declare_parameter(
            "set2_slot_birth_labels",
            ["fruit_photo_cube", "apple", "orange", "banana", "pineapple"],
        )
        self.declare_parameter("set2_slot_grid_lock_enabled", True)
        self.declare_parameter("set2_slot_grid_lock_radius_m", 0.26)
        self.declare_parameter("set2_slot_grid_rows", 6)
        self.declare_parameter("set2_slot_grid_cols", 7)
        self.declare_parameter("set2_slot_grid_spacing_m", 0.50)
        self.declare_parameter("set2_slot_grid_origin_x_m", -1.50)
        self.declare_parameter("set2_slot_grid_origin_y_m", -1.50)
        self.declare_parameter("set2_slot_follow_map_transform", False)
        # Opportunistic Set2: keep the main phase on Set1, but if the target fruit cube is already
        # confidently identified nearby while passing through a zone, pick it and then resume Set1.
        self.declare_parameter("opportunistic_set2_enabled", False)
        self.declare_parameter("opportunistic_set2_max_picks", 2)
        self.declare_parameter("opportunistic_set2_max_dist_m", 0.70)
        self.declare_parameter("opportunistic_set2_min_conf", 0.75)
        self.declare_parameter("opportunistic_set2_min_nobs", 3)

        self.set1_label = str(self.get_parameter("set1_label").value)
        self.set2_label = str(self.get_parameter("set2_label").value)
        self.start_phase = int(self.get_parameter("start_phase").value)
        self.mixed_target_mode = bool(self.get_parameter("mixed_target_mode").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.pick_track_conf = float(self.get_parameter("pick_track_conf").value)
        self.approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        self.align_settle_sec = float(self.get_parameter("align_settle_sec").value)
        self.classify_timeout_sec = float(self.get_parameter("classify_timeout_sec").value)
        self.grab_x = float(self.get_parameter("grab_x").value)
        self.grab_y = float(self.get_parameter("grab_y").value)
        self.grab_min_x = float(self.get_parameter("grab_min_x").value)
        self.align_tol = float(self.get_parameter("align_tol_m").value)
        self.align_fwd_tol = float(self.get_parameter("align_fwd_tol_m").value)
        self.align_kp = float(self.get_parameter("align_kp").value)
        self.align_vmax = float(self.get_parameter("align_vmax").value)
        self.align_vmin = float(self.get_parameter("align_vmin").value)
        self.align_timeout_sec = float(self.get_parameter("align_timeout_sec").value)
        self.max_align_fails = int(self.get_parameter("max_align_fails").value)
        self._align_fail_count = 0
        self.align_pulse_sec = float(self.get_parameter("align_pulse_sec").value)
        self.align_settle_pulse_sec = float(self.get_parameter("align_settle_pulse_sec").value)
        self.align_step_fwd_duty = float(self.get_parameter("align_step_fwd_duty").value)
        self.align_step_fwd_sec = float(self.get_parameter("align_step_fwd_sec").value)
        self.align_step_strafe_duty = float(self.get_parameter("align_step_strafe_duty").value)
        self.align_step_strafe_sec = float(self.get_parameter("align_step_strafe_sec").value)
        self.align_adaptive_steps_enabled = bool(self.get_parameter("align_adaptive_steps_enabled").value)
        self.align_mid_error_m = float(self.get_parameter("align_mid_error_m").value)
        self.align_step_fwd_mid_sec = float(self.get_parameter("align_step_fwd_mid_sec").value)
        self.align_step_strafe_mid_sec = float(self.get_parameter("align_step_strafe_mid_sec").value)
        self.align_body_lost_backoff_enabled = bool(self.get_parameter("align_body_lost_backoff_enabled").value)
        self.align_body_lost_backoff_speed = float(self.get_parameter("align_body_lost_backoff_speed").value)
        self.align_body_lost_backoff_sec = float(self.get_parameter("align_body_lost_backoff_sec").value)
        self.align_body_lost_backoff_settle_sec = float(
            self.get_parameter("align_body_lost_backoff_settle_sec").value
        )
        self.align_body_lost_same_slot_retry_enabled = bool(
            self.get_parameter("align_body_lost_same_slot_retry_enabled").value
        )
        self.align_body_lost_same_slot_retries = int(
            self.get_parameter("align_body_lost_same_slot_retries").value
        )
        self.align_retry_heading_tol_rad = float(
            self.get_parameter("align_retry_heading_tol_rad").value
        )
        self.align_retry_heading_timeout_sec = float(
            self.get_parameter("align_retry_heading_timeout_sec").value
        )
        self.align_body_lost_yaw_scan_enabled = bool(
            self.get_parameter("align_body_lost_yaw_scan_enabled").value
        )
        self.align_body_lost_yaw_scan_deg = float(
            self.get_parameter("align_body_lost_yaw_scan_deg").value
        )
        self.align_body_lost_yaw_scan_omega = float(
            self.get_parameter("align_body_lost_yaw_scan_omega").value
        )
        self.align_body_lost_yaw_scan_settle_sec = float(
            self.get_parameter("align_body_lost_yaw_scan_settle_sec").value
        )
        self.align_body_lost_yaw_scan_max_attempts = int(
            self.get_parameter("align_body_lost_yaw_scan_max_attempts").value
        )
        self._align_phase = "measure"      # measure -> pulse/body_lost_backoff -> settle/SELECT -> measure ...
        self._align_phase_start = 0.0
        self._pulse_vx = 0.0
        self._pulse_vy = 0.0
        self._pulse_sec = self.align_step_fwd_sec   # duration of the current unit step (per direction)
        self._slot_body_lost_retry_counts: dict[int, int] = {}
        self._slot_yaw_scan_counts: dict[int, int] = {}
        self._target_yaw_scan_counts: dict[int, int] = {}
        self._slot_missing_track_counts: dict[int, int] = {}
        self._yaw_scan_step = 0
        self._yaw_scan_turn_sec = 0.0
        self._yaw_scan_omega_cmd = 0.0
        self._require_precise_heading_before_align = False
        self._precise_heading_retry_start_s: float | None = None
        # Body-cam ground homography for POSE-INDEPENDENT servo: project the object's body pixel
        # straight to base_link (no robot pose), so pose drift can't wander the servo target.
        self.declare_parameter("body_ground_homography_path",
                               "/home/seventt/seventt/workspace/data/calib/body_ground.npz")
        self.declare_parameter("body_cam_nadir_x", 0.055)
        self.declare_parameter("body_cam_height_m", 0.155)
        self.declare_parameter("object_center_height_m", 0.04)
        # Body-cam downscale compensation: multiply each body detection pixel by these before the
        # ALIGN homography (calibrated at 1640x1232). 1.0 = body published at full res.
        self.declare_parameter("body_px_scale_x", 1.0)
        self.declare_parameter("body_px_scale_y", 1.0)
        self._body_px_sx = float(self.get_parameter("body_px_scale_x").value)
        self._body_px_sy = float(self.get_parameter("body_px_scale_y").value)
        self._body_H = None
        try:
            self._body_H = np.load(str(self.get_parameter("body_ground_homography_path").value))["H"].astype(np.float64)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"body homography load failed ({exc}); ALIGN servo uses pose-based fallback")
        self._bnx = float(self.get_parameter("body_cam_nadir_x").value)
        self._bH = float(self.get_parameter("body_cam_height_m").value)
        self._boh = float(self.get_parameter("object_center_height_m").value)
        self._body_dets: list = []   # latest /camera_body/detections as (u, v_center, label)
        self._body_dets_stamp_s = 0.0
        self.pick_duration_sec = float(self.get_parameter("pick_duration_sec").value)
        self.storage_x = float(self.get_parameter("storage_x").value)
        self.storage_y = float(self.get_parameter("storage_y").value)
        self.shape_target_total = int(self.get_parameter("shape_target_total").value)
        self.fruit_target_total = int(self.get_parameter("fruit_target_total").value)
        self.dry_pick = bool(self.get_parameter("dry_pick").value)
        self.end_after_quota = bool(self.get_parameter("end_after_quota").value)
        self.select_timeout_sec = float(self.get_parameter("select_timeout_sec").value)
        self.opening_enabled = bool(self.get_parameter("opening_enabled").value)
        self.startup_warmup_sec = float(self.get_parameter("startup_warmup_sec").value)
        self.opening_speed = float(self.get_parameter("opening_speed").value)
        self.opening_forward_sec = float(self.get_parameter("opening_forward_sec").value)
        self.opening_strafe_speed = float(self.get_parameter("opening_strafe_speed").value)
        self.opening_strafe_right_sec = float(self.get_parameter("opening_strafe_right_sec").value)
        self.opening_turn_deg = float(self.get_parameter("opening_turn_deg").value)
        self.opening_turn_omega = float(self.get_parameter("opening_turn_omega").value)
        self.opening_wait_after_turn_sec = float(self.get_parameter("opening_wait_after_turn_sec").value)
        self.opening_release_at_sec = max(0.0, float(self.get_parameter("opening_release_at_sec").value))
        self.scan_search_omega = float(self.get_parameter("scan_search_omega").value)
        self.direct_nav_speed = float(self.get_parameter("direct_nav_speed").value)
        self.direct_nav_kp_ang = float(self.get_parameter("direct_nav_kp_ang").value)
        self.direct_nav_omega_max = float(self.get_parameter("direct_nav_omega_max").value)
        self.direct_nav_face_tol = float(self.get_parameter("direct_nav_face_tol").value)
        self.direct_nav_stop_radius_m = float(self.get_parameter("direct_nav_stop_radius_m").value)
        self.map_center_x = float(self.get_parameter("map_center_x").value)
        self.map_center_y = float(self.get_parameter("map_center_y").value)
        wp = [float(v) for v in self.get_parameter("patrol_waypoints").value]
        self._patrol_waypoints = [(wp[i], wp[i + 1]) for i in range(0, len(wp) - 1, 2)] or [(0.0, 0.0)]
        self.patrol_reach_tol = float(self.get_parameter("patrol_reach_tol").value)
        self._patrol_idx = 0
        self.zone_mission_enabled = bool(self.get_parameter("zone_mission_enabled").value)
        self.zone_order = [int(v) for v in self.get_parameter("zone_order").value] or [1, 2, 3, 4]
        zb = [float(v) for v in self.get_parameter("zone_bounds_m").value]
        self.zone_bounds: dict[int, tuple[float, float, float, float]] = {}
        for i in range(0, min(len(zb), 16), 4):
            zid = i // 4 + 1
            self.zone_bounds[zid] = (zb[i], zb[i + 1], zb[i + 2], zb[i + 3])
        za = [float(v) for v in self.get_parameter("zone_anchor_xy").value]
        self.zone_anchors: dict[int, tuple[float, float]] = {}
        for i in range(0, min(len(za), 8), 2):
            self.zone_anchors[i // 2 + 1] = (za[i], za[i + 1])
        self.zone_no_target_advance_sec = float(self.get_parameter("zone_no_target_advance_sec").value)
        self.zone_center_reach_tol_m = float(self.get_parameter("zone_center_reach_tol_m").value)
        self.zone_anchor_nav_enabled = bool(self.get_parameter("zone_anchor_nav_enabled").value)
        self.zone_stabilize_enabled = bool(self.get_parameter("zone_stabilize_enabled").value)
        self.zone_stabilize_sec = float(self.get_parameter("zone_stabilize_sec").value)
        self.zone_stabilize_require_fresh_seen = bool(
            self.get_parameter("zone_stabilize_require_fresh_seen").value
        )
        self._zone_idx = 0
        self._zone_no_target_since: float | None = None
        self._zone_stabilized_idx: int | None = None
        self._zone_anchor_reached_idx: int | None = None
        self._zone_stabilize_start_s = 0.0
        self._zone_stabilized_after_s = 0.0

        # ---- LANE-GRAPH waypoint planner (50cm object grid, 40cm robot) ----
        # Travel goals (SCAN sweep, APPROACH stand-off, DRIVE_TO_STORAGE) route through collision-free
        # lane midlines instead of a straight shot; go_to_goal still drives each via car-like + reactive.
        self.declare_parameter("planner_enabled", True)
        self.declare_parameter("grid_spacing_m", 0.50)
        self.declare_parameter("grid_origin_mode", "infer")     # infer | fixed
        self.declare_parameter("grid_origin_xy", [0.0, 0.0])
        self.declare_parameter("phase_tol_m", 0.08)
        self.declare_parameter("field_bounds_m", [-2.0, 2.0, -2.0, 2.0])   # mirror go_to_goal
        self.declare_parameter("robot_margin_m", 0.22)
        self.declare_parameter("lane_block_radius_m", 0.24)     # obstacle-to-lane dist that blocks an edge
        self.declare_parameter("comfort_clear_m", 0.35)
        self.declare_parameter("clearance_weight", 2.0)         # prefer roomy lanes over tight gates
        self.declare_parameter("lane_simplify_enabled", True)   # false keeps raw 4-connected lane vias
        self.declare_parameter("direct_fallback_enabled", True)  # false holds position if no lane route exists
        self.declare_parameter("start_connect_k", 4)
        self.declare_parameter("wp_reach_tol_m", 0.12)          # advance to next via within this
        self.declare_parameter("replan_period_sec", 1.5)
        self.declare_parameter("replan_goal_move_m", 0.15)      # replan if the destination drifts
        self.declare_parameter("replan_throttle_sec", 0.5)
        self.declare_parameter("obstacle_min_conf", 0.4)        # phantom filter for obstacles
        self.declare_parameter("obstacle_min_nobs", 2)
        self.declare_parameter("exclude_target_radius_m", 0.30)  # drop objects near dest (final leg reachable)
        self.declare_parameter("front_escape_enabled", True)
        self.declare_parameter("front_escape_x_min_m", 0.05)
        self.declare_parameter("front_escape_x_max_m", 0.45)
        self.declare_parameter("front_escape_y_abs_m", 0.25)
        self.declare_parameter("front_escape_step_m", 0.25)
        self.declare_parameter("front_escape_forward_m", 0.0)
        self.planner_enabled = bool(self.get_parameter("planner_enabled").value)
        fb = [float(v) for v in self.get_parameter("field_bounds_m").value]
        self.wp_reach_tol_m = float(self.get_parameter("wp_reach_tol_m").value)
        self.replan_period_sec = float(self.get_parameter("replan_period_sec").value)
        self.replan_goal_move_m = float(self.get_parameter("replan_goal_move_m").value)
        self.replan_throttle_sec = float(self.get_parameter("replan_throttle_sec").value)
        self.obstacle_min_conf = float(self.get_parameter("obstacle_min_conf").value)
        self.obstacle_min_nobs = int(self.get_parameter("obstacle_min_nobs").value)
        self.exclude_target_radius_m = float(self.get_parameter("exclude_target_radius_m").value)
        self.front_escape_enabled = bool(self.get_parameter("front_escape_enabled").value)
        self.front_escape_x_min_m = float(self.get_parameter("front_escape_x_min_m").value)
        self.front_escape_x_max_m = float(self.get_parameter("front_escape_x_max_m").value)
        self.front_escape_y_abs_m = float(self.get_parameter("front_escape_y_abs_m").value)
        self.front_escape_step_m = float(self.get_parameter("front_escape_step_m").value)
        self.front_escape_forward_m = float(self.get_parameter("front_escape_forward_m").value)
        self.direct_fallback_enabled = bool(self.get_parameter("direct_fallback_enabled").value)
        self._planner = LanePlanner(
            spacing=float(self.get_parameter("grid_spacing_m").value),
            bounds=(fb[0], fb[1], fb[2], fb[3]) if len(fb) == 4 else (-2.0, 2.0, -2.0, 2.0),
            margin=float(self.get_parameter("robot_margin_m").value),
            block_radius=float(self.get_parameter("lane_block_radius_m").value),
            comfort_clear=float(self.get_parameter("comfort_clear_m").value),
            clearance_weight=float(self.get_parameter("clearance_weight").value),
            phase_tol=float(self.get_parameter("phase_tol_m").value),
            origin_mode=str(self.get_parameter("grid_origin_mode").value),
            origin_xy=tuple(float(v) for v in self.get_parameter("grid_origin_xy").value)[:2] or (0.0, 0.0),
            start_connect_k=int(self.get_parameter("start_connect_k").value),
            simplify=bool(self.get_parameter("lane_simplify_enabled").value),
        )
        self._plan: list[tuple[float, float]] | None = None   # active via list (last = dest)
        self._plan_idx = 0
        self._plan_dest: tuple[float, float] | None = None
        self._plan_escape = False
        self._plan_stamp = 0.0
        self._last_replan_t = 0.0
        self._coverage: list[tuple[float, float]] | None = None   # SCAN lane-sweep waypoints
        self._coverage_idx = 0
        self.search_turn_sec = float(self.get_parameter("search_turn_sec").value)
        self.search_look_sec = float(self.get_parameter("search_look_sec").value)
        self.approach_lost_grace_sec = float(self.get_parameter("approach_lost_grace_sec").value)
        self.approach_standoff_tol = float(self.get_parameter("approach_standoff_tol").value)
        self.classify_standoff_sec = float(self.get_parameter("classify_standoff_sec").value)
        self.set2_require_fruit_label = bool(self.get_parameter("set2_require_fruit_label").value)
        self.set2_slot_enabled = bool(self.get_parameter("set2_slot_enabled").value)
        self.set2_slot_merge_radius_m = float(self.get_parameter("set2_slot_merge_radius_m").value)
        self.set2_slot_attach_radius_m = float(self.get_parameter("set2_slot_attach_radius_m").value)
        self.set2_slot_retry_enabled = bool(self.get_parameter("set2_slot_retry_enabled").value)
        self.set2_slot_retry_limit = int(self.get_parameter("set2_slot_retry_limit").value)
        self.set2_slot_retry_cooldown_sec = float(
            self.get_parameter("set2_slot_retry_cooldown_sec").value
        )
        self.set2_slot_body_max_age_sec = float(
            self.get_parameter("set2_slot_body_max_age_sec").value
        )
        self.set2_slot_require_current_track_before_approach = bool(
            self.get_parameter("set2_slot_require_current_track_before_approach").value
        )
        self.set2_slot_missing_track_recheck_attempts = int(
            self.get_parameter("set2_slot_missing_track_recheck_attempts").value
        )
        self.set2_slot_distractor_front_x_m = float(
            self.get_parameter("set2_slot_distractor_front_x_m").value
        )
        self.set2_slot_distractor_side_y_m = float(
            self.get_parameter("set2_slot_distractor_side_y_m").value
        )
        self.set2_slot_birth_min_confidence = float(
            self.get_parameter("set2_slot_birth_min_confidence").value
        )
        self.set2_slot_birth_min_obs_wide = int(
            self.get_parameter("set2_slot_birth_min_obs_wide").value
        )
        self.set2_slot_birth_min_obs_body = int(
            self.get_parameter("set2_slot_birth_min_obs_body").value
        )
        self.set2_slot_birth_current_zone_only = bool(
            self.get_parameter("set2_slot_birth_current_zone_only").value
        )
        self.set2_slot_birth_after_stabilize_only = bool(
            self.get_parameter("set2_slot_birth_after_stabilize_only").value
        )
        self.set2_slot_birth_labels = {
            str(v) for v in self.get_parameter("set2_slot_birth_labels").value
        }
        self.set2_slot_grid_lock_enabled = bool(
            self.get_parameter("set2_slot_grid_lock_enabled").value
        )
        self.set2_slot_grid_lock_radius_m = float(
            self.get_parameter("set2_slot_grid_lock_radius_m").value
        )
        self.set2_slot_grid_rows = int(self.get_parameter("set2_slot_grid_rows").value)
        self.set2_slot_grid_cols = int(self.get_parameter("set2_slot_grid_cols").value)
        self.set2_slot_grid_spacing_m = float(
            self.get_parameter("set2_slot_grid_spacing_m").value
        )
        self.set2_slot_grid_origin_x_m = float(
            self.get_parameter("set2_slot_grid_origin_x_m").value
        )
        self.set2_slot_grid_origin_y_m = float(
            self.get_parameter("set2_slot_grid_origin_y_m").value
        )
        self.set2_slot_follow_map_transform = bool(
            self.get_parameter("set2_slot_follow_map_transform").value
        )
        self.slot_inventory = SlotInventory(
            enabled_set_types={2},
            merge_radius_m=self.set2_slot_merge_radius_m,
            confirm_min_obs=int(self.get_parameter("set2_slot_confirm_min_obs").value),
        )
        self.opportunistic_set2_enabled = bool(self.get_parameter("opportunistic_set2_enabled").value)
        self.opportunistic_set2_max_picks = int(self.get_parameter("opportunistic_set2_max_picks").value)
        self.opportunistic_set2_max_dist_m = float(self.get_parameter("opportunistic_set2_max_dist_m").value)
        self.opportunistic_set2_min_conf = float(self.get_parameter("opportunistic_set2_min_conf").value)
        self.opportunistic_set2_min_nobs = int(self.get_parameter("opportunistic_set2_min_nobs").value)
        self._standoff_arrived_s = None
        self._search_phase = "look"     # step-wise search: look <-> turn
        self._search_t0 = self._now_s()
        self._appr_tgt_xy = None        # last-known target field xy (survives momentary dropout)
        self._appr_last_seen_s = 0.0
        rate = float(self.get_parameter("publish_rate_hz").value)

        # --- runtime state ---
        # Start with the hardcoded opening move, then fall into SCAN. Disable via param.
        self.state = "OPENING" if self.opening_enabled else "SCAN"
        self._opening_leg = "wait"      # wait -> forward -> strafe_right -> turn -> settle -> SCAN
        self._opening_turn_start_theta: float | None = None
        self._world_mapping_enabled = not self.opening_enabled
        self._got_world = False         # set on first /world_model (perception up)
        self._node_start_s = self._now_s()
        self.phase = 2 if self.start_phase == 2 else 1  # 1 = Set1, 2 = Set2 (mapping is continuous)
        self.tray_shape = 0
        self.tray_fruit = 0
        self.current_target: Object | None = None    # latched selected target (id, set_type, ...)
        self.current_slot_id: int | None = None      # Set2 target, stable across world track IDs
        self.set_type = 0                             # set_type confirmed by the pick gate
        self._opportunistic_set2_active = False

        # latest inbound messages
        self.world: WorldModel | None = None
        self.selected: Object | None = None
        self.siglip: Classification | None = None
        self.shape: Classification | None = None
        self.siglip_stamp_s: float | None = None      # arrival time (node clock) of last siglip
        self.shape_stamp_s: float | None = None        # arrival time (node clock) of last shape

        # time bookkeeping (seconds, node clock)
        self.state_enter_s = self._now_s()

        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(Object, "/selected_target", self.on_target, 10)
        self.create_subscription(Classification, "/classification/siglip", self.on_siglip, 10)
        self.create_subscription(Classification, "/classification/shape", self.on_shape, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_dets, 10)
        self.create_subscription(Empty, "/state_advance", self.on_advance, 10)
        self.create_subscription(
            Float32MultiArray, "/localization/wall_map_transform", self.on_wall_map_transform, 10
        )

        self.pub_state = self.create_publisher(MissionState, "/mission_state", 10)
        self.pub_blacklist = self.create_publisher(UInt64, "/world_model/blacklist_add", 10)
        self.pub_planning_obstacles = self.create_publisher(PoseArray, "/planning/obstacles", 10)
        self.pub_pick = self.create_publisher(Bool, "/arm/pick_trigger", 10)
        self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)   # sole base-motion owner
        # Current pick phase (1=Set1, 2=Set2) for the target selector's phase filter.
        self.pub_phase = self.create_publisher(Int8, "/planning/phase", 10)
        self.pub_zone = self.create_publisher(Int8, "/planning/zone", 10)
        # Human-readable decision feed (PICK / PASS / SKIP / PHASE / END) for the visualiser.
        self.pub_decision = self.create_publisher(String, "/planning/decision", 10)
        self.pub_slots = self.create_publisher(String, "/planning/object_slots", 10)
        self.pub_mapping_enabled = self.create_publisher(Bool, "/world_model/mapping_enabled", 10)
        self.pub_wall_fast = self.create_publisher(Bool, "/localization/wall_fast_correction", 10)

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"FSM started at {self.state}  set1='{self.set1_label}' set2='{self.set2_label}' "
            f"conf>={self.conf_threshold} approach<{self.approach_dist_m}m rate={rate}Hz "
            f"dry_pick={self.dry_pick} end_after_quota={self.end_after_quota} "
            f"zone_mission={self.zone_mission_enabled} zone={self._active_zone_id()} "
            f"select_timeout={self.select_timeout_sec}s set2_slots={self.set2_slot_enabled} "
            f"mixed_target_mode={self.mixed_target_mode}"
        )

    # ------------------------------------------------------------------ utils
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _time_in_state(self) -> float:
        return self._now_s() - self.state_enter_s

    def _wall_fast_correction_requested(self) -> bool:
        """Request fast wall correction only in states intended to be stationary."""
        if self.state in {
            "OPENING",
            "ZONE_STABILIZE",
            "ALIGN",
            "PICK",
            "STORE_IN_TRAY",
            "ALIGN_OVER_BIN",
            "DUMP_ALL",
        }:
            return True
        if self.state == "SCAN" and self._search_phase == "look":
            return True
        if self.state == "APPROACH" and self._standoff_arrived_s is not None:
            return True
        return False

    def _enter(self, new_state: str) -> None:
        prev = self.state
        self.state = new_state
        self.state_enter_s = self._now_s()
        self._plan = None                       # a state change invalidates the active travel plan
        self._plan_escape = False
        if new_state == "ALIGN":
            self._align_phase = "measure"       # start each ALIGN by measuring the settled position
            self._yaw_scan_step = 0
            self._yaw_scan_omega_cmd = 0.0
        if new_state == "APPROACH":
            self._appr_tgt_xy = (
                (float(self.current_target.x), float(self.current_target.y))
                if self.current_target is not None else None
            )
            self._appr_last_seen_s = self._now_s()
            self._standoff_arrived_s = None
        if new_state == "ZONE_STABILIZE":
            self.current_target = None
            self._clear_current_slot()
            self._opportunistic_set2_active = False
            self._reset_zone_scan_timer()
            self._zone_stabilize_start_s = self._now_s()
        self.get_logger().info(
            f"transition {prev} -> {new_state}  "
            f"tray=(shape:{self.tray_shape}, fruit:{self.tray_fruit})  "
            f"target_id={self.current_target.id if self.current_target else 0} "
            f"slot_id={self.current_slot_id or 0}"
        )

    def _lookup_object(self, obj_id: int) -> Object | None:
        """Fresh world-model entry for a tracker id (None if not present)."""
        if self.world is None or obj_id == 0:
            return None
        for obj in self.world.objects:
            if obj.id == obj_id:
                return obj
        return None

    def _to_base(self, fx: float, fy: float) -> tuple[float, float] | None:
        """Field xy -> base_link xy (x fwd, y left) using the world model's robot pose."""
        if self.world is None:
            return None
        dx, dy = fx - self.world.robot_x, fy - self.world.robot_y
        ct, st = math.cos(self.world.robot_theta), math.sin(self.world.robot_theta)
        return (ct * dx + st * dy, -st * dx + ct * dy)

    def _front_target(self, set_type: int, max_r: float = 0.45,
                      want_label: str | None = None) -> Object | None:
        """The non-blacklisted object of this set_type (and, if given, this class_label) nearest the
        arm grab point (base_link). Position-based, not a latched id (which churns). want_label lets
        CLASSIFY confirm the SPECIFIC target shape (icosahedron), ignoring neighbouring cubes."""
        if self.world is None:
            return None
        best = None
        bestd = max_r
        for o in self.world.objects:
            if o.blacklisted or o.set_type != set_type:
                continue
            if want_label is not None and str(o.class_label) != want_label:
                continue
            base = self._to_base(o.x, o.y)
            if base is None:
                continue
            d = math.hypot(base[0] - self.grab_x, base[1] - self.grab_y)
            if d < bestd:
                bestd = d
                best = o
        return best

    def _patrol_search(self) -> None:
        """SEARCH: sweep the free lane midlines in a systematic boustrophedon (via the planner) to
        cover the whole field, driving car-like via _drive_toward. The SCAN handler leaves this the
        instant a phase target comes into view; if the sweep completes it loops (never give up)."""
        if self.zone_mission_enabled:
            if not self.zone_anchor_nav_enabled:
                self._search_step()
                return
            if self._zone_anchor_reached_idx == self._zone_idx:
                self._search_step()
                return
            zx, zy = self._zone_center()
            if (d := self._distance_to(zx, zy)) is not None and d <= self.zone_center_reach_tol_m:
                self._zone_anchor_reached_idx = self._zone_idx
                self._search_step()
            else:
                th = self.world.robot_theta if self.world is not None else 0.0
                self._drive_toward(zx, zy, th, exclude_id=0)
            return
        if not self.planner_enabled:
            wp = self._patrol_waypoints[self._patrol_idx]
            th = self.world.robot_theta if self.world is not None else 0.0
            self._drive_toward_direct(wp[0], wp[1], th)
            if (d := self._distance_to(wp[0], wp[1])) is not None and d < self.patrol_reach_tol:
                self._patrol_idx = (self._patrol_idx + 1) % len(self._patrol_waypoints)
            return
        if not self._coverage:
            rxy = self._robot_xy() or (0.0, 0.0)
            self._coverage = self._planner.coverage_path(rxy, self._obstacles_snapshot())
            self._coverage_idx = 0
            if not self._coverage:                       # no lattice yet -> fall back to fixed patrol
                self._coverage = list(self._patrol_waypoints)
        wp = self._coverage[self._coverage_idx]
        th = self.world.robot_theta if self.world is not None else 0.0
        self._drive_toward(wp[0], wp[1], th, exclude_id=0)
        if (d := self._distance_to(wp[0], wp[1])) is not None and d < self.patrol_reach_tol:
            self._coverage_idx += 1
            self._plan = None                            # next sweep waypoint replans its route
            if self._coverage_idx >= len(self._coverage):
                self._coverage = None                    # sweep done -> rebuild (map grew) & loop
                self._coverage_idx = 0

    def _body_label(self) -> str:
        """Body-cam YOLO label to ALIGN to this phase: the set1 shape (phase 1), or the generic fruit
        cube (phase 2 — the specific fruit is confirmed separately by SigLIP in CLASSIFY)."""
        return "fruit_photo_cube" if self._opportunistic_set2_active or self.phase == 2 else self.set1_label

    def _body_align_labels(self) -> set[str]:
        if self._opportunistic_set2_active or self.phase == 2:
            return {"fruit_photo_cube", "cube"}
        return {self.set1_label}

    def _nearest_phase_object(self, ref_xy: tuple[float, float] | None = None) -> Object | None:
        """The non-blacklisted world-model object for THIS pick phase nearest a REFERENCE point
        (default = the robot). Purely position/class based (NO track id) so it never churns. During
        an approach the reference is the LATCHED target xy, so we stay committed to ONE object instead
        of shuttling to whichever is momentarily closest (that was the set2 back-and-forth). phase1 =
        the set1 shape (by label); phase2 = a fruit cube (type confirmed later by SigLIP)."""
        if self.world is None:
            return None
        want_set = 1 if self.phase == 1 else 2
        want_label = self.set1_label
        rx, ry = ref_xy if ref_xy is not None else (self.world.robot_x, self.world.robot_y)
        best = None
        bestd = 1e9
        for o in self.world.objects:
            if o.blacklisted or o.set_type != want_set:
                continue
            if (self.zone_mission_enabled
                    and not self._object_in_active_zone(o)
                    and (self.current_target is None or int(o.id) != int(self.current_target.id))):
                continue
            if (self.zone_mission_enabled
                    and (self.current_target is None or int(o.id) != int(self.current_target.id))
                    and not self._object_seen_after_zone_stabilize(o)):
                continue
            if want_set == 1 and str(o.class_label) != want_label:
                continue
            # phase 2: a fruit cube already SigLIP-typed as a DIFFERENT fruit is not our target —
            # skip it so we don't waste an approach+align on an apple/banana. Untyped cubes ('') are
            # still approached (they get typed once the body cam is close enough).
            if want_set == 2:
                fl = str(o.fruit_label)
                if self.set2_require_fruit_label and fl and fl != self.set2_label:
                    continue
            if float(o.confidence) < self.pick_track_conf:
                continue
            d = math.hypot(o.x - rx, o.y - ry)
            if d < bestd:
                bestd = d
                best = o
        return best

    def _shape_quota_available(self) -> bool:
        return bool(self.set1_label) and self.tray_shape < self.shape_target_total

    def _fruit_quota_available(self) -> bool:
        return bool(self.set2_label) and self.tray_fruit < self.fruit_target_total

    def _nearest_set1_target(self, ref_xy: tuple[float, float] | None = None) -> Object | None:
        if self.world is None or not self._shape_quota_available():
            return None
        rx, ry = ref_xy if ref_xy is not None else (self.world.robot_x, self.world.robot_y)
        best = None
        bestd = 1e9
        for o in self.world.objects:
            if o.blacklisted or int(o.set_type) != 1:
                continue
            if str(o.class_label) != self.set1_label:
                continue
            if self.zone_mission_enabled and not self._object_in_active_zone(o):
                continue
            if self.zone_mission_enabled and not self._object_seen_after_zone_stabilize(o):
                continue
            if float(o.confidence) < self.pick_track_conf:
                continue
            d = math.hypot(float(o.x) - rx, float(o.y) - ry)
            if d < bestd:
                best = o
                bestd = d
        return best

    def _nearest_set2_target_object(self, ref_xy: tuple[float, float] | None = None) -> Object | None:
        if self.world is None or not self._fruit_quota_available():
            return None
        rx, ry = ref_xy if ref_xy is not None else (self.world.robot_x, self.world.robot_y)
        best = None
        bestd = 1e9
        for o in self.world.objects:
            if o.blacklisted or int(o.set_type) != 2:
                continue
            if self.zone_mission_enabled and not self._object_in_active_zone(o):
                continue
            if self.zone_mission_enabled and not self._object_seen_after_zone_stabilize(o):
                continue
            fl = str(o.fruit_label)
            if self.set2_require_fruit_label and fl and fl != self.set2_label:
                continue
            if float(o.confidence) < self.pick_track_conf:
                continue
            d = math.hypot(float(o.x) - rx, float(o.y) - ry)
            if d < bestd:
                best = o
                bestd = d
        return best

    def _select_next_mixed_target(self):
        if not self.mixed_target_mode or self.world is None:
            return None
        rx, ry = self.world.robot_x, self.world.robot_y
        candidates = []
        set1 = self._nearest_set1_target()
        if set1 is not None:
            candidates.append((math.hypot(set1.x - rx, set1.y - ry), "set1", set1))
        if self._fruit_quota_available():
            if self.set2_slot_enabled:
                slot = self._select_next_set2_slot()
                if slot is not None:
                    candidates.append((slot.distance_to(rx, ry), "set2_slot", slot))
            else:
                set2 = self._nearest_set2_target_object()
                if set2 is not None:
                    candidates.append((math.hypot(set2.x - rx, set2.y - ry), "set2_object", set2))
        if not candidates:
            return None
        _, kind, target = min(candidates, key=lambda item: item[0])
        return kind, target

    def _has_unresolved_mixed_set2_slot(self) -> bool:
        return bool(
            self.mixed_target_mode
            and self.set2_slot_enabled
            and self._fruit_quota_available()
            and self._has_unresolved_set2_slot()
        )

    def _latch_mixed_target(self, kind: str, target) -> bool:
        self._reset_zone_scan_timer()
        self._opportunistic_set2_active = False
        if kind == "set1":
            self.phase = 1
            self.current_target = target
            self._clear_current_slot()
            self._decide(f"MIXED SET1 {target.class_label} #{target.id}")
            return True
        if kind == "set2_slot":
            self.phase = 2
            self._latch_set2_slot(target, decision_prefix="MIXED SLOT")
            return True
        if kind == "set2_object":
            self.phase = 2
            self.current_target = target
            self._clear_current_slot()
            self._decide(f"MIXED SET2 {target.fruit_label or target.class_label} #{target.id}")
            return True
        return False

    # --------------------------------------------------------- Set2 slot inventory
    def _set2_slot_mode(self) -> bool:
        return self.set2_slot_enabled and (self.phase == 2 or self._opportunistic_set2_active)

    def _current_slot(self) -> ObjectSlot | None:
        return self.slot_inventory.get(self.current_slot_id)

    def _slot_in_active_zone(self, slot: ObjectSlot) -> bool:
        if not self.zone_mission_enabled:
            return True
        xmin, xmax, ymin, ymax = self.zone_bounds.get(
            self._active_zone_id(), (-2.0, 2.0, -2.0, 2.0)
        )
        return xmin <= slot.x <= xmax and ymin <= slot.y <= ymax

    def _slot_seen_after_s(self) -> float:
        if (self.zone_mission_enabled
                and self.zone_stabilize_enabled
                and self.zone_stabilize_require_fresh_seen
                and self._zone_stabilized_idx == self._zone_idx):
            return self._zone_stabilized_after_s
        return 0.0

    def _allow_new_set2_slot(self, obj: Object, seen_s: float) -> bool:
        """Gate persistent slot birth while still allowing existing slots to update."""
        if int(obj.set_type) != 2 or bool(obj.blacklisted):
            return False
        if (self.set2_slot_birth_current_zone_only
                and not self._object_in_active_zone(obj)):
            return False
        if self.set2_slot_birth_after_stabilize_only and self.zone_mission_enabled:
            if not self._zone_is_stabilized():
                return False
            if seen_s < self._slot_seen_after_s():
                return False
        labels = {str(obj.class_label), str(obj.fruit_label)}
        if self.set2_slot_birth_labels and labels.isdisjoint(self.set2_slot_birth_labels):
            return False
        if float(obj.confidence) < self.set2_slot_birth_min_confidence:
            return False
        source = str(obj.source).lower()
        min_obs = (
            self.set2_slot_birth_min_obs_body
            if "body" in source
            else self.set2_slot_birth_min_obs_wide
        )
        if int(obj.n_obs) < max(1, min_obs):
            return False
        if self.set2_slot_grid_lock_enabled:
            return self._nearest_set2_slot_grid(float(obj.x), float(obj.y)) is not None
        return True

    def _nearest_set2_slot_grid(self, x: float, y: float) -> tuple[int, float, float] | None:
        if self.set2_slot_grid_rows <= 0 or self.set2_slot_grid_cols <= 0:
            return None
        spacing = max(1e-6, self.set2_slot_grid_spacing_m)
        limit = max(0.0, self.set2_slot_grid_lock_radius_m)
        best: tuple[float, int, float, float] | None = None
        for row in range(self.set2_slot_grid_rows):
            gy = self.set2_slot_grid_origin_y_m + row * spacing
            for col in range(self.set2_slot_grid_cols):
                gx = self.set2_slot_grid_origin_x_m + col * spacing
                dist = math.hypot(float(x) - gx, float(y) - gy)
                if dist <= limit and (best is None or dist < best[0]):
                    best = (dist, row * self.set2_slot_grid_cols + col + 1, gx, gy)
        if best is None:
            return None
        return best[1], best[2], best[3]

    def _select_next_set2_slot(self) -> ObjectSlot | None:
        robot_xy = self._robot_xy()
        if not self.set2_slot_enabled or robot_xy is None:
            return None
        for _ in range(len(self.slot_inventory.slots)):
            slot = self.slot_inventory.select_next_slot(
                robot_xy=robot_xy,
                now_s=self._now_s(),
                min_confidence=self.pick_track_conf,
                seen_after_s=self._slot_seen_after_s(),
                predicate=self._slot_in_active_zone,
            )
            if slot is None:
                return None
            if not self.set2_slot_require_current_track_before_approach:
                return slot
            if self._object_for_slot(slot, max_distance_m=self.set2_slot_attach_radius_m) is not None:
                self._slot_missing_track_counts.pop(slot.slot_id, None)
                return slot
            self._defer_or_drop_untracked_slot(slot)
        return None

    def _defer_or_drop_untracked_slot(self, slot: ObjectSlot) -> None:
        """Do not drive to stale Set2 slots that have no current world track nearby."""
        now = self._now_s()
        if not self.set2_slot_retry_enabled:
            self.slot_inventory.mark_non_target(slot.slot_id, now)
            slot.physical_state = "lost_suspect"
            self._decide(f"SLOT DROP F{slot.slot_id} (track#0)")
            self.get_logger().warn(
                f"Set2 slot F{slot.slot_id}: no current track within "
                f"{self.set2_slot_attach_radius_m:.2f}m -> close slot (retry disabled)"
            )
            return
        missing_count = self._slot_missing_track_counts.get(slot.slot_id, 0)
        if missing_count < max(1, self.set2_slot_missing_track_recheck_attempts):
            self._slot_missing_track_counts[slot.slot_id] = missing_count + 1
            self.slot_inventory.mark_retry(
                slot.slot_id,
                now_s=now,
                retry_limit=self.set2_slot_retry_limit,
                retry_cooldown_sec=self.set2_slot_retry_cooldown_sec,
            )
            self._decide(f"SLOT RECHECK F{slot.slot_id} (track#0)")
            self.get_logger().warn(
                f"Set2 slot F{slot.slot_id}: no current track within "
                f"{self.set2_slot_attach_radius_m:.2f}m -> recheck after "
                f"{self.set2_slot_retry_cooldown_sec:.1f}s"
            )
            return
        self.slot_inventory.mark_non_target(slot.slot_id, now)
        slot.physical_state = "lost_suspect"
        self._decide(f"SLOT LOST F{slot.slot_id} (track#0)")
        self.get_logger().warn(
            f"Set2 slot F{slot.slot_id}: still no current track after recheck -> close slot"
        )

    def _has_unresolved_set2_slot(self) -> bool:
        if not self.set2_slot_enabled:
            return False
        return self.slot_inventory.has_unresolved(
            min_confidence=self.pick_track_conf,
            seen_after_s=self._slot_seen_after_s(),
            predicate=self._slot_in_active_zone,
        )

    def _slot_for_object(self, obj: Object) -> ObjectSlot | None:
        slot = self.slot_inventory.slot_for_track(int(obj.id))
        if slot is not None:
            return slot
        return self.slot_inventory.nearest_slot(
            float(obj.x),
            float(obj.y),
            set_type=2,
            max_distance_m=self.set2_slot_merge_radius_m,
        )

    def _object_for_slot(
        self,
        slot: ObjectSlot,
        *,
        max_distance_m: float | None = None,
    ) -> Object | None:
        if self.world is None:
            return None
        limit = self.set2_slot_merge_radius_m if max_distance_m is None else float(max_distance_m)
        candidates = []
        if slot.track_id:
            fresh = self._lookup_object(int(slot.track_id))
            if fresh is not None:
                candidates.append(fresh)
        candidates.extend(self.world.objects)
        best = None
        best_dist = max(0.0, limit)
        seen_ids = set()
        for obj in candidates:
            if int(obj.id) in seen_ids:
                continue
            seen_ids.add(int(obj.id))
            if obj.blacklisted or int(obj.set_type) != 2:
                continue
            dist = slot.distance_to(float(obj.x), float(obj.y))
            if dist <= best_dist:
                best = obj
                best_dist = dist
        if best is not None:
            self.slot_inventory.link_track(slot.slot_id, int(best.id))
        return best

    def _latch_set2_slot(self, slot: ObjectSlot, *, decision_prefix: str = "SLOT") -> None:
        self.current_slot_id = slot.slot_id
        self.current_target = self._object_for_slot(slot)
        track_id = int(self.current_target.id) if self.current_target is not None else 0
        self._decide(f"{decision_prefix} F{slot.slot_id} track#{track_id}")
        self.get_logger().info(
            f"Set2 slot F{slot.slot_id} latched at ({slot.x:.2f},{slot.y:.2f}), track={track_id}"
        )

    def _clear_current_slot(self) -> None:
        self.current_slot_id = None
        self._require_precise_heading_before_align = False
        self._precise_heading_retry_start_s = None

    def _same_slot_reapproach_after_body_lost(self) -> bool:
        slot = self._current_slot()
        if (slot is None
                or not self.align_body_lost_same_slot_retry_enabled
                or self.align_body_lost_same_slot_retries <= 0):
            return False
        used = self._slot_body_lost_retry_counts.get(slot.slot_id, 0)
        if used >= self.align_body_lost_same_slot_retries:
            return False
        self._slot_body_lost_retry_counts[slot.slot_id] = used + 1
        self.current_target = self._object_for_slot(slot)
        self._plan = None
        self._decide(f"SLOT REAPPROACH F{slot.slot_id} after body lost")
        self.get_logger().info(
            f"Set2 slot F{slot.slot_id}: body lost -> same-slot reapproach "
            f"{used + 1}/{self.align_body_lost_same_slot_retries}"
        )
        self._enter("APPROACH")
        self._appr_tgt_xy = (slot.x, slot.y)
        self._appr_last_seen_s = self._now_s()
        self._require_precise_heading_before_align = True
        self._precise_heading_retry_start_s = self._now_s()
        return True

    def _body_lost_yaw_scan_allowed(self) -> bool:
        slot = self._current_slot()
        if (not self.align_body_lost_yaw_scan_enabled
                or self.align_body_lost_yaw_scan_max_attempts <= 0):
            return False
        robot_xy = self._robot_xy()
        if robot_xy is None:
            return False
        if self._set2_slot_mode() and slot is not None:
            if self._slot_yaw_scan_counts.get(slot.slot_id, 0) >= self.align_body_lost_yaw_scan_max_attempts:
                return False
            if (self.set2_slot_retry_enabled
                    and self._slot_body_lost_retry_counts.get(slot.slot_id, 0)
                    >= self.align_body_lost_same_slot_retries):
                return False
            if slot.distance_to(robot_xy[0], robot_xy[1]) > self.approach_dist_m + 0.25:
                return False
            return self._align_target_still_in_world()

        if self.current_target is None:
            return False
        if self._target_yaw_scan_counts.get(int(self.current_target.id), 0) >= self.align_body_lost_yaw_scan_max_attempts:
            return False
        if not self._align_target_still_in_world() or self.current_target is None:
            return False
        dx = float(self.current_target.x) - robot_xy[0]
        dy = float(self.current_target.y) - robot_xy[1]
        return math.hypot(dx, dy) <= self.approach_dist_m + 0.25

    def _start_body_lost_yaw_scan(self) -> bool:
        if not self._body_lost_yaw_scan_allowed():
            return False
        omega = abs(self.align_body_lost_yaw_scan_omega)
        if omega <= 1e-6:
            return False
        slot = self._current_slot()
        if self._set2_slot_mode() and slot is not None:
            self._slot_yaw_scan_counts[slot.slot_id] = self._slot_yaw_scan_counts.get(slot.slot_id, 0) + 1
            self._decide(f"SLOT YAW SCAN F{slot.slot_id}")
            self.get_logger().info(
                f"Set2 slot F{slot.slot_id}: body lost -> yaw scan "
                f"+/-{self.align_body_lost_yaw_scan_deg:.1f}deg"
            )
        else:
            if self.current_target is None:
                return False
            target_id = int(self.current_target.id)
            self._target_yaw_scan_counts[target_id] = self._target_yaw_scan_counts.get(target_id, 0) + 1
            self._decide(f"TARGET YAW SCAN #{target_id}")
            self.get_logger().info(
                f"Target id={target_id} '{self.current_target.class_label}': body lost -> yaw scan "
                f"+/-{self.align_body_lost_yaw_scan_deg:.1f}deg"
            )
        self._yaw_scan_turn_sec = math.radians(abs(self.align_body_lost_yaw_scan_deg)) / omega
        self._yaw_scan_step = 0
        return self._start_next_yaw_scan_turn()

    def _start_next_yaw_scan_turn(self) -> bool:
        signs = (1.0, -1.0, 1.0)
        multipliers = (1.0, 2.0, 1.0)
        if self._yaw_scan_step >= len(signs):
            return False
        omega = abs(self.align_body_lost_yaw_scan_omega)
        self._yaw_scan_omega_cmd = signs[self._yaw_scan_step] * omega
        self._pulse_sec = self._yaw_scan_turn_sec * multipliers[self._yaw_scan_step]
        self._yaw_scan_step += 1
        self._align_phase = "body_lost_yaw_scan_turn"
        self._align_phase_start = self._now_s()
        self._drive(0.0, 0.0, self._yaw_scan_omega_cmd)
        return True

    def _ready_for_precise_align_retry(self, bearing: float) -> bool:
        if not self._require_precise_heading_before_align or self.world is None:
            return True
        now = self._now_s()
        if self._precise_heading_retry_start_s is None:
            self._precise_heading_retry_start_s = now
        err = self._wrap_pi(float(bearing) - float(self.world.robot_theta))
        if abs(err) <= self.align_retry_heading_tol_rad:
            self._require_precise_heading_before_align = False
            self._precise_heading_retry_start_s = None
            return True
        if (self.align_retry_heading_timeout_sec > 0.0
                and now - self._precise_heading_retry_start_s >= self.align_retry_heading_timeout_sec):
            self.get_logger().warn(
                f"Set2 retry heading timeout ({err:+.2f}rad) -> ALIGN anyway",
                throttle_duration_sec=1.0,
            )
            self._decide("SLOT HEADING TIMEOUT -> ALIGN")
            self._require_precise_heading_before_align = False
            self._precise_heading_retry_start_s = None
            return True
        self._drive_toward_direct(
            float(self.world.robot_x), float(self.world.robot_y), float(bearing)
        )
        return False

    def _retry_current_slot(self, reason: str) -> None:
        slot = self._current_slot()
        if slot is not None:
            if not self.set2_slot_retry_enabled:
                self.slot_inventory.mark_non_target(slot.slot_id, self._now_s())
                slot.physical_state = "lost_suspect"
                self._decide(f"SLOT DROP F{slot.slot_id} ({reason})")
                self.get_logger().warn(
                    f"Set2 slot F{slot.slot_id}: {reason} -> close slot (retry disabled)"
                )
                self.current_target = None
                self._clear_current_slot()
                self.set_type = 0
                self._opportunistic_set2_active = False
                return
            self.slot_inventory.mark_retry(
                slot.slot_id,
                now_s=self._now_s(),
                retry_limit=self.set2_slot_retry_limit,
                retry_cooldown_sec=self.set2_slot_retry_cooldown_sec,
            )
            self._decide(f"SLOT RETRY F{slot.slot_id} ({reason})")
            self.get_logger().warn(
                f"Set2 slot F{slot.slot_id}: {reason} -> retry {slot.retry_count} "
                f"after {self.set2_slot_retry_cooldown_sec:.1f}s"
            )
        self.current_target = None
        self._clear_current_slot()
        self.set_type = 0
        self._opportunistic_set2_active = False

    def _front_target_for_current_slot(self, max_r: float = 0.45) -> Object | None:
        slot = self._current_slot()
        if slot is None or self.world is None:
            return None
        best = None
        best_grab_dist = float(max_r)
        for obj in self.world.objects:
            if obj.blacklisted or int(obj.set_type) != 2:
                continue
            if slot.distance_to(float(obj.x), float(obj.y)) > self.set2_slot_attach_radius_m:
                continue
            base = self._to_base(float(obj.x), float(obj.y))
            if base is None:
                continue
            grab_dist = math.hypot(base[0] - self.grab_x, base[1] - self.grab_y)
            if grab_dist < best_grab_dist:
                best = obj
                best_grab_dist = grab_dist
        if best is not None:
            self.slot_inventory.link_track(slot.slot_id, int(best.id))
        return best

    def _body_base_to_field(self, bx: float, by: float) -> tuple[float, float] | None:
        if self.world is None:
            return None
        ct, st = math.cos(self.world.robot_theta), math.sin(self.world.robot_theta)
        return (
            self.world.robot_x + ct * bx - st * by,
            self.world.robot_y + st * bx + ct * by,
        )

    def _body_base_matches_current_slot(self, bx: float, by: float) -> bool:
        slot = self._current_slot()
        field_xy = self._body_base_to_field(bx, by)
        return bool(
            slot is not None
            and field_xy is not None
            and slot.distance_to(field_xy[0], field_xy[1]) <= self.set2_slot_attach_radius_m
        )

    def _attach_fresh_siglip_to_current_slot(self) -> bool:
        """Attach identity while stopped when the nearest body fruit box matches this slot."""
        slot = self._current_slot()
        if slot is None or self.siglip is None or self.siglip_stamp_s is None:
            return False
        now = self._now_s()
        if self.siglip_stamp_s < self.state_enter_s:
            return False
        if now - self.siglip_stamp_s > self.set2_slot_body_max_age_sec:
            return False
        if (self._body_dets_stamp_s < self.state_enter_s
                or now - self._body_dets_stamp_s > self.set2_slot_body_max_age_sec):
            return False
        if (not bool(self.siglip.image_face_visible)
                or float(self.siglip.confidence) < self.conf_threshold
                or not str(self.siglip.label)):
            return False

        nearest_slot_dist = None
        for u, v, label in self._body_dets:
            if label != "fruit_photo_cube":
                continue
            base = self._body_pixel_base(u, v)
            field_xy = self._body_base_to_field(*base) if base is not None else None
            if field_xy is None:
                continue
            dist = slot.distance_to(field_xy[0], field_xy[1])
            if nearest_slot_dist is None or dist < nearest_slot_dist:
                nearest_slot_dist = dist
        if nearest_slot_dist is None or nearest_slot_dist > self.set2_slot_attach_radius_m:
            return False

        self.slot_inventory.attach_identity(
            slot.slot_id,
            fruit_label=str(self.siglip.label),
            confidence=float(self.siglip.confidence),
            inspected_s=now,
        )
        self._decide(
            f"SLOT F{slot.slot_id}={slot.fruit_label} conf={slot.fruit_confidence:.2f}"
        )
        self.siglip_stamp_s = None
        return True

    def _publish_slot_debug(self) -> None:
        payload = {
            "enabled": self.set2_slot_enabled,
            "current_slot_id": self.current_slot_id or 0,
            "slots": self.slot_inventory.debug_dicts() if self.set2_slot_enabled else [],
        }
        self.pub_slots.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def _opportunistic_set2_quota_available(self) -> bool:
        if not self.opportunistic_set2_enabled or not self.set2_label or self.phase != 1:
            return False
        if self.opportunistic_set2_max_picks <= 0:
            return True
        return self.tray_fruit < self.opportunistic_set2_max_picks

    def _nearest_opportunistic_set2(self) -> Object | None:
        """A target Set2 fruit cube worth interrupting Set1 for: already SigLIP-confirmed, nearby,
        in the active zone, and re-seen after the zone stabilize gate when that gate is enabled."""
        if self.world is None or not self._opportunistic_set2_quota_available():
            return None
        rx, ry = self.world.robot_x, self.world.robot_y
        best = None
        bestd = self.opportunistic_set2_max_dist_m
        for o in self.world.objects:
            if o.blacklisted or int(o.set_type) != 2:
                continue
            if self.set2_slot_enabled:
                slot = self._slot_for_object(o)
                if slot is None or not slot.actionable:
                    continue
            if self.set2_require_fruit_label and str(o.fruit_label) != self.set2_label:
                continue
            if float(o.confidence) < self.opportunistic_set2_min_conf:
                continue
            if int(o.n_obs) < self.opportunistic_set2_min_nobs:
                continue
            if self.zone_mission_enabled and not self._object_in_active_zone(o):
                continue
            if self.zone_mission_enabled and not self._object_seen_after_zone_stabilize(o):
                continue
            d = math.hypot(float(o.x) - rx, float(o.y) - ry)
            if d < bestd:
                bestd = d
                best = o
        return best

    def _current_object_for_approach(self, ref_xy: tuple[float, float] | None = None) -> Object | None:
        if self._set2_slot_mode() and (slot := self._current_slot()) is not None:
            return self._object_for_slot(slot)
        if not self._opportunistic_set2_active:
            if self.current_target is not None:
                fresh = self._lookup_object(int(self.current_target.id))
                if (fresh is not None
                        and not fresh.blacklisted
                        and int(fresh.set_type) == int(self.current_target.set_type)):
                    if int(fresh.set_type) == 1 and str(fresh.class_label) == self.set1_label:
                        return fresh
                    if int(fresh.set_type) == 2:
                        fl = str(fresh.fruit_label)
                        if not self.set2_require_fruit_label or not fl or fl == self.set2_label:
                            return fresh
            if ref_xy is None and self.current_target is not None:
                ref_xy = (float(self.current_target.x), float(self.current_target.y))
            return self._nearest_phase_object(ref_xy=ref_xy)
        if self.world is None:
            return None
        candidates = []
        if self.current_target is not None:
            fresh = self._lookup_object(int(self.current_target.id))
            if fresh is not None:
                candidates.append(fresh)
        candidates.extend(o for o in self.world.objects if int(o.set_type) == 2)
        rx, ry = ref_xy if ref_xy is not None else (
            (self.current_target.x, self.current_target.y) if self.current_target is not None
            else (self.world.robot_x, self.world.robot_y)
        )
        best = None
        bestd = 1e9
        for o in candidates:
            if o.blacklisted or int(o.set_type) != 2:
                continue
            if self.set2_require_fruit_label and str(o.fruit_label) != self.set2_label:
                continue
            if float(o.confidence) < self.opportunistic_set2_min_conf:
                continue
            d = math.hypot(float(o.x) - rx, float(o.y) - ry)
            if d < bestd:
                bestd = d
                best = o
        return best

    def on_body_dets(self, msg: DetectionArray) -> None:
        # Keep the raw body-cam LABEL: ALIGN must only align to the target SHAPE (e.g. icosahedron),
        # not any set_type-1 shape. If the body cam says 'cube' it is NOT the set1 target -> ignore it.
        self._body_dets = [(float(d.x_center), float(d.y_center), str(d.label))
                           for d in msg.detections]
        self._body_dets_stamp_s = self._now_s()

    def _body_pixel_base(self, u: float, v: float) -> tuple[float, float] | None:
        if self._body_H is None:
            return None
        us, vs = float(u) * self._body_px_sx, float(v) * self._body_px_sy
        p = cv2.perspectiveTransform(np.array([[[us, vs]]], np.float64), self._body_H)[0][0]
        k = self._boh / self._bH
        return float(p[0]) - k * (float(p[0]) - self._bnx), float(p[1]) - k * float(p[1])

    def _body_target_base(self, want_label: str) -> tuple[float, float] | None:
        """POSE-INDEPENDENT servo target: project each body detection whose LABEL == want_label to
        base_link (body homography + height correction), return the one nearest the grab point. Only
        the target shape/kind is aligned to — a body 'cube' in front of a set1(icosahedron) run is NOT
        a target, so ALIGN gets None and moves on instead of grabbing the wrong object."""
        if self._body_H is None or not self._body_dets:
            return None
        allowed_labels = self._body_align_labels()
        best = None
        bestd = 0.5
        for u, v, label in self._body_dets:
            if label not in allowed_labels:
                continue
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            bx, by = base
            if self._set2_slot_mode() and self._current_slot() is not None:
                if not self._body_base_matches_current_slot(bx, by):
                    continue
            d = math.hypot(bx - self.grab_x, by - self.grab_y)
            if d < bestd:
                bestd = d
                best = (bx, by)
        return best

    def _body_nearest_any(self) -> tuple[str, float, float] | None:
        """Nearest body detection of ANY label to the grab point -> (label, bx, by). Lets ALIGN spot
        a DISTRACTOR (e.g. a cube) sitting where the target was expected and skip that spot fast."""
        if self._body_H is None or not self._body_dets:
            return None
        best = None
        bestd = 0.25   # only "at the grab point" counts as blocking
        for u, v, label in self._body_dets:
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            bx, by = base
            d = math.hypot(bx - self.grab_x, by - self.grab_y)
            if d < bestd:
                bestd = d
                best = (label, bx, by)
        return best

    def _body_set1_distractor_in_front(self) -> tuple[str, float, float] | None:
        """Set2 slot guard: any Set1 body detection in the near front view means the slot is wrong."""
        if self._body_H is None or not self._body_dets:
            return None
        best = None
        bestd = float("inf")
        max_x = max(0.0, self.set2_slot_distractor_front_x_m)
        max_y = max(0.0, self.set2_slot_distractor_side_y_m)
        for u, v, label in self._body_dets:
            if _LABEL_ST.get(str(label), 0) != 1:
                continue
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            bx, by = base
            if 0.0 <= bx <= max_x and abs(by) <= max_y:
                d = math.hypot(bx - self.grab_x, by - self.grab_y)
                if d < bestd:
                    bestd = d
                    best = (str(label), bx, by)
        return best

    def _align_target_still_in_world(self) -> bool:
        """True when the latched ALIGN target is still visible in the wide/world map."""
        if self._set2_slot_mode() and (slot := self._current_slot()) is not None:
            fresh = self._object_for_slot(slot)
            if fresh is not None:
                self.current_target = fresh
                return True
            return False
        if self.current_target is None or self.world is None:
            return False
        fresh = self._lookup_object(int(self.current_target.id))
        if fresh is not None and not fresh.blacklisted:
            self.current_target = fresh
            return True
        ref = (float(self.current_target.x), float(self.current_target.y))
        return self._current_object_for_approach(ref_xy=ref) is not None

    def _drive(self, vx: float, vy: float, omega: float = 0.0) -> None:
        c = BaseCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = "base_link"
        c.vx, c.vy, c.omega = float(vx), float(vy), float(omega)
        self.pub_cmd.publish(c)

    def _drive_toward_direct(self, dest_x: float, dest_y: float, yaw: float = 0.0) -> None:
        """Drive to a field waypoint by publishing /base_command directly from the FSM.

        This follows the same command ownership model as the OPENING routine: rotate in place until
        the waypoint is in front of the robot, then drive forward.
        """
        if self.world is None:
            self._drive(0.0, 0.0, 0.0)
            return
        base = self._to_base(dest_x, dest_y)
        if base is None:
            self._drive(0.0, 0.0, 0.0)
            return
        bx, by = base
        dist = math.hypot(bx, by)
        if dist <= self.direct_nav_stop_radius_m:
            yaw_err = self._wrap_pi(float(yaw) - float(self.world.robot_theta))
            if abs(yaw_err) <= self.direct_nav_face_tol:
                self._drive(0.0, 0.0, 0.0)
            else:
                om = max(
                    -self.direct_nav_omega_max,
                    min(self.direct_nav_omega_max, self.direct_nav_kp_ang * yaw_err),
                )
                self._drive(0.0, 0.0, om)
            return
        heading_err = math.atan2(by, bx)
        om = max(
            -self.direct_nav_omega_max,
            min(self.direct_nav_omega_max, self.direct_nav_kp_ang * heading_err),
        )
        if abs(heading_err) > self.direct_nav_face_tol:
            self._drive(0.0, 0.0, om)
        else:
            self._drive(max(0.0, self.direct_nav_speed), 0.0, 0.0)

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _robot_xy(self) -> tuple[float, float] | None:
        if self.world is None:
            return None
        return (self.world.robot_x, self.world.robot_y)

    def _hold_current_goal(self, yaw: float | None = None) -> None:
        """Hold position, optionally turning in place to the requested field heading."""
        if self.world is None:
            self._drive(0.0, 0.0)
            return
        th = float(self.world.robot_theta) if yaw is None else float(yaw)
        self._drive_toward_direct(float(self.world.robot_x), float(self.world.robot_y), th)

    def _face_heading(self, tx: float, ty: float) -> float:
        """Field heading that points the robot (its forward/body cam) AT the target (tx,ty).
        Approaching with this yaw keeps the object in the forward body cam so ALIGN can see it."""
        if self.world is None:
            return 0.0
        return math.atan2(ty - self.world.robot_y, tx - self.world.robot_x)

    def _distance_to(self, x: float, y: float) -> float | None:
        rxy = self._robot_xy()
        if rxy is None:
            return None
        return math.hypot(x - rxy[0], y - rxy[1])

    def _active_zone_id(self) -> int:
        if not self.zone_order:
            return 0
        return self.zone_order[self._zone_idx % len(self.zone_order)]

    def _zone_center(self, zone_id: int | None = None) -> tuple[float, float]:
        zid = self._active_zone_id() if zone_id is None else int(zone_id)
        if zid in self.zone_anchors:
            return self.zone_anchors[zid]
        xmin, xmax, ymin, ymax = self.zone_bounds.get(zid, (-2.0, 2.0, -2.0, 2.0))
        return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5)

    def _object_in_active_zone(self, obj: Object) -> bool:
        if not self.zone_mission_enabled:
            return True
        xmin, xmax, ymin, ymax = self.zone_bounds.get(self._active_zone_id(), (-2.0, 2.0, -2.0, 2.0))
        return xmin <= float(obj.x) <= xmax and ymin <= float(obj.y) <= ymax

    def _reset_zone_scan_timer(self) -> None:
        self._zone_no_target_since = None

    def _zone_is_stabilized(self) -> bool:
        if not self.zone_mission_enabled or not self.zone_stabilize_enabled:
            return True
        return self._zone_stabilized_idx == self._zone_idx

    def _zone_ready_for_selection(self) -> bool:
        if not self.zone_mission_enabled:
            return True
        if not self.zone_anchor_nav_enabled:
            return self._zone_is_stabilized()
        return self._zone_anchor_reached_idx == self._zone_idx and self._zone_is_stabilized()

    @staticmethod
    def _time_msg_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _object_seen_after_zone_stabilize(self, obj: Object) -> bool:
        if (not self.zone_stabilize_require_fresh_seen
                or not self.zone_stabilize_enabled
                or self._zone_stabilized_idx != self._zone_idx):
            return True
        return self._time_msg_to_sec(obj.last_seen) >= self._zone_stabilized_after_s

    def _advance_zone_or_phase(self) -> None:
        if not self.zone_mission_enabled:
            return
        prev = self._active_zone_id()
        if self._zone_idx + 1 < len(self.zone_order):
            self._zone_idx += 1
            self._plan = None
            self._coverage = None
            self.current_target = None
            self._clear_current_slot()
            self._opportunistic_set2_active = False
            self._zone_stabilized_idx = None
            self._zone_anchor_reached_idx = None
            self._zone_stabilized_after_s = 0.0
            self._reset_zone_scan_timer()
            self.get_logger().info(f"zone {prev} done -> zone {self._active_zone_id()}")
            self._decide(f"ZONE {prev}->{self._active_zone_id()}")
            self._enter("SCAN")
            return
        self._zone_idx = 0
        self._clear_current_slot()
        self._opportunistic_set2_active = False
        self._zone_stabilized_idx = None
        self._zone_anchor_reached_idx = None
        self._zone_stabilized_after_s = 0.0
        self._reset_zone_scan_timer()
        self.get_logger().info(f"zone {prev} done -> phase sweep complete")
        self._decide(f"ZONE {prev} complete")
        self._advance_phase_or_end()

    # ------------------------------------------------------ lane-graph planning
    def _obstacles_snapshot(self, exclude_id: int = 0,
                            dest_xy: tuple[float, float] | None = None) -> list[tuple[float, float]]:
        """Physical obstacles (field xy) for the planner from the current world snapshot. Drops the
        target being approached (by id) + anything within exclude_target_radius of the destination
        (so the final leg is reachable) + storage flags; keeps blacklisted objects (still there)."""
        if self.world is None:
            return []
        out = []
        for o in self.world.objects:
            if int(o.set_type) == 3:                          # storage flag, not a body
                continue
            if int(o.id) == int(exclude_id):
                continue
            if float(o.confidence) < self.obstacle_min_conf or int(o.n_obs) < self.obstacle_min_nobs:
                continue
            if dest_xy is not None and math.hypot(o.x - dest_xy[0], o.y - dest_xy[1]) < self.exclude_target_radius_m:
                continue
            out.append((float(o.x), float(o.y)))
        return out

    def _base_obstacles(
        self,
        obstacles: list[tuple[float, float]],
    ) -> list[tuple[float, float, float, float]]:
        """Obstacle list as (field_x, field_y, base_x, base_y)."""
        if self.world is None:
            return []
        out: list[tuple[float, float, float, float]] = []
        for ox, oy in obstacles:
            base = self._to_base(ox, oy)
            if base is None:
                continue
            bx, by = base
            out.append((float(ox), float(oy), float(bx), float(by)))
        return out

    def _front_escape_waypoint(
        self,
        obstacles: list[tuple[float, float]],
    ) -> tuple[float, float, int] | None:
        """Return a latched side-step waypoint when a large obstacle blocks the front."""
        if not self.front_escape_enabled or self.world is None:
            return None
        base_obs = self._base_obstacles(obstacles)
        front = [
            item for item in base_obs
            if self.front_escape_x_min_m <= item[2] <= self.front_escape_x_max_m
            and abs(item[3]) <= self.front_escape_y_abs_m
        ]
        if not front:
            return None

        left_penalty = 0.0
        right_penalty = 0.0
        for _ox, _oy, bx, by in base_obs:
            if bx < -0.10 or bx > self.front_escape_x_max_m + 0.35:
                continue
            weight = 1.0 / max(0.08, math.hypot(bx, by))
            if by >= 0.0:
                left_penalty += weight
            else:
                right_penalty += weight
        # +1 means robot-left escape; -1 means robot-right escape.
        sign = 1 if left_penalty <= right_penalty else -1
        step = max(0.05, abs(self.front_escape_step_m))
        fwd = max(0.0, self.front_escape_forward_m)
        th = float(self.world.robot_theta)
        ct, st = math.cos(th), math.sin(th)
        ex = float(self.world.robot_x) + fwd * ct - sign * step * st
        ey = float(self.world.robot_y) + fwd * st + sign * step * ct
        self._decide(
            f"FRONT ESCAPE {'L' if sign > 0 else 'R'} "
            f"front={len(front)} L={left_penalty:.1f} R={right_penalty:.1f}"
        )
        return ex, ey, sign

    def _publish_planning_obstacles(self, obstacles: list[tuple[float, float]]) -> None:
        """Share the exact obstacle set used for lane planning with local go-to-goal avoidance."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        for ox, oy in obstacles:
            pose = Pose()
            pose.position.x = float(ox)
            pose.position.y = float(oy)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub_planning_obstacles.publish(msg)

    def _drive_toward(self, dest_x: float, dest_y: float, yaw: float = 0.0, exclude_id: int = 0) -> None:
        """Route to (dest_x,dest_y) and directly command the active lane waypoint each tick."""
        dest = (float(dest_x), float(dest_y))
        obstacles = self._obstacles_snapshot(exclude_id, dest)
        self._publish_planning_obstacles(obstacles)
        if not self.planner_enabled:
            self._drive_toward_direct(dest_x, dest_y, yaw)
            return
        rxy = self._robot_xy()
        if rxy is None:
            self._drive(0.0, 0.0, 0.0)
            return
        now = self._now_s()
        drifted = (self._plan_dest is None
                   or math.hypot(dest[0] - self._plan_dest[0], dest[1] - self._plan_dest[1]) > self.replan_goal_move_m)
        stale = now - self._plan_stamp > self.replan_period_sec
        # No plan at all -> MUST plan now (ignore throttle); drift/stale replans are throttled.
        if self._plan is None or ((drifted or stale) and now - self._last_replan_t >= self.replan_throttle_sec):
            self._last_replan_t = now
            vias = self._planner.plan(rxy, dest, obstacles)
            self._plan_escape = False
            if vias:
                self._plan = vias
            else:
                escape = self._front_escape_waypoint(obstacles)
                if escape is not None:
                    self._plan = [(escape[0], escape[1])]
                    self._plan_escape = True
                elif self.direct_fallback_enabled:
                    self._plan = [dest]     # no lane route and no front block -> direct fallback
                else:
                    self._plan = []         # lane-only mode: stop and wait for a future replan
            self._plan_idx = 0
            self._plan_dest = dest
            self._plan_stamp = now
        if not self._plan:                            # safety: never index a None/empty plan
            self._drive(0.0, 0.0, 0.0)
            return
        # advance monotonically past intermediate vias already reached
        while self._plan_idx < len(self._plan) - 1:
            wx, wy = self._plan[self._plan_idx]
            d = self._distance_to(wx, wy)
            if d is not None and d < self.wp_reach_tol_m:
                self._plan_idx += 1
            else:
                break
        wx, wy = self._plan[self._plan_idx]
        if self._plan_escape:
            d_escape = self._distance_to(wx, wy)
            if d_escape is not None and d_escape < self.wp_reach_tol_m:
                self._plan = None
                self._plan_escape = False
                self._drive(0.0, 0.0, 0.0)
                return
        last = self._plan_idx == len(self._plan) - 1
        # Intermediate vias keep the current heading target; the final via carries the requested yaw.
        via_yaw = yaw if last else (self.world.robot_theta if self.world is not None else yaw)
        self._drive_toward_direct(wx, wy, via_yaw)

    def _publish_pick(self, trigger: bool) -> None:
        self.pub_pick.publish(Bool(data=trigger))

    def _blacklist(self, obj_id: int) -> None:
        if obj_id != 0:
            self.pub_blacklist.publish(UInt64(data=int(obj_id)))

    def _decide(self, text: str) -> None:
        """Publish a human-readable decision for the visualiser's decision feed."""
        self.pub_decision.publish(String(data=text))

    def _set_world_mapping_enabled(self, enabled: bool, reason: str = "") -> None:
        enabled = bool(enabled)
        if enabled == self._world_mapping_enabled:
            return
        self._world_mapping_enabled = enabled
        suffix = f" ({reason})" if reason else ""
        self.get_logger().info(
            f"world model object mapping {'enabled' if enabled else 'disabled'}{suffix}"
        )

    def _count_non_blacklisted(self) -> int:
        if self.world is None:
            return 0
        return sum(1 for o in self.world.objects if not o.blacklisted)

    # --------------------------------------------------------------- callbacks
    def on_world(self, msg: WorldModel) -> None:
        self.world = msg
        self._got_world = True
        if self.set2_slot_enabled:
            for obj in msg.objects:
                seen_s = self._time_msg_to_sec(obj.last_seen)
                slot_grid = (
                    self._nearest_set2_slot_grid(float(obj.x), float(obj.y))
                    if self.set2_slot_grid_lock_enabled
                    else None
                )
                self.slot_inventory.add_or_merge_observation(
                    track_id=int(obj.id),
                    set_type=int(obj.set_type),
                    x=float(obj.x),
                    y=float(obj.y),
                    confidence=float(obj.confidence),
                    n_obs=int(obj.n_obs),
                    class_label=str(obj.class_label),
                    source=str(obj.source),
                    seen_s=seen_s,
                    blacklisted=bool(obj.blacklisted),
                    allow_create=self._allow_new_set2_slot(obj, seen_s),
                    fixed_xy=(slot_grid[1], slot_grid[2]) if slot_grid is not None else None,
                    grid_id=slot_grid[0] if slot_grid is not None else None,
                    position_locked=slot_grid is not None,
                )

    def on_wall_map_transform(self, msg: Float32MultiArray) -> None:
        if not self.set2_slot_enabled or len(msg.data) < 3:
            return
        self.slot_inventory.apply_transform(
            float(msg.data[0]), float(msg.data[1]), float(msg.data[2]),
            include_grid_locked=self.set2_slot_follow_map_transform,
        )

    def on_target(self, msg: Object) -> None:
        self.selected = msg

    def on_siglip(self, msg: Classification) -> None:
        self.siglip = msg
        self.siglip_stamp_s = self._now_s()

    def on_shape(self, msg: Classification) -> None:
        self.shape = msg
        self.shape_stamp_s = self._now_s()

    def on_advance(self, _: Empty) -> None:
        """Manual debug/dry-run override: force one transition along the chain."""
        idx = STATES.index(self.state)
        if self.state == "STORE_IN_TRAY":
            self._do_store_in_tray()
        elif self.state == "CLASSIFY":
            # force a PICK so the downstream chain can be exercised
            self.set_type = self.current_target.set_type if self.current_target else 0
            if not self.dry_pick:
                self._publish_pick(True)
            self._enter("PICK")
        elif self.state == "DUMP_ALL":
            self._enter("END")
        elif idx + 1 < len(STATES):
            self._enter(STATES[idx + 1])

    # ------------------------------------------------------------ state helper
    def _do_store_in_tray(self) -> None:
        """Tray bookkeeping + blacklist picked object, then branch (phase-aware)."""
        if self.set_type == 1:
            self.tray_shape += 1
        elif self.set_type == 2:
            self.tray_fruit += 1
            if (slot := self._current_slot()) is not None:
                self.slot_inventory.mark_picked(slot.slot_id, self._now_s())
                self._decide(f"SLOT PICKED F{slot.slot_id}")
        if self.current_target is not None:
            self._blacklist(self.current_target.id)   # picked -> never reselect
        self.current_target = None
        self._clear_current_slot()
        self.set_type = 0
        self._opportunistic_set2_active = False

        # In the legacy phased flow, advance to Set2 once Set1 quota is met. Mixed mode keeps
        # selecting from both quotas and only uses phase as the current target type.
        if (not self.mixed_target_mode
                and self.phase == 1
                and self.tray_shape >= self.shape_target_total
                and self.fruit_target_total > 0):
            self.phase = 2
            self._zone_idx = 0
            self._clear_current_slot()
            self._opportunistic_set2_active = False
            self._zone_stabilized_idx = None
            self._zone_anchor_reached_idx = None
            self._zone_stabilized_after_s = 0.0
            self._reset_zone_scan_timer()
            self.get_logger().info("Set1 quota met -> phase 2 (Set2)")
            self._decide("PHASE 1->2 (Set1 quota met)")

        both_met = (
            self.tray_shape >= self.shape_target_total
            and self.tray_fruit >= self.fruit_target_total
        )
        if both_met and self.end_after_quota:
            self.get_logger().info("both quotas met (test mode) -> END")
            self._decide("END (quotas met)")
            self._enter("END")
        elif both_met:
            self._enter("DRIVE_TO_STORAGE")
        else:
            self._enter("SELECT_TARGET")

    # -------------------------------------------------------------- transitions
    def _step(self) -> None:
        """Condition-driven transition for the current state (one tick)."""
        if self.state == "OPENING":
            self._step_opening()

        elif self.state == "SCAN":
            if self.zone_mission_enabled and self.zone_anchor_nav_enabled:
                if self._zone_anchor_reached_idx != self._zone_idx:
                    zx, zy = self._zone_center()
                    d_zone = self._distance_to(zx, zy)
                    if d_zone is None or d_zone > self.zone_center_reach_tol_m:
                        self._reset_zone_scan_timer()
                        th = self.world.robot_theta if self.world is not None else 0.0
                        self._drive_toward(zx, zy, th, exclude_id=0)
                        return
                    self._zone_anchor_reached_idx = self._zone_idx
                    self._decide(f"ZONE {self._active_zone_id()} ANCHOR REACHED")
                if not self._zone_is_stabilized():
                    self._drive(0.0, 0.0)
                    self._decide(f"ZONE {self._active_zone_id()} STABILIZE")
                    self._enter("ZONE_STABILIZE")
                    return
            phase_target_available = (
                self._select_next_mixed_target() is not None
                if self.mixed_target_mode
                else (
                    self._select_next_set2_slot() is not None
                    if self.phase == 2 and self.set2_slot_enabled
                    else self._nearest_phase_object() is not None
                )
            )
            if phase_target_available:
                self._reset_zone_scan_timer()
                self._enter("SELECT_TARGET")                   # phase target in view -> pursue it
            else:
                if ((self.phase == 2 and self.set2_slot_enabled and self._has_unresolved_set2_slot())
                        or self._has_unresolved_mixed_set2_slot()):
                    self._reset_zone_scan_timer()               # retry cooldown: keep this zone alive
                    self._search_step()
                    return
                if self.zone_mission_enabled:
                    now = self._now_s()
                    if self._zone_no_target_since is None:
                        self._zone_no_target_since = now
                    elif now - self._zone_no_target_since >= self.zone_no_target_advance_sec:
                        self._advance_zone_or_phase()
                        return
                self._patrol_search()                          # not in view -> patrol the field to find it

        elif self.state == "ZONE_STABILIZE":
            self._drive(0.0, 0.0)
            if self._time_in_state() >= self.zone_stabilize_sec:
                self._zone_stabilized_idx = self._zone_idx
                self._zone_stabilized_after_s = self._zone_stabilize_start_s
                self._reset_zone_scan_timer()
                self.get_logger().info(
                    f"zone {self._active_zone_id()} stabilized for {self.zone_stabilize_sec:.1f}s -> SCAN")
                self._decide(f"ZONE {self._active_zone_id()} STABILIZED")
                self._enter("SCAN")

        elif self.state == "SELECT_TARGET":
            if not self._zone_ready_for_selection():
                self._enter("SCAN")
                return
            if self.mixed_target_mode:
                mixed = self._select_next_mixed_target()
                if mixed is not None and self._latch_mixed_target(mixed[0], mixed[1]):
                    self._enter("APPROACH")
                else:
                    self._enter("SCAN")
                return
            opp = self._nearest_opportunistic_set2()
            if opp is not None:
                self._reset_zone_scan_timer()
                self._opportunistic_set2_active = True
                if self.set2_slot_enabled and (slot := self._slot_for_object(opp)) is not None:
                    self._latch_set2_slot(slot, decision_prefix="OPP SLOT")
                else:
                    self.current_target = opp
                self.get_logger().info(
                    f"opportunistic Set2 target #{opp.id} '{opp.fruit_label}' while staying in Set1 phase")
                self._decide(f"OPP SET2 {opp.fruit_label} #{opp.id}")
                self._enter("APPROACH")
                return
            if self.phase == 2 and self.set2_slot_enabled:
                slot = self._select_next_set2_slot()
                if slot is not None:
                    self._reset_zone_scan_timer()
                    self._opportunistic_set2_active = False
                    self._latch_set2_slot(slot)
                    self._enter("APPROACH")
                else:
                    self._enter("SCAN")
                return
            tgt = self._nearest_phase_object()         # nearest visible object of this kind
            if tgt is not None:
                self._reset_zone_scan_timer()
                self.current_target = tgt
                self._opportunistic_set2_active = False
                self._enter("APPROACH")
            else:
                self._enter("SCAN")                    # not in view -> go PATROL to find it (never give up)

        elif self.state == "APPROACH":
            if not self.mixed_target_mode and not self._opportunistic_set2_active:
                opp = self._nearest_opportunistic_set2()
                if opp is not None:
                    self._opportunistic_set2_active = True
                    if self.set2_slot_enabled and (slot := self._slot_for_object(opp)) is not None:
                        self._latch_set2_slot(slot, decision_prefix="OPP SLOT INTERRUPT")
                        self._appr_tgt_xy = (slot.x, slot.y)
                    else:
                        self.current_target = opp
                        self._appr_tgt_xy = (opp.x, opp.y)
                    self._appr_last_seen_s = self._now_s()
                    self._plan = None
                    self.get_logger().info(
                        f"APPROACH interrupt: opportunistic Set2 target #{opp.id} '{opp.fruit_label}'")
                    self._decide(f"OPP SET2 INTERRUPT {opp.fruit_label} #{opp.id}")
            # Stay COMMITTED to the latched object: pick the phase object nearest the LATCHED xy (not
            # the globally nearest), so we don't shuttle to whichever cube is momentarily closest.
            slot = self._current_slot() if self._set2_slot_mode() else None
            if slot is not None:
                tgt = self._object_for_slot(slot)
                if tgt is not None:
                    self.current_target = tgt
                    self._appr_last_seen_s = self._now_s()
                self._appr_tgt_xy = (slot.x, slot.y)
            else:
                tgt = self._current_object_for_approach(ref_xy=self._appr_tgt_xy)
                if tgt is not None:
                    self.current_target = tgt
                    self._appr_tgt_xy = (tgt.x, tgt.y)
                    self._appr_last_seen_s = self._now_s()
                elif (self._appr_tgt_xy is None
                      or self._now_s() - self._appr_last_seen_s > self.approach_lost_grace_sec):
                    self._enter("SCAN")                # nothing of this kind visible -> look / go centre
                    return
            # Car-like: aim the STAND-OFF along the ROBOT->TARGET line and face the target, so the body
            # cam sees it head-on on arrival. Route there via collision-free lanes (target excluded so
            # the final leg is reachable). stand-off = target - approach_dist*(cos,sin) of the bearing.
            gx, gy = self._appr_tgt_xy
            bearing = self._face_heading(gx, gy)
            sx = gx - self.approach_dist_m * math.cos(bearing)
            sy = gy - self.approach_dist_m * math.sin(bearing)
            exclude = self.current_target.id if self.current_target is not None else 0
            self._drive_toward(sx, sy, bearing, exclude_id=exclude)
            d = self._distance_to(sx, sy)
            if d is not None and d < self.approach_standoff_tol:
                if not self._ready_for_precise_align_retry(bearing):
                    return
                # phase 2: at the stand-off, let SigLIP type the fruit BEFORE aligning — don't waste a
                # full align on a non-orange. Orange -> align now; still untyped -> wait briefly then
                # align closer; a typed non-orange is already dropped by _nearest_phase_object.
                if self._set2_slot_mode() and self.set2_require_fruit_label:
                    self._hold_current_goal(bearing)
                    if self._standoff_arrived_s is None:
                        self._standoff_arrived_s = self._now_s()
                    self._attach_fresh_siglip_to_current_slot()
                    slot = self._current_slot()
                    if slot is not None and slot.fruit_label:
                        if slot.fruit_label != self.set2_label:
                            self.slot_inventory.mark_non_target(slot.slot_id, self._now_s())
                            if self.current_target is not None:
                                self._blacklist(self.current_target.id)
                            self._decide(
                                f"SLOT NON-TARGET F{slot.slot_id} {slot.fruit_label}"
                            )
                            self.current_target = None
                            self._clear_current_slot()
                            self._opportunistic_set2_active = False
                            self._enter("SELECT_TARGET")
                            return
                    if slot is not None and slot.fruit_label == self.set2_label:
                        self._enter("ALIGN")
                        return
                    if self._now_s() - self._standoff_arrived_s < self.classify_standoff_sec:
                        return                      # hold at stand-off, wait for the fruit type
                elif ((self.phase == 2 or self._opportunistic_set2_active)
                        and self.set2_require_fruit_label
                        and tgt is not None and str(tgt.fruit_label) != self.set2_label):
                    self._hold_current_goal(bearing)
                    if self._standoff_arrived_s is None:
                        self._standoff_arrived_s = self._now_s()
                    if self._now_s() - self._standoff_arrived_s < self.classify_standoff_sec:
                        return
                self._enter("ALIGN")

        elif self.state == "ALIGN":
            self._step_align()

        elif self.state == "CLASSIFY":
            self._step_classify()

        elif self.state == "PICK":
            if self._time_in_state() >= self.pick_duration_sec:
                self._enter("STORE_IN_TRAY")

        elif self.state == "STORE_IN_TRAY":
            self._do_store_in_tray()

        elif self.state == "DRIVE_TO_STORAGE":
            th = self.world.robot_theta if self.world is not None else 0.0
            self._drive_toward(self.storage_x, self.storage_y, th, exclude_id=0)  # collision-free to bin
            d = self._distance_to(self.storage_x, self.storage_y)
            if d is not None and d < self.approach_dist_m:
                self._enter("ALIGN_OVER_BIN")

        elif self.state == "ALIGN_OVER_BIN":
            if self._time_in_state() >= self.align_settle_sec:
                self._enter("DUMP_ALL")

        elif self.state == "DUMP_ALL":
            if not self.dry_pick:
                self._publish_pick(False)   # release / dump tray
            if self._time_in_state() >= self.pick_duration_sec:
                self.get_logger().info("DUMP_ALL complete -> END")
                self._enter("END")

        # END: terminal, no transition

    def _commit_pick(self, set_type: int, label: str, detail: str) -> None:
        """Enter PICK for a confirmed target. In dry_pick mode, log instead of firing the arm."""
        self.set_type = set_type
        obj_id = self.current_target.id if self.current_target else 0
        if self.dry_pick:
            self.get_logger().info(
                f"[DRY PICK] picked id={obj_id} class='{label}' set={set_type} ({detail})"
            )
        else:
            self._publish_pick(True)
            self.get_logger().info(f"GATE PICK set{set_type} '{label}' ({detail})")
        self._decide(f"{'DRY-' if self.dry_pick else ''}PICK set{set_type} {label} #{obj_id} ({detail})")
        self._enter("PICK")

    def _advance_phase_or_end(self) -> None:
        """No phase-appropriate target left: Set1 phase -> Set2 phase, or Set2 phase -> END."""
        if self.mixed_target_mode:
            self.get_logger().info("mixed target sweep exhausted -> storage/end")
            self._decide("END (mixed targets exhausted)")
            if (self.tray_shape > 0 or self.tray_fruit > 0) and not self.end_after_quota:
                self._enter("DRIVE_TO_STORAGE")
            else:
                self._enter("END")
            return
        if self.phase == 1:
            if self.fruit_target_total <= 0:
                self.get_logger().info("phase 1 exhausted and Set2 quota is 0 -> storage/end")
                self._decide("END SET2 DISABLED")
                if self.tray_shape > 0 and not self.end_after_quota:
                    self._enter("DRIVE_TO_STORAGE")
                else:
                    self._enter("END")
                return
            self.phase = 2
            self._zone_idx = 0
            self._clear_current_slot()
            self._opportunistic_set2_active = False
            self._zone_stabilized_idx = None
            self._zone_anchor_reached_idx = None
            self._zone_stabilized_after_s = 0.0
            self._reset_zone_scan_timer()
            self.get_logger().info("phase 1 (Set1) exhausted -> phase 2 (Set2)")
            self._decide("PHASE 1->2 (Set1 exhausted)")
            self._enter("SELECT_TARGET")   # reset the timer and re-select for Set2
        else:
            self.get_logger().info("phase 2 (Set2) exhausted -> END")
            self._decide("END (Set2 exhausted)")
            self._enter("END")

    def _skip_target(self, reason: str) -> None:
        """Abandon the current target WITHOUT blacklisting (wrong phase; keep for later)."""
        obj_id = self.current_target.id if self.current_target else 0
        self.get_logger().info(f"skip id={obj_id} ({reason}) -> reselect (not blacklisted)")
        self._decide(f"SKIP #{obj_id} ({reason})")
        self.current_target = None
        self._clear_current_slot()
        self.set_type = 0
        self._opportunistic_set2_active = False
        self._enter("SELECT_TARGET")

    def _step_opening(self) -> None:
        """Hardcoded match opening: wait -> forward -> right strafe -> turn -> timed hold -> SCAN.

        BODY-frame commands are timed; state_enter_s is reset at each leg so _time_in_state()
        measures that leg only. The release gate uses match elapsed time from node startup.
        """
        self._set_world_mapping_enabled(False, "opening move / wall settle")
        t = self._time_in_state()
        leg = self._opening_leg
        if leg == "wait":
            # Hold briefly, then run the opening while YOLO/world_model are still warming up.
            self._drive(0.0, 0.0)
            warm = (self._now_s() - self._node_start_s) >= self.startup_warmup_sec
            if warm:
                self.get_logger().info(
                    f"startup hold done -> opening move "
                    f"(forward + right strafe + {self.opening_turn_deg:.0f}deg turn)"
                )
                self._opening_leg = "forward"
                self.state_enter_s = self._now_s()
        elif leg == "forward":
            if t < self.opening_forward_sec:
                self._drive(self.opening_speed, 0.0)          # +vx = robot forward
            else:
                self._drive(0.0, 0.0)
                self._opening_leg = "strafe_right"
                self.state_enter_s = self._now_s()
        elif leg == "strafe_right":
            if t < self.opening_strafe_right_sec:
                self._drive(0.0, -self.opening_strafe_speed)  # -vy = robot right
            else:
                self._drive(0.0, 0.0)
                self._opening_leg = "turn"
                self._opening_turn_start_theta = (
                    float(self.world.robot_theta) if self.world is not None else None
                )
                self.state_enter_s = self._now_s()
        elif leg == "turn":
            omega = self.opening_turn_omega
            target = abs(math.radians(self.opening_turn_deg))
            timed_sec = 0.0
            if abs(omega) > 1e-6:
                timed_sec = target / abs(omega)
            done_by_heading = False
            if self._opening_turn_start_theta is not None and self.world is not None:
                direction = 1.0 if omega >= 0.0 else -1.0
                delta = self._wrap_pi(float(self.world.robot_theta) - self._opening_turn_start_theta)
                done_by_heading = direction * delta >= target
            timeout_sec = max(timed_sec + 0.5, timed_sec * 1.5)
            if not done_by_heading and t < timeout_sec:
                self._drive(0.0, 0.0, omega)
            else:
                self._drive(0.0, 0.0)
                self._opening_leg = "settle"
                self.state_enter_s = self._now_s()
        else:  # settle
            self._drive(0.0, 0.0)
            match_elapsed = self._now_s() - self._node_start_s
            release_ready = (
                self.opening_release_at_sec <= 0.0
                or match_elapsed >= self.opening_release_at_sec
            )
            if t >= self.opening_wait_after_turn_sec and release_ready:
                self._set_world_mapping_enabled(True, "opening wall settle complete")
                if (
                    self.zone_mission_enabled
                    and self.zone_anchor_nav_enabled
                    and self._active_zone_id() == 1
                ):
                    self._zone_anchor_reached_idx = self._zone_idx
                    self._zone_stabilized_idx = self._zone_idx
                    self._zone_stabilized_after_s = self.state_enter_s
                    self._reset_zone_scan_timer()
                    self._decide("ZONE 1 ANCHOR SKIPPED AFTER OPENING")
                self.get_logger().info(
                    f"opening done at t={match_elapsed:.1f}s "
                    f"(release_at={self.opening_release_at_sec:.1f}s) -> SCAN"
                )
                self._enter("SCAN")

    def _search_step(self) -> None:
        """Step-wise CW search: turn briefly, STOP to identify, repeat. The stationary 'look' phases
        give the wide cam clean, blur-free frames so tracks don't churn (spinning ruins detection)."""
        now = self._now_s()
        dt = now - self._search_t0
        if self._search_phase == "look":
            self._drive(0.0, 0.0)                               # hold still + perceive
            if dt >= self.search_look_sec:
                self._search_phase = "turn"
                self._search_t0 = now
        else:  # turn
            self._drive(0.0, 0.0, self.scan_search_omega)       # brief slow CW turn
            if dt >= self.search_turn_sec:
                self._search_phase = "look"
                self._search_t0 = now

    def _step_align(self) -> None:
        """Pulse+settle visual align: nudge the base briefly, let it FULLY STOP (inertia dissipates),
        then measure the object's body-cam base_link position (pose-independent) and grab if within
        tolerance (the gripper opening absorbs the residual). Otherwise nudge again. The base is
        stationary at grab time, so the object won't drift while the arm descends."""
        now = self._now_s()
        if self._time_in_state() > self.align_timeout_sec:
            self._drive(0.0, 0.0)
            if self._set2_slot_mode() and self._current_slot() is not None:
                self._retry_current_slot("align timeout")
                self._align_fail_count = 0
                self._enter("SELECT_TARGET")
                return
            self._align_fail_count += 1
            if self._align_fail_count >= self.max_align_fails and self.current_target is not None:
                self.get_logger().warn(
                    f"ALIGN failed {self._align_fail_count}x on id={self.current_target.id} -> blacklist, move on")
                self._blacklist(self.current_target.id)
                self.current_target = None
                self._align_fail_count = 0
            else:
                self.get_logger().warn(
                    f"ALIGN: timeout ({self._align_fail_count}/{self.max_align_fails}) -> retry")
            self._enter("SELECT_TARGET")
            return

        if self._align_phase == "pulse":
            if now - self._align_phase_start < self._pulse_sec:  # calibrated per-direction duration
                self._drive(self._pulse_vx, self._pulse_vy)     # one fixed unit step
            else:
                self._drive(0.0, 0.0)
                self._align_phase = "settle"
                self._align_phase_start = now
            return
        if self._align_phase == "settle":
            self._drive(0.0, 0.0)                                # let the heavy base fully stop
            if now - self._align_phase_start >= self.align_settle_pulse_sec:
                self._align_phase = "measure"
            return
        if self._align_phase == "body_lost_yaw_scan_turn":
            if now - self._align_phase_start < self._pulse_sec:
                self._drive(0.0, 0.0, self._yaw_scan_omega_cmd)
            else:
                self._drive(0.0, 0.0)
                self._align_phase = "body_lost_yaw_scan_settle"
                self._align_phase_start = now
            return
        if self._align_phase == "body_lost_yaw_scan_settle":
            self._drive(0.0, 0.0)
            if now - self._align_phase_start >= self.align_body_lost_yaw_scan_settle_sec:
                self._align_phase = "measure"
            return
        if self._align_phase == "body_lost_backoff":
            if now - self._align_phase_start < self.align_body_lost_backoff_sec:
                self._drive(-abs(self.align_body_lost_backoff_speed), 0.0)
            else:
                self._drive(0.0, 0.0)
                self._align_phase = "body_lost_backoff_settle"
                self._align_phase_start = now
            return
        if self._align_phase == "body_lost_backoff_settle":
            self._drive(0.0, 0.0)
            if now - self._align_phase_start >= self.align_body_lost_backoff_settle_sec:
                self.get_logger().info("ALIGN: body lost target -> backed off, settled")
                self._decide("ALIGN BODY LOST -> BACKOFF+SETTLE")
                if self._set2_slot_mode() and self._same_slot_reapproach_after_body_lost():
                    return
                if self._set2_slot_mode() and self._current_slot() is not None:
                    self._retry_current_slot("body target lost")
                self._enter("SELECT_TARGET")
            return

        # measure (base is settled/stationary) — align ONLY to the target label (body-cam truth)
        base = self._body_target_base(self._body_label())
        if base is None:
            if self._set2_slot_mode() and self._current_slot() is not None:
                set1_front = self._body_set1_distractor_in_front()
                if set1_front is not None:
                    self.get_logger().info(
                        f"ALIGN: Set2 slot sees Set1 '{set1_front[0]}' in front -> skip this slot")
                    self._retry_current_slot(f"body distractor {set1_front[0]}")
                    self._align_fail_count = 0
                    self._enter("SELECT_TARGET")
                    return
            # The target label isn't in front. If a DIFFERENT object (e.g. a body 'cube' on a set1 run)
            # is sitting at the grab point, this spot is wrong -> blacklist it and go elsewhere.
            other = self._body_nearest_any()
            if other is not None and other[0] not in self._body_align_labels():
                want = "/".join(sorted(self._body_align_labels()))
                self.get_logger().info(
                    f"ALIGN: body sees '{other[0]}' (not '{want}') at grab -> skip this spot")
                if self._set2_slot_mode() and self._current_slot() is not None:
                    self._retry_current_slot(f"body distractor {other[0]}")
                    self._align_fail_count = 0
                    self._enter("SELECT_TARGET")
                    return
                if self.current_target is not None:
                    self._blacklist(self.current_target.id)
                self.current_target = None
                self._align_fail_count = 0
                self._enter("SELECT_TARGET")
                return
            if self._yaw_scan_step > 0 and self._start_next_yaw_scan_turn():
                return
            if self._start_body_lost_yaw_scan():
                return
            if self.align_body_lost_backoff_enabled and self._align_target_still_in_world():
                self.get_logger().info(
                    "ALIGN: target in world/wide but not body -> back off to reacquire")
                self._drive(-abs(self.align_body_lost_backoff_speed), 0.0)
                self._align_phase = "body_lost_backoff"
                self._align_phase_start = now
                return
            # nothing there (or the target just blinked) -> re-settle & re-measure, don't bail.
            self._drive(0.0, 0.0)
            self._align_phase = "settle"
            self._align_phase_start = now
            return
        obj_x, obj_y = base
        self._yaw_scan_step = 0
        self._yaw_scan_omega_cmd = 0.0
        ex = obj_x - self.grab_x
        ey = obj_y - self.grab_y
        ex_tol = self.align_fwd_tol
        if abs(ex) < ex_tol and abs(ey) < self.align_tol:       # both axes settled -> grab
            self._drive(0.0, 0.0)
            self.get_logger().info(
                f"ALIGN ok (cam): fwd_err={ex * 100:+.1f}cm lat_err={ey * 100:+.1f}cm -> grab")
            self._align_fail_count = 0
            self.siglip_stamp_s = None
            self.shape_stamp_s = None
            self._enter("CLASSIFY")
            return
        # ---- Latch ONE calibrated unit step toward the WORST-out-of-tolerance axis ----
        # Each pulse is a fixed duty*time that moves a near-constant distance (fwd ~1.5-2 cm, strafe
        # ~2.5 cm). We attack the axis that is furthest out of its tolerance (normalized), fix the
        # other next cycle. copysign gives direction: +ex=object too far->forward(+vx); ex<0 (object
        # too near)->back(-vx). +ey=object to the left->strafe left(+vy). Re-measure after settle.
        lat_out = abs(ey) - self.align_tol
        fwd_out = abs(ex) - ex_tol
        step_lateral = lat_out > 0.0 and (fwd_out <= 0.0 or lat_out >= fwd_out)
        if step_lateral:
            vx, vy = 0.0, math.copysign(self.align_step_strafe_duty, ey)
            if self.align_adaptive_steps_enabled and abs(ey) >= self.align_mid_error_m:
                self._pulse_sec = self.align_step_strafe_mid_sec
            else:
                self._pulse_sec = self.align_step_strafe_sec
        else:
            vx, vy = math.copysign(self.align_step_fwd_duty, ex), 0.0
            if self.align_adaptive_steps_enabled and abs(ex) >= self.align_mid_error_m:
                self._pulse_sec = self.align_step_fwd_mid_sec
            else:
                self._pulse_sec = self.align_step_fwd_sec
            # Forward safety: never step past the body-cam blind limit. ex<0 still steps back freely.
            if obj_x <= self.grab_min_x and vx > 0.0:
                vx = 0.0
        self._pulse_vx, self._pulse_vy = vx, vy
        self._align_phase = "pulse"
        self._align_phase_start = now
        self._drive(vx, vy)

    def _step_classify_set2_slot(self) -> None:
        """Classify and pick only the latched Set2 slot; uncertain results are retried."""
        self._drive(0.0, 0.0)
        slot = self._current_slot()
        if slot is None:
            self.current_target = None
            self._enter("SELECT_TARGET")
            return

        tgt = self._front_target_for_current_slot()
        if tgt is not None:
            self.current_target = tgt
        self._attach_fresh_siglip_to_current_slot()

        if self.set2_require_fruit_label and slot.fruit_label:
            if slot.fruit_label != self.set2_label:
                self.slot_inventory.mark_non_target(slot.slot_id, self._now_s())
                if tgt is not None:
                    self._blacklist(tgt.id)   # still retained by world_model as a physical obstacle
                self._decide(f"SLOT NON-TARGET F{slot.slot_id} {slot.fruit_label}")
                self.get_logger().info(
                    f"Set2 slot F{slot.slot_id}: '{slot.fruit_label}' != '{self.set2_label}'"
                )
                self.current_target = None
                self._clear_current_slot()
                self._opportunistic_set2_active = False
                self._enter("SELECT_TARGET")
                return
            if tgt is not None and float(tgt.confidence) >= self.pick_track_conf:
                self.slot_inventory.mark_target_confirmed(slot.slot_id, self._now_s())
                self._commit_pick(
                    2,
                    slot.fruit_label,
                    f"slot F{slot.slot_id} track#{tgt.id} conf={tgt.confidence:.2f}",
                )
                return
        elif (not self.set2_require_fruit_label
              and tgt is not None
              and float(tgt.confidence) >= self.pick_track_conf):
            self.slot_inventory.mark_target_confirmed(slot.slot_id, self._now_s())
            self._commit_pick(
                2,
                "fruit_photo_cube",
                f"slot F{slot.slot_id} track#{tgt.id} conf={tgt.confidence:.2f} (fruit cube test)",
            )
            return

        if self._time_in_state() > self.classify_timeout_sec:
            self._retry_current_slot("classification uncertain")
            self._enter("SELECT_TARGET")

    def _step_classify(self) -> None:
        """Phase-aware pick gate from fresh siglip + shape classifications.

        Pick order is enforced here: in phase 1 only a confirmed Set1 shape is picked; a Set2
        object met here is skipped (NOT blacklisted) so it can be picked in phase 2, and vice
        versa. Recognition/mapping itself is phase-independent and runs continuously upstream.
        """
        if self._set2_slot_mode() and self._current_slot() is not None:
            self._step_classify_set2_slot()
            return
        # SPATIAL gate: confirm from the CURRENT TARGET's own world-model track (its fused wide+body
        # identity at THIS track's position), not a frame-global /classification/shape or /siglip
        # that may belong to a neighbouring distractor. Set1 = the track already carries the shape
        # (YOLO, wide+body); Set2 = the track carries the SigLIP fruit in fruit_label. Re-look it up
        # fresh so close-range body/siglip observations during APPROACH are included.
        # Target = object of this phase's set_type nearest the grab point (robust to id churn).
        # Confirm the SPECIFIC target in front: phase 1 needs the set1 shape by label (a neighbouring
        # cube is NOT it); phase 2 any fruit cube (its fruit type is checked via fruit_label below).
        if self._opportunistic_set2_active or self.phase == 2:
            tgt = self._front_target(2)
        else:
            tgt = self._front_target(1, want_label=self.set1_label)
        if tgt is None:
            # Momentarily not detected -> HOLD and wait for it to reappear (don't thrash back to
            # SELECT). Only give up after classify_timeout, and never blacklist a phantom (reselect).
            self._drive(0.0, 0.0)
            if self._time_in_state() > self.classify_timeout_sec:
                self.get_logger().info("CLASSIFY: target lost past timeout -> reselect")
                self.current_target = None
                self._opportunistic_set2_active = False
                self._enter("SELECT_TARGET")
            return
        self.current_target = tgt   # keep the FSM latched to the object actually in front

        set1_ok = bool(
            self.set1_label
            and tgt is not None
            and tgt.set_type == 1
            and tgt.class_label == self.set1_label
            and tgt.confidence >= self.pick_track_conf
        )
        # Set2 fruit: the track's SigLIP-derived fruit_label (only set when a face was actually read).
        set2_ok = bool(
            tgt is not None
            and tgt.set_type == 2
            and (not self.set2_require_fruit_label or tgt.fruit_label == self.set2_label)
            and tgt.confidence >= self.pick_track_conf
        )
        set2_pick_label = str(tgt.fruit_label) if tgt.fruit_label else "fruit_photo_cube"

        if self._opportunistic_set2_active:
            if set2_ok:
                self._commit_pick(2, set2_pick_label, f"opportunistic track conf={tgt.confidence:.2f}")
                return
            if self.set2_require_fruit_label and tgt.fruit_label and str(tgt.fruit_label) != self.set2_label:
                self.get_logger().info(
                    f"CLASSIFY: opportunistic fruit '{tgt.fruit_label}' != {self.set2_label} -> reject")
                self._blacklist(tgt.id)
                self.current_target = None
                self.set_type = 0
                self._opportunistic_set2_active = False
                self._enter("SELECT_TARGET")
                return
        elif self.phase == 1:
            if set1_ok:
                self._commit_pick(1, tgt.class_label, f"track conf={tgt.confidence:.2f}")
                return
            if set2_ok:
                self._skip_target(f"set2 '{tgt.fruit_label}' during Set1 phase")
                return
        else:  # phase 2
            if set2_ok:
                detail = "siglip fruit" if self.set2_require_fruit_label else "fruit cube test"
                self._commit_pick(2, set2_pick_label, f"track conf={tgt.confidence:.2f} ({detail})")
                return
            if set1_ok:
                self._skip_target(f"stray set1 '{tgt.class_label}' during Set2 phase")
                return
            # SigLIP already typed it a DIFFERENT fruit -> blacklist NOW (don't wait the classify timeout)
            if self.set2_require_fruit_label and tgt.fruit_label and str(tgt.fruit_label) != self.set2_label:
                self.get_logger().info(f"CLASSIFY: fruit '{tgt.fruit_label}' != {self.set2_label} -> skip now")
                self._blacklist(tgt.id)
                self.current_target = None
                self.set_type = 0
                self._opportunistic_set2_active = False
                self._enter("SELECT_TARGET")
                return

        # No confident phase-appropriate decision: pass (blacklist) once we time out.
        if self._time_in_state() > self.classify_timeout_sec:
            obj_id = self.current_target.id if self.current_target else 0
            self.get_logger().warn(f"GATE PASS id={obj_id} (classify timeout) -> blacklist")
            self._decide(f"PASS #{obj_id} (classify timeout)")
            self._blacklist(obj_id)
            self.current_target = None
            self.set_type = 0
            self._opportunistic_set2_active = False
            self._enter("SELECT_TARGET")

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        # 1) drive condition-based transitions
        self._step()
        if self.state != "OPENING":
            self._set_world_mapping_enabled(True)
        self.pub_mapping_enabled.publish(Bool(data=bool(self._world_mapping_enabled)))
        self.pub_wall_fast.publish(Bool(data=self._wall_fast_correction_requested()))

        # 2) publish mission state every tick
        msg = MissionState()
        msg.state = self.state
        msg.tray_shape_count = self.tray_shape
        msg.tray_fruit_count = self.tray_fruit
        msg.current_target_id = self.current_target.id if self.current_target else 0
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)

        # 3) publish current pick phase (target selector filters candidates by it)
        self.pub_phase.publish(Int8(data=int(self.phase)))
        self.pub_zone.publish(Int8(data=int(self._active_zone_id() if self.zone_mission_enabled else 0)))
        self._publish_slot_debug()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionFsmNode()
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
