"""Mission FSM for the AI Robot Challenge.

States:
    SCAN              - top cam world model build/update
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
(/base/goal_pose) and arm (/arm/pick_trigger). A manual /state_advance trigger
still force-advances one transition for debug/dry-run exercising.

Pick-gate (rulebook §6/§7, mispick on Set2 = -40, so be conservative):
  - Set2 fruit: siglip == today's fruit + image_face_visible + conf >= thresh -> PICK.
  - Set1 cube : shape == today's cube  + NOT image_face_visible + conf >= thresh -> PICK.
  - otherwise (or classify timeout) -> PASS (blacklist, reselect).
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_interfaces.msg import BaseCommand, Classification, DetectionArray, MissionState, Object, WorldModel
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty, Int8, String, UInt64

from robot_planning.lane_planner import LanePlanner

# Body detection label -> set_type (mirror of world_model), for the ALIGN visual-servo filter.
_LABEL_ST = {"cube": 1, "octahedron": 1, "dodecahedron": 1, "icosahedron": 1, "fruit_photo_cube": 2}


STATES = [
    "OPENING", "SCAN", "SELECT_TARGET", "APPROACH", "ALIGN", "CLASSIFY", "PICK",
    "STORE_IN_TRAY", "DRIVE_TO_STORAGE", "ALIGN_OVER_BIN", "DUMP_ALL", "END",
]


class MissionFsmNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_fsm_node")

        # Today's announced targets (rulebook §6.1, §7.3).
        self.declare_parameter("set1_label", "")      # e.g. "icosahedron"
        self.declare_parameter("set2_label", "")      # e.g. "apple"
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
        # Front HC-SR04 sets the FORWARD grab distance directly (no body-cam projection error): drive
        # so the sonar reads grab_range_m. Body cam still does the LATERAL (y) align. grab_range_m must
        # be CALIBRATED (place a cube at the grab point, read /ultrasonic/range). 0 or stale -> body-cam x.
        self.declare_parameter("use_sonar_align", True)
        self.declare_parameter("grab_range_m", 0.12)    # sonar reading when object is at the grab point (TUNE)
        self.declare_parameter("align_sonar_tol_m", 0.02)
        self.declare_parameter("sonar_timeout_sec", 0.5)
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
        # Medium pulse: when the selected ALIGN axis is still far from the grab point, keep the same
        # duty but pulse longer. Once close, fall back to the short calibrated pulse above.
        self.declare_parameter("align_adaptive_steps_enabled", False)
        self.declare_parameter("align_mid_error_m", 0.08)
        self.declare_parameter("align_step_fwd_mid_sec", 0.225)
        self.declare_parameter("align_step_strafe_mid_sec", 0.24)
        # If the target remains in the wide/world map but drops out of the body cam at ALIGN,
        # back up once so the body cam can reacquire it instead of waiting stationary for timeout.
        self.declare_parameter("align_body_lost_backoff_enabled", True)
        self.declare_parameter("align_body_lost_backoff_speed", 0.189)
        self.declare_parameter("align_body_lost_backoff_sec", 0.54)
        self.declare_parameter("align_body_lost_backoff_settle_sec", 1.0)
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
        # Drive forward, strafe right, rotate in place by opening_turn_deg, wait, then hand over to SCAN.
        # Timed through /base_command so the base_controller's start-boost + stop-brake apply.
        self.declare_parameter("opening_enabled", True)
        # Hold STATIONARY at startup until perception is warm: YOLO (wide.pt@1280) takes ~10-15 s to
        # load, and moving before detections flow makes the localizer's object-flow/VO drift and lose
        # heading. Wait this long AND until the first /world_model arrives before the opening move.
        self.declare_parameter("startup_warmup_sec", 15.0)
        self.declare_parameter("opening_speed", 0.35)        # >= wheel_min so it's the actual speed
        self.declare_parameter("opening_forward_sec", 1.0)
        self.declare_parameter("opening_strafe_speed", 0.35)
        self.declare_parameter("opening_strafe_right_sec", 0.0)
        self.declare_parameter("opening_turn_deg", 45.0)
        self.declare_parameter("opening_turn_omega", -0.17)   # negative = CW, positive = CCW
        self.declare_parameter("opening_wait_after_turn_sec", 1.0)
        # SCAN search: the start pose likely sees nothing, so after the opening the robot turns slowly
        # CLOCKWISE in place to sweep for objects. Negative omega = CW (REP-103 +z is up). The actual
        # turn speed is set by base_controller wheel_min_rot (this just needs to be non-zero CW).
        self.declare_parameter("scan_search_omega", -0.17)
        # When nothing is visible, DRIVE to the map centre for a better view instead of spinning in
        # place (the wide fisheye already sees all around; a central vantage just helps).
        self.declare_parameter("map_center_x", 0.0)
        self.declare_parameter("map_center_y", 0.0)
        # If the phase target isn't in view, PATROL these field waypoints (looking with the wide cam)
        # until it appears — never give up. Flattened [x0,y0,x1,y1,...]. Kept inside the wall margin.
        self.declare_parameter("patrol_waypoints",
                               [0.0, 0.0, 1.2, 1.2, 1.2, -1.2, -1.2, -1.2, -1.2, 1.2])
        self.declare_parameter("patrol_reach_tol", 0.3)   # advance to next waypoint within this
        # Zone mission: split the field into four 2x2m zones. Zone1 is the start area
        # (bottom-right on the live map), then zone2 above, zone3 upper-left, zone4 storage side.
        self.declare_parameter("zone_mission_enabled", False)
        self.declare_parameter("zone_order", [1, 2, 3, 4])
        self.declare_parameter(
            "zone_bounds_m",
            [-2.0, 0.0, 0.0, 2.0, -2.0, 0.0, -2.0, 0.0,
             0.0, 2.0, -2.0, 0.0, 0.0, 2.0, 0.0, 2.0],
        )
        self.declare_parameter("zone_no_target_advance_sec", 6.0)
        self.declare_parameter("zone_center_reach_tol_m", 0.25)
        # Step-wise search: turn a little, STOP to let the cameras identify (clean, blur-free frames),
        # turn again. Continuous spinning motion-blurs the wide cam and churns tracks.
        self.declare_parameter("search_turn_sec", 0.5)    # rotate this long per step (~small angle)
        self.declare_parameter("search_look_sec", 1.6)    # then hold still this long to identify
        # APPROACH (FSM-driven, not go_to_goal): first rotate slowly to FACE the target (so the body
        # cam sees it), then drive forward. Proportional -> converges and stops (no endless spin).
        self.declare_parameter("approach_face_tol", 0.25)  # rad; within this heading error -> drive
        self.declare_parameter("approach_speed", 0.32)     # forward speed once facing
        self.declare_parameter("approach_kp_ang", 1.0)     # omega = kp * heading-to-target (clamped)
        self.declare_parameter("approach_omega_max", 0.23)
        # APPROACH is also STEP-WISE (perceive from stop, then one short move) so motion never blurs
        # the wide cam / churns the target track (that churn was the spin). And a grace window keeps
        # the target latched through a momentary dropout instead of thrashing back to SELECT.
        self.declare_parameter("approach_look_sec", 1.0)   # hold still + perceive the target
        self.declare_parameter("approach_move_sec", 0.5)   # then one short rotate-or-forward step
        self.declare_parameter("approach_lost_grace_sec", 2.5)  # keep target this long if it drops out
        self.declare_parameter("approach_standoff_tol", 0.09)   # reached the stand-off within this -> ALIGN
        # phase 2: at the stand-off, hold up to this long for SigLIP to type the fruit BEFORE aligning,
        # so we don't waste a full align on an apple/banana. Orange -> align now; typed non-orange ->
        # dropped by _nearest_phase_object; still untyped after this -> align closer for a better view.
        self.declare_parameter("classify_standoff_sec", 2.0)

        self.set1_label = str(self.get_parameter("set1_label").value)
        self.set2_label = str(self.get_parameter("set2_label").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.pick_track_conf = float(self.get_parameter("pick_track_conf").value)
        self.approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        self.align_settle_sec = float(self.get_parameter("align_settle_sec").value)
        self.classify_timeout_sec = float(self.get_parameter("classify_timeout_sec").value)
        self.use_sonar_align = bool(self.get_parameter("use_sonar_align").value)
        self.grab_range_m = float(self.get_parameter("grab_range_m").value)
        self.align_sonar_tol = float(self.get_parameter("align_sonar_tol_m").value)
        self.sonar_timeout = float(self.get_parameter("sonar_timeout_sec").value)
        self._front_range = float("inf")
        self._front_range_t = None
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
        self._align_phase = "measure"      # measure -> pulse/body_lost_backoff -> settle/SELECT -> measure ...
        self._align_phase_start = 0.0
        self._pulse_vx = 0.0
        self._pulse_vy = 0.0
        self._pulse_sec = self.align_step_fwd_sec   # duration of the current unit step (per direction)
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
        self._body_dets: list = []   # latest /camera_body/detections as (u, v_center, set_type)
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
        self.scan_search_omega = float(self.get_parameter("scan_search_omega").value)
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
        self.zone_no_target_advance_sec = float(self.get_parameter("zone_no_target_advance_sec").value)
        self.zone_center_reach_tol_m = float(self.get_parameter("zone_center_reach_tol_m").value)
        self._zone_idx = 0
        self._zone_no_target_since: float | None = None

        # ---- LANE-GRAPH waypoint planner (50cm object grid, 40cm robot) ----
        # Travel goals (SCAN sweep, APPROACH stand-off, DRIVE_TO_STORAGE) route through collision-free
        # lane midlines instead of a straight shot; go_to_goal still drives each via car-like + reactive.
        self.declare_parameter("planner_enabled", True)
        self.declare_parameter("planner_mode", "lane")          # lane | taxi_hybrid
        self.declare_parameter("taxi_final_direct_m", 0.35)     # last short leg may leave taxi lanes
        self.declare_parameter("grid_spacing_m", 0.50)
        self.declare_parameter("grid_origin_mode", "infer")     # infer | fixed
        self.declare_parameter("grid_origin_xy", [0.0, 0.0])
        self.declare_parameter("phase_tol_m", 0.08)
        self.declare_parameter("field_bounds_m", [-2.0, 2.0, -2.0, 2.0])   # mirror go_to_goal
        self.declare_parameter("robot_margin_m", 0.22)
        self.declare_parameter("lane_block_radius_m", 0.24)     # obstacle-to-lane dist that blocks an edge
        self.declare_parameter("comfort_clear_m", 0.35)
        self.declare_parameter("clearance_weight", 2.0)         # prefer roomy lanes over tight gates
        self.declare_parameter("start_connect_k", 4)
        self.declare_parameter("wp_reach_tol_m", 0.12)          # advance to next via within this
        self.declare_parameter("replan_period_sec", 1.5)
        self.declare_parameter("replan_goal_move_m", 0.15)      # replan if the destination drifts
        self.declare_parameter("replan_throttle_sec", 0.5)
        self.declare_parameter("obstacle_min_conf", 0.4)        # phantom filter for obstacles
        self.declare_parameter("obstacle_min_nobs", 2)
        self.declare_parameter("exclude_target_radius_m", 0.30)  # drop objects near dest (final leg reachable)
        self.planner_enabled = bool(self.get_parameter("planner_enabled").value)
        fb = [float(v) for v in self.get_parameter("field_bounds_m").value]
        self.wp_reach_tol_m = float(self.get_parameter("wp_reach_tol_m").value)
        self.replan_period_sec = float(self.get_parameter("replan_period_sec").value)
        self.replan_goal_move_m = float(self.get_parameter("replan_goal_move_m").value)
        self.replan_throttle_sec = float(self.get_parameter("replan_throttle_sec").value)
        self.obstacle_min_conf = float(self.get_parameter("obstacle_min_conf").value)
        self.obstacle_min_nobs = int(self.get_parameter("obstacle_min_nobs").value)
        self.exclude_target_radius_m = float(self.get_parameter("exclude_target_radius_m").value)
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
            mode=str(self.get_parameter("planner_mode").value),
            taxi_final_direct_m=float(self.get_parameter("taxi_final_direct_m").value),
        )
        self._plan: list[tuple[float, float]] | None = None   # active via list (last = dest)
        self._plan_idx = 0
        self._plan_dest: tuple[float, float] | None = None
        self._plan_stamp = 0.0
        self._last_replan_t = 0.0
        self._coverage: list[tuple[float, float]] | None = None   # SCAN lane-sweep waypoints
        self._coverage_idx = 0
        self.search_turn_sec = float(self.get_parameter("search_turn_sec").value)
        self.search_look_sec = float(self.get_parameter("search_look_sec").value)
        self.approach_face_tol = float(self.get_parameter("approach_face_tol").value)
        self.approach_speed = float(self.get_parameter("approach_speed").value)
        self.approach_kp_ang = float(self.get_parameter("approach_kp_ang").value)
        self.approach_omega_max = float(self.get_parameter("approach_omega_max").value)
        self.approach_look_sec = float(self.get_parameter("approach_look_sec").value)
        self.approach_move_sec = float(self.get_parameter("approach_move_sec").value)
        self.approach_lost_grace_sec = float(self.get_parameter("approach_lost_grace_sec").value)
        self.approach_standoff_tol = float(self.get_parameter("approach_standoff_tol").value)
        self.classify_standoff_sec = float(self.get_parameter("classify_standoff_sec").value)
        self._standoff_arrived_s = None
        self._search_phase = "look"     # step-wise search: look <-> turn
        self._search_t0 = self._now_s()
        self._approach_phase = "look"   # step-wise approach: look <-> move
        self._approach_t0 = self._now_s()
        self._appr_tgt_xy = None        # last-known target field xy (survives momentary dropout)
        self._appr_last_seen_s = 0.0
        rate = float(self.get_parameter("publish_rate_hz").value)

        # --- runtime state ---
        # Start with the hardcoded opening move, then fall into SCAN. Disable via param.
        self.state = "OPENING" if self.opening_enabled else "SCAN"
        self._opening_leg = "wait"      # wait -> forward -> strafe_right -> turn -> settle -> SCAN
        self._opening_turn_start_theta: float | None = None
        self._got_world = False         # set on first /world_model (perception up)
        self._node_start_s = self._now_s()
        self.phase = 1 if self.shape_target_total > 0 else 2
        # 1 = pursue Set1, 2 = pursue Set2. With zone_mission enabled this is zone-local:
        # sweep Set1 in the current zone, then Set2 in the same zone, then advance zones.
        self.tray_shape = 0
        self.tray_fruit = 0
        self.current_target: Object | None = None    # latched selected target (id, set_type, ...)
        self.set_type = 0                             # set_type confirmed by the pick gate

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
        self.create_subscription(Range, "/ultrasonic/range", self.on_range, 10)   # front grab distance
        self.create_subscription(Empty, "/state_advance", self.on_advance, 10)

        self.pub_state = self.create_publisher(MissionState, "/mission_state", 10)
        self.pub_blacklist = self.create_publisher(UInt64, "/world_model/blacklist_add", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/base/goal_pose", 10)
        self.pub_pick = self.create_publisher(Bool, "/arm/pick_trigger", 10)
        self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)   # ALIGN visual servo
        # Current pick phase (1=Set1, 2=Set2) for the target selector's phase filter.
        self.pub_phase = self.create_publisher(Int8, "/planning/phase", 10)
        self.pub_zone = self.create_publisher(Int8, "/planning/zone", 10)
        # Human-readable decision feed (PICK / PASS / SKIP / PHASE / END) for the visualiser.
        self.pub_decision = self.create_publisher(String, "/planning/decision", 10)

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"FSM started at {self.state}  set1='{self.set1_label}' set2='{self.set2_label}' "
            f"conf>={self.conf_threshold} approach<{self.approach_dist_m}m rate={rate}Hz "
            f"dry_pick={self.dry_pick} end_after_quota={self.end_after_quota} "
            f"zone_mission={self.zone_mission_enabled} zone={self._active_zone_id()} "
            f"select_timeout={self.select_timeout_sec}s"
        )

    # ------------------------------------------------------------------ utils
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _time_in_state(self) -> float:
        return self._now_s() - self.state_enter_s

    def _enter(self, new_state: str) -> None:
        prev = self.state
        self.state = new_state
        self.state_enter_s = self._now_s()
        self._plan = None                       # a state change invalidates the active travel plan
        if new_state == "ALIGN":
            self._align_phase = "measure"       # start each ALIGN by measuring the settled position
        if new_state == "APPROACH":
            self._approach_phase = "look"       # start each approach by perceiving from a stop
            self._approach_t0 = self._now_s()
            self._appr_tgt_xy = None
            self._appr_last_seen_s = self._now_s()
            self._standoff_arrived_s = None
        self.get_logger().info(
            f"transition {prev} -> {new_state}  "
            f"tray=(shape:{self.tray_shape}, fruit:{self.tray_fruit})  "
            f"target_id={self.current_target.id if self.current_target else 0}"
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
            zx, zy = self._zone_center()
            if (d := self._distance_to(zx, zy)) is not None and d <= self.zone_center_reach_tol_m:
                self._search_step()
            else:
                th = self.world.robot_theta if self.world is not None else 0.0
                self._drive_toward(zx, zy, th, exclude_id=0)
            return
        if not self.planner_enabled:
            wp = self._patrol_waypoints[self._patrol_idx]
            th = self.world.robot_theta if self.world is not None else 0.0
            self._publish_goal(wp[0], wp[1], th)
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
        return self.set1_label if self.phase == 1 else "fruit_photo_cube"

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
            if want_set == 1 and str(o.class_label) != want_label:
                continue
            # phase 2: a fruit cube already SigLIP-typed as a DIFFERENT fruit is not our target —
            # skip it so we don't waste an approach+align on an apple/banana. Untyped cubes ('') are
            # still approached (they get typed once the body cam is close enough).
            if want_set == 2:
                fl = str(o.fruit_label)
                if fl and fl != self.set2_label:
                    continue
            if float(o.confidence) < self.pick_track_conf:
                continue
            d = math.hypot(o.x - rx, o.y - ry)
            if d < bestd:
                bestd = d
                best = o
        return best

    def on_body_dets(self, msg: DetectionArray) -> None:
        # Keep the raw body-cam LABEL: ALIGN must only align to the target SHAPE (e.g. icosahedron),
        # not any set_type-1 shape. If the body cam says 'cube' it is NOT the set1 target -> ignore it.
        self._body_dets = [(float(d.x_center), float(d.y_center), str(d.label))
                           for d in msg.detections]

    def on_range(self, msg: Range) -> None:
        self._front_range = float(msg.range)
        self._front_range_t = self._now_s()

    def _sonar_fresh(self) -> bool:
        return (self.use_sonar_align and self._front_range_t is not None
                and (self._now_s() - self._front_range_t) <= self.sonar_timeout
                and math.isfinite(self._front_range))

    def _body_target_base(self, want_label: str) -> tuple[float, float] | None:
        """POSE-INDEPENDENT servo target: project each body detection whose LABEL == want_label to
        base_link (body homography + height correction), return the one nearest the grab point. Only
        the target shape/kind is aligned to — a body 'cube' in front of a set1(icosahedron) run is NOT
        a target, so ALIGN gets None and moves on instead of grabbing the wrong object."""
        if self._body_H is None or not self._body_dets:
            return None
        best = None
        bestd = 0.5
        k = self._boh / self._bH
        for u, v, label in self._body_dets:
            if label != want_label:
                continue
            # body cam may be downscaled; upscale pixel to the homography's 1640x1232 calibration domain
            us, vs = u * self._body_px_sx, v * self._body_px_sy
            p = cv2.perspectiveTransform(np.array([[[us, vs]]], np.float64), self._body_H)[0][0]
            bx, by = float(p[0]) - k * (float(p[0]) - self._bnx), float(p[1]) - k * float(p[1])
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
        k = self._boh / self._bH
        for u, v, label in self._body_dets:
            us, vs = u * self._body_px_sx, v * self._body_px_sy
            p = cv2.perspectiveTransform(np.array([[[us, vs]]], np.float64), self._body_H)[0][0]
            bx, by = float(p[0]) - k * (float(p[0]) - self._bnx), float(p[1]) - k * float(p[1])
            d = math.hypot(bx - self.grab_x, by - self.grab_y)
            if d < bestd:
                bestd = d
                best = (label, bx, by)
        return best

    def _align_target_still_in_world(self) -> bool:
        """True when the latched ALIGN target is still visible in the wide/world map."""
        if self.current_target is None or self.world is None:
            return False
        fresh = self._lookup_object(int(self.current_target.id))
        if fresh is not None and not fresh.blacklisted:
            self.current_target = fresh
            return True
        ref = (float(self.current_target.x), float(self.current_target.y))
        return self._nearest_phase_object(ref_xy=ref) is not None

    def _drive(self, vx: float, vy: float, omega: float = 0.0) -> None:
        c = BaseCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.vx, c.vy, c.omega = float(vx), float(vy), float(omega)
        self.pub_cmd.publish(c)

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _robot_xy(self) -> tuple[float, float] | None:
        if self.world is None:
            return None
        return (self.world.robot_x, self.world.robot_y)

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
        xmin, xmax, ymin, ymax = self.zone_bounds.get(zid, (-2.0, 2.0, -2.0, 2.0))
        return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5)

    def _object_in_active_zone(self, obj: Object) -> bool:
        if not self.zone_mission_enabled:
            return True
        xmin, xmax, ymin, ymax = self.zone_bounds.get(self._active_zone_id(), (-2.0, 2.0, -2.0, 2.0))
        return xmin <= float(obj.x) <= xmax and ymin <= float(obj.y) <= ymax

    def _reset_zone_scan_timer(self) -> None:
        self._zone_no_target_since = None

    def _shape_needed(self) -> bool:
        return self.shape_target_total > 0 and self.tray_shape < self.shape_target_total

    def _fruit_needed(self) -> bool:
        return self.fruit_target_total > 0 and self.tray_fruit < self.fruit_target_total

    def _both_quotas_met(self) -> bool:
        return (
            self.tray_shape >= self.shape_target_total
            and self.tray_fruit >= self.fruit_target_total
        )

    def _first_needed_phase(self) -> int:
        if self._shape_needed():
            return 1
        if self._fruit_needed():
            return 2
        return 0

    def _reset_zone_phase_context(self) -> None:
        self._plan = None
        self._coverage = None
        self.current_target = None
        self._reset_zone_scan_timer()

    def _advance_zone_or_phase(self) -> None:
        if not self.zone_mission_enabled:
            self._advance_phase_or_end()
            return
        prev = self._active_zone_id()
        if self.phase == 1 and self._fruit_needed():
            self.phase = 2
            self._reset_zone_phase_context()
            self.get_logger().info(f"zone {prev} Set1 done -> Set2 in same zone")
            self._decide(f"ZONE {prev} SET1->SET2")
            self._enter("SCAN")
            return
        if self._zone_idx + 1 < len(self.zone_order):
            self._zone_idx += 1
            next_phase = self._first_needed_phase()
            if next_phase == 0:
                self._zone_idx -= 1
                self._reset_zone_phase_context()
                self.get_logger().info("all quotas met while advancing zones -> storage/end")
                self._decide("END (quotas met)")
                if self.tray_shape > 0 and not self.end_after_quota:
                    self._enter("DRIVE_TO_STORAGE")
                else:
                    self._enter("END")
                return
            self.phase = next_phase
            self._reset_zone_phase_context()
            self.get_logger().info(
                f"zone {prev} done -> zone {self._active_zone_id()} Set{self.phase}")
            self._decide(f"ZONE {prev}->{self._active_zone_id()} SET{self.phase}")
            self._enter("SCAN")
            return
        self._zone_idx = 0
        self._reset_zone_phase_context()
        self.get_logger().info(f"zone {prev} done -> zone-local sweep complete")
        self._decide(f"ZONE {prev} complete")
        if self.tray_shape > 0 and not self.end_after_quota:
            self._enter("DRIVE_TO_STORAGE")
        else:
            self._enter("END")

    def _publish_goal(self, x: float, y: float, theta: float = 0.0) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        # quaternion from yaw (REP-103): qz=sin(theta/2), qw=cos(theta/2), qx=qy=0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self.pub_goal.publish(msg)

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

    def _drive_toward(self, dest_x: float, dest_y: float, yaw: float = 0.0, exclude_id: int = 0) -> None:
        """Route to (dest_x,dest_y) via collision-free lane waypoints, publishing the ACTIVE via to
        go_to_goal each tick and advancing on the FSM's own arrival test. Falls back to a direct goal
        when the planner is disabled or finds no lane route. The caller keeps its own dest-arrival
        test to fire the state transition."""
        if not self.planner_enabled:
            self._publish_goal(dest_x, dest_y, yaw)
            return
        rxy = self._robot_xy()
        if rxy is None:
            self._publish_goal(dest_x, dest_y, yaw)
            return
        now = self._now_s()
        dest = (float(dest_x), float(dest_y))
        drifted = (self._plan_dest is None
                   or math.hypot(dest[0] - self._plan_dest[0], dest[1] - self._plan_dest[1]) > self.replan_goal_move_m)
        stale = now - self._plan_stamp > self.replan_period_sec
        # No plan at all -> MUST plan now (ignore throttle); drift/stale replans are throttled.
        if self._plan is None or ((drifted or stale) and now - self._last_replan_t >= self.replan_throttle_sec):
            self._last_replan_t = now
            vias = self._planner.plan(rxy, dest, self._obstacles_snapshot(exclude_id, dest))
            self._plan = vias if vias else [dest]     # None (no lane route) -> direct goal fallback
            self._plan_idx = 0
            self._plan_dest = dest
            self._plan_stamp = now
        if not self._plan:                            # safety: never index a None/empty plan
            self._publish_goal(dest_x, dest_y, yaw)
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
        last = self._plan_idx == len(self._plan) - 1
        # intermediate vias pass current heading (go_to_goal faces the via by position anyway); only the
        # FINAL via carries the intended yaw (APPROACH: face the target; storage: as given).
        via_yaw = yaw if last else (self.world.robot_theta if self.world is not None else yaw)
        self._publish_goal(wx, wy, via_yaw)

    def _publish_pick(self, trigger: bool) -> None:
        self.pub_pick.publish(Bool(data=trigger))

    def _blacklist(self, obj_id: int) -> None:
        if obj_id != 0:
            self.pub_blacklist.publish(UInt64(data=int(obj_id)))

    def _decide(self, text: str) -> None:
        """Publish a human-readable decision for the visualiser's decision feed."""
        self.pub_decision.publish(String(data=text))

    def _count_non_blacklisted(self) -> int:
        if self.world is None:
            return 0
        return sum(1 for o in self.world.objects if not o.blacklisted)

    # --------------------------------------------------------------- callbacks
    def on_world(self, msg: WorldModel) -> None:
        self.world = msg
        self._got_world = True

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
        if self.current_target is not None:
            self._blacklist(self.current_target.id)   # picked -> never reselect
        self.current_target = None
        self.set_type = 0

        if self.zone_mission_enabled:
            # In zone mode, Set2 is handled before leaving the current zone. Quotas can still cut a
            # sweep short: once all Set1 are picked, continue with Set2 in this same zone; once all
            # Set2 are picked but Set1 remains globally, move on to the next zone's Set1 sweep.
            if self.phase == 1 and not self._shape_needed() and self._fruit_needed():
                self._advance_zone_or_phase()
                return
            if self.phase == 2 and not self._fruit_needed() and self._shape_needed():
                self._advance_zone_or_phase()
                return

        # Non-zone mode keeps the original global Set1 sweep -> Set2 sweep behavior.
        if (not self.zone_mission_enabled
                and self.phase == 1
                and self.tray_shape >= self.shape_target_total
                and self.fruit_target_total > 0):
            self.phase = 2
            self._zone_idx = 0
            self._reset_zone_scan_timer()
            self.get_logger().info("Set1 quota met -> phase 2 (Set2)")
            self._decide("PHASE 1->2 (Set1 quota met)")

        both_met = self._both_quotas_met()
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
            if self._nearest_phase_object() is not None:
                self._reset_zone_scan_timer()
                self._enter("SELECT_TARGET")                   # phase target in view -> pursue it
            else:
                if self.zone_mission_enabled:
                    zx, zy = self._zone_center()
                    d_zone = self._distance_to(zx, zy)
                    if d_zone is None or d_zone > self.zone_center_reach_tol_m:
                        self._reset_zone_scan_timer()
                        self._patrol_search()
                        return
                    now = self._now_s()
                    if self._zone_no_target_since is None:
                        self._zone_no_target_since = now
                    elif now - self._zone_no_target_since >= self.zone_no_target_advance_sec:
                        self._advance_zone_or_phase()
                        return
                self._patrol_search()                          # not in view -> patrol the field to find it

        elif self.state == "SELECT_TARGET":
            tgt = self._nearest_phase_object()         # nearest visible object of this kind
            if tgt is not None:
                self._reset_zone_scan_timer()
                self.current_target = tgt
                self._enter("APPROACH")
            else:
                self._enter("SCAN")                    # not in view -> go PATROL to find it (never give up)

        elif self.state == "APPROACH":
            # Stay COMMITTED to the latched object: pick the phase object nearest the LATCHED xy (not
            # the globally nearest), so we don't shuttle to whichever cube is momentarily closest.
            tgt = self._nearest_phase_object(ref_xy=self._appr_tgt_xy)
            if tgt is not None:
                self.current_target = tgt
                self._appr_tgt_xy = (tgt.x, tgt.y)
                self._appr_last_seen_s = self._now_s()
            elif (self._appr_tgt_xy is None
                  or self._now_s() - self._appr_last_seen_s > self.approach_lost_grace_sec):
                self._enter("SCAN")                    # nothing of this kind visible -> look / go centre
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
                # phase 2: at the stand-off, let SigLIP type the fruit BEFORE aligning — don't waste a
                # full align on a non-orange. Orange -> align now; still untyped -> wait briefly then
                # align closer; a typed non-orange is already dropped by _nearest_phase_object.
                if self.phase == 2 and tgt is not None and str(tgt.fruit_label) != self.set2_label:
                    self._drive(0.0, 0.0)
                    if self._standoff_arrived_s is None:
                        self._standoff_arrived_s = self._now_s()
                    if self._now_s() - self._standoff_arrived_s < self.classify_standoff_sec:
                        return                      # hold at stand-off, wait for the fruit type
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
        self.set_type = 0
        self._enter("SELECT_TARGET")

    def _step_opening(self) -> None:
        """Hardcoded match opening: forward -> right strafe -> 45deg turn -> wait -> SCAN.

        BODY-frame commands are timed; state_enter_s is reset at each leg so _time_in_state()
        measures that leg only. The turn duration is derived from opening_turn_deg/omega.
        """
        t = self._time_in_state()
        leg = self._opening_leg
        if leg == "wait":
            # Hold STATIONARY for the configured warmup, then run the opening even if perception
            # is still late. This guarantees the match-start motion happens after the 15 s wait.
            self._drive(0.0, 0.0)
            warm = (self._now_s() - self._node_start_s) >= self.startup_warmup_sec
            if warm:
                self.get_logger().info("warmup done -> opening move (forward + right strafe + 45deg turn)")
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
            if t >= self.opening_wait_after_turn_sec:
                self.get_logger().info("opening done (forward + right strafe + 45deg turn + wait) -> SCAN")
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

    def _step_approach(self, tgt_xy) -> None:
        """STEP-WISE approach to a field xy: LOOK (hold still, perceive) then one short MOVE step
        (rotate toward it if not facing, else drive forward), repeat. Perceiving only while stopped
        keeps the wide cam sharp so the target track stays put — that motion churn was the spin."""
        now = self._now_s()
        dt = now - self._approach_t0
        to = self._to_base(tgt_xy[0], tgt_xy[1])   # target in base_link (x fwd, y left)
        if to is None:
            self._drive(0.0, 0.0)
            return
        bx, by = to
        dist = math.hypot(bx, by)
        if dist < self.approach_dist_m:            # close enough -> hand off to ALIGN
            self._drive(0.0, 0.0)
            self._enter("ALIGN")
            return
        if self._approach_phase == "look":
            self._drive(0.0, 0.0)                  # hold still + let perception settle the target
            if dt >= self.approach_look_sec:
                self._approach_phase = "move"
                self._approach_t0 = now
            return
        # move: ONE short step toward the target, then back to look
        heading_err = math.atan2(by, bx)           # + = target to the robot's left
        om = max(-self.approach_omega_max, min(self.approach_omega_max, self.approach_kp_ang * heading_err))
        if abs(heading_err) > self.approach_face_tol:
            self._drive(0.0, 0.0, om)              # not facing: rotate a step toward it (slow)
        else:
            self._drive(self.approach_speed, 0.0, 0.0)   # facing: drive forward a step
        if dt >= self.approach_move_sec:
            self._drive(0.0, 0.0)
            self._approach_phase = "look"
            self._approach_t0 = now

    def _step_align(self) -> None:
        """Pulse+settle visual align: nudge the base briefly, let it FULLY STOP (inertia dissipates),
        then measure the object's body-cam base_link position (pose-independent) and grab if within
        tolerance (the gripper opening absorbs the residual). Otherwise nudge again. The base is
        stationary at grab time, so the object won't drift while the arm descends."""
        now = self._now_s()
        if self._time_in_state() > self.align_timeout_sec:
            self._drive(0.0, 0.0)
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
                self.get_logger().info("ALIGN: body lost target -> backed off, settled, reselect")
                self._decide("ALIGN BODY LOST -> BACKOFF+SETTLE")
                self._enter("SELECT_TARGET")
            return

        # measure (base is settled/stationary) — align ONLY to the target label (body-cam truth)
        base = self._body_target_base(self._body_label())
        if base is None:
            # The target label isn't in front. If a DIFFERENT object (e.g. a body 'cube' on a set1 run)
            # is sitting at the grab point, this spot is wrong -> blacklist it and go elsewhere.
            other = self._body_nearest_any()
            if other is not None and other[0] != self._body_label():
                self.get_logger().info(
                    f"ALIGN: body sees '{other[0]}' (not '{self._body_label()}') at grab -> skip this spot")
                if self.current_target is not None:
                    self._blacklist(self.current_target.id)
                self.current_target = None
                self._align_fail_count = 0
                self._enter("SELECT_TARGET")
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
        ey = obj_y - self.grab_y                                # LATERAL always from the body cam
        # FORWARD: the front sonar reads distance to the object DIRECTLY (no body-cam projection error)
        # -> drive so it equals grab_range_m. Fall back to the body-cam x when the sonar is stale/absent.
        if self._sonar_fresh():
            ex = self._front_range - self.grab_range_m          # + = object too far -> drive forward
            ex_tol = self.align_sonar_tol
        else:
            ex = obj_x - self.grab_x
            ex_tol = self.align_fwd_tol
        if abs(ex) < ex_tol and abs(ey) < self.align_tol:       # both axes settled -> grab
            self._drive(0.0, 0.0)
            src = "sonar" if self._sonar_fresh() else "cam"
            self.get_logger().info(
                f"ALIGN ok ({src}): fwd_err={ex * 100:+.1f}cm lat_err={ey * 100:+.1f}cm -> grab")
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
            # Forward safety (body-cam mode): never step PAST the blind limit. ex<0 (too near) still
            # steps back freely; sonar mode already backs off when nearer than grab_range_m.
            if not self._sonar_fresh() and obj_x <= self.grab_min_x and vx > 0.0:
                vx = 0.0
        self._pulse_vx, self._pulse_vy = vx, vy
        self._align_phase = "pulse"
        self._align_phase_start = now
        self._drive(vx, vy)

    def _step_classify(self) -> None:
        """Phase-aware pick gate from fresh siglip + shape classifications.

        Pick order is enforced here: in phase 1 only a confirmed Set1 shape is picked; a Set2
        object met here is skipped (NOT blacklisted) so it can be picked in phase 2, and vice
        versa. Recognition/mapping itself is phase-independent and runs continuously upstream.
        """
        # SPATIAL gate: confirm from the CURRENT TARGET's own world-model track (its fused wide+body
        # identity at THIS track's position), not a frame-global /classification/shape or /siglip
        # that may belong to a neighbouring distractor. Set1 = the track already carries the shape
        # (YOLO, wide+body); Set2 = the track carries the SigLIP fruit in fruit_label. Re-look it up
        # fresh so close-range body/siglip observations during APPROACH are included.
        # Target = object of this phase's set_type nearest the grab point (robust to id churn).
        # Confirm the SPECIFIC target in front: phase 1 needs the set1 shape by label (a neighbouring
        # cube is NOT it); phase 2 any fruit cube (its fruit type is checked via fruit_label below).
        if self.phase == 1:
            tgt = self._front_target(1, want_label=self.set1_label)
        else:
            tgt = self._front_target(2)
        if tgt is None:
            # Momentarily not detected -> HOLD and wait for it to reappear (don't thrash back to
            # SELECT). Only give up after classify_timeout, and never blacklist a phantom (reselect).
            self._drive(0.0, 0.0)
            if self._time_in_state() > self.classify_timeout_sec:
                self.get_logger().info("CLASSIFY: target lost past timeout -> reselect")
                self.current_target = None
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
            self.set2_label
            and tgt is not None
            and tgt.set_type == 2
            and tgt.fruit_label == self.set2_label
            and tgt.confidence >= self.pick_track_conf
        )

        if self.phase == 1:
            if set1_ok:
                self._commit_pick(1, tgt.class_label, f"track conf={tgt.confidence:.2f}")
                return
            if set2_ok:
                self._skip_target(f"set2 '{tgt.fruit_label}' during Set1 phase")
                return
        else:  # phase 2
            if set2_ok:
                self._commit_pick(2, tgt.fruit_label, f"track conf={tgt.confidence:.2f} (siglip fruit)")
                return
            if set1_ok:
                self._skip_target(f"stray set1 '{tgt.class_label}' during Set2 phase")
                return
            # SigLIP already typed it a DIFFERENT fruit -> blacklist NOW (don't wait the classify timeout)
            if tgt.fruit_label and str(tgt.fruit_label) != self.set2_label:
                self.get_logger().info(f"CLASSIFY: fruit '{tgt.fruit_label}' != {self.set2_label} -> skip now")
                self._blacklist(tgt.id)
                self.current_target = None
                self.set_type = 0
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
            self._enter("SELECT_TARGET")

    # --------------------------------------------------------------------- tick
    def tick(self) -> None:
        # 1) drive condition-based transitions
        self._step()

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
