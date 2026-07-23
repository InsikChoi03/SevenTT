"""Simple hunter mission node — "trust strong positives only, go fast".

A deliberately minimal alternative to mission_fsm_node (same topics, same arm/base
interfaces, same perception stack). Behaviour:

    strong high-confidence target on the world map?
        -> drive straight at it (LanePlanner collision check, direct when clear)
        -> body-cam translation-pulse ALIGN (verified field constants)
        -> trigger the 2R pick, resume driving the moment the arm lifts
        -> next target
    no strong target?
        -> race to the nearest 0.5 m grid node the wide camera has NOT covered yet,
           marking coverage with the calibrated wide-FOV ellipse while moving

Everything else from the field FSM is intentionally dropped: no zones, no anchors,
no slots, no zigzag route, no classify voting, no retry hierarchies, no storage
return (param-gated END on quota / match timer instead).

Interfaces (identical to mission_fsm_node so the rest of the stack is untouched):
    sub  /world_model, /localization/pose, /camera_body/detections,
         /arm/pick_sequence_phase, /competition/state, /state_advance
    pub  /base_command, /arm/pick_trigger, /mission_state, /planning/phase,
         /world_model/mapping_enabled, /world_model/track_birth_enabled,
         /world_model/blacklist_add
"""
from __future__ import annotations

import math

import numpy as np

try:
    import cv2
except Exception:  # noqa: BLE001 - node must come up even without OpenCV (no ALIGN servo)
    cv2 = None

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Int8, String, UInt64

from robot_interfaces.msg import (
    BaseCommand,
    Classification,
    DetectionArray,
    MissionState,
    WorldModel,
)

from robot_planning.global_target_planner import (
    grid_nodes,
    nodes_in_ellipse_fov,
)
from robot_planning.lane_planner import LanePlanner
from robot_planning.simple_hunter import (
    align_translation_axis,
    clamp_outward_field_velocity,
    fruit_conflict_action,
    is_strong_set2,
    position_banned,
    prune_bans,
    rectify_rectilinear,
    radial_standoff,
    select_body_candidate,
    select_hunt_target,
    wrap_angle,
)

COMPETITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Arm phases that mean the grasp is done and the base may drive again.
ARM_LIFTED_PHASES = {"LIFT", "TO_PLACE", "PLACE", "STOW", "COMPLETE"}


class SimpleHunterNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_hunter_node")

        # ---------------------------------------------------------------- params
        p = self.declare_parameter
        p("rate_hz", 10.0)
        p("auto_start", False)             # true: skip the START button (bench test)
        p("set1_label", "cube")
        p("set2_label", "pineapple")
        p("set1_total", 4)
        p("set2_total", 3)
        p("match_stop_sec", 0.0)           # >0: hard stop this long after start (no storage)

        # strong-positive gate — the whole point of this node
        p("strong_set1_min_conf", 0.80)
        p("strong_set2_min_margin_body", 0.50)
        p("strong_set2_min_margin_wide", 0.70)
        p("strong_set2_min_track_conf", 0.50)
        p("strong_min_obs", 2)
        p("target_lost_grace_sec", 2.5)
        p("siglip_fresh_sec", 2.5)         # freshest body SigLIP read usable at the grab point
        p("target_match_radius_m", 0.25)   # re-associate a target whose track id changed
        p("ban_radius_m", 0.22)
        p("fail_ban_sec", 30.0)            # align failure: retryable later
        p("picked_ban_sec", 0.0)           # 0 = permanent (stale track at pick site)

        # driving
        p("hunt_speed", 0.18)
        p("explore_speed", 0.20)
        p("final_speed", 0.12)
        p("slowdown_radius_m", 0.50)
        p("face_tol_rad", 0.35)
        p("face_exit_tol_rad", 0.12)       # keep turning until this close (hysteresis)
        # rotation: continuous down to a small error (fast), then stop + short trim
        # pulses (continuous control alone oscillates on this base; tiny pulses alone
        # take forever — 0.12 rad/s * 0.22 s is ~1.5deg per pulse)
        p("turn_continuous_min_rad", 0.25)  # switch to trim pulses below this error
        p("turn_pulse_omega", 0.15)
        p("turn_pulse_sec", 0.15)
        p("turn_pulse_fine_sec", 0.10)     # short pulse when error < turn_fine_rad
        p("turn_fine_rad", 0.17)
        p("turn_settle_sec", 0.25)         # full stop after each pulse to remeasure
        p("explore_turn_weight", 0.6)      # metres of detour worth one radian of turning
        p("turn_kp", 1.5)
        p("turn_omega_max", 0.405)
        p("turn_omega_min", 0.12)
        p("drive_kp", 0.6)
        p("drive_omega_max", 0.08)
        p("wp_reach_tol_m", 0.12)
        p("explore_reach_tol_m", 0.18)
        p("pose_max_age_sec", 0.75)

        # field / planner / coverage (matches world model + field FSM geometry)
        p("field_bounds_m", [-2.0, 2.0, -2.0, 2.0])
        p("robot_margin_m", 0.22)
        p("boundary_guard_m", 0.03)
        p("grid_spacing_m", 0.50)
        p("lane_block_radius_m", 0.24)
        # false = always travel the lane midlines (match-proven; the map misses objects,
        # so a "clear" direct shortcut can charge through unmapped cubes)
        p("lane_simplify_enabled", False)
        p("direct_fallback_clear_m", 0.18)  # min clearance for the no-lane-route straight shot
        p("obstacle_min_conf", 0.4)
        p("obstacle_min_nobs", 2)
        p("exclude_target_radius_m", 0.12)
        p("replan_goal_move_m", 0.15)
        p("coverage_rows", 6)
        p("coverage_cols", 7)
        p("coverage_origin_x_m", -1.50)
        p("coverage_origin_y_m", -1.50)
        # coverage "seen" ellipse: intentionally MUCH smaller than the physical wide
        # FOV (1.1/1.3). "Seen" must mean "identification-grade look", not "grazed the
        # footprint" — a generous ellipse marks the cells ahead at match start and
        # the sweep skips its own first waypoints, beelining into the field.
        p("wide_fov_forward_m", 0.65)
        p("wide_fov_lateral_m", 0.65)
        p("wide_fov_center_x_m", 0.4)
        p("wide_fov_min_range_m", 0.15)

        # approach / align / pick — verified field constants (motion_tuning.yaml values)
        p("approach_dist_m", 0.375)
        p("approach_standoff_tol_m", 0.09)
        p("approach_brake_settle_sec", 0.40)
        p("approach_heading_tol_rad", 0.10)
        p("approach_heading_timeout_sec", 6.0)
        p("body_fresh_max_age_sec", 0.35)
        # drive-in align: once the body cam sees the target, keep DRIVING and steer the
        # grab point onto it; speed tapers with remaining distance and lateral error is
        # corrected DURING the drive so the final pulse align is rarely needed.
        p("drive_in_align_enabled", True)
        p("drive_in_speed_max", 0.13)
        p("drive_in_speed_min", 0.06)
        p("drive_in_speed_kv", 0.5)        # v = kv * (bx - stop_x), clamped to [min, max]
        p("drive_in_kp", 1.2)
        p("drive_in_omega_max", 0.15)
        p("drive_in_vy_kp", 1.5)           # lateral duty per metre of body-y error
        p("drive_in_vy_max", 0.20)
        p("drive_in_stop_lead_m", 0.02)    # brake-slip allowance before grab_x
        p("drive_in_lost_sec", 0.6)
        p("drive_in_engage_dist_m", 0.90)  # only near the committed target (body sees ~0.68m)
        # investigate: a plausible-but-unproven map candidate beats blind exploration —
        # drive into body-cam range and let the evidence decide
        p("investigate_min_conf", 0.45)
        p("investigate_standoff_m", 0.55)
        p("investigate_wait_sec", 2.0)
        p("investigate_fail_ban_sec", 15.0)
        # only check candidates we are already passing — the sweep visits everything,
        # and chasing every distant fruit cube (12 on the field) thrashes the robot
        p("investigate_max_range_m", 0.90)
        p("investigate_cooldown_sec", 4.0)
        p("grab_x", 0.20)
        p("grab_y", 0.0)
        p("grab_min_x", 0.17)
        p("align_fwd_tol_m", 0.03)
        p("align_tol_m", 0.05)
        p("align_step_fwd_duty", 0.27)
        p("align_step_fwd_sec", 0.15)
        p("align_step_fwd_mid_sec", 0.25)
        p("align_step_strafe_duty", 0.315)
        p("align_step_strafe_sec", 0.30)
        p("align_step_strafe_mid_sec", 0.60)
        p("align_mid_error_m", 0.06)
        p("align_settle_pulse_sec", 0.6)
        p("align_confirm_frames", 3)
        p("align_timeout_sec", 10.0)
        p("align_body_lost_sec", 1.2)
        p("align_backoff_speed", 0.189)
        p("align_backoff_sec", 0.54)
        p("align_backoff_settle_sec", 1.0)
        p("pick_duration_sec", 11.0)
        p("arm_phase_max_age_sec", 0.50)

        # stuck escape (minimal)
        p("stuck_detect_sec", 3.0)
        p("stuck_pose_delta_m", 0.025)
        p("stuck_heading_delta_rad", 0.04)
        p("stuck_escape_pulse_sec", 0.35)
        p("stuck_escape_speed", 0.12)

        # body-cam ground homography (same calibration as mission_fsm_node)
        p("body_ground_homography_path",
          "/home/seventt/seventt/workspace/data/calib/body_ground.npz")
        p("body_cam_nadir_x", 0.055)
        p("body_cam_height_m", 0.155)
        p("object_center_height_m", 0.04)
        p("body_px_scale_x", 2.5625)
        p("body_px_scale_y", 2.566667)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.auto_start = bool(g("auto_start"))
        self.set1_label = str(g("set1_label"))
        self.set2_label = str(g("set2_label"))
        self.set1_total = int(g("set1_total"))
        self.set2_total = int(g("set2_total"))
        self.match_stop_sec = float(g("match_stop_sec"))
        self.strong_set1_min_conf = float(g("strong_set1_min_conf"))
        self.strong_set2_min_margin_body = float(g("strong_set2_min_margin_body"))
        self.strong_set2_min_margin_wide = float(g("strong_set2_min_margin_wide"))
        self.strong_set2_min_track_conf = float(g("strong_set2_min_track_conf"))
        self.strong_min_obs = int(g("strong_min_obs"))
        self.target_lost_grace_sec = float(g("target_lost_grace_sec"))
        self.siglip_fresh_sec = float(g("siglip_fresh_sec"))
        self.target_match_radius_m = float(g("target_match_radius_m"))
        self.ban_radius_m = float(g("ban_radius_m"))
        self.fail_ban_sec = float(g("fail_ban_sec"))
        self.picked_ban_sec = float(g("picked_ban_sec"))
        self.hunt_speed = float(g("hunt_speed"))
        self.explore_speed = float(g("explore_speed"))
        self.final_speed = float(g("final_speed"))
        self.slowdown_radius_m = float(g("slowdown_radius_m"))
        self.face_tol_rad = float(g("face_tol_rad"))
        self.face_exit_tol_rad = float(g("face_exit_tol_rad"))
        self.turn_continuous_min_rad = float(g("turn_continuous_min_rad"))
        self.turn_pulse_omega = float(g("turn_pulse_omega"))
        self.turn_pulse_sec = float(g("turn_pulse_sec"))
        self.turn_pulse_fine_sec = float(g("turn_pulse_fine_sec"))
        self.turn_fine_rad = float(g("turn_fine_rad"))
        self.turn_settle_sec = float(g("turn_settle_sec"))
        self.explore_turn_weight = float(g("explore_turn_weight"))
        self.turn_kp = float(g("turn_kp"))
        self.turn_omega_max = float(g("turn_omega_max"))
        self.turn_omega_min = float(g("turn_omega_min"))
        self.drive_kp = float(g("drive_kp"))
        self.drive_omega_max = float(g("drive_omega_max"))
        self.wp_reach_tol_m = float(g("wp_reach_tol_m"))
        self.explore_reach_tol_m = float(g("explore_reach_tol_m"))
        self.pose_max_age_sec = float(g("pose_max_age_sec"))
        fb = [float(v) for v in g("field_bounds_m")]
        self.field_bounds = (fb[0], fb[1], fb[2], fb[3])
        self.robot_margin_m = float(g("robot_margin_m"))
        self.boundary_guard_m = float(g("boundary_guard_m"))
        self.obstacle_min_conf = float(g("obstacle_min_conf"))
        self.obstacle_min_nobs = int(g("obstacle_min_nobs"))
        self.exclude_target_radius_m = float(g("exclude_target_radius_m"))
        self.replan_goal_move_m = float(g("replan_goal_move_m"))
        self.wide_fov_forward_m = float(g("wide_fov_forward_m"))
        self.wide_fov_lateral_m = float(g("wide_fov_lateral_m"))
        self.wide_fov_center_x_m = float(g("wide_fov_center_x_m"))
        self.wide_fov_min_range_m = float(g("wide_fov_min_range_m"))
        self.approach_dist_m = float(g("approach_dist_m"))
        self.approach_standoff_tol_m = float(g("approach_standoff_tol_m"))
        self.approach_brake_settle_sec = float(g("approach_brake_settle_sec"))
        self.approach_heading_tol_rad = float(g("approach_heading_tol_rad"))
        self.approach_heading_timeout_sec = float(g("approach_heading_timeout_sec"))
        self.body_fresh_max_age_sec = float(g("body_fresh_max_age_sec"))
        self.drive_in_align_enabled = bool(g("drive_in_align_enabled"))
        self.drive_in_speed_max = float(g("drive_in_speed_max"))
        self.drive_in_speed_min = float(g("drive_in_speed_min"))
        self.drive_in_speed_kv = float(g("drive_in_speed_kv"))
        self.drive_in_kp = float(g("drive_in_kp"))
        self.drive_in_omega_max = float(g("drive_in_omega_max"))
        self.drive_in_vy_kp = float(g("drive_in_vy_kp"))
        self.drive_in_vy_max = float(g("drive_in_vy_max"))
        self.drive_in_stop_lead_m = float(g("drive_in_stop_lead_m"))
        self.drive_in_lost_sec = float(g("drive_in_lost_sec"))
        self.drive_in_engage_dist_m = float(g("drive_in_engage_dist_m"))
        self.investigate_min_conf = float(g("investigate_min_conf"))
        self.investigate_standoff_m = float(g("investigate_standoff_m"))
        self.investigate_wait_sec = float(g("investigate_wait_sec"))
        self.investigate_fail_ban_sec = float(g("investigate_fail_ban_sec"))
        self.investigate_max_range_m = float(g("investigate_max_range_m"))
        self.investigate_cooldown_sec = float(g("investigate_cooldown_sec"))
        self.grab_x = float(g("grab_x"))
        self.grab_y = float(g("grab_y"))
        self.grab_min_x = float(g("grab_min_x"))
        self.align_fwd_tol_m = float(g("align_fwd_tol_m"))
        self.align_tol_m = float(g("align_tol_m"))
        self.align_step_fwd_duty = float(g("align_step_fwd_duty"))
        self.align_step_fwd_sec = float(g("align_step_fwd_sec"))
        self.align_step_fwd_mid_sec = float(g("align_step_fwd_mid_sec"))
        self.align_step_strafe_duty = float(g("align_step_strafe_duty"))
        self.align_step_strafe_sec = float(g("align_step_strafe_sec"))
        self.align_step_strafe_mid_sec = float(g("align_step_strafe_mid_sec"))
        self.align_mid_error_m = float(g("align_mid_error_m"))
        self.align_settle_pulse_sec = float(g("align_settle_pulse_sec"))
        self.align_confirm_frames = int(g("align_confirm_frames"))
        self.align_timeout_sec = float(g("align_timeout_sec"))
        self.align_body_lost_sec = float(g("align_body_lost_sec"))
        self.align_backoff_speed = float(g("align_backoff_speed"))
        self.align_backoff_sec = float(g("align_backoff_sec"))
        self.align_backoff_settle_sec = float(g("align_backoff_settle_sec"))
        self.pick_duration_sec = float(g("pick_duration_sec"))
        self.arm_phase_max_age_sec = float(g("arm_phase_max_age_sec"))
        self.stuck_detect_sec = float(g("stuck_detect_sec"))
        self.stuck_pose_delta_m = float(g("stuck_pose_delta_m"))
        self.stuck_heading_delta_rad = float(g("stuck_heading_delta_rad"))
        self.stuck_escape_pulse_sec = float(g("stuck_escape_pulse_sec"))
        self.stuck_escape_speed = float(g("stuck_escape_speed"))

        self.direct_fallback_clear_m = float(g("direct_fallback_clear_m"))
        spacing = float(g("grid_spacing_m"))
        self.planner = LanePlanner(
            spacing=spacing,
            bounds=self.field_bounds,
            margin=self.robot_margin_m,
            block_radius=float(g("lane_block_radius_m")),
            origin_mode="fixed",
            origin_xy=(0.0, 0.0),
            simplify=bool(g("lane_simplify_enabled")),
        )
        self.coverage_nodes = grid_nodes(
            int(g("coverage_rows")), int(g("coverage_cols")),
            float(g("coverage_origin_x_m")), float(g("coverage_origin_y_m")), spacing,
        )
        self.seen_nodes: set[int] = set()
        # object-lattice coordinate -> coverage node index, for the sweep skip check
        self._cov_index = {
            (round(x * 100), round(y * 100)): i
            for i, (x, y) in enumerate(self.coverage_nodes)
        }
        self.sweep: list[tuple[float, float]] = []
        self.sweep_idx = 0

        # body homography (pose-independent ALIGN servo, same math as the field FSM)
        self._body_px_sx = float(g("body_px_scale_x"))
        self._body_px_sy = float(g("body_px_scale_y"))
        self._bnx = float(g("body_cam_nadir_x"))
        self._bH = float(g("body_cam_height_m"))
        self._boh = float(g("object_center_height_m"))
        self._body_H = None
        if cv2 is not None:
            try:
                path = str(g("body_ground_homography_path"))
                self._body_H = np.load(path)["H"].astype(np.float64)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"body homography load failed ({exc}); ALIGN unavailable")

        # ---------------------------------------------------------------- state
        self.state = "WAIT_START"
        self.state_enter_s = 0.0
        self.match_start_s: float | None = None
        self.tray_shape = 0
        self.tray_fruit = 0
        self.bans: list[tuple[float, float, float]] = []

        self.pose: tuple[float, float, float] | None = None
        self.pose_stamp_s = 0.0
        self.world_objects: list = []
        self.world_stamp_s = 0.0

        self.target = None                       # latest matched Object
        self.target_xy: tuple[float, float] | None = None   # frozen approach anchor
        self.target_set_type = 0
        self.target_last_seen_s = 0.0
        self.standoff_xy: tuple[float, float] | None = None

        self.route: list[tuple[float, float]] = []
        self.route_goal: tuple[float, float] | None = None
        self._route_turning = False
        self._turn_pulse_until = 0.0
        self._turn_settle_until = 0.0
        self._turn_pulse_cmd = 0.0
        self.explore_goal: tuple[float, float] | None = None
        self.explore_goal_idx: int | None = None

        self.approach_phase = "travel"           # travel | drive_in | settle | heading
        self.phase_t0 = 0.0
        self.drive_in_seen_s = 0.0
        self.invest_xy: tuple[float, float] | None = None
        self.invest_standoff: tuple[float, float] | None = None
        self._invest_standoff_for = (0.0, 0.0)
        self.invest_cooldown_until_s = 0.0
        self.invest_seen_s = 0.0
        self.invest_phase = "travel"             # travel | wait

        self.align_phase = "settle"        # settle|measure|pulse|backoff|backoff_settle
        self.align_pulse = (0.0, 0.0, 0.0)       # vx, vy, until_s
        self.align_ok_frames = 0
        self.align_last_body_seq = -1
        self.align_body_seen_s = 0.0
        self.align_backoffs = 0

        self.body_dets: list[tuple[float, float, str, float]] = []
        self.body_stamp_s = 0.0
        self.body_seq = 0
        self.siglip: tuple[str, float, bool, float] | None = None  # label, margin, face, rx_s

        self.arm_phase = ""
        self.arm_phase_rx_s = 0.0
        self.pick_enter_s = 0.0
        self.pick_triggered = False
        self.fruit_conflict_wait = False

        self.stuck_ref: tuple[float, float, float, float] | None = None  # x, y, th, t
        self.stuck_pulse_until_s = 0.0
        self.stuck_pulse_cmd = (0.0, 0.0)

        # ---------------------------------------------------------------- I/O
        self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)
        self.pub_pick = self.create_publisher(Bool, "/arm/pick_trigger", 10)
        self.pub_state = self.create_publisher(MissionState, "/mission_state", 10)
        self.pub_phase = self.create_publisher(Int8, "/planning/phase", 10)
        self.pub_mapping = self.create_publisher(Bool, "/world_model/mapping_enabled", 10)
        self.pub_birth = self.create_publisher(Bool, "/world_model/track_birth_enabled", 10)
        self.pub_blacklist = self.create_publisher(UInt64, "/world_model/blacklist_add", 10)

        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_dets, 10)
        self.create_subscription(String, "/arm/pick_sequence_phase", self.on_arm_phase, 10)
        self.create_subscription(Classification, "/classification/siglip", self.on_siglip, 10)
        self.create_subscription(
            String, "/competition/state", self.on_competition_state, COMPETITION_QOS)
        self.create_subscription(Empty, "/state_advance", self.on_advance, 10)

        rate = float(g("rate_hz"))
        self.create_timer(1.0 / max(1.0, rate), self.tick)
        self.get_logger().info(
            f"simple_hunter ready: set1={self.set1_label}x{self.set1_total} "
            f"set2={self.set2_label}x{self.set2_total} "
            f"strong(conf>={self.strong_set1_min_conf}, "
            f"margin body>={self.strong_set2_min_margin_body}"
            f"/wide>={self.strong_set2_min_margin_wide}) auto_start={self.auto_start}"
        )

    # ------------------------------------------------------------------ utils
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _time_in_state(self) -> float:
        return self._now_s() - self.state_enter_s

    def _enter(self, state: str) -> None:
        if state != self.state:
            self.get_logger().info(f"[{self.state} -> {state}]")
        self.state = state
        self.state_enter_s = self._now_s()
        self.fruit_conflict_wait = False
        self.explore_goal = None
        self.explore_goal_idx = None

    def _drive(self, vx: float, vy: float, omega: float = 0.0) -> None:
        if self.pose is not None and (vx != 0.0 or vy != 0.0):
            vx, vy = clamp_outward_field_velocity(
                vx, vy, self.pose, self.field_bounds,
                self.robot_margin_m, self.boundary_guard_m,
            )
        c = BaseCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = "base_link"
        c.vx, c.vy, c.omega = float(vx), float(vy), float(omega)
        self.pub_cmd.publish(c)

    def _stop(self) -> None:
        self._drive(0.0, 0.0, 0.0)

    def _pose_fresh(self) -> bool:
        if self.pose is None:
            return False
        return (self._now_s() - self.pose_stamp_s) <= self.pose_max_age_sec

    # ------------------------------------------------------------------ inputs
    def on_world(self, msg: WorldModel) -> None:
        self.world_objects = list(msg.objects)
        self.world_stamp_s = self._now_s()

    def on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (float(msg.pose.position.x), float(msg.pose.position.y), theta)
        self.pose_stamp_s = self._now_s()

    def on_body_dets(self, msg: DetectionArray) -> None:
        self.body_dets = [
            (float(d.x_center), float(d.y_center), str(d.label), float(d.confidence))
            for d in msg.detections
        ]
        self.body_stamp_s = self._now_s()
        self.body_seq += 1

    def on_arm_phase(self, msg: String) -> None:
        self.arm_phase = str(msg.data).strip().upper()
        self.arm_phase_rx_s = self._now_s()

    def on_siglip(self, msg: Classification) -> None:
        self.siglip = (
            str(msg.label), float(msg.confidence), bool(msg.image_face_visible), self._now_s())

    def _fresh_siglip(self, min_rx_s: float = 0.0) -> tuple[str, float, bool] | None:
        if self.siglip is None:
            return None
        label, margin, face, rx_s = self.siglip
        if rx_s < min_rx_s or self._now_s() - rx_s > self.siglip_fresh_sec:
            return None
        return (label, margin, face)

    def on_competition_state(self, msg: String) -> None:
        if str(msg.data).strip().upper() == "RUNNING" and self.state == "WAIT_START":
            self._start_match()

    def on_advance(self, _msg: Empty) -> None:
        if self.state == "WAIT_START":
            self._start_match()

    def _start_match(self) -> None:
        self.match_start_s = self._now_s()
        self._enter("EXPLORE")
        self.get_logger().info("match start -> hunting")

    # ------------------------------------------------------------------ body servo
    def _body_pixel_base(self, u: float, v: float) -> tuple[float, float] | None:
        if self._body_H is None or cv2 is None:
            return None
        us, vs = float(u) * self._body_px_sx, float(v) * self._body_px_sy
        pt = cv2.perspectiveTransform(np.array([[[us, vs]]], np.float64), self._body_H)[0][0]
        k = self._boh / self._bH
        return float(pt[0]) - k * (float(pt[0]) - self._bnx), float(pt[1]) - k * float(pt[1])

    def _align_allowed_labels(self) -> set[str]:
        if self.target_set_type == 2:
            # near the gripper the fruit face is often occluded -> generic cube is fine;
            # the fruit identity was already committed from the map's strong positive.
            return {"fruit_photo_cube", "cube"}
        return {self.set1_label}

    def _body_target(self) -> tuple[tuple[str, float, float, float] | None, bool]:
        """Return (candidate near grab point, fruit_conflict).

        fruit_conflict: the set1 "cube" target shows a fruit_photo_cube face at the
        grab point — it is a fruit cube, NOT a plain cube. The caller decides whether
        to switch to a set2 pick or walk away; it must never be picked as set1.
        """
        if not self.body_dets:
            return None, False
        allowed = self._align_allowed_labels()
        candidates = []
        fruit_near = None
        for u, v, label, conf in self.body_dets:
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            if label == "fruit_photo_cube":
                d = math.hypot(base[0] - self.grab_x, base[1] - self.grab_y)
                if fruit_near is None or d < fruit_near[0]:
                    fruit_near = (d, base)
            if label in allowed:
                candidates.append((label, base[0], base[1], conf))
        chosen = select_body_candidate(candidates, (self.grab_x, self.grab_y))
        if self.target_set_type == 1 and self.set1_label == "cube" and fruit_near is not None:
            _d, (fx, fy) = fruit_near
            if chosen is None and _d <= 0.50:
                return None, True          # only the fruit face is visible at the grab point
            if chosen is not None and math.hypot(fx - chosen[1], fy - chosen[2]) <= 0.10:
                return None, True          # nested cube+fruit at the same spot
        return chosen, False

    def _body_guidance_xy(self) -> tuple[float, float] | None:
        """Steering-only body fix: nearest plausible object to the grab point.

        Label identity is NOT decided here (that happens at the ALIGN measure, incl.
        the fruit-conflict rules); this is just "where is the thing I'm driving at".
        """
        if self._now_s() - self.body_stamp_s > self.body_fresh_max_age_sec:
            return None
        labels = self._align_allowed_labels() | {"fruit_photo_cube"}
        best = None
        best_d = 0.65
        for u, v, label, _conf in self.body_dets:
            if label not in labels:
                continue
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            d = math.hypot(base[0] - self.grab_x, base[1] - self.grab_y)
            if d < best_d:
                best_d = d
                best = base
        return best

    # ------------------------------------------------------------------ map helpers
    def _obstacles(self, exclude_xy: tuple[float, float] | None) -> list[tuple[float, float]]:
        obs = []
        for o in self.world_objects:
            if float(o.confidence) < self.obstacle_min_conf:
                continue
            if int(o.n_obs) < self.obstacle_min_nobs:
                continue
            x, y = float(o.x), float(o.y)
            if exclude_xy is not None:
                d = math.hypot(x - exclude_xy[0], y - exclude_xy[1])
                if d <= self.exclude_target_radius_m:
                    continue
            obs.append((x, y))
        return obs

    def _refresh_target(self) -> None:
        """Track the committed target across id changes; update last-seen time."""
        if self.target is None or self.target_xy is None:
            return
        tid = int(self.target.id)
        tx, ty = self.target_xy
        best = None
        best_d = self.target_match_radius_m
        for o in self.world_objects:
            if int(o.id) == tid:
                best = o
                break
            # match by position only: the class/set_type vote often flips while
            # closing in (fruit cube <-> cube), and that must not orphan the target
            d = math.hypot(float(o.x) - tx, float(o.y) - ty)
            if d <= best_d:
                best, best_d = o, d
        if best is not None and not bool(best.blacklisted):
            self.target = best
            self.target_last_seen_s = self._now_s()

    def _select_target(self):
        if self.pose is None:
            return None
        return select_hunt_target(
            self.world_objects,
            (self.pose[0], self.pose[1]),
            set1_label=self.set1_label,
            set2_label=self.set2_label,
            set1_remaining=self.set1_total - self.tray_shape,
            set2_remaining=self.set2_total - self.tray_fruit,
            set1_min_conf=self.strong_set1_min_conf,
            set2_min_margin_body=self.strong_set2_min_margin_body,
            set2_min_margin_wide=self.strong_set2_min_margin_wide,
            set2_min_track_conf=self.strong_set2_min_track_conf,
            min_obs=self.strong_min_obs,
            bans=self.bans,
            now_s=self._now_s(),
            ban_radius_m=self.ban_radius_m,
        )

    def _select_investigate(self):
        """Plausible-but-unproven map candidate worth driving into camera range for.

        A wide-cam sighting that has not crossed the strong-positive bar yet should
        pull the robot toward it — driving blindly to unseen grid nodes while a
        likely target sits on the map is wasted time.
        """
        if self.pose is None:
            return None
        now = self._now_s()
        set1_left = self.set1_total - self.tray_shape > 0
        set2_left = self.set2_total - self.tray_fruit > 0
        rx, ry = self.pose[0], self.pose[1]
        best = None
        best_d = math.inf
        for o in self.world_objects:
            if bool(o.blacklisted):
                continue
            x, y = float(o.x), float(o.y)
            if position_banned(x, y, self.bans, now, self.ban_radius_m):
                continue
            fruit = str(o.fruit_label or "")
            if fruit and fruit != self.set2_label:
                continue                     # map already says a different fruit
            plausible = False
            if set2_left and (fruit or str(o.class_label) == "fruit_photo_cube"):
                plausible = True
            if (
                set1_left and not fruit and int(o.set_type) == 1
                and str(o.class_label) == self.set1_label
                and float(o.confidence) >= self.investigate_min_conf
            ):
                plausible = True
            if not plausible:
                continue
            d = math.hypot(x - rx, y - ry)
            if d > self.investigate_max_range_m:
                continue                     # the sweep will bring us past it anyway
            if d < best_d:
                best_d = d
                best = o
        return best

    def _commit_target(self, obj) -> None:
        self.target = obj
        self.target_xy = (float(obj.x), float(obj.y))
        # fruit evidence outranks the frozen class vote: a fruit-labeled track is a
        # fruit cube even when its set_type froze as 1 ("cube" at distance)
        strong2 = is_strong_set2(
            obj, self.set2_label,
            self.strong_set2_min_margin_body, self.strong_set2_min_margin_wide,
            self.strong_set2_min_track_conf, self.strong_min_obs,
        )
        self.target_set_type = 2 if strong2 else int(obj.set_type)
        self.target_last_seen_s = self._now_s()
        self.standoff_xy = None
        self.route = []
        self.route_goal = None
        self.approach_phase = "travel"
        self.get_logger().info(
            f"HUNT id={int(obj.id)} {obj.class_label}/{obj.fruit_label or '-'} "
            f"conf={float(obj.confidence):.2f} at ({obj.x:.2f},{obj.y:.2f})"
        )
        self._enter("APPROACH")

    def _abandon_target(self, reason: str, ban_sec: float, *, blacklist: bool = False) -> None:
        # transient failures only BAN the position for a while so the target can be
        # retried later; blacklist (permanent, propagated to the world model) is
        # reserved for "definitively not our target" verdicts.
        if self.target is not None and self.target_xy is not None:
            self.get_logger().info(f"ABANDON id={int(self.target.id)}: {reason}")
            expire = self._now_s() + ban_sec if ban_sec > 0.0 else 0.0
            self.bans.append((self.target_xy[0], self.target_xy[1], expire))
            if blacklist:
                self.pub_blacklist.publish(UInt64(data=int(self.target.id)))
        self.target = None
        self.target_xy = None
        self.standoff_xy = None
        self.route = []
        self._enter("EXPLORE")

    # ------------------------------------------------------------------ coverage
    def _update_coverage(self) -> None:
        if not self._pose_fresh():
            return
        self.seen_nodes |= nodes_in_ellipse_fov(
            self.pose, self.coverage_nodes,
            self.wide_fov_forward_m, self.wide_fov_lateral_m,
            self.wide_fov_center_x_m, self.wide_fov_min_range_m,
        )

    # ------------------------------------------------------------------ routing
    def _plan_route(
        self, goal: tuple[float, float], exclude_xy: tuple[float, float] | None
    ) -> bool:
        """Plan a collision-checked route to goal. False = no safe route exists."""
        if self.pose is None:
            return False
        obstacles = self._obstacles(exclude_xy)
        start = (self.pose[0], self.pose[1])
        path = self.planner.plan(start, goal, obstacles, route_mode="legacy")
        if path:
            # collapse the 4-connected staircase into long straight runs with single
            # L corners (stays on lane lines; no diagonal cuts across lattice points)
            pts = rectify_rectilinear(
                [start] + list(path),
                lambda a, b: self.planner._seg_free(a, b, obstacles),
            )
            self.route = pts[1:] if len(pts) > 1 else list(path)
            self.route_goal = goal
            return True
        # no lane route: a straight shot is allowed ONLY when the known map clears it
        if self.planner._seg_min_clear(start, goal, obstacles) >= self.direct_fallback_clear_m:
            self.route = [goal]
            self.route_goal = goal
            return True
        self.get_logger().info(f"route to ({goal[0]:.2f},{goal[1]:.2f}) blocked")
        self.route = []
        self.route_goal = None
        return False

    def _follow_route(self, speed: float, reach_tol: float) -> bool:
        """Drive along self.route; True when the final point is reached."""
        if self.pose is None or not self.route:
            self._stop()
            return False
        rx, ry, theta = self.pose
        wx, wy = self.route[0]
        last = len(self.route) == 1
        d = math.hypot(wx - rx, wy - ry)
        tol = reach_tol if last else self.wp_reach_tol_m
        if d <= tol:
            self.route.pop(0)
            if not self.route:
                self._stop()
                return True
            return False
        err = wrap_angle(math.atan2(wy - ry, wx - rx) - theta)
        # hysteresis: once turning, keep turning until well-aligned; once driving,
        # only stop to turn when the error grows past face_tol (no boundary chatter)
        if self._route_turning:
            if abs(err) <= self.face_exit_tol_rad:
                self._route_turning = False
        elif abs(err) > self.face_tol_rad:
            self._route_turning = True
        if self._route_turning:
            self._turn_toward(err)
            return False
        v = speed
        if last and d < self.slowdown_radius_m:
            ratio = d / self.slowdown_radius_m
            v = min(v, self.final_speed + (speed - self.final_speed) * ratio)
        omega = max(-self.drive_omega_max, min(self.drive_omega_max, self.drive_kp * err))
        self._drive(v, 0.0, omega)
        return False

    def _turn_toward(self, err: float) -> None:
        """In-place rotation toward a heading error, pulsed near the target.

        Continuous proportional control oscillates left-right on this base (pose lag
        + brake slip), so below turn_continuous_min_rad we pulse-stop-remeasure like
        the field FSM's verified turn controller.
        """
        now = self._now_s()
        if now < self._turn_pulse_until:
            self._drive(0.0, 0.0, self._turn_pulse_cmd)
            return
        if now < self._turn_settle_until:
            self._stop()
            return
        if abs(err) >= self.turn_continuous_min_rad:
            omega = math.copysign(
                max(self.turn_omega_min, min(self.turn_omega_max, self.turn_kp * abs(err))), err)
            self._drive(0.0, 0.0, omega)
            # brake + settle the moment we cross into the trim band, else we coast past it
            self._turn_settle_until = self._now_s() + self.turn_settle_sec \
                if abs(err) < self.turn_continuous_min_rad * 1.3 else 0.0
            return
        fine = abs(err) < self.turn_fine_rad
        pulse_sec = self.turn_pulse_fine_sec if fine else self.turn_pulse_sec
        self._turn_pulse_cmd = math.copysign(self.turn_pulse_omega, err)
        self._turn_pulse_until = now + pulse_sec
        self._turn_settle_until = self._turn_pulse_until + self.turn_settle_sec
        self._drive(0.0, 0.0, self._turn_pulse_cmd)

    # ------------------------------------------------------------------ stuck escape
    def _stuck_check(self, commanding: bool) -> bool:
        """True while an escape pulse is being driven (caller should return)."""
        now = self._now_s()
        if now < self.stuck_pulse_until_s:
            self._drive(self.stuck_pulse_cmd[0], self.stuck_pulse_cmd[1], 0.0)
            return True
        if not commanding or not self._pose_fresh():
            self.stuck_ref = None
            return False
        x, y, th = self.pose
        if self.stuck_ref is None:
            self.stuck_ref = (x, y, th, now)
            return False
        rx, ry, rth, t0 = self.stuck_ref
        moved = math.hypot(x - rx, y - ry) > self.stuck_pose_delta_m
        turned = abs(wrap_angle(th - rth)) > self.stuck_heading_delta_rad
        if moved or turned:
            self.stuck_ref = (x, y, th, now)
            return False
        if now - t0 >= self.stuck_detect_sec:
            self.get_logger().warn("stuck -> reverse pulse + replan")
            self.stuck_pulse_until_s = now + self.stuck_escape_pulse_sec
            self.stuck_pulse_cmd = (-self.stuck_escape_speed, 0.0)
            self.stuck_ref = None
            self.route = []
            self.route_goal = None
            return True
        return False

    # ------------------------------------------------------------------ states
    def tick(self) -> None:
        now = self._now_s()
        self.bans = prune_bans(self.bans, now)
        if self.state not in ("WAIT_START", "END"):
            self._update_coverage()
            self._refresh_target()
            if (
                self.match_stop_sec > 0.0
                and self.match_start_s is not None
                and now - self.match_start_s >= self.match_stop_sec
                and self.state != "PICK"
            ):
                self.get_logger().info("match timer expired -> END")
                self._enter("END")

        handler = {
            "WAIT_START": self._step_wait,
            "EXPLORE": self._step_explore,
            "INVESTIGATE": self._step_investigate,
            "APPROACH": self._step_approach,
            "ALIGN": self._step_align,
            "PICK": self._step_pick,
            "END": self._step_end,
        }[self.state]
        handler()
        self._publish_status()

    def _step_wait(self) -> None:
        self._stop()
        if self.auto_start:
            self._start_match()

    def _quota_done(self) -> bool:
        return self.tray_shape >= self.set1_total and self.tray_fruit >= self.set2_total

    def _step_explore(self) -> None:
        if self._quota_done():
            self.get_logger().info("quota met -> END")
            self._enter("END")
            return
        cand = self._select_target()
        if cand is not None:
            self._commit_target(cand)
            return
        if not self._pose_fresh():
            self._stop()
            return
        inv = None
        if self._now_s() >= self.invest_cooldown_until_s:
            inv = self._select_investigate()
        if inv is not None:
            self.invest_xy = (float(inv.x), float(inv.y))
            self.invest_standoff = None
            self.invest_seen_s = self._now_s()
            self.invest_phase = "travel"
            self.route = []
            self.route_goal = None
            self.get_logger().info(
                f"INVESTIGATE id={int(inv.id)} {inv.class_label}/{inv.fruit_label or '-'} "
                f"conf={float(inv.confidence):.2f} at ({inv.x:.2f},{inv.y:.2f})")
            self._enter("INVESTIGATE")
            return
        if self._stuck_check(commanding=True):
            return
        # Systematic boustrophedon sweep over the LANE holes (never over object
        # lattice points, where an unmapped cube may sit). Greedy nearest-unseen kept
        # beelining across the field from the start pose; the serpentine is
        # predictable and matches the proven zigzag opening.
        if self.explore_goal is None:
            goal = self._next_sweep_waypoint()
            if goal is None:
                self.get_logger().info("coverage sweep complete -> resetting seen set")
                self.seen_nodes.clear()
                self.sweep = []
                self._stop()
                return
            self.explore_goal = goal
            self.get_logger().info(
                f"EXPLORE -> sweep {self.sweep_idx}/{len(self.sweep)} "
                f"({goal[0]:.2f},{goal[1]:.2f}), "
                f"{len(self.seen_nodes)}/{len(self.coverage_nodes)} cells seen")
        if not self.route and not self._plan_route(self.explore_goal, None):
            self.sweep_idx += 1                  # unreachable hole -> skip it
            self.explore_goal = None
            return
        if self._follow_route(self.explore_speed, self.explore_reach_tol_m):
            self.sweep_idx += 1
            self.explore_goal = None
            self.route_goal = None

    def _build_sweep(self) -> list[tuple[float, float]]:
        """Serpentine over the lane holes that actually border object cells.

        coverage_path() covers the whole field including the empty wall corridors;
        dropping those rows afterwards breaks the serpentine chaining (the next row
        would start at the far end), so re-chain each kept row to start nearest the
        previous row's end point.
        """
        raw = self.planner.coverage_path((self.pose[0], self.pose[1]), [])
        useful = [w for w in raw if self._hole_corners_in_grid(w[0], w[1]) > 0]
        rows: list[list[tuple[float, float]]] = []
        for w in useful:
            if rows and abs(rows[-1][-1][1] - w[1]) < 1e-6:
                rows[-1].append(w)
            else:
                rows.append([w])
        seq: list[tuple[float, float]] = []
        prev_end_x = self.pose[0]
        for row in rows:
            row = sorted(row)
            if abs(row[0][0] - prev_end_x) > abs(row[-1][0] - prev_end_x):
                row.reverse()
            seq.extend(row)
            prev_end_x = row[-1][0]
        return seq

    def _hole_corners_in_grid(self, wx: float, wy: float) -> int:
        count = 0
        for dx in (-0.25, 0.25):
            for dy in (-0.25, 0.25):
                if (round((wx + dx) * 100), round((wy + dy) * 100)) in self._cov_index:
                    count += 1
        return count

    def _next_sweep_waypoint(self) -> tuple[float, float] | None:
        """Next serpentine lane hole whose surrounding cells still need a look."""
        if not self.sweep:
            self.sweep = self._build_sweep()
            self.sweep_idx = 0
        obstacles = self._obstacles(None)
        while self.sweep_idx < len(self.sweep):
            wx, wy = self.sweep[self.sweep_idx]
            if self._sweep_hole_useful(wx, wy, obstacles):
                return (wx, wy)
            self.sweep_idx += 1
        return None

    def _sweep_hole_useful(self, wx: float, wy: float, obstacles) -> bool:
        """A lane hole is worth visiting if any of its 4 lattice corners is unseen."""
        if any(math.hypot(ox - wx, oy - wy) <= 0.24 for ox, oy in obstacles):
            return False                         # hole blocked by a known object
        for dx in (-0.25, 0.25):
            for dy in (-0.25, 0.25):
                idx = self._cov_index.get((round((wx + dx) * 100), round((wy + dy) * 100)))
                if idx is not None and idx not in self.seen_nodes:
                    return True
        return False                             # fully seen

    def _step_investigate(self) -> None:
        cand = self._select_target()
        if cand is not None:                     # evidence crossed the bar -> hunt it
            self._commit_target(cand)
            return
        if not self._pose_fresh() or self.invest_xy is None:
            self._stop()
            if self.invest_xy is None:
                self._enter("EXPLORE")
            return
        now = self._now_s()
        # track the candidate by position; if the map dropped it, go back to exploring
        obj = None
        ix, iy = self.invest_xy
        for o in self.world_objects:
            if bool(o.blacklisted):
                continue
            if math.hypot(float(o.x) - ix, float(o.y) - iy) <= self.target_match_radius_m:
                obj = o
                break
        if obj is not None:
            self.invest_xy = (float(obj.x), float(obj.y))
            self.invest_seen_s = now
        elif now - self.invest_seen_s > self.target_lost_grace_sec:
            self.invest_xy = None
            self.invest_standoff = None
            self.route = []
            self.route_goal = None
            self._enter("EXPLORE")
            return
        rx, ry, theta = self.pose
        ix, iy = self.invest_xy
        dist = math.hypot(ix - rx, iy - ry)

        if self.invest_phase == "travel":
            if self._stuck_check(commanding=True):
                return
            if dist <= self.investigate_standoff_m + self.wp_reach_tol_m:
                self._stop()
                self.invest_phase = "wait"
                self.phase_t0 = now
                return
            # standoff is LATCHED once; recomputing it every tick from the live pose
            # makes the goal orbit the candidate and the robot thrash left-right
            if (
                self.invest_standoff is not None
                and math.hypot(ix - self._invest_standoff_for[0],
                               iy - self._invest_standoff_for[1]) > self.replan_goal_move_m
            ):
                self.invest_standoff = None      # candidate itself moved -> re-anchor
            if self.invest_standoff is None:
                self.invest_standoff = radial_standoff(
                    (rx, ry), (ix, iy), self.investigate_standoff_m)
                self._invest_standoff_for = (ix, iy)
                self.route = []
                self.route_goal = None
            if not self.route and not self._plan_route(self.invest_standoff, (ix, iy)):
                self.bans.append((ix, iy, now + self.investigate_fail_ban_sec))
                self.invest_xy = None
                self.invest_standoff = None
                self.invest_cooldown_until_s = now + self.investigate_cooldown_sec
                self._enter("EXPLORE")
                return
            if self._follow_route(self.explore_speed, self.wp_reach_tol_m):
                self._stop()
                self.invest_phase = "wait"
                self.phase_t0 = now
            return

        # wait: face the candidate, hold still, and let the cameras decide
        err = wrap_angle(math.atan2(iy - ry, ix - rx) - theta)
        if abs(err) > self.approach_heading_tol_rad:
            self.phase_t0 = now                  # the verdict clock starts once we face it
            self._turn_toward(err)
            return
        self._stop()
        if now - self.phase_t0 >= self.investigate_wait_sec:
            self.get_logger().info("INVESTIGATE inconclusive -> brief ban, back to explore")
            self.bans.append((ix, iy, now + self.investigate_fail_ban_sec))
            self.invest_xy = None
            self.invest_standoff = None
            self.invest_cooldown_until_s = now + self.investigate_cooldown_sec
            self.route = []
            self.route_goal = None
            self._enter("EXPLORE")

    def _step_approach(self) -> None:
        if self.target is None or self.target_xy is None:
            self._enter("EXPLORE")
            return
        now = self._now_s()
        # during drive-in the body cam sees the object directly; the map track often
        # drops out at close range and must not abort the grab
        if (
            self.approach_phase != "drive_in"
            and now - self.target_last_seen_s > self.target_lost_grace_sec
        ):
            self._abandon_target("track lost", self.fail_ban_sec)
            return
        if not self._pose_fresh():
            self._stop()
            return
        tx, ty = self.target_xy
        rx, ry, theta = self.pose

        if self.approach_phase == "travel":
            if self._stuck_check(commanding=True):
                return
            # natural align: the moment the body cam has the target, stop waypointing
            # and drive the grab point straight onto it
            if (
                self.drive_in_align_enabled
                and self._body_H is not None
                and math.hypot(tx - rx, ty - ry) <= self.drive_in_engage_dist_m
                and self._body_guidance_xy() is not None
            ):
                self.approach_phase = "drive_in"
                self.drive_in_seen_s = now
                return
            if self.standoff_xy is None:
                dist = math.hypot(tx - rx, ty - ry)
                if dist <= self.approach_dist_m + self.wp_reach_tol_m:
                    self.standoff_xy = (rx, ry)      # already near -> align from here
                else:
                    self.standoff_xy = radial_standoff((rx, ry), (tx, ty), self.approach_dist_m)
            if not self.route and not self._plan_route(self.standoff_xy, (tx, ty)):
                self._abandon_target("route blocked", self.fail_ban_sec)
                return
            if self._follow_route(self.hunt_speed, self.approach_standoff_tol_m):
                self.approach_phase = "settle"
                self.phase_t0 = now
            return

        if self.approach_phase == "drive_in":
            guide = self._body_guidance_xy()
            if guide is None:
                if now - self.drive_in_seen_s > self.drive_in_lost_sec:
                    self._stop()                 # lost it while closing -> settle + map heading
                    self.approach_phase = "settle"
                    self.phase_t0 = now
                else:
                    self._drive(self.drive_in_speed_min, 0.0, 0.0)  # brief dropout: creep
                return
            self.drive_in_seen_s = now
            bx, by = guide
            stop_x = self.grab_x + self.drive_in_stop_lead_m
            if bx <= stop_x or bx <= self.grab_min_x:
                self._stop()
                self._begin_align()              # verify; pulses only fix the residual
                return
            # speed tapers with the remaining distance so the final contact is gentle
            v = max(self.drive_in_speed_min,
                    min(self.drive_in_speed_max, self.drive_in_speed_kv * (bx - stop_x)))
            ey = by - self.grab_y
            err = math.atan2(ey, max(0.05, bx))
            omega = max(-self.drive_in_omega_max,
                        min(self.drive_in_omega_max, self.drive_in_kp * err))
            # correct lateral error WHILE driving (gentle mecanum diagonal) so we
            # arrive already centred and the pulse align has nothing left to do
            vy = 0.0
            if abs(ey) > 0.015:
                vy = max(-self.drive_in_vy_max,
                         min(self.drive_in_vy_max, self.drive_in_vy_kp * ey))
            self._drive(v, vy, omega)
            return

        if self.approach_phase == "settle":
            self._stop()
            if now - self.phase_t0 < self.approach_brake_settle_sec:
                return
            # body cam already sees the target -> skip the map-heading turn entirely
            body_fresh = now - self.body_stamp_s <= self.body_fresh_max_age_sec
            chosen, conflict = self._body_target() if body_fresh else (None, False)
            if chosen is not None or conflict:
                self._begin_align()
                return
            self.approach_phase = "heading"
            self.phase_t0 = now
            return

        # heading: face the frozen target position, then align
        err = wrap_angle(math.atan2(ty - ry, tx - rx) - theta)
        timed_out = now - self.phase_t0 > self.approach_heading_timeout_sec
        if abs(err) <= self.approach_heading_tol_rad or timed_out:
            self._stop()
            self._begin_align()
            return
        self._turn_toward(err)

    def _begin_align(self) -> None:
        if self._body_H is None:
            # no servo possible; better to skip than to blind-pick at the wrong spot
            self._abandon_target("no body homography", self.fail_ban_sec)
            return
        self.align_phase = "settle"
        self.phase_t0 = self._now_s()
        self.align_ok_frames = 0
        self.align_last_body_seq = self.body_seq
        self.align_body_seen_s = self._now_s()
        self.align_backoffs = 0
        self._enter("ALIGN")

    def _step_align(self) -> None:
        now = self._now_s()
        if self._time_in_state() > self.align_timeout_sec:
            self._abandon_target("align timeout", self.fail_ban_sec)
            return

        if self.align_phase == "pulse":
            vx, vy, until = self.align_pulse
            if now < until:
                self._drive(vx, vy, 0.0)
                return
            self.align_phase = "settle"
            self.phase_t0 = now
            return

        if self.align_phase == "backoff":
            if now - self.phase_t0 < self.align_backoff_sec:
                self._drive(-self.align_backoff_speed, 0.0, 0.0)
                return
            self.align_phase = "backoff_settle"
            self.phase_t0 = now
            return

        if self.align_phase == "backoff_settle":
            self._stop()
            if now - self.phase_t0 >= self.align_backoff_settle_sec:
                self.align_phase = "settle"
                self.phase_t0 = now
                self.align_last_body_seq = self.body_seq
                self.align_body_seen_s = now
            return

        if self.align_phase == "settle":
            self._stop()
            if now - self.phase_t0 >= self.align_settle_pulse_sec:
                self.align_phase = "measure"
                self.align_last_body_seq = self.body_seq
            return

        # measure: need a NEW body frame taken after the base stopped
        self._stop()
        if self.body_seq == self.align_last_body_seq:
            if now - self.align_body_seen_s > self.align_body_lost_sec:
                self._align_body_lost()
            return
        self.align_last_body_seq = self.body_seq
        chosen, conflict = self._body_target()
        if conflict:
            self._handle_fruit_conflict()
            return
        if chosen is None:
            if now - self.align_body_seen_s > self.align_body_lost_sec:
                self._align_body_lost()
            return
        self.align_body_seen_s = now
        _label, bx, by, _conf = chosen
        ex, ey = bx - self.grab_x, by - self.grab_y
        axis = align_translation_axis(ex, ey, self.align_fwd_tol_m, self.align_tol_m)
        if axis is None:
            self.align_ok_frames += 1
            if self.align_ok_frames >= self.align_confirm_frames:
                self._begin_pick()
            return
        self.align_ok_frames = 0
        name, err = axis
        if name == "x":
            if err > 0.0 and bx <= self.grab_min_x:
                return                             # never push the object closer than grab_min_x
            mid = abs(err) >= self.align_mid_error_m
            sec = self.align_step_fwd_mid_sec if mid else self.align_step_fwd_sec
            self.align_pulse = (math.copysign(self.align_step_fwd_duty, err), 0.0, now + sec)
        else:
            mid = abs(err) >= self.align_mid_error_m
            sec = self.align_step_strafe_mid_sec if mid else self.align_step_strafe_sec
            self.align_pulse = (0.0, math.copysign(self.align_step_strafe_duty, err), now + sec)
        self.align_phase = "pulse"

    def _handle_fruit_conflict(self) -> None:
        """The set1 "cube" target turned out to be a fruit cube at the grab point."""
        self.align_body_seen_s = self._now_s()   # we DO see the object; not a body-lost case
        # only reads captured at THIS object: received after ALIGN entry
        action = fruit_conflict_action(
            self._fresh_siglip(min_rx_s=self.state_enter_s), self.set2_label,
            self.strong_set2_min_margin_body,
            self.set2_total - self.tray_fruit,
        )
        self.fruit_conflict_wait = action == "wait"
        if action == "wait":
            return                               # hold still; align_timeout_sec bounds this
        if action == "switch":
            self.get_logger().info(
                f"fruit cube at grab point IS {self.set2_label} -> switching to set2 pick")
            self.target_set_type = 2
            self.align_ok_frames = 0
            return
        if action == "reject_quota":
            self._abandon_target("fruit cube, set2 quota full", 0.0, blacklist=True)
        else:
            self._abandon_target("fruit cube of a different fruit", 0.0, blacklist=True)

    def _align_body_lost(self) -> None:
        if self.align_backoffs < 1:
            self.align_backoffs += 1
            self.get_logger().info("ALIGN body lost -> single backoff")
            self.align_phase = "backoff"
            self.phase_t0 = self._now_s()
        else:
            self._abandon_target("body lost", self.fail_ban_sec)

    def _begin_pick(self) -> None:
        self._stop()
        self.pick_triggered = False
        self.pick_enter_s = self._now_s()
        self.get_logger().info(
            f"PICK id={int(self.target.id) if self.target else 0} set{self.target_set_type}")
        self._enter("PICK")

    def _arm_ready(self) -> bool:
        # stale phase feed -> assume ready (sequencer may be down; fixed wait still bounds us)
        if self._now_s() - self.arm_phase_rx_s > self.arm_phase_max_age_sec:
            return True
        return self.arm_phase in ("IDLE", "COMPLETE")

    def _step_pick(self) -> None:
        self._stop()
        now = self._now_s()
        if not self.pick_triggered:
            # previous pick's place cycle may still be running -> wait before re-triggering
            if not self._arm_ready() and now - self.pick_enter_s < 15.0:
                return
            self.pub_pick.publish(Bool(data=True))
            self.pick_triggered = True
            self.pick_enter_s = now
            return
        lifted = (
            self.arm_phase in ARM_LIFTED_PHASES
            and self.arm_phase_rx_s >= self.pick_enter_s
            and now - self.arm_phase_rx_s <= self.arm_phase_max_age_sec
        )
        if not lifted and now - self.pick_enter_s < self.pick_duration_sec:
            return
        # count it, suppress the stale track at the pick site, move on immediately
        if self.target_set_type == 2:
            self.tray_fruit += 1
        else:
            self.tray_shape += 1
        if self.target_xy is not None:
            expire = self._now_s() + self.picked_ban_sec if self.picked_ban_sec > 0.0 else 0.0
            self.bans.append((self.target_xy[0], self.target_xy[1], expire))
        if self.target is not None:
            self.pub_blacklist.publish(UInt64(data=int(self.target.id)))
        self.get_logger().info(f"PICKED -> tray shape={self.tray_shape} fruit={self.tray_fruit}")
        self.target = None
        self.target_xy = None
        self.standoff_xy = None
        self.route = []
        self._enter("EXPLORE")

    def _step_end(self) -> None:
        self._stop()

    # ------------------------------------------------------------------ status out
    def _publish_status(self) -> None:
        # keep the /mission_state vocabulary of the main stack so siglip/world_model/viz
        # state-dependent behaviour (transit classify etc.) keeps working
        state_alias = {
            "WAIT_START": "SCAN", "EXPLORE": "SCAN", "INVESTIGATE": "SCAN",
        }.get(self.state, self.state)
        if self.state == "ALIGN" and self.fruit_conflict_wait:
            # trigger siglip_gate's final-classify profile (3 Hz, grab-point-centred crop)
            state_alias = "CLASSIFY"
        m = MissionState()
        m.state = state_alias
        m.tray_shape_count = int(self.tray_shape)
        m.tray_fruit_count = int(self.tray_fruit)
        m.current_target_id = int(self.target.id) if self.target is not None else 0
        m.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(m)
        running = self.state not in ("WAIT_START", "END")
        self.pub_mapping.publish(Bool(data=running))
        self.pub_birth.publish(Bool(data=running))
        hunting_fruit = self.target_set_type == 2 and self.state in ("APPROACH", "ALIGN", "PICK")
        phase = 2 if hunting_fruit else 1
        self.pub_phase.publish(Int8(data=phase))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleHunterNode()
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
