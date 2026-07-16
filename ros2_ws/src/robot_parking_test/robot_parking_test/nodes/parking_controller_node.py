"""Independent arrival-based reverse parking controller.

Inputs:
  /camera_top/detections  robot_interfaces/DetectionArray, label == "arrival"
  /localization/pose      geometry_msgs/PoseStamped, field-frame robot pose
  /parking_test/start     std_msgs/Bool
  /parking_test/arm       std_msgs/Bool
  /parking_test/deadman   std_msgs/Bool, publish true at >=2 Hz while driving
  /parking_test/abort     std_msgs/Bool

Output:
  /base_command           robot_interfaces/BaseCommand

This node does not depend on mission_fsm, target_selector, explorer, go_to_goal, or arm nodes.
"""
from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_interfaces.msg import BaseCommand, DetectionArray, WorldModel
from std_msgs.msg import Bool, Float32MultiArray, String

from robot_planning.lane_planner import LanePlanner
from robot_parking_test.parking_fsm import (
    ArrivalParkingFsm,
    BaseCommandValue,
    ParkingConfig,
    target_from_marker_center,
)
from robot_parking_test.parking_geometry import (
    ArrivalObservation,
    BodyGroundProjector,
    BodyProjectionConfig,
    CameraProjectionConfig,
    PairScoreConfig,
    WideGroundProjector,
    score_floor_pairs,
    wrap_pi,
    yaw_from_quat,
)


class ParkingControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_controller_node")

        # Safety and operator gates.
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("armed", False)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("deadman_timeout_sec", 0.5)
        self.declare_parameter("check_base_command_publishers", True)
        self.declare_parameter("detection_timeout_sec", 0.6)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)

        # Competition-like route: match-style opening, then route to the arrival area.
        self.declare_parameter("route_to_zone4_enabled", True)
        self.declare_parameter("route_requires_world_model", True)
        self.declare_parameter("startup_warmup_sec", 15.0)
        self.declare_parameter("startup_perception_stable_sec", 0.0)
        self.declare_parameter("startup_requires_detections", False)
        self.declare_parameter("startup_requires_world_model", False)
        self.declare_parameter("world_model_timeout_sec", 1.0)
        self.declare_parameter("route_timeout_sec", 35.0)
        self.declare_parameter("opening_enabled", True)
        self.declare_parameter("opening_forward_sec", 1.5)
        self.declare_parameter("opening_forward_speed", 0.10)
        self.declare_parameter("opening_strafe_right_sec", 1.0)
        self.declare_parameter("opening_strafe_speed", 0.18)
        self.declare_parameter("opening_turn_deg", 45.0)
        self.declare_parameter("opening_turn_omega", -0.255)
        self.declare_parameter("opening_wait_after_turn_sec", 1.0)
        self.declare_parameter("zone4_staging_x", 1.35)
        self.declare_parameter("zone4_staging_y", 1.35)
        self.declare_parameter("zone4_staging_yaw_deg", -135.0)
        self.declare_parameter("route_goal_tol_m", 0.12)
        self.declare_parameter("route_goal_yaw_tol_deg", 12.0)
        self.declare_parameter("route_waypoint_tol_m", 0.12)
        self.declare_parameter("route_replan_sec", 1.0)
        self.declare_parameter("route_max_speed", 0.16)
        self.declare_parameter("route_min_speed", 0.04)
        self.declare_parameter("route_kp_lin", 0.8)
        self.declare_parameter("route_kp_ang", 1.2)
        self.declare_parameter("route_max_omega", 0.35)
        self.declare_parameter("route_face_tol_deg", 25.0)
        self.declare_parameter("route_yaw_deadband_deg", 6.0)
        self.declare_parameter("route_fine_radius_m", 0.18)
        self.declare_parameter("route_field_bounds_m", [-2.0, 2.0, -2.0, 2.0])
        self.declare_parameter("route_robot_margin_m", 0.24)
        self.declare_parameter("route_grid_spacing_m", 0.50)
        self.declare_parameter("route_obstacle_block_radius_m", 0.24)
        self.declare_parameter("route_obstacle_comfort_m", 0.35)
        self.declare_parameter("route_obstacle_min_conf", 0.35)
        self.declare_parameter("route_obstacle_min_nobs", 2)
        self.declare_parameter("route_exclude_goal_radius_m", 0.35)
        self.declare_parameter("prepark_search_omega", -0.08)
        self.declare_parameter("prepark_approach_max_speed", 0.07)

        # Arrival detection and pair scoring.
        self.declare_parameter("arrival_label", "arrival")
        self.declare_parameter("manual_floor_pair_indices", [-1, -1])
        self.declare_parameter("center_hint_x", 1.8)
        self.declare_parameter("center_hint_y", 1.8)
        self.declare_parameter("center_hint_scale_m", 0.65)
        self.declare_parameter("floor_pair_spacing_m", 0.36)
        self.declare_parameter("floor_pair_spacing_tolerance_m", 0.18)
        self.declare_parameter("floor_pair_min_spacing_m", 0.15)
        self.declare_parameter("floor_pair_max_spacing_m", 0.65)
        self.declare_parameter("min_pair_score", 0.35)
        self.declare_parameter("use_body_arrival_fallback", True)
        self.declare_parameter("parking_start_requires_wide", True)
        self.declare_parameter("parking_start_stable_sec", 1.0)
        self.declare_parameter("parking_start_min_pair_score", 0.45)
        self.declare_parameter("parking_start_min_bbox_height_px", 24.0)
        self.declare_parameter("parking_start_min_bbox_area_px", 700.0)
        self.declare_parameter("parking_start_max_reverse_start_dist_m", 0.45)

        # Parking geometry and control.
        self.declare_parameter("corner_direction_deg", 45.0)
        self.declare_parameter("target_yaw_deg", -135.0)
        self.declare_parameter("parking_depth_offset_m", 0.05)
        self.declare_parameter("latch_stable_sec", 1.0)
        self.declare_parameter("position_tol_m", 0.03)
        self.declare_parameter("yaw_tol_deg", 5.0)
        self.declare_parameter("total_timeout_sec", 6.0)
        self.declare_parameter("approach_timeout_sec", 6.0)
        self.declare_parameter("approach_standoff_m", 0.30)
        self.declare_parameter("approach_pos_tol_m", 0.05)
        self.declare_parameter("approach_max_speed", 0.14)
        self.declare_parameter("approach_min_speed", 0.035)
        self.declare_parameter("turn_timeout_sec", 2.0)
        self.declare_parameter("reverse_timeout_sec", 3.0)
        self.declare_parameter("max_reverse_speed", 0.10)
        self.declare_parameter("min_reverse_speed", 0.035)
        self.declare_parameter("lateral_error_threshold_m", 0.05)
        self.declare_parameter("max_blind_reverse_m", 0.05)

        # Wide camera projection, copied from the calibrated perception config by default.
        self.declare_parameter("top_fx", 669.125)
        self.declare_parameter("top_fy", 670.185)
        self.declare_parameter("top_cx", 639.976)
        self.declare_parameter("top_cy", 479.677)
        self.declare_parameter("fisheye_model", True)
        self.declare_parameter("dist_coeffs", [-0.085721, 0.053910, -0.034649, 0.007943])
        self.declare_parameter("image_rotated_180", True)
        self.declare_parameter("wide_homography_path", "/home/seventt/seventt/workspace/data/calib/wide_ground.npz")
        self.declare_parameter("cam_height_m", 0.885)
        self.declare_parameter("cam_pitch_deg", 88.0)
        self.declare_parameter("cam_offset_x", -0.15)
        self.declare_parameter("cam_offset_y", 0.0)
        self.declare_parameter("cam_yaw_deg", 90.0)
        self.declare_parameter("body_ground_homography_path", "/home/seventt/seventt/workspace/data/calib/body_ground.npz")
        self.declare_parameter("body_px_scale_x", 2.5625)
        self.declare_parameter("body_px_scale_y", 2.566667)
        self.declare_parameter("body_workspace_m", [0.20, 0.75, -0.45, 0.45])

        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.armed = bool(self.get_parameter("armed").value)
        self.require_deadman = bool(self.get_parameter("require_deadman").value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout_sec").value)
        self.check_publishers = bool(self.get_parameter("check_base_command_publishers").value)
        self.detection_timeout = float(self.get_parameter("detection_timeout_sec").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self.route_enabled = bool(self.get_parameter("route_to_zone4_enabled").value)
        self.route_requires_world = bool(self.get_parameter("route_requires_world_model").value)
        self.startup_warmup_sec = float(self.get_parameter("startup_warmup_sec").value)
        self.startup_perception_stable_sec = float(
            self.get_parameter("startup_perception_stable_sec").value
        )
        self.startup_requires_detections = bool(self.get_parameter("startup_requires_detections").value)
        self.startup_requires_world = bool(self.get_parameter("startup_requires_world_model").value)
        self.world_timeout = float(self.get_parameter("world_model_timeout_sec").value)
        self.route_timeout = float(self.get_parameter("route_timeout_sec").value)
        self.opening_enabled = bool(self.get_parameter("opening_enabled").value)
        self.opening_forward_sec = float(self.get_parameter("opening_forward_sec").value)
        self.opening_forward_speed = float(self.get_parameter("opening_forward_speed").value)
        self.opening_strafe_right_sec = float(self.get_parameter("opening_strafe_right_sec").value)
        self.opening_strafe_speed = float(self.get_parameter("opening_strafe_speed").value)
        self.opening_turn_deg = float(self.get_parameter("opening_turn_deg").value)
        self.opening_turn_omega = float(self.get_parameter("opening_turn_omega").value)
        self.opening_wait_after_turn_sec = float(self.get_parameter("opening_wait_after_turn_sec").value)
        self.zone4_staging = (
            float(self.get_parameter("zone4_staging_x").value),
            float(self.get_parameter("zone4_staging_y").value),
            math.radians(float(self.get_parameter("zone4_staging_yaw_deg").value)),
        )
        self.route_goal_tol = float(self.get_parameter("route_goal_tol_m").value)
        self.route_goal_yaw_tol = math.radians(float(self.get_parameter("route_goal_yaw_tol_deg").value))
        self.route_waypoint_tol = float(self.get_parameter("route_waypoint_tol_m").value)
        self.route_replan_sec = float(self.get_parameter("route_replan_sec").value)
        self.route_max_speed = float(self.get_parameter("route_max_speed").value)
        self.route_min_speed = float(self.get_parameter("route_min_speed").value)
        self.route_kp_lin = float(self.get_parameter("route_kp_lin").value)
        self.route_kp_ang = float(self.get_parameter("route_kp_ang").value)
        self.route_max_omega = float(self.get_parameter("route_max_omega").value)
        self.route_face_tol = math.radians(float(self.get_parameter("route_face_tol_deg").value))
        self.route_yaw_deadband = math.radians(float(self.get_parameter("route_yaw_deadband_deg").value))
        self.route_fine_radius = float(self.get_parameter("route_fine_radius_m").value)
        self.route_bounds = tuple(float(v) for v in self.get_parameter("route_field_bounds_m").value[:4])
        self.route_robot_margin = float(self.get_parameter("route_robot_margin_m").value)
        self.route_obstacle_min_conf = float(self.get_parameter("route_obstacle_min_conf").value)
        self.route_obstacle_min_nobs = int(self.get_parameter("route_obstacle_min_nobs").value)
        self.route_exclude_goal_radius = float(self.get_parameter("route_exclude_goal_radius_m").value)
        self.prepark_search_omega = float(self.get_parameter("prepark_search_omega").value)
        self.prepark_approach_max_speed = float(self.get_parameter("prepark_approach_max_speed").value)
        self.route_planner = LanePlanner(
            spacing=float(self.get_parameter("route_grid_spacing_m").value),
            bounds=self.route_bounds,
            margin=self.route_robot_margin,
            block_radius=float(self.get_parameter("route_obstacle_block_radius_m").value),
            comfort_clear=float(self.get_parameter("route_obstacle_comfort_m").value),
        )
        self.arrival_label = str(self.get_parameter("arrival_label").value)
        manual_pair_raw = [int(v) for v in self.get_parameter("manual_floor_pair_indices").value]
        self.manual_pair = tuple(v for v in manual_pair_raw if v >= 0)
        self.min_pair_score = float(self.get_parameter("min_pair_score").value)
        self.use_body_fallback = bool(self.get_parameter("use_body_arrival_fallback").value)
        self.parking_start_requires_wide = bool(self.get_parameter("parking_start_requires_wide").value)
        self.parking_start_stable_sec = float(self.get_parameter("parking_start_stable_sec").value)
        self.parking_start_min_pair_score = float(self.get_parameter("parking_start_min_pair_score").value)
        self.parking_start_min_bbox_height = float(
            self.get_parameter("parking_start_min_bbox_height_px").value
        )
        self.parking_start_min_bbox_area = float(self.get_parameter("parking_start_min_bbox_area_px").value)
        self.parking_start_max_reverse_start_dist = float(
            self.get_parameter("parking_start_max_reverse_start_dist_m").value
        )

        self.pair_cfg = PairScoreConfig(
            center_hint_x=float(self.get_parameter("center_hint_x").value),
            center_hint_y=float(self.get_parameter("center_hint_y").value),
            center_hint_scale_m=float(self.get_parameter("center_hint_scale_m").value),
            floor_pair_spacing_m=float(self.get_parameter("floor_pair_spacing_m").value),
            floor_pair_spacing_tolerance_m=float(self.get_parameter("floor_pair_spacing_tolerance_m").value),
            floor_pair_min_spacing_m=float(self.get_parameter("floor_pair_min_spacing_m").value),
            floor_pair_max_spacing_m=float(self.get_parameter("floor_pair_max_spacing_m").value),
        )
        self.parking_cfg = ParkingConfig(
            corner_direction_deg=float(self.get_parameter("corner_direction_deg").value),
            target_yaw_deg=float(self.get_parameter("target_yaw_deg").value),
            parking_depth_offset_m=float(self.get_parameter("parking_depth_offset_m").value),
            latch_stable_sec=float(self.get_parameter("latch_stable_sec").value),
            position_tol_m=float(self.get_parameter("position_tol_m").value),
            yaw_tol_rad=math.radians(float(self.get_parameter("yaw_tol_deg").value)),
            total_timeout_sec=float(self.get_parameter("total_timeout_sec").value),
            approach_timeout_sec=float(self.get_parameter("approach_timeout_sec").value),
            approach_standoff_m=float(self.get_parameter("approach_standoff_m").value),
            approach_pos_tol_m=float(self.get_parameter("approach_pos_tol_m").value),
            approach_max_speed=float(self.get_parameter("approach_max_speed").value),
            approach_min_speed=float(self.get_parameter("approach_min_speed").value),
            turn_timeout_sec=float(self.get_parameter("turn_timeout_sec").value),
            reverse_timeout_sec=float(self.get_parameter("reverse_timeout_sec").value),
            max_reverse_speed=float(self.get_parameter("max_reverse_speed").value),
            min_reverse_speed=float(self.get_parameter("min_reverse_speed").value),
            lateral_error_threshold_m=float(self.get_parameter("lateral_error_threshold_m").value),
            max_blind_reverse_m=float(self.get_parameter("max_blind_reverse_m").value),
        )
        cam_cfg = CameraProjectionConfig(
            top_fx=float(self.get_parameter("top_fx").value),
            top_fy=float(self.get_parameter("top_fy").value),
            top_cx=float(self.get_parameter("top_cx").value),
            top_cy=float(self.get_parameter("top_cy").value),
            fisheye_model=bool(self.get_parameter("fisheye_model").value),
            dist_coeffs=tuple(float(v) for v in self.get_parameter("dist_coeffs").value[:4]),
            image_rotated_180=bool(self.get_parameter("image_rotated_180").value),
            wide_homography_path=str(self.get_parameter("wide_homography_path").value),
            cam_height_m=float(self.get_parameter("cam_height_m").value),
            cam_pitch_deg=float(self.get_parameter("cam_pitch_deg").value),
            cam_offset_x=float(self.get_parameter("cam_offset_x").value),
            cam_offset_y=float(self.get_parameter("cam_offset_y").value),
            cam_yaw_deg=float(self.get_parameter("cam_yaw_deg").value),
        )
        self.projector = WideGroundProjector(cam_cfg)
        body_ws = [float(v) for v in self.get_parameter("body_workspace_m").value]
        if len(body_ws) != 4:
            body_ws = [0.20, 0.75, -0.45, 0.45]
        self.body_projector = BodyGroundProjector(
            BodyProjectionConfig(
                body_ground_homography_path=str(self.get_parameter("body_ground_homography_path").value),
                body_px_scale_x=float(self.get_parameter("body_px_scale_x").value),
                body_px_scale_y=float(self.get_parameter("body_px_scale_y").value),
                body_workspace_m=tuple(body_ws),
            )
        )
        self.fsm = ArrivalParkingFsm(self.parking_cfg)

        self.pose: tuple[float, float, float] | None = None
        self.pose_t: float | None = None
        self.world_pose: tuple[float, float, float] | None = None
        self.world_t: float | None = None
        self.world_objects = []
        self.pose_hist: deque[tuple[float, float, float, float]] = deque(maxlen=80)
        self.latest_target = None
        self.latest_pair = None
        self.latest_pair_source = ""
        self.latest_det_t: float | None = None
        self.last_wide_msg_t: float | None = None
        self.last_body_msg_t: float | None = None
        self.last_wide_det_count = 0
        self.last_body_det_count = 0
        self.route_active = False
        self.route_done = not self.route_enabled
        self.route_started_t: float | None = None
        self.route_phase = "IDLE"
        self.route_phase_t: float | None = None
        self.perception_ready_since: float | None = None
        self.route_last_plan_t: float | None = None
        self.route_path: list[tuple[float, float]] = []
        self.route_index = 0
        self.route_reason = "idle"
        self.opening_turn_start_theta: float | None = None
        self.parking_gate_since: float | None = None
        self.parking_gate_reason = "idle"
        self.projection_ok = self.projector.can_project
        self.body_projection_ok = self.body_projector.can_project
        self._last_state = ""
        self._last_zero_reason = ""
        self._deadman_t: float | None = None

        self.create_subscription(DetectionArray, "/camera_top/detections", self.on_detections, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_detections, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(Bool, "/parking_test/start", self.on_start, 10)
        self.create_subscription(Bool, "/parking_test/arm", self.on_arm, 10)
        self.create_subscription(Bool, "/parking_test/deadman", self.on_deadman, 10)
        self.create_subscription(Bool, "/parking_test/abort", self.on_abort, 10)
        self.create_subscription(Bool, "/parking_test/reset", self.on_reset, 10)

        self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)
        self.pub_state = self.create_publisher(String, "/parking_test/state", 10)
        self.pub_debug = self.create_publisher(Float32MultiArray, "/parking_test/debug", 10)
        rate = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate), self.tick)

        if bool(self.get_parameter("auto_start").value):
            self._request_start(self._now_s())

        self.get_logger().warn(
            "arrival parking test ready. Default is DISARMED/observe-only; no rear distance sensor is available."
        )
        self.get_logger().info(
            f"drive_enabled={self.drive_enabled} armed={self.armed} "
            f"deadman_required={self.require_deadman} wide_homography={self.projector.has_homography} "
            f"body_fallback={self.use_body_fallback} body_homography={self.body_projector.can_project} "
            f"route_to_zone4={self.route_enabled}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _request_start(self, now: float) -> None:
        if self.route_enabled:
            self.route_active = True
            self.route_done = False
            self.route_started_t = None
            self.route_phase = "WAIT_PERCEPTION"
            self.route_phase_t = now
            self.perception_ready_since = None
            self.route_last_plan_t = None
            self.route_path = []
            self.route_index = 0
            self.route_reason = "waiting for YOLO/world_model warmup"
            self.parking_gate_since = None
            self.parking_gate_reason = "waiting for route"
            self.fsm.reset(now)
            return
        self.fsm.start(now)

    def _current_pose(self, now: float) -> tuple[tuple[float, float, float] | None, bool]:
        if self.pose is not None and self.pose_t is not None and now - self.pose_t <= self.pose_timeout:
            return self.pose, True
        if self.world_pose is not None and self.world_t is not None and now - self.world_t <= self.world_timeout:
            return self.world_pose, True
        return None, False

    def _pose_at(self, stamp) -> tuple[float, float, float] | None:
        if self.pose is None:
            return None
        hist = list(self.pose_hist)
        if not hist:
            return self.pose
        t = self._stamp_to_sec(stamp)
        if t <= 0.0 or t >= hist[-1][0]:
            return self.pose
        if t <= hist[0][0]:
            return (hist[0][1], hist[0][2], hist[0][3])
        for i in range(len(hist) - 1):
            t0, x0, y0, th0 = hist[i]
            t1, x1, y1, th1 = hist[i + 1]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                dth = wrap_pi(th1 - th0)
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), th0 + a * dth)
        return self.pose

    def on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.pose = (float(msg.pose.position.x), float(msg.pose.position.y), yaw)
        t = self._stamp_to_sec(msg.header.stamp)
        now = self._now_s()
        self.pose_t = now
        self.pose_hist.append((t if t > 0.0 else now, self.pose[0], self.pose[1], self.pose[2]))

    def on_world(self, msg: WorldModel) -> None:
        self.world_pose = (float(msg.robot_x), float(msg.robot_y), float(msg.robot_theta))
        self.world_t = self._now_s()
        self.world_objects = list(msg.objects)

    def on_start(self, msg: Bool) -> None:
        if msg.data:
            self._request_start(self._now_s())
        else:
            self.fsm.started = False
            self.route_active = False
            self.route_phase = "IDLE"

    def on_arm(self, msg: Bool) -> None:
        self.armed = bool(msg.data)

    def on_deadman(self, msg: Bool) -> None:
        if msg.data:
            self._deadman_t = self._now_s()

    def on_abort(self, msg: Bool) -> None:
        if msg.data:
            self.route_active = False
            self.fsm.abort(self._now_s())
            self._publish_cmd(BaseCommandValue())

    def on_reset(self, msg: Bool) -> None:
        if msg.data:
            self.fsm.reset(self._now_s())
            self.latest_target = None
            self.latest_pair = None
            self.route_active = False
            self.route_done = not self.route_enabled
            self.route_phase = "IDLE"
            self.route_phase_t = None
            self.perception_ready_since = None
            self.route_path = []
            self.route_index = 0
            self.route_last_plan_t = None
            self.route_reason = "reset"
            self.parking_gate_since = None
            self.parking_gate_reason = "reset"
            self._publish_cmd(BaseCommandValue())

    def on_detections(self, msg: DetectionArray) -> None:
        self.last_wide_msg_t = self._now_s()
        self.last_wide_det_count = len(msg.detections)
        pose = self._pose_at(msg.header.stamp)
        if pose is None:
            return
        observations: list[ArrivalObservation] = []
        arrival_idx = 0
        for det in msg.detections:
            if str(det.label) != self.arrival_label:
                continue
            projected = self.projector.pixel_to_field(float(det.x_center), float(det.y_center), pose)
            if projected is None:
                self.projection_ok = False
                continue
            self.projection_ok = True
            fx, fy, bx, by = projected
            observations.append(
                ArrivalObservation(
                    index=arrival_idx,
                    u=float(det.x_center),
                    v=float(det.y_center),
                    confidence=float(det.confidence),
                    field_x=fx,
                    field_y=fy,
                    width=float(det.width),
                    height=float(det.height),
                    base_x=bx,
                    base_y=by,
                )
            )
            arrival_idx += 1
        self._accept_pair(self._select_pair(observations), "wide")

    def on_body_detections(self, msg: DetectionArray) -> None:
        self.last_body_msg_t = self._now_s()
        self.last_body_det_count = len(msg.detections)
        if not self.use_body_fallback:
            return
        pose = self._pose_at(msg.header.stamp)
        if pose is None:
            return
        observations: list[ArrivalObservation] = []
        arrival_idx = 0
        for det in msg.detections:
            if str(det.label) != self.arrival_label:
                continue
            projected = self.body_projector.pixel_to_field(float(det.x_center), float(det.y_center), pose)
            if projected is None:
                self.body_projection_ok = False
                continue
            self.body_projection_ok = True
            fx, fy, bx, by = projected
            observations.append(
                ArrivalObservation(
                    index=arrival_idx,
                    u=float(det.x_center),
                    v=float(det.y_center),
                    confidence=float(det.confidence),
                    field_x=fx,
                    field_y=fy,
                    width=float(det.width),
                    height=float(det.height),
                    base_x=bx,
                    base_y=by,
                )
            )
            arrival_idx += 1
        # Prefer a fresh wide target when it exists; otherwise body close-range fallback is allowed.
        if self.latest_pair_source == "wide" and self.latest_det_t is not None:
            if self._now_s() - self.latest_det_t <= self.detection_timeout:
                return
        self._accept_pair(self._select_pair(observations), "body")

    def _accept_pair(self, pair, source: str) -> None:
        if pair is None:
            return
        self.latest_pair = pair
        self.latest_pair_source = source
        self.latest_det_t = self._now_s()
        self.latest_target = target_from_marker_center(
            pair.center_x,
            pair.center_y,
            pair.key,
            pair.score,
            self.parking_cfg,
        )

    def _select_pair(self, observations: list[ArrivalObservation]):
        if len(observations) < 2:
            return None
        if len(self.manual_pair) == 2:
            by_idx = {o.index: o for o in observations}
            a = by_idx.get(self.manual_pair[0])
            b = by_idx.get(self.manual_pair[1])
            if a is None or b is None:
                return None
            pairs = score_floor_pairs([a, b], self.pair_cfg)
            return pairs[0] if pairs else None
        pairs = score_floor_pairs(observations, self.pair_cfg)
        if not pairs or pairs[0].score < self.min_pair_score:
            return None
        return pairs[0]

    def _world_fresh(self, now: float) -> bool:
        return self.world_t is not None and now - self.world_t <= self.world_timeout

    def _detection_streams_fresh(self, now: float) -> bool:
        return (
            self.last_wide_msg_t is not None
            and now - self.last_wide_msg_t <= self.world_timeout
            and self.last_body_msg_t is not None
            and now - self.last_body_msg_t <= self.world_timeout
        )

    def _startup_perception_ready(self, now: float) -> bool:
        if self.route_phase_t is None:
            return False
        warm = now - self.route_phase_t >= self.startup_warmup_sec
        detections_ready = (not self.startup_requires_detections) or self._detection_streams_fresh(now)
        world_ready = (not self.startup_requires_world) or self._world_fresh(now)
        ready = warm and world_ready and detections_ready
        if not ready:
            self.perception_ready_since = None
            return False
        if self.perception_ready_since is None:
            self.perception_ready_since = now
            return False
        return now - self.perception_ready_since >= self.startup_perception_stable_sec

    def _enter_route_phase(self, phase: str, now: float, reason: str) -> None:
        self.route_phase = phase
        self.route_phase_t = now
        self.route_reason = reason
        if phase != "PREPARK_SEARCH":
            self.parking_gate_since = None
        if phase == "NAV_TO_ZONE4":
            self.route_started_t = now
            self.route_last_plan_t = None
            self.route_path = []
            self.route_index = 0

    def _latest_target_fresh(self, now: float) -> bool:
        return (
            self.latest_target is not None
            and self.latest_pair is not None
            and self.latest_det_t is not None
            and now - self.latest_det_t <= self.detection_timeout
        )

    def _reverse_start_for_target(self, target) -> tuple[float, float, float]:
        fx = math.cos(target.yaw)
        fy = math.sin(target.yaw)
        return (
            target.goal_x + fx * self.parking_cfg.approach_standoff_m,
            target.goal_y + fy * self.parking_cfg.approach_standoff_m,
            target.yaw,
        )

    def _parking_gate_ready(self, now: float, pose: tuple[float, float, float]) -> bool:
        if not self._latest_target_fresh(now):
            self.parking_gate_since = None
            self.parking_gate_reason = "waiting for fresh arrival pair"
            return False
        if self.parking_start_requires_wide and self.latest_pair_source != "wide":
            self.parking_gate_since = None
            self.parking_gate_reason = f"waiting for wide floor pair, source={self.latest_pair_source}"
            return False

        pair = self.latest_pair
        if pair.score < self.parking_start_min_pair_score:
            self.parking_gate_since = None
            self.parking_gate_reason = (
                f"pair score low {pair.score:.2f}<{self.parking_start_min_pair_score:.2f}"
            )
            return False
        if (
            self.parking_start_min_bbox_height > 0.0
            and pair.min_height_px < self.parking_start_min_bbox_height
        ):
            self.parking_gate_since = None
            self.parking_gate_reason = (
                f"wide arrival too small h={pair.min_height_px:.0f}px"
            )
            return False
        if (
            self.parking_start_min_bbox_area > 0.0
            and pair.mean_area_px < self.parking_start_min_bbox_area
        ):
            self.parking_gate_since = None
            self.parking_gate_reason = (
                f"wide arrival too small area={pair.mean_area_px:.0f}px"
            )
            return False

        sx, sy, _ = self._reverse_start_for_target(self.latest_target)
        dist = math.hypot(sx - pose[0], sy - pose[1])
        if dist > self.parking_start_max_reverse_start_dist:
            self.parking_gate_since = None
            self.parking_gate_reason = (
                f"approaching reverse_start dist={dist:.2f}m"
            )
            return False

        if self.parking_gate_since is None:
            self.parking_gate_since = now
            self.parking_gate_reason = "parking gate stabilizing"
            return False
        age = now - self.parking_gate_since
        if age < self.parking_start_stable_sec:
            self.parking_gate_reason = f"parking gate stabilizing {age:.1f}s"
            return False
        self.parking_gate_reason = (
            f"parking gate ready score={pair.score:.2f} h={pair.min_height_px:.0f}px "
            f"area={pair.mean_area_px:.0f}px"
        )
        return True

    @staticmethod
    def _cap_translation(cmd: BaseCommandValue, max_speed: float) -> BaseCommandValue:
        speed = math.hypot(cmd.vx, cmd.vy)
        if speed <= max_speed or speed <= 1e-6:
            return cmd
        scale = max_speed / speed
        return BaseCommandValue(cmd.vx * scale, cmd.vy * scale, cmd.omega)

    def _tick_pre_route(
        self,
        now: float,
        pose: tuple[float, float, float],
    ) -> BaseCommandValue | None:
        if self.route_phase == "WAIT_PERCEPTION":
            wide_age = None if self.last_wide_msg_t is None else now - self.last_wide_msg_t
            body_age = None if self.last_body_msg_t is None else now - self.last_body_msg_t
            det_mode = "detections_required" if self.startup_requires_detections else "detections_optional"
            world_mode = "world_required" if self.startup_requires_world else "world_optional"
            self.route_reason = (
                "waiting for YOLO/world_model warmup "
                f"{det_mode} {world_mode} wide_age={_fmt_age(wide_age)} body_age={_fmt_age(body_age)}"
            )
            if self._startup_perception_ready(now):
                if self.opening_enabled:
                    self._enter_route_phase("OPENING_FORWARD", now, "opening forward")
                else:
                    self._enter_route_phase("NAV_TO_ZONE4", now, "opening disabled")
            return BaseCommandValue()

        if self.route_phase == "OPENING_FORWARD":
            if now - self.route_phase_t < self.opening_forward_sec:
                return BaseCommandValue(self.opening_forward_speed, 0.0, 0.0)
            self._enter_route_phase("OPENING_STRAFE_RIGHT", now, "opening strafe right")
            return BaseCommandValue()

        if self.route_phase == "OPENING_STRAFE_RIGHT":
            if now - self.route_phase_t < self.opening_strafe_right_sec:
                return BaseCommandValue(0.0, -self.opening_strafe_speed, 0.0)
            self.opening_turn_start_theta = pose[2]
            self._enter_route_phase("OPENING_TURN", now, "opening turn")
            return BaseCommandValue()

        if self.route_phase == "OPENING_TURN":
            target = abs(math.radians(self.opening_turn_deg))
            timed_sec = target / abs(self.opening_turn_omega) if abs(self.opening_turn_omega) > 1e-6 else 0.0
            timeout_sec = max(timed_sec + 0.5, timed_sec * 1.5)
            done_by_heading = False
            if self.opening_turn_start_theta is not None:
                direction = 1.0 if self.opening_turn_omega >= 0.0 else -1.0
                delta = wrap_pi(pose[2] - self.opening_turn_start_theta)
                done_by_heading = direction * delta >= target
            if not done_by_heading and now - self.route_phase_t < timeout_sec:
                return BaseCommandValue(0.0, 0.0, self.opening_turn_omega)
            self._enter_route_phase("OPENING_SETTLE", now, "opening settle")
            return BaseCommandValue()

        if self.route_phase == "OPENING_SETTLE":
            if now - self.route_phase_t < self.opening_wait_after_turn_sec:
                return BaseCommandValue()
            self._enter_route_phase("NAV_TO_ZONE4", now, "opening done; route to zone4")
            return BaseCommandValue()

        if self.route_phase == "PREPARK_SEARCH":
            if self.route_started_t is not None and now - self.route_started_t > self.route_timeout:
                self.route_active = False
                self.fsm.fault(now, "zone4 pre-parking search timeout")
                return BaseCommandValue()
            if self.route_requires_world and not self._world_fresh(now):
                self.parking_gate_since = None
                self.route_reason = "prepark waiting for world_model objects"
                return BaseCommandValue()
            if self._parking_gate_ready(now, pose):
                self.route_active = False
                self.route_done = True
                self.route_reason = self.parking_gate_reason
                self.fsm.start(now)
                return BaseCommandValue()
            if self.parking_gate_since is not None:
                self.route_reason = self.parking_gate_reason
                return BaseCommandValue()
            usable_prepark_target = (
                self._latest_target_fresh(now)
                and self.latest_target is not None
                and self.latest_pair is not None
                and (not self.parking_start_requires_wide or self.latest_pair_source == "wide")
                and self.latest_pair.score >= self.parking_start_min_pair_score
            )
            if usable_prepark_target:
                sx, sy, syaw = self._reverse_start_for_target(self.latest_target)
                dist = math.hypot(sx - pose[0], sy - pose[1])
                if dist > self.parking_start_max_reverse_start_dist:
                    self.route_reason = self.parking_gate_reason
                    cmd = self._route_drive_command(pose, sx, sy, syaw, True)
                    return self._cap_translation(cmd, self.prepark_approach_max_speed)
            self.route_reason = f"prepark scan; {self.parking_gate_reason}"
            return BaseCommandValue(0.0, 0.0, self.prepark_search_omega)

        return None

    def _route_obstacles(self, dest: tuple[float, float]) -> list[tuple[float, float]]:
        out = []
        for obj in self.world_objects:
            label = str(obj.class_label)
            if label == self.arrival_label or int(obj.set_type) == 3:
                continue
            if float(obj.confidence) < self.route_obstacle_min_conf:
                continue
            if int(obj.n_obs) < self.route_obstacle_min_nobs:
                continue
            ox, oy = float(obj.x), float(obj.y)
            if math.hypot(ox - dest[0], oy - dest[1]) < self.route_exclude_goal_radius:
                continue
            out.append((ox, oy))
        return out

    def _plan_route(self, pose: tuple[float, float, float], now: float) -> None:
        dest = (self.zone4_staging[0], self.zone4_staging[1])
        obstacles = self._route_obstacles(dest)
        path = self.route_planner.plan((pose[0], pose[1]), dest, obstacles)
        self.route_path = path if path else [dest]
        self.route_index = 0
        self.route_last_plan_t = now
        self.route_reason = f"planned {len(self.route_path)} waypoint(s), obstacles={len(obstacles)}"

    def _tick_route_to_zone4(self, now: float, pose: tuple[float, float, float]) -> BaseCommandValue:
        if self.route_started_t is not None and now - self.route_started_t > self.route_timeout:
            self.route_active = False
            self.fsm.fault(now, "zone1 to zone4 route timeout")
            return BaseCommandValue()

        if self._parking_gate_ready(now, pose):
            self.route_active = False
            self.route_done = True
            self.route_reason = self.parking_gate_reason
            self.fsm.start(now)
            return BaseCommandValue()

        gx, gy, gyaw = self.zone4_staging
        final_dist = math.hypot(gx - pose[0], gy - pose[1])
        final_yaw_err = wrap_pi(gyaw - pose[2])
        if final_dist <= self.route_goal_tol and abs(final_yaw_err) <= self.route_goal_yaw_tol:
            self._enter_route_phase(
                "PREPARK_SEARCH",
                now,
                "zone4 staging reached; waiting for visible wide arrival",
            )
            return BaseCommandValue()

        if (
            not self.route_path
            or self.route_last_plan_t is None
            or now - self.route_last_plan_t >= self.route_replan_sec
        ):
            self._plan_route(pose, now)

        while self.route_index < len(self.route_path) - 1:
            wx, wy = self.route_path[self.route_index]
            if math.hypot(wx - pose[0], wy - pose[1]) > self.route_waypoint_tol:
                break
            self.route_index += 1

        wx, wy = self.route_path[min(self.route_index, len(self.route_path) - 1)]
        is_final = self.route_index >= len(self.route_path) - 1
        yaw = gyaw if is_final else math.atan2(wy - pose[1], wx - pose[0])
        return self._route_drive_command(pose, wx, wy, yaw, is_final)

    def _route_drive_command(
        self,
        pose: tuple[float, float, float],
        gx: float,
        gy: float,
        gyaw: float,
        is_final: bool,
    ) -> BaseCommandValue:
        rx, ry, rtheta = pose
        ex, ey = gx - rx, gy - ry
        dist = math.hypot(ex, ey)
        yaw_err = wrap_pi(gyaw - rtheta)
        c, s = math.cos(rtheta), math.sin(rtheta)

        if dist > self.route_fine_radius:
            bearing = math.atan2(ey, ex)
            head_err = wrap_pi(bearing - rtheta)
            omega = 0.0 if abs(head_err) < self.route_yaw_deadband else _clamp(
                self.route_kp_ang * head_err,
                -self.route_max_omega,
                self.route_max_omega,
            )
            if abs(head_err) < self.route_face_tol:
                vx = _clamp(self.route_kp_lin * dist, self.route_min_speed, self.route_max_speed)
                vx *= max(0.0, math.cos(head_err))
                vy = 0.0
            else:
                vx = 0.0
                vy = 0.0
            return BaseCommandValue(vx, vy, omega)

        bx = c * ex + s * ey
        by = -s * ex + c * ey
        speed = min(self.route_max_speed, max(self.route_min_speed, self.route_kp_lin * dist))
        if dist > 1e-6:
            vx = speed * bx / dist
            vy = speed * by / dist
        else:
            vx = 0.0
            vy = 0.0
        omega = _clamp(
            self.route_kp_ang * yaw_err if is_final else 0.0,
            -self.route_max_omega,
            self.route_max_omega,
        )
        return BaseCommandValue(vx, vy, omega)

    def _route_wall_guard(
        self,
        cmd: BaseCommandValue,
        pose: tuple[float, float, float],
    ) -> BaseCommandValue:
        if abs(cmd.vx) < 1e-6 and abs(cmd.vy) < 1e-6:
            return cmd
        xmin, xmax, ymin, ymax = self.route_bounds
        margin = self.route_robot_margin
        th = pose[2]
        field_vx = cmd.vx * math.cos(th) - cmd.vy * math.sin(th)
        field_vy = cmd.vx * math.sin(th) + cmd.vy * math.cos(th)
        outward = (
            (pose[0] <= xmin + margin and field_vx < 0.0)
            or (pose[0] >= xmax - margin and field_vx > 0.0)
            or (pose[1] <= ymin + margin and field_vy < 0.0)
            or (pose[1] >= ymax - margin and field_vy > 0.0)
        )
        if outward:
            self._last_zero_reason = "wall_guard"
            return BaseCommandValue(0.0, 0.0, cmd.omega)
        return cmd

    def tick(self) -> None:
        now = self._now_s()
        pose, pose_fresh = self._current_pose(now)
        det_fresh = self.latest_det_t is not None and now - self.latest_det_t <= self.detection_timeout
        if self.route_active and self.fsm.state not in ("FAULT", "ABORT"):
            if not pose_fresh or pose is None:
                self.route_reason = "waiting for localization pose"
                self._publish_cmd(BaseCommandValue())
                self._publish_state(now, BaseCommandValue(), pose_fresh, det_fresh)
                return
            pre_cmd = self._tick_pre_route(now, pose)
            if pre_cmd is not None:
                safe_cmd = self._gate_command(self._route_wall_guard(pre_cmd, pose), now)
                self._publish_cmd(safe_cmd)
                self._publish_state(now, safe_cmd, pose_fresh, det_fresh)
                return
            if self.route_requires_world and not self._world_fresh(now):
                self.route_reason = "waiting for world_model objects"
                self._publish_cmd(BaseCommandValue())
                self._publish_state(now, BaseCommandValue(), pose_fresh, det_fresh)
                return
            cmd = self._tick_route_to_zone4(now, pose)
            safe_cmd = self._gate_command(self._route_wall_guard(cmd, pose), now)
            self._publish_cmd(safe_cmd)
            self._publish_state(now, safe_cmd, pose_fresh, det_fresh)
            return

        moving_state = self.fsm.state in (
            "APPROACH_REVERSE_START",
            "TURN_FOR_REVERSE",
            "STRAIGHT_REVERSE",
            "SETTLE",
        )
        if (self.fsm.started and not self.projection_ok and not self.body_projection_ok
                and self.fsm.latched_target is None):
            self.fsm.fault(now, "arrival projection failed")
        if self.fsm.started and not pose_fresh and moving_state:
            self.fsm.fault(now, "pose stale")
        elif self.fsm.started and not pose_fresh:
            self._publish_cmd(BaseCommandValue())
            self._publish_state(now, BaseCommandValue(), pose_fresh, det_fresh)
            return

        cmd = self.fsm.tick(now, pose if pose_fresh else None, self.latest_target, det_fresh)
        safe_cmd = self._gate_command(cmd, now)
        self._publish_cmd(safe_cmd)
        self._publish_state(now, safe_cmd, pose_fresh, det_fresh)

    def _gate_command(self, cmd: BaseCommandValue, now: float) -> BaseCommandValue:
        if self.fsm.state == "STRAIGHT_REVERSE" and cmd.vx > 0.0:
            self.fsm.fault(now, "forward vx blocked")
            return BaseCommandValue()
        active = abs(cmd.vx) > 1e-6 or abs(cmd.vy) > 1e-6 or abs(cmd.omega) > 1e-6
        if not active:
            return cmd
        if not self.drive_enabled:
            self._last_zero_reason = "drive_disabled"
            return BaseCommandValue()
        if not self.armed:
            self._last_zero_reason = "disarmed"
            return BaseCommandValue()
        if self.require_deadman and self._deadman_t is None:
            self._last_zero_reason = "waiting_deadman"
            return BaseCommandValue()
        if self.require_deadman and now - self._deadman_t > self.deadman_timeout:
            self.fsm.fault(now, "deadman lost")
            self._last_zero_reason = "deadman_lost"
            return BaseCommandValue()
        if self.check_publishers and self._other_base_command_publishers():
            self.fsm.fault(now, "another /base_command publisher is active")
            self._last_zero_reason = "publisher_conflict"
            return BaseCommandValue()
        self._last_zero_reason = ""
        return cmd

    def _other_base_command_publishers(self) -> bool:
        infos = self.get_publishers_info_by_topic("/base_command")
        for info in infos:
            if info.node_name != self.get_name():
                return True
        return False

    def _publish_cmd(self, cmd: BaseCommandValue) -> None:
        out = BaseCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.vx = float(cmd.vx)
        out.vy = float(cmd.vy)
        out.omega = float(cmd.omega)
        self.pub_cmd.publish(out)

    def _publish_state(
        self,
        now: float,
        cmd: BaseCommandValue,
        pose_fresh: bool,
        det_fresh: bool,
    ) -> None:
        target = self.fsm.latched_target or self.latest_target
        display_state = self.fsm.state
        display_reason = self.fsm.reason
        if self.route_active and self.fsm.state not in ("FAULT", "ABORT"):
            display_state = self.route_phase
            display_reason = self.route_reason
        if display_state != self._last_state:
            self.get_logger().info(f"parking state -> {display_state}: {display_reason}")
            self._last_state = display_state
        msg = String()
        msg.data = (
            f"state={display_state} reason={display_reason} "
            f"drive_enabled={self.drive_enabled} armed={self.armed} "
            f"pose_fresh={pose_fresh} det_fresh={det_fresh} "
            f"world_fresh={self._world_fresh(now)} route_done={self.route_done} "
            f"source={self.latest_pair_source or 'none'} target={_fmt_target(target)} "
            f"zero_reason={self._last_zero_reason}"
        )
        self.pub_state.publish(msg)
        dbg = Float32MultiArray()
        pair = self.latest_pair
        pose = self.pose or self.world_pose
        yaw_err = 0.0
        pos_err = 0.0
        if self.route_active and pose is not None:
            if self.route_phase == "PREPARK_SEARCH" and target is not None:
                sx, sy, syaw = self._reverse_start_for_target(target)
                pos_err = math.hypot(sx - pose[0], sy - pose[1])
                yaw_err = wrap_pi(syaw - pose[2])
            else:
                pos_err = math.hypot(self.zone4_staging[0] - pose[0], self.zone4_staging[1] - pose[1])
                yaw_err = wrap_pi(self.zone4_staging[2] - pose[2])
        elif target is not None and pose is not None:
            pos_err = math.hypot(target.goal_x - pose[0], target.goal_y - pose[1])
            yaw_err = wrap_pi(target.yaw - pose[2])
        dbg.data = [
            float(now),
            _state_code(display_state),
            float(cmd.vx),
            float(cmd.vy),
            float(cmd.omega),
            float(pos_err),
            float(yaw_err),
            float(pair.center_x if pair is not None else 0.0),
            float(pair.center_y if pair is not None else 0.0),
            float(pair.spacing_m if pair is not None else 0.0),
            float(pair.score if pair is not None else 0.0),
            1.0 if pose_fresh else 0.0,
            1.0 if det_fresh else 0.0,
        ]
        self.pub_debug.publish(dbg)


def _fmt_target(target) -> str:
    if target is None:
        return "none"
    return (
        f"center=({target.marker_center_x:.3f},{target.marker_center_y:.3f}) "
        f"goal=({target.goal_x:.3f},{target.goal_y:.3f}) "
        f"yaw={math.degrees(target.yaw):.1f}deg score={target.score:.3f}"
    )


def _state_code(state: str) -> float:
    states = [
        "IDLE",
        "WAIT_PERCEPTION",
        "OPENING_FORWARD",
        "OPENING_STRAFE_RIGHT",
        "OPENING_TURN",
        "OPENING_SETTLE",
        "NAV_TO_ZONE4",
        "PREPARK_SEARCH",
        "ACQUIRE",
        "READY",
        "APPROACH_REVERSE_START",
        "TURN_FOR_REVERSE",
        "STRAIGHT_REVERSE",
        "SETTLE",
        "SUCCESS",
        "FAULT",
        "ABORT",
    ]
    try:
        return float(states.index(state))
    except ValueError:
        return -1.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def _fmt_age(value: float | None) -> str:
    return "none" if value is None else f"{value:.1f}s"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(BaseCommandValue())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
