"""Safe ROS wrapper for the isolated local-anchor fruit-inspection FSM."""
from __future__ import annotations

import json
import math
import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_interfaces.msg import (
    BaseCommand,
    Classification,
    DetectionArray,
    MissionState,
    WorldModel,
)
from robot_planning.local_anchor_fsm import (
    LocalAnchorConfig,
    LocalAnchorFruitFsm,
    LocalObservation,
    MotionCommand,
    TERMINAL_STATES,
    integrate_imu_yaw,
    wrap_angle,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32, String


def quaternion_yaw(msg: PoseStamped) -> float:
    """Extract planar yaw from a PoseStamped quaternion."""
    q = msg.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LocalAnchorTestNode(Node):
    """Connect perception and base topics without touching the competition mission FSM."""

    def __init__(self) -> None:
        super().__init__("local_anchor_test_node")
        self._declare_parameters()
        self.config = self._read_config()
        self.fsm = LocalAnchorFruitFsm(self.config)

        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.armed = bool(self.get_parameter("armed").value)
        self.require_deadman = bool(self.get_parameter("require_deadman").value)
        self.deadman_timeout_sec = float(self.get_parameter("deadman_timeout_sec").value)
        self.require_running = bool(self.get_parameter("require_competition_running").value)
        self.check_publishers = bool(self.get_parameter("check_base_command_publishers").value)
        self.placement_seed = int(self.get_parameter("placement_seed").value)
        self.relative_topic_timeout_sec = float(
            self.get_parameter("relative_topic_timeout_sec").value
        )
        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)
        self.imu_timeout_sec = float(self.get_parameter("imu_timeout_sec").value)
        self.candidate_labels = {
            str(label).strip() for label in self.get_parameter("candidate_labels").value
        }
        self.body_labels = {
            str(label).strip() for label in self.get_parameter("body_align_labels").value
        }

        self.current_yaw: float | None = None
        self.anchor_yaw: float | None = None
        self.imu_relative_yaw = 0.0
        self.last_imu_s = -math.inf
        self.last_relative_s = -math.inf
        self.last_deadman_s = -math.inf
        self.competition_state = "UNKNOWN"
        self._last_status_s = -math.inf
        self._last_status_state = ""
        self._active = False
        self._pending_start = False
        self._last_start_attempt_s = -math.inf
        self._stop_publish_until_s = -math.inf
        self._load_body_projection()

        self.pub_cmd = self.create_publisher(BaseCommand, "/base_command", 10)
        self.pub_state = self.create_publisher(MissionState, "/mission_state", 10)
        self.pub_status = self.create_publisher(String, "/local_anchor_test/status", 10)
        self.pub_pick = self.create_publisher(Bool, "/arm/pick_trigger", 10)
        self.create_subscription(
            WorldModel,
            "/world_model/wide_relative_objects",
            self._on_relative_objects,
            10,
        )
        self.create_subscription(PoseStamped, "/localization/pose", self._on_pose, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 20)
        self.create_subscription(
            Float32,
            "/localization/imu_yaw_delta",
            self._on_imu_yaw_delta,
            20,
        )
        self.create_subscription(
            Classification, "/classification/siglip", self._on_classification, 10
        )
        self.create_subscription(
            DetectionArray, "/camera_body/detections", self._on_body_detections, 10
        )
        self.create_subscription(String, "/competition/state", self._on_competition, 10)
        self.create_subscription(String, "/local_anchor_test/control", self._on_control, 10)
        self.create_subscription(Empty, "/local_anchor_test/deadman", self._on_deadman, 10)
        rate = max(2.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().warn(
            "LOCAL-ANCHOR ISOLATED TEST: mission_fsm must be OFF. "
            f"drive_enabled={self.drive_enabled} armed={self.armed} "
            f"deadman={self.require_deadman} align={self.config.enable_align}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("armed", False)
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("deadman_timeout_sec", 0.75)
        self.declare_parameter("require_competition_running", True)
        self.declare_parameter("check_base_command_publishers", True)
        self.declare_parameter("relative_topic_timeout_sec", 1.0)
        self.declare_parameter("use_imu_yaw", True)
        self.declare_parameter("imu_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("placement_seed", 1)
        self.declare_parameter(
            "candidate_labels",
            [
                "cube",
                "octahedron",
                "dodecahedron",
                "icosahedron",
                "fruit_photo_cube",
                "apple",
                "orange",
                "banana",
                "pineapple",
            ],
        )
        self.declare_parameter(
            "fruit_inspection_labels",
            ["fruit_photo_cube", "apple", "orange", "banana", "pineapple"],
        )
        self.declare_parameter(
            "body_align_labels",
            ["fruit_photo_cube", "cube", "apple", "orange", "banana", "pineapple"],
        )
        self.declare_parameter("scan_step_deg", 45.0)
        self.declare_parameter("scan_positions", 8)
        self.declare_parameter("turn_omega", 0.10)
        self.declare_parameter("turn_slow_omega", 0.07)
        self.declare_parameter("turn_slowdown_deg", 10.0)
        self.declare_parameter("turn_pulse_sec", 0.24)
        self.declare_parameter("turn_burst_pause_sec", 0.18)
        self.declare_parameter("turn_settle_sec", 0.70)
        self.declare_parameter("turn_verify_sec", 0.60)
        self.declare_parameter("turn_verify_max_corrections", 4)
        self.declare_parameter("turn_correction_pulse_sec", 0.10)
        self.declare_parameter("turn_correction_settle_sec", 0.35)
        self.declare_parameter("turn_tolerance_deg", 5.0)
        self.declare_parameter("max_turn_pulses", 60)
        self.declare_parameter("scan_observe_sec", 1.00)
        self.declare_parameter("single_lap_inspection", True)
        self.declare_parameter("initial_inventory_observe_sec", 2.00)
        self.declare_parameter("face_observe_sec", 0.75)
        self.declare_parameter("candidate_radius_min_m", 0.12)
        self.declare_parameter("candidate_radius_max_m", 0.50)
        self.declare_parameter("candidate_merge_radius_m", 0.14)
        self.declare_parameter("candidate_merge_bearing_deg", 12.0)
        self.declare_parameter("candidate_merge_radial_m", 0.10)
        self.declare_parameter("candidate_min_hits", 2)
        self.declare_parameter("inventory_max_candidates", 4)
        self.declare_parameter("fruit_cube_override_confidence", 0.60)
        self.declare_parameter("expected_candidates_min", 3)
        self.declare_parameter("expected_candidates_max", 4)
        self.declare_parameter("classify_min_confidence", 0.08)
        self.declare_parameter("classify_stable_frames", 2)
        self.declare_parameter("classify_timeout_sec", 4.0)
        self.declare_parameter("classify_hold_on_failure", True)
        self.declare_parameter("visual_heading_enabled", True)
        self.declare_parameter("visual_heading_min_confidence", 0.60)
        self.declare_parameter("visual_heading_center_x_px", 320.0)
        self.declare_parameter("visual_heading_center_tolerance_px", 40.0)
        self.declare_parameter("visual_heading_confirm_frames", 3)
        self.declare_parameter("visual_heading_coarse_gate_deg", 20.0)
        self.declare_parameter("visual_heading_max_correction_deg", 20.0)
        self.declare_parameter("target_fruit_label", "banana")
        self.declare_parameter("enable_align", False)
        self.declare_parameter("pick_enabled", False)
        self.declare_parameter("pick_duration_sec", 9.0)
        self.declare_parameter("grab_x_m", 0.19)
        self.declare_parameter("grab_y_m", 0.0)
        self.declare_parameter("grab_min_x_m", 0.17)
        self.declare_parameter("align_fwd_tolerance_m", 0.02)
        self.declare_parameter("align_lateral_tolerance_m", 0.02)
        self.declare_parameter("align_fwd_duty", 0.27)
        self.declare_parameter("align_fwd_pulse_sec", 0.15)
        self.declare_parameter("align_strafe_duty", 0.315)
        self.declare_parameter("align_strafe_pulse_sec", 0.30)
        self.declare_parameter("align_adaptive_steps_enabled", True)
        self.declare_parameter("align_mid_error_m", 0.06)
        self.declare_parameter("align_fwd_mid_pulse_sec", 0.25)
        self.declare_parameter("align_strafe_mid_pulse_sec", 0.60)
        self.declare_parameter("align_settle_sec", 0.60)
        self.declare_parameter("align_target_timeout_sec", 2.0)
        self.declare_parameter("align_timeout_sec", 12.0)
        self.declare_parameter(
            "body_ground_homography_path",
            "/home/seventt/seventt/workspace/data/calib/body_ground.npz",
        )
        self.declare_parameter("body_px_scale_x", 2.5625)
        self.declare_parameter("body_px_scale_y", 2.566667)
        self.declare_parameter("body_cam_nadir_x", 0.055)
        self.declare_parameter("body_cam_height_m", 0.155)
        self.declare_parameter("object_center_height_m", 0.04)

    def _read_config(self) -> LocalAnchorConfig:
        def value(name):
            return self.get_parameter(name).value

        return LocalAnchorConfig(
            scan_step_rad=math.radians(float(value("scan_step_deg"))),
            scan_positions=int(value("scan_positions")),
            turn_omega=float(value("turn_omega")),
            turn_slow_omega=float(value("turn_slow_omega")),
            turn_slowdown_rad=math.radians(float(value("turn_slowdown_deg"))),
            turn_pulse_sec=float(value("turn_pulse_sec")),
            turn_burst_pause_sec=float(value("turn_burst_pause_sec")),
            turn_settle_sec=float(value("turn_settle_sec")),
            turn_verify_sec=float(value("turn_verify_sec")),
            turn_verify_max_corrections=int(value("turn_verify_max_corrections")),
            turn_correction_pulse_sec=float(value("turn_correction_pulse_sec")),
            turn_correction_settle_sec=float(value("turn_correction_settle_sec")),
            turn_tolerance_rad=math.radians(float(value("turn_tolerance_deg"))),
            max_turn_pulses=int(value("max_turn_pulses")),
            scan_observe_sec=float(value("scan_observe_sec")),
            single_lap_inspection=bool(value("single_lap_inspection")),
            initial_inventory_observe_sec=float(
                value("initial_inventory_observe_sec")
            ),
            face_observe_sec=float(value("face_observe_sec")),
            candidate_radius_min_m=float(value("candidate_radius_min_m")),
            candidate_radius_max_m=float(value("candidate_radius_max_m")),
            candidate_merge_radius_m=float(value("candidate_merge_radius_m")),
            candidate_merge_bearing_rad=math.radians(
                float(value("candidate_merge_bearing_deg"))
            ),
            candidate_merge_radial_m=float(value("candidate_merge_radial_m")),
            candidate_min_hits=int(value("candidate_min_hits")),
            inventory_max_candidates=int(value("inventory_max_candidates")),
            fruit_cube_override_confidence=float(
                value("fruit_cube_override_confidence")
            ),
            expected_candidates_min=int(value("expected_candidates_min")),
            expected_candidates_max=int(value("expected_candidates_max")),
            fruit_inspection_labels=tuple(
                str(label) for label in value("fruit_inspection_labels")
            ),
            classify_min_confidence=float(value("classify_min_confidence")),
            classify_stable_frames=int(value("classify_stable_frames")),
            classify_timeout_sec=float(value("classify_timeout_sec")),
            classify_hold_on_failure=bool(value("classify_hold_on_failure")),
            visual_heading_enabled=bool(value("visual_heading_enabled")),
            visual_heading_min_confidence=float(
                value("visual_heading_min_confidence")
            ),
            visual_heading_center_x_px=float(value("visual_heading_center_x_px")),
            visual_heading_center_tolerance_px=float(
                value("visual_heading_center_tolerance_px")
            ),
            visual_heading_confirm_frames=int(value("visual_heading_confirm_frames")),
            visual_heading_coarse_gate_rad=math.radians(
                float(value("visual_heading_coarse_gate_deg"))
            ),
            visual_heading_max_correction_rad=math.radians(
                float(value("visual_heading_max_correction_deg"))
            ),
            target_fruit_label=str(value("target_fruit_label")),
            enable_align=bool(value("enable_align")),
            enable_pick=bool(value("pick_enabled")),
            pick_duration_sec=float(value("pick_duration_sec")),
            grab_x_m=float(value("grab_x_m")),
            grab_y_m=float(value("grab_y_m")),
            grab_min_x_m=float(value("grab_min_x_m")),
            align_fwd_tolerance_m=float(value("align_fwd_tolerance_m")),
            align_lateral_tolerance_m=float(value("align_lateral_tolerance_m")),
            align_fwd_duty=float(value("align_fwd_duty")),
            align_fwd_pulse_sec=float(value("align_fwd_pulse_sec")),
            align_strafe_duty=float(value("align_strafe_duty")),
            align_strafe_pulse_sec=float(value("align_strafe_pulse_sec")),
            align_adaptive_steps_enabled=bool(
                value("align_adaptive_steps_enabled")
            ),
            align_mid_error_m=float(value("align_mid_error_m")),
            align_fwd_mid_pulse_sec=float(value("align_fwd_mid_pulse_sec")),
            align_strafe_mid_pulse_sec=float(value("align_strafe_mid_pulse_sec")),
            align_settle_sec=float(value("align_settle_sec")),
            align_target_timeout_sec=float(value("align_target_timeout_sec")),
            align_timeout_sec=float(value("align_timeout_sec")),
        )

    def _load_body_projection(self) -> None:
        self.body_h = None
        path = str(self.get_parameter("body_ground_homography_path").value)
        try:
            if not os.path.isfile(path):
                raise FileNotFoundError(path)
            self.body_h = np.load(path)["H"].astype(np.float64)
        except Exception as exc:  # noqa: BLE001 - test may intentionally run without ALIGN
            self.get_logger().warn(f"body homography unavailable: {exc}")
        self.body_scale_x = float(self.get_parameter("body_px_scale_x").value)
        self.body_scale_y = float(self.get_parameter("body_px_scale_y").value)
        self.body_nadir_x = float(self.get_parameter("body_cam_nadir_x").value)
        self.body_height = float(self.get_parameter("body_cam_height_m").value)
        self.object_height = float(self.get_parameter("object_center_height_m").value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _relative_yaw(self) -> float:
        if self.use_imu_yaw:
            return wrap_angle(self.imu_relative_yaw)
        if self.current_yaw is None or self.anchor_yaw is None:
            return 0.0
        return wrap_angle(self.current_yaw - self.anchor_yaw)

    def _on_pose(self, msg: PoseStamped) -> None:
        self.current_yaw = quaternion_yaw(msg)

    def _on_imu(self, _msg: Imu) -> None:
        self.last_imu_s = self._now()

    def _on_imu_yaw_delta(self, msg: Float32) -> None:
        delta = float(msg.data)
        if self._active and math.isfinite(delta):
            self.imu_relative_yaw = integrate_imu_yaw(self.imu_relative_yaw, delta)

    def _on_competition(self, msg: String) -> None:
        self.competition_state = str(msg.data).strip().upper()

    def _on_deadman(self, _msg: Empty) -> None:
        self.last_deadman_s = self._now()

    def _on_relative_objects(self, msg: WorldModel) -> None:
        now = self._now()
        self.last_relative_s = now
        yaw = self._relative_yaw()
        c, s = math.cos(yaw), math.sin(yaw)
        observations = []
        for obj in msg.objects:
            label = str(obj.class_label).strip()
            if label not in self.candidate_labels:
                continue
            bx, by = float(obj.x), float(obj.y)
            observations.append(
                LocalObservation(
                    x=c * bx - s * by,
                    y=s * bx + c * by,
                    label=label,
                    confidence=float(obj.confidence),
                )
            )
        self.fsm.add_observations(observations)

    def _on_classification(self, msg: Classification) -> None:
        if self.fsm.state != "CLASSIFY":
            return
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_s > 0.0 and stamp_s + 1e-6 < self.fsm.state_enter_s:
            return
        self.fsm.note_classification(
            str(msg.label),
            float(msg.confidence),
            bool(msg.image_face_visible),
            self._now(),
            is_target=bool(msg.is_target),
        )

    def _body_pixel_base(self, u: float, v: float) -> tuple[float, float] | None:
        if self.body_h is None or self.body_height <= 1e-6:
            return None
        point = np.array(
            [[[float(u) * self.body_scale_x, float(v) * self.body_scale_y]]],
            dtype=np.float64,
        )
        projected = cv2.perspectiveTransform(point, self.body_h)[0][0]
        ratio = self.object_height / self.body_height
        bx = float(projected[0]) - ratio * (float(projected[0]) - self.body_nadir_x)
        by = float(projected[1]) - ratio * float(projected[1])
        return bx, by

    def _on_body_detections(self, msg: DetectionArray) -> None:
        now = self._now()
        visual_best = None
        visual_confidence = 0.0
        visual_turn_hint = 0.0
        if self.config.visual_heading_enabled:
            for detection in msg.detections:
                if str(detection.label).strip().lower() != "fruit_photo_cube":
                    continue
                confidence = float(detection.confidence)
                if confidence < self.config.visual_heading_min_confidence:
                    continue
                center_error = abs(
                    float(detection.x_center) - self.config.visual_heading_center_x_px
                )
                score = (center_error, -confidence)
                if visual_best is None or score < visual_best[0]:
                    visual_best = (score, float(detection.x_center))
                    visual_confidence = confidence
                    base = self._body_pixel_base(
                        detection.x_center,
                        detection.y_center,
                    )
                    if base is not None:
                        visual_turn_hint = math.atan2(base[1], max(0.01, base[0]))
            previous_state = self.fsm.state
            self.fsm.note_visual_fruit_center(
                None if visual_best is None else visual_best[1],
                visual_confidence,
                now,
                self._relative_yaw(),
                visual_turn_hint,
            )
            if (
                previous_state in {"TURN_CONTINUOUS", "TURN_PULSE"}
                and self.fsm.state == "TURN_VERIFY"
            ):
                self._publish_command(MotionCommand())

        if not self.fsm.state.startswith("ALIGN"):
            return
        best = None
        best_distance = 0.60
        for detection in msg.detections:
            if str(detection.label).strip() not in self.body_labels:
                continue
            base = self._body_pixel_base(detection.x_center, detection.y_center)
            if base is None:
                continue
            distance = math.hypot(
                base[0] - self.config.grab_x_m,
                base[1] - self.config.grab_y_m,
            )
            if distance < best_distance:
                best = base
                best_distance = distance
        if best is not None:
            self.fsm.note_align_target(best[0], best[1], now)

    def _other_base_publishers(self) -> list[str]:
        others = []
        for info in self.get_publishers_info_by_topic("/base_command"):
            if info.node_name != self.get_name():
                others.append(f"{info.node_namespace.rstrip('/')}/{info.node_name}")
        return sorted(set(others))

    def _start_checks(self, now: float) -> str | None:
        config_error = self._config_error()
        if config_error is not None:
            return config_error
        if self.current_yaw is None:
            return "no /localization/pose yet"
        if self.use_imu_yaw and now - self.last_imu_s > self.imu_timeout_sec:
            return "IMU stream is stale"
        if now - self.last_relative_s > self.relative_topic_timeout_sec:
            return "wide relative-object stream is stale"
        if self.require_running and self.competition_state != "RUNNING":
            return f"competition state is {self.competition_state}, not RUNNING"
        if self.drive_enabled and not self.armed:
            return "drive_enabled=true but software arm is false"
        if self.drive_enabled and self.require_deadman:
            if now - self.last_deadman_s > self.deadman_timeout_sec:
                return "deadman heartbeat is missing"
        if self.check_publishers:
            publishers = self._other_base_publishers()
            if publishers:
                return f"another /base_command publisher is active: {publishers}"
        if self.config.enable_align and self.body_h is None:
            return "enable_align=true but body homography is unavailable"
        if self.config.enable_pick:
            if not self.drive_enabled:
                return "pick_enabled=true requires drive_enabled=true"
            if not self.get_subscriptions_info_by_topic("/arm/pick_trigger"):
                return "pick sequencer is not subscribed to /arm/pick_trigger"
        return None

    def _config_error(self) -> str | None:
        cfg = self.config
        checks = [
            (0.03 <= abs(cfg.turn_omega) <= 0.50, "turn_omega must be in [0.03, 0.50]"),
            (
                0.03 <= abs(cfg.turn_slow_omega) <= abs(cfg.turn_omega),
                "turn_slow_omega must be in [0.03, turn_omega]",
            ),
            (0.03 <= cfg.turn_pulse_sec <= 0.50, "turn_pulse_sec must be in [0.03, 0.50]"),
            (
                0.10 <= cfg.turn_burst_pause_sec <= 0.50,
                "turn_burst_pause_sec must be in [0.10, 0.50]",
            ),
            (0.20 <= cfg.turn_settle_sec <= 2.0, "turn_settle_sec must be in [0.20, 2.0]"),
            (
                0.20 <= cfg.turn_verify_sec <= 2.0,
                "turn_verify_sec must be in [0.20, 2.0]",
            ),
            (
                cfg.turn_tolerance_rad < cfg.turn_slowdown_rad <= math.radians(45.0),
                "turn_slowdown_deg must be between tolerance and 45deg",
            ),
            (
                0 <= cfg.turn_verify_max_corrections <= 10,
                "turn_verify_max_corrections must be in [0, 10]",
            ),
            (
                0.03 <= cfg.turn_correction_pulse_sec <= 0.30,
                "turn_correction_pulse_sec must be in [0.03, 0.30]",
            ),
            (
                0.10 <= cfg.turn_correction_settle_sec <= 1.0,
                "turn_correction_settle_sec must be in [0.10, 1.0]",
            ),
            (1 <= cfg.scan_positions <= 16, "scan_positions must be in [1, 16]"),
            (
                0.5 <= cfg.initial_inventory_observe_sec <= 10.0,
                "initial_inventory_observe_sec must be in [0.5, 10.0]",
            ),
            (
                0.1 <= self.imu_timeout_sec <= 2.0,
                "imu_timeout_sec must be in [0.1, 2.0]",
            ),
            (
                0.10 <= cfg.candidate_radius_max_m <= 0.60,
                "candidate_radius_max_m must be in [0.10, 0.60]",
            ),
            (
                0.0 <= cfg.fruit_cube_override_confidence <= 1.0,
                "fruit_cube_override_confidence must be in [0.0, 1.0]",
            ),
            (0.0 < cfg.align_fwd_duty <= 0.50, "align_fwd_duty must be in (0, 0.50]"),
            (
                0.0 < cfg.align_strafe_duty <= 0.50,
                "align_strafe_duty must be in (0, 0.50]",
            ),
            (
                0.03 <= cfg.align_fwd_pulse_sec <= 0.80,
                "align_fwd_pulse_sec must be in [0.03, 0.80]",
            ),
            (
                0.03 <= cfg.align_strafe_pulse_sec <= 0.80,
                "align_strafe_pulse_sec must be in [0.03, 0.80]",
            ),
            (
                1.0 <= cfg.pick_duration_sec <= 20.0,
                "pick_duration_sec must be in [1.0, 20.0]",
            ),
        ]
        for valid, reason in checks:
            if not valid:
                return f"unsafe test config: {reason}"
        return None

    def _on_control(self, msg: String) -> None:
        now = self._now()
        command = str(msg.data).strip().upper()
        if command == "ARM":
            self.armed = True
            self.get_logger().warn("software motion arm enabled")
            return
        if command == "DISARM":
            self.armed = False
            self._pending_start = False
            self._abort_and_stop(now, "DISARM")
            return
        if command == "ABORT":
            self._pending_start = False
            self._abort_and_stop(now, "operator ABORT")
            return
        if command == "RESET":
            self._active = False
            self._pending_start = False
            self.anchor_yaw = None
            self.imu_relative_yaw = 0.0
            self.fsm = LocalAnchorFruitFsm(self.config)
            self._stop_publish_until_s = now + 0.5
            self.get_logger().info("local-anchor test reset")
            return
        if command != "START":
            self.get_logger().warn(f"unknown control command: {command}")
            return
        if self._active:
            self.get_logger().info("START ignored: local-anchor test is already active")
            return
        self._pending_start = True
        self._try_pending_start(now)

    def _try_pending_start(self, now: float) -> None:
        if not self._pending_start or self._active:
            return
        if now - self._last_start_attempt_s < 0.5:
            return
        self._last_start_attempt_s = now
        reason = self._start_checks(now)
        if reason is not None:
            detail = f"START waiting: {reason}"
            if detail != self.fsm.detail:
                self.fsm.detail = detail
                self._publish_status(now, force=True)
            return
        self.anchor_yaw = self.current_yaw
        self.imu_relative_yaw = 0.0
        self.fsm.start(now, current_yaw=0.0)
        self._active = True
        self._pending_start = False
        self.get_logger().warn(
            f"START accepted at local anchor yaw={self.anchor_yaw:.3f}; "
            f"target={self.config.target_fruit_label}"
        )

    def _abort_and_stop(self, now: float, reason: str) -> None:
        self.fsm.abort(now, reason)
        self._active = False
        self._stop_publish_until_s = now + 1.0
        self._publish_command(MotionCommand())
        self.get_logger().error(f"local-anchor test stopped: {reason}")

    def _runtime_safety_reason(self, now: float) -> str | None:
        if not self._active:
            return None
        if self.require_running and self.competition_state != "RUNNING":
            return f"competition left RUNNING ({self.competition_state})"
        if self.drive_enabled and not self.armed:
            return "software arm disabled"
        if self.drive_enabled and self.require_deadman:
            if now - self.last_deadman_s > self.deadman_timeout_sec:
                return "deadman heartbeat timed out"
        if self.check_publishers:
            publishers = self._other_base_publishers()
            if publishers:
                return f"another /base_command publisher appeared: {publishers}"
        return None

    def _publish_command(self, command: MotionCommand) -> None:
        msg = BaseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.vx = float(command.vx)
        msg.vy = float(command.vy)
        msg.omega = float(command.omega)
        self.pub_cmd.publish(msg)

    def _publish_mission_state(self, now: float) -> None:
        msg = MissionState()
        msg.state = "ALIGN" if self.fsm.state.startswith("ALIGN") else "LOCAL_ANCHOR_TEST"
        msg.tray_shape_count = 0
        msg.tray_fruit_count = 0
        active = self.fsm.active_candidate
        msg.current_target_id = active.candidate_id if active is not None else 0
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_state.publish(msg)

    def _publish_status(self, now: float, force: bool = False) -> None:
        unchanged = self.fsm.state == self._last_status_state
        if not force and unchanged and now - self._last_status_s < 0.5:
            return
        payload = self.fsm.summary()
        payload.update(
            {
                "active": self._active,
                "pending_start": self._pending_start,
                "drive_enabled": self.drive_enabled,
                "armed": self.armed,
                "deadman_age_sec": (
                    None if not math.isfinite(self.last_deadman_s) else now - self.last_deadman_s
                ),
                "competition_state": self.competition_state,
                "placement_seed": self.placement_seed,
                "pick_enabled": self.config.enable_pick,
                "relative_yaw_deg": round(math.degrees(self._relative_yaw()), 2),
                "heading_source": "imu_delta" if self.use_imu_yaw else "localization_pose",
                "imu_age_sec": (
                    None if not math.isfinite(self.last_imu_s) else now - self.last_imu_s
                ),
            }
        )
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.pub_status.publish(String(data=encoded))
        if self.fsm.state != self._last_status_state:
            self.get_logger().info(encoded)
        self._last_status_s = now
        self._last_status_state = self.fsm.state

    def _tick(self) -> None:
        now = self._now()
        self._try_pending_start(now)
        reason = self._runtime_safety_reason(now)
        if reason is not None:
            self._abort_and_stop(now, reason)

        command = MotionCommand()
        if self._active:
            command = self.fsm.tick(now, self._relative_yaw())
            if command.request_pick and self.config.enable_pick:
                self.pub_pick.publish(Bool(data=True))
                self.get_logger().warn("real /arm/pick_trigger sent; base remains locked")
            if self.fsm.state in TERMINAL_STATES:
                self._active = False
                self._stop_publish_until_s = now + 1.0
        can_drive = self.drive_enabled and self.armed and self._active
        if can_drive:
            self._publish_command(command)
        elif now <= self._stop_publish_until_s:
            self._publish_command(MotionCommand())
        self._publish_mission_state(now)
        self._publish_status(now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalAnchorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_command(MotionCommand())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
