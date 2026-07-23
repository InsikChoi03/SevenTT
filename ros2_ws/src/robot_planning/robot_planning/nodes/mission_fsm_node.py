"""Mission FSM for the AI Robot Challenge.

States:
    SCAN              - top cam world model build/update
    LOCAL_ANCHOR_INSPECTION - run one isolated Set2 fruit sweep at each zone anchor
    ZONE_STABILIZE    - stop at the active zone anchor and trust stationary observations
    SELECT_TARGET     - pick next object via /selected_target
    APPROACH          - drive base toward target
    ALIGN             - body cam visual servo
    CLASSIFY          - SigLIP gate (+ shape heuristic for set1)
    PICK              - arm pickup motion
    STORE_IN_TRAY     - place in body tray, increment counters
    WAIT_FOR_STORAGE  - hold after early mission completion until the timed parking deadline
    DRIVE_TO_STORAGE  - approach bottom wall, face right, reverse to left wall
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

from dataclasses import dataclass
import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from robot_interfaces.msg import BaseCommand, Classification, DetectionArray, MissionState, Object, WorldModel
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, Int8, String, UInt64

from robot_planning.anchor_slot_inventory import (
    AnchorSlot,
    AnchorSlotInventory,
    body_observation_matches_slot,
    slot_requires_body_confirmation,
)
from robot_planning.global_target_planner import (
    best_observation_pose,
    grid_nodes,
    next_zigzag_observation_pose,
    nodes_in_ellipse_fov,
    order_targets_min_path,
)
from robot_planning.lane_planner import LanePlanner, cardinal_segment_heading
from robot_planning.local_anchor_fsm import (
    LocalAnchorConfig,
    LocalAnchorFruitFsm,
    LocalObservation,
    TERMINAL_STATES as LOCAL_ANCHOR_TERMINAL_STATES,
    integrate_imu_yaw,
)
from robot_planning.object_slot_inventory import ObjectSlot, SlotInventory
from robot_planning.relative_anchor_grid import (
    RelativeAnchorGridTracker,
    RelativeObservation,
)

# Body detection label -> set_type (mirror of world_model), for the ALIGN visual-servo filter.
_LABEL_ST = {"cube": 1, "octahedron": 1, "dodecahedron": 1, "icosahedron": 1, "fruit_photo_cube": 2}


LOCAL_ANCHOR_STATE = "LOCAL_ANCHOR_INSPECTION"
LOCAL_ANCHOR_ALIGN_STATE = "LOCAL_ANCHOR_ALIGN"


@dataclass(frozen=True)
class Set2IdentityLatch:
    """A Body SigLIP identity bound to one selected field target at capture time."""

    target_id: int
    target_x: float
    target_y: float
    verdict: str
    label: str
    confidence: float
    capture_s: float
    received_s: float
    association_error_m: float
    visit_serial: int


def parse_body_siglip_pixel(source: str) -> tuple[float, float] | None:
    """Decode ``siglip_body:u:v`` provenance without depending on perception internals."""
    parts = str(source).strip().split(":")
    if len(parts) != 3 or parts[0] != "siglip_body":
        return None
    try:
        u, v = float(parts[1]), float(parts[2])
    except ValueError:
        return None
    return (u, v) if math.isfinite(u) and math.isfinite(v) else None


def fresh_set2_siglip_verdict(
    classification,
    capture_s: float | None,
    *,
    received_s: float | None = None,
    state_enter_s: float,
    now_s: float,
    max_age_sec: float,
    confidence_threshold: float,
    target_label: str,
    spatially_aligned: bool,
) -> tuple[str, str, float]:
    """Return ``target``, ``non_target`` or ``pending`` for a final Body read."""
    if classification is None or capture_s is None or not spatially_aligned:
        return "pending", "", 0.0
    # Capture time proves that the image belongs to this stopped CLASSIFY visit. Freshness after
    # that uses receive time because Jetson GPU inference normally finishes 0.8-1.0 s later.
    freshness_s = capture_s if received_s is None else received_s
    if capture_s + 1e-6 < state_enter_s or now_s - freshness_s > max_age_sec:
        return "pending", "", 0.0
    return set2_classification_verdict(
        classification,
        confidence_threshold=confidence_threshold,
        target_label=target_label,
    )


def set2_classification_verdict(
    classification,
    *,
    confidence_threshold: float,
    target_label: str,
) -> tuple[str, str, float]:
    """Convert one spatially-associated SigLIP read to a Set2 identity verdict."""
    if classification is None:
        return "pending", "", 0.0
    label = str(classification.label).strip()
    confidence = float(classification.confidence)
    if not bool(classification.image_face_visible) or confidence < confidence_threshold:
        return "pending", label, confidence
    if label == str(target_label) and bool(classification.is_target):
        return "target", label, confidence
    if label and label != str(target_label):
        return "non_target", label, confidence
    return "pending", label, confidence


def closest_pose_sample(
    samples: list[tuple[float, float, float, float]],
    capture_s: float,
    max_skew_sec: float,
) -> tuple[float, float, float] | None:
    """Return the localization pose nearest a camera capture timestamp."""
    if not samples or capture_s <= 0.0:
        return None
    best = min(samples, key=lambda sample: abs(float(sample[0]) - float(capture_s)))
    if abs(float(best[0]) - float(capture_s)) > max(0.0, float(max_skew_sec)):
        return None
    return float(best[1]), float(best[2]), float(best[3])


def field_point_to_base_at_pose(
    field_xy: tuple[float, float],
    pose: tuple[float, float, float],
) -> tuple[float, float]:
    """Transform a fixed field point into base_link at a historical robot pose."""
    dx = float(field_xy[0]) - float(pose[0])
    dy = float(field_xy[1]) - float(pose[1])
    ct, st = math.cos(float(pose[2])), math.sin(float(pose[2]))
    return ct * dx + st * dy, -st * dx + ct * dy


def body_source_matches_target_at_capture(
    source_base_xy: tuple[float, float],
    target_field_xy: tuple[float, float],
    capture_pose: tuple[float, float, float],
    match_radius_m: float,
) -> bool:
    """Bind a Body SigLIP crop to the selected map target in the same time frame."""
    expected = field_point_to_base_at_pose(target_field_xy, capture_pose)
    return math.hypot(
        float(source_base_xy[0]) - expected[0],
        float(source_base_xy[1]) - expected[1],
    ) <= max(0.0, float(match_radius_m))


def set2_classification_timing_is_valid(
    capture_s: float,
    received_s: float,
    visit_started_s: float,
    max_inference_delay_sec: float,
) -> bool:
    """Reject pre-visit, future-clock, and excessively delayed classification frames."""
    values = (capture_s, received_s, visit_started_s, max_inference_delay_sec)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    delay_s = float(received_s) - float(capture_s)
    return bool(
        float(capture_s) + 1e-6 >= float(visit_started_s)
        and -0.25 <= delay_s <= max(0.0, float(max_inference_delay_sec))
    )


def set2_latch_target_matches(
    target,
    latch: Set2IdentityLatch,
    match_radius_m: float,
    *,
    locally_blacklisted: bool = False,
) -> bool:
    """Match by physical field position; a tracker ID alone is never pick authority."""
    if (
        target is None
        or int(target.set_type) != 2
        or bool(getattr(target, "blacklisted", False))
        or locally_blacklisted
    ):
        return False
    return math.hypot(
        float(target.x) - float(latch.target_x),
        float(target.y) - float(latch.target_y),
    ) <= max(0.0, float(match_radius_m))


def select_body_candidate(
    candidates: list[tuple[str, float, float, float]],
    *,
    grab_xy: tuple[float, float],
    expected_xy: tuple[float, float] | None,
    expected_match_radius_m: float,
    grab_search_radius_m: float = 0.50,
) -> tuple[str, float, float, float] | None:
    """Associate Body geometry with the selected map target before considering the grab point."""
    gx, gy = float(grab_xy[0]), float(grab_xy[1])
    if expected_xy is not None:
        ex, ey = float(expected_xy[0]), float(expected_xy[1])
        eligible = [
            candidate
            for candidate in candidates
            if math.hypot(candidate[1] - ex, candidate[2] - ey)
            <= max(0.0, float(expected_match_radius_m))
        ]
        return min(
            eligible,
            key=lambda candidate: math.hypot(candidate[1] - ex, candidate[2] - ey),
            default=None,
        )
    return min(
        (
            candidate
            for candidate in candidates
            if math.hypot(candidate[1] - gx, candidate[2] - gy)
            <= max(0.0, float(grab_search_radius_m))
        ),
        key=lambda candidate: math.hypot(candidate[1] - gx, candidate[2] - gy),
        default=None,
    )


def set2_pick_allowed(
    target,
    *,
    pick_track_confidence: float,
    require_fruit_label: bool,
    body_verdict: str,
) -> bool:
    """Enforce that routing labels alone never authorize a Set2 arm command."""
    if (
        target is None
        or bool(getattr(target, "blacklisted", False))
        or int(target.set_type) != 2
    ):
        return False
    if float(target.confidence) < float(pick_track_confidence):
        return False
    return body_verdict == "target" if require_fruit_label else True


def keep_latched_global_approach(
    *,
    global_target_mode: bool,
    target_xy: tuple[float, float] | None,
    current_target,
) -> bool:
    """Keep one frozen global stand-off route across temporary perception dropout."""
    return bool(
        global_target_mode
        and target_xy is not None
        and current_target is not None
        and not bool(current_target.blacklisted)
    )


def global_target_absence_confirmed(
    *,
    elapsed_s: float,
    fresh_missing_body_frames: int,
    timeout_sec: float,
    confirm_frames: int,
) -> bool:
    """Confirm a vanished map target without reacting to one dropped camera frame."""
    return bool(
        float(elapsed_s) >= max(0.0, float(timeout_sec))
        and int(fresh_missing_body_frames) >= max(1, int(confirm_frames))
    )


def target_is_spatially_deferred(
    target,
    deferred: list[tuple[int, float, float, float]],
    now_s: float,
    match_radius_m: float,
) -> bool:
    """Match a temporary rejection by physical position, surviving tracker-ID churn."""
    return any(
        int(target.set_type) == int(set_type)
        and float(until_s) > float(now_s)
        and math.hypot(float(target.x) - float(x), float(target.y) - float(y))
        <= max(0.0, float(match_radius_m))
        for set_type, x, y, until_s in deferred
    )


def global_retarget_is_worthwhile(
    current_distance_m: float,
    new_distance_m: float,
    min_gain: float,
    lock_distance_m: float,
) -> bool:
    """Allow a route switch only before final approach and for a material gain."""
    current = max(0.0, float(current_distance_m))
    if current <= max(0.0, float(lock_distance_m)):
        return False
    return float(new_distance_m) <= (1.0 - float(min_gain)) * current


def clamp_outward_field_velocity(
    vx: float,
    vy: float,
    theta: float,
    robot_x: float,
    robot_y: float,
    interior_bounds: tuple[float, float, float, float],
    guard_m: float = 0.03,
) -> tuple[float, float]:
    """Remove only the field-velocity component that would cross a safe arena edge."""
    c, s = math.cos(float(theta)), math.sin(float(theta))
    field_vx = c * float(vx) - s * float(vy)
    field_vy = s * float(vx) + c * float(vy)
    xmin, xmax, ymin, ymax = (float(value) for value in interior_bounds)
    guard = max(0.0, float(guard_m))
    if float(robot_x) <= xmin + guard and field_vx < 0.0:
        field_vx = 0.0
    elif float(robot_x) >= xmax - guard and field_vx > 0.0:
        field_vx = 0.0
    if float(robot_y) <= ymin + guard and field_vy < 0.0:
        field_vy = 0.0
    elif float(robot_y) >= ymax - guard and field_vy > 0.0:
        field_vy = 0.0
    return c * field_vx + s * field_vy, -s * field_vx + c * field_vy

# These names intentionally mirror local_anchor_test.yaml.  The prefix keeps the
# embedded mission instance independent from the main ALIGN/CLASSIFY tuning.
LOCAL_ANCHOR_PARAMETER_DEFAULTS = {
    "enabled": False,
    "run_timeout_sec": 120.0,
    "input_timeout_sec": 0.75,
    "pose_confirm_frames": 3,
    "world_confirm_frames": 2,
    "pose_max_age_sec": 0.50,
    "pose_world_xy_tolerance_m": 0.03,
    "pose_world_theta_tolerance_deg": 3.0,
    "candidate_labels": [
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
    "fruit_inspection_labels": [
        "fruit_photo_cube",
        "apple",
        "orange",
        "banana",
        "pineapple",
    ],
    "body_align_labels": ["fruit_photo_cube", "cube"],
    "scan_step_deg": 45.0,
    "scan_positions": 8,
    "turn_omega": 0.10,
    "turn_slow_omega": 0.07,
    "turn_slowdown_deg": 10.0,
    "turn_pulse_sec": 0.24,
    "turn_burst_pause_sec": 0.18,
    "turn_settle_sec": 0.70,
    "turn_verify_sec": 0.60,
    "turn_verify_max_corrections": 4,
    "turn_correction_pulse_sec": 0.10,
    "turn_correction_settle_sec": 0.35,
    "turn_tolerance_deg": 5.0,
    "max_turn_pulses": 60,
    "scan_observe_sec": 1.00,
    "single_lap_inspection": True,
    "initial_inventory_observe_sec": 2.00,
    "face_observe_sec": 0.75,
    "candidate_radius_min_m": 0.12,
    "candidate_radius_max_m": 0.50,
    "candidate_merge_radius_m": 0.14,
    "candidate_merge_bearing_deg": 12.0,
    "candidate_merge_radial_m": 0.10,
    "candidate_min_hits": 2,
    "inventory_max_candidates": 4,
    "fruit_cube_override_confidence": 0.60,
    "expected_candidates_min": 3,
    "expected_candidates_max": 4,
    "classify_min_confidence": 0.08,
    "classify_stable_frames": 2,
    "classify_timeout_sec": 4.0,
    "classify_hold_on_failure": True,
    "visual_heading_enabled": True,
    "visual_heading_min_confidence": 0.60,
    "visual_heading_center_x_px": 320.0,
    "visual_heading_center_tolerance_px": 40.0,
    "visual_heading_confirm_frames": 3,
    "visual_heading_coarse_gate_deg": 10.0,
    "visual_heading_max_correction_deg": 20.0,
    "enable_align": True,
    "pick_enabled": False,
    "pick_duration_sec": 11.0,
    "grab_x_m": 0.19,
    "grab_y_m": 0.0,
    "grab_min_x_m": 0.17,
    "align_fwd_tolerance_m": 0.02,
    "align_lateral_tolerance_m": 0.02,
    "align_fwd_duty": 0.27,
    "align_fwd_pulse_sec": 0.15,
    "align_strafe_duty": 0.315,
    "align_strafe_pulse_sec": 0.30,
    "align_adaptive_steps_enabled": True,
    "align_mid_error_m": 0.06,
    "align_fwd_mid_pulse_sec": 0.25,
    "align_strafe_mid_pulse_sec": 0.60,
    "align_settle_sec": 1.00,
    "align_target_timeout_sec": 2.0,
    "align_timeout_sec": 12.0,
    "picked_track_match_radius_m": 0.25,
}


def _local_anchor_observations(
    observations: list[RelativeObservation],
    relative_yaw: float,
    candidate_labels: set[str],
) -> list[LocalObservation]:
    """Rotate fresh base-link observations into the run's fixed anchor frame."""
    c, s = math.cos(float(relative_yaw)), math.sin(float(relative_yaw))
    converted: list[LocalObservation] = []
    for observation in observations:
        label = str(observation.label).strip().lower()
        if label not in candidate_labels:
            continue
        bx, by = float(observation.x), float(observation.y)
        converted.append(
            LocalObservation(
                x=c * bx - s * by,
                y=s * bx + c * by,
                label=label,
                confidence=float(observation.confidence),
            )
        )
    return converted


def plain_cube_approach_invalid(
    set1_label: str,
    phase: int,
    opportunistic_set2_active: bool,
    target: Object | None,
) -> bool:
    """Return whether a latched plain-cube approach became a confirmed other identity."""
    return bool(
        set1_label == "cube"
        and int(phase) == 1
        and not opportunistic_set2_active
        and target is not None
        and int(target.set_type) != 0
        and (int(target.set_type) != 1 or str(target.class_label) != "cube")
    )


def _parse_zone_anchor_candidates(
    values: list[float], fallback_xy: list[float]
) -> dict[int, list[tuple[float, float]]]:
    """Parse [zone_id, x, y, ...], falling back to one legacy anchor per zone."""
    candidates: dict[int, list[tuple[float, float]]] = {}
    for i in range(0, len(values) - 2, 3):
        zone_id = int(round(values[i]))
        if zone_id > 0:
            candidates.setdefault(zone_id, []).append((values[i + 1], values[i + 2]))
    if candidates:
        return candidates
    for i in range(0, len(fallback_xy) - 1, 2):
        candidates[i // 2 + 1] = [(fallback_xy[i], fallback_xy[i + 1])]
    return candidates


def _nearest_zone_anchor(
    candidates: list[tuple[float, float]], robot_xy: tuple[float, float]
) -> tuple[int, tuple[float, float]]:
    """Return the candidate index and point nearest the robot at zone entry."""
    return min(
        enumerate(candidates),
        key=lambda item: math.hypot(item[1][0] - robot_xy[0], item[1][1] - robot_xy[1]),
    )


def _opening_zone_entry_waypoints(
    robot_xy: tuple[float, float],
    anchor_xy: tuple[float, float],
    axis_tolerance_m: float,
) -> list[tuple[float, float]]:
    """Build the post-opening route: correct field x first, then drive along field y."""
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    ax, ay = float(anchor_xy[0]), float(anchor_xy[1])
    waypoints: list[tuple[float, float]] = []
    if abs(ax - rx) > max(0.0, float(axis_tolerance_m)):
        waypoints.append((ax, ry))
    if not waypoints or math.hypot(ax - waypoints[-1][0], ay - waypoints[-1][1]) > 1e-9:
        waypoints.append((ax, ay))
    return waypoints


def _axis_strafe_zone_entry_waypoints(
    robot_xy: tuple[float, float],
    anchor_xy: tuple[float, float],
    strafe_axis: str,
    axis_tolerance_m: float,
) -> list[tuple[float, float]]:
    """Correct one field axis by strafing, then approach the anchor straight forward."""
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    ax, ay = float(anchor_xy[0]), float(anchor_xy[1])
    tolerance = max(0.0, float(axis_tolerance_m))
    waypoints: list[tuple[float, float]] = []
    if strafe_axis == "x":
        if abs(ax - rx) > tolerance:
            waypoints.append((ax, ry))
    elif strafe_axis == "y":
        if abs(ay - ry) > tolerance:
            waypoints.append((rx, ay))
    else:
        raise ValueError(f"unsupported zone-entry strafe axis: {strafe_axis}")
    if not waypoints or math.hypot(ax - waypoints[-1][0], ay - waypoints[-1][1]) > 1e-9:
        waypoints.append((ax, ay))
    return waypoints


def _zone_transition_strafe_profile(
    from_zone: int,
    to_zone: int,
) -> tuple[str, float] | None:
    """Return (strafe axis, forward heading) for the fixed clockwise zone transitions."""
    return {
        (1, 2): ("x", -math.pi / 2.0),
        (2, 3): ("y", 0.0),
        (3, 4): ("x", math.pi / 2.0),
    }.get((int(from_zone), int(to_zone)))


def _zone_anchor_strafe_velocity(
    field_axis_error: float,
    robot_heading: float,
    strafe_axis: str,
    duty: float,
) -> float:
    """Return body-left velocity whose field projection reduces the selected axis error."""
    if strafe_axis == "x":
        body_left_axis = -math.sin(float(robot_heading))
    elif strafe_axis == "y":
        body_left_axis = math.cos(float(robot_heading))
    else:
        raise ValueError(f"unsupported zone-entry strafe axis: {strafe_axis}")
    projected_error = float(field_axis_error) * body_left_axis
    if abs(projected_error) <= 1e-9 or duty <= 0.0:
        return 0.0
    return math.copysign(abs(float(duty)), projected_error)


def _zone_anchor_strafe_pulse_duration(
    axis_error_m: float,
    long_threshold_m: float,
    medium_threshold_m: float,
    long_pulse_sec: float,
    medium_pulse_sec: float,
    short_pulse_sec: float,
) -> float:
    """Choose one fixed pulse duration from the remaining anchor-axis error."""
    error_m = abs(float(axis_error_m))
    if error_m >= float(long_threshold_m):
        return max(0.03, float(long_pulse_sec))
    if error_m >= float(medium_threshold_m):
        return max(0.03, float(medium_pulse_sec))
    return max(0.03, float(short_pulse_sec))


def _opening_anchor_strafe_velocity(
    field_x_error: float,
    robot_heading: float,
    duty: float,
) -> float:
    """Return a pure body-left command that reduces field-x error."""
    return _zone_anchor_strafe_velocity(field_x_error, robot_heading, "x", duty)


def _zone_anchor_slowdown_state(
    distance_m: float | None,
    enabled: bool,
    slowdown_distance_m: float,
    slow_speed: float,
    latched: bool,
) -> tuple[bool, float | None]:
    """Latch a low-speed cap near a zone anchor until that anchor visit ends."""
    active = bool(latched)
    if (
        not active
        and enabled
        and distance_m is not None
        and distance_m <= max(0.0, float(slowdown_distance_m))
    ):
        active = True
    if not active:
        return False, None
    return True, max(0.0, float(slow_speed))


def _cardinal_goal_reached_or_passed(
    start_xy: tuple[float, float],
    robot_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    tolerance_m: float,
) -> tuple[bool, bool]:
    """Return (reached, passed) so a cardinal segment cannot drive beyond its goal line."""
    sx, sy = float(start_xy[0]), float(start_xy[1])
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    if math.hypot(gx - rx, gy - ry) <= max(0.0, float(tolerance_m)):
        return True, False
    dx, dy = gx - sx, gy - sy
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return True, False
    passed = (rx - sx) * dx + (ry - sy) * dy >= length_sq
    return passed, passed


def _cardinal_waypoint_status(
    start_xy: tuple[float, float],
    robot_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    reach_tolerance_m: float,
    pass_lateral_tolerance_m: float,
) -> tuple[bool, bool, float]:
    """Return (reached, passed, lateral_error) for a cardinal lane waypoint."""
    sx, sy = float(start_xy[0]), float(start_xy[1])
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    distance = math.hypot(gx - rx, gy - ry)
    if distance <= max(0.0, float(reach_tolerance_m)):
        return True, False, 0.0

    dx, dy = gx - sx, gy - sy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return True, False, 0.0

    ux, uy = dx / length, dy / length
    along = (rx - sx) * ux + (ry - sy) * uy
    lateral_error = abs((rx - sx) * uy - (ry - sy) * ux)
    passed = along >= length
    reached = passed and lateral_error <= max(
        0.0, float(pass_lateral_tolerance_m)
    )
    return reached, passed, lateral_error


def lane_route_replan_required(
    plan: list[tuple[float, float]] | None,
    route_mode_changed: bool,
    drifted: bool,
    stale: bool,
    now_s: float,
    last_replan_s: float,
    throttle_sec: float,
) -> bool:
    """Return whether the lane planner must run on this control tick.

    ``None`` means no route has been attempted yet and is planned immediately.  An empty list
    means the previous attempt found no safe route; unlike a valid latched route, it must be
    retried after the normal throttle so a transient obstacle layout cannot hold SCAN forever.
    """
    if plan is None or route_mode_changed:
        return True
    retry_ready = float(now_s) - float(last_replan_s) >= max(
        0.0, float(throttle_sec)
    )
    if not plan:
        return retry_ready
    return bool(drifted or stale) and retry_ready


def global_target_in_route_corridor(
    target_xy: tuple[float, float],
    segment_start_xy: tuple[float, float],
    segment_end_xy: tuple[float, float],
    corridor_radius_m: float,
) -> bool:
    """Return whether a target is close enough to interrupt the active sweep leg."""
    px, py = float(target_xy[0]), float(target_xy[1])
    ax, ay = float(segment_start_xy[0]), float(segment_start_xy[1])
    bx, by = float(segment_end_xy[0]), float(segment_end_xy[1])
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        distance = math.hypot(px - ax, py - ay)
    else:
        projection = ((px - ax) * dx + (py - ay) * dy) / length_sq
        projection = max(0.0, min(1.0, projection))
        nearest_x = ax + projection * dx
        nearest_y = ay + projection * dy
        distance = math.hypot(px - nearest_x, py - nearest_y)
    return distance <= max(0.0, float(corridor_radius_m))


def holonomic_waypoint_velocity(
    base_x: float,
    base_y: float,
    speed_mps: float,
    stop_radius_m: float,
) -> tuple[float, float]:
    """Translate toward a base-frame waypoint without changing robot heading."""
    distance = math.hypot(float(base_x), float(base_y))
    if distance <= max(0.0, float(stop_radius_m)) or distance <= 1e-9:
        return 0.0, 0.0
    speed = max(0.0, float(speed_mps))
    scale = speed / distance
    return float(base_x) * scale, float(base_y) * scale


def radial_object_standoff(
    robot_xy: tuple[float, float],
    target_xy: tuple[float, float],
    standoff_m: float,
) -> tuple[float, float]:
    """Return the point on the robot-to-target line at the requested target distance."""
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    tx, ty = float(target_xy[0]), float(target_xy[1])
    dx, dy = tx - rx, ty - ry
    distance = math.hypot(dx, dy)
    stand = max(0.0, float(standoff_m))
    if distance <= stand or distance <= 1e-9:
        return rx, ry
    scale = (distance - stand) / distance
    return rx + dx * scale, ry + dy * scale


def heading_control_command(
    current_heading: float,
    target_heading: float,
    tolerance_rad: float,
    kp: float,
    omega_max: float,
) -> tuple[bool, float]:
    """Return closed-loop heading completion and omega command from actual robot heading."""
    error = math.atan2(
        math.sin(float(target_heading) - float(current_heading)),
        math.cos(float(target_heading) - float(current_heading)),
    )
    if abs(error) <= max(0.0, float(tolerance_rad)):
        return True, 0.0
    limit = max(0.0, float(omega_max))
    omega = max(-limit, min(limit, max(0.0, float(kp)) * error))
    return False, omega


def align_heading_error(
    target_x: float,
    target_y: float,
    grab_y: float,
) -> float:
    """Heading change that puts a target on the straight line through ``grab_y``.

    A subsequent forward-only move preserves lateral ``y``.  Therefore the facing target is not
    the final grab-point bearing; it is the bearing whose lateral component at the current range
    equals ``grab_y``.
    """
    radius = math.hypot(float(target_x), float(target_y))
    if radius <= 1e-9:
        return 0.0
    desired_bearing = math.asin(max(-1.0, min(1.0, float(grab_y) / radius)))
    observed_bearing = math.atan2(float(target_y), float(target_x))
    return math.atan2(
        math.sin(observed_bearing - desired_bearing),
        math.cos(observed_bearing - desired_bearing),
    )


def nearest_align_wide_point(
    expected: tuple[float, float] | None,
    observations: list[tuple[float, float]],
    match_radius_m: float,
) -> tuple[float, float] | None:
    """Associate a raw Wide observation to the latched target geometry.

    ``expected`` may come from the fused world/slot model, but the returned control point is
    always one of the raw Wide base-frame observations.  This prevents Body homography lateral
    error from leaking back into ALIGN yaw through a fused world track.
    """
    if expected is None:
        return None
    ex, ey = float(expected[0]), float(expected[1])
    best: tuple[float, float] | None = None
    best_distance = max(0.0, float(match_radius_m))
    for ox, oy in observations:
        distance = math.hypot(float(ox) - ex, float(oy) - ey)
        if distance <= best_distance:
            best_distance = distance
            best = (float(ox), float(oy))
    return best


def nearest_align_wide_observation(
    expected: tuple[float, float] | None,
    observations: list[RelativeObservation],
    match_radius_m: float,
) -> RelativeObservation | None:
    """Associate one labelled raw-Wide observation with the latched target geometry."""
    if expected is None:
        return None
    ex, ey = float(expected[0]), float(expected[1])
    best: RelativeObservation | None = None
    best_distance = max(0.0, float(match_radius_m))
    for observation in observations:
        distance = math.hypot(float(observation.x) - ex, float(observation.y) - ey)
        if distance <= best_distance:
            best_distance = distance
            best = observation
    return best


def split_align_control_points(
    wide_point: tuple[float, float] | None,
    body_point: tuple[float, float] | None,
) -> tuple[tuple[float, float] | None, tuple[float, float] | None]:
    """Return independent ``(yaw_point, range_point)`` for ALIGN.

    Yaw has no Body fallback. Range uses Body once available and otherwise uses Wide only for
    acquisition. This explicit split makes it impossible for Body y to steer the base.
    """
    if wide_point is None:
        return None, None
    return wide_point, body_point if body_point is not None else wide_point


def align_translation_axis(
    forward_error_m: float,
    lateral_error_m: float,
    forward_tolerance_m: float,
    lateral_tolerance_m: float,
) -> str | None:
    """Select the worst out-of-tolerance Body axis for the next ALIGN translation pulse."""
    forward_out = abs(float(forward_error_m)) - max(0.0, float(forward_tolerance_m))
    lateral_out = abs(float(lateral_error_m)) - max(0.0, float(lateral_tolerance_m))
    if forward_out <= 0.0 and lateral_out <= 0.0:
        return None
    if lateral_out > 0.0 and (forward_out <= 0.0 or lateral_out >= forward_out):
        return "lateral"
    return "forward"


def circular_heading_filter(previous: float | None, raw_heading: float, alpha: float) -> float:
    """Update a wrap-safe heading EMA without averaging across the +/-pi discontinuity."""
    raw = math.atan2(math.sin(float(raw_heading)), math.cos(float(raw_heading)))
    if previous is None:
        return raw
    weight = max(0.0, min(1.0, float(alpha)))
    delta = math.atan2(math.sin(raw - previous), math.cos(raw - previous))
    updated = float(previous) + weight * delta
    return math.atan2(math.sin(updated), math.cos(updated))


def lane_heading_violation_time_step(
    heading_error: float,
    tolerance_rad: float,
    violation_start_s: float | None,
    now_s: float,
    required_sec: float,
) -> tuple[float | None, bool]:
    """Require a continuous heading violation before requesting realignment."""
    if abs(heading_error) <= max(0.0, tolerance_rad):
        return None, False
    start_s = float(now_s) if violation_start_s is None else float(violation_start_s)
    return start_s, float(now_s) - start_s >= max(0.0, float(required_sec))


def lane_cross_track_feedback(
    start: tuple[float, float],
    dest: tuple[float, float],
    robot_xy: tuple[float, float],
    center_band_half_width_m: float,
    lookahead_m: float,
    gain: float,
    max_heading_bias_rad: float,
) -> tuple[float, float, float]:
    """Return signed lane error, error outside the dead band, and steering bias.

    Positive cross-track error is to the left of travel.  The returned heading bias therefore
    has the opposite sign so a robot that drifts left steers smoothly back to the right.  Error
    inside the configured centre band produces no lane-return command.
    """
    sx, sy = float(start[0]), float(start[1])
    dx = float(dest[0]) - sx
    dy = float(dest[1]) - sy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 0.0, 0.0, 0.0
    cross_track_error = (
        (float(robot_xy[0]) - sx) * (-dy)
        + (float(robot_xy[1]) - sy) * dx
    ) / length
    band = max(0.0, float(center_band_half_width_m))
    effective_error = math.copysign(
        max(0.0, abs(cross_track_error) - band), cross_track_error
    )
    heading_bias = -math.atan2(
        max(0.0, float(gain)) * effective_error,
        max(1e-3, float(lookahead_m)),
    )
    limit = max(0.0, float(max_heading_bias_rad))
    heading_bias = max(-limit, min(limit, heading_bias))
    return cross_track_error, effective_error, heading_bias


def timed_storage_due(
    *,
    run_started: bool,
    triggered: bool,
    completed: bool,
    now_s: float,
    run_start_s: float,
    trigger_sec: float,
    dynamic_enabled: bool = False,
    match_duration_sec: float = 180.0,
    route_distance_m: float | None = None,
    effective_speed_mps: float = 0.10,
    fixed_overhead_sec: float = 12.0,
    safety_margin_sec: float = 10.0,
    distance_safety_factor: float = 1.0,
) -> bool:
    """Return whether fixed-latest or distance-based RUNNING storage time is due."""
    fixed_due = bool(
        run_started
        and not triggered
        and not completed
        and trigger_sec > 0.0
        and float(now_s) - float(run_start_s) >= float(trigger_sec)
    )
    if fixed_due:
        return True
    if (
        not dynamic_enabled
        or not run_started
        or triggered
        or completed
        or route_distance_m is None
        or effective_speed_mps <= 0.0
        or match_duration_sec <= 0.0
    ):
        return False
    elapsed_sec = max(0.0, float(now_s) - float(run_start_s))
    remaining_sec = float(match_duration_sec) - elapsed_sec
    required_sec = (
        max(0.0, float(route_distance_m))
        * max(1.0, float(distance_safety_factor))
        / float(effective_speed_mps)
        + max(0.0, float(fixed_overhead_sec))
        + max(0.0, float(safety_margin_sec))
    )
    return remaining_sec <= required_sec + 1e-6


def arm_pick_lift_started(
    phase: str,
    *,
    phase_rx_s: float,
    pick_enter_s: float,
    now_s: float,
    max_age_sec: float,
) -> bool:
    """Return true once the current pick has reached LIFT or a later arm step."""
    return bool(
        str(phase).strip().upper() in {"LIFT", "TO_PLACE", "PLACE", "STOW", "COMPLETE"}
        and float(phase_rx_s) + 1e-6 >= float(pick_enter_s)
        and float(now_s) - float(phase_rx_s) <= max(0.0, float(max_age_sec))
    )


def directional_wall_distance(
    segments: list[tuple[float, float, float, float]],
    robot_x: float,
    robot_y: float,
    *,
    axis: str,
    direction: int,
    angle_tolerance_rad: float = math.radians(35.0),
) -> float | None:
    """Return the nearest observed axis wall in the requested field direction."""
    distances: list[float] = []
    sign = 1.0 if int(direction) >= 0 else -1.0
    for x0, y0, x1, y1 in segments:
        dx, dy = float(x1) - float(x0), float(y1) - float(y0)
        if math.hypot(dx, dy) < 0.12:
            continue
        angle = math.atan2(dy, dx) % math.pi
        horizontal = min(angle, math.pi - angle) <= abs(float(angle_tolerance_rad))
        if axis == "y":
            if not horizontal:
                continue
            delta = sign * (0.5 * (float(y0) + float(y1)) - float(robot_y))
        elif axis == "x":
            if horizontal:
                continue
            delta = sign * (0.5 * (float(x0) + float(x1)) - float(robot_x))
        else:
            raise ValueError(f"unsupported wall axis: {axis}")
        if delta >= 0.0:
            distances.append(delta)
    return min(distances) if distances else None


def parking_face_heading(
    staging_x: float,
    staging_y: float,
    face_x: float,
    face_y: float,
) -> float:
    """Heading from the parking staging point toward the requested face point."""
    return math.atan2(float(face_y) - float(staging_y), float(face_x) - float(staging_x))


def straight_forward_command(
    *,
    current_heading: float,
    target_heading: float,
    distance_m: float,
    max_speed: float,
    heading_kp: float,
    omega_max: float,
) -> tuple[float, float, float]:
    """Return a forward-only command with zero lateral translation and heading hold."""
    error = math.atan2(
        math.sin(float(target_heading) - float(current_heading)),
        math.cos(float(target_heading) - float(current_heading)),
    )
    speed = min(abs(float(max_speed)), max(0.04, 0.5 * max(0.0, float(distance_m))))
    omega_limit = abs(float(omega_max))
    omega = max(-omega_limit, min(omega_limit, max(0.0, float(heading_kp)) * error))
    return speed, 0.0, omega


def straight_reverse_command(
    *,
    current_heading: float,
    target_heading: float,
    distance_m: float,
    max_speed: float,
    heading_kp: float,
    omega_max: float,
) -> tuple[float, float, float]:
    """Return a reverse-only command with zero lateral translation and heading hold."""
    error = math.atan2(
        math.sin(float(target_heading) - float(current_heading)),
        math.cos(float(target_heading) - float(current_heading)),
    )
    speed = min(abs(float(max_speed)), max(0.04, 0.5 * max(0.0, float(distance_m))))
    omega_limit = abs(float(omega_max))
    omega = max(-omega_limit, min(omega_limit, max(0.0, float(heading_kp)) * error))
    return -speed, 0.0, omega


class PulsedHeadingController:
    """Closed-loop pulse/settle controller for torque-stable in-place turns."""

    def __init__(
        self,
        *,
        pulse_omega: float = 0.10,
        slowdown_rad: float = math.radians(10.0),
        continuous_min_error_rad: float = math.radians(40.0),
        coarse_pulse_sec: float = 0.24,
        coarse_settle_sec: float = 0.18,
        verify_sec: float = 0.60,
        fine_pulse_sec: float = 0.10,
        fine_settle_sec: float = 0.35,
        max_pulses: int = 60,
        timeout_sec: float = 30.0,
    ) -> None:
        self.pulse_omega = abs(float(pulse_omega))
        self.slowdown_rad = abs(float(slowdown_rad))
        self.continuous_min_error_rad = max(
            self.slowdown_rad, abs(float(continuous_min_error_rad))
        )
        self.coarse_pulse_sec = max(0.0, float(coarse_pulse_sec))
        self.coarse_settle_sec = max(0.0, float(coarse_settle_sec))
        self.verify_sec = max(0.0, float(verify_sec))
        self.fine_pulse_sec = max(0.0, float(fine_pulse_sec))
        self.fine_settle_sec = max(0.0, float(fine_settle_sec))
        self.max_pulses = max(1, int(max_pulses))
        self.timeout_sec = max(0.0, float(timeout_sec))
        self.reset()

    def reset(self) -> None:
        self.key = None
        self.target_heading = 0.0
        self.phase = "measure"
        self.phase_start_s = 0.0
        self.turn_start_s = 0.0
        self.pulse_sign = 0.0
        self.pulse_duration_sec = 0.0
        self.settle_duration_sec = 0.0
        self.pulse_count = 0

    @staticmethod
    def _error(target_heading: float, current_heading: float) -> float:
        return math.atan2(
            math.sin(float(target_heading) - float(current_heading)),
            math.cos(float(target_heading) - float(current_heading)),
        )

    def step(
        self,
        *,
        now_s: float,
        current_heading: float,
        target_heading: float,
        tolerance_rad: float,
        key: object,
        omega_limit: float | None = None,
    ) -> tuple[bool, float, str | None]:
        """Return ``(aligned, omega, event)``; translation is always owned by the caller."""
        now = float(now_s)
        if key != self.key:
            self.reset()
            self.key = key
            self.target_heading = math.atan2(
                math.sin(float(target_heading)), math.cos(float(target_heading))
            )
            self.turn_start_s = now

        error = self._error(self.target_heading, current_heading)
        tolerance = max(0.0, float(tolerance_rad))
        omega = self.pulse_omega
        if omega_limit is not None:
            omega = min(omega, max(0.0, abs(float(omega_limit))))

        if self.phase == "hold":
            return False, 0.0, None

        if self.phase == "continuous":
            timed_out = (
                self.timeout_sec > 0.0
                and now - self.turn_start_s >= self.timeout_sec
            )
            crossed_target = error * self.pulse_sign <= 0.0
            if abs(error) <= self.slowdown_rad or crossed_target:
                self.phase = "verify"
                self.phase_start_s = now
                return False, 0.0, "continuous_settle"
            if timed_out or omega <= 0.0:
                self.phase = "hold"
                return False, 0.0, "timeout"
            return False, self.pulse_sign * omega, None

        if self.phase == "pulse":
            if now - self.phase_start_s < self.pulse_duration_sec:
                return False, self.pulse_sign * omega, None
            self.phase = "settle"
            self.phase_start_s = now
            return False, 0.0, "settle"

        if self.phase == "settle":
            if now - self.phase_start_s >= self.settle_duration_sec:
                self.phase = "measure"
            return False, 0.0, None

        if self.phase == "verify":
            if now - self.phase_start_s < self.verify_sec:
                return False, 0.0, None
            if abs(error) <= tolerance:
                self.reset()
                return True, 0.0, "complete"
            timed_out = self.timeout_sec > 0.0 and now - self.turn_start_s >= self.timeout_sec
            if self.pulse_count >= self.max_pulses or timed_out or omega <= 0.0:
                self.phase = "hold"
                return False, 0.0, "timeout"
            self.pulse_count += 1
            self.pulse_sign = math.copysign(1.0, error)
            self.pulse_duration_sec = self.fine_pulse_sec
            self.settle_duration_sec = self.fine_settle_sec
            self.phase = "pulse"
            self.phase_start_s = now
            return False, 0.0, "fine_pulse"

        if abs(error) <= tolerance:
            self.reset()
            return True, 0.0, "complete"
        timed_out = self.timeout_sec > 0.0 and now - self.turn_start_s >= self.timeout_sec
        if self.pulse_count >= self.max_pulses or timed_out or omega <= 0.0:
            self.phase = "hold"
            return False, 0.0, "timeout"
        if abs(error) <= self.slowdown_rad:
            self.phase = "verify"
            self.phase_start_s = now
            return False, 0.0, "verify"

        if abs(error) >= self.continuous_min_error_rad:
            self.pulse_count += 1
            self.pulse_sign = math.copysign(1.0, error)
            self.phase = "continuous"
            self.phase_start_s = now
            return False, self.pulse_sign * omega, "continuous_turn"

        self.pulse_count += 1
        self.pulse_sign = math.copysign(1.0, error)
        self.pulse_duration_sec = self.coarse_pulse_sec
        self.settle_duration_sec = self.coarse_settle_sec
        self.phase = "pulse"
        self.phase_start_s = now
        return False, 0.0, "coarse_pulse"


COMPETITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


STATES = [
    "OPENING", "WALL_INIT", "SCAN", "ANCHOR_OBSERVE", "SELECT_TARGET", "ANCHOR_FACE_TARGET",
    "APPROACH", "ALIGN", "CLASSIFY", "WAIT_FOR_ARM", "PICK", "STORE_IN_TRAY", "ANCHOR_RETURN",
    "ANCHOR_FACE_NEXT", LOCAL_ANCHOR_STATE, "ZONE_STABILIZE", "WAIT_FOR_STORAGE",
    "DRIVE_TO_STORAGE", "ALIGN_OVER_BIN", "DUMP_ALL", "END",
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
        self.declare_parameter("classify_body_max_age_sec", 0.60)
        # Body SigLIP takes close to one second on Jetson. Bind the answer to the selected
        # field target at the image capture pose so ALIGN motion cannot invalidate a correct read.
        self.declare_parameter("set2_identity_latch_max_age_sec", 20.0)
        self.declare_parameter("set2_identity_pose_max_skew_sec", 0.20)
        self.declare_parameter("set2_identity_max_inference_delay_sec", 3.0)
        self.declare_parameter("set2_committed_cube_pick_enabled", True)
        self.declare_parameter("classify_body_settle_sec", 0.40)
        self.declare_parameter("classify_body_max_frames", 10)
        self.declare_parameter("classify_body_retry_backoff_sec", 0.25)
        self.declare_parameter("classify_body_retry_settle_sec", 0.40)
        # At very close range the Body model can mistake an octahedron for a cube. Only for an
        # octahedron target, use the spatially associated raw-Wide label for final identity while
        # retaining Body solely as the fresh object-at-grab presence/range safety gate.
        self.declare_parameter("classify_octa_wide_final_enabled", True)
        self.declare_parameter("classify_wide_confirm_frames", 3)
        # APPROACH owns target heading. ALIGN keeps that heading fixed and uses Body x/y only for
        # calibrated forward/back and mecanum lateral translation pulses.
        self.declare_parameter("grab_x", 0.264)
        self.declare_parameter("grab_y", 0.007)
        self.declare_parameter("grab_min_x", 0.255)     # never push the target closer than this
        self.declare_parameter("align_tol_m", 0.02)
        self.declare_parameter("align_fwd_tol_m", 0.08)  # Body-x forward/back tolerance
        self.declare_parameter("align_body_confirm_frames", 3)
        self.declare_parameter("align_wide_match_radius_m", 0.30)
        self.declare_parameter("align_wide_max_age_sec", 0.80)
        self.declare_parameter("global_body_target_match_radius_m", 0.18)
        # A frozen global route may outlive a short-lived world track. At stand-off, abandon
        # it quickly only when neither a same-position world track nor a matching Body object
        # remains; this avoids spending the full ALIGN timeout staring at a neighbouring fruit.
        self.declare_parameter("global_target_absent_timeout_sec", 0.60)
        self.declare_parameter("global_target_absent_confirm_frames", 3)
        self.declare_parameter("align_body_lost_confirm_sec", 0.40)
        self.declare_parameter("align_body_lost_confirm_frames", 3)
        self.declare_parameter("align_body_lost_max_backoffs", 1)
        self.declare_parameter("align_body_lost_reselect_cooldown_sec", 6.0)
        self.declare_parameter("align_kp", 0.6)         # m/s per m of error
        self.declare_parameter("align_vmax", 0.16)      # cap
        # STICTION: the heavy base won't move below ~this speed (motor just buzzes), so any nonzero
        # servo command is boosted to at least this. Bigger tol above absorbs the coarser steps.
        self.declare_parameter("align_vmin", 0.13)
        self.declare_parameter("align_timeout_sec", 12.0)
        self.declare_parameter("max_align_fails", 3)   # consecutive ALIGN timeouts on an object -> blacklist it
        self.declare_parameter("align_distractor_confirm_frames", 5)
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
        self.declare_parameter("align_step_strafe_duty", 0.315)
        self.declare_parameter("align_step_strafe_sec", 0.30)
        self.declare_parameter("align_heading_tolerance_rad", math.radians(3.0))
        self.declare_parameter("align_adaptive_steps_enabled", False)
        self.declare_parameter("align_mid_error_m", 0.06)
        self.declare_parameter("align_step_fwd_mid_sec", 0.24)
        self.declare_parameter("align_step_strafe_mid_sec", 0.60)
        # If the target remains in the wide/world map but drops out of the body cam at ALIGN,
        # back up once so the body cam can reacquire it instead of waiting stationary for timeout.
        self.declare_parameter("align_body_lost_backoff_enabled", True)
        self.declare_parameter("align_body_lost_backoff_speed", 0.189)
        self.declare_parameter("align_body_lost_backoff_sec", 0.54)
        self.declare_parameter("align_body_lost_backoff_settle_sec", 1.0)
        self.declare_parameter("align_body_lost_lateral_sweep_enabled", True)
        self.declare_parameter("align_body_lost_lateral_sweep_duty", 0.315)
        self.declare_parameter("align_body_lost_lateral_sweep_sec", 1.2)
        self.declare_parameter("align_body_lost_lateral_sweep_settle_sec", 0.4)
        self.declare_parameter("align_body_lost_same_slot_retry_enabled", True)
        self.declare_parameter("align_body_lost_same_slot_retries", 1)
        self.declare_parameter("align_retry_heading_tol_rad", 0.10)
        self.declare_parameter("align_retry_heading_timeout_sec", 2.0)
        self.declare_parameter("pick_duration_sec", 3.0)
        self.declare_parameter("pick_release_on_lift_enabled", True)
        self.declare_parameter("arm_pick_phase_max_age_sec", 0.50)
        self.declare_parameter("storage_x", 0.2)
        self.declare_parameter("storage_y", 0.2)
        self.declare_parameter("timed_storage_enabled", False)
        self.declare_parameter("timed_storage_start_sec", 150.0)
        self.declare_parameter("timed_storage_dynamic_enabled", True)
        self.declare_parameter("timed_storage_match_duration_sec", 180.0)
        self.declare_parameter("timed_storage_effective_speed_mps", 0.10)
        self.declare_parameter("timed_storage_fixed_overhead_sec", 12.0)
        self.declare_parameter("timed_storage_safety_margin_sec", 10.0)
        self.declare_parameter("timed_storage_distance_safety_factor", 1.20)
        self.declare_parameter("storage_wall_guided_enabled", False)
        self.declare_parameter("wall_initialization_enabled", True)
        self.declare_parameter("wall_initialization_min_observe_sec", 1.5)
        self.declare_parameter("wall_initialization_confirm_frames", 3)
        self.declare_parameter("wall_initialization_settle_sec", 0.5)
        self.declare_parameter("wall_initialization_timeout_sec", 4.0)
        self.declare_parameter("storage_wall_warmup_sec", 0.8)
        self.declare_parameter("storage_wall_warmup_timeout_sec", 2.0)
        self.declare_parameter("storage_reverse_entry_enabled", False)
        self.declare_parameter("storage_fast_return_enabled", False)
        self.declare_parameter("storage_fast_nav_speed", 0.20)
        self.declare_parameter("storage_fast_reverse_speed", 0.16)
        self.declare_parameter("storage_fast_heading_tolerance_rad", math.radians(15.0))
        self.declare_parameter("storage_final_heading_rad", math.pi / 4.0)
        self.declare_parameter("storage_staging_x", 1.5)
        self.declare_parameter("storage_staging_y", 1.5)
        self.declare_parameter("storage_face_x", 0.0)
        self.declare_parameter("storage_face_y", 0.0)
        self.declare_parameter("storage_staging_reach_tol_m", 0.10)
        self.declare_parameter("storage_heading_tolerance_rad", 0.06)
        self.declare_parameter("storage_reverse_speed", 0.08)
        self.declare_parameter("storage_reverse_heading_kp", 1.0)
        self.declare_parameter("storage_reverse_omega_max", 0.06)
        self.declare_parameter("storage_reverse_realign_rad", 0.1745)
        self.declare_parameter("storage_reach_tol_m", 0.08)
        self.declare_parameter("storage_down_heading_rad", math.pi / 2.0)
        self.declare_parameter("storage_right_heading_rad", math.pi)
        self.declare_parameter("storage_bottom_wall_stop_m", 0.30)
        self.declare_parameter("storage_left_wall_stop_m", 0.20)
        self.declare_parameter("storage_wall_approach_speed", 0.08)
        self.declare_parameter("storage_wall_confirm_frames", 3)
        self.declare_parameter("storage_wall_max_age_sec", 1.50)
        self.declare_parameter("storage_wall_detection_required_m", 0.60)
        self.declare_parameter("storage_pose_fallback_enabled", False)
        self.declare_parameter("storage_pose_fallback_hold_sec", 0.8)
        self.declare_parameter("storage_pose_fallback_contact_speed", 0.06)
        self.declare_parameter("storage_last_chance_enabled", False)
        self.declare_parameter("storage_last_chance_trigger_sec", 145.0)
        self.declare_parameter("storage_last_chance_pre_backoff_m", 0.05)
        self.declare_parameter("storage_last_chance_pre_backoff_speed", 0.10)
        self.declare_parameter("storage_last_chance_turn_left_deg", 95.0)
        self.declare_parameter("storage_last_chance_reverse_sec", 5.0)
        self.declare_parameter("storage_last_chance_alternate_sec", 2.0)
        self.declare_parameter("storage_last_chance_force_dump_after_sec", 12.0)
        self.declare_parameter("storage_last_chance_speed", 0.20)
        self.declare_parameter("storage_contact_hold_enabled", True)
        self.declare_parameter("storage_contact_hold_speed", 0.08)
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
        # Move forward, strafe right, then turn clockwise by a relative 45 degrees. No post-opening
        # pose is injected: stationary wall observations establish heading first and x/y second.
        # Mapping and object-flow start only after both wall estimates have converged.
        # Timed through /base_command so the base_controller's start-boost + stop-brake apply.
        self.declare_parameter("opening_enabled", True)
        self.declare_parameter("opening_wall_validation_enabled", True)
        self.declare_parameter("opening_explore_while_moving", False)
        self.declare_parameter("startup_warmup_sec", 0.0)
        self.declare_parameter("opening_speed", 0.35)        # >= wheel_min so it's the actual speed
        self.declare_parameter("opening_forward_sec", 1.0)
        self.declare_parameter("opening_strafe_speed", 0.20)
        self.declare_parameter("opening_strafe_right_sec", 1.0)
        self.declare_parameter("opening_turn_deg", 45.0)
        self.declare_parameter("opening_turn_omega", -0.17)   # negative = CW, positive = CCW
        self.declare_parameter("opening_turn_tolerance_rad", 0.06)
        self.declare_parameter("opening_turn_timeout_sec", 7.0)
        self.declare_parameter("opening_wait_after_turn_sec", 1.0)
        self.declare_parameter("opening_release_at_sec", 0.0)  # if >0, do not start SCAN before this match time
        self.declare_parameter("opening_heading_observe_sec", 1.2)
        self.declare_parameter("opening_wall_heading_tolerance_rad", 0.035)
        self.declare_parameter("opening_wall_heading_max_stddev_rad", 0.0873)
        self.declare_parameter("opening_wall_heading_min_segments", 2)
        self.declare_parameter("opening_heading_stable_frames", 3)
        self.declare_parameter("opening_position_observe_sec", 0.8)
        self.declare_parameter("opening_wall_position_tolerance_m", 0.04)
        self.declare_parameter("opening_wall_position_min_confidence", 0.90)
        self.declare_parameter("opening_position_stable_frames", 3)
        self.declare_parameter("opening_anchor_strafe_duty", 0.315)
        self.declare_parameter("opening_anchor_strafe_long_threshold_m", 0.30)
        self.declare_parameter("opening_anchor_strafe_medium_threshold_m", 0.15)
        self.declare_parameter("opening_anchor_strafe_long_pulse_sec", 1.0)
        self.declare_parameter("opening_anchor_strafe_medium_pulse_sec", 0.7)
        self.declare_parameter("opening_anchor_strafe_short_pulse_sec", 0.4)
        self.declare_parameter("opening_anchor_strafe_settle_sec", 0.60)
        self.declare_parameter("opening_anchor_strafe_max_pulses", 8)
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
        # Local-anchor-proven in-place turn profile: pulse, stop, remeasure, then pulse again.
        self.declare_parameter("nav_turn_pulse_omega", 0.10)
        self.declare_parameter("nav_turn_slowdown_rad", math.radians(10.0))
        self.declare_parameter("nav_turn_continuous_min_error_rad", math.radians(40.0))
        self.declare_parameter("nav_turn_coarse_pulse_sec", 0.24)
        self.declare_parameter("nav_turn_coarse_settle_sec", 0.18)
        self.declare_parameter("nav_turn_verify_sec", 0.60)
        self.declare_parameter("nav_turn_fine_pulse_sec", 0.10)
        self.declare_parameter("nav_turn_fine_settle_sec", 0.35)
        self.declare_parameter("nav_turn_max_pulses", 60)
        self.declare_parameter("nav_turn_timeout_sec", 30.0)
        self.declare_parameter("lane_heading_lock_enabled", True)
        self.declare_parameter("lane_heading_initial_align_enabled", False)
        self.declare_parameter("lane_heading_axis_tolerance_m", 0.06)
        self.declare_parameter("lane_heading_align_tolerance_rad", 0.0349)
        self.declare_parameter("lane_heading_realign_tolerance_rad", 0.1745)
        self.declare_parameter("lane_heading_realign_arm_sec", 1.2)
        self.declare_parameter("lane_heading_soft_entry_tolerance_rad", 0.1745)
        self.declare_parameter("lane_heading_soft_entry_speed", 0.07)
        self.declare_parameter("lane_heading_soft_entry_kp", 0.40)
        self.declare_parameter("lane_heading_soft_entry_omega_max", 0.04)
        self.declare_parameter("lane_heading_soft_entry_timeout_sec", 1.0)
        self.declare_parameter("lane_heading_realign_hold_sec", 0.8)
        self.declare_parameter("lane_heading_drive_omega_max", 0.03)
        self.declare_parameter("lane_heading_filter_alpha", 0.20)
        self.declare_parameter("lane_heading_settle_sec", 0.30)
        self.declare_parameter("lane_heading_reverse_settle_sec", 0.30)
        self.declare_parameter("lane_heading_kp", 1.2)
        self.declare_parameter("lane_heading_omega_max", 0.16)
        self.declare_parameter("lane_heading_deadband_rad", 0.0175)
        self.declare_parameter("lane_cross_track_enabled", True)
        self.declare_parameter("lane_center_band_half_width_m", 0.05)
        self.declare_parameter("lane_cross_track_lookahead_m", 0.55)
        self.declare_parameter("lane_cross_track_gain", 1.0)
        self.declare_parameter("lane_cross_track_filter_alpha", 0.20)
        self.declare_parameter("lane_cross_track_max_heading_bias_rad", math.radians(10.0))
        self.declare_parameter("lane_cross_track_heading_kp", 1.2)
        self.declare_parameter("lane_cross_track_slow_error_m", 0.10)
        self.declare_parameter("lane_cross_track_full_slow_error_m", 0.20)
        self.declare_parameter("lane_cross_track_min_speed_mps", 0.07)
        self.declare_parameter("stuck_escape_enabled", False)
        self.declare_parameter("stuck_detect_duration_sec", 3.0)
        self.declare_parameter("stuck_min_cmd_linear_mps", 0.06)
        self.declare_parameter("stuck_min_cmd_omega_radps", 0.10)
        self.declare_parameter("stuck_pose_delta_min_m", 0.025)
        self.declare_parameter("stuck_heading_delta_min_rad", 0.04)
        self.declare_parameter("stuck_escape_stop_sec", 0.20)
        self.declare_parameter("stuck_escape_pulse_sec", 0.35)
        self.declare_parameter("stuck_escape_settle_sec", 0.25)
        self.declare_parameter("stuck_escape_linear_speed", 0.09)
        self.declare_parameter("stuck_escape_omega_speed", 0.12)
        # When nothing is visible, DRIVE to the map centre for a better view instead of spinning in
        # place (the wide fisheye already sees all around; a central vantage just helps).
        self.declare_parameter("map_center_x", 0.0)
        self.declare_parameter("map_center_y", 0.0)
        # If the phase target isn't in view, PATROL these field waypoints (looking with the wide cam)
        # until it appears — never give up. Flattened [x0,y0,x1,y1,...]. Kept inside the wall margin.
        self.declare_parameter("patrol_waypoints",
                               [0.0, 0.0, 1.2, 1.2, 1.2, -1.2, -1.2, -1.2, -1.2, 1.2])
        self.declare_parameter("patrol_reach_tol", 0.3)   # advance to next waypoint within this
        # Main three-anchor mission.  Each anchor owns four immutable 50 cm-grid slots.  The first
        # three-second snapshot is locked; later detections may update an existing track's identity
        # but can never add another mission target to that anchor.
        self.declare_parameter("anchor_mission_enabled", False)
        self.declare_parameter("anchor_xy", [-1.25, 0.75, -1.25, -0.25, -1.25, -1.25])
        self.declare_parameter("anchor_reach_tol_m", 0.15)
        self.declare_parameter("anchor_observe_sec", 3.0)
        self.declare_parameter("anchor_birth_delay_sec", 0.30)
        self.declare_parameter("anchor_slot_match_radius_m", 0.22)
        self.declare_parameter("anchor_track_attach_radius_m", 0.24)
        self.declare_parameter("anchor_face_tol_rad", 0.10)
        # Wide owns the four immutable slots.  Body may confirm only the current slot,
        # and only for fruit cubes, Wide-uncertain objects, or today's Set1 target.
        self.declare_parameter("body_slot_wide_low_confidence", 0.70)
        self.declare_parameter("body_slot_forward_tol_m", 0.12)
        self.declare_parameter("body_slot_lateral_tol_m", 0.10)
        self.declare_parameter("body_slot_bearing_tol_deg", 10.0)
        self.declare_parameter("body_slot_unique_margin_m", 0.08)
        self.declare_parameter("body_slot_continuity_radius_m", 0.24)
        self.declare_parameter("body_slot_confirm_frames", 3)
        self.declare_parameter("body_slot_backoff_retries", 2)
        self.declare_parameter("body_slot_final_retry_enabled", True)
        # Pose-free anchor navigation.  The wide camera fixes K1's four virtual slots during a
        # stationary acquisition; K2/K3 are projected on the same one-metre local lattice.  Every
        # later waypoint is expressed in the *current* base_link frame, never field coordinates.
        self.declare_parameter("relative_anchor_nav_enabled", False)
        self.declare_parameter("relative_anchor_acquire_sec", 2.0)
        self.declare_parameter("relative_anchor_expected_k1_base", [0.55, 0.05])
        self.declare_parameter("relative_anchor_route_unit_base", [1.0, 0.0])
        self.declare_parameter("relative_anchor_step_m", 1.0)
        self.declare_parameter("relative_anchor_grid_yaw_base", 0.78539816339)
        self.declare_parameter("relative_anchor_association_radius_m", 0.22)
        self.declare_parameter("relative_anchor_cluster_radius_m", 0.10)
        self.declare_parameter("relative_anchor_stale_sec", 0.60)
        self.declare_parameter("relative_anchor_dead_reckon_sec", 12.0)
        self.declare_parameter("relative_anchor_max_capture_age_sec", 0.75)
        self.declare_parameter("relative_anchor_min_cluster_hits", 2)
        self.declare_parameter("relative_anchor_min_points", 2)
        self.declare_parameter("relative_anchor_slot_min_hits", 2)
        self.declare_parameter("relative_anchor_center_gate_m", 0.80)
        self.declare_parameter("relative_anchor_face_tol_rad", 0.12)
        self.declare_parameter("relative_anchor_entry_margin_m", 0.10)
        self.declare_parameter("relative_anchor_entry_reach_tol_m", 0.12)
        # GLOBAL target mode: trust the wide-cam world map instead of touring zones. Every
        # confirmed target on the map (both sets) is visited in min-path order; the fixed
        # grid nodes the camera has not yet covered are swept only when the map is empty.
        # Expects zone_mission_enabled=false (the zone gates stay active otherwise).
        self.declare_parameter("global_target_mode", False)
        self.declare_parameter("global_retarget_min_gain", 0.20)
        self.declare_parameter("global_retarget_check_sec", 1.0)
        self.declare_parameter("global_retarget_lock_distance_m", 0.80)
        self.declare_parameter("global_mismatched_hint_verify_max_dist_m", 0.0)
        self.declare_parameter("global_fov_deg", 140.0)
        self.declare_parameter("global_fov_max_range_m", 2.5)
        self.declare_parameter("global_fov_min_range_m", 0.15)
        self.declare_parameter("global_wide_fov_forward_m", 1.1)
        self.declare_parameter("global_wide_fov_lateral_m", 1.3)
        self.declare_parameter("global_wide_fov_center_x_m", 0.4)
        self.declare_parameter("global_sensor_max_age_sec", 0.75)
        self.declare_parameter("global_initial_map_settle_sec", 1.0)
        self.declare_parameter("global_observe_min_sec", 0.20)
        self.declare_parameter("global_fast_safe_routes_enabled", True)
        self.declare_parameter("global_zigzag_patrol_enabled", True)
        self.declare_parameter("global_initial_sweep_holonomic_enabled", False)
        self.declare_parameter("global_initial_sweep_once_enabled", True)
        self.declare_parameter("global_initial_sweep_slowdown_distance_m", 0.45)
        self.declare_parameter("global_initial_sweep_slow_speed_mps", 0.07)
        self.declare_parameter("global_target_corridor_gate_enabled", False)
        self.declare_parameter("global_target_corridor_radius_m", 0.45)
        self.declare_parameter("global_strong_fruit_positive_min_confidence", 0.50)
        self.declare_parameter("global_non_target_fruit_reject_min_confidence", 0.40)
        self.declare_parameter("global_zigzag_start_xy", [-1.25, 1.60])
        # Fixed Z2-first vertical sweep.  Each horizontal connector and following vertical
        # leg produce the requested U-turn sequence: left-left, right-right, left-left.
        self.declare_parameter(
            "global_zigzag_waypoints_xy",
            [-1.75, -1.25, -0.75, -1.25, -0.75, 1.25,
             0.25, 1.25, 0.25, -1.25, 1.25, -1.25, 1.25, 1.25],
        )
        self.declare_parameter("global_approach_speed_mps", 0.12)
        self.declare_parameter("global_approach_route_blocked_defer_enabled", True)
        self.declare_parameter("global_approach_route_blocked_confirm_sec", 0.6)
        self.declare_parameter("global_approach_route_blocked_confirm_attempts", 2)
        self.declare_parameter("global_patrol_heading_tol_rad", 0.08)
        self.declare_parameter("global_patrol_grid_rows", 6)
        self.declare_parameter("global_patrol_grid_cols", 7)
        self.declare_parameter("global_patrol_grid_origin_x_m", -1.50)
        self.declare_parameter("global_patrol_grid_origin_y_m", -1.50)
        self.declare_parameter("global_patrol_grid_spacing_m", 0.50)
        self.declare_parameter("global_patrol_reach_tol_m", 0.15)
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
            [-1.0, 0.5, -1.0, -1.0, 1.25, -1.25, 0.75, 0.5],
        )
        self.declare_parameter(
            "zone_anchor_candidates",
            [1.0, -1.0, 0.5, 2.0, -1.0, -1.0, 3.0, 1.25, -1.25, 4.0, 0.75, 0.5],
        )
        self.declare_parameter("zone_no_target_advance_sec", 6.0)
        self.declare_parameter("zone_center_reach_tol_m", 0.25)
        self.declare_parameter("zone_anchor_nav_enabled", True)
        self.declare_parameter("zone_anchor_slowdown_enabled", True)
        self.declare_parameter("zone_anchor_slowdown_distance_m", 0.50)
        self.declare_parameter("zone_anchor_slow_speed", 0.07)
        self.declare_parameter("zone_stabilize_enabled", True)
        self.declare_parameter("zone_stabilize_sec", 3.0)
        self.declare_parameter("zone_stabilize_require_fresh_seen", True)
        self.declare_parameter("plain_cube_ambiguous_observations", 5)
        self.declare_parameter("zone_ambiguous_exclusion_radius_m", 0.18)
        # One isolated local fruit inspection runs immediately after each zone anchor arrival.
        # The embedded pure FSM owns no ROS publishers; this mission node remains the sole base
        # and arm command owner.  Prefix every knob so existing Set1 ALIGN tuning is untouched.
        for suffix, default in LOCAL_ANCHOR_PARAMETER_DEFAULTS.items():
            self.declare_parameter(f"local_anchor_{suffix}", default)
        # Step-wise search: turn a little, STOP to let the cameras identify (clean, blur-free frames),
        # turn again. Continuous spinning motion-blurs the wide cam and churns tracks.
        self.declare_parameter("search_turn_sec", 0.5)    # rotate this long per step (~small angle)
        self.declare_parameter("search_look_sec", 1.6)    # then hold still this long to identify
        # APPROACH uses the direct-navigation parameters above. Keep the target latched through a
        # momentary dropout instead of thrashing back to SELECT.
        self.declare_parameter("approach_lost_grace_sec", 2.5)  # keep target this long if it drops out
        self.declare_parameter("approach_standoff_tol", 0.09)   # reached the stand-off within this -> ALIGN
        self.declare_parameter("approach_brake_settle_sec", 0.40)
        self.declare_parameter("approach_heading_filter_alpha", 0.35)
        self.declare_parameter("approach_heading_stable_frames", 3)
        # Once the selected object is visible in a fresh Body frame, Body x/y is a safer
        # final reference than a map-derived bearing. Rotating to the latter can push an
        # already acquired object out of the camera immediately before ALIGN.
        self.declare_parameter("approach_body_visible_skip_heading_enabled", True)
        self.declare_parameter("approach_body_visible_max_age_sec", 0.35)
        self.declare_parameter("approach_connector_omega_max", 0.08)
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
        self.classify_body_max_age_sec = max(
            0.0, float(self.get_parameter("classify_body_max_age_sec").value)
        )
        self.set2_identity_latch_max_age_sec = max(
            0.1, float(self.get_parameter("set2_identity_latch_max_age_sec").value)
        )
        self.set2_identity_pose_max_skew_sec = max(
            0.0, float(self.get_parameter("set2_identity_pose_max_skew_sec").value)
        )
        self.set2_identity_max_inference_delay_sec = max(
            0.1,
            float(self.get_parameter("set2_identity_max_inference_delay_sec").value),
        )
        self.classify_body_settle_sec = max(
            0.0, float(self.get_parameter("classify_body_settle_sec").value)
        )
        self.classify_body_max_frames = max(
            1, int(self.get_parameter("classify_body_max_frames").value)
        )
        self.classify_body_retry_backoff_sec = max(
            0.0, float(self.get_parameter("classify_body_retry_backoff_sec").value)
        )
        self.classify_body_retry_settle_sec = max(
            0.0, float(self.get_parameter("classify_body_retry_settle_sec").value)
        )
        self.classify_octa_wide_final_enabled = bool(
            self.get_parameter("classify_octa_wide_final_enabled").value
        )
        self.classify_wide_confirm_frames = max(
            1, int(self.get_parameter("classify_wide_confirm_frames").value)
        )
        self.grab_x = float(self.get_parameter("grab_x").value)
        self.grab_y = float(self.get_parameter("grab_y").value)
        self.grab_min_x = float(self.get_parameter("grab_min_x").value)
        self.align_tol = float(self.get_parameter("align_tol_m").value)
        self.align_fwd_tol = float(self.get_parameter("align_fwd_tol_m").value)
        self.align_body_confirm_frames = max(
            1, int(self.get_parameter("align_body_confirm_frames").value)
        )
        self.align_wide_match_radius_m = max(
            0.0, float(self.get_parameter("align_wide_match_radius_m").value)
        )
        self.align_wide_max_age_sec = max(
            0.05, float(self.get_parameter("align_wide_max_age_sec").value)
        )
        self.align_kp = float(self.get_parameter("align_kp").value)
        self.align_vmax = float(self.get_parameter("align_vmax").value)
        self.align_vmin = float(self.get_parameter("align_vmin").value)
        self.align_timeout_sec = float(self.get_parameter("align_timeout_sec").value)
        self.max_align_fails = int(self.get_parameter("max_align_fails").value)
        self.align_distractor_confirm_frames = max(
            1, int(self.get_parameter("align_distractor_confirm_frames").value)
        )
        self._align_fail_count = 0
        self._classify_verify_phase = "settle"
        self._classify_verify_phase_start_s = 0.0
        self._classify_verify_retry_count = 0
        self._classify_body_last_seq = -1
        self._classify_wide_last_seq = -1
        self._classify_body_frame_count = 0
        self._classify_body_target_count = 0
        self._classify_body_other_counts: dict[str, int] = {}
        self.align_pulse_sec = float(self.get_parameter("align_pulse_sec").value)
        self.align_settle_pulse_sec = float(self.get_parameter("align_settle_pulse_sec").value)
        self.align_step_fwd_duty = float(self.get_parameter("align_step_fwd_duty").value)
        self.align_step_fwd_sec = float(self.get_parameter("align_step_fwd_sec").value)
        self.align_step_strafe_duty = float(
            self.get_parameter("align_step_strafe_duty").value
        )
        self.align_step_strafe_sec = float(
            self.get_parameter("align_step_strafe_sec").value
        )
        self.global_body_target_match_radius_m = max(
            0.05, float(self.get_parameter("global_body_target_match_radius_m").value)
        )
        self.global_target_absent_timeout_sec = max(
            0.0, float(self.get_parameter("global_target_absent_timeout_sec").value)
        )
        self.global_target_absent_confirm_frames = max(
            1, int(self.get_parameter("global_target_absent_confirm_frames").value)
        )
        self.align_body_lost_confirm_sec = max(
            0.0, float(self.get_parameter("align_body_lost_confirm_sec").value)
        )
        self.align_body_lost_confirm_frames = max(
            1, int(self.get_parameter("align_body_lost_confirm_frames").value)
        )
        self.align_body_lost_max_backoffs = max(
            0, int(self.get_parameter("align_body_lost_max_backoffs").value)
        )
        self.align_body_lost_reselect_cooldown_sec = max(
            0.0,
            float(self.get_parameter("align_body_lost_reselect_cooldown_sec").value),
        )
        self.align_heading_tolerance_rad = max(
            0.0, float(self.get_parameter("align_heading_tolerance_rad").value)
        )
        self.align_adaptive_steps_enabled = bool(self.get_parameter("align_adaptive_steps_enabled").value)
        self.align_mid_error_m = float(self.get_parameter("align_mid_error_m").value)
        self.align_step_fwd_mid_sec = float(self.get_parameter("align_step_fwd_mid_sec").value)
        self.align_step_strafe_mid_sec = float(
            self.get_parameter("align_step_strafe_mid_sec").value
        )
        self.align_body_lost_backoff_enabled = bool(self.get_parameter("align_body_lost_backoff_enabled").value)
        self.align_body_lost_backoff_speed = float(self.get_parameter("align_body_lost_backoff_speed").value)
        self.align_body_lost_backoff_sec = float(self.get_parameter("align_body_lost_backoff_sec").value)
        self.align_body_lost_backoff_settle_sec = float(
            self.get_parameter("align_body_lost_backoff_settle_sec").value
        )
        self.align_body_lost_lateral_sweep_enabled = bool(
            self.get_parameter("align_body_lost_lateral_sweep_enabled").value
        )
        self.align_body_lost_lateral_sweep_duty = abs(
            float(self.get_parameter("align_body_lost_lateral_sweep_duty").value)
        )
        self.align_body_lost_lateral_sweep_sec = max(
            0.0, float(self.get_parameter("align_body_lost_lateral_sweep_sec").value)
        )
        self.align_body_lost_lateral_sweep_settle_sec = max(
            0.0,
            float(self.get_parameter("align_body_lost_lateral_sweep_settle_sec").value),
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
        self._align_phase = "measure"      # measure -> face/pulse/backoff -> settle -> measure
        self._align_phase_start = 0.0
        self._pulse_vx = 0.0
        self._pulse_vy = 0.0
        self._pulse_sec = self.align_step_fwd_sec   # duration of the current unit step (per direction)
        self._slot_body_lost_retry_counts: dict[int, int] = {}
        self._global_body_lost_retry_counts: dict[tuple[int, int, int], int] = {}
        self._body_lost_lateral_sweep_counts: dict[tuple[int, int, int], int] = {}
        self._slot_missing_track_counts: dict[int, int] = {}
        self._align_heading_target: float | None = None
        self._align_body_presence_last_seq = -1
        self._align_body_presence_count = 0
        self._align_body_presence_xy: tuple[float, float] | None = None
        self._global_target_absent_since_s: float | None = None
        self._global_target_absent_last_body_seq = -1
        self._global_target_absent_frames = 0
        self._set2_track_loss_align_override_logged = False
        self._global_approach_committed_from_wide = False
        self._align_body_missing_since_s: float | None = None
        self._align_body_missing_last_seq = -1
        self._align_body_missing_frames = 0
        self._align_body_lost_backoff_attempts = 0
        self._align_body_lost_sweep_strafe_sign = 0.0
        self._align_arrival_geometry_logged = False
        self._global_deferred_targets: list[tuple[int, float, float, float]] = []
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
        # Latest Body boxes as (u, v_center, label, confidence).  A sequence counter ensures
        # confirmation counts distinct camera frames rather than repeated FSM timer ticks.
        self._body_dets: list = []
        self._body_dets_stamp_s = 0.0
        self._body_dets_seq = 0
        self._body_dets_history: list[
            tuple[tuple[int, int], list[tuple[float, float, str, float]]]
        ] = []
        self._siglip_body_dets: list[tuple[float, float, str, float]] = []
        self.pick_duration_sec = float(self.get_parameter("pick_duration_sec").value)
        self.pick_release_on_lift_enabled = bool(
            self.get_parameter("pick_release_on_lift_enabled").value
        )
        self.arm_pick_phase_max_age_sec = max(
            0.10, float(self.get_parameter("arm_pick_phase_max_age_sec").value)
        )
        self.storage_x = float(self.get_parameter("storage_x").value)
        self.storage_y = float(self.get_parameter("storage_y").value)
        self.timed_storage_enabled = bool(
            self.get_parameter("timed_storage_enabled").value
        )
        self.timed_storage_start_sec = max(
            0.0, float(self.get_parameter("timed_storage_start_sec").value)
        )
        self.timed_storage_dynamic_enabled = bool(
            self.get_parameter("timed_storage_dynamic_enabled").value
        )
        self.timed_storage_match_duration_sec = max(
            0.0, float(self.get_parameter("timed_storage_match_duration_sec").value)
        )
        self.timed_storage_effective_speed_mps = max(
            0.01, float(self.get_parameter("timed_storage_effective_speed_mps").value)
        )
        self.timed_storage_fixed_overhead_sec = max(
            0.0, float(self.get_parameter("timed_storage_fixed_overhead_sec").value)
        )
        self.timed_storage_safety_margin_sec = max(
            0.0, float(self.get_parameter("timed_storage_safety_margin_sec").value)
        )
        self.timed_storage_distance_safety_factor = max(
            1.0, float(self.get_parameter("timed_storage_distance_safety_factor").value)
        )
        self.storage_wall_guided_enabled = bool(
            self.get_parameter("storage_wall_guided_enabled").value
        )
        self.wall_initialization_enabled = bool(
            self.get_parameter("wall_initialization_enabled").value
        )
        self.wall_initialization_min_observe_sec = max(
            0.0, float(self.get_parameter("wall_initialization_min_observe_sec").value)
        )
        self.wall_initialization_confirm_frames = max(
            1, int(self.get_parameter("wall_initialization_confirm_frames").value)
        )
        self.wall_initialization_settle_sec = max(
            0.0, float(self.get_parameter("wall_initialization_settle_sec").value)
        )
        self.wall_initialization_timeout_sec = max(
            self.wall_initialization_min_observe_sec,
            float(self.get_parameter("wall_initialization_timeout_sec").value),
        )
        self.storage_wall_warmup_sec = max(
            0.0, float(self.get_parameter("storage_wall_warmup_sec").value)
        )
        self.storage_wall_warmup_timeout_sec = max(
            self.storage_wall_warmup_sec,
            float(self.get_parameter("storage_wall_warmup_timeout_sec").value),
        )
        self.storage_reverse_entry_enabled = bool(
            self.get_parameter("storage_reverse_entry_enabled").value
        )
        self.storage_fast_return_enabled = bool(
            self.get_parameter("storage_fast_return_enabled").value
        )
        self.storage_fast_nav_speed = max(
            0.0, float(self.get_parameter("storage_fast_nav_speed").value)
        )
        self.storage_fast_reverse_speed = max(
            0.0, float(self.get_parameter("storage_fast_reverse_speed").value)
        )
        self.storage_final_heading_rad = float(
            self.get_parameter("storage_final_heading_rad").value
        )
        self.storage_staging_x = float(self.get_parameter("storage_staging_x").value)
        self.storage_staging_y = float(self.get_parameter("storage_staging_y").value)
        self.storage_face_x = float(self.get_parameter("storage_face_x").value)
        self.storage_face_y = float(self.get_parameter("storage_face_y").value)
        self.storage_staging_reach_tol_m = max(
            0.02, float(self.get_parameter("storage_staging_reach_tol_m").value)
        )
        self.storage_heading_tolerance_rad = max(
            0.01, float(self.get_parameter("storage_heading_tolerance_rad").value)
        )
        self.storage_fast_heading_tolerance_rad = max(
            self.storage_heading_tolerance_rad,
            float(self.get_parameter("storage_fast_heading_tolerance_rad").value),
        )
        self.storage_reverse_speed = abs(
            float(self.get_parameter("storage_reverse_speed").value)
        )
        self.storage_reverse_heading_kp = max(
            0.0, float(self.get_parameter("storage_reverse_heading_kp").value)
        )
        self.storage_reverse_omega_max = abs(
            float(self.get_parameter("storage_reverse_omega_max").value)
        )
        self.storage_reverse_realign_rad = max(
            self.storage_heading_tolerance_rad,
            float(self.get_parameter("storage_reverse_realign_rad").value),
        )
        self.storage_reach_tol_m = max(
            0.02, float(self.get_parameter("storage_reach_tol_m").value)
        )
        self.storage_down_heading_rad = float(
            self.get_parameter("storage_down_heading_rad").value
        )
        self.storage_right_heading_rad = float(
            self.get_parameter("storage_right_heading_rad").value
        )
        self.storage_bottom_wall_stop_m = max(
            0.05, float(self.get_parameter("storage_bottom_wall_stop_m").value)
        )
        self.storage_left_wall_stop_m = max(
            0.05, float(self.get_parameter("storage_left_wall_stop_m").value)
        )
        self.storage_wall_approach_speed = abs(
            float(self.get_parameter("storage_wall_approach_speed").value)
        )
        self.storage_wall_confirm_frames = max(
            1, int(self.get_parameter("storage_wall_confirm_frames").value)
        )
        self.storage_wall_max_age_sec = max(
            0.10, float(self.get_parameter("storage_wall_max_age_sec").value)
        )
        self.storage_wall_detection_required_m = max(
            self.storage_bottom_wall_stop_m,
            self.storage_left_wall_stop_m,
            float(self.get_parameter("storage_wall_detection_required_m").value),
        )
        self.storage_pose_fallback_enabled = bool(
            self.get_parameter("storage_pose_fallback_enabled").value
        )
        self.storage_pose_fallback_hold_sec = max(
            0.0, float(self.get_parameter("storage_pose_fallback_hold_sec").value)
        )
        self.storage_pose_fallback_contact_speed = abs(
            float(self.get_parameter("storage_pose_fallback_contact_speed").value)
        )
        self.storage_last_chance_enabled = bool(
            self.get_parameter("storage_last_chance_enabled").value
        )
        self.storage_last_chance_trigger_sec = max(
            0.0, float(self.get_parameter("storage_last_chance_trigger_sec").value)
        )
        self.storage_last_chance_pre_backoff_m = max(
            0.0, float(self.get_parameter("storage_last_chance_pre_backoff_m").value)
        )
        self.storage_last_chance_pre_backoff_speed = abs(
            float(self.get_parameter("storage_last_chance_pre_backoff_speed").value)
        )
        self.storage_last_chance_turn_left_deg = float(
            self.get_parameter("storage_last_chance_turn_left_deg").value
        )
        self.storage_last_chance_reverse_sec = max(
            0.0, float(self.get_parameter("storage_last_chance_reverse_sec").value)
        )
        self.storage_last_chance_alternate_sec = max(
            0.1, float(self.get_parameter("storage_last_chance_alternate_sec").value)
        )
        self.storage_last_chance_force_dump_after_sec = max(
            0.0, float(self.get_parameter("storage_last_chance_force_dump_after_sec").value)
        )
        self.storage_last_chance_speed = abs(
            float(self.get_parameter("storage_last_chance_speed").value)
        )
        self.storage_contact_hold_enabled = bool(
            self.get_parameter("storage_contact_hold_enabled").value
        )
        self.storage_contact_hold_speed = abs(
            float(self.get_parameter("storage_contact_hold_speed").value)
        )
        self.shape_target_total = int(self.get_parameter("shape_target_total").value)
        self.fruit_target_total = int(self.get_parameter("fruit_target_total").value)
        self.dry_pick = bool(self.get_parameter("dry_pick").value)
        self.end_after_quota = bool(self.get_parameter("end_after_quota").value)
        self.select_timeout_sec = float(self.get_parameter("select_timeout_sec").value)
        self.opening_enabled = bool(self.get_parameter("opening_enabled").value)
        self.opening_wall_validation_enabled = bool(
            self.get_parameter("opening_wall_validation_enabled").value
        )
        self.opening_explore_while_moving = bool(
            self.get_parameter("opening_explore_while_moving").value
        )
        self.startup_warmup_sec = float(self.get_parameter("startup_warmup_sec").value)
        self.opening_speed = float(self.get_parameter("opening_speed").value)
        self.opening_forward_sec = float(self.get_parameter("opening_forward_sec").value)
        self.opening_strafe_speed = abs(
            float(self.get_parameter("opening_strafe_speed").value)
        )
        self.opening_strafe_right_sec = max(
            0.0, float(self.get_parameter("opening_strafe_right_sec").value)
        )
        self.opening_turn_deg = float(self.get_parameter("opening_turn_deg").value)
        self.opening_turn_omega = float(self.get_parameter("opening_turn_omega").value)
        self.opening_turn_tolerance_rad = max(
            0.01, float(self.get_parameter("opening_turn_tolerance_rad").value)
        )
        self.opening_turn_timeout_sec = max(
            0.1, float(self.get_parameter("opening_turn_timeout_sec").value)
        )
        self.opening_wait_after_turn_sec = float(self.get_parameter("opening_wait_after_turn_sec").value)
        self.opening_release_at_sec = max(0.0, float(self.get_parameter("opening_release_at_sec").value))
        self.opening_heading_observe_sec = max(
            0.0, float(self.get_parameter("opening_heading_observe_sec").value)
        )
        self.opening_wall_heading_tolerance_rad = max(
            0.005, float(self.get_parameter("opening_wall_heading_tolerance_rad").value)
        )
        self.opening_wall_heading_max_stddev_rad = max(
            0.0, float(self.get_parameter("opening_wall_heading_max_stddev_rad").value)
        )
        self.opening_wall_heading_min_segments = max(
            1, int(self.get_parameter("opening_wall_heading_min_segments").value)
        )
        self.opening_heading_stable_frames = max(
            1, int(self.get_parameter("opening_heading_stable_frames").value)
        )
        self.opening_position_observe_sec = max(
            0.0, float(self.get_parameter("opening_position_observe_sec").value)
        )
        self.opening_wall_position_tolerance_m = max(
            0.005, float(self.get_parameter("opening_wall_position_tolerance_m").value)
        )
        self.opening_wall_position_min_confidence = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter("opening_wall_position_min_confidence").value),
            ),
        )
        self.opening_position_stable_frames = max(
            1, int(self.get_parameter("opening_position_stable_frames").value)
        )
        self.opening_anchor_strafe_duty = max(
            0.0, float(self.get_parameter("opening_anchor_strafe_duty").value)
        )
        self.opening_anchor_strafe_medium_threshold_m = max(
            0.0,
            float(self.get_parameter("opening_anchor_strafe_medium_threshold_m").value),
        )
        self.opening_anchor_strafe_long_threshold_m = max(
            self.opening_anchor_strafe_medium_threshold_m,
            float(self.get_parameter("opening_anchor_strafe_long_threshold_m").value),
        )
        self.opening_anchor_strafe_long_pulse_sec = max(
            0.03,
            float(self.get_parameter("opening_anchor_strafe_long_pulse_sec").value),
        )
        self.opening_anchor_strafe_medium_pulse_sec = max(
            0.03,
            float(self.get_parameter("opening_anchor_strafe_medium_pulse_sec").value),
        )
        self.opening_anchor_strafe_short_pulse_sec = max(
            0.03,
            float(self.get_parameter("opening_anchor_strafe_short_pulse_sec").value),
        )
        self.opening_anchor_strafe_settle_sec = max(
            0.0, float(self.get_parameter("opening_anchor_strafe_settle_sec").value)
        )
        self.opening_anchor_strafe_max_pulses = max(
            1, int(self.get_parameter("opening_anchor_strafe_max_pulses").value)
        )
        self.scan_search_omega = float(self.get_parameter("scan_search_omega").value)
        self.direct_nav_speed = float(self.get_parameter("direct_nav_speed").value)
        self.direct_nav_kp_ang = float(self.get_parameter("direct_nav_kp_ang").value)
        self.direct_nav_omega_max = float(self.get_parameter("direct_nav_omega_max").value)
        self.direct_nav_face_tol = float(self.get_parameter("direct_nav_face_tol").value)
        self.direct_nav_stop_radius_m = float(self.get_parameter("direct_nav_stop_radius_m").value)
        self._pulsed_heading = PulsedHeadingController(
            pulse_omega=float(self.get_parameter("nav_turn_pulse_omega").value),
            slowdown_rad=float(self.get_parameter("nav_turn_slowdown_rad").value),
            continuous_min_error_rad=float(
                self.get_parameter("nav_turn_continuous_min_error_rad").value
            ),
            coarse_pulse_sec=float(self.get_parameter("nav_turn_coarse_pulse_sec").value),
            coarse_settle_sec=float(self.get_parameter("nav_turn_coarse_settle_sec").value),
            verify_sec=float(self.get_parameter("nav_turn_verify_sec").value),
            fine_pulse_sec=float(self.get_parameter("nav_turn_fine_pulse_sec").value),
            fine_settle_sec=float(self.get_parameter("nav_turn_fine_settle_sec").value),
            max_pulses=int(self.get_parameter("nav_turn_max_pulses").value),
            timeout_sec=float(self.get_parameter("nav_turn_timeout_sec").value),
        )
        self.lane_heading_lock_enabled = bool(
            self.get_parameter("lane_heading_lock_enabled").value
        )
        self.lane_heading_initial_align_enabled = bool(
            self.get_parameter("lane_heading_initial_align_enabled").value
        )
        self.lane_heading_axis_tolerance_m = max(
            0.0, float(self.get_parameter("lane_heading_axis_tolerance_m").value)
        )
        self.lane_heading_align_tolerance_rad = max(
            0.0, float(self.get_parameter("lane_heading_align_tolerance_rad").value)
        )
        self.lane_heading_realign_tolerance_rad = max(
            self.lane_heading_align_tolerance_rad,
            float(self.get_parameter("lane_heading_realign_tolerance_rad").value),
        )
        self.lane_heading_realign_arm_sec = max(
            0.0, float(self.get_parameter("lane_heading_realign_arm_sec").value)
        )
        self.lane_heading_soft_entry_tolerance_rad = max(
            self.lane_heading_realign_tolerance_rad,
            float(self.get_parameter("lane_heading_soft_entry_tolerance_rad").value),
        )
        self.lane_heading_soft_entry_speed = max(
            0.0, float(self.get_parameter("lane_heading_soft_entry_speed").value)
        )
        self.lane_heading_soft_entry_kp = max(
            0.0, float(self.get_parameter("lane_heading_soft_entry_kp").value)
        )
        self.lane_heading_soft_entry_omega_max = max(
            0.0, float(self.get_parameter("lane_heading_soft_entry_omega_max").value)
        )
        self.lane_heading_soft_entry_timeout_sec = max(
            0.0, float(self.get_parameter("lane_heading_soft_entry_timeout_sec").value)
        )
        self.lane_heading_realign_hold_sec = max(
            0.0, float(self.get_parameter("lane_heading_realign_hold_sec").value)
        )
        self.lane_heading_drive_omega_max = max(
            0.0, float(self.get_parameter("lane_heading_drive_omega_max").value)
        )
        self.lane_heading_filter_alpha = max(
            0.0, min(1.0, float(self.get_parameter("lane_heading_filter_alpha").value))
        )
        self.lane_heading_settle_sec = max(
            0.0, float(self.get_parameter("lane_heading_settle_sec").value)
        )
        self.lane_heading_reverse_settle_sec = max(
            0.0, float(self.get_parameter("lane_heading_reverse_settle_sec").value)
        )
        self.lane_heading_kp = max(
            0.0, float(self.get_parameter("lane_heading_kp").value)
        )
        self.lane_heading_omega_max = max(
            0.0, float(self.get_parameter("lane_heading_omega_max").value)
        )
        self.lane_heading_deadband_rad = max(
            0.0, float(self.get_parameter("lane_heading_deadband_rad").value)
        )
        self.lane_cross_track_enabled = bool(
            self.get_parameter("lane_cross_track_enabled").value
        )
        self.lane_center_band_half_width_m = max(
            0.0, float(self.get_parameter("lane_center_band_half_width_m").value)
        )
        self.lane_cross_track_lookahead_m = max(
            1e-3, float(self.get_parameter("lane_cross_track_lookahead_m").value)
        )
        self.lane_cross_track_gain = max(
            0.0, float(self.get_parameter("lane_cross_track_gain").value)
        )
        self.lane_cross_track_filter_alpha = max(
            0.0,
            min(1.0, float(self.get_parameter("lane_cross_track_filter_alpha").value)),
        )
        self.lane_cross_track_max_heading_bias_rad = max(
            0.0,
            float(self.get_parameter("lane_cross_track_max_heading_bias_rad").value),
        )
        self.lane_cross_track_heading_kp = max(
            0.0, float(self.get_parameter("lane_cross_track_heading_kp").value)
        )
        self.lane_cross_track_slow_error_m = max(
            self.lane_center_band_half_width_m,
            float(self.get_parameter("lane_cross_track_slow_error_m").value),
        )
        self.lane_cross_track_full_slow_error_m = max(
            self.lane_cross_track_slow_error_m + 1e-3,
            float(self.get_parameter("lane_cross_track_full_slow_error_m").value),
        )
        self.lane_cross_track_min_speed_mps = max(
            0.0, float(self.get_parameter("lane_cross_track_min_speed_mps").value)
        )
        self.stuck_escape_enabled = bool(
            self.get_parameter("stuck_escape_enabled").value
        )
        self.stuck_detect_duration_sec = max(
            0.0, float(self.get_parameter("stuck_detect_duration_sec").value)
        )
        self.stuck_min_cmd_linear_mps = max(
            0.0, float(self.get_parameter("stuck_min_cmd_linear_mps").value)
        )
        self.stuck_min_cmd_omega_radps = max(
            0.0, float(self.get_parameter("stuck_min_cmd_omega_radps").value)
        )
        self.stuck_pose_delta_min_m = max(
            0.0, float(self.get_parameter("stuck_pose_delta_min_m").value)
        )
        self.stuck_heading_delta_min_rad = max(
            0.0, float(self.get_parameter("stuck_heading_delta_min_rad").value)
        )
        self.stuck_escape_stop_sec = max(
            0.0, float(self.get_parameter("stuck_escape_stop_sec").value)
        )
        self.stuck_escape_pulse_sec = max(
            0.0, float(self.get_parameter("stuck_escape_pulse_sec").value)
        )
        self.stuck_escape_settle_sec = max(
            0.0, float(self.get_parameter("stuck_escape_settle_sec").value)
        )
        self.stuck_escape_linear_speed = max(
            0.0, float(self.get_parameter("stuck_escape_linear_speed").value)
        )
        self.stuck_escape_omega_speed = max(
            0.0, float(self.get_parameter("stuck_escape_omega_speed").value)
        )
        self.map_center_x = float(self.get_parameter("map_center_x").value)
        self.map_center_y = float(self.get_parameter("map_center_y").value)
        wp = [float(v) for v in self.get_parameter("patrol_waypoints").value]
        self._patrol_waypoints = [(wp[i], wp[i + 1]) for i in range(0, len(wp) - 1, 2)] or [(0.0, 0.0)]
        self.patrol_reach_tol = float(self.get_parameter("patrol_reach_tol").value)
        self._patrol_idx = 0
        self.anchor_mission_enabled = bool(self.get_parameter("anchor_mission_enabled").value)
        anchor_xy = [float(v) for v in self.get_parameter("anchor_xy").value]
        if len(anchor_xy) != 6:
            raise ValueError("anchor_xy must contain K1,K2,K3 as three x,y pairs")
        self.anchor_reach_tol_m = float(self.get_parameter("anchor_reach_tol_m").value)
        self.anchor_observe_sec = float(self.get_parameter("anchor_observe_sec").value)
        self.anchor_birth_delay_sec = float(self.get_parameter("anchor_birth_delay_sec").value)
        self.anchor_slot_match_radius_m = float(
            self.get_parameter("anchor_slot_match_radius_m").value
        )
        self.anchor_track_attach_radius_m = float(
            self.get_parameter("anchor_track_attach_radius_m").value
        )
        self.anchor_face_tol_rad = float(self.get_parameter("anchor_face_tol_rad").value)
        self.body_slot_wide_low_confidence = min(
            1.0,
            max(0.0, float(self.get_parameter("body_slot_wide_low_confidence").value)),
        )
        self.body_slot_forward_tol_m = max(
            0.0, float(self.get_parameter("body_slot_forward_tol_m").value)
        )
        self.body_slot_lateral_tol_m = max(
            0.0, float(self.get_parameter("body_slot_lateral_tol_m").value)
        )
        self.body_slot_bearing_tol_rad = math.radians(
            max(0.0, float(self.get_parameter("body_slot_bearing_tol_deg").value))
        )
        self.body_slot_unique_margin_m = max(
            0.0, float(self.get_parameter("body_slot_unique_margin_m").value)
        )
        # Keep post-bind Body tracking strictly inside half of the 50 cm slot spacing,
        # independent of Wide track-association tuning.
        self.body_slot_continuity_radius_m = min(
            0.24,
            max(0.0, float(self.get_parameter("body_slot_continuity_radius_m").value)),
        )
        self.body_slot_confirm_frames = max(
            1, int(self.get_parameter("body_slot_confirm_frames").value)
        )
        self.body_slot_backoff_retries = max(
            0, int(self.get_parameter("body_slot_backoff_retries").value)
        )
        self.body_slot_final_retry_enabled = bool(
            self.get_parameter("body_slot_final_retry_enabled").value
        )
        self.anchor_inventory = AnchorSlotInventory(
            snapshot_radius_m=self.anchor_slot_match_radius_m,
            anchor_xy=anchor_xy,
        )
        self.relative_anchor_nav_enabled = bool(
            self.get_parameter("relative_anchor_nav_enabled").value
        )
        self.relative_anchor_acquire_sec = max(
            0.0, float(self.get_parameter("relative_anchor_acquire_sec").value)
        )
        expected_k1 = [
            float(v) for v in self.get_parameter("relative_anchor_expected_k1_base").value
        ]
        route_unit = [
            float(v) for v in self.get_parameter("relative_anchor_route_unit_base").value
        ]
        if len(expected_k1) != 2:
            raise ValueError("relative_anchor_expected_k1_base must contain [x,y]")
        if len(route_unit) != 2:
            raise ValueError("relative_anchor_route_unit_base must contain [x,y]")
        self.relative_anchor_expected_k1_base = (expected_k1[0], expected_k1[1])
        self.relative_anchor_grid_yaw_base = float(
            self.get_parameter("relative_anchor_grid_yaw_base").value
        )
        self.relative_anchor_max_capture_age_sec = max(
            0.05, float(self.get_parameter("relative_anchor_max_capture_age_sec").value)
        )
        self.relative_anchor_min_cluster_hits = max(
            1, int(self.get_parameter("relative_anchor_min_cluster_hits").value)
        )
        self.relative_anchor_min_points = max(
            2, int(self.get_parameter("relative_anchor_min_points").value)
        )
        self.relative_anchor_slot_min_hits = max(
            1, int(self.get_parameter("relative_anchor_slot_min_hits").value)
        )
        self.relative_anchor_center_gate_m = max(
            0.05, float(self.get_parameter("relative_anchor_center_gate_m").value)
        )
        self.relative_anchor_face_tol_rad = max(
            0.01, float(self.get_parameter("relative_anchor_face_tol_rad").value)
        )
        self.relative_anchor_entry_margin_m = max(
            0.0, float(self.get_parameter("relative_anchor_entry_margin_m").value)
        )
        self.relative_anchor_entry_reach_tol_m = max(
            0.01, float(self.get_parameter("relative_anchor_entry_reach_tol_m").value)
        )
        self._relative_anchor_tracker = RelativeAnchorGridTracker(
            spacing_m=0.50,
            anchor_step_m=float(self.get_parameter("relative_anchor_step_m").value),
            route_unit_base=(route_unit[0], route_unit[1]),
            association_radius_m=float(
                self.get_parameter("relative_anchor_association_radius_m").value
            ),
            cluster_radius_m=float(
                self.get_parameter("relative_anchor_cluster_radius_m").value
            ),
            stale_timeout_s=float(self.get_parameter("relative_anchor_stale_sec").value),
            dead_reckon_timeout_s=float(
                self.get_parameter("relative_anchor_dead_reckon_sec").value
            ),
        )
        self._relative_acquisition_started_s: float | None = None
        self._relative_acquisition_first_frame_s: float | None = None
        self._relative_last_frame_s: float | None = None
        self._relative_last_observation_s: float | None = None
        self._wide_relative_observations: list[tuple[float, float]] = []
        self._wide_relative_labeled_observations: list[RelativeObservation] = []
        self._wide_relative_last_frame_s: float | None = None
        self._wide_relative_seq = 0
        self._relative_motion_last_s = self._now_s()
        self._relative_last_command = (0.0, 0.0, 0.0)
        self._relative_entry_edges: dict[int, tuple[int, int]] = {}
        self._relative_entry_reached: set[int] = set()
        self._relative_anchor_arrived: set[int] = set()
        self._relative_observe_hit_baseline: dict[int, int] = {}
        self._relative_wait_reason = ""
        self._anchor_idx = 0
        self._anchor_current_slot_id: int | None = None
        self._anchor_observe_start_s = 0.0
        self._anchor_slot_heading = 0.0
        self._anchor_body_backoff_counts: dict[int, int] = {}
        self._anchor_body_last_seq = self._body_dets_seq
        self._anchor_body_confirm_count = 0
        self._anchor_body_missing_count = 0
        self._anchor_body_candidate_label = ""
        self._anchor_body_candidate_confidence = 0.0
        self._anchor_body_confirmed_label = ""
        self._anchor_body_confirmed_confidence = 0.0
        self._anchor_body_bound_point: tuple[float, float] | None = None
        self.global_target_mode = bool(self.get_parameter("global_target_mode").value)
        self.global_retarget_min_gain = float(self.get_parameter("global_retarget_min_gain").value)
        self.global_retarget_check_sec = float(self.get_parameter("global_retarget_check_sec").value)
        self.global_retarget_lock_distance_m = max(
            0.0, float(self.get_parameter("global_retarget_lock_distance_m").value)
        )
        self.global_mismatched_hint_verify_max_dist_m = max(
            0.0,
            float(self.get_parameter("global_mismatched_hint_verify_max_dist_m").value),
        )
        self.global_fov_rad = math.radians(float(self.get_parameter("global_fov_deg").value))
        self.global_fov_max_range_m = float(self.get_parameter("global_fov_max_range_m").value)
        self.global_fov_min_range_m = float(self.get_parameter("global_fov_min_range_m").value)
        self.global_wide_fov_forward_m = float(
            self.get_parameter("global_wide_fov_forward_m").value
        )
        self.global_wide_fov_lateral_m = float(
            self.get_parameter("global_wide_fov_lateral_m").value
        )
        self.global_wide_fov_center_x_m = float(
            self.get_parameter("global_wide_fov_center_x_m").value
        )
        self.global_sensor_max_age_sec = max(
            0.05, float(self.get_parameter("global_sensor_max_age_sec").value)
        )
        self.global_initial_map_settle_sec = max(
            0.0, float(self.get_parameter("global_initial_map_settle_sec").value)
        )
        self.global_observe_min_sec = max(
            0.0, float(self.get_parameter("global_observe_min_sec").value)
        )
        self.global_fast_safe_routes_enabled = bool(
            self.get_parameter("global_fast_safe_routes_enabled").value
        )
        self.global_zigzag_patrol_enabled = bool(
            self.get_parameter("global_zigzag_patrol_enabled").value
        )
        self.global_initial_sweep_holonomic_enabled = bool(
            self.get_parameter("global_initial_sweep_holonomic_enabled").value
        )
        self.global_initial_sweep_once_enabled = bool(
            self.get_parameter("global_initial_sweep_once_enabled").value
        )
        self.global_initial_sweep_slowdown_distance_m = max(
            0.0,
            float(
                self.get_parameter("global_initial_sweep_slowdown_distance_m").value
            ),
        )
        self.global_initial_sweep_slow_speed_mps = max(
            0.0,
            float(self.get_parameter("global_initial_sweep_slow_speed_mps").value),
        )
        self.global_target_corridor_gate_enabled = bool(
            self.get_parameter("global_target_corridor_gate_enabled").value
        )
        self.global_target_corridor_radius_m = max(
            0.0,
            float(self.get_parameter("global_target_corridor_radius_m").value),
        )
        self.global_strong_fruit_positive_min_confidence = max(
            0.0,
            float(
                self.get_parameter(
                    "global_strong_fruit_positive_min_confidence"
                ).value
            ),
        )
        self.global_non_target_fruit_reject_min_confidence = max(
            0.0,
            float(
                self.get_parameter(
                    "global_non_target_fruit_reject_min_confidence"
                ).value
            ),
        )
        zigzag_start_xy = [
            float(value)
            for value in self.get_parameter("global_zigzag_start_xy").value
        ]
        self.global_zigzag_start = (
            (zigzag_start_xy[0], zigzag_start_xy[1])
            if len(zigzag_start_xy) >= 2
            else (-1.25, 1.60)
        )
        zigzag_xy = [
            float(value)
            for value in self.get_parameter("global_zigzag_waypoints_xy").value
        ]
        self.global_zigzag_waypoints = [
            (zigzag_xy[index], zigzag_xy[index + 1])
            for index in range(0, len(zigzag_xy) - 1, 2)
        ]
        self.global_zigzag_fixed_route = bool(self.global_zigzag_waypoints)
        self.global_approach_speed_mps = max(
            0.0, float(self.get_parameter("global_approach_speed_mps").value)
        )
        self.global_approach_route_blocked_defer_enabled = bool(
            self.get_parameter("global_approach_route_blocked_defer_enabled").value
        )
        self.global_approach_route_blocked_confirm_sec = max(
            0.0,
            float(
                self.get_parameter("global_approach_route_blocked_confirm_sec").value
            ),
        )
        self.global_approach_route_blocked_confirm_attempts = max(
            1,
            int(
                self.get_parameter(
                    "global_approach_route_blocked_confirm_attempts"
                ).value
            ),
        )
        self.global_patrol_heading_tol_rad = max(
            0.01, float(self.get_parameter("global_patrol_heading_tol_rad").value)
        )
        self.global_patrol_reach_tol_m = float(self.get_parameter("global_patrol_reach_tol_m").value)
        self._global_patrol_nodes = grid_nodes(
            int(self.get_parameter("global_patrol_grid_rows").value),
            int(self.get_parameter("global_patrol_grid_cols").value),
            float(self.get_parameter("global_patrol_grid_origin_x_m").value),
            float(self.get_parameter("global_patrol_grid_origin_y_m").value),
            float(self.get_parameter("global_patrol_grid_spacing_m").value),
        )
        self._global_seen_nodes: set[int] = set()
        self._global_retarget_last_s = 0.0
        self._global_seen_wide_seq = -1
        self._global_patrol_goal: tuple[float, float, float, set[int]] | None = None
        self._global_patrol_goal_zigzag_index: int | None = None
        self._global_zigzag_waypoints: list[tuple[float, float]] = list(
            self.global_zigzag_waypoints
        )
        self._global_zigzag_idx = 0
        self._global_initial_sweep_start_xy: tuple[float, float] | None = None
        self._global_initial_sweep_complete = not bool(self._global_zigzag_waypoints)
        self._global_patrol_observe_seq = -1
        self._global_patrol_observe_start_s = 0.0
        self.zone_mission_enabled = bool(self.get_parameter("zone_mission_enabled").value)
        self.zone_order = [int(v) for v in self.get_parameter("zone_order").value] or [1, 2, 3, 4]
        zb = [float(v) for v in self.get_parameter("zone_bounds_m").value]
        self.zone_bounds: dict[int, tuple[float, float, float, float]] = {}
        for i in range(0, min(len(zb), 16), 4):
            zid = i // 4 + 1
            self.zone_bounds[zid] = (zb[i], zb[i + 1], zb[i + 2], zb[i + 3])
        za = [float(v) for v in self.get_parameter("zone_anchor_xy").value]
        zc = [float(v) for v in self.get_parameter("zone_anchor_candidates").value]
        self.zone_anchor_candidates = _parse_zone_anchor_candidates(zc, za)
        self.zone_no_target_advance_sec = float(self.get_parameter("zone_no_target_advance_sec").value)
        self.zone_center_reach_tol_m = float(self.get_parameter("zone_center_reach_tol_m").value)
        self.zone_anchor_nav_enabled = bool(self.get_parameter("zone_anchor_nav_enabled").value)
        self.zone_anchor_slowdown_enabled = bool(
            self.get_parameter("zone_anchor_slowdown_enabled").value
        )
        self.zone_anchor_slowdown_distance_m = max(
            0.0, float(self.get_parameter("zone_anchor_slowdown_distance_m").value)
        )
        self.zone_anchor_slow_speed = max(
            0.0, float(self.get_parameter("zone_anchor_slow_speed").value)
        )
        self.zone_stabilize_enabled = bool(self.get_parameter("zone_stabilize_enabled").value)
        self.zone_stabilize_sec = float(self.get_parameter("zone_stabilize_sec").value)
        self.zone_stabilize_require_fresh_seen = bool(
            self.get_parameter("zone_stabilize_require_fresh_seen").value
        )
        self.plain_cube_ambiguous_observations = max(
            1, int(self.get_parameter("plain_cube_ambiguous_observations").value)
        )
        self.zone_ambiguous_exclusion_radius_m = max(
            0.01, float(self.get_parameter("zone_ambiguous_exclusion_radius_m").value)
        )
        self._zone_idx = 0
        self._zone_no_target_since: float | None = None
        self._zone_stabilized_idx: int | None = None
        self._zone_anchor_reached_idx: int | None = None
        self._zone_entry_anchor_idx: int | None = None
        self._zone_entry_anchor: tuple[float, float] | None = None
        self._zone_anchor_slowdown_active = False
        self._opening_zone_entry_pending = self.opening_enabled
        self._opening_zone_entry_path: list[tuple[float, float]] | None = None
        self._opening_zone_entry_start: tuple[float, float] | None = None
        self._opening_zone_entry_idx = 0
        self._opening_zone_entry_generation = 0
        self._opening_anchor_strafe_phase = "align"
        self._opening_anchor_strafe_phase_start_s = 0.0
        self._opening_anchor_strafe_pulses = 0
        self._opening_anchor_strafe_vy = 0.0
        self._opening_anchor_strafe_pulse_duration_sec = 0.0
        self._zone_stabilize_start_s = 0.0
        self._zone_stabilized_after_s = 0.0
        self._zone_ambiguous_excluded_ids: set[int] = set()
        self._zone_ambiguous_excluded_xy: list[tuple[float, float]] = []
        self._plain_cube_unknown_count = 0
        self._plain_cube_unknown_last_stamp: tuple[int, int] | None = None
        self.local_anchor_enabled = bool(
            self.get_parameter("local_anchor_enabled").value
        )
        self.local_anchor_run_timeout_sec = max(
            1.0, float(self.get_parameter("local_anchor_run_timeout_sec").value)
        )
        self.local_anchor_input_timeout_sec = max(
            0.05,
            float(self.get_parameter("local_anchor_input_timeout_sec").value),
        )
        self.local_anchor_pose_confirm_frames = max(
            1,
            int(self.get_parameter("local_anchor_pose_confirm_frames").value),
        )
        self.local_anchor_world_confirm_frames = max(
            1,
            int(self.get_parameter("local_anchor_world_confirm_frames").value),
        )
        self.local_anchor_pose_max_age_sec = max(
            0.05,
            float(self.get_parameter("local_anchor_pose_max_age_sec").value),
        )
        self.local_anchor_pose_world_xy_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    "local_anchor_pose_world_xy_tolerance_m"
                ).value
            ),
        )
        self.local_anchor_pose_world_theta_tolerance_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        "local_anchor_pose_world_theta_tolerance_deg"
                    ).value
                ),
            )
        )
        self.local_anchor_candidate_labels = {
            str(value).strip().lower()
            for value in self.get_parameter("local_anchor_candidate_labels").value
        }
        self.local_anchor_body_align_labels = {
            str(value).strip().lower()
            for value in self.get_parameter("local_anchor_body_align_labels").value
        }
        self.local_anchor_picked_track_match_radius_m = max(
            0.0,
            float(
                self.get_parameter("local_anchor_picked_track_match_radius_m").value
            ),
        )
        self.local_anchor_config = self._build_local_anchor_config()
        self._local_anchor_fsm = LocalAnchorFruitFsm(self.local_anchor_config)
        self._local_anchor_visited_zones: set[int] = set()
        self._local_anchor_active_zone_id: int | None = None
        self._local_anchor_relative_yaw = 0.0
        self._local_anchor_start_pose: tuple[float, float, float] | None = None
        self._local_anchor_pick_triggered = False
        self._local_anchor_pick_accounted = False
        self._local_anchor_pose_baseline_seq = 0
        self._local_anchor_world_baseline_seq = 0
        self._local_anchor_pose_refresh_required = False

        # ---- LANE-GRAPH waypoint planner (50cm object grid, 40cm robot) ----
        # Travel goals (SCAN sweep, APPROACH stand-off, DRIVE_TO_STORAGE) route through collision-free
        # lane midlines instead of a straight shot; go_to_goal still drives each via car-like + reactive.
        self.declare_parameter("planner_enabled", True)
        self.declare_parameter("grid_spacing_m", 0.50)
        self.declare_parameter("grid_origin_mode", "fixed")     # infer | fixed
        self.declare_parameter("grid_origin_xy", [0.0, 0.0])
        self.declare_parameter("phase_tol_m", 0.08)
        self.declare_parameter("field_bounds_m", [-2.0, 2.0, -2.0, 2.0])   # mirror go_to_goal
        self.declare_parameter("robot_margin_m", 0.22)
        self.declare_parameter("field_boundary_guard_m", 0.03)
        self.declare_parameter("lane_block_radius_m", 0.24)     # obstacle-to-lane dist that blocks an edge
        self.declare_parameter("comfort_clear_m", 0.35)
        self.declare_parameter("clearance_weight", 2.0)         # prefer roomy lanes over tight gates
        self.declare_parameter("lane_simplify_enabled", False)  # false keeps raw 4-connected lane vias
        self.declare_parameter("direct_fallback_enabled", True)
        self.declare_parameter("start_connect_k", 4)
        self.declare_parameter("wp_reach_tol_m", 0.12)          # advance to next via within this
        self.declare_parameter("lane_pass_lateral_tol_m", 0.15)
        self.declare_parameter("replan_period_sec", 1.5)
        self.declare_parameter("periodic_replan_enabled", False)
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
        self._field_bounds = (
            (fb[0], fb[1], fb[2], fb[3])
            if len(fb) == 4
            else (-2.0, 2.0, -2.0, 2.0)
        )
        self.field_boundary_guard_m = max(
            0.0, float(self.get_parameter("field_boundary_guard_m").value)
        )
        self.wp_reach_tol_m = float(self.get_parameter("wp_reach_tol_m").value)
        self.lane_pass_lateral_tol_m = max(
            0.0, float(self.get_parameter("lane_pass_lateral_tol_m").value)
        )
        self.replan_period_sec = float(self.get_parameter("replan_period_sec").value)
        self.periodic_replan_enabled = bool(
            self.get_parameter("periodic_replan_enabled").value
        )
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
            bounds=self._field_bounds,
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
        self._plan_start_xy: tuple[float, float] | None = None
        self._plan_route_mode = "grid_only"
        self._post_pick_lane_entry_pending = False
        self._plan_generation = 0
        self._lane_heading_segment_key: tuple[int, int] | None = None
        self._lane_heading_phase = "align"
        self._lane_heading_filtered: float | None = None
        self._lane_heading_violation_start_s: float | None = None
        self._lane_heading_realign_armed_at_s = 0.0
        self._lane_heading_settle_start_s = 0.0
        self._lane_heading_entry_start_s = 0.0
        self._lane_heading_turn_sign = 0
        self._lane_heading_reverse_start_s = 0.0
        self._lane_cross_track_filtered = None
        self._lane_cross_track_outside = False
        self._object_connector_brake_key: tuple[int, int] | None = None
        self._object_connector_brake_start_s = 0.0
        self._plan_dest: tuple[float, float] | None = None
        self._plan_escape = False
        self._plan_flexible_fallback = False
        self._plan_stamp = 0.0
        self._last_replan_t = 0.0
        self._last_drive_route_blocked = False
        self._last_drive_command: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._stuck_watch_started_s: float | None = None
        self._stuck_watch_pose: tuple[float, float, float] | None = None
        self._stuck_watch_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._stuck_escape_phase: str | None = None
        self._stuck_escape_phase_start_s = 0.0
        self._stuck_escape_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._stuck_escape_count = 0
        self._approach_route_blocked_since_s: float | None = None
        self._approach_route_blocked_attempts = 0
        self._coverage: list[tuple[float, float]] | None = None   # SCAN lane-sweep waypoints
        self._coverage_idx = 0
        self.search_turn_sec = float(self.get_parameter("search_turn_sec").value)
        self.search_look_sec = float(self.get_parameter("search_look_sec").value)
        self.approach_lost_grace_sec = float(self.get_parameter("approach_lost_grace_sec").value)
        self.approach_standoff_tol = float(self.get_parameter("approach_standoff_tol").value)
        self.approach_brake_settle_sec = max(
            0.0, float(self.get_parameter("approach_brake_settle_sec").value)
        )
        self.approach_heading_filter_alpha = max(
            0.0, min(1.0, float(self.get_parameter("approach_heading_filter_alpha").value))
        )
        self.approach_heading_stable_frames = max(
            1, int(self.get_parameter("approach_heading_stable_frames").value)
        )
        self.approach_body_visible_skip_heading_enabled = bool(
            self.get_parameter("approach_body_visible_skip_heading_enabled").value
        )
        self.approach_body_visible_max_age_sec = max(
            0.05,
            float(self.get_parameter("approach_body_visible_max_age_sec").value),
        )
        self.approach_connector_omega_max = max(
            0.0, float(self.get_parameter("approach_connector_omega_max").value)
        )
        self.classify_standoff_sec = float(self.get_parameter("classify_standoff_sec").value)
        self.set2_require_fruit_label = bool(self.get_parameter("set2_require_fruit_label").value)
        self.set2_committed_cube_pick_enabled = bool(
            self.get_parameter("set2_committed_cube_pick_enabled").value
        )
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
        self._approach_standoff_xy: tuple[float, float] | None = None
        self._approach_heading_latched: float | None = None
        self._approach_motion_phase = "travel"
        self._approach_brake_start_s = 0.0
        self._approach_heading_filtered: float | None = None
        self._approach_heading_stable_count = 0
        rate = float(self.get_parameter("publish_rate_hz").value)

        # --- runtime state ---
        # Competition safety owns t=0. The mission remains STANDBY through the READY perception
        # phase and is initialized at OPENING only on the first latched RUNNING message.
        self.state = "STANDBY"
        self._competition_state = "STANDBY"
        self._run_started = False
        self._run_initial_state = (
            "OPENING"
            if self.opening_enabled
            else (
                "WALL_INIT"
                if self.storage_wall_guided_enabled and self.wall_initialization_enabled
                else "SCAN"
            )
        )
        self._timed_storage_triggered = False
        self._storage_route_completed = False
        self._storage_route_phase = "face_bottom_wall"
        self._storage_staging_heading: float | None = None
        self._storage_reverse_heading: float | None = self.storage_right_heading_rad
        self._storage_pose_fallback_key: str | None = None
        self._storage_pose_fallback_start_s = 0.0
        self._storage_last_chance_active = False
        self._storage_last_chance_phase = "idle"
        self._storage_last_chance_started_s = 0.0
        self._storage_last_chance_phase_started_s = 0.0
        self._storage_last_chance_turn_heading: float | None = None
        self._wall_segments: list[tuple[float, float, float, float]] = []
        self._wall_segments_s = 0.0
        self._wall_segments_seq = 0
        self._storage_wall_confirm_count = 0
        self._storage_wall_confirm_last_seq = -1
        self._wall_initialization_last_seq = -1
        self._wall_initialization_valid_frames = 0
        self._wall_initialization_confirmed_s: float | None = None
        self._storage_wall_warmup_last_seq = -1
        self._storage_wall_warmup_valid_frames = 0
        self._arm_pick_phase = "UNKNOWN"
        self._arm_pick_phase_rx_s = -math.inf
        self._pending_pick: tuple[int, str, str] | None = None
        self._local_blacklisted_ids: set[int] = set()
        self._opening_leg = "wait"
        self._opening_turn_target: float | None = None
        self._opening_heading_stable_count = 0
        self._opening_heading_debug_seq = 0
        self._opening_heading_checked_seq = 0
        self._opening_heading_debug: tuple[float, float, float, float] | None = None
        self._opening_heading_debug_s: float | None = None
        self._opening_position_debug: tuple[float, float, float] | None = None
        self._opening_position_debug_s: float | None = None
        self._opening_position_debug_seq = 0
        self._opening_position_checked_seq = 0
        self._opening_position_stable_count = 0
        self._opening_wall_heading_valid = False
        self._wall_translation_unlocked = False
        self._world_mapping_enabled = False
        self._global_mapping_enabled_since_s = -math.inf
        self._got_world = False         # set on first /world_model (perception up)
        self._world_seq = 0
        self._world_last_rx_s = -math.inf
        self._localization_pose_seq = 0
        self._localization_pose_last_s = -math.inf
        self._localization_pose: tuple[float, float, float] | None = None
        self._localization_pose_history: list[tuple[float, float, float, float]] = []
        self._imu_last_s = -math.inf
        self._process_start_s = self._now_s()
        # Mission/match t=0 is overwritten on the first latched RUNNING button event.
        # Keep process uptime separate so deadline logs cannot be confused with launch time.
        self._node_start_s = self._process_start_s
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
        self.siglip_stamp_s: float | None = None      # source image capture time of last SigLIP
        self.siglip_rx_s: float | None = None         # subscriber receive time after inference
        self._set2_identity_latch: Set2IdentityLatch | None = None
        self._set2_identity_visit_serial = 0
        self._set2_identity_visit_started_s = -math.inf
        self._set2_identity_visit_target_id = 0
        self._set2_identity_visit_target_xy: tuple[float, float] | None = None
        self.shape_stamp_s: float | None = None        # arrival time (node clock) of last shape

        # time bookkeeping (seconds, node clock)
        self.state_enter_s = self._now_s()

        self.create_subscription(WorldModel, "/world_model", self.on_world, 10)
        self.create_subscription(
            PoseStamped,
            "/localization/pose",
            self.on_localization_pose,
            10,
        )
        self.create_subscription(
            WorldModel,
            "/world_model/wide_relative_objects",
            self.on_relative_wide,
            1,
        )
        self.create_subscription(Object, "/selected_target", self.on_target, 10)
        self.create_subscription(Classification, "/classification/siglip", self.on_siglip, 10)
        self.create_subscription(Classification, "/classification/shape", self.on_shape, 10)
        self.create_subscription(DetectionArray, "/camera_body/detections", self.on_body_dets, 10)
        self.create_subscription(Empty, "/state_advance", self.on_advance, 10)
        self.create_subscription(
            Float32MultiArray, "/localization/wall_map_transform", self.on_wall_map_transform, 10
        )
        self.create_subscription(
            Float32MultiArray, "/localization/wall_raw_segments", self.on_wall_segments, 10
        )
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_heading_debug",
            self.on_wall_heading_debug,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/localization/wall_field_correction",
            self.on_wall_field_correction,
            10,
        )
        self.create_subscription(
            Float32, "/localization/imu_yaw_delta", self.on_imu_yaw_delta, 10
        )
        self.create_subscription(
            Imu,
            "/imu/data",
            self.on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/competition/state", self.on_competition_state, COMPETITION_QOS
        )
        self.create_subscription(
            String, "/arm/pick_sequence_phase", self.on_arm_pick_phase, 10
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
        self.pub_track_birth_enabled = self.create_publisher(
            Bool, "/world_model/track_birth_enabled", 10
        )
        self.pub_track_birth_slots = self.create_publisher(
            PoseArray, "/world_model/track_birth_slots", 10
        )
        self.pub_wall_fast = self.create_publisher(Bool, "/localization/wall_fast_correction", 10)
        self.pub_wall_mode = self.create_publisher(
            String, "/localization/wall_correction_mode", 10
        )
        self.pub_wall_processing = self.create_publisher(
            Bool, "/localization/wall_processing_enabled", 10
        )
        # Debug/operator timing: [process uptime, match elapsed since RUNNING,
        # timed-storage trigger, last-chance trigger, match duration].
        self.pub_match_time = self.create_publisher(Float32MultiArray, "/planning/match_time", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"FSM started at STANDBY; RUNNING will initialize {self._run_initial_state}  "
            f"set1='{self.set1_label}' set2='{self.set2_label}' "
            f"conf>={self.conf_threshold} approach<{self.approach_dist_m}m rate={rate}Hz "
            f"dry_pick={self.dry_pick} end_after_quota={self.end_after_quota} "
            f"anchor_mission={self.anchor_mission_enabled} "
            f"zone_mission={self.zone_mission_enabled} zone={self._active_zone_id()} "
            f"local_anchor={self.local_anchor_enabled} "
            f"select_timeout={self.select_timeout_sec}s set2_slots={self.set2_slot_enabled} "
            f"mixed_target_mode={self.mixed_target_mode}"
        )

    # ------------------------------------------------------------------ utils
    def _build_local_anchor_config(self) -> LocalAnchorConfig:
        """Build the embedded FSM config from its namespaced mission parameters."""
        def value(suffix: str):
            return self.get_parameter(f"local_anchor_{suffix}").value

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
                str(label).strip().lower()
                for label in value("fruit_inspection_labels")
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
            # One authority only: the embedded target always follows today's Set2 label.
            target_fruit_label=self.set2_label,
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
            align_adaptive_steps_enabled=bool(value("align_adaptive_steps_enabled")),
            align_mid_error_m=float(value("align_mid_error_m")),
            align_fwd_mid_pulse_sec=float(value("align_fwd_mid_pulse_sec")),
            align_strafe_mid_pulse_sec=float(value("align_strafe_mid_pulse_sec")),
            align_settle_sec=float(value("align_settle_sec")),
            align_target_timeout_sec=float(value("align_target_timeout_sec")),
            align_timeout_sec=float(value("align_timeout_sec")),
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _process_elapsed_s(self) -> float:
        return max(0.0, self._now_s() - getattr(self, "_process_start_s", self._node_start_s))

    def on_competition_state(self, msg: String) -> None:
        """Hold safely through STANDBY/READY and start OPENING once on RUNNING."""
        state = str(msg.data).strip().upper()
        if state not in {"STANDBY", "READY", "RUNNING"}:
            return
        self._competition_state = state
        if state == "RUNNING" and not self._run_started:
            self._start_competition_run()

    def on_arm_pick_phase(self, msg: String) -> None:
        self._arm_pick_phase = str(msg.data).strip().upper() or "UNKNOWN"
        self._arm_pick_phase_rx_s = self._now_s()

    def _start_competition_run(self) -> None:
        """Establish the mission clock and pristine opening state at competition t=0."""
        now = self._now_s()
        self._run_started = True
        # This is the official match clock zero: the second accepted button press drives
        # /competition/state to RUNNING, the same moment OPENING begins moving.
        self._node_start_s = now
        self.state = self._run_initial_state
        self.state_enter_s = now
        self.phase = 2 if self.start_phase == 2 else 1
        self.tray_shape = 0
        self.tray_fruit = 0
        self.current_target = None
        self.current_slot_id = None
        self.selected = None
        self.set_type = 0
        self._opportunistic_set2_active = False
        self._global_seen_nodes.clear()
        self._global_retarget_last_s = 0.0
        self._global_seen_wide_seq = -1
        self._global_patrol_goal = None
        self._global_patrol_goal_zigzag_index = None
        self._global_zigzag_waypoints = list(self.global_zigzag_waypoints)
        self._global_zigzag_idx = 0
        self._global_initial_sweep_start_xy = None
        self._global_initial_sweep_complete = not bool(self._global_zigzag_waypoints)
        self._global_patrol_observe_seq = -1
        self._global_patrol_observe_start_s = 0.0
        self._timed_storage_triggered = False
        self._pending_pick = None
        self._local_blacklisted_ids.clear()
        self._slot_body_lost_retry_counts.clear()
        self._global_body_lost_retry_counts.clear()
        self._body_lost_lateral_sweep_counts.clear()
        self._global_deferred_targets.clear()
        self._last_drive_route_blocked = False
        self._last_drive_command = (0.0, 0.0, 0.0)
        self._reset_stuck_escape()
        self._approach_route_blocked_since_s = None
        self._approach_route_blocked_attempts = 0
        self.siglip_stamp_s = None
        self.siglip_rx_s = None
        self._set2_identity_latch = None
        self._set2_identity_visit_serial = 0
        self._set2_identity_visit_started_s = -math.inf
        self._set2_identity_visit_target_id = 0
        self._set2_identity_visit_target_xy = None
        self._localization_pose_history.clear()
        self._storage_route_completed = False
        self._storage_route_phase = "face_bottom_wall"
        self._storage_staging_heading = None
        self._storage_reverse_heading = self.storage_right_heading_rad
        self._storage_pose_fallback_key = None
        self._storage_pose_fallback_start_s = 0.0
        self._storage_last_chance_active = False
        self._storage_last_chance_phase = "idle"
        self._storage_last_chance_started_s = 0.0
        self._storage_last_chance_phase_started_s = 0.0
        self._storage_last_chance_turn_heading = None
        self._reset_storage_wall_confirmation()
        self._wall_initialization_last_seq = self._wall_segments_seq
        self._wall_initialization_valid_frames = 0
        self._wall_initialization_confirmed_s = None
        self._storage_wall_warmup_last_seq = self._wall_segments_seq
        self._storage_wall_warmup_valid_frames = 0
        self._opening_leg = "wait"
        self._opening_turn_target = None
        self._opening_heading_stable_count = 0
        self._opening_heading_debug_seq = 0
        self._opening_heading_checked_seq = 0
        self._opening_heading_debug = None
        self._opening_heading_debug_s = None
        self._opening_position_debug = None
        self._opening_position_debug_s = None
        self._opening_position_debug_seq = 0
        self._opening_position_checked_seq = 0
        self._opening_position_stable_count = 0
        self._opening_wall_heading_valid = False
        self._wall_translation_unlocked = False
        self._world_mapping_enabled = self.state not in {"OPENING", "WALL_INIT"}
        self._global_mapping_enabled_since_s = (
            now if self._world_mapping_enabled else -math.inf
        )
        self._opening_zone_entry_pending = self.opening_enabled
        self._opening_zone_entry_path = None
        self._opening_zone_entry_start = None
        self._opening_zone_entry_idx = 0
        self._zone_anchor_slowdown_active = False
        self._opening_anchor_strafe_phase = "align"
        self._opening_anchor_strafe_phase_start_s = 0.0
        self._opening_anchor_strafe_pulses = 0
        self._opening_anchor_strafe_vy = 0.0
        self._opening_anchor_strafe_pulse_duration_sec = 0.0
        self._clear_zone_ambiguous_exclusions()
        self._local_anchor_fsm = LocalAnchorFruitFsm(self.local_anchor_config)
        self._local_anchor_visited_zones.clear()
        self._local_anchor_active_zone_id = None
        self._local_anchor_relative_yaw = 0.0
        self._local_anchor_start_pose = None
        self._local_anchor_pick_triggered = False
        self._local_anchor_pick_accounted = False
        self._local_anchor_pose_baseline_seq = self._localization_pose_seq
        self._local_anchor_world_baseline_seq = self._world_seq
        self._local_anchor_pose_refresh_required = False
        self._plan = None
        self._plan_start_xy = None
        self._post_pick_lane_entry_pending = False
        self._lane_heading_segment_key = None
        self._lane_heading_phase = "align"
        self._lane_heading_filtered = None
        self._lane_heading_violation_start_s = None
        self._lane_heading_realign_armed_at_s = 0.0
        self._lane_heading_settle_start_s = 0.0
        self._lane_heading_entry_start_s = 0.0
        self._lane_heading_turn_sign = 0
        self._lane_heading_reverse_start_s = 0.0
        self._lane_cross_track_filtered = None
        self._lane_cross_track_outside = False
        self._reset_object_connector_brake()
        self._plan_escape = False
        self._plan_flexible_fallback = False
        self._last_drive_route_blocked = False
        self._search_phase = "look"
        self._search_t0 = now
        self._relative_anchor_tracker.reset()
        self._relative_acquisition_started_s = None
        self._relative_acquisition_first_frame_s = None
        self._relative_last_frame_s = None
        self._relative_last_observation_s = None
        self._relative_motion_last_s = now
        self._relative_last_command = (0.0, 0.0, 0.0)
        self.get_logger().info(
            f"competition RUNNING: mission t=0 -> {self.state}"
        )

    def _time_in_state(self) -> float:
        return self._now_s() - self.state_enter_s

    def _wall_fast_correction_requested(self) -> bool:
        """Request fast wall correction only in states intended to be stationary."""
        if self.state == "WALL_INIT":
            return True
        if self.state == "OPENING":
            return self._opening_leg in {
                "heading_observe", "heading_verify", "position_observe", "settle"
            }
        if self.state in {
            "ANCHOR_OBSERVE",
            "ANCHOR_FACE_TARGET",
            "ANCHOR_FACE_NEXT",
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

    def _wall_correction_mode_requested(self) -> str:
        """Initialize heading first, then x/y; never mix their opening convergence phases."""
        if self.state == "WALL_INIT":
            return "FULL"
        if self.state == "OPENING":
            if self._opening_leg in {"heading_observe", "heading_verify"}:
                return "HEADING_ONLY"
            if self._opening_leg == "position_observe":
                return "TRANSLATION_ONLY"
            if self._opening_leg == "settle" and self._wall_translation_unlocked:
                return "TRANSLATION_ONLY"
            return "OFF"
        return "TRANSLATION_ONLY" if self._wall_translation_unlocked else "OFF"

    def _wall_processing_requested(self) -> bool:
        """Run expensive wall segmentation only while it can affect startup or parking."""
        if not self.storage_wall_guided_enabled:
            # Preserve explicit wall-localizer diagnostics when wall-guided storage is disabled.
            return True
        return self.state in {
            "WALL_INIT", "DRIVE_TO_STORAGE", "ALIGN_OVER_BIN", "DUMP_ALL", "END"
        }

    def _step_wall_initialization(self) -> None:
        """Calibrate the initial field pose before any object is committed to the map."""
        self._drive(0.0, 0.0, 0.0)
        now = self._now_s()
        elapsed = self._time_in_state()
        if self._wall_segments_seq != self._wall_initialization_last_seq:
            self._wall_initialization_last_seq = self._wall_segments_seq
            if self._wall_segments:
                self._wall_initialization_valid_frames += 1
            else:
                self._wall_initialization_valid_frames = 0

        enough_frames = (
            self._wall_initialization_valid_frames
            >= self.wall_initialization_confirm_frames
        )
        if (
            self._wall_initialization_confirmed_s is None
            and elapsed >= self.wall_initialization_min_observe_sec
            and enough_frames
        ):
            self._wall_initialization_confirmed_s = now
            self._decide(
                "WALL INIT CONFIRMED -> SETTLE "
                f"{self.wall_initialization_settle_sec:.1f}s"
            )

        settled = (
            self._wall_initialization_confirmed_s is not None
            and now - self._wall_initialization_confirmed_s
            >= self.wall_initialization_settle_sec
        )
        timed_out = elapsed >= self.wall_initialization_timeout_sec
        if not settled and not timed_out:
            return
        if timed_out and not settled:
            self.get_logger().warn(
                "initial wall calibration timed out; starting exploration with current pose"
            )
            self._decide("WALL INIT TIMEOUT -> START ZIGZAG")
        else:
            self._decide("WALL INIT COMPLETE -> START ZIGZAG")
        self._set_world_mapping_enabled(True, "initial wall calibration complete")
        self._enter("SCAN")

    def _enter(self, new_state: str) -> None:
        prev = self.state
        now = self._now_s()
        if (
            new_state == "DRIVE_TO_STORAGE"
            and self.timed_storage_enabled
            and self._run_started
            and not self._timed_storage_triggered
            and not self._storage_route_completed
        ):
            new_state = "WAIT_FOR_STORAGE"
        if (
            self.anchor_mission_enabled
            and self._anchor_current_slot_id is not None
            and prev in {"APPROACH", "ALIGN", "CLASSIFY"}
            and new_state in {"SCAN", "SELECT_TARGET"}
        ):
            self._anchor_defer_unconfirmed(f"{prev.lower()} incomplete")
            new_state = "ANCHOR_RETURN"
        if (
            prev == LOCAL_ANCHOR_STATE
            and new_state != LOCAL_ANCHOR_STATE
            and self._local_anchor_fsm.state not in LOCAL_ANCHOR_TERMINAL_STATES
        ):
            self._local_anchor_fsm.abort(now, f"mission transition to {new_state}")
        self.state = new_state
        self.state_enter_s = now
        self._plan = None                       # a state change invalidates the active travel plan
        self._plan_start_xy = None
        self._lane_heading_segment_key = None
        self._lane_heading_phase = "align"
        self._lane_heading_filtered = None
        self._lane_heading_violation_start_s = None
        self._lane_heading_realign_armed_at_s = 0.0
        self._lane_heading_settle_start_s = 0.0
        self._lane_heading_entry_start_s = 0.0
        self._lane_heading_turn_sign = 0
        self._lane_heading_reverse_start_s = 0.0
        self._lane_cross_track_filtered = None
        self._lane_cross_track_outside = False
        self._reset_object_connector_brake()
        self._plan_escape = False
        self._plan_flexible_fallback = False
        self._reset_stuck_escape()
        if new_state == "DRIVE_TO_STORAGE":
            self._storage_route_phase = "face_bottom_wall"
            self._storage_staging_heading = None
            self._storage_reverse_heading = self.storage_right_heading_rad
            self._storage_pose_fallback_key = None
            self._storage_pose_fallback_start_s = 0.0
            self._storage_last_chance_active = False
            self._storage_last_chance_phase = "idle"
            self._storage_last_chance_started_s = 0.0
            self._storage_last_chance_phase_started_s = 0.0
            self._storage_last_chance_turn_heading = None
            self._reset_storage_wall_confirmation()
            self._storage_wall_warmup_last_seq = self._wall_segments_seq
            self._storage_wall_warmup_valid_frames = 0
            self._reset_pulsed_heading()
        if new_state == "ALIGN":
            self._align_phase = "measure"       # start each ALIGN by measuring the settled position
            self._align_heading_target = None
            self._align_body_lost_backoff_attempts = 0
            self._align_body_lost_sweep_strafe_sign = 0.0
            self._align_arrival_geometry_logged = False
            self._reset_align_body_loss_confirmation()
            self._reset_pulsed_heading()
            self._reset_align_body_presence()
            self._reset_global_target_absence()
            if self._current_anchor_slot() is not None:
                self._reset_anchor_body_confirmation(clear_latched=True)
        if new_state == "CLASSIFY":
            self._classify_verify_phase = "settle"
            self._classify_verify_phase_start_s = self.state_enter_s
            self._classify_verify_retry_count = 0
            self._reset_set1_final_classification()
        if new_state == "APPROACH":
            self._reset_global_target_absence()
            self._set2_track_loss_align_override_logged = False
            target = self.current_target
            self._global_approach_committed_from_wide = bool(
                self.global_target_mode
                and target is not None
                and (
                    (
                        int(target.set_type) == 1
                        and str(target.class_label) == str(self.set1_label)
                    )
                    or (
                        int(target.set_type) == 2
                        and bool(str(target.fruit_label))
                        and str(target.fruit_label) == str(self.set2_label)
                    )
                )
            )
            if self.current_target is not None and int(self.current_target.set_type) == 2:
                self._begin_set2_identity_visit()
            else:
                self._set2_identity_latch = None
                self._set2_identity_visit_started_s = -math.inf
                self._set2_identity_visit_target_id = 0
                self._set2_identity_visit_target_xy = None
            self._reset_plain_cube_reobservation()
            anchor_slot = self._current_anchor_slot()
            object_slot = self._current_slot()
            self._appr_tgt_xy = (
                (anchor_slot.x, anchor_slot.y)
                if anchor_slot is not None
                else (
                    (object_slot.x, object_slot.y)
                    if object_slot is not None
                    else (
                        (float(self.current_target.x), float(self.current_target.y))
                        if self.current_target is not None else None
                    )
                )
            )
            self._appr_last_seen_s = self._now_s()
            self._standoff_arrived_s = None
            self._reset_approach_motion()
            if self._appr_tgt_xy is not None:
                self._latch_approach_goal(self._appr_tgt_xy)
        if new_state == "ANCHOR_OBSERVE":
            self.current_target = None
            self._anchor_current_slot_id = None
            self._anchor_observe_start_s = self._now_s()
            self._relative_observe_hit_baseline = {}
            if self._relative_mode() and self._relative_anchor_tracker.locked:
                for slot in self.anchor_inventory.anchor_slots(self._active_anchor_id()):
                    evidence = self._relative_anchor_tracker.slot_evidence(
                        slot.anchor_id, slot.slot_index
                    )
                    self._relative_observe_hit_baseline[slot.slot_id] = (
                        evidence.hits if evidence is not None else 0
                    )
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
            f"slot_id={self._anchor_current_slot_id or self.current_slot_id or 0}"
        )

    def _lookup_object(self, obj_id: int) -> Object | None:
        """Fresh world-model entry for a tracker id (None if not present)."""
        if (
            self.world is None
            or obj_id == 0
            or int(obj_id) in getattr(self, "_local_blacklisted_ids", set())
        ):
            return None
        for obj in self.world.objects:
            if obj.id == obj_id:
                return obj
        return None

    def _is_locally_blacklisted(self, obj_id: int) -> bool:
        return int(obj_id) in getattr(self, "_local_blacklisted_ids", set())

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
            if (
                o.blacklisted
                or int(o.id) in getattr(self, "_local_blacklisted_ids", set())
                or o.set_type != set_type
            ):
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
        if self.global_target_mode:
            self._global_patrol_step()                   # empty map -> sweep unseen grid nodes only
            return
        if self.zone_mission_enabled:
            if not self.zone_anchor_nav_enabled:
                self._search_step()
                return
            if self._zone_anchor_reached_idx == self._zone_idx:
                self._search_step()
                return
            entry_anchor = self._zone_entry_anchor_point()
            if entry_anchor is None:
                self._drive(0.0, 0.0)
                return
            zx, zy = entry_anchor
            if (d := self._distance_to(zx, zy)) is not None and d <= self.zone_center_reach_tol_m:
                self._zone_anchor_reached_idx = self._zone_idx
                self._search_step()
            else:
                th = self.world.robot_theta if self.world is not None else 0.0
                self._drive_toward(
                    zx,
                    zy,
                    th,
                    exclude_id=0,
                    route_mode=self._next_travel_route_mode(),
                )
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
        self._drive_toward(
            wp[0],
            wp[1],
            th,
            exclude_id=0,
            route_mode=self._next_travel_route_mode(),
        )
        if (d := self._distance_to(wp[0], wp[1])) is not None and d < self.patrol_reach_tol:
            self._coverage_idx += 1
            self._plan = None                            # next sweep waypoint replans its route
            if self._coverage_idx >= len(self._coverage):
                self._coverage = None                    # sweep done -> rebuild (map grew) & loop
                self._coverage_idx = 0

    def _body_label(self) -> str:
        """Body-cam YOLO label to ALIGN to this phase: the set1 shape (phase 1), or the generic fruit
        cube (phase 2 — the specific fruit is confirmed separately by SigLIP in CLASSIFY)."""
        if self.anchor_mission_enabled and (slot := self._current_anchor_slot()) is not None:
            set_type = int(self.current_target.set_type) if self.current_target is not None else int(slot.set_type)
            if set_type == 2:
                return "fruit_photo_cube"
            label = str(self.current_target.class_label) if self.current_target is not None else ""
            return str(label or slot.class_label or "cube")
        return "fruit_photo_cube" if self._opportunistic_set2_active or self.phase == 2 else self.set1_label

    def _body_align_labels(self) -> set[str]:
        if self.anchor_mission_enabled and (slot := self._current_anchor_slot()) is not None:
            wide_uncertain = bool(
                int(slot.set_type) not in {1, 2}
                or not str(slot.class_label)
                or float(slot.confidence) < self.body_slot_wide_low_confidence
            )
            if wide_uncertain:
                # Low-confidence Wide identity may be corrected, but only inside the hard
                # current-slot geometry gate.
                return set(_LABEL_ST)
            if int(slot.set_type) == 2:
                # A trusted Wide fruit slot may appear as generic cube in Body, but an
                # unrelated shape label must not override its set type.
                return {"fruit_photo_cube", "cube"}
            # The only trusted Set1 slot sent to Body is today's target.  Body may reject its
            # exact shape identity, while a fruit label cannot replace the trusted set type.
            return {"cube", "octahedron", "dodecahedron", "icosahedron"}
        if self._opportunistic_set2_active or self.phase == 2:
            return {"fruit_photo_cube", "cube"}
        return {self.set1_label}

    # ------------------------------------------------------ fixed three-anchor mission
    def _active_anchor_id(self) -> int:
        return min(3, max(1, self._anchor_idx + 1))

    def _active_anchor(self):
        return self.anchor_inventory.get_anchor(self._active_anchor_id())

    def _current_anchor_slot(self) -> AnchorSlot | None:
        if not self.anchor_mission_enabled or self._anchor_current_slot_id is None:
            return None
        try:
            return self.anchor_inventory.get(self._anchor_current_slot_id)
        except KeyError:
            return None

    def _relative_mode(self) -> bool:
        return bool(self.anchor_mission_enabled and self.relative_anchor_nav_enabled)

    def _relative_hold(self, reason: str) -> None:
        """Fail closed when the camera-relative lattice is not trustworthy."""
        self._drive(0.0, 0.0, 0.0)
        if reason != self._relative_wait_reason:
            self._relative_wait_reason = reason
            self._decide(f"RELATIVE NAV HOLD ({reason})")
            self.get_logger().warn(f"relative anchor navigation holding: {reason}")

    def _relative_try_acquire(self) -> bool:
        """Run one stationary scan window, timed from the first valid post-opening WIDE frame."""
        if not self._relative_mode():
            return True
        if self._relative_anchor_tracker.locked:
            return True
        now = self._now_s()
        if self._relative_acquisition_started_s is None:
            # Ignore every frame captured while OPENING was moving.  Only this explicitly
            # stationary window is allowed to define the virtual grid.
            self._relative_anchor_tracker.reset()
            self._relative_acquisition_started_s = now
            self._relative_acquisition_first_frame_s = None
            self._relative_last_frame_s = None
            self._relative_last_observation_s = None
            self._relative_hold(
                f"acquiring wide lattice {self.relative_anchor_acquire_sec:.1f}s"
            )
            return False
        if self._relative_acquisition_first_frame_s is None:
            self._relative_hold("waiting for first post-opening wide frame")
            return False
        elapsed = now - self._relative_acquisition_first_frame_s
        if elapsed < self.relative_anchor_acquire_sec:
            self._drive(0.0, 0.0, 0.0)
            return False
        if (
            self._relative_last_frame_s is None
            or now - self._relative_last_frame_s > self._relative_anchor_tracker.stale_timeout
        ):
            self._relative_hold("wide relative stream is stale during acquisition")
            return False
        if (
            self._relative_last_observation_s is not None
            and now - self._relative_last_observation_s
            > self._relative_anchor_tracker.stale_timeout
        ):
            self._relative_last_observation_s = None
        # Once wall initialization has established the global heading, express the field-grid axis
        # in the current base frame from that estimate instead of assuming an exact 45-degree turn.
        expected_grid_yaw = (
            self._wrap_pi(-float(self.world.robot_theta))
            if self.world is not None
            else self.relative_anchor_grid_yaw_base
        )
        if self._relative_anchor_tracker.lock(
            expected_k1_center=self.relative_anchor_expected_k1_base,
            expected_grid_yaw=expected_grid_yaw,
            min_cluster_hits=self.relative_anchor_min_cluster_hits,
            min_points=self.relative_anchor_min_points,
            center_gate_m=self.relative_anchor_center_gate_m,
            allow_prior_fallback=True,
            now_s=now,
        ):
            self._relative_wait_reason = ""
            estimate = self._relative_anchor_tracker.estimate(1, now_s=now)
            self._decide(
                f"RELATIVE GRID LOCKED points={estimate.matched_count} "
                f"K1=({estimate.center_x:.2f},{estimate.center_y:.2f})"
            )
            self.get_logger().info(
                f"relative grid locked from "
                f"{self._relative_anchor_tracker.acquisition_cluster_count} clusters; "
                f"K1 base=({estimate.center_x:.3f},{estimate.center_y:.3f})"
            )
            # The user's first two-second stationary view is K1's one-shot inventory.  Geometry
            # lock and physical arrival remain separate, so this never skips the drive to K1.
            self.anchor_inventory.lock_anchor_snapshot(
                1,
                self._relative_initial_snapshot_observations(1),
                now_s=now,
            )
            self._anchor_resolve_wide_trusted(1)
            counts = self.anchor_inventory.counts(1)
            self._decide(
                f"ANCHOR K1 INITIAL SNAPSHOT objects={counts['objects']} "
                f"empty={counts['empty']} pending={counts['pending']}"
            )
            return True
        self._relative_hold(
            "could not lock four virtual slots from the post-opening scan"
        )
        return False

    def _relative_estimate(self, anchor_id: int | None = None):
        if not self._relative_mode() or not self._relative_anchor_tracker.locked:
            return None
        return self._relative_anchor_tracker.estimate(
            self._active_anchor_id() if anchor_id is None else int(anchor_id),
            now_s=self._now_s(),
        )

    def _relative_slot_point(self, slot: AnchorSlot) -> tuple[float, float] | None:
        if not self._relative_mode():
            return None
        return self._relative_anchor_tracker.slot_point(slot.anchor_id, slot.slot_index)

    def _sync_relative_landmark_exclusions(self) -> None:
        if not self._relative_mode():
            return
        excluded = {
            (slot.anchor_id, slot.slot_index)
            for slot in self.anchor_inventory.slots
            if slot.state == "PICKED"
        }
        current = self._current_anchor_slot()
        if current is not None:
            # The object being approached/aligned/picked may move; it must never drag the map.
            excluded.add((current.anchor_id, current.slot_index))
        self._relative_anchor_tracker.set_excluded_slots(excluded)

    def _predict_relative_command_translation(self, now_s: float) -> None:
        """Move the virtual lattice by the inverse of the last commanded body translation."""
        now = float(now_s)
        dt = max(0.0, min(0.35, now - self._relative_motion_last_s))
        self._relative_motion_last_s = now
        if not self._relative_mode() or not self._relative_anchor_tracker.locked:
            return
        vx, vy, _ = self._relative_last_command
        self._relative_anchor_tracker.predict_robot_motion(
            dx_body=float(vx) * dt,
            dy_body=float(vy) * dt,
            dtheta=0.0,
            now_s=now,
        )

    def _relative_object_for_slot(self, slot: AnchorSlot, *, require_fresh: bool) -> Object | None:
        evidence = self._relative_anchor_tracker.slot_evidence(
            slot.anchor_id, slot.slot_index
        )
        if evidence is None:
            return None
        if require_fresh and self._now_s() - evidence.last_seen_s > self._relative_anchor_tracker.stale_timeout:
            return None
        obj = Object()
        obj.id = int(1_000_000 + slot.slot_id)
        obj.class_label = str(evidence.label)
        obj.set_type = int(evidence.set_type)
        # AnchorSlotInventory intentionally retains nominal field coordinates for bookkeeping.
        # Navigation never consumes these two fields in relative mode.
        obj.x = float(slot.x)
        obj.y = float(slot.y)
        obj.confidence = float(evidence.confidence)
        obj.last_seen = self.get_clock().now().to_msg()
        obj.blacklisted = False
        obj.n_obs = int(evidence.hits)
        obj.source = "wide_relative_slot"
        obj.fruit_label = (
            str(evidence.label)
            if evidence.label in {"apple", "orange", "banana", "pineapple"}
            else ""
        )
        obj.locked = True
        return obj

    def _relative_drive_to_point(self, point: tuple[float, float], stop_radius: float) -> bool:
        """Stop-turn-forward controller for a waypoint already expressed in base_link."""
        x, y = float(point[0]), float(point[1])
        distance = math.hypot(x, y)
        if distance <= max(0.01, float(stop_radius)):
            self._drive(0.0, 0.0, 0.0)
            return True
        bearing = math.atan2(y, x)
        if abs(bearing) > self.relative_anchor_face_tol_rad:
            omega = max(
                -self.direct_nav_omega_max,
                min(self.direct_nav_omega_max, self.direct_nav_kp_ang * bearing),
            )
            self._drive(0.0, 0.0, omega)
        else:
            self._drive(self.direct_nav_speed, 0.0, 0.0)
        return False

    def _relative_rotate_to_point(self, point: tuple[float, float]) -> bool:
        bearing = math.atan2(float(point[1]), float(point[0]))
        if abs(bearing) <= self.relative_anchor_face_tol_rad:
            self._drive(0.0, 0.0, 0.0)
            return True
        omega = max(
            -self.direct_nav_omega_max,
            min(self.direct_nav_omega_max, self.direct_nav_kp_ang * bearing),
        )
        self._drive(0.0, 0.0, omega)
        return False

    def _relative_entry_point(self, anchor_id: int) -> tuple[float, float] | None:
        """Nearest outside-edge midpoint, preserving a collision-free gap into the anchor."""
        estimate = self._relative_estimate(anchor_id)
        if estimate is None or len(estimate.slots) != 4:
            return None
        edges = ((0, 1), (2, 3), (0, 2), (1, 3))
        edge = self._relative_entry_edges.get(int(anchor_id))
        candidates: list[tuple[float, tuple[int, int], tuple[float, float]]] = []
        for first, second in edges:
            ax, ay = estimate.slots[first]
            bx, by = estimate.slots[second]
            mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
            ox, oy = mx - estimate.center_x, my - estimate.center_y
            norm = math.hypot(ox, oy)
            if norm > 1e-9:
                mx += self.relative_anchor_entry_margin_m * ox / norm
                my += self.relative_anchor_entry_margin_m * oy / norm
            candidates.append((math.hypot(mx, my), (first, second), (mx, my)))
        if edge is None:
            _, edge, point = min(candidates, key=lambda value: value[0])
            self._relative_entry_edges[int(anchor_id)] = edge
            return point
        for _, candidate_edge, point in candidates:
            if candidate_edge == edge:
                return point
        return None

    def _relative_drive_to_anchor(self, anchor_id: int, *, use_entry: bool) -> bool:
        estimate = self._relative_estimate(anchor_id)
        if estimate is None:
            self._relative_hold("relative lattice not locked")
            return False
        if not estimate.navigable:
            self._relative_hold(
                f"{estimate.reason} age={estimate.age_s:.2f}s matched={estimate.matched_count}"
            )
            return False
        self._relative_wait_reason = ""
        if use_entry and int(anchor_id) not in self._relative_entry_reached:
            entry = self._relative_entry_point(anchor_id)
            if entry is None:
                self._relative_hold("cannot construct virtual anchor entry")
                return False
            # If the robot is already inside the four-slot square, do not command it back out.
            if math.hypot(estimate.center_x, estimate.center_y) <= 0.35:
                self._relative_entry_reached.add(int(anchor_id))
            elif self._relative_drive_to_point(
                entry, self.relative_anchor_entry_reach_tol_m
            ):
                self._relative_entry_reached.add(int(anchor_id))
            return False
        return self._relative_drive_to_point(
            (estimate.center_x, estimate.center_y), self.anchor_reach_tol_m
        )

    def _anchor_snapshot_observations(self) -> list[Object]:
        """Objects freshly seen during this anchor's first and only capture window."""
        if self._relative_mode():
            result: list[Object] = []
            for slot in self.anchor_inventory.anchor_slots(self._active_anchor_id()):
                evidence = self._relative_anchor_tracker.slot_evidence(
                    slot.anchor_id, slot.slot_index
                )
                baseline = self._relative_observe_hit_baseline.get(slot.slot_id, 0)
                if (
                    evidence is None
                    or evidence.hits - baseline < self.relative_anchor_slot_min_hits
                    or evidence.last_seen_s < self._anchor_observe_start_s
                ):
                    continue
                obj = self._relative_object_for_slot(slot, require_fresh=True)
                if obj is not None:
                    result.append(obj)
            return result
        if self.world is None:
            return []
        return [
            obj for obj in self.world.objects
            if not bool(obj.blacklisted)
            and int(obj.set_type) in {1, 2}
            and self._time_msg_to_sec(obj.last_seen) >= self._anchor_observe_start_s
        ]

    def _relative_initial_snapshot_observations(self, anchor_id: int) -> list[Object]:
        result: list[Object] = []
        for slot in self.anchor_inventory.anchor_slots(int(anchor_id)):
            evidence = self._relative_anchor_tracker.slot_evidence(
                slot.anchor_id, slot.slot_index
            )
            if evidence is None or evidence.hits < self.relative_anchor_slot_min_hits:
                continue
            obj = self._relative_object_for_slot(slot, require_fresh=True)
            if obj is not None:
                result.append(obj)
        return result

    def _anchor_track_for_slot(self, slot: AnchorSlot) -> Object | None:
        """Reattach identity only inside the frozen slot; never substitute a global target."""
        if self._relative_mode():
            return self._relative_object_for_slot(slot, require_fresh=True)
        if self.world is None:
            return None
        candidates: list[Object] = []
        if slot.track_id:
            exact = self._lookup_object(int(slot.track_id))
            if exact is not None:
                candidates.append(exact)
        candidates.extend(self.world.objects)
        seen: set[int] = set()
        best = None
        best_dist = max(0.0, self.anchor_track_attach_radius_m)
        for obj in candidates:
            oid = int(obj.id)
            if oid in seen:
                continue
            seen.add(oid)
            if bool(obj.blacklisted) or int(obj.set_type) not in {1, 2}:
                continue
            dist = math.hypot(float(obj.x) - slot.x, float(obj.y) - slot.y)
            if dist <= best_dist:
                best = obj
                best_dist = dist
        return best

    @staticmethod
    def _anchor_snapshot_object(slot: AnchorSlot) -> Object:
        """Fallback target made from the locked observation when its live track briefly drops."""
        obj = Object()
        obj.id = int(slot.track_id)
        obj.class_label = str(slot.class_label)
        obj.set_type = int(slot.set_type)
        obj.x = float(slot.x)
        obj.y = float(slot.y)
        obj.confidence = float(slot.confidence)
        obj.blacklisted = False
        obj.n_obs = 1
        obj.source = "anchor_snapshot"
        obj.fruit_label = str(slot.fruit_label)
        obj.locked = True
        return obj

    def _anchor_slot_needs_body(self, slot: AnchorSlot) -> bool:
        return slot_requires_body_confirmation(
            slot,
            set1_label=self.set1_label,
            wide_low_confidence=self.body_slot_wide_low_confidence,
        )

    def _anchor_resolve_wide_trusted(self, anchor_id: int) -> None:
        """Close high-confidence Wide non-targets without unnecessary Body visits."""
        now = self._now_s()
        for slot in self.anchor_inventory.anchor_slots(anchor_id):
            if slot.state != "OCCUPIED" or self._anchor_slot_needs_body(slot):
                continue
            self.anchor_inventory.mark_checked(
                slot.slot_id,
                target=False,
                class_label=slot.class_label,
                fruit_label=slot.fruit_label,
                confidence=slot.confidence,
                now_s=now,
            )
            self._decide(
                f"ANCHOR K{slot.anchor_id}.{slot.slot_index} WIDE TRUSTED "
                f"{slot.class_label} conf={slot.confidence:.2f} -> NO BODY"
            )

    def _reset_anchor_body_confirmation(self, *, clear_latched: bool) -> None:
        self._anchor_body_last_seq = self._body_dets_seq
        self._anchor_body_confirm_count = 0
        self._anchor_body_missing_count = 0
        self._anchor_body_candidate_label = ""
        self._anchor_body_candidate_confidence = 0.0
        if clear_latched:
            self._anchor_body_confirmed_label = ""
            self._anchor_body_confirmed_confidence = 0.0

    def _anchor_note_body_candidate(
        self,
        candidate: tuple[str, float, float, float] | None,
    ) -> bool:
        """Count only new Body frames and latch an identity after a stable streak."""
        if self._body_dets_seq == self._anchor_body_last_seq:
            return bool(
                candidate is not None
                and self._anchor_body_confirm_count >= self.body_slot_confirm_frames
                and candidate[0] == self._anchor_body_candidate_label
            )
        self._anchor_body_last_seq = self._body_dets_seq
        if candidate is None:
            self._anchor_body_confirm_count = 0
            self._anchor_body_candidate_label = ""
            self._anchor_body_candidate_confidence = 0.0
            self._anchor_body_missing_count += 1
            return False

        label, bx, by, confidence = candidate
        self._anchor_body_missing_count = 0
        if str(label) == self._anchor_body_candidate_label:
            self._anchor_body_confirm_count += 1
            self._anchor_body_candidate_confidence = min(
                self._anchor_body_candidate_confidence,
                float(confidence),
            )
        else:
            self._anchor_body_candidate_label = str(label)
            self._anchor_body_candidate_confidence = float(confidence)
            self._anchor_body_confirm_count = 1
        if self._anchor_body_confirm_count < self.body_slot_confirm_frames:
            return False
        self._anchor_body_confirmed_label = self._anchor_body_candidate_label
        self._anchor_body_confirmed_confidence = self._anchor_body_candidate_confidence
        self._anchor_body_bound_point = (float(bx), float(by))
        return True

    def _anchor_defer_unconfirmed(self, reason: str) -> None:
        slot = self._current_anchor_slot()
        if slot is None or slot.state != "OCCUPIED":
            return
        self.anchor_inventory.mark_unconfirmed(slot.slot_id, now_s=self._now_s())
        self._align_fail_count = 0
        self._decide(
            f"ANCHOR K{slot.anchor_id}.{slot.slot_index} UNCONFIRMED ({reason})"
        )
        self.get_logger().warn(
            f"anchor K{slot.anchor_id}.{slot.slot_index}: {reason} -> UNCONFIRMED"
        )

    def _anchor_start_body_backoff_or_defer(self, now_s: float, reason: str) -> None:
        slot = self._current_anchor_slot()
        if slot is None:
            return
        count = self._anchor_body_backoff_counts.get(slot.slot_id, 0)
        if count >= self.body_slot_backoff_retries:
            self._drive(0.0, 0.0, 0.0)
            self._anchor_defer_unconfirmed(
                f"{reason} after {self.body_slot_backoff_retries} backoffs"
            )
            self._enter("ANCHOR_RETURN")
            return
        count += 1
        self._anchor_body_backoff_counts[slot.slot_id] = count
        self._reset_anchor_body_confirmation(clear_latched=True)
        self.get_logger().info(
            f"anchor K{slot.anchor_id}.{slot.slot_index}: {reason} -> "
            f"10cm backoff {count}/{self.body_slot_backoff_retries}"
        )
        self._decide(
            f"ANCHOR K{slot.anchor_id}.{slot.slot_index} BODY BACKOFF "
            f"{count}/{self.body_slot_backoff_retries}"
        )
        self._drive(-abs(self.align_body_lost_backoff_speed), 0.0, 0.0)
        self._align_phase = "body_lost_backoff"
        self._align_phase_start = float(now_s)

    def _anchor_latch_slot(self, slot: AnchorSlot) -> None:
        anchor = self._active_anchor()
        self._anchor_current_slot_id = slot.slot_id
        self._sync_relative_landmark_exclusions()
        relative_point = self._relative_slot_point(slot)
        self._anchor_slot_heading = (
            math.atan2(relative_point[1], relative_point[0])
            if relative_point is not None
            else math.atan2(slot.y - anchor.y, slot.x - anchor.x)
        )
        self.current_target = self._anchor_track_for_slot(slot) or self._anchor_snapshot_object(slot)
        self.phase = 2 if int(slot.set_type) == 2 else 1
        self._anchor_body_backoff_counts[slot.slot_id] = 0
        self._anchor_body_bound_point = None
        self._reset_anchor_body_confirmation(clear_latched=True)
        self._decide(
            f"ANCHOR K{anchor.anchor_id}.{slot.slot_index} OCCUPIED "
            f"track#{self.current_target.id if self.current_target else 0}"
        )

    def _anchor_resolve_checked(self, reason: str, *, target: bool | None = None) -> None:
        slot = self._current_anchor_slot()
        if slot is None or slot.state != "OCCUPIED":
            return
        # Once Body has confirmed the current physical slot, a fresh Wide track must not
        # overwrite that close-range identity while the result is being resolved.
        fresh = None if self._anchor_body_confirmed_label else self._anchor_track_for_slot(slot)
        if fresh is not None:
            self.current_target = fresh
        obj = self.current_target
        self.anchor_inventory.mark_checked(
            slot.slot_id,
            target=target,
            class_label=str(obj.class_label) if obj is not None else slot.class_label,
            fruit_label=str(obj.fruit_label) if obj is not None else slot.fruit_label,
            confidence=float(obj.confidence) if obj is not None else slot.confidence,
            now_s=self._now_s(),
        )
        self._align_fail_count = 0
        self._decide(f"ANCHOR K{slot.anchor_id}.{slot.slot_index} CHECKED ({reason})")

    def _anchor_clear_current(self) -> None:
        self._anchor_current_slot_id = None
        self.current_target = None
        self.set_type = 0
        self._opportunistic_set2_active = False
        self._anchor_body_bound_point = None
        self._reset_anchor_body_confirmation(clear_latched=True)
        self._sync_relative_landmark_exclusions()

    def _anchor_next_pending(self) -> AnchorSlot | None:
        pending = self.anchor_inventory.pending(self._active_anchor_id())
        if not pending:
            return None
        if self._relative_mode():
            return min(
                pending,
                key=lambda slot: (
                    abs(math.atan2(point[1], point[0]))
                    if (point := self._relative_slot_point(slot)) is not None
                    else math.inf
                ),
            )
        heading = float(self.world.robot_theta) if self.world is not None else 0.0
        anchor = self._active_anchor()
        return min(
            pending,
            key=lambda slot: abs(
                self._wrap_pi(math.atan2(slot.y - anchor.y, slot.x - anchor.x) - heading)
            ),
        )

    def _publish_track_birth_gate(self) -> None:
        slots_msg = PoseArray()
        slots_msg.header.stamp = self.get_clock().now().to_msg()
        slots_msg.header.frame_id = "field"
        if self.state == LOCAL_ANCHOR_STATE:
            # Keep existing tracks/localization alive, but do not let a rotating local sweep
            # create new field-map targets that can perturb the resumed Set1 mission.
            self.pub_track_birth_slots.publish(slots_msg)
            self.pub_track_birth_enabled.publish(Bool(data=False))
            return
        if self._relative_mode():
            # Field-map tracks are not an authority in relative mode.  Keeping this gate closed
            # also prevents late field projection drift from creating extra anchor targets.
            self.pub_track_birth_slots.publish(slots_msg)
            self.pub_track_birth_enabled.publish(Bool(data=False))
            return
        observing = (
            self.anchor_mission_enabled
            and self.state == "ANCHOR_OBSERVE"
            and not self.anchor_inventory.anchor_locked(self._active_anchor_id())
        )
        if self.anchor_mission_enabled:
            for slot in self.anchor_inventory.anchor_slots(self._active_anchor_id()):
                pose = Pose()
                pose.position.x = float(slot.x)
                pose.position.y = float(slot.y)
                pose.orientation.w = 1.0
                slots_msg.poses.append(pose)
        self.pub_track_birth_slots.publish(slots_msg)
        if self.anchor_mission_enabled:
            # Publish the slots for a few cycles before opening the gate, avoiding a cross-topic
            # arrival race where an enabled gate could briefly have no spatial restriction.
            enabled = observing and self._time_in_state() >= self.anchor_birth_delay_sec
        else:
            enabled = True
        self.pub_track_birth_enabled.publish(Bool(data=bool(enabled)))

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
            if (
                o.blacklisted
                or int(o.id) in getattr(self, "_local_blacklisted_ids", set())
                or o.set_type != want_set
            ):
                continue
            if self._is_zone_ambiguous_excluded(o):
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

    def _iter_set1_targets(self):
        """All eligible Set1 objects for selection (shared by nearest- and tour-ordering)."""
        if self.world is None or not self._shape_quota_available():
            return
        for o in self.world.objects:
            if (
                o.blacklisted
                or int(o.id) in getattr(self, "_local_blacklisted_ids", set())
                or int(o.set_type) != 1
            ):
                continue
            if self._is_zone_ambiguous_excluded(o):
                continue
            if self.global_target_mode and MissionFsmNode._global_target_is_deferred(self, o):
                continue
            if str(o.class_label) != self.set1_label:
                continue
            if self.zone_mission_enabled and not self._object_in_active_zone(o):
                continue
            if self.zone_mission_enabled and not self._object_seen_after_zone_stabilize(o):
                continue
            if float(o.confidence) < self.pick_track_conf:
                continue
            yield o

    def _iter_set2_target_objects(self):
        """Yield eligible Set2 tracks; global mode keeps hint-mismatches verifiable."""
        if self.world is None or not self._fruit_quota_available():
            return
        for o in self.world.objects:
            if (
                o.blacklisted
                or int(o.id) in getattr(self, "_local_blacklisted_ids", set())
                or int(o.set_type) != 2
            ):
                continue
            if self.zone_mission_enabled and not self._object_in_active_zone(o):
                continue
            if self.zone_mission_enabled and not self._object_seen_after_zone_stabilize(o):
                continue
            if self.global_target_mode and MissionFsmNode._global_target_is_deferred(self, o):
                continue
            fl = str(o.fruit_label)
            if (
                self.set2_require_fruit_label
                and fl
                and fl != self.set2_label
                and not self.global_target_mode
            ):
                continue
            if float(o.confidence) < self.pick_track_conf:
                continue
            yield o

    def _nearest_set1_target(self, ref_xy: tuple[float, float] | None = None) -> Object | None:
        if self.world is None:
            return None
        rx, ry = ref_xy if ref_xy is not None else (self.world.robot_x, self.world.robot_y)
        return min(
            self._iter_set1_targets(),
            key=lambda o: math.hypot(float(o.x) - rx, float(o.y) - ry),
            default=None,
        )

    def _nearest_set2_target_object(self, ref_xy: tuple[float, float] | None = None) -> Object | None:
        if self.world is None:
            return None
        rx, ry = ref_xy if ref_xy is not None else (self.world.robot_x, self.world.robot_y)
        return min(
            self._iter_set2_target_objects(),
            key=lambda o: math.hypot(float(o.x) - rx, float(o.y) - ry),
            default=None,
        )

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
        if kind != "set2_object" or (
            self._set2_identity_latch is not None
            and not self._target_matches_set2_latch(target, self._set2_identity_latch)
        ):
            self._set2_identity_latch = None
        if kind == "set1":
            self.phase = 1
            self.current_target = target
            self._global_approach_committed_from_wide = bool(self.global_target_mode)
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
            self._global_approach_committed_from_wide = bool(
                self.global_target_mode
                and str(target.fruit_label) == str(self.set2_label)
                and bool(str(target.fruit_label))
            )
            self._clear_current_slot()
            self._decide(f"MIXED SET2 {target.fruit_label or target.class_label} #{target.id}")
            return True
        return False

    # ------------------------------------------------------- global target mode
    def _global_target_in_active_corridor(self, target: Object) -> bool:
        """During the initial sweep, allow diversions only beside its fixed active leg."""
        if not getattr(self, "global_target_corridor_gate_enabled", False):
            return True
        if getattr(self, "_global_initial_sweep_complete", False):
            return True
        waypoints = getattr(self, "_global_zigzag_waypoints", [])
        if self.world is None or not waypoints:
            return True
        goal_index = getattr(self, "_global_patrol_goal_zigzag_index", None)
        if goal_index is None:
            goal_index = int(getattr(self, "_global_zigzag_idx", 0))
        goal_index = max(0, min(int(goal_index), len(waypoints) - 1))
        goal_x, goal_y = waypoints[goal_index]
        segment_start_fn = getattr(self, "_initial_sweep_segment_start", None)
        segment_start = (
            segment_start_fn(goal_index) if callable(segment_start_fn) else None
        )
        if segment_start is None:
            segment_start = (
                getattr(self, "global_zigzag_start", (-1.25, 1.60))
                if goal_index == 0
                else waypoints[goal_index - 1]
            )
        return global_target_in_route_corridor(
            (float(target.x), float(target.y)),
            (float(segment_start[0]), float(segment_start[1])),
            (float(goal_x), float(goal_y)),
            getattr(self, "global_target_corridor_radius_m", 0.45),
        )

    def _advance_global_initial_sweep(self) -> None:
        """Advance the fixed route without wrapping when it is configured as one-shot."""
        goal_index = self._global_patrol_goal_zigzag_index
        if goal_index is None or not self._global_zigzag_waypoints:
            return
        next_index = int(goal_index) + 1
        if (
            self.global_initial_sweep_once_enabled
            and next_index >= len(self._global_zigzag_waypoints)
        ):
            self._global_zigzag_idx = len(self._global_zigzag_waypoints)
            if not self._global_initial_sweep_complete:
                self._global_initial_sweep_complete = True
                self._decide(
                    "INITIAL SWEEP COMPLETE -> OPTIMIZE MAPPED TARGET TOUR"
                )
            return
        self._global_zigzag_idx = next_index % len(self._global_zigzag_waypoints)

    def _initial_sweep_segment_start(
        self,
        goal_index: int,
    ) -> tuple[float, float] | None:
        """Return the fixed straight-leg start for the active initial sweep waypoint."""
        waypoints = getattr(self, "_global_zigzag_waypoints", [])
        if goal_index <= 0:
            if self._global_initial_sweep_start_xy is None:
                rxy = self._robot_xy()
                if rxy is not None:
                    self._global_initial_sweep_start_xy = rxy
                else:
                    self._global_initial_sweep_start_xy = getattr(
                        self, "global_zigzag_start", (-1.25, 1.60)
                    )
            return self._global_initial_sweep_start_xy
        if goal_index - 1 < len(waypoints):
            return waypoints[goal_index - 1]
        return None

    def _drive_initial_sweep_segment(
        self,
        goal_x: float,
        goal_y: float,
        goal_index: int,
    ) -> None:
        """Drive one configured initial-sweep leg as a straight, non-holonomic segment."""
        segment_start = self._initial_sweep_segment_start(goal_index)
        if segment_start is None:
            self._drive(0.0, 0.0, 0.0)
            return
        self._publish_planning_obstacles([])
        self._plan = None
        self._plan_start_xy = None
        self._plan_dest = None
        segment_key = ("initial_sweep", int(goal_index))
        distance = self._distance_to(float(goal_x), float(goal_y))
        speed_limit_mps = None
        if (
            distance is not None
            and self.global_initial_sweep_slowdown_distance_m > 0.0
            and distance <= self.global_initial_sweep_slowdown_distance_m
        ):
            speed_limit_mps = self.global_initial_sweep_slow_speed_mps
        handled = self._drive_cardinal_lane_segment(
            segment_start,
            (float(goal_x), float(goal_y)),
            segment_key,
            stop_radius_m=self.global_patrol_reach_tol_m,
            speed_limit_mps=speed_limit_mps,
            allow_non_cardinal=True,
        )
        if handled:
            return
        yaw = math.atan2(
            float(goal_y) - float(segment_start[1]),
            float(goal_x) - float(segment_start[0]),
        )
        self._drive_toward_direct(float(goal_x), float(goal_y), yaw)

    def _initial_sweep_goal_arrived(
        self,
        goal_x: float,
        goal_y: float,
        goal_index: int,
    ) -> bool:
        """Accept a fixed sweep waypoint once the robot reaches or crosses its goal line."""
        robot_xy = self._robot_xy()
        segment_start = self._initial_sweep_segment_start(goal_index)
        if robot_xy is None or segment_start is None:
            return False
        reached, passed, lateral_error = _cardinal_waypoint_status(
            segment_start,
            robot_xy,
            (float(goal_x), float(goal_y)),
            self.global_patrol_reach_tol_m,
            self.lane_pass_lateral_tol_m,
        )
        if reached:
            if passed:
                self._decide(
                    f"GLOBAL ZIGZAG WAYPOINT PASSED lateral={lateral_error:.2f}m"
                )
            return True
        if passed:
            self._drive(0.0, 0.0, 0.0)
            self._decide(
                f"GLOBAL ZIGZAG WAYPOINT OVERRUN lateral={lateral_error:.2f}m "
                "-> ACCEPT TURN"
            )
            return True
        return False

    def _select_next_global_target(self) -> tuple[str, Object] | None:
        """Tour-ordered next target across the WHOLE map, both sets at once.

        Priority A: a Set2 cube already SigLIP-typed as today's fruit. Confirmed
        fruit always precedes Set1 shapes, regardless of path length. Priority B:
        an untyped fruit cube that still needs a Body read. Priority C: today's
        Set1 shape. A cube already typed as another fruit is never approached.
        """
        if not self.global_target_mode or self.world is None:
            return None
        rxy = (float(self.world.robot_x), float(self.world.robot_y))
        fruit_cands: dict[tuple[str, int], Object] = {}
        shape_cands: dict[tuple[str, int], Object] = {}
        planner = getattr(self, "_planner", None)
        safe_xmin, safe_xmax, safe_ymin, safe_ymax = (
            planner.interior
            if planner is not None
            else (-math.inf, math.inf, -math.inf, math.inf)
        )
        in_safe_field = lambda obj: (  # noqa: E731 - compact local candidate gate
            safe_xmin <= float(obj.x) <= safe_xmax
            and safe_ymin <= float(obj.y) <= safe_ymax
        )
        for o in self._iter_set1_targets():
            if in_safe_field(o) and self._global_target_in_active_corridor(o):
                shape_cands[("set1", int(o.id))] = o
        for o in self._iter_set2_target_objects():
            if not in_safe_field(o) or not self._global_target_in_active_corridor(o):
                continue
            if str(o.fruit_label) == self.set2_label:
                fruit_cands[("set2_object", int(o.id))] = o
        strong_fruit_cands = {
            key: obj
            for key, obj in fruit_cands.items()
            if getattr(self, "_global_initial_sweep_complete", False)
            and float(getattr(obj, "fruit_confidence", 0.0))
            >= getattr(self, "global_strong_fruit_positive_min_confidence", 0.50)
        }
        if strong_fruit_cands:
            order = order_targets_min_path(
                rxy,
                [
                    (key, float(obj.x), float(obj.y))
                    for key, obj in strong_fruit_cands.items()
                ],
            )
            kind, _ = order[0]
            return kind, strong_fruit_cands[order[0]]
        if fruit_cands:
            order = order_targets_min_path(
                rxy,
                [(k, float(o.x), float(o.y)) for k, o in fruit_cands.items()],
            )
            kind, _ = order[0]
            return kind, fruit_cands[order[0]]
        # A weak non-target fruit label is not enough to discard a cube beside the active route.
        # Treat it like an untyped cube and let the closer Body view settle the identity. Only a
        # sufficiently confident non-target label is rejected without an inspection visit.
        remaining_set2 = [
            o
            for o in self._iter_set2_target_objects()
            if in_safe_field(o) and self._global_target_in_active_corridor(o)
        ]
        untyped = [obj for obj in remaining_set2 if not str(obj.fruit_label)]
        weak_non_target = [
            obj
            for obj in remaining_set2
            if str(obj.fruit_label)
            and bool(str(getattr(obj, "fruit_label_source", "")))
            and 0.0 < float(getattr(obj, "fruit_confidence", 0.0))
            < getattr(
                self,
                "global_non_target_fruit_reject_min_confidence",
                0.40,
            )
        ]
        verify = min(
            untyped or weak_non_target,
            key=lambda o: math.hypot(float(o.x) - rxy[0], float(o.y) - rxy[1]),
            default=None,
        )
        if verify is not None:
            return "set2_object", verify
        if shape_cands:
            order = order_targets_min_path(
                rxy,
                [(k, float(o.x), float(o.y)) for k, o in shape_cands.items()],
            )
            kind, _ = order[0]
            return kind, shape_cands[order[0]]
        return None

    def _maybe_global_retarget(self) -> None:
        """Mid-APPROACH switch to a clearly better tour head (hysteresis-gated).

        Background (wide/body SigLIP) typing keeps adding confirmed fruits while
        driving; a new head must shorten the immediate leg by at least
        global_retarget_min_gain to win, so the base never oscillates between
        two near-equal targets.
        """
        now = self._now_s()
        if now - self._global_retarget_last_s < self.global_retarget_check_sec:
            return
        self._global_retarget_last_s = now
        if self.current_target is None or self._appr_tgt_xy is None:
            return
        # Once Wide/world has supplied an explicit mission target, finish that physical visit.
        # A later tracker dropout or a newly discovered nearer object must not erase the pending
        # Body verification. Untyped exploratory cubes may still yield to a confirmed target.
        if self._committed_global_target_can_try_align():
            return
        sel = self._select_next_global_target()
        if sel is None:
            return
        kind, tgt = sel
        if int(tgt.id) == int(self.current_target.id):
            return
        if not str(tgt.fruit_label) and int(tgt.set_type) == 2:
            return                                       # never abandon a target for an untyped cube
        rxy = self._robot_xy()
        if rxy is None:
            return
        d_cur = math.hypot(self._appr_tgt_xy[0] - rxy[0], self._appr_tgt_xy[1] - rxy[1])
        d_new = math.hypot(float(tgt.x) - rxy[0], float(tgt.y) - rxy[1])
        if not global_retarget_is_worthwhile(
            d_cur,
            d_new,
            self.global_retarget_min_gain,
            getattr(self, "global_retarget_lock_distance_m", 0.80),
        ):
            return
        if self._latch_mixed_target(kind, tgt):
            if int(tgt.set_type) == 2:
                self._begin_set2_identity_visit()
            self._appr_tgt_xy = (float(tgt.x), float(tgt.y))
            self._appr_last_seen_s = now
            self._plan = None
            self._reset_approach_motion()
            self._latch_approach_goal(self._appr_tgt_xy)
            self.get_logger().info(
                f"global retarget: #{tgt.id} '{tgt.fruit_label or tgt.class_label}' "
                f"({d_new:.2f}m vs {d_cur:.2f}m)")
            self._decide(f"GLOBAL RETARGET #{tgt.id} {tgt.fruit_label or tgt.class_label}")

    def _update_global_seen_nodes(self) -> None:
        """Mark coverage once per fresh Wide frame and only while field mapping is valid."""
        now = self._now_s()
        if (
            not self.global_target_mode
            or self.world is None
            or not self._world_mapping_enabled
            or self.state == "OPENING"
            or self._wide_relative_seq == self._global_seen_wide_seq
            or self._wide_relative_last_frame_s is None
            or now - self._wide_relative_last_frame_s > self.global_sensor_max_age_sec
            or now - self._world_last_rx_s > self.global_sensor_max_age_sec
        ):
            return
        self._global_seen_wide_seq = self._wide_relative_seq
        self._global_seen_nodes |= nodes_in_ellipse_fov(
            (float(self.world.robot_x), float(self.world.robot_y), float(self.world.robot_theta)),
            self._global_patrol_nodes,
            self.global_wide_fov_forward_m,
            self.global_wide_fov_lateral_m,
            self.global_wide_fov_center_x_m,
            self.global_fov_min_range_m,
        )

    def _global_navigation_ready(self) -> tuple[bool, str]:
        """Require settled mapping plus fresh pose, world map, and Wide frames before moving."""
        now = self._now_s()
        if not self._world_mapping_enabled:
            return False, "mapping disabled"
        if now - self._global_mapping_enabled_since_s < self.global_initial_map_settle_sec:
            return False, "initial map settling"
        if self.world is None or now - self._world_last_rx_s > self.global_sensor_max_age_sec:
            return False, "world model stale"
        if now - self._localization_pose_last_s > self.global_sensor_max_age_sec:
            return False, "robot pose stale"
        if (
            self._wide_relative_last_frame_s is None
            or now - self._wide_relative_last_frame_s > self.global_sensor_max_age_sec
        ):
            return False, "wide camera stale"
        return True, "ready"

    def _global_patrol_step(self) -> None:
        """Sweep unseen object slots from ordered safe lane centres.

        In zigzag mode the LanePlanner lawn-mower order persists across target interruptions,
        so completed field regions are not revisited while any new region remains. At arrival
        the robot faces along the sweep and waits for a genuinely new Wide frame.
        """
        if self.world is None:
            self._drive(0.0, 0.0, 0.0)
            return
        initial_sweep_active = (
            self.global_zigzag_patrol_enabled
            and bool(self._global_zigzag_waypoints)
            and not self._global_initial_sweep_complete
        )
        if (
            len(self._global_seen_nodes) >= len(self._global_patrol_nodes)
            and not initial_sweep_active
        ):
            self._global_seen_nodes.clear()
            self._global_patrol_goal = None
            self._global_patrol_goal_zigzag_index = None
            self._decide("GLOBAL SWEEP RESTART")
            return
        if (
            self._global_patrol_goal is not None
            and not self.global_zigzag_fixed_route
            and not (self._global_patrol_goal[3] - self._global_seen_nodes)
        ):
            if (
                self._global_patrol_goal_zigzag_index is not None
                and self._global_zigzag_waypoints
            ):
                self._global_zigzag_idx = (
                    self._global_patrol_goal_zigzag_index + 1
                ) % len(self._global_zigzag_waypoints)
            self._global_patrol_goal = None
            self._global_patrol_goal_zigzag_index = None
            self._plan = None
        if self._global_patrol_goal is None:
            robot_pose = (
                float(self.world.robot_x),
                float(self.world.robot_y),
                float(self.world.robot_theta),
            )
            viewpoints = self._planner.coverage_path(
                robot_pose[:2], self._obstacles_snapshot()
            )
            if initial_sweep_active:
                zigzag = next_zigzag_observation_pose(
                    self._global_patrol_nodes,
                    self._global_seen_nodes,
                    self._global_zigzag_waypoints,
                    self._global_zigzag_idx,
                    self.global_wide_fov_forward_m,
                    self.global_wide_fov_lateral_m,
                    self.global_wide_fov_center_x_m,
                    self.global_fov_min_range_m,
                    skip_fully_seen=not self.global_zigzag_fixed_route,
                )
                if zigzag is not None:
                    index, gx, gy, heading, covered = zigzag
                    self._global_zigzag_idx = index
                    self._global_patrol_goal_zigzag_index = index
                    self._global_patrol_goal = (gx, gy, heading, covered)
                else:
                    self._global_patrol_goal = None
                    self._global_patrol_goal_zigzag_index = None
            else:
                self._global_patrol_goal = best_observation_pose(
                    robot_pose,
                    self._global_patrol_nodes,
                    self._global_seen_nodes,
                    viewpoints,
                    self.global_wide_fov_forward_m,
                    self.global_wide_fov_lateral_m,
                    self.global_wide_fov_center_x_m,
                    self.global_fov_min_range_m,
                )
                self._global_patrol_goal_zigzag_index = None
            self._global_patrol_observe_seq = -1
            self._global_patrol_observe_start_s = 0.0
            if self._global_patrol_goal is None:
                self._drive(0.0, 0.0, 0.0)
                self.get_logger().warn(
                    "no safe lane-centre observation pose available",
                    throttle_duration_sec=2.0,
                )
                return
            gx, gy, heading, covered = self._global_patrol_goal
            if self._global_patrol_goal_zigzag_index is not None:
                self._decide(
                    f"GLOBAL ZIGZAG Z2-FIRST {self._global_patrol_goal_zigzag_index + 1}/"
                    f"{len(self._global_zigzag_waypoints)} -> ({gx:.2f},{gy:.2f}) "
                    f"new_slots={len(covered)}"
                )
            else:
                self._decide(
                    f"GLOBAL VIEW ({gx:.2f},{gy:.2f}) "
                    f"heading={math.degrees(heading):+.0f}deg slots={len(covered)}"
                )

        gx, gy, heading, _covered = self._global_patrol_goal
        distance = self._distance_to(gx, gy)
        initial_sweep_arrived = False
        if (
            initial_sweep_active
            and not self.global_initial_sweep_holonomic_enabled
            and self._global_patrol_goal_zigzag_index is not None
        ):
            initial_sweep_arrived = self._initial_sweep_goal_arrived(
                gx,
                gy,
                self._global_patrol_goal_zigzag_index,
            )
        if (
            not initial_sweep_arrived
            and (distance is None or distance > self.global_patrol_reach_tol_m)
        ):
            if (
                initial_sweep_active
                and not self.global_initial_sweep_holonomic_enabled
                and self._global_patrol_goal_zigzag_index is not None
            ):
                self._drive_initial_sweep_segment(
                    gx,
                    gy,
                    self._global_patrol_goal_zigzag_index,
                )
            else:
                self._drive_toward(
                    gx,
                    gy,
                    heading,
                    exclude_id=0,
                    route_mode=self._global_exploration_route_mode(),
                    holonomic_translation=(
                        initial_sweep_active
                        and self.global_initial_sweep_holonomic_enabled
                    ),
                )
            return

        self._plan = None
        aligned = bool(
            initial_sweep_active
            and self.global_initial_sweep_holonomic_enabled
        ) or self._turn_in_place_pulsed(
            heading,
            self.global_patrol_heading_tol_rad,
            ("global_view", round(gx, 3), round(gy, 3), round(heading, 3)),
            self.direct_nav_omega_max,
        )
        if not aligned:
            self._global_patrol_observe_seq = -1
            return
        now = self._now_s()
        self._drive(0.0, 0.0, 0.0)
        if self._global_patrol_observe_seq < 0:
            self._global_patrol_observe_seq = self._wide_relative_seq
            self._global_patrol_observe_start_s = now
            return
        if (
            now - self._global_patrol_observe_start_s >= self.global_observe_min_sec
            and self._wide_relative_seq > self._global_patrol_observe_seq
        ):
            if (
                self._global_patrol_goal_zigzag_index is not None
                and self._global_zigzag_waypoints
            ):
                self._advance_global_initial_sweep()
            self._global_patrol_goal = None
            self._global_patrol_goal_zigzag_index = None
            self._global_patrol_observe_seq = -1

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
        if (
            not self.align_body_lost_same_slot_retry_enabled
            or self.align_body_lost_same_slot_retries <= 0
        ):
            return False

        if slot is not None:
            used = self._slot_body_lost_retry_counts.get(slot.slot_id, 0)
            if used >= self.align_body_lost_same_slot_retries:
                return False
            self._slot_body_lost_retry_counts[slot.slot_id] = used + 1
            self.current_target = self._object_for_slot(slot)
            target_xy = (float(slot.x), float(slot.y))
            target_name = f"slot F{slot.slot_id}"
        elif self.global_target_mode and self.current_target is not None:
            target_xy = (
                self._appr_tgt_xy
                if self._appr_tgt_xy is not None
                else (float(self.current_target.x), float(self.current_target.y))
            )
            key = self._body_lost_recovery_key()
            if key is None:
                return False
            used = self._global_body_lost_retry_counts.get(key, 0)
            if used >= self.align_body_lost_same_slot_retries:
                return False
            self._global_body_lost_retry_counts[key] = used + 1
            target_name = (
                f"target #{int(self.current_target.id)} "
                f"at ({target_xy[0]:.2f},{target_xy[1]:.2f})"
            )
        else:
            return False

        self._plan = None
        self._decide(f"SAME TARGET REAPPROACH after Body lost ({target_name})")
        self.get_logger().info(
            f"ALIGN: Body lost -> immediate same-target reapproach {target_name} "
            f"{used + 1}/{self.align_body_lost_same_slot_retries}"
        )
        self._enter("APPROACH")
        self._appr_tgt_xy = target_xy
        self._latch_approach_goal(target_xy)
        self._appr_last_seen_s = self._now_s()
        self._require_precise_heading_before_align = True
        self._precise_heading_retry_start_s = self._now_s()
        return True

    def _body_lost_recovery_key(self) -> tuple[int, int, int] | None:
        """Stable spatial key so track-ID churn cannot repeat an expensive recovery loop."""
        if self.current_target is None:
            return None
        x, y = (
            self._appr_tgt_xy
            if self._appr_tgt_xy is not None
            else (float(self.current_target.x), float(self.current_target.y))
        )
        return (
            int(self.current_target.set_type),
            int(round(float(x) / 0.10)),
            int(round(float(y) / 0.10)),
        )

    def _start_align_body_lost_lateral_sweep(self, now: float) -> bool:
        """Strafe toward the Wide/frozen target once before route-level retry."""
        if (
            not self.align_body_lost_lateral_sweep_enabled
            or not self.global_target_mode
            or self.world is None
        ):
            return False
        key = self._body_lost_recovery_key()
        if key is None or self._body_lost_lateral_sweep_counts.get(key, 0) >= 1:
            return False
        point = self._wide_align_target_base() or self._wide_align_expected_base()
        if point is None:
            return False

        _, by = float(point[0]), float(point[1])
        side_sign = math.copysign(1.0, by) if abs(by) >= 0.01 else (
            1.0 if (abs(key[1]) + abs(key[2])) % 2 == 0 else -1.0
        )

        self._body_lost_lateral_sweep_counts[key] = 1
        self._align_body_lost_sweep_strafe_sign = side_sign
        self._align_phase = "body_lost_lateral_sweep"
        self._align_phase_start = float(now)
        self._reset_align_body_loss_confirmation()
        self._reset_align_body_presence()
        self.get_logger().info(
            f"ALIGN: backoff reacquire failed -> one translation-only lateral sweep "
            f"wide_y={by:+.2f}m, "
            f"strafe={'left' if side_sign > 0.0 else 'right'}"
        )
        self._decide(
            f"ALIGN LATERAL RECOVERY "
            f"strafe={'L' if side_sign > 0.0 else 'R'} ~10cm / NO TURN"
        )
        return True

    def _ready_for_precise_align_retry(self, bearing: float) -> bool:
        if not self._require_precise_heading_before_align or self.world is None:
            return True
        now = self._now_s()
        if self._precise_heading_retry_start_s is None:
            self._precise_heading_retry_start_s = now
        measured_heading = (
            self._approach_heading_filtered
            if self._approach_heading_filtered is not None
            else float(self.world.robot_theta)
        )
        err = self._wrap_pi(float(bearing) - measured_heading)
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

    def _body_base_matches_current_anchor_slot(self, bx: float, by: float) -> bool:
        slot = self._current_anchor_slot()
        if slot is None:
            return False
        if self._anchor_body_bound_point is not None:
            # Once three fresh frames have hard-bound this physical object to the current
            # Wide slot, follow it by short-range Body continuity.  This avoids accumulating
            # command-integration error during calibrated ALIGN pulses while the current
            # landmark is intentionally excluded from the relative grid update.  The radius
            # remains below half the 50 cm slot spacing, so it cannot jump to the next row.
            return math.hypot(
                float(bx) - self._anchor_body_bound_point[0],
                float(by) - self._anchor_body_bound_point[1],
            ) <= self.body_slot_continuity_radius_m
        slot_points: dict[int, tuple[float, float]] = {}
        for candidate in self.anchor_inventory.anchor_slots(slot.anchor_id):
            point = (
                self._relative_slot_point(candidate)
                if self._relative_mode()
                else self._to_base(candidate.x, candidate.y)
            )
            if point is not None:
                slot_points[candidate.slot_id] = (float(point[0]), float(point[1]))
        return body_observation_matches_slot(
            observed_xy=(float(bx), float(by)),
            current_slot_id=slot.slot_id,
            slot_points=slot_points,
            forward_tol_m=self.body_slot_forward_tol_m,
            lateral_tol_m=self.body_slot_lateral_tol_m,
            bearing_tol_rad=self.body_slot_bearing_tol_rad,
            unique_margin_m=self.body_slot_unique_margin_m,
        )

    def _anchor_siglip_matches_current_slot(self) -> bool:
        """Accept SigLIP only when its source frame has one unambiguous current-slot crop."""
        eligible = [
            detection
            for detection in self._siglip_body_dets
            if detection[2] in {"fruit_photo_cube", "cube"}
        ]
        if len(eligible) != 1:
            return False
        u, v, _, _ = eligible[0]
        base = self._body_pixel_base(u, v)
        return bool(
            base is not None
            and self._body_base_matches_current_anchor_slot(base[0], base[1])
        )

    def _siglip_source_at_grab(self) -> bool:
        """Whether the exact Body crop named by SigLIP is centred at the gripper."""
        if self.siglip is None:
            return False
        source_pixel = parse_body_siglip_pixel(self.siglip.source)
        eligible = [
            detection
            for detection in self._siglip_body_dets
            if detection[2] in {"fruit_photo_cube", "cube"}
        ]
        if source_pixel is None:
            # Backward-compatible but conservative: an untagged legacy result is usable
            # only when its source frame contains exactly one possible fruit crop.
            if len(eligible) != 1:
                return False
            source_pixel = (float(eligible[0][0]), float(eligible[0][1]))
        elif not any(
            abs(float(detection[0]) - source_pixel[0]) <= 1.5
            and abs(float(detection[1]) - source_pixel[1]) <= 1.5
            for detection in eligible
        ):
            return False
        base = self._body_pixel_base(source_pixel[0], source_pixel[1])
        if base is None:
            return False
        return bool(
            abs(float(base[0]) - self.grab_x) < self.align_fwd_tol
            and abs(float(base[1]) - self.grab_y) < self.align_tol
        )

    def _target_matches_set2_latch(
        self,
        target: Object | None,
        latch: Set2IdentityLatch,
    ) -> bool:
        """Keep a valid identity through harmless world-track ID churn at the same spot."""
        return set2_latch_target_matches(
            target,
            latch,
            self.global_body_target_match_radius_m,
            locally_blacklisted=bool(
                target is not None and self._is_locally_blacklisted(int(target.id))
            ),
        )

    def _begin_set2_identity_visit(self) -> None:
        """Invalidate delayed classifications whenever the FSM selects a new Set2 visit."""
        target = self.current_target
        self._set2_identity_visit_serial += 1
        self._set2_identity_visit_started_s = self._now_s()
        self._set2_identity_latch = None
        if target is None or int(target.set_type) != 2:
            self._set2_identity_visit_target_id = 0
            self._set2_identity_visit_target_xy = None
            return
        self._set2_identity_visit_target_id = int(target.id)
        self._set2_identity_visit_target_xy = (
            float(target.x),
            float(target.y),
        )

    def _latched_set2_identity(self) -> Set2IdentityLatch | None:
        """Return the identity for this exact visit, expiring stale/unrelated reads."""
        latch = self._set2_identity_latch
        if latch is None:
            return None
        if (
            latch.visit_serial != self._set2_identity_visit_serial
            or latch.capture_s + 1e-6 < self._set2_identity_visit_started_s
            or self._now_s() - latch.received_s
            > self.set2_identity_latch_max_age_sec
        ):
            self._set2_identity_latch = None
            return None
        if self._target_matches_set2_latch(self.current_target, latch):
            return latch
        if self._appr_tgt_xy is not None and math.hypot(
            float(self._appr_tgt_xy[0]) - latch.target_x,
            float(self._appr_tgt_xy[1]) - latch.target_y,
        ) <= self.global_body_target_match_radius_m:
            return latch
        return None

    def _target_for_set2_latch(self, latch: Set2IdentityLatch) -> Object | None:
        """Resolve the same physical target without switching to a neighbouring front cube."""
        current = self.current_target
        if self._target_matches_set2_latch(current, latch):
            fresh = self._lookup_object(int(current.id))
            if fresh is None:
                return current
            if self._target_matches_set2_latch(fresh, latch):
                return fresh
        if self.world is None:
            return None
        candidates = [
            obj
            for obj in self.world.objects
            if (
                not obj.blacklisted
                and not self._is_locally_blacklisted(int(obj.id))
                and int(obj.set_type) == 2
                and math.hypot(
                    float(obj.x) - latch.target_x,
                    float(obj.y) - latch.target_y,
                ) <= self.global_body_target_match_radius_m
            )
        ]
        return min(
            candidates,
            key=lambda obj: math.hypot(
                float(obj.x) - latch.target_x,
                float(obj.y) - latch.target_y,
            ),
            default=None,
        )

    def _maybe_latch_set2_identity(
        self,
        classification: Classification,
        capture_s: float | None,
        received_s: float | None,
    ) -> None:
        """Bind a delayed Body classification to the selected target at image time."""
        target = self.current_target
        if (
            target is None
            or int(target.set_type) != 2
            or bool(target.blacklisted)
            or self._is_locally_blacklisted(int(target.id))
            or self.state not in {"APPROACH", "ALIGN", "CLASSIFY"}
            or capture_s is None
            or received_s is None
            or self._set2_identity_visit_target_xy is None
        ):
            return
        if not set2_classification_timing_is_valid(
            capture_s,
            received_s,
            self._set2_identity_visit_started_s,
            self.set2_identity_max_inference_delay_sec,
        ):
            return
        source_pixel = parse_body_siglip_pixel(classification.source)
        if source_pixel is None:
            return
        eligible = [
            detection
            for detection in self._siglip_body_dets
            if detection[2] in {"fruit_photo_cube", "cube"}
        ]
        if not any(
            abs(float(detection[0]) - source_pixel[0]) <= 1.5
            and abs(float(detection[1]) - source_pixel[1]) <= 1.5
            for detection in eligible
        ):
            return
        source_base = self._body_pixel_base(*source_pixel)
        capture_pose = closest_pose_sample(
            self._localization_pose_history,
            capture_s,
            self.set2_identity_pose_max_skew_sec,
        )
        if source_base is None or capture_pose is None:
            return
        target_xy = self._set2_identity_visit_target_xy
        if math.hypot(
            float(target.x) - target_xy[0],
            float(target.y) - target_xy[1],
        ) > self.global_body_target_match_radius_m:
            return
        expected_base = field_point_to_base_at_pose(target_xy, capture_pose)
        association_error = math.hypot(
            float(source_base[0]) - expected_base[0],
            float(source_base[1]) - expected_base[1],
        )
        if not body_source_matches_target_at_capture(
            source_base,
            target_xy,
            capture_pose,
            self.global_body_target_match_radius_m,
        ):
            return
        verdict, label, confidence = set2_classification_verdict(
            classification,
            confidence_threshold=self.conf_threshold,
            target_label=self.set2_label,
        )
        if verdict == "pending":
            return
        previous = self._set2_identity_latch
        if (
            previous is not None
            and previous.visit_serial == self._set2_identity_visit_serial
            and capture_s + 1e-6 < previous.capture_s
        ):
            return
        self._set2_identity_latch = Set2IdentityLatch(
            target_id=self._set2_identity_visit_target_id,
            target_x=target_xy[0],
            target_y=target_xy[1],
            verdict=verdict,
            label=label,
            confidence=confidence,
            capture_s=float(capture_s),
            received_s=float(received_s),
            association_error_m=association_error,
            visit_serial=self._set2_identity_visit_serial,
        )
        if (
            previous is None
            or previous.target_id != self._set2_identity_visit_target_id
            or previous.verdict != verdict
            or previous.label != label
        ):
            self.get_logger().info(
                f"SET2 IDENTITY LATCH target=#{self._set2_identity_visit_target_id} "
                f"{verdict} '{label}' "
                f"conf={confidence:.3f} capture_match={association_error * 100:.1f}cm"
            )
            self._decide(
                f"SET2 BODY {verdict.upper()} {label} "
                f"#{self._set2_identity_visit_target_id} "
                f"match={association_error * 100:.1f}cm"
            )

    def _reject_latched_set2_non_target(self) -> bool:
        """Skip a selected non-target immediately, before spending time in final ALIGN."""
        latch = self._latched_set2_identity()
        if (
            not self.set2_require_fruit_label
            or latch is None
            or latch.verdict != "non_target"
        ):
            return False
        current_id = int(self.current_target.id) if self.current_target is not None else 0
        self._drive(0.0, 0.0, 0.0)
        self.get_logger().info(
            f"SET2 early reject #{current_id or latch.target_id}: "
            f"Body '{latch.label}' != {self.set2_label}"
        )
        self._decide(
            f"SET2 EARLY NON-TARGET {latch.label} #{current_id or latch.target_id}"
        )
        self._blacklist(latch.target_id)
        if current_id and current_id != latch.target_id:
            self._blacklist(current_id)
        self._set2_identity_latch = None
        self.current_target = None
        self.set_type = 0
        self._opportunistic_set2_active = False
        self._enter("SELECT_TARGET")
        return True

    def _fresh_world_set1_non_target(self) -> Object | None:
        """Return the latched Set1 track once fresh world evidence says it is another shape."""
        target = self.current_target
        if (
            target is None
            or self.phase != 1
            or int(target.set_type) != 1
            or self.world is None
        ):
            return None
        fresh = self._lookup_object(int(target.id))
        if fresh is None or bool(fresh.blacklisted) or int(fresh.set_type) != 1:
            return None
        self.current_target = fresh
        label = str(fresh.class_label)
        if (
            label
            and label != str(self.set1_label)
            and float(fresh.confidence) >= float(self.pick_track_conf)
        ):
            return fresh
        return None

    def _fresh_body_set1_non_target_at_latched_position(
        self,
    ) -> tuple[str, float, float, float] | None:
        """Return a fresh Body shape mismatch spatially tied to the frozen Set1 visit."""
        target = self.current_target
        if (
            target is None
            or self.phase != 1
            or int(target.set_type) != 1
            or self._body_H is None
            or not self._body_dets
            or self._body_dets_stamp_s <= 0.0
            or self._body_dets_stamp_s < self.state_enter_s
            or self._now_s() - self._body_dets_stamp_s
            > max(self.approach_body_visible_max_age_sec, self.classify_body_max_age_sec)
        ):
            return None
        candidates: list[tuple[str, float, float, float]] = []
        for u, v, label, confidence in self._body_dets:
            label = str(label)
            if _LABEL_ST.get(label, 0) != 1:
                continue
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            bx, by = base
            candidates.append((label, bx, by, float(confidence)))
        if not candidates:
            return None
        expected_xy = (
            getattr(self, "_appr_tgt_xy", None)
            if getattr(self, "_appr_tgt_xy", None) is not None
            else (float(target.x), float(target.y))
        )
        expected = self._to_base(float(expected_xy[0]), float(expected_xy[1]))
        selected = select_body_candidate(
            candidates,
            grab_xy=(self.grab_x, self.grab_y),
            expected_xy=expected,
            expected_match_radius_m=self.global_body_target_match_radius_m,
            grab_search_radius_m=0.25,
        )
        if selected is None:
            return None
        label, bx, by, confidence = selected
        if (
            str(label) != str(self.set1_label)
            and float(confidence) >= float(self.pick_track_conf)
        ):
            return str(label), float(bx), float(by), float(confidence)
        return None

    def _reject_latched_set1_non_target(self) -> bool:
        """Set1 keeps the v5.2.0-style final stopped Body vote.

        Wide/world labels may route us to a Set1 candidate, but a close-range transient Body label
        such as ``cube`` must not blacklist the target before ALIGN/CLASSIFY gets a settled
        multi-frame vote.
        """
        return False

    def _fresh_set2_body_verdict(self) -> tuple[str, str, float]:
        """Final Set2 decision from a fresh, spatially matched Body SigLIP crop only."""
        return fresh_set2_siglip_verdict(
            self.siglip,
            self.siglip_stamp_s,
            received_s=self.siglip_rx_s,
            state_enter_s=self.state_enter_s,
            now_s=self._now_s(),
            max_age_sec=self.classify_body_max_age_sec,
            confidence_threshold=self.conf_threshold,
            target_label=self.set2_label,
            spatially_aligned=self._siglip_source_at_grab(),
        )

    def _fresh_aligned_set2_body_target(self) -> tuple[str, float] | None:
        """Return a fresh gripper-bound target fruit even if the latest Body YOLO frame is empty."""
        target = self.current_target
        if not (
            self.phase == 2
            or self._opportunistic_set2_active
            or (target is not None and int(target.set_type) == 2)
        ):
            return None
        verdict, label, confidence = self._fresh_set2_body_verdict()
        if verdict != "target":
            return None
        return label, confidence

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

        source_pixel = parse_body_siglip_pixel(self.siglip.source)
        eligible = [
            detection
            for detection in self._siglip_body_dets
            if detection[2] in {"fruit_photo_cube", "cube"}
        ]
        if source_pixel is None:
            if len(eligible) != 1:
                return False
            source_pixel = (float(eligible[0][0]), float(eligible[0][1]))
        elif not any(
            abs(float(detection[0]) - source_pixel[0]) <= 1.5
            and abs(float(detection[1]) - source_pixel[1]) <= 1.5
            for detection in eligible
        ):
            return False
        base = self._body_pixel_base(source_pixel[0], source_pixel[1])
        field_xy = self._body_base_to_field(*base) if base is not None else None
        if field_xy is None:
            return False
        slot_distance = slot.distance_to(field_xy[0], field_xy[1])
        if slot_distance > self.set2_slot_attach_radius_m:
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
        if self.anchor_mission_enabled:
            payload = self.anchor_inventory.debug_payload(
                current_anchor_id=self._active_anchor_id(),
                current_slot_id=self._anchor_current_slot_id or 0,
            )
            if self.relative_anchor_nav_enabled:
                estimate = self._relative_estimate()
                payload["relative_nav"] = {
                    "locked": self._relative_anchor_tracker.locked,
                    "navigable": bool(estimate.navigable) if estimate is not None else False,
                    "reason": estimate.reason if estimate is not None else "not_locked",
                    "matched": estimate.matched_count if estimate is not None else 0,
                    "age_s": round(estimate.age_s, 3) if estimate is not None else None,
                    "anchor_base": (
                        [round(estimate.center_x, 3), round(estimate.center_y, 3)]
                        if estimate is not None else None
                    ),
                    "slots_base": (
                        [[round(x, 3), round(y, 3)] for x, y in estimate.slots]
                        if estimate is not None else []
                    ),
                }
        else:
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
                latched_set_type = int(self.current_target.set_type)
                fresh = self._lookup_object(int(self.current_target.id))
                if fresh is not None:
                    # Keep the latched object synchronized even when it has just been
                    # blacklisted or its provisional Wide fruit label changes.
                    self.current_target = fresh
                if (fresh is not None
                        and not fresh.blacklisted
                        and int(fresh.set_type) == latched_set_type):
                    if int(fresh.set_type) == 1 and str(fresh.class_label) == self.set1_label:
                        return fresh
                    if int(fresh.set_type) == 2:
                        fl = str(fresh.fruit_label)
                        # In global mode Wide/transit fruit labels are routing hints only.
                        # A wrong hint is intentionally revisited for stopped Body
                        # verification, so it must not invalidate that same object halfway
                        # through APPROACH.  Doing so caused APPROACH->SCAN every 2.5 s and
                        # repeatedly restarted the lane/object heading controllers.
                        if (self.global_target_mode
                                or not self.set2_require_fruit_label
                                or not fl
                                or fl == self.set2_label):
                            return fresh
                    if self.global_target_mode:
                        # In the 3-point/global tour a selected visit owns one frozen physical
                        # position. If that exact track becomes a non-target, do not borrow a
                        # neighbouring target and spend time aligning to the wrong object; the
                        # caller will early-reject/defer this visit and resume the lane.
                        return None
            if ref_xy is None and self.current_target is not None:
                ref_xy = (float(self.current_target.x), float(self.current_target.y))
            if self.global_target_mode and self.current_target is not None:
                return None
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
        detections = [
            (float(d.x_center), float(d.y_center), str(d.label), float(d.confidence))
            for d in msg.detections
        ]
        self._body_dets = detections
        self._body_dets_stamp_s = self._now_s()
        self._body_dets_seq += 1
        capture_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if capture_key != (0, 0):
            self._body_dets_history.append((capture_key, list(detections)))
            del self._body_dets_history[:-30]
        self._feed_local_anchor_body_detections()

    def _body_pixel_base(self, u: float, v: float) -> tuple[float, float] | None:
        if self._body_H is None:
            return None
        us, vs = float(u) * self._body_px_sx, float(v) * self._body_px_sy
        p = cv2.perspectiveTransform(np.array([[[us, vs]]], np.float64), self._body_H)[0][0]
        k = self._boh / self._bH
        return float(p[0]) - k * (float(p[0]) - self._bnx), float(p[1]) - k * float(p[1])

    def _body_target_base(self, want_label: str) -> tuple[str, float, float, float] | None:
        """Return the gated Body detection nearest the grab point in ``base_link``.

        Legacy missions keep their label filter.  Anchor mode accepts any mission-object
        label only after the immutable current-slot geometry gate, allowing Body to correct
        a low-confidence Wide identity without ever changing slot membership.
        """
        if self._body_H is None or not self._body_dets:
            return None
        allowed_labels = self._body_align_labels()
        candidates: list[tuple[str, float, float, float]] = []
        for u, v, label, confidence in self._body_dets:
            if label not in allowed_labels:
                continue
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            bx, by = base
            if self._set2_slot_mode() and self._current_slot() is not None:
                if not self._body_base_matches_current_slot(bx, by):
                    continue
            if self.anchor_mission_enabled and self._current_anchor_slot() is not None:
                if not self._body_base_matches_current_anchor_slot(bx, by):
                    continue
            candidates.append((str(label), bx, by, float(confidence)))
        expected = None
        if self.global_target_mode and self.current_target is not None:
            # Track coordinates can jump just before an ID disappears. During a committed
            # visit, associate Body detections with the immutable Wide position that actually
            # caused the approach rather than the latest mutable tracker coordinate.
            expected_xy = (
                getattr(self, "_appr_tgt_xy", None)
                if getattr(self, "_appr_tgt_xy", None) is not None
                else (float(self.current_target.x), float(self.current_target.y))
            )
            expected = self._to_base(float(expected_xy[0]), float(expected_xy[1]))
        return select_body_candidate(
            candidates,
            grab_xy=(self.grab_x, self.grab_y),
            expected_xy=expected,
            expected_match_radius_m=self.global_body_target_match_radius_m,
        )

    def _fresh_approach_body_target(self) -> tuple[str, float, float, float] | None:
        """Return the selected target only when the latest Body frame is current.

        This is deliberately stricter than merely having cached detections: a stale object
        must never suppress the final approach heading correction.
        """
        if not self.approach_body_visible_skip_heading_enabled:
            return None
        if (
            self._body_dets_stamp_s <= 0.0
            or self._now_s() - self._body_dets_stamp_s
            > self.approach_body_visible_max_age_sec
        ):
            return None
        return self._body_target_base(self._body_label())

    def _fresh_body_set1_pick_candidate(self) -> tuple[str, float, float, float] | None:
        """Return a fresh, spatially aligned Body confirmation for the latched Set1 target."""
        if (
            self._opportunistic_set2_active
            or self.phase != 1
            or self.current_target is None
            or int(self.current_target.set_type) != 1
            or str(self.current_target.class_label) != self.set1_label
            or self._body_dets_stamp_s < self.state_enter_s
            or self._now_s() - self._body_dets_stamp_s > self.classify_body_max_age_sec
        ):
            return None
        body_target = self._body_target_base(self.set1_label)
        if body_target is None:
            return None
        label, bx, by, confidence = body_target
        ex, ey = bx - self.grab_x, by - self.grab_y
        if (
            label != self.set1_label
            or confidence < max(self.pick_track_conf, self.conf_threshold)
            or abs(ex) >= self.align_fwd_tol
            or abs(ey) >= self.align_tol
        ):
            return None
        return label, confidence, ex, ey

    def _fresh_body_set1_pick_presence(self) -> tuple[str, float, float] | None:
        """Return fresh label-agnostic Body presence at the grab point for Wide verification."""
        if (
            self._opportunistic_set2_active
            or self.phase != 1
            or self.current_target is None
            or int(self.current_target.set_type) != 1
            or str(self.current_target.class_label) != self.set1_label
            or self._body_dets_stamp_s < self.state_enter_s
            or self._now_s() - self._body_dets_stamp_s > self.classify_body_max_age_sec
        ):
            return None
        body_object = self._body_nearest_any()
        if body_object is None:
            return None
        label, bx, by = body_object
        ex, ey = float(bx) - self.grab_x, float(by) - self.grab_y
        if abs(ex) >= self.align_fwd_tol or abs(ey) >= self.align_tol:
            return None
        return str(label), ex, ey

    def _fresh_committed_set2_cube_at_grab(
        self,
    ) -> tuple[str, float, float, float] | None:
        """Accept a blank cube face only at a previously target-fruit-marked position.

        Fruit-photo cubes do not carry a photo on every face.  This fallback therefore uses a
        new post-CLASSIFY Body YOLO frame as presence evidence, while retaining the frozen
        target-fruit-marked world position as identity evidence. It never accepts another shape or a
        cube outside the calibrated gripper tolerance.
        """
        if (
            not bool(getattr(self, "set2_committed_cube_pick_enabled", False))
            or not self._committed_set2_target_can_try_align()
            or self._body_dets_stamp_s < self.state_enter_s
            or self._now_s() - self._body_dets_stamp_s > self.classify_body_max_age_sec
        ):
            return None
        body_target = self._body_target_base("fruit_photo_cube")
        if body_target is None:
            return None
        label, bx, by, confidence = body_target
        ex, ey = float(bx) - self.grab_x, float(by) - self.grab_y
        if (
            str(label) not in {"fruit_photo_cube", "cube"}
            or float(confidence) < self.pick_track_conf
            or abs(ex) >= self.align_fwd_tol
            or abs(ey) >= self.align_tol
        ):
            return None
        return str(label), float(confidence), ex, ey

    def _body_nearest_any(self) -> tuple[str, float, float] | None:
        """Nearest body detection of ANY label to the grab point -> (label, bx, by). Lets ALIGN spot
        a DISTRACTOR (e.g. a cube) sitting where the target was expected and skip that spot fast."""
        if self._body_H is None or not self._body_dets:
            return None
        candidates: list[tuple[str, float, float, float]] = []
        for u, v, label, _ in self._body_dets:
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            bx, by = base
            if self._set2_slot_mode() and self._current_slot() is not None:
                if not self._body_base_matches_current_slot(bx, by):
                    continue
            if self.anchor_mission_enabled and self._current_anchor_slot() is not None:
                if not self._body_base_matches_current_anchor_slot(bx, by):
                    continue
            candidates.append((str(label), bx, by, 0.0))
        expected = None
        if self.global_target_mode and self.current_target is not None:
            expected = self._to_base(
                float(self.current_target.x), float(self.current_target.y)
            )
        selected = select_body_candidate(
            candidates,
            grab_xy=(self.grab_x, self.grab_y),
            expected_xy=expected,
            expected_match_radius_m=self.global_body_target_match_radius_m,
            grab_search_radius_m=0.25,
        )
        return None if selected is None else (selected[0], selected[1], selected[2])

    def _reset_align_body_presence(self) -> None:
        """Require a fresh stopped Body streak after every ALIGN motion."""
        self._align_body_presence_last_seq = int(self._body_dets_seq)
        self._align_body_presence_count = 0
        self._align_body_presence_xy = None

    def _reset_align_body_loss_confirmation(self) -> None:
        """Forget a missing-frame streak as soon as Body sees the target again."""
        self._align_body_missing_since_s = None
        self._align_body_missing_last_seq = int(getattr(self, "_body_dets_seq", -1))
        self._align_body_missing_frames = 0

    def _align_body_loss_confirmed(self, now_s: float) -> bool:
        """Debounce Body loss using time plus distinct DetectionArray frames."""
        if self._align_body_missing_since_s is None:
            self._align_body_missing_since_s = float(now_s)
        seq = int(getattr(self, "_body_dets_seq", -1))
        if seq != self._align_body_missing_last_seq:
            self._align_body_missing_last_seq = seq
            self._align_body_missing_frames += 1
        elapsed = float(now_s) - float(self._align_body_missing_since_s)
        return global_target_absence_confirmed(
            elapsed_s=elapsed,
            fresh_missing_body_frames=self._align_body_missing_frames,
            timeout_sec=self.align_body_lost_confirm_sec,
            confirm_frames=self.align_body_lost_confirm_frames,
        )

    def _global_target_is_deferred(self, target) -> bool:
        """Ignore a recently failed physical position without permanently blacklisting it."""
        now_fn = getattr(self, "_now_s", None)
        now = float(now_fn()) if callable(now_fn) else 0.0
        deferred = [
            entry
            for entry in getattr(self, "_global_deferred_targets", [])
            if entry[3] > now
        ]
        self._global_deferred_targets = deferred
        return target_is_spatially_deferred(
            target,
            deferred,
            now,
            getattr(self, "global_body_target_match_radius_m", 0.18),
        )

    def _defer_current_global_target(self, reason: str) -> None:
        """Temporarily suppress the current location after one failed Body reacquisition."""
        if not self.global_target_mode or self.current_target is None:
            return
        x, y = (
            self._appr_tgt_xy
            if self._appr_tgt_xy is not None
            else (float(self.current_target.x), float(self.current_target.y))
        )
        until = self._now_s() + self.align_body_lost_reselect_cooldown_sec
        self._global_deferred_targets.append(
            (int(self.current_target.set_type), float(x), float(y), float(until))
        )
        self.get_logger().info(
            f"GLOBAL: defer target position ({x:.2f},{y:.2f}) for "
            f"{self.align_body_lost_reselect_cooldown_sec:.1f}s ({reason})"
        )
        self._decide(
            f"GLOBAL DEFER {self.align_body_lost_reselect_cooldown_sec:.1f}s ({reason})"
        )

    def _handle_global_approach_route_blocked(self, now_s: float) -> bool:
        """Do not let one unreachable object freeze the whole global lane tour."""
        if (
            not getattr(self, "global_target_mode", False)
            or not getattr(self, "global_approach_route_blocked_defer_enabled", True)
            or getattr(self, "state", "") != "APPROACH"
            or self.current_target is None
            or self._appr_tgt_xy is None
        ):
            self._approach_route_blocked_since_s = None
            self._approach_route_blocked_attempts = 0
            return False
        if not getattr(self, "_last_drive_route_blocked", False):
            self._approach_route_blocked_since_s = None
            self._approach_route_blocked_attempts = 0
            return False

        if self._approach_route_blocked_since_s is None:
            self._approach_route_blocked_since_s = float(now_s)
            self._approach_route_blocked_attempts = 0
        self._approach_route_blocked_attempts += 1
        elapsed = float(now_s) - float(self._approach_route_blocked_since_s)
        if (
            elapsed < self.global_approach_route_blocked_confirm_sec
            or self._approach_route_blocked_attempts < (
                self.global_approach_route_blocked_confirm_attempts
            )
        ):
            return False

        target_id = int(getattr(self.current_target, "id", 0))
        target_xy = self._appr_tgt_xy
        self._defer_current_global_target("APPROACH route unavailable")
        self.current_target = None
        self._appr_tgt_xy = None
        self._standoff_arrived_s = None
        self._opportunistic_set2_active = False
        self._plan = None
        self._plan_dest = None
        self._last_drive_route_blocked = False
        self._drive(0.0, 0.0, 0.0)
        self._decide(
            f"APPROACH ROUTE BLOCKED target=#{target_id} "
            f"({target_xy[0]:.2f},{target_xy[1]:.2f}) -> DEFER / RESUME LANE"
        )
        self._enter("SCAN")
        return True

    def _reset_global_target_absence(self) -> None:
        """Start a fresh world+Body absence confirmation window."""
        self._global_target_absent_since_s = None
        self._global_target_absent_last_body_seq = int(
            getattr(self, "_body_dets_seq", -1)
        )
        self._global_target_absent_frames = 0

    def _global_target_absence_guard_enabled(self) -> bool:
        """Limit the fast stale-target exit to non-slot whole-map approaches."""
        return bool(
            getattr(self, "global_target_mode", False)
            and self.current_target is not None
            and not bool(self.current_target.blacklisted)
            and not self._is_locally_blacklisted(int(self.current_target.id))
            and self._current_anchor_slot() is None
            and not self._set2_slot_mode()
        )

    def _committed_set2_target_can_try_align(self) -> bool:
        """Keep visiting a target-fruit-marked physical position despite transient track loss.

        The public fruit label is routing evidence only, so this never authorizes PICK.  It
        merely preserves the frozen approach coordinate long enough for the gripper-bound,
        fresh Body SigLIP gate in CLASSIFY to make the final decision.
        """
        target = self.current_target
        return bool(
            target is not None
            and int(target.set_type) == 2
            and str(target.fruit_label) == str(getattr(self, "set2_label", ""))
            and bool(str(target.fruit_label))
            and self._appr_tgt_xy is not None
        )

    def _committed_global_target_can_try_align(self) -> bool:
        """Keep a Wide-confirmed goal until its frozen position gets a Body visit."""
        return bool(
            getattr(self, "global_target_mode", False)
            and getattr(self, "_global_approach_committed_from_wide", False)
            and self.current_target is not None
            and self._appr_tgt_xy is not None
        )

    def _align_body_lost_backoff_limit(self) -> int:
        """Reserve extra reacquisition attempts for the committed target-fruit workflow."""
        configured = max(0, int(self.align_body_lost_max_backoffs))
        if self._committed_set2_target_can_try_align():
            return configured
        return min(1, configured)

    def _handle_absent_global_target(
        self,
        now: float,
        body_point: tuple | None,
        *,
        context: str,
    ) -> bool:
        """Hold briefly, then reselect a vanished global target.

        ``True`` means this tick was fully handled (holding or transitioned). A Body point
        spatially associated with the frozen map position always wins over world-track dropout.
        """
        if not self._global_target_absence_guard_enabled():
            self._reset_global_target_absence()
            return False
        if body_point is not None or self._align_target_still_in_world():
            self._reset_global_target_absence()
            return False
        if (
            context in {"APPROACH stand-off", "ALIGN"}
            and self._committed_global_target_can_try_align()
        ):
            self._reset_global_target_absence()
            if not self._set2_track_loss_align_override_logged:
                self._set2_track_loss_align_override_logged = True
                self.get_logger().warn(
                    f"{context}: Wide-confirmed target #{self.current_target.id} lost its "
                    "world track -> preserve position and grant Body reacquisition"
                )
                self._decide(
                    f"WIDE TARGET COMMITTED #{self.current_target.id} "
                    "-> KEEP POSITION / BODY VERIFY"
                )
            return False

        seq = int(getattr(self, "_body_dets_seq", -1))
        if self._global_target_absent_since_s is None:
            self._global_target_absent_since_s = float(now)
            self._global_target_absent_last_body_seq = seq
            self._global_target_absent_frames = 0
            self.get_logger().info(
                f"{context}: selected global target #{self.current_target.id} is absent "
                "from world and Body -> confirming"
            )
        elif seq != self._global_target_absent_last_body_seq:
            self._global_target_absent_last_body_seq = seq
            self._global_target_absent_frames += 1

        self._drive(0.0, 0.0, 0.0)
        elapsed = float(now) - float(self._global_target_absent_since_s)
        if not global_target_absence_confirmed(
            elapsed_s=elapsed,
            fresh_missing_body_frames=self._global_target_absent_frames,
            timeout_sec=self.global_target_absent_timeout_sec,
            confirm_frames=self.global_target_absent_confirm_frames,
        ):
            return True

        stale_id = int(self.current_target.id)
        self.get_logger().warn(
            f"GLOBAL TARGET DROP #{stale_id}: absent from world and matching Body "
            f"for {elapsed:.2f}s/{self._global_target_absent_frames} fresh frames "
            f"during {context} -> reselect"
        )
        self._decide(
            f"GLOBAL TARGET ABSENT #{stale_id} "
            f"{elapsed:.2f}s/{self._global_target_absent_frames}f -> RESELECT"
        )
        # Blacklist only this short-lived tracker ID. A real object that reappears under a new
        # world-model ID remains eligible, while the vanished ID cannot immediately thrash back in.
        self._blacklist(stale_id)
        self.current_target = None
        self._appr_tgt_xy = None
        self._set2_identity_latch = None
        self.set_type = 0
        self._opportunistic_set2_active = False
        self._reset_global_target_absence()
        self._enter("SELECT_TARGET")
        return True

    def _update_align_body_presence(
        self,
        body_point: tuple[str, float, float, float] | None,
    ) -> tuple[int, bool]:
        """Count spatially continuous Body observations, once per fresh camera frame.

        Label flicker is intentionally ignored here: stopped CLASSIFY owns identity.  The spatial
        continuity gate prevents two nearby detections from being counted as one physical object.
        """
        seq = int(self._body_dets_seq)
        if seq == self._align_body_presence_last_seq:
            return self._align_body_presence_count, False
        self._align_body_presence_last_seq = seq
        if body_point is None:
            self._align_body_presence_count = 0
            self._align_body_presence_xy = None
            return 0, True

        _, bx, by, _ = body_point
        point = (float(bx), float(by))
        continuity_radius = max(0.03, float(self.align_tol))
        if (
            self._align_body_presence_xy is not None
            and math.hypot(
                point[0] - self._align_body_presence_xy[0],
                point[1] - self._align_body_presence_xy[1],
            ) <= continuity_radius
        ):
            self._align_body_presence_count += 1
        else:
            self._align_body_presence_count = 1
        self._align_body_presence_xy = point
        return self._align_body_presence_count, True

    def _reset_set1_final_classification(self) -> None:
        """Start a fresh, stopped final vote window before moving the arm."""
        self._classify_body_last_seq = int(self._body_dets_seq)
        self._classify_wide_last_seq = int(getattr(self, "_wide_relative_seq", 0))
        self._classify_body_frame_count = 0
        self._classify_body_target_count = 0
        self._classify_body_other_counts = {}

    def _set1_final_uses_wide(self) -> bool:
        """Use Wide identity only for the known close-range octahedron failure mode."""
        return bool(getattr(self, "classify_octa_wide_final_enabled", True)) and (
            str(self.set1_label) == "octahedron"
        )

    def _update_set1_final_classification(self) -> tuple[str, str, bool]:
        """Consume one fresh source frame and return (decision, observed_label, fresh)."""
        use_wide = self._set1_final_uses_wide()
        if use_wide:
            seq = int(getattr(self, "_wide_relative_seq", 0))
            if seq == self._classify_wide_last_seq:
                return "pending", "", False
            self._classify_wide_last_seq = seq
        else:
            seq = int(self._body_dets_seq)
            if seq == self._classify_body_last_seq:
                return "pending", "", False
            self._classify_body_last_seq = seq
        self._classify_body_frame_count += 1

        observed_label = ""
        if use_wide:
            # Wide owns only identity. A fresh Body detection must still prove that some physical
            # object remains at the aligned grab point; its close-range label is intentionally
            # ignored for octahedra.
            observation = self._wide_align_target_observation()
            if (
                self._fresh_body_set1_pick_presence() is not None
                and observation is not None
                and float(observation.confidence) >= max(
                    self.pick_track_conf, self.conf_threshold
                )
            ):
                observed_label = str(observation.label)
        elif self._fresh_body_set1_pick_candidate() is not None:
            observed_label = str(self.set1_label)
        else:
            other = self._body_nearest_any()
            if other is not None and str(other[0]) != str(self.set1_label):
                observed_label = str(other[0])

        if observed_label == str(self.set1_label):
            self._classify_body_target_count += 1
        elif observed_label:
            self._classify_body_other_counts[observed_label] = (
                self._classify_body_other_counts.get(observed_label, 0) + 1
            )

        required = (
            int(self.classify_wide_confirm_frames)
            if use_wide
            else int(self.align_distractor_confirm_frames)
        )
        if self._classify_body_target_count >= required:
            return "target", observed_label, True
        for label, count in self._classify_body_other_counts.items():
            if count >= required:
                return "distractor", label, True
        if self._classify_body_frame_count >= int(self.classify_body_max_frames):
            return "inconclusive", observed_label, True
        return "pending", observed_label, True

    def _body_set1_distractor_in_front(self) -> tuple[str, float, float] | None:
        """Set2 slot guard: any Set1 body detection in the near front view means the slot is wrong."""
        if self._body_H is None or not self._body_dets:
            return None
        best = None
        bestd = float("inf")
        max_x = max(0.0, self.set2_slot_distractor_front_x_m)
        max_y = max(0.0, self.set2_slot_distractor_side_y_m)
        for u, v, label, _ in self._body_dets:
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
        if self.anchor_mission_enabled and (slot := self._current_anchor_slot()) is not None:
            fresh = self._anchor_track_for_slot(slot)
            if fresh is not None:
                self.current_target = fresh
                return True
            # Wide's one-shot inventory remains the occupancy authority even if its
            # live track is briefly occluded at close range.
            return slot.state == "OCCUPIED"
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
        if self.global_target_mode:
            # Track IDs may churn, but only a same-set replacement at the frozen physical
            # position is the same target. The old nearest-phase fallback had no radius and could
            # let a distant banana keep a vanished target alive indefinitely.
            ref = (
                self._appr_tgt_xy
                if self._appr_tgt_xy is not None
                else (float(self.current_target.x), float(self.current_target.y))
            )
            set_type = int(self.current_target.set_type)
            candidates = [
                obj
                for obj in self.world.objects
                if (
                    not obj.blacklisted
                    and not self._is_locally_blacklisted(int(obj.id))
                    and int(obj.set_type) == set_type
                    and float(obj.confidence) >= self.pick_track_conf
                    and math.hypot(float(obj.x) - ref[0], float(obj.y) - ref[1])
                    <= self.global_body_target_match_radius_m
                )
            ]
            replacement = min(
                candidates,
                key=lambda obj: math.hypot(
                    float(obj.x) - ref[0], float(obj.y) - ref[1]
                ),
                default=None,
            )
            if replacement is not None:
                self.current_target = replacement
                return True
            return False
        ref = (float(self.current_target.x), float(self.current_target.y))
        return self._current_object_for_approach(ref_xy=ref) is not None

    def _wide_align_expected_base(self) -> tuple[float, float] | None:
        """Return the current expected base-frame point for raw-Wide target association."""
        expected: tuple[float, float] | None = None
        anchor_slot = self._current_anchor_slot()
        if self._relative_mode() and anchor_slot is not None:
            point = self._relative_slot_point(anchor_slot)
            if point is not None:
                expected = (float(point[0]), float(point[1]))

        if expected is None and self.current_target is not None and self.world is not None:
            fresh = self._lookup_object(int(self.current_target.id))
            if fresh is not None and not fresh.blacklisted:
                self.current_target = fresh
            point = self._to_base(float(self.current_target.x), float(self.current_target.y))
            if point is not None:
                expected = (float(point[0]), float(point[1]))

        slot = self._current_slot()
        if expected is None and slot is not None:
            point = self._to_base(float(slot.x), float(slot.y))
            if point is not None:
                expected = (float(point[0]), float(point[1]))

        if expected is None and anchor_slot is not None:
            point = self._to_base(float(anchor_slot.x), float(anchor_slot.y))
            if point is not None:
                expected = (float(point[0]), float(point[1]))

        if expected is None and self._appr_tgt_xy is not None:
            point = self._to_base(float(self._appr_tgt_xy[0]), float(self._appr_tgt_xy[1]))
            if point is not None:
                expected = (float(point[0]), float(point[1]))

        return expected

    def _wide_align_target_observation(self) -> RelativeObservation | None:
        """Return the fresh labelled raw-Wide observation associated with the target."""
        expected = self._wide_align_expected_base()

        last_frame_s = self._wide_relative_last_frame_s
        if (
            last_frame_s is None
            or self._now_s() - last_frame_s > self.align_wide_max_age_sec
        ):
            return None
        return nearest_align_wide_observation(
            expected,
            self._wide_relative_labeled_observations,
            self.align_wide_match_radius_m,
        )

    def _wide_align_target_base(self) -> tuple[float, float] | None:
        """Return fresh raw-Wide geometry associated to the latched world/slot target."""
        observation = self._wide_align_target_observation()
        if observation is None:
            return None
        return float(observation.x), float(observation.y)

    def _drive(self, vx: float, vy: float, omega: float = 0.0) -> None:
        requested_vx, requested_vy = float(vx), float(vy)
        if self.world is not None and hasattr(self, "_planner"):
            vx, vy = clamp_outward_field_velocity(
                requested_vx,
                requested_vy,
                float(self.world.robot_theta),
                float(self.world.robot_x),
                float(self.world.robot_y),
                self._planner.interior,
                getattr(self, "field_boundary_guard_m", 0.03),
            )
            if abs(vx - requested_vx) > 1e-6 or abs(vy - requested_vy) > 1e-6:
                self.get_logger().warn(
                    "field boundary guard removed outward translation",
                    throttle_duration_sec=1.0,
                )
        now = self._now_s()
        self._predict_relative_command_translation(now)
        self._relative_last_command = (float(vx), float(vy), float(omega))
        self._last_drive_command = (float(vx), float(vy), float(omega))
        c = BaseCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = "base_link"
        c.vx, c.vy, c.omega = float(vx), float(vy), float(omega)
        self.pub_cmd.publish(c)

    def _pulsed_heading_controller(self) -> PulsedHeadingController:
        controller = getattr(self, "_pulsed_heading", None)
        if controller is None:
            controller = PulsedHeadingController()
            self._pulsed_heading = controller
        return controller

    def _reset_pulsed_heading(self) -> None:
        controller = getattr(self, "_pulsed_heading", None)
        if controller is not None:
            controller.reset()

    def _turn_in_place_pulsed(
        self,
        target_heading: float,
        tolerance_rad: float,
        key: object,
        omega_limit: float | None = None,
        *,
        stop_when_aligned: bool = True,
    ) -> bool:
        """Rotate with torque-restoring pulses and zero translation between measurements."""
        if self.world is None:
            self._drive(0.0, 0.0, 0.0)
            return False
        aligned, omega, event = self._pulsed_heading_controller().step(
            now_s=self._now_s(),
            current_heading=float(self.world.robot_theta),
            target_heading=float(target_heading),
            tolerance_rad=float(tolerance_rad),
            key=key,
            omega_limit=omega_limit,
        )
        if not aligned or stop_when_aligned:
            self._drive(0.0, 0.0, omega)
        if event == "continuous_turn":
            self._decide(
                f"TURN CONTINUOUS target={math.degrees(target_heading):+.1f}deg"
            )
        elif event in {"coarse_pulse", "fine_pulse"}:
            self._decide(
                f"TURN {event.upper()} target={math.degrees(target_heading):+.1f}deg"
            )
        elif event == "timeout":
            self.get_logger().error(
                "pulsed in-place turn did not converge; holding base stopped",
                throttle_duration_sec=2.0,
            )
        return aligned

    def _drive_toward_direct(
        self,
        dest_x: float,
        dest_y: float,
        yaw: float = 0.0,
        speed_limit_mps: float | None = None,
        speed_override_mps: float | None = None,
    ) -> None:
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
            self._turn_in_place_pulsed(
                yaw,
                self.direct_nav_face_tol,
                ("direct_final", round(float(dest_x), 3), round(float(dest_y), 3)),
                self.direct_nav_omega_max,
            )
            return
        target_heading = math.atan2(
            float(dest_y) - float(self.world.robot_y),
            float(dest_x) - float(self.world.robot_x),
        )
        aligned = self._turn_in_place_pulsed(
            target_heading,
            self.direct_nav_face_tol,
            ("direct_travel", round(float(dest_x), 3), round(float(dest_y), 3)),
            self.direct_nav_omega_max,
            stop_when_aligned=False,
        )
        if aligned:
            speed = max(
                0.0,
                self.direct_nav_speed
                if speed_override_mps is None
                else float(speed_override_mps),
            )
            if speed_limit_mps is not None:
                speed = min(speed, max(0.0, float(speed_limit_mps)))
            self._drive(speed, 0.0, 0.0)

    def _drive_toward_holonomic(
        self,
        dest_x: float,
        dest_y: float,
        speed_limit_mps: float | None = None,
        speed_override_mps: float | None = None,
    ) -> None:
        """Move toward a waypoint with vx/vy while preserving the current heading."""
        base = self._to_base(dest_x, dest_y)
        if base is None:
            self._drive(0.0, 0.0, 0.0)
            return
        speed = max(
            0.0,
            self.direct_nav_speed
            if speed_override_mps is None
            else float(speed_override_mps),
        )
        if speed_limit_mps is not None:
            speed = min(speed, max(0.0, float(speed_limit_mps)))
        vx, vy = holonomic_waypoint_velocity(
            base[0], base[1], speed, self.direct_nav_stop_radius_m
        )
        self._drive(vx, vy, 0.0)

    def _rotate_to_heading(
        self, target_heading: float, tolerance_rad: float, omega_max: float
    ) -> bool:
        """Pulse in place until measured robot heading reaches the requested field heading."""
        return self._turn_in_place_pulsed(
            target_heading,
            tolerance_rad,
            ("rotate", getattr(self, "state", ""), round(float(target_heading), 3)),
            omega_max,
        )

    def _drive_to_post_pick_lane_entry(self, dest_x: float, dest_y: float) -> None:
        """Face the first lane waypoint, then enter it with zero commanded yaw."""
        if self.world is None:
            self._drive(0.0, 0.0, 0.0)
            return
        dx = float(dest_x) - float(self.world.robot_x)
        dy = float(dest_y) - float(self.world.robot_y)
        if math.hypot(dx, dy) <= self.wp_reach_tol_m:
            self._drive(0.0, 0.0, 0.0)
            return
        target_heading = math.atan2(dy, dx)
        aligned = self._turn_in_place_pulsed(
            target_heading,
            self.lane_heading_align_tolerance_rad,
            ("post_pick_entry", round(float(dest_x), 3), round(float(dest_y), 3)),
            self.direct_nav_omega_max,
            stop_when_aligned=False,
        )
        if aligned:
            self._drive(max(0.0, self.direct_nav_speed), 0.0, 0.0)

    def _reset_approach_motion(self) -> None:
        self._approach_standoff_xy = None
        self._approach_heading_latched = None
        self._approach_route_mode = "grid_only"
        self._approach_motion_phase = "travel"
        self._approach_brake_start_s = 0.0
        self._approach_heading_filtered = None
        self._approach_heading_stable_count = 0
        self._approach_route_blocked_since_s = None
        self._approach_route_blocked_attempts = 0

    def _latch_approach_goal(self, target_xy: tuple[float, float]) -> bool:
        """Freeze one stand-off point and target heading for the current APPROACH."""
        if self.world is None:
            return False
        gx, gy = float(target_xy[0]), float(target_xy[1])
        if self.planner_enabled:
            robot_xy = self._robot_xy()
            if robot_xy is None:
                return False
            exclude_id = int(self.current_target.id) if self.current_target is not None else 0
            obstacles = self._obstacles_snapshot(exclude_id, (gx, gy))
            route_mode = self._global_exploration_route_mode()
            target_distance = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
            if target_distance <= self.approach_dist_m + self.wp_reach_tol_m:
                stand_off = robot_xy
                self._approach_route_mode = "legacy"
                self._decide(
                    f"APPROACH TARGET ALREADY NEAR d={target_distance:.2f}m "
                    "-> BODY ALIGN"
                )
            else:
                result = self._planner.plan_object_standoff(
                    robot_xy,
                    (gx, gy),
                    obstacles,
                    route_mode=route_mode,
                )
                if result is None:
                    stand_off = radial_object_standoff(
                        robot_xy, (gx, gy), self.approach_dist_m
                    )
                    self._approach_route_mode = "legacy"
                    self._decide(
                        "APPROACH GRID STAND-OFF unavailable "
                        f"-> DIRECT ({stand_off[0]:.2f},{stand_off[1]:.2f})"
                    )
                else:
                    stand_off, _ = result
                    self._approach_route_mode = route_mode
        else:
            entry_bearing = math.atan2(
                gy - float(self.world.robot_y), gx - float(self.world.robot_x)
            )
            stand_off = (
                gx - self.approach_dist_m * math.cos(entry_bearing),
                gy - self.approach_dist_m * math.sin(entry_bearing),
            )
        bearing = math.atan2(gy - stand_off[1], gx - stand_off[0])
        self._approach_heading_latched = bearing
        self._approach_standoff_xy = stand_off
        self._approach_motion_phase = "travel"
        self._approach_brake_start_s = 0.0
        self._approach_heading_filtered = None
        self._approach_heading_stable_count = 0
        self._decide(
            f"APPROACH GOAL ({self._approach_standoff_xy[0]:.2f},"
            f"{self._approach_standoff_xy[1]:.2f}) "
            f"heading={math.degrees(bearing):+.1f}deg"
        )
        return True

    def _step_approach_arrival(
        self,
        distance: float,
        heading_tolerance_rad: float,
        omega_max: float,
        body_target_visible: bool = False,
    ) -> tuple[bool, bool]:
        """Brake and settle once, then confirm filtered heading for consecutive frames."""
        if self.world is None or self._approach_heading_latched is None:
            self._drive(0.0, 0.0, 0.0)
            return True, False

        now = self._now_s()
        if self._approach_motion_phase == "travel":
            if distance >= self.approach_standoff_tol:
                return False, False
            self._approach_motion_phase = "brake_settle"
            self._approach_brake_start_s = now
            self._approach_heading_filtered = float(self.world.robot_theta)
            self._approach_heading_stable_count = 0
            self._decide(
                f"APPROACH STAND-OFF REACHED d={distance:.2f}m -> BRAKE"
            )

        if self._approach_motion_phase == "brake_settle":
            self._drive(0.0, 0.0, 0.0)
            self._approach_heading_filtered = float(self.world.robot_theta)
            if now - self._approach_brake_start_s >= self.approach_brake_settle_sec:
                if body_target_visible:
                    self._approach_motion_phase = "ready"
                    self._approach_heading_stable_count = 0
                    self._decide(
                        "APPROACH BODY TARGET VISIBLE -> SKIP MAP HEADING / ALIGN"
                    )
                    return True, True
                self._approach_motion_phase = "heading_align"
                self._approach_heading_stable_count = 0
                self._decide("APPROACH BRAKE SETTLED -> HEADING ALIGN")
            return True, False

        if self._approach_motion_phase == "ready":
            self._drive(0.0, 0.0, 0.0)
            return True, True

        if body_target_visible:
            self._reset_pulsed_heading()
            self._drive(0.0, 0.0, 0.0)
            self._approach_motion_phase = "ready"
            self._approach_heading_stable_count = 0
            self._decide(
                "APPROACH BODY TARGET VISIBLE -> STOP HEADING TURN / ALIGN"
            )
            return True, True

        self._approach_heading_filtered = circular_heading_filter(
            self._approach_heading_filtered,
            float(self.world.robot_theta),
            self.approach_heading_filter_alpha,
        )
        error = self._wrap_pi(
            self._approach_heading_latched - self._approach_heading_filtered
        )
        if abs(error) <= max(0.0, float(heading_tolerance_rad)):
            self._reset_pulsed_heading()
            self._approach_heading_stable_count += 1
            self._drive(0.0, 0.0, 0.0)
            if self._approach_heading_stable_count >= self.approach_heading_stable_frames:
                self._approach_motion_phase = "ready"
                self._decide(
                    f"APPROACH HEADING STABLE "
                    f"{self._approach_heading_stable_count}/{self.approach_heading_stable_frames}"
                )
                return True, True
            return True, False

        self._approach_heading_stable_count = 0
        self._turn_in_place_pulsed(
            self._approach_heading_latched,
            heading_tolerance_rad,
            (
                "approach_heading",
                round(float(self._approach_standoff_xy[0]), 3)
                if self._approach_standoff_xy is not None
                else 0.0,
                round(float(self._approach_standoff_xy[1]), 3)
                if self._approach_standoff_xy is not None
                else 0.0,
            ),
            omega_max,
        )
        return True, False

    def _reset_object_connector_brake(self) -> None:
        self._object_connector_brake_key = None
        self._object_connector_brake_start_s = 0.0

    def _object_connector_brake_required(self, waypoint_idx: int) -> bool:
        if (
            self._plan_route_mode != "object_approach"
            or self._plan is None
            or waypoint_idx != len(self._plan) - 2
        ):
            return False
        return True

    def _step_object_connector_brake(self, waypoint_idx: int) -> bool:
        """Hold the last lane waypoint before entering the final diagonal connector."""
        key = (self._plan_generation, int(waypoint_idx))
        now = self._now_s()
        if self._object_connector_brake_key != key:
            self._object_connector_brake_key = key
            self._object_connector_brake_start_s = now
            waypoint = self._plan[waypoint_idx]
            self._decide(
                f"OBJECT CONNECTOR BRAKE ({waypoint[0]:.2f},{waypoint[1]:.2f})"
            )
        self._drive(0.0, 0.0, 0.0)
        if now - self._object_connector_brake_start_s < self.approach_brake_settle_sec:
            return True
        self._decide("OBJECT CONNECTOR BRAKE SETTLED -> FINAL APPROACH")
        self._reset_object_connector_brake()
        return False

    def _drive_cardinal_lane_segment(
        self,
        start: tuple[float, float],
        dest: tuple[float, float],
        segment_key: tuple[object, int],
        *,
        force_initial_align: bool = False,
        forward_only: bool = False,
        stop_radius_m: float | None = None,
        speed_limit_mps: float | None = None,
        speed_override_mps: float | None = None,
        allow_non_cardinal: bool = False,
    ) -> bool:
        """Face a cardinal lane leg, then use pose feedback to stay in its centre band."""
        if not self.lane_heading_lock_enabled or self.world is None:
            return False
        heading = cardinal_segment_heading(
            start, dest, self.lane_heading_axis_tolerance_m
        )
        if heading is None:
            if not allow_non_cardinal:
                return False
            sx, sy = float(start[0]), float(start[1])
            dx = float(dest[0]) - sx
            dy = float(dest[1]) - sy
            if math.hypot(dx, dy) <= 1e-6:
                return False
            heading = math.atan2(dy, dx)
        now = self._now_s()
        raw_heading = float(self.world.robot_theta)
        error = self._wrap_pi(heading - raw_heading)
        if self._lane_heading_segment_key != segment_key:
            self._reset_pulsed_heading()
            self._lane_heading_segment_key = segment_key
            self._lane_heading_filtered = raw_heading
            self._lane_heading_violation_start_s = None
            self._lane_heading_realign_armed_at_s = now + self.lane_heading_realign_arm_sec
            self._lane_heading_settle_start_s = 0.0
            self._lane_heading_entry_start_s = 0.0
            self._lane_heading_turn_sign = 0
            self._lane_heading_reverse_start_s = 0.0
            self._lane_cross_track_filtered = None
            self._lane_cross_track_outside = False
            self._decide(f"LANE HEADING {math.degrees(heading):+.0f}deg")
            if abs(error) <= self.lane_heading_align_tolerance_rad:
                self._lane_heading_phase = "drive"
            else:
                self._lane_heading_phase = "align"
                self._decide(
                    f"LANE ENTRY PULSE ALIGN error={math.degrees(error):+.1f}deg"
                )

        stop_radius = (
            self.direct_nav_stop_radius_m
            if stop_radius_m is None
            else max(0.0, float(stop_radius_m))
        )
        if self._distance_to(dest[0], dest[1]) <= stop_radius:
            self._drive(0.0, 0.0, 0.0)
            return True

        if self._lane_heading_phase == "reverse_settle":
            self._drive(0.0, 0.0, 0.0)
            self._reset_pulsed_heading()
            self._lane_heading_filtered = raw_heading
            if now - self._lane_heading_reverse_start_s >= self.lane_heading_reverse_settle_sec:
                self._lane_heading_phase = "align"
                self._lane_heading_reverse_start_s = 0.0
                self._lane_heading_turn_sign = 0
            return True

        if self._lane_heading_phase == "align":
            aligned = self._turn_in_place_pulsed(
                heading,
                self.lane_heading_align_tolerance_rad,
                ("lane_heading", segment_key),
                self.lane_heading_omega_max,
            )
            if aligned:
                self._lane_heading_phase = "settle"
                self._lane_heading_settle_start_s = now
                self._lane_heading_filtered = raw_heading
                self._lane_heading_turn_sign = 0
            return True

        if self._lane_heading_phase == "settle":
            self._drive(0.0, 0.0, 0.0)
            self._lane_heading_filtered = raw_heading
            if now - self._lane_heading_settle_start_s >= self.lane_heading_settle_sec:
                self._lane_heading_phase = "drive"
                self._lane_heading_violation_start_s = None
                self._lane_heading_realign_armed_at_s = now + self.lane_heading_realign_arm_sec
                self._lane_heading_entry_start_s = 0.0
            return True

        self._lane_heading_filtered = circular_heading_filter(
            self._lane_heading_filtered,
            raw_heading,
            self.lane_heading_filter_alpha,
        )
        filtered_error = self._wrap_pi(heading - self._lane_heading_filtered)
        if now < self._lane_heading_realign_armed_at_s:
            self._lane_heading_violation_start_s = None
            realign = False
        else:
            self._lane_heading_violation_start_s, realign = lane_heading_violation_time_step(
                filtered_error,
                self.lane_heading_realign_tolerance_rad,
                self._lane_heading_violation_start_s,
                now,
                self.lane_heading_realign_hold_sec,
            )
        if realign:
            self._drive(0.0, 0.0, 0.0)
            self._reset_pulsed_heading()
            self._lane_heading_phase = "align"
            self._lane_heading_filtered = raw_heading
            self._lane_heading_violation_start_s = None
            self._lane_heading_entry_start_s = 0.0
            self._lane_heading_turn_sign = 0
            self._lane_heading_reverse_start_s = 0.0
            self._decide(
                f"LANE RE-ALIGN error={math.degrees(filtered_error):+.1f}deg "
                f"held={self.lane_heading_realign_hold_sec:.2f}s"
            )
            return True

        base = self._to_base(dest[0], dest[1])
        if base is None:
            self._drive(0.0, 0.0, 0.0)
            return True
        bx, by = base
        base_dist = math.hypot(bx, by)
        if base_dist <= 1e-6:
            self._drive(0.0, 0.0, 0.0)
            return True
        speed = max(
            0.0,
            self.direct_nav_speed
            if speed_override_mps is None
            else float(speed_override_mps),
        )
        if speed_limit_mps is not None:
            speed = min(speed, max(0.0, float(speed_limit_mps)))

        control_error = filtered_error
        cross_track_error = 0.0
        filtered_cross_track = 0.0
        heading_bias = 0.0
        lane_feedback_enabled = bool(getattr(self, "lane_cross_track_enabled", False))
        if lane_feedback_enabled:
            cross_track_error, _, _ = lane_cross_track_feedback(
                start,
                dest,
                (float(self.world.robot_x), float(self.world.robot_y)),
                self.lane_center_band_half_width_m,
                self.lane_cross_track_lookahead_m,
                self.lane_cross_track_gain,
                self.lane_cross_track_max_heading_bias_rad,
            )
            previous_cross_track = getattr(self, "_lane_cross_track_filtered", None)
            if previous_cross_track is None:
                filtered_cross_track = cross_track_error
            else:
                filtered_cross_track = float(previous_cross_track) + (
                    self.lane_cross_track_filter_alpha
                    * (cross_track_error - float(previous_cross_track))
                )
            self._lane_cross_track_filtered = filtered_cross_track
            effective_cross_track = math.copysign(
                max(
                    0.0,
                    abs(filtered_cross_track) - self.lane_center_band_half_width_m,
                ),
                filtered_cross_track,
            )
            heading_bias = -math.atan2(
                self.lane_cross_track_gain * effective_cross_track,
                self.lane_cross_track_lookahead_m,
            )
            heading_bias = max(
                -self.lane_cross_track_max_heading_bias_rad,
                min(self.lane_cross_track_max_heading_bias_rad, heading_bias),
            )
            control_heading = self._wrap_pi(heading + heading_bias)
            control_error = self._wrap_pi(control_heading - self._lane_heading_filtered)

            outside = abs(filtered_cross_track) > self.lane_center_band_half_width_m
            if outside != bool(getattr(self, "_lane_cross_track_outside", False)):
                state = "EXIT" if outside else "RECOVERED"
                self._decide(
                    f"LANE CENTER {state} cte={filtered_cross_track * 100.0:+.1f}cm "
                    f"band=+/-{self.lane_center_band_half_width_m * 100.0:.0f}cm"
                )
            self._lane_cross_track_outside = outside

            error_abs = abs(filtered_cross_track)
            if error_abs > self.lane_cross_track_slow_error_m:
                slow_span = (
                    self.lane_cross_track_full_slow_error_m
                    - self.lane_cross_track_slow_error_m
                )
                slow_ratio = min(
                    1.0,
                    (error_abs - self.lane_cross_track_slow_error_m) / slow_span,
                )
                speed = max(
                    self.lane_cross_track_min_speed_mps,
                    speed
                    - slow_ratio * (speed - self.lane_cross_track_min_speed_mps),
                )
        if now < self._lane_heading_realign_armed_at_s:
            speed = min(speed, self.lane_heading_soft_entry_speed)
            # Keep large intentional entry-angle differences translation-only, but correct a
            # small filtered drift immediately instead of allowing the hardware bias to build
            # throughout the re-align arming window.
            if abs(filtered_error) <= self.lane_heading_realign_tolerance_rad:
                omega = max(
                    -self.lane_heading_drive_omega_max,
                    min(
                        self.lane_heading_drive_omega_max,
                        (
                            self.lane_cross_track_heading_kp
                            if lane_feedback_enabled
                            else self.lane_heading_soft_entry_kp
                        )
                        * control_error,
                    ),
                )
                if (
                    abs(control_error) <= self.lane_heading_deadband_rad
                    and abs(filtered_cross_track) <= getattr(
                        self, "lane_center_band_half_width_m", 0.0
                    )
                ):
                    omega = 0.0
            else:
                omega = 0.0
        else:
            omega = max(
                -self.lane_heading_drive_omega_max,
                min(
                    self.lane_heading_drive_omega_max,
                    (
                        self.lane_cross_track_heading_kp
                        if lane_feedback_enabled
                        else self.lane_heading_soft_entry_kp
                    )
                    * control_error,
                ),
            )
            if (
                abs(control_error) <= self.lane_heading_deadband_rad
                and abs(filtered_cross_track) <= getattr(
                    self, "lane_center_band_half_width_m", 0.0
                )
            ):
                omega = 0.0
        if lane_feedback_enabled and hasattr(self, "get_logger"):
            self.get_logger().info(
                f"LANE feedback raw={cross_track_error * 100.0:+.1f}cm "
                f"filtered={filtered_cross_track * 100.0:+.1f}cm "
                f"bias={math.degrees(heading_bias):+.1f}deg "
                f"cmd=(vx={speed:.3f},omega={omega:+.3f})",
                throttle_duration_sec=1.0,
            )
        self._drive(speed, 0.0, omega)
        return True

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _robot_xy(self) -> tuple[float, float] | None:
        if self.world is None:
            return None
        return (self.world.robot_x, self.world.robot_y)

    def _robot_pose_tuple(self) -> tuple[float, float, float] | None:
        if self.world is None:
            return None
        return (
            float(self.world.robot_x),
            float(self.world.robot_y),
            float(self.world.robot_theta),
        )

    def _reset_stuck_escape(self) -> None:
        self._stuck_watch_started_s = None
        self._stuck_watch_pose = None
        self._stuck_watch_cmd = (0.0, 0.0, 0.0)
        self._stuck_escape_phase = None
        self._stuck_escape_phase_start_s = 0.0
        self._stuck_escape_cmd = (0.0, 0.0, 0.0)

    def _stuck_escape_allowed(self) -> bool:
        return bool(
            getattr(self, "stuck_escape_enabled", False)
            and self.state in {"SCAN", "APPROACH"}
            and self.world is not None
            and self._run_started
            and self._competition_state == "RUNNING"
        )

    def _stuck_command_active(self, cmd: tuple[float, float, float]) -> bool:
        vx, vy, omega = (float(cmd[0]), float(cmd[1]), float(cmd[2]))
        return bool(
            math.hypot(vx, vy) >= self.stuck_min_cmd_linear_mps
            or abs(omega) >= self.stuck_min_cmd_omega_radps
        )

    def _stuck_command_changed(
        self,
        prev: tuple[float, float, float],
        curr: tuple[float, float, float],
    ) -> bool:
        pvx, pvy, pom = (float(prev[0]), float(prev[1]), float(prev[2]))
        cvx, cvy, com = (float(curr[0]), float(curr[1]), float(curr[2]))
        linear_delta = math.hypot(cvx - pvx, cvy - pvy)
        omega_delta = abs(com - pom)
        prev_linear = math.hypot(pvx, pvy)
        curr_linear = math.hypot(cvx, cvy)
        if linear_delta > 0.03 or omega_delta > 0.05:
            return True
        if prev_linear >= self.stuck_min_cmd_linear_mps and curr_linear >= self.stuck_min_cmd_linear_mps:
            # A sign flip means the previous no-motion window should not be reused.
            return (pvx * cvx + pvy * cvy) < 0.0
        if abs(pom) >= self.stuck_min_cmd_omega_radps and abs(com) >= self.stuck_min_cmd_omega_radps:
            return (pom * com) < 0.0
        return False

    def _stuck_pose_moved(
        self,
        start: tuple[float, float, float],
        current: tuple[float, float, float],
    ) -> bool:
        dx = float(current[0]) - float(start[0])
        dy = float(current[1]) - float(start[1])
        dth = self._wrap_pi(float(current[2]) - float(start[2]))
        return bool(
            math.hypot(dx, dy) >= self.stuck_pose_delta_min_m
            or abs(dth) >= self.stuck_heading_delta_min_rad
        )

    def _opposite_stuck_escape_cmd(
        self, cmd: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        vx, vy, omega = (float(cmd[0]), float(cmd[1]), float(cmd[2]))
        if math.hypot(vx, vy) >= self.stuck_min_cmd_linear_mps:
            speed = self.stuck_escape_linear_speed
            if abs(vx) >= abs(vy):
                return (-math.copysign(speed, vx), 0.0, 0.0)
            return (0.0, -math.copysign(speed, vy), 0.0)
        if abs(omega) >= self.stuck_min_cmd_omega_radps:
            return (0.0, 0.0, -math.copysign(self.stuck_escape_omega_speed, omega))
        return (0.0, 0.0, 0.0)

    def _clear_active_route_for_replan(self) -> None:
        self._plan = None
        self._plan_start_xy = None
        self._plan_dest = None
        self._plan_escape = False
        self._plan_flexible_fallback = False
        self._lane_heading_segment_key = None
        self._lane_heading_phase = "align"
        self._lane_heading_filtered = None
        self._lane_heading_violation_start_s = None
        self._lane_heading_realign_armed_at_s = 0.0
        self._lane_heading_settle_start_s = 0.0
        self._lane_heading_entry_start_s = 0.0
        self._lane_heading_turn_sign = 0
        self._lane_heading_reverse_start_s = 0.0
        self._lane_cross_track_filtered = None
        self._lane_cross_track_outside = False
        self._last_drive_route_blocked = False
        self._approach_route_blocked_since_s = None
        self._approach_route_blocked_attempts = 0
        self._reset_object_connector_brake()

    def _start_stuck_escape(self, cmd: tuple[float, float, float]) -> None:
        self._stuck_escape_cmd = self._opposite_stuck_escape_cmd(cmd)
        self._stuck_escape_phase = "stop"
        self._stuck_escape_phase_start_s = self._now_s()
        self._stuck_escape_count += 1
        self._clear_active_route_for_replan()
        self._drive(0.0, 0.0, 0.0)
        vx, vy, omega = cmd
        evx, evy, eomega = self._stuck_escape_cmd
        self.get_logger().warn(
            "stuck escape: command did not move pose for "
            f"{self.stuck_detect_duration_sec:.1f}s; "
            f"cmd=({vx:+.3f},{vy:+.3f},{omega:+.3f}) "
            f"escape=({evx:+.3f},{evy:+.3f},{eomega:+.3f})"
        )
        self._decide(
            f"STUCK ESCAPE #{self._stuck_escape_count}: "
            f"cmd=({vx:+.2f},{vy:+.2f},{omega:+.2f}) -> "
            f"pulse=({evx:+.2f},{evy:+.2f},{eomega:+.2f}) / REPLAN"
        )

    def _update_stuck_watchdog(self) -> None:
        if self._stuck_escape_phase is not None:
            return
        if not self._stuck_escape_allowed():
            self._reset_stuck_escape()
            return
        pose = self._robot_pose_tuple()
        if pose is None:
            self._reset_stuck_escape()
            return
        cmd = self._last_drive_command
        if not self._stuck_command_active(cmd):
            self._stuck_watch_started_s = None
            self._stuck_watch_pose = None
            self._stuck_watch_cmd = (0.0, 0.0, 0.0)
            return
        if (
            self._stuck_watch_pose is None
            or self._stuck_watch_started_s is None
            or self._stuck_command_changed(self._stuck_watch_cmd, cmd)
        ):
            self._stuck_watch_started_s = self._now_s()
            self._stuck_watch_pose = pose
            self._stuck_watch_cmd = cmd
            return
        if self._stuck_pose_moved(self._stuck_watch_pose, pose):
            self._stuck_watch_started_s = self._now_s()
            self._stuck_watch_pose = pose
            self._stuck_watch_cmd = cmd
            return
        if self._now_s() - float(self._stuck_watch_started_s) >= self.stuck_detect_duration_sec:
            self._start_stuck_escape(cmd)

    def _step_stuck_escape(self) -> bool:
        if self._stuck_escape_phase is None:
            return False
        if not self._stuck_escape_allowed():
            self._reset_stuck_escape()
            return False

        now = self._now_s()
        elapsed = now - self._stuck_escape_phase_start_s
        phase = self._stuck_escape_phase
        if phase == "stop":
            self._drive(0.0, 0.0, 0.0)
            if elapsed >= self.stuck_escape_stop_sec:
                self._stuck_escape_phase = "pulse"
                self._stuck_escape_phase_start_s = now
            return True
        if phase == "pulse":
            self._drive(*self._stuck_escape_cmd)
            if elapsed >= self.stuck_escape_pulse_sec:
                self._stuck_escape_phase = "settle"
                self._stuck_escape_phase_start_s = now
            return True
        if phase == "settle":
            self._drive(0.0, 0.0, 0.0)
            if elapsed >= self.stuck_escape_settle_sec:
                self._clear_active_route_for_replan()
                self._reset_stuck_escape()
            return True

        self._reset_stuck_escape()
        return False

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

    def _local_anchor_runtime_safety_reason(self, now: float) -> str | None:
        """Return why the embedded sweep cannot safely keep commanding motion."""
        if now - self._imu_last_s > self.local_anchor_input_timeout_sec:
            return "IMU stream is stale"
        relative_s = self._wide_relative_last_frame_s
        if (
            relative_s is None
            or now - relative_s > self.local_anchor_input_timeout_sec
        ):
            return "wide relative-object stream is stale"
        if self.local_anchor_config.enable_align and self._body_H is None:
            return "body homography is unavailable"
        return None

    def _local_anchor_pick_subscriber_ready(self) -> bool:
        """Require the real arm consumer before accounting for a live local pick."""
        if self.dry_pick or not self.local_anchor_config.enable_pick:
            return True
        return bool(self.get_subscriptions_info_by_topic("/arm/pick_trigger"))

    def _local_anchor_pose_refresh_status(self) -> tuple[bool, str]:
        """Confirm that post-local localization has propagated into WorldModel."""
        if not self._local_anchor_pose_refresh_required:
            return True, "not required"
        pose_frames = self._localization_pose_seq - self._local_anchor_pose_baseline_seq
        if pose_frames < self.local_anchor_pose_confirm_frames:
            return False, (
                f"localization pose frames {pose_frames}/"
                f"{self.local_anchor_pose_confirm_frames}"
            )
        world_frames = self._world_seq - self._local_anchor_world_baseline_seq
        if world_frames < self.local_anchor_world_confirm_frames:
            return False, (
                f"world frames {world_frames}/"
                f"{self.local_anchor_world_confirm_frames}"
            )
        now = self._now_s()
        if now - self._localization_pose_last_s > self.local_anchor_pose_max_age_sec:
            return False, "localization pose is stale"
        if now - self._world_last_rx_s > self.local_anchor_pose_max_age_sec:
            return False, "world model is stale"
        if self._localization_pose is None or self.world is None:
            return False, "pose data is unavailable"
        px, py, pth = self._localization_pose
        xy_error = math.hypot(
            float(self.world.robot_x) - px,
            float(self.world.robot_y) - py,
        )
        theta_error = abs(
            self._wrap_pi(float(self.world.robot_theta) - pth)
        )
        if xy_error > self.local_anchor_pose_world_xy_tolerance_m:
            return False, f"world/pose xy mismatch {xy_error:.3f}m"
        if theta_error > self.local_anchor_pose_world_theta_tolerance_rad:
            return False, (
                f"world/pose heading mismatch {math.degrees(theta_error):.1f}deg"
            )
        return True, (
            f"pose confirmed from {pose_frames} localization and "
            f"{world_frames} world frames"
        )

    def _local_anchor_due(self) -> bool:
        """Return whether the active zone still owns its one local inspection run."""
        zone_id = self._active_zone_id()
        return bool(
            self.local_anchor_enabled
            and self.zone_mission_enabled
            and self.zone_anchor_nav_enabled
            and zone_id > 0
            and zone_id not in self._local_anchor_visited_zones
        )

    def _start_local_anchor_inspection(self) -> bool:
        """Start one embedded local-anchor run without changing global localization."""
        if not self._local_anchor_due() or self.world is None:
            return False
        now = self._now_s()
        zone_id = self._active_zone_id()
        # Mark at entry, not completion: a fault, manual transition, or phase revisit must never
        # execute a second local sweep in the same zone.
        self._local_anchor_visited_zones.add(zone_id)
        self._local_anchor_active_zone_id = zone_id
        self._local_anchor_relative_yaw = 0.0
        self._local_anchor_start_pose = (
            float(self.world.robot_x),
            float(self.world.robot_y),
            float(self.world.robot_theta),
        )
        self._local_anchor_pick_triggered = False
        self._local_anchor_pick_accounted = False
        self.current_target = None
        self._clear_current_slot()
        self._opportunistic_set2_active = False
        self._local_anchor_fsm = LocalAnchorFruitFsm(self.local_anchor_config)
        self._enter(LOCAL_ANCHOR_STATE)
        self._local_anchor_fsm.start(now, current_yaw=0.0)
        self._drive(0.0, 0.0, 0.0)
        self._decide(
            f"ZONE {zone_id} LOCAL FRUIT START target={self.set2_label or 'unset'}"
        )
        self.get_logger().info(
            f"zone {zone_id} local-anchor inspection started at pose="
            f"({self._local_anchor_start_pose[0]:.2f},"
            f"{self._local_anchor_start_pose[1]:.2f},"
            f"{math.degrees(self._local_anchor_start_pose[2]):+.1f}deg)"
        )
        return True

    def _feed_local_anchor_body_detections(self) -> None:
        """Adapt the cached Body detections to visual heading and local ALIGN inputs."""
        if self.state != LOCAL_ANCHOR_STATE:
            return
        now = self._now_s()
        fsm = self._local_anchor_fsm
        cfg = self.local_anchor_config

        if cfg.visual_heading_enabled:
            visual_best: tuple[tuple[float, float], float] | None = None
            visual_confidence = 0.0
            visual_turn_hint = 0.0
            for u, v, label, confidence in self._body_dets:
                if str(label).strip().lower() != "fruit_photo_cube":
                    continue
                confidence = float(confidence)
                if confidence < cfg.visual_heading_min_confidence:
                    continue
                score = (abs(float(u) - cfg.visual_heading_center_x_px), -confidence)
                if visual_best is None or score < visual_best[0]:
                    visual_best = (score, float(u))
                    visual_confidence = confidence
                    base = self._body_pixel_base(u, v)
                    if base is not None:
                        visual_turn_hint = math.atan2(base[1], max(0.01, base[0]))
            previous_state = fsm.state
            fsm.note_visual_fruit_center(
                None if visual_best is None else visual_best[1],
                visual_confidence,
                now,
                self._local_anchor_relative_yaw,
                visual_turn_hint,
            )
            if (
                previous_state in {"TURN_CONTINUOUS", "TURN_PULSE"}
                and fsm.state == "TURN_VERIFY"
            ):
                # A camera-centre lock is an asynchronous stop condition.  Do not wait for the
                # next 20 Hz mission tick while a turn pulse remains latched.
                self._drive(0.0, 0.0, 0.0)

        if not fsm.state.startswith("ALIGN"):
            return
        best: tuple[float, float] | None = None
        best_distance = 0.60
        for u, v, label, _confidence in self._body_dets:
            if str(label).strip().lower() not in self.local_anchor_body_align_labels:
                continue
            base = self._body_pixel_base(u, v)
            if base is None:
                continue
            distance = math.hypot(base[0] - cfg.grab_x_m, base[1] - cfg.grab_y_m)
            if distance < best_distance:
                best = base
                best_distance = distance
        if best is not None:
            fsm.note_align_target(best[0], best[1], now)

    def _feed_local_anchor_classification(self, msg: Classification) -> None:
        """Forward only a fresh SigLIP result belonging to the active local face."""
        fsm = self._local_anchor_fsm
        if self.state != LOCAL_ANCHOR_STATE or fsm.state != "CLASSIFY":
            return
        capture_s = self._time_msg_to_sec(msg.header.stamp)
        if capture_s > 0.0 and capture_s + 1e-6 < fsm.state_enter_s:
            return
        fsm.note_classification(
            str(msg.label),
            float(msg.confidence),
            bool(msg.image_face_visible),
            self._now_s(),
            is_target=bool(msg.is_target),
        )

    def _blacklist_local_anchor_pick_track(self) -> None:
        """Best-effort association of the removed local fruit to its field-map track."""
        if self.world is None or self._local_anchor_start_pose is None:
            return
        picked = next(
            (
                candidate
                for candidate in self._local_anchor_fsm.candidates
                if candidate.status == "PICKED"
            ),
            None,
        )
        if picked is None:
            return
        sx, sy, sth = self._local_anchor_start_pose
        c, s = math.cos(sth), math.sin(sth)
        field_x = sx + c * picked.x - s * picked.y
        field_y = sy + s * picked.x + c * picked.y
        matches = [
            obj
            for obj in self.world.objects
            if not bool(obj.blacklisted)
            and int(obj.set_type) == 2
            and math.hypot(float(obj.x) - field_x, float(obj.y) - field_y)
            <= self.local_anchor_picked_track_match_radius_m
        ]
        if not matches:
            return
        track = min(
            matches,
            key=lambda obj: math.hypot(float(obj.x) - field_x, float(obj.y) - field_y),
        )
        self._blacklist(int(track.id))
        self._decide(f"LOCAL FRUIT PICKED -> BLACKLIST #{int(track.id)}")

    def _finish_local_anchor_inspection(self) -> None:
        """Account for a local pick once, then rejoin the unchanged zone flow."""
        zone_id = self._local_anchor_active_zone_id or self._active_zone_id()
        terminal = self._local_anchor_fsm.state
        if (
            terminal == "COMPLETE"
            and self._local_anchor_pick_triggered
            and not self._local_anchor_pick_accounted
        ):
            self._local_anchor_pick_accounted = True
            self.tray_fruit += 1
            self._blacklist_local_anchor_pick_track()
            self._decide(f"ZONE {zone_id} LOCAL FRUIT PICK COMPLETE")

        pose_detail = "pose unavailable"
        if self.world is not None:
            pose_detail = (
                f"last_pose=({float(self.world.robot_x):.2f},"
                f"{float(self.world.robot_y):.2f},"
                f"{math.degrees(float(self.world.robot_theta)):+.1f}deg)"
            )
        summary = self._local_anchor_fsm.summary()
        self.get_logger().info(
            f"zone {zone_id} local-anchor {terminal.lower()}: "
            f"{summary['detail']}; {pose_detail} -> resume zone mission"
        )
        self._decide(f"ZONE {zone_id} LOCAL FRUIT {terminal} -> POSE STABILIZE")
        self._local_anchor_active_zone_id = None
        self._local_anchor_relative_yaw = 0.0
        self._local_anchor_start_pose = None
        self._local_anchor_pose_baseline_seq = self._localization_pose_seq
        self._local_anchor_world_baseline_seq = self._world_seq
        self._local_anchor_pose_refresh_required = True
        self._reset_zone_scan_timer()
        # Even when ordinary zone stabilization is disabled, the local sweep must wait for fresh
        # localization and WorldModel frames before replanning.  Never inject or reset the pose.
        self._enter("ZONE_STABILIZE")

    def _step_local_anchor_inspection(self) -> None:
        """Run one pure local FSM tick through the mission's sole command publishers."""
        now = self._now_s()
        fsm = self._local_anchor_fsm
        if (
            getattr(self, "pick_release_on_lift_enabled", False)
            and not self.dry_pick
            and fsm.state == "PICK_WAIT"
            and arm_pick_lift_started(
                self._arm_pick_phase,
                phase_rx_s=self._arm_pick_phase_rx_s,
                pick_enter_s=fsm.state_enter_s,
                now_s=now,
                max_age_sec=self.arm_pick_phase_max_age_sec,
            )
        ):
            fsm.complete_pick_on_lift(now)
        if (
            fsm.state not in {"PICK_TRIGGER", "PICK_WAIT"}
            and fsm.state not in LOCAL_ANCHOR_TERMINAL_STATES
        ):
            safety_reason = self._local_anchor_runtime_safety_reason(now)
            if safety_reason is not None:
                fsm.abort(now, safety_reason)
        if (
            self._time_in_state() >= self.local_anchor_run_timeout_sec
            and fsm.state not in {"PICK_TRIGGER", "PICK_WAIT"}
            and fsm.state not in LOCAL_ANCHOR_TERMINAL_STATES
        ):
            fsm.abort(now, f"local run timeout {self.local_anchor_run_timeout_sec:.1f}s")

        command = fsm.tick(now, self._local_anchor_relative_yaw)
        if command.request_pick and not self._local_anchor_pick_triggered:
            if not self._local_anchor_pick_subscriber_ready():
                fsm.abort(now, "pick sequencer subscriber is unavailable")
                self.get_logger().error(
                    "local-anchor pick aborted: /arm/pick_trigger has no subscriber"
                )
            else:
                self._local_anchor_pick_triggered = True
                if self.dry_pick:
                    self.get_logger().info(
                        "[DRY PICK] local-anchor Set2 trigger suppressed"
                    )
                elif self.local_anchor_config.enable_pick:
                    self._publish_pick(True)
                    self.get_logger().info(
                        "local-anchor /arm/pick_trigger sent once"
                    )
        self._drive(command.vx, command.vy, command.omega)
        if fsm.state in LOCAL_ANCHOR_TERMINAL_STATES:
            self._drive(0.0, 0.0, 0.0)
            self._finish_local_anchor_inspection()

    def _clear_zone_entry_anchor(self) -> None:
        self._zone_entry_anchor_idx = None
        self._zone_entry_anchor = None
        self._zone_anchor_slowdown_active = False
        self._opening_zone_entry_path = None
        self._opening_zone_entry_start = None
        self._opening_zone_entry_idx = 0
        self._reset_opening_anchor_strafe()

    def _zone_anchor_speed_limit(
        self, anchor: tuple[float, float]
    ) -> float | None:
        distance = self._distance_to(anchor[0], anchor[1])
        was_active = self._zone_anchor_slowdown_active
        active, speed_limit = _zone_anchor_slowdown_state(
            distance,
            self.zone_anchor_slowdown_enabled,
            self.zone_anchor_slowdown_distance_m,
            self.zone_anchor_slow_speed,
            was_active,
        )
        self._zone_anchor_slowdown_active = active
        if active and not was_active:
            self._decide(
                f"ZONE {self._active_zone_id()} ANCHOR SLOW "
                f"d={distance:.2f}m speed={speed_limit:.2f}"
            )
        return speed_limit

    def _reset_opening_anchor_strafe(self) -> None:
        self._opening_anchor_strafe_phase = "align"
        self._opening_anchor_strafe_phase_start_s = 0.0
        self._opening_anchor_strafe_pulses = 0
        self._opening_anchor_strafe_vy = 0.0
        self._opening_anchor_strafe_pulse_duration_sec = 0.0
        self._reset_pulsed_heading()

    def _step_zone_anchor_axis_strafe(
        self,
        goal_axis: float,
        target_heading: float,
        strafe_axis: str,
        route_label: str,
    ) -> bool:
        """Face the final travel direction, then pulse sideways onto its perpendicular axis."""
        if self.world is None:
            self._drive(0.0, 0.0, 0.0)
            return False
        current_axis = (
            float(self.world.robot_x) if strafe_axis == "x" else float(self.world.robot_y)
        )
        axis_error = float(goal_axis) - current_axis
        if abs(axis_error) <= self.lane_heading_axis_tolerance_m:
            self._drive(0.0, 0.0, 0.0)
            self._reset_opening_anchor_strafe()
            return True

        heading_error = self._wrap_pi(target_heading - float(self.world.robot_theta))
        phase = self._opening_anchor_strafe_phase
        now = self._now_s()

        if (
            phase not in {"align", "heading_settle"}
            and abs(heading_error) > self.lane_heading_align_tolerance_rad
        ):
            self._opening_anchor_strafe_phase = "align"
            self._opening_anchor_strafe_phase_start_s = 0.0
            self._reset_pulsed_heading()
            phase = "align"

        if phase == "align":
            if self._opening_anchor_strafe_phase_start_s <= 0.0:
                self._opening_anchor_strafe_phase_start_s = now
                self._decide(
                    f"{route_label} STRAFE-{strafe_axis.upper()} HEADING "
                    f"{math.degrees(target_heading):+.0f}deg"
                )
            aligned = self._turn_in_place_pulsed(
                target_heading,
                self.lane_heading_align_tolerance_rad,
                ("zone_anchor_strafe", self._opening_zone_entry_generation),
                self.lane_heading_omega_max,
            )
            if aligned:
                self._opening_anchor_strafe_phase = "heading_settle"
                self._opening_anchor_strafe_phase_start_s = now
            return False

        if phase == "heading_settle":
            self._drive(0.0, 0.0, 0.0)
            if now - self._opening_anchor_strafe_phase_start_s >= self.lane_heading_settle_sec:
                self._opening_anchor_strafe_phase = "ready"
                self._opening_anchor_strafe_phase_start_s = now
            return False

        if phase == "pulse":
            if (
                now - self._opening_anchor_strafe_phase_start_s
                < self._opening_anchor_strafe_pulse_duration_sec
            ):
                self._drive(0.0, self._opening_anchor_strafe_vy, 0.0)
                return False
            self._drive(0.0, 0.0, 0.0)
            self._opening_anchor_strafe_phase = "pulse_settle"
            self._opening_anchor_strafe_phase_start_s = now
            return False

        if phase == "pulse_settle":
            self._drive(0.0, 0.0, 0.0)
            if (
                now - self._opening_anchor_strafe_phase_start_s
                >= self.opening_anchor_strafe_settle_sec
            ):
                self._opening_anchor_strafe_phase = "align"
                self._opening_anchor_strafe_phase_start_s = 0.0
                self._reset_pulsed_heading()
            return False

        if self._opening_anchor_strafe_pulses >= self.opening_anchor_strafe_max_pulses:
            self._drive(0.0, 0.0, 0.0)
            self.get_logger().error(
                f"{route_label} {strafe_axis}-strafe reached its pulse limit; holding stopped",
                throttle_duration_sec=2.0,
            )
            return False

        vy = _zone_anchor_strafe_velocity(
            axis_error,
            target_heading,
            strafe_axis,
            self.opening_anchor_strafe_duty,
        )
        pulse_duration_sec = _zone_anchor_strafe_pulse_duration(
            axis_error,
            self.opening_anchor_strafe_long_threshold_m,
            self.opening_anchor_strafe_medium_threshold_m,
            self.opening_anchor_strafe_long_pulse_sec,
            self.opening_anchor_strafe_medium_pulse_sec,
            self.opening_anchor_strafe_short_pulse_sec,
        )
        self._opening_anchor_strafe_pulses += 1
        self._opening_anchor_strafe_vy = vy
        self._opening_anchor_strafe_pulse_duration_sec = pulse_duration_sec
        self._opening_anchor_strafe_phase = "pulse"
        self._opening_anchor_strafe_phase_start_s = now
        self._decide(
            f"{route_label} STRAFE-{strafe_axis.upper()} PULSE "
            f"{self._opening_anchor_strafe_pulses}/"
            f"{self.opening_anchor_strafe_max_pulses} error={axis_error:+.2f}m "
            f"duration={pulse_duration_sec:.2f}s vy={vy:+.3f}"
        )
        self._drive(0.0, vy, 0.0)
        return False

    def _step_axis_strafe_zone_entry(
        self,
        anchor: tuple[float, float],
        strafe_axis: str,
        target_heading: float,
        route_label: str,
    ) -> bool:
        """Align, strafe onto the anchor's cross-axis coordinate, then drive forward."""
        robot_xy = self._robot_xy()
        if robot_xy is None:
            self._drive(0.0, 0.0, 0.0)
            return False
        if self._opening_zone_entry_path is None:
            self._opening_zone_entry_start = robot_xy
            self._opening_zone_entry_path = _axis_strafe_zone_entry_waypoints(
                robot_xy,
                anchor,
                strafe_axis,
                self.lane_heading_axis_tolerance_m,
            )
            self._opening_zone_entry_idx = 0
            self._opening_zone_entry_generation += 1
            route = " -> ".join(
                f"({x:.2f},{y:.2f})" for x, y in self._opening_zone_entry_path
            )
            self._reset_opening_anchor_strafe()
            forward_axis = "Y" if strafe_axis == "x" else "X"
            self._decide(
                f"{route_label} STRAFE-{strafe_axis.upper()}-THEN-FORWARD-"
                f"{forward_axis} ROUTE {route}"
            )

        path = self._opening_zone_entry_path
        if not path or self._opening_zone_entry_start is None:
            self._drive(0.0, 0.0, 0.0)
            return True
        idx = self._opening_zone_entry_idx
        if idx >= len(path):
            self._drive(0.0, 0.0, 0.0)
            return True
        start = self._opening_zone_entry_start if idx == 0 else path[idx - 1]
        goal = path[idx]
        if strafe_axis == "x":
            axis_strafe_leg = (
                idx == 0
                and len(path) > 1
                and abs(goal[0] - start[0]) > self.lane_heading_axis_tolerance_m
                and abs(goal[1] - start[1]) <= self.lane_heading_axis_tolerance_m
            )
            goal_axis = goal[0]
        else:
            axis_strafe_leg = (
                idx == 0
                and len(path) > 1
                and abs(goal[1] - start[1]) > self.lane_heading_axis_tolerance_m
                and abs(goal[0] - start[0]) <= self.lane_heading_axis_tolerance_m
            )
            goal_axis = goal[1]
        if axis_strafe_leg:
            if self._step_zone_anchor_axis_strafe(
                goal_axis,
                target_heading,
                strafe_axis,
                route_label,
            ):
                self._decide(
                    f"{route_label} STRAFE-{strafe_axis.upper()} REACHED "
                    f"{strafe_axis}={goal_axis:+.2f}"
                )
                self._opening_zone_entry_idx += 1
                self._lane_heading_segment_key = None
                self._lane_heading_phase = "align"
            return False
        tolerance = (
            self.zone_center_reach_tol_m
            if idx == len(path) - 1
            else self.lane_heading_axis_tolerance_m
        )
        reached, passed = _cardinal_goal_reached_or_passed(start, robot_xy, goal, tolerance)
        if reached:
            self._drive(0.0, 0.0, 0.0)
            label = "PASSED" if passed else "REACHED"
            self._decide(f"{route_label} LEG {idx + 1}/{len(path)} {label}")
            self._opening_zone_entry_idx += 1
            self._lane_heading_segment_key = None
            self._lane_heading_phase = "align"
            return self._opening_zone_entry_idx >= len(path)

        segment_key = (-self._opening_zone_entry_generation, idx)
        speed_limit = (
            self._zone_anchor_speed_limit(anchor)
            if idx == len(path) - 1
            else None
        )
        handled = self._drive_cardinal_lane_segment(
            start,
            goal,
            segment_key,
            stop_radius_m=0.0,
            speed_limit_mps=speed_limit,
        )
        if not handled:
            self._drive(0.0, 0.0, 0.0)
            self.get_logger().error(
                f"{route_label} entry produced a non-cardinal forward segment",
                throttle_duration_sec=2.0,
            )
        return False

    def _step_opening_zone_entry(self, anchor: tuple[float, float]) -> bool:
        """Strafe onto Z1 anchor x, then drive forward along field y."""
        robot_xy = self._robot_xy()
        target_heading = -math.pi / 2.0
        if robot_xy is not None and float(anchor[1]) >= float(robot_xy[1]):
            target_heading = math.pi / 2.0
        return self._step_axis_strafe_zone_entry(
            anchor,
            "x",
            target_heading,
            "OPENING Z1",
        )

    def _step_zone_transition_entry(self, anchor: tuple[float, float]) -> bool | None:
        """Run the configured strafe-then-forward entry between consecutive zones."""
        if self._zone_idx <= 0 or self._zone_idx >= len(self.zone_order):
            return None
        from_zone = int(self.zone_order[self._zone_idx - 1])
        to_zone = int(self.zone_order[self._zone_idx])
        profile = _zone_transition_strafe_profile(from_zone, to_zone)
        if profile is None:
            return None
        strafe_axis, target_heading = profile
        return self._step_axis_strafe_zone_entry(
            anchor,
            strafe_axis,
            target_heading,
            f"ZONE {from_zone}->{to_zone}",
        )

    def _zone_entry_anchor_point(self) -> tuple[float, float] | None:
        """Latch the closest candidate once when the active zone is entered."""
        if self._zone_entry_anchor_idx == self._zone_idx and self._zone_entry_anchor is not None:
            return self._zone_entry_anchor
        zone_id = self._active_zone_id()
        candidates = self.zone_anchor_candidates.get(zone_id, [])
        if not candidates:
            xmin, xmax, ymin, ymax = self.zone_bounds.get(zone_id, (-2.0, 2.0, -2.0, 2.0))
            candidates = [((xmin + xmax) * 0.5, (ymin + ymax) * 0.5)]
        robot_xy = self._robot_xy()
        if robot_xy is None:
            return None
        candidate_idx, point = _nearest_zone_anchor(candidates, robot_xy)
        self._zone_entry_anchor_idx = self._zone_idx
        self._zone_entry_anchor = point
        self._decide(
            f"ZONE {zone_id} ENTRY ANCHOR A{candidate_idx + 1}/{len(candidates)} "
            f"({point[0]:.2f},{point[1]:.2f})"
        )
        return point

    def _object_in_active_zone(self, obj: Object) -> bool:
        if not self.zone_mission_enabled:
            return True
        xmin, xmax, ymin, ymax = self.zone_bounds.get(self._active_zone_id(), (-2.0, 2.0, -2.0, 2.0))
        return xmin <= float(obj.x) <= xmax and ymin <= float(obj.y) <= ymax

    @staticmethod
    def _object_last_seen_key(obj: Object) -> tuple[int, int] | None:
        stamp = getattr(obj, "last_seen", None)
        if stamp is None:
            return None
        return int(stamp.sec), int(stamp.nanosec)

    def _reset_plain_cube_reobservation(self) -> None:
        self._plain_cube_unknown_count = 0
        self._plain_cube_unknown_last_stamp = None

    def _plain_cube_approach_identity_action(self, target: Object | None) -> str:
        """Return continue, hold, or exclude for a latched Set1 plain-cube target."""
        active = (
            self.set1_label == "cube"
            and int(self.phase) == 1
            and not self._opportunistic_set2_active
        )
        if not active or target is None:
            return "continue"
        if int(target.set_type) == 1 and str(target.class_label) == "cube":
            self._reset_plain_cube_reobservation()
            return "continue"
        if int(target.set_type) != 0:
            return "exclude"

        stamp_key = self._object_last_seen_key(target)
        if stamp_key is not None and stamp_key != self._plain_cube_unknown_last_stamp:
            self._plain_cube_unknown_last_stamp = stamp_key
            self._plain_cube_unknown_count += 1
        if self._plain_cube_unknown_count >= self.plain_cube_ambiguous_observations:
            return "exclude"
        return "hold"

    def _nearest_object_at(
        self, point: tuple[float, float] | None, radius_m: float
    ) -> Object | None:
        if self.world is None or point is None:
            return None
        candidates = [
            obj for obj in self.world.objects
            if not bool(obj.blacklisted)
            and math.hypot(float(obj.x) - point[0], float(obj.y) - point[1]) <= radius_m
        ]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda obj: math.hypot(float(obj.x) - point[0], float(obj.y) - point[1]),
        )

    def _is_zone_ambiguous_excluded(self, obj: Object) -> bool:
        if not self.zone_mission_enabled:
            return False
        if int(obj.id) in self._zone_ambiguous_excluded_ids:
            return True
        return any(
            math.hypot(float(obj.x) - x, float(obj.y) - y)
            <= self.zone_ambiguous_exclusion_radius_m
            for x, y in self._zone_ambiguous_excluded_xy
        )

    def _exclude_ambiguous_for_active_zone(
        self,
        obj: Object,
        fallback_xy: tuple[float, float] | None,
        reason: str,
    ) -> None:
        obj_id = int(obj.id)
        point = fallback_xy or (float(obj.x), float(obj.y))
        if self.zone_mission_enabled:
            if obj_id > 0:
                self._zone_ambiguous_excluded_ids.add(obj_id)
            if not any(
                math.hypot(point[0] - x, point[1] - y)
                <= self.zone_ambiguous_exclusion_radius_m * 0.5
                for x, y in self._zone_ambiguous_excluded_xy
            ):
                self._zone_ambiguous_excluded_xy.append(point)
        self._decide(
            f"ZONE {self._active_zone_id()} AMBIGUOUS EXCLUDE #{obj_id} "
            f"({point[0]:.2f},{point[1]:.2f}) {reason}"
        )

    def _clear_zone_ambiguous_exclusions(self) -> None:
        self._zone_ambiguous_excluded_ids.clear()
        self._zone_ambiguous_excluded_xy.clear()
        self._reset_plain_cube_reobservation()

    def _reset_zone_scan_timer(self) -> None:
        self._zone_no_target_since = None

    def _next_travel_route_mode(self, default: str = "grid_only") -> str:
        """Use the direct first-lane connector for exactly one post-pick travel plan."""
        if (
            getattr(self, "_plan", None) is not None
            and getattr(self, "_plan_route_mode", default) == "post_pick_entry"
        ):
            return "post_pick_entry"
        if getattr(self, "_post_pick_lane_entry_pending", False):
            return "post_pick_entry"
        return default

    def _global_exploration_route_mode(self) -> str:
        """Prefer a collision-checked direct leg for global targets and unseen viewpoints."""
        if (
            getattr(self, "global_target_mode", False)
            and getattr(self, "global_fast_safe_routes_enabled", False)
        ):
            return "legacy"
        return self._next_travel_route_mode()

    def _plan_with_flexible_fallback(
        self,
        start: tuple[float, float],
        dest: tuple[float, float],
        obstacles: list[tuple[float, float]],
        route_mode: str,
    ) -> tuple[list[tuple[float, float]] | None, bool]:
        """Retry a failed strict lane plan with collision-checked flexible connectors."""
        vias = self._planner.plan(start, dest, obstacles, route_mode=route_mode)
        flexible = route_mode == "legacy"
        if vias or not self.direct_fallback_enabled or flexible:
            return vias, flexible
        vias = self._planner.plan(start, dest, obstacles, route_mode="legacy")
        return vias, bool(vias)

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
            self._clear_zone_ambiguous_exclusions()
            self._plan = None
            self._coverage = None
            self.current_target = None
            self._clear_current_slot()
            self._opportunistic_set2_active = False
            self._zone_stabilized_idx = None
            self._zone_anchor_reached_idx = None
            self._clear_zone_entry_anchor()
            self._zone_stabilized_after_s = 0.0
            self._reset_zone_scan_timer()
            self.get_logger().info(f"zone {prev} done -> zone {self._active_zone_id()}")
            self._decide(f"ZONE {prev}->{self._active_zone_id()}")
            self._enter("SCAN")
            return
        self._zone_idx = 0
        self._clear_zone_ambiguous_exclusions()
        self._clear_current_slot()
        self._opportunistic_set2_active = False
        self._zone_stabilized_idx = None
        self._zone_anchor_reached_idx = None
        self._clear_zone_entry_anchor()
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

    def _drive_toward(
        self,
        dest_x: float,
        dest_y: float,
        yaw: float = 0.0,
        exclude_id: int = 0,
        route_mode: str = "grid_only",
        speed_limit_mps: float | None = None,
        speed_override_mps: float | None = None,
        exclude_dest_obstacles: bool = True,
        holonomic_translation: bool = False,
    ) -> bool:
        """Drive the active lane route and report when its final waypoint is reached."""
        self._last_drive_route_blocked = False
        dest = (float(dest_x), float(dest_y))
        obstacles = self._obstacles_snapshot(
            exclude_id, dest if exclude_dest_obstacles else None
        )
        self._publish_planning_obstacles(obstacles)
        if not self.planner_enabled:
            if holonomic_translation:
                self._drive_toward_holonomic(
                    dest_x,
                    dest_y,
                    speed_limit_mps=speed_limit_mps,
                    speed_override_mps=speed_override_mps,
                )
            else:
                self._drive_toward_direct(
                    dest_x,
                    dest_y,
                    yaw,
                    speed_limit_mps=speed_limit_mps,
                    speed_override_mps=speed_override_mps,
                )
            distance = self._distance_to(dest_x, dest_y)
            return distance is not None and distance <= self.wp_reach_tol_m
        rxy = self._robot_xy()
        if rxy is None:
            self._drive(0.0, 0.0, 0.0)
            return False
        now = self._now_s()
        drifted = (self._plan_dest is None
                   or math.hypot(dest[0] - self._plan_dest[0], dest[1] - self._plan_dest[1]) > self.replan_goal_move_m)
        route_mode_changed = route_mode != self._plan_route_mode
        stale = (
            self.periodic_replan_enabled
            and now - self._plan_stamp > self.replan_period_sec
        )
        # A failed attempt is stored as []. Retry it after the normal throttle instead of
        # mistaking it for a valid latched route and holding forever.
        if lane_route_replan_required(
            self._plan,
            route_mode_changed,
            drifted,
            stale,
            now,
            self._last_replan_t,
            self.replan_throttle_sec,
        ):
            self._last_replan_t = now
            vias, flexible_fallback = self._plan_with_flexible_fallback(
                rxy, dest, obstacles, route_mode
            )
            self._plan_escape = False
            self._plan_flexible_fallback = False
            if vias:
                self._plan = vias
                self._plan_flexible_fallback = flexible_fallback
                if route_mode == "post_pick_entry":
                    self._post_pick_lane_entry_pending = False
            else:
                escape = self._front_escape_waypoint(obstacles)
                if escape is not None:
                    self._plan = [(escape[0], escape[1])]
                    self._plan_escape = True
                else:
                    self._plan = []
            self._plan_idx = 0
            self._plan_start_xy = rxy
            self._plan_route_mode = route_mode
            self._plan_generation += 1
            self._lane_heading_segment_key = None
            self._lane_heading_phase = "align"
            self._lane_heading_filtered = None
            self._lane_heading_violation_start_s = None
            self._lane_heading_realign_armed_at_s = 0.0
            self._lane_heading_settle_start_s = 0.0
            self._lane_heading_entry_start_s = 0.0
            self._lane_heading_turn_sign = 0
            self._lane_heading_reverse_start_s = 0.0
            self._reset_object_connector_brake()
            self._plan_dest = dest
            self._plan_stamp = now
            if self._plan:
                route = " -> ".join(f"({x:.2f},{y:.2f})" for x, y in self._plan)
                if self._plan_flexible_fallback:
                    if route_mode == "legacy":
                        self._decide(f"FAST SAFE ROUTE {route}")
                        self.get_logger().info(
                            "using collision-checked direct/flexible route for global travel"
                        )
                    else:
                        self._decide(f"LANE ROUTE FLEXIBLE FALLBACK {route}")
                        self.get_logger().warn(
                            "strict lane route unavailable; using collision-checked flexible route",
                            throttle_duration_sec=3.0,
                        )
                elif route_mode == "post_pick_entry":
                    first_x, first_y = self._plan[0]
                    self._decide(f"POST-PICK LANE ENTRY ({first_x:.2f},{first_y:.2f})")
                if not self._plan_flexible_fallback:
                    self._decide(f"LANE ROUTE {route}")
                self.get_logger().info(f"latched lane route: {route}")
            else:
                self._decide("SAFE ROUTE BLOCKED -> HOLD AND REPLAN")
                self.get_logger().warn(
                    "strict and flexible routes unavailable; holding for safe replan",
                    throttle_duration_sec=3.0,
                )
        if not self._plan:                            # safety: never index a None/empty plan
            if self._plan_dest == dest and not getattr(self, "_plan_escape", False):
                self._last_drive_route_blocked = True
            self._drive(0.0, 0.0, 0.0)
            return False
        # advance monotonically past intermediate vias already reached
        while self._plan_idx < len(self._plan) - 1:
            wx, wy = self._plan[self._plan_idx]
            d = self._distance_to(wx, wy)
            segment_start = (
                self._plan_start_xy
                if self._plan_idx == 0
                else self._plan[self._plan_idx - 1]
            )
            reached = d is not None and d <= self.wp_reach_tol_m
            passed = False
            lateral_error = 0.0
            if (
                rxy is not None
                and segment_start is not None
                and cardinal_segment_heading(
                    segment_start, (wx, wy), self.lane_heading_axis_tolerance_m
                )
                is not None
            ):
                reached, passed, lateral_error = _cardinal_waypoint_status(
                    segment_start,
                    rxy,
                    (wx, wy),
                    self.wp_reach_tol_m,
                    self.lane_pass_lateral_tol_m,
                )
            if passed and not reached:
                self._drive(0.0, 0.0, 0.0)
                self._decide(
                    f"LANE WAYPOINT MISSED lateral={lateral_error:.2f}m -> REPLAN"
                )
                self._plan = None
                self._plan_start_xy = None
                self._lane_heading_segment_key = None
                self._reset_object_connector_brake()
                return False
            brake_key = (self._plan_generation, self._plan_idx)
            connector_braking = self._object_connector_brake_key == brake_key
            if reached or connector_braking:
                if (
                    self._object_connector_brake_required(self._plan_idx)
                    and self._step_object_connector_brake(self._plan_idx)
                ):
                    return False
                if passed:
                    self._decide(
                        f"LANE WAYPOINT PASSED lateral={lateral_error:.2f}m"
                    )
                self._plan_idx += 1
                if passed:
                    self._drive(0.0, 0.0, 0.0)
                    return False
            else:
                break
        wx, wy = self._plan[self._plan_idx]
        if self._plan_escape:
            d_escape = self._distance_to(wx, wy)
            if d_escape is not None and d_escape < self.wp_reach_tol_m:
                self._plan = None
                self._plan_escape = False
                self._drive(0.0, 0.0, 0.0)
                return False
        last = self._plan_idx == len(self._plan) - 1
        # Intermediate vias keep the current heading target; the final via carries the requested yaw.
        via_yaw = yaw if last else (self.world.robot_theta if self.world is not None else yaw)
        segment_start = (
            self._plan_start_xy
            if self._plan_idx == 0
            else self._plan[self._plan_idx - 1]
        )
        segment_key = (self._plan_generation, self._plan_idx)
        post_pick_plan = self._plan_route_mode == "post_pick_entry"
        final_distance = self._distance_to(wx, wy)
        final_reached = (
            final_distance is not None and final_distance <= self.wp_reach_tol_m
        )
        final_passed = False
        final_lateral_error = 0.0
        if (
            segment_start is not None
            and cardinal_segment_heading(
                segment_start, (wx, wy), self.lane_heading_axis_tolerance_m
            )
            is not None
        ):
            final_reached, final_passed, final_lateral_error = (
                _cardinal_waypoint_status(
                    segment_start,
                    rxy,
                    (wx, wy),
                    self.wp_reach_tol_m,
                    self.lane_pass_lateral_tol_m,
                )
            )
        if final_passed and not final_reached:
            self._drive(0.0, 0.0, 0.0)
            self._decide(
                f"LANE FINAL MISSED lateral={final_lateral_error:.2f}m -> REPLAN"
            )
            self._plan = None
            self._plan_start_xy = None
            self._lane_heading_segment_key = None
            self._reset_object_connector_brake()
            return False
        if final_reached:
            self._drive(0.0, 0.0, 0.0)
            if final_passed:
                self._decide(
                    f"LANE FINAL PASSED lateral={final_lateral_error:.2f}m"
                )
            return True
        if holonomic_translation:
            self._lane_heading_segment_key = None
            self._lane_heading_phase = "align"
            self._drive_toward_holonomic(
                wx,
                wy,
                speed_limit_mps=speed_limit_mps,
                speed_override_mps=speed_override_mps,
            )
            return False
        cardinal_handled = segment_start is not None and self._drive_cardinal_lane_segment(
            segment_start,
            (wx, wy),
            segment_key,
            force_initial_align=post_pick_plan,
            forward_only=post_pick_plan,
            speed_limit_mps=speed_limit_mps,
            speed_override_mps=speed_override_mps,
        )
        if not cardinal_handled:
            self._lane_heading_segment_key = None
            self._lane_heading_phase = "align"
            self._lane_heading_filtered = None
            self._lane_heading_violation_start_s = None
            self._lane_heading_realign_armed_at_s = 0.0
            self._lane_heading_settle_start_s = 0.0
            self._lane_heading_entry_start_s = 0.0
            self._lane_heading_turn_sign = 0
            self._lane_heading_reverse_start_s = 0.0
            if self._plan_flexible_fallback:
                self._drive_toward_direct(
                    wx,
                    wy,
                    via_yaw,
                    speed_limit_mps=speed_limit_mps,
                    speed_override_mps=speed_override_mps,
                )
            elif self._plan_route_mode == "post_pick_entry" and self._plan_idx == 0:
                self._drive_to_post_pick_lane_entry(wx, wy)
            elif self._plan_route_mode == "object_approach" and last:
                self._drive_toward_direct(
                    wx,
                    wy,
                    via_yaw,
                    speed_limit_mps=speed_limit_mps,
                    speed_override_mps=speed_override_mps,
                )
            else:
                self._drive(0.0, 0.0, 0.0)
                self.get_logger().error(
                    "non-cardinal lane segment blocked in grid-only route",
                    throttle_duration_sec=2.0,
                )
        return False

    def _publish_pick(self, trigger: bool) -> None:
        self.pub_pick.publish(Bool(data=trigger))

    def _blacklist(self, obj_id: int) -> None:
        if obj_id != 0:
            if not hasattr(self, "_local_blacklisted_ids"):
                self._local_blacklisted_ids = set()
            self._local_blacklisted_ids.add(int(obj_id))
            self.pub_blacklist.publish(UInt64(data=int(obj_id)))
            latch = getattr(self, "_set2_identity_latch", None)
            if latch is not None and int(latch.target_id) == int(obj_id):
                self._set2_identity_latch = None

    def _decide(self, text: str) -> None:
        """Publish a human-readable decision for the visualiser's decision feed."""
        self.pub_decision.publish(String(data=text))

    def _set_world_mapping_enabled(self, enabled: bool, reason: str = "") -> None:
        enabled = bool(enabled)
        if enabled == self._world_mapping_enabled:
            return
        self._world_mapping_enabled = enabled
        if enabled:
            self._global_mapping_enabled_since_s = self._now_s()
            self._global_seen_wide_seq = self._wide_relative_seq
        else:
            self._global_mapping_enabled_since_s = -math.inf
            self._global_patrol_goal = None
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
        self._world_seq += 1
        self._world_last_rx_s = self._now_s()
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

    def on_localization_pose(self, msg: PoseStamped) -> None:
        """Track direct pose freshness independently from WorldModel publication."""
        now = self._now_s()
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._localization_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            math.atan2(siny_cosp, cosy_cosp),
        )
        self._localization_pose_seq += 1
        self._localization_pose_last_s = now
        capture_s = self._time_msg_to_sec(msg.header.stamp)
        pose_s = capture_s if capture_s > 0.0 else now
        self._localization_pose_history.append((pose_s, *self._localization_pose))
        del self._localization_pose_history[:-400]

    def on_imu(self, _msg: Imu) -> None:
        """Track raw IMU freshness even while stationary yaw deltas are suppressed."""
        self._imu_last_s = self._now_s()

    def on_imu_yaw_delta(self, msg: Float32) -> None:
        """Propagate relative geometry from physical gyro motion, excluding map correction."""
        dtheta = float(msg.data)
        if self.state == LOCAL_ANCHOR_STATE and math.isfinite(dtheta):
            self._local_anchor_relative_yaw = integrate_imu_yaw(
                self._local_anchor_relative_yaw,
                dtheta,
            )
        if not self._relative_mode() or not self._relative_anchor_tracker.locked:
            return
        if not math.isfinite(dtheta):
            return
        self._relative_anchor_tracker.predict_robot_motion(
            dx_body=0.0,
            dy_body=0.0,
            dtheta=dtheta,
            now_s=self._now_s(),
        )

    def on_wall_heading_debug(self, msg: Float32MultiArray) -> None:
        """Cache [raw, applied, count, variance] for post-opening alignment validation."""
        if len(msg.data) < 5:
            return
        raw = float(msg.data[0])
        applied = float(msg.data[2])
        count = float(msg.data[3])
        variance = float(msg.data[4])
        if not all(math.isfinite(value) for value in (raw, applied, count, variance)):
            return
        self._opening_heading_debug = (raw, applied, count, variance)
        self._opening_heading_debug_s = self._now_s()
        self._opening_heading_debug_seq += 1

    def on_wall_field_correction(self, msg: Float32MultiArray) -> None:
        """Cache the current two-axis wall residual used to establish opening x/y."""
        if len(msg.data) < 4:
            return
        dx, dy, confidence = float(msg.data[0]), float(msg.data[1]), float(msg.data[3])
        if not all(math.isfinite(value) for value in (dx, dy, confidence)):
            return
        self._opening_position_debug = (dx, dy, confidence)
        self._opening_position_debug_s = self._now_s()
        self._opening_position_debug_seq += 1

    def on_relative_wide(self, msg: WorldModel) -> None:
        """Cache raw base-link Wide points for ALIGN and feed the optional anchor lattice."""
        now = self._now_s()
        if str(msg.header.frame_id) != "base_link":
            self.get_logger().warn(
                f"relative wide frame must be base_link, got '{msg.header.frame_id}'",
                throttle_duration_sec=5.0,
            )
            return
        capture_s = self._time_msg_to_sec(msg.header.stamp)
        capture_age = now - capture_s if capture_s > 0.0 else 0.0
        if capture_age > self.relative_anchor_max_capture_age_sec or capture_age < -0.25:
            self.get_logger().warn(
                f"dropping delayed relative wide frame age={capture_age:.2f}s",
                throttle_duration_sec=2.0,
            )
            return
        observations = [
            RelativeObservation(
                x=float(obj.x),
                y=float(obj.y),
                label=str(obj.class_label),
                set_type=int(obj.set_type),
                confidence=float(obj.confidence),
            )
            for obj in msg.objects
            if not bool(obj.blacklisted) and int(obj.set_type) in {1, 2}
        ]
        self._wide_relative_observations = [
            (float(observation.x), float(observation.y)) for observation in observations
        ]
        self._wide_relative_labeled_observations = list(observations)
        self._wide_relative_last_frame_s = now
        self._wide_relative_seq += 1

        if self.state == LOCAL_ANCHOR_STATE:
            self._local_anchor_fsm.add_observations(
                _local_anchor_observations(
                    observations,
                    self._local_anchor_relative_yaw,
                    self.local_anchor_candidate_labels,
                )
            )

        if not self._relative_mode() or self._relative_acquisition_started_s is None:
            return
        self._sync_relative_landmark_exclusions()
        self._relative_anchor_tracker.observe(observations, now_s=now)
        if self._relative_acquisition_first_frame_s is None:
            self._relative_acquisition_first_frame_s = now
            self.get_logger().info(
                "first post-opening wide frame -> start "
                f"{self.relative_anchor_acquire_sec:.1f}s slot scan"
            )
        self._relative_last_frame_s = now
        if observations:
            self._relative_last_observation_s = now

    def on_wall_map_transform(self, msg: Float32MultiArray) -> None:
        if not self.set2_slot_enabled or len(msg.data) < 3:
            return
        self.slot_inventory.apply_transform(
            float(msg.data[0]), float(msg.data[1]), float(msg.data[2]),
            include_grid_locked=self.set2_slot_follow_map_transform,
        )

    def on_wall_segments(self, msg: Float32MultiArray) -> None:
        """Cache raw wall segments in field coordinates for the timed parking route."""
        values = [float(value) for value in msg.data]
        segments: list[tuple[float, float, float, float]] = []
        for idx in range(0, len(values) - 3, 4):
            segment = tuple(values[idx:idx + 4])
            if all(math.isfinite(value) for value in segment):
                segments.append(segment)
        self._wall_segments = segments
        self._wall_segments_s = self._now_s()
        self._wall_segments_seq += 1

    def on_target(self, msg: Object) -> None:
        self.selected = msg

    def on_siglip(self, msg: Classification) -> None:
        self.siglip = msg
        self.siglip_rx_s = self._now_s()
        capture_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        capture_s = self._time_msg_to_sec(msg.header.stamp)
        self.siglip_stamp_s = capture_s if capture_s > 0.0 else self.siglip_rx_s
        self._siglip_body_dets = []
        if capture_key != (0, 0):
            for key, detections in reversed(self._body_dets_history):
                if key == capture_key:
                    self._siglip_body_dets = list(detections)
                    break
        self._maybe_latch_set2_identity(msg, self.siglip_stamp_s, self.siglip_rx_s)
        self._feed_local_anchor_classification(msg)

    def on_shape(self, msg: Classification) -> None:
        self.shape = msg
        self.shape_stamp_s = self._now_s()

    def on_advance(self, _: Empty) -> None:
        """Manual debug/dry-run override: force one transition along the chain."""
        if not self._run_started or self._competition_state != "RUNNING":
            return
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
    def _anchor_complete_or_face_next(self) -> None:
        anchor_id = self._active_anchor_id()
        held = self.anchor_inventory.unconfirmed(anchor_id)
        if held and self.body_slot_final_retry_enabled:
            retried = self.anchor_inventory.retry_unconfirmed_once(
                anchor_id,
                now_s=self._now_s(),
            )
            if retried:
                slot_names = ",".join(str(slot.slot_index) for slot in retried)
                self._decide(
                    f"ANCHOR K{anchor_id} FINAL BODY RETRY slots={slot_names}"
                )
                self._enter("SELECT_TARGET")
                return
        if held:
            slot_names = ",".join(str(slot.slot_index) for slot in held)
            self._decide(
                f"ANCHOR K{anchor_id} SKIP UNCONFIRMED slots={slot_names}"
            )
        if self._anchor_idx >= len(self.anchor_inventory.anchors) - 1:
            self._drive(0.0, 0.0)
            self._decide("ANCHOR K3 COMPLETE -> END")
            self._enter("END")
        else:
            self._decide(f"ANCHOR K{anchor_id} COMPLETE -> FACE K{anchor_id + 1}")
            self._enter("ANCHOR_FACE_NEXT")

    def _relative_ready(self, anchor_id: int | None = None):
        estimate = self._relative_estimate(anchor_id)
        if estimate is None:
            self._relative_hold("relative lattice not locked")
            return None
        if not estimate.navigable:
            self._relative_hold(
                f"{estimate.reason} age={estimate.age_s:.2f}s matched={estimate.matched_count}"
            )
            return None
        self._relative_wait_reason = ""
        return estimate

    def _restart_relative_observe_window(self) -> None:
        """Require a new continuous stationary capture after a relative-data dropout."""
        now = self._now_s()
        self.state_enter_s = now
        self._anchor_observe_start_s = now
        self._relative_observe_hit_baseline = {}
        for slot in self.anchor_inventory.anchor_slots(self._active_anchor_id()):
            evidence = self._relative_anchor_tracker.slot_evidence(
                slot.anchor_id, slot.slot_index
            )
            self._relative_observe_hit_baseline[slot.slot_id] = (
                evidence.hits if evidence is not None else 0
            )

    def _step_anchor_classify(self) -> None:
        """Inspect the frozen slot, picking only today's target and closing every other slot."""
        self._drive(0.0, 0.0)
        slot = self._current_anchor_slot()
        if slot is None:
            self._enter("ANCHOR_RETURN")
            return
        fresh = self._anchor_track_for_slot(slot)
        if fresh is not None:
            self.current_target = fresh
        obj = self.current_target
        set_type = int(obj.set_type) if obj is not None else int(slot.set_type)
        class_label = str(obj.class_label) if obj is not None else str(slot.class_label)
        fruit_label = str(obj.fruit_label) if obj is not None else str(slot.fruit_label)
        confidence = float(obj.confidence) if obj is not None else float(slot.confidence)
        wide_identity_uncertain = bool(
            int(slot.set_type) not in {1, 2}
            or not str(slot.class_label)
            or float(slot.confidence) < self.body_slot_wide_low_confidence
        )

        # Body may correct a low-confidence Wide identity, but it can never move the slot.  A
        # generic Body "cube" on a Wide fruit slot remains a fruit geometry candidate because the
        # Body detector historically aliases some photo cubes to that label.
        body_label = self._anchor_body_confirmed_label
        if body_label:
            if int(slot.set_type) == 2 and body_label == "cube":
                set_type = 2
                class_label = "fruit_photo_cube"
            else:
                set_type = int(_LABEL_ST.get(body_label, 0))
                class_label = body_label
            confidence = float(self._anchor_body_confirmed_confidence)
            if set_type == 2:
                # Every fruit slot requires a fresh, stationary Body/SigLIP identity.
                fruit_label = ""
            if obj is not None:
                obj.set_type = set_type
                obj.class_label = class_label
                obj.confidence = confidence
                obj.fruit_label = fruit_label

        # Consume only a classification produced after entering CLASSIFY.  This prevents the
        # Wide/top identity or a previous slot's Body frame from deciding the current fruit slot.
        if (
            set_type == 2
            and self.siglip is not None
            and self.siglip_stamp_s is not None
            and self.siglip_stamp_s >= self.state_enter_s
            and bool(self.siglip.image_face_visible)
            and float(self.siglip.confidence) >= self.conf_threshold
            and str(self.siglip.label)
            and self._anchor_siglip_matches_current_slot()
        ):
            fruit_label = str(self.siglip.label)
            confidence = float(self.siglip.confidence)
            if obj is not None:
                obj.fruit_label = fruit_label
                obj.confidence = confidence

        if set_type == 1:
            is_target = bool(self.set1_label and class_label == self.set1_label)
            if not is_target:
                if confidence < self.conf_threshold:
                    self._anchor_defer_unconfirmed(
                        f"non-target shape confidence {confidence:.2f}"
                    )
                    self._enter("ANCHOR_RETURN")
                    return
                self._anchor_resolve_checked(
                    f"body shape {class_label or 'unknown'}",
                    target=False,
                )
                self._enter("ANCHOR_RETURN")
                return
            if not self._shape_quota_available():
                self._anchor_resolve_checked("shape quota already met", target=True)
                self._enter("ANCHOR_RETURN")
                return
            target_pick_conf = (
                max(self.pick_track_conf, self.conf_threshold)
                if wide_identity_uncertain
                else self.pick_track_conf
            )
            if confidence >= target_pick_conf:
                self._commit_pick(
                    1,
                    class_label,
                    f"anchor K{slot.anchor_id}.{slot.slot_index} conf={confidence:.2f}",
                )
                return
            if self._time_in_state() > self.classify_timeout_sec:
                self._anchor_defer_unconfirmed(
                    f"target shape confidence {confidence:.2f}"
                )
                self._enter("ANCHOR_RETURN")
            return

        if set_type == 2:
            is_target = bool(
                self.set2_label
                and (
                    not self.set2_require_fruit_label
                    or fruit_label == self.set2_label
                )
            )
            if fruit_label and not is_target:
                self._anchor_resolve_checked(f"body fruit {fruit_label}", target=False)
                self._enter("ANCHOR_RETURN")
                return
            if is_target and not self._fruit_quota_available():
                self._anchor_resolve_checked("fruit quota already met", target=True)
                self._enter("ANCHOR_RETURN")
                return
            if is_target and confidence >= self.pick_track_conf:
                self._commit_pick(
                    2,
                    fruit_label or "fruit_photo_cube",
                    f"anchor K{slot.anchor_id}.{slot.slot_index} conf={confidence:.2f}",
                )
                return
            if self._time_in_state() > self.classify_timeout_sec:
                reason = (
                    "fruit identity unknown"
                    if not fruit_label
                    else f"target fruit confidence {confidence:.2f}"
                )
                self._anchor_defer_unconfirmed(reason)
                self._enter("ANCHOR_RETURN")
            return

        if self._time_in_state() > self.classify_timeout_sec:
            self._anchor_defer_unconfirmed("body identity unknown")
            self._enter("ANCHOR_RETURN")

    def _step_relative_anchor_mission(self) -> None:
        """K1 -> K2 -> K3 using only the current base_link virtual lattice."""
        if self.state == "SCAN":
            if not self._relative_try_acquire():
                return
            anchor_id = self._active_anchor_id()
            if anchor_id in self._relative_anchor_arrived:
                if self.anchor_inventory.pending(anchor_id):
                    self._enter("SELECT_TARGET")
                else:
                    self._anchor_complete_or_face_next()
                return
            if self._relative_drive_to_anchor(anchor_id, use_entry=True):
                self._drive(0.0, 0.0, 0.0)
                self._relative_anchor_arrived.add(anchor_id)
                if self.anchor_inventory.anchor_locked(anchor_id):
                    self._decide(f"ANCHOR K{anchor_id} RELATIVE REACHED -> INITIAL QUEUE")
                    if self.anchor_inventory.pending(anchor_id):
                        self._enter("SELECT_TARGET")
                    else:
                        self._anchor_complete_or_face_next()
                else:
                    self._decide(f"ANCHOR K{anchor_id} RELATIVE REACHED -> OBSERVE 3s")
                    self._enter("ANCHOR_OBSERVE")
            return

        if self.state == "ANCHOR_OBSERVE":
            self._drive(0.0, 0.0, 0.0)
            if self._relative_ready() is None:
                self._restart_relative_observe_window()
                return
            if self._time_in_state() < self.anchor_observe_sec:
                return
            anchor_id = self._active_anchor_id()
            self.anchor_inventory.lock_anchor_snapshot(
                anchor_id,
                self._anchor_snapshot_observations(),
                now_s=self._now_s(),
            )
            self._anchor_resolve_wide_trusted(anchor_id)
            counts = self.anchor_inventory.counts(anchor_id)
            self._decide(
                f"ANCHOR K{anchor_id} RELATIVE SNAPSHOT objects={counts['objects']} "
                f"fruit_cubes={counts['fruit_cubes']} empty={counts['empty']} "
                f"pending={counts['pending']}"
            )
            self.get_logger().info(
                f"K{anchor_id} relative snapshot locked: objects={counts['objects']} "
                f"fruit_cubes={counts['fruit_cubes']} empty={counts['empty']} "
                f"pending={counts['pending']}"
            )
            if counts["pending"]:
                self._enter("SELECT_TARGET")
            else:
                self._anchor_complete_or_face_next()
            return

        if self.state == "SELECT_TARGET":
            slot = self._anchor_next_pending()
            if slot is None:
                self._anchor_complete_or_face_next()
                return
            self._anchor_latch_slot(slot)
            self._enter("ANCHOR_FACE_TARGET")
            return

        if self.state == "ANCHOR_FACE_TARGET":
            slot = self._current_anchor_slot()
            if slot is None:
                self._enter("SELECT_TARGET")
                return
            estimate = self._relative_ready(slot.anchor_id)
            if estimate is None:
                return
            if math.hypot(estimate.center_x, estimate.center_y) > self.anchor_reach_tol_m:
                self._relative_drive_to_anchor(slot.anchor_id, use_entry=False)
                return
            point = self._relative_slot_point(slot)
            if point is None:
                self._relative_hold("current virtual slot unavailable")
                return
            fresh = self._anchor_track_for_slot(slot)
            if fresh is not None:
                self.current_target = fresh
            self._anchor_slot_heading = math.atan2(point[1], point[0])
            if self._relative_rotate_to_point(point):
                self._enter("APPROACH")
            return

        if self.state == "APPROACH":
            slot = self._current_anchor_slot()
            if slot is None:
                self._enter("ANCHOR_RETURN")
                return
            if self._relative_ready(slot.anchor_id) is None:
                return
            point = self._relative_slot_point(slot)
            if point is None:
                self._relative_hold("current virtual slot unavailable")
                return
            fresh = self._anchor_track_for_slot(slot)
            if fresh is not None:
                self.current_target = fresh
                self._appr_last_seen_s = self._now_s()
            distance = math.hypot(point[0], point[1])
            bearing = math.atan2(point[1], point[0])
            self._anchor_slot_heading = bearing
            if abs(bearing) > self.relative_anchor_face_tol_rad:
                self._relative_rotate_to_point(point)
                return
            if abs(distance - self.approach_dist_m) <= self.approach_standoff_tol:
                self._drive(0.0, 0.0, 0.0)
                self._enter("ALIGN")
                return
            # Keep the object straight ahead: advance when too far, reverse when the anchor centre
            # is already nearer than the configured body-camera standoff.
            velocity = self.direct_nav_speed if distance > self.approach_dist_m else -self.direct_nav_speed
            self._drive(velocity, 0.0, 0.0)
            return

        if self.state == "ALIGN":
            self._step_align()
            return

        if self.state == "CLASSIFY":
            self._step_anchor_classify()
            return

        if self.state == "PICK":
            if self._pick_base_release_ready():
                self._enter("STORE_IN_TRAY")
            return

        if self.state == "STORE_IN_TRAY":
            self._do_store_in_tray()
            return

        if self.state == "ANCHOR_RETURN":
            anchor_id = self._active_anchor_id()
            if self._relative_drive_to_anchor(anchor_id, use_entry=False):
                self._drive(0.0, 0.0, 0.0)
                self._anchor_clear_current()
                if self.anchor_inventory.pending(anchor_id):
                    self._enter("SELECT_TARGET")
                else:
                    self._anchor_complete_or_face_next()
            return

        if self.state == "ANCHOR_FACE_NEXT":
            current_id = self._active_anchor_id()
            estimate = self._relative_ready(current_id)
            if estimate is None:
                return
            if math.hypot(estimate.center_x, estimate.center_y) > self.anchor_reach_tol_m:
                self._relative_drive_to_anchor(current_id, use_entry=False)
                return
            next_point = self._relative_anchor_tracker.anchor_point(current_id + 1)
            if next_point is None:
                self._relative_hold("next virtual anchor unavailable")
                return
            if self._relative_rotate_to_point(next_point):
                self._anchor_idx += 1
                self._anchor_clear_current()
                self._decide(f"ANCHOR FACE K{self._active_anchor_id()} -> RELATIVE SCAN")
                self._enter("SCAN")
            return

        self._drive(0.0, 0.0, 0.0)

    def _step_anchor_mission(self) -> None:
        """K1 -> K2 -> K3 fixed-snapshot route, isolated from the legacy zone mission."""
        if self.relative_anchor_nav_enabled:
            self._step_relative_anchor_mission()
            return
        if self.state == "SCAN":
            anchor = self._active_anchor()
            if self.anchor_inventory.anchor_locked(anchor.anchor_id):
                if self.anchor_inventory.pending(anchor.anchor_id):
                    self._enter("SELECT_TARGET")
                else:
                    self._anchor_complete_or_face_next()
                return
            heading = float(self.world.robot_theta) if self.world is not None else 0.0
            self._drive_toward(anchor.x, anchor.y, heading, exclude_id=0)
            distance = self._distance_to(anchor.x, anchor.y)
            if distance is not None and distance <= self.anchor_reach_tol_m:
                self._drive(0.0, 0.0)
                self._decide(f"ANCHOR K{anchor.anchor_id} REACHED -> OBSERVE 3s")
                self._enter("ANCHOR_OBSERVE")
            return

        if self.state == "ANCHOR_OBSERVE":
            self._drive(0.0, 0.0)
            if self._time_in_state() < self.anchor_observe_sec:
                return
            anchor_id = self._active_anchor_id()
            self.anchor_inventory.lock_anchor_snapshot(
                anchor_id,
                self._anchor_snapshot_observations(),
                now_s=self._now_s(),
            )
            self._anchor_resolve_wide_trusted(anchor_id)
            counts = self.anchor_inventory.counts(anchor_id)
            self._decide(
                f"ANCHOR K{anchor_id} SNAPSHOT objects={counts['objects']} "
                f"fruit_cubes={counts['fruit_cubes']} empty={counts['empty']} "
                f"pending={counts['pending']}"
            )
            self.get_logger().info(
                f"K{anchor_id} snapshot locked: objects={counts['objects']} "
                f"fruit_cubes={counts['fruit_cubes']} empty={counts['empty']} "
                f"pending={counts['pending']}"
            )
            if counts["pending"]:
                self._enter("SELECT_TARGET")
            else:
                self._anchor_complete_or_face_next()
            return

        if self.state == "SELECT_TARGET":
            slot = self._anchor_next_pending()
            if slot is None:
                self._anchor_complete_or_face_next()
                return
            self._anchor_latch_slot(slot)
            self._enter("ANCHOR_FACE_TARGET")
            return

        if self.state == "ANCHOR_FACE_TARGET":
            slot = self._current_anchor_slot()
            if slot is None:
                self._enter("SELECT_TARGET")
                return
            anchor = self._active_anchor()
            self._drive_toward_direct(anchor.x, anchor.y, self._anchor_slot_heading)
            distance = self._distance_to(anchor.x, anchor.y)
            if self.world is None or distance is None:
                return
            yaw_error = self._wrap_pi(self._anchor_slot_heading - float(self.world.robot_theta))
            if distance <= self.anchor_reach_tol_m and abs(yaw_error) <= self.anchor_face_tol_rad:
                self._drive(0.0, 0.0)
                self._enter("APPROACH")
            return

        if self.state == "APPROACH":
            slot = self._current_anchor_slot()
            if slot is None:
                self._enter("ANCHOR_RETURN")
                return
            fresh = self._anchor_track_for_slot(slot)
            if fresh is not None:
                self.current_target = fresh
                self._appr_last_seen_s = self._now_s()
            self._appr_tgt_xy = (slot.x, slot.y)
            bearing = self._anchor_slot_heading
            sx = slot.x - self.approach_dist_m * math.cos(bearing)
            sy = slot.y - self.approach_dist_m * math.sin(bearing)
            exclude_id = int(self.current_target.id) if self.current_target is not None else 0
            distance = self._distance_to(sx, sy)
            if distance is None or self.world is None:
                return
            if distance <= self.approach_standoff_tol:
                if self._rotate_to_heading(bearing, self.anchor_face_tol_rad, self.direct_nav_omega_max):
                    self._enter("ALIGN")
                return
            self._drive_toward(
                sx, sy, bearing, exclude_id=exclude_id, route_mode="object_approach"
            )
            return

        if self.state == "ALIGN":
            self._step_align()
            return

        if self.state == "CLASSIFY":
            self._step_anchor_classify()
            return

        if self.state == "PICK":
            if self._pick_base_release_ready():
                self._enter("STORE_IN_TRAY")
            return

        if self.state == "STORE_IN_TRAY":
            self._do_store_in_tray()
            return

        if self.state == "ANCHOR_RETURN":
            anchor = self._active_anchor()
            exclude_id = int(self.current_target.id) if self.current_target is not None else 0
            heading = float(self.world.robot_theta) if self.world is not None else 0.0
            self._drive_toward(anchor.x, anchor.y, heading, exclude_id=exclude_id)
            distance = self._distance_to(anchor.x, anchor.y)
            if distance is not None and distance <= self.anchor_reach_tol_m:
                self._drive(0.0, 0.0)
                self._anchor_clear_current()
                if self.anchor_inventory.pending(anchor.anchor_id):
                    self._enter("SELECT_TARGET")
                else:
                    self._anchor_complete_or_face_next()
            return

        if self.state == "ANCHOR_FACE_NEXT":
            current = self._active_anchor()
            next_anchor = self.anchor_inventory.get_anchor(current.anchor_id + 1)
            heading = math.atan2(next_anchor.y - current.y, next_anchor.x - current.x)
            self._drive_toward_direct(current.x, current.y, heading)
            distance = self._distance_to(current.x, current.y)
            if self.world is None or distance is None:
                return
            yaw_error = self._wrap_pi(heading - float(self.world.robot_theta))
            if distance <= self.anchor_reach_tol_m and abs(yaw_error) <= self.anchor_face_tol_rad:
                self._drive(0.0, 0.0)
                self._anchor_idx += 1
                self._anchor_clear_current()
                self._decide(f"ANCHOR FACE K{self._active_anchor_id()} -> SCAN")
                self._enter("SCAN")
            return

        self._drive(0.0, 0.0)  # END and any unexpected anchor-mode state fail safe

    def _do_store_in_tray(self) -> None:
        """Tray bookkeeping + blacklist picked object, then branch (phase-aware)."""
        if self.anchor_mission_enabled and (slot := self._current_anchor_slot()) is not None:
            if self.set_type == 1:
                self.tray_shape += 1
            elif self.set_type == 2:
                self.tray_fruit += 1
            self.anchor_inventory.mark_picked(slot.slot_id, now_s=self._now_s())
            if self.current_target is not None and not self._relative_mode():
                self._blacklist(self.current_target.id)
            self._decide(f"ANCHOR K{slot.anchor_id}.{slot.slot_index} PICKED -> RETURN")
            self.set_type = 0
            self._opportunistic_set2_active = False
            self._enter("ANCHOR_RETURN")
            return
        picked_shape = self.set_type == 1
        if picked_shape:
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
        if self.zone_mission_enabled and picked_shape:
            self._post_pick_lane_entry_pending = True
            self._decide("PICK COMPLETE -> NEXT LANE ENTRY")

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
            self._clear_zone_entry_anchor()
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

    def _maybe_start_timed_storage(self) -> None:
        """Preempt when the RUNNING-relative time left only covers return and dumping."""
        if not self.timed_storage_enabled:
            return
        route_distance = (
            self._estimate_storage_return_path_distance()
            if getattr(self, "timed_storage_dynamic_enabled", False)
            else None
        )
        due = timed_storage_due(
            run_started=self._run_started,
            triggered=self._timed_storage_triggered,
            completed=self._storage_route_completed,
            now_s=self._now_s(),
            run_start_s=self._node_start_s,
            trigger_sec=self.timed_storage_start_sec,
            dynamic_enabled=getattr(self, "timed_storage_dynamic_enabled", False),
            match_duration_sec=getattr(
                self, "timed_storage_match_duration_sec", 180.0
            ),
            route_distance_m=route_distance,
            effective_speed_mps=getattr(
                self, "timed_storage_effective_speed_mps", 0.10
            ),
            fixed_overhead_sec=getattr(
                self, "timed_storage_fixed_overhead_sec", 12.0
            ),
            safety_margin_sec=getattr(
                self, "timed_storage_safety_margin_sec", 10.0
            ),
            distance_safety_factor=getattr(
                self, "timed_storage_distance_safety_factor", 1.0
            ),
        )
        if not due:
            return
        if self.state in {"DRIVE_TO_STORAGE", "ALIGN_OVER_BIN", "DUMP_ALL"}:
            self._timed_storage_triggered = True
            return
        # Do not drive away while the arm is physically executing a pick. STORE_IN_TRAY runs on the
        # next tick to commit the completed pick, then the deadline starts parking immediately.
        local_pick_running = (
            self.state == LOCAL_ANCHOR_STATE
            and self._local_anchor_fsm.state in {"PICK_TRIGGER", "PICK_WAIT"}
        )
        if (
            self.state in {"PICK", "STORE_IN_TRAY"}
            or local_pick_running
            or self._local_anchor_pose_refresh_required
        ):
            self._drive(0.0, 0.0, 0.0)
            return
        if self.state == LOCAL_ANCHOR_STATE:
            now = self._now_s()
            self._local_anchor_fsm.abort(
                now,
                "timed storage requested; stop for pose refresh",
            )
            self._drive(0.0, 0.0, 0.0)
            self._finish_local_anchor_inspection()
            return

        self._timed_storage_triggered = True
        self._pending_pick = None
        self.current_target = None
        self._clear_current_slot()
        self._anchor_current_slot_id = None
        self.set_type = 0
        self._opportunistic_set2_active = False
        self._post_pick_lane_entry_pending = False
        destination = f"BOTTOM WALL {self.storage_bottom_wall_stop_m:.2f}m"
        now = self._now_s()
        match_elapsed = now - self._node_start_s
        process_elapsed = now - getattr(self, "_process_start_s", self._node_start_s)
        distance_text = (
            f" path={route_distance:.2f}m" if route_distance is not None else " path=unknown"
        )
        self._decide(
            f"STORAGE DEADLINE match_t={match_elapsed:.1f}s "
            f"run_uptime={process_elapsed:.1f}s{distance_text} -> {destination}"
        )
        self._enter("DRIVE_TO_STORAGE")

    def _estimate_storage_return_path_distance(self) -> float | None:
        """Estimate collision-checked travel to staging plus the final reverse entry."""
        if self.world is None:
            return None
        start = (float(self.world.robot_x), float(self.world.robot_y))
        reverse_entry = bool(getattr(self, "storage_reverse_entry_enabled", False))
        nav = (
            (self.storage_staging_x, self.storage_staging_y)
            if reverse_entry
            else (self.storage_x, self.storage_y)
        )
        obstacles = self._obstacles_snapshot()
        route_mode = (
            "legacy" if getattr(self, "storage_fast_return_enabled", False) else "grid_only"
        )
        path = self._planner.plan(start, nav, obstacles, route_mode=route_mode)
        if path is None:
            # A transient blocked grid must not suppress the deadline. Euclidean distance is
            # retained and the configured safety factor makes this fallback conservative.
            distance = math.hypot(nav[0] - start[0], nav[1] - start[1])
        else:
            distance = 0.0
            previous = start
            for point in path:
                distance += math.hypot(point[0] - previous[0], point[1] - previous[1])
                previous = point
        if reverse_entry:
            distance += math.hypot(
                self.storage_x - self.storage_staging_x,
                self.storage_y - self.storage_staging_y,
            )
        return distance

    def _reset_storage_wall_confirmation(self) -> None:
        self._storage_wall_confirm_count = 0
        self._storage_wall_confirm_last_seq = -1

    def _storage_wall_distance(self, *, axis: str, direction: int) -> float | None:
        if self._now_s() - self._wall_segments_s > self.storage_wall_max_age_sec:
            return None
        return directional_wall_distance(
            self._wall_segments,
            float(self.world.robot_x),
            float(self.world.robot_y),
            axis=axis,
            direction=direction,
        )

    def _storage_wall_stop_confirmed(self, distance: float | None, threshold: float) -> bool:
        """Require distinct wall-localizer frames before accepting a wall stop distance."""
        if self._wall_segments_seq == self._storage_wall_confirm_last_seq:
            return self._storage_wall_confirm_count >= self.storage_wall_confirm_frames
        self._storage_wall_confirm_last_seq = self._wall_segments_seq
        if distance is not None and distance <= threshold:
            self._storage_wall_confirm_count += 1
        else:
            self._storage_wall_confirm_count = 0
        return self._storage_wall_confirm_count >= self.storage_wall_confirm_frames

    def _storage_pose_wall_distance(self, *, axis: str, direction: int) -> float:
        xmin, xmax, ymin, ymax = self._field_bounds
        if axis == "y":
            wall = ymax if direction >= 0 else ymin
            position = float(self.world.robot_y)
        else:
            wall = xmax if direction >= 0 else xmin
            position = float(self.world.robot_x)
        return max(0.0, (wall - position) if direction >= 0 else (position - wall))

    def _storage_contact_command(self, *, reverse: bool) -> tuple[float, float, float]:
        """Keep a small heading-controlled drive command applied against the storage wall."""
        if not self.storage_contact_hold_enabled or self.world is None:
            return 0.0, 0.0, 0.0
        command_fn = straight_reverse_command if reverse else straight_forward_command
        target_heading = (
            self.storage_right_heading_rad if reverse else self.storage_down_heading_rad
        )
        return command_fn(
            current_heading=float(self.world.robot_theta),
            target_heading=target_heading,
            distance_m=1.0,
            max_speed=self.storage_contact_hold_speed,
            heading_kp=self.storage_reverse_heading_kp,
            omega_max=self.storage_reverse_omega_max,
        )

    def _drive_storage_contact_hold(self) -> None:
        if not self.storage_wall_guided_enabled:
            self._drive(0.0, 0.0, 0.0)
            return
        self._drive(*self._storage_contact_command(reverse=True))

    def _finish_storage_reverse(self, reason: str) -> None:
        self._drive(0.0, 0.0, 0.0)
        self._storage_route_completed = True
        self._decide(f"STORAGE LEFT WALL PARKED {reason}")
        self._enter("ALIGN_OVER_BIN")

    def _mission_elapsed_s(self) -> float:
        if not self._run_started:
            return 0.0
        return max(0.0, self._now_s() - self._node_start_s)

    def _publish_match_time(self) -> None:
        now = self._now_s()
        process_elapsed = now - getattr(self, "_process_start_s", self._node_start_s)
        match_elapsed = now - self._node_start_s if self._run_started else -1.0
        msg = Float32MultiArray()
        msg.data = [
            float(max(0.0, process_elapsed)),
            float(max(-1.0, match_elapsed)),
            float(getattr(self, "timed_storage_start_sec", 0.0)),
            float(getattr(self, "storage_last_chance_trigger_sec", 0.0)),
            float(getattr(self, "timed_storage_match_duration_sec", 180.0)),
        ]
        self.pub_match_time.publish(msg)

    def _fresh_storage_wall_available(self) -> bool:
        return (
            bool(self._wall_segments)
            and self._now_s() - self._wall_segments_s <= self.storage_wall_max_age_sec
        )

    def _maybe_start_storage_last_chance(self) -> bool:
        if self._storage_last_chance_active:
            return True
        if not self.storage_last_chance_enabled:
            return False
        if self._storage_route_completed or self.state != "DRIVE_TO_STORAGE":
            return False
        elapsed = self._mission_elapsed_s()
        if elapsed < self.storage_last_chance_trigger_sec:
            return False
        if self._fresh_storage_wall_available():
            return False
        now = self._now_s()
        self._storage_last_chance_active = True
        self._storage_last_chance_phase = "pre_backoff"
        self._storage_last_chance_started_s = now
        self._storage_last_chance_phase_started_s = now
        self._storage_last_chance_turn_heading = None
        self._storage_pose_fallback_key = None
        self._reset_storage_wall_confirmation()
        self._reset_pulsed_heading()
        self._decide(
            "STORAGE LAST-CHANCE no fresh wall "
            f"t={elapsed:.1f}s -> BACKOFF {self.storage_last_chance_pre_backoff_m * 100.0:.0f}cm"
        )
        return True

    def _step_storage_last_chance(self) -> None:
        if self.world is None:
            self._drive(0.0, 0.0, 0.0)
            return
        now = self._now_s()
        total_elapsed = now - self._storage_last_chance_started_s
        if (
            self.storage_last_chance_force_dump_after_sec > 0.0
            and total_elapsed >= self.storage_last_chance_force_dump_after_sec
        ):
            self._decide(
                f"STORAGE LAST-CHANCE force dump after {total_elapsed:.1f}s"
            )
            self._finish_storage_reverse("last-chance force dump")
            return

        speed = max(0.0, self.storage_last_chance_speed)
        phase = self._storage_last_chance_phase
        phase_elapsed = now - self._storage_last_chance_phase_started_s

        if phase == "pre_backoff":
            backoff_speed = max(0.01, self.storage_last_chance_pre_backoff_speed)
            backoff_sec = self.storage_last_chance_pre_backoff_m / backoff_speed
            if phase_elapsed < backoff_sec:
                self._drive(-backoff_speed, 0.0, 0.0)
                return
            self._storage_last_chance_phase = "turn_left"
            self._storage_last_chance_phase_started_s = now
            self._storage_last_chance_turn_heading = self._wrap_pi(
                float(self.world.robot_theta)
                + math.radians(self.storage_last_chance_turn_left_deg)
            )
            self._reset_pulsed_heading()
            self._decide(
                f"STORAGE LAST-CHANCE left turn {self.storage_last_chance_turn_left_deg:.0f}deg"
            )
            phase = self._storage_last_chance_phase

        if phase == "turn_left":
            target = self._storage_last_chance_turn_heading
            if target is None:
                target = self._wrap_pi(
                    float(self.world.robot_theta)
                    + math.radians(self.storage_last_chance_turn_left_deg)
                )
                self._storage_last_chance_turn_heading = target
            aligned = self._turn_in_place_pulsed(
                target,
                self.storage_heading_tolerance_rad,
                key=("storage_last_chance_turn_left", round(target, 6)),
            )
            if aligned:
                self._storage_last_chance_phase = "reverse_charge"
                self._storage_last_chance_phase_started_s = now
                self._decide(
                    f"STORAGE LAST-CHANCE reverse charge {self.storage_last_chance_reverse_sec:.1f}s "
                    f"speed={speed:.2f}"
                )
            return

        if phase == "reverse_charge":
            if phase_elapsed < self.storage_last_chance_reverse_sec:
                self._drive(-speed, 0.0, 0.0)
                return
            self._storage_last_chance_phase = "strafe_right"
            self._storage_last_chance_phase_started_s = now
            self._decide(
                f"STORAGE LAST-CHANCE alternate right strafe/reverse "
                f"{self.storage_last_chance_alternate_sec:.1f}s each"
            )
            self._drive(0.0, -speed, 0.0)
            return

        if phase == "strafe_right":
            if phase_elapsed >= self.storage_last_chance_alternate_sec:
                self._storage_last_chance_phase = "alternate_reverse"
                self._storage_last_chance_phase_started_s = now
                self._decide("STORAGE LAST-CHANCE alternate reverse")
                self._drive(-speed, 0.0, 0.0)
                return
            self._drive(0.0, -speed, 0.0)
            return

        if phase == "alternate_reverse":
            if phase_elapsed >= self.storage_last_chance_alternate_sec:
                self._storage_last_chance_phase = "strafe_right"
                self._storage_last_chance_phase_started_s = now
                self._decide("STORAGE LAST-CHANCE alternate right strafe")
                self._drive(0.0, -speed, 0.0)
                return
            self._drive(-speed, 0.0, 0.0)
            return

        self._storage_last_chance_phase = "turn_left"
        self._storage_last_chance_phase_started_s = now
        self._drive(0.0, 0.0, 0.0)

    def _step_drive_to_storage(self) -> None:
        """Approach the bottom wall, face right, then reverse toward the left wall."""
        if self.world is None:
            self._drive(0.0, 0.0, 0.0)
            return
        if self._maybe_start_storage_last_chance():
            self._step_storage_last_chance()
            return

        if self._storage_route_phase == "face_bottom_wall":
            aligned = self._turn_in_place_pulsed(
                self.storage_down_heading_rad,
                self.storage_heading_tolerance_rad,
                key=("storage_face_bottom_wall", round(self.storage_down_heading_rad, 6)),
            )
            if aligned:
                self._storage_route_phase = "approach_bottom_wall"
                self._reset_storage_wall_confirmation()
                self._decide(
                    f"STORAGE FACE DOWN {math.degrees(self.storage_down_heading_rad):+.1f}deg "
                    f"-> BOTTOM WALL {self.storage_bottom_wall_stop_m:.2f}m"
                )
            return

        if self._storage_route_phase == "approach_bottom_wall":
            observed = self._storage_wall_distance(axis="y", direction=1)
            pose_distance = self._storage_pose_wall_distance(axis="y", direction=1)
            if observed is not None and observed <= self.storage_bottom_wall_stop_m:
                self._storage_pose_fallback_key = None
                self._drive(0.0, 0.0, 0.0)
                if self._storage_wall_stop_confirmed(
                    observed, self.storage_bottom_wall_stop_m
                ):
                    self._storage_route_phase = "face_right"
                    self._reset_storage_wall_confirmation()
                    self._reset_pulsed_heading()
                    self._decide(
                        f"STORAGE BOTTOM WALL {observed:.2f}m CONFIRMED -> FACE RIGHT"
                    )
                return
            self._storage_wall_stop_confirmed(observed, self.storage_bottom_wall_stop_m)
            if observed is None and pose_distance <= self.storage_wall_detection_required_m:
                if self.storage_pose_fallback_enabled:
                    key = "bottom_wall"
                    now = self._now_s()
                    if self._storage_pose_fallback_key != key:
                        self._storage_pose_fallback_key = key
                        self._storage_pose_fallback_start_s = now
                        self._decide(
                            f"STORAGE POSE FALLBACK bottom wall pose={pose_distance:.2f}m "
                            f"-> CONTACT {self.storage_pose_fallback_hold_sec:.1f}s"
                        )
                    elapsed = now - self._storage_pose_fallback_start_s
                    if elapsed >= self.storage_pose_fallback_hold_sec:
                        self._drive(0.0, 0.0, 0.0)
                        self._storage_route_phase = "face_right"
                        self._storage_pose_fallback_key = None
                        self._reset_storage_wall_confirmation()
                        self._reset_pulsed_heading()
                        self._decide(
                            f"STORAGE BOTTOM WALL POSE FALLBACK {pose_distance:.2f}m "
                            "-> FACE RIGHT"
                        )
                        return
                    vx, vy, omega = straight_forward_command(
                        current_heading=float(self.world.robot_theta),
                        target_heading=self.storage_down_heading_rad,
                        distance_m=1.0,
                        max_speed=self.storage_pose_fallback_contact_speed,
                        heading_kp=self.storage_reverse_heading_kp,
                        omega_max=self.storage_reverse_omega_max,
                    )
                    self._drive(vx, vy, omega)
                    return
                self._drive(0.0, 0.0, 0.0)
                self.get_logger().warn(
                    "storage bottom wall is near by pose but has no fresh wall detection; "
                    "holding",
                    throttle_duration_sec=2.0,
                )
                return
            self._storage_pose_fallback_key = None
            heading_error = self._wrap_pi(
                self.storage_down_heading_rad - float(self.world.robot_theta)
            )
            if abs(heading_error) >= self.storage_reverse_realign_rad:
                self._drive(0.0, 0.0, 0.0)
                self._storage_route_phase = "face_bottom_wall"
                self._reset_pulsed_heading()
                self._decide(
                    f"STORAGE BOTTOM RE-ALIGN error={math.degrees(heading_error):+.1f}deg"
                )
                return
            distance = observed if observed is not None else pose_distance
            vx, vy, omega = straight_forward_command(
                current_heading=float(self.world.robot_theta),
                target_heading=self.storage_down_heading_rad,
                distance_m=max(0.0, distance - self.storage_bottom_wall_stop_m),
                max_speed=self.storage_wall_approach_speed,
                heading_kp=self.storage_reverse_heading_kp,
                omega_max=self.storage_reverse_omega_max,
            )
            self._drive(vx, vy, omega)
            return

        if self._storage_route_phase == "face_right":
            aligned = self._turn_in_place_pulsed(
                self.storage_right_heading_rad,
                self.storage_heading_tolerance_rad,
                key=("storage_face_right", round(self.storage_right_heading_rad, 6)),
            )
            if aligned:
                self._storage_route_phase = "reverse_to_left_wall"
                self._reset_storage_wall_confirmation()
                self._decide(
                    f"STORAGE FACE RIGHT {math.degrees(self.storage_right_heading_rad):+.1f}deg "
                    f"-> REVERSE TO LEFT WALL {self.storage_left_wall_stop_m:.2f}m"
                )
            return

        if self._storage_route_phase != "reverse_to_left_wall":
            self._drive(0.0, 0.0, 0.0)
            self.get_logger().error(
                f"unknown storage route phase '{self._storage_route_phase}'; holding stopped",
                throttle_duration_sec=2.0,
            )
            return

        observed = self._storage_wall_distance(axis="x", direction=1)
        pose_distance = self._storage_pose_wall_distance(axis="x", direction=1)
        if observed is not None and observed <= self.storage_left_wall_stop_m:
            self._storage_pose_fallback_key = None
            self._drive(0.0, 0.0, 0.0)
            if self._storage_wall_stop_confirmed(observed, self.storage_left_wall_stop_m):
                self._finish_storage_reverse(f"left wall={observed:.2f}m")
            return
        self._storage_wall_stop_confirmed(observed, self.storage_left_wall_stop_m)
        if observed is None and pose_distance <= self.storage_wall_detection_required_m:
            if self.storage_pose_fallback_enabled:
                key = "left_wall"
                now = self._now_s()
                if self._storage_pose_fallback_key != key:
                    self._storage_pose_fallback_key = key
                    self._storage_pose_fallback_start_s = now
                    self._decide(
                        f"STORAGE POSE FALLBACK left wall pose={pose_distance:.2f}m "
                        f"-> REVERSE CONTACT {self.storage_pose_fallback_hold_sec:.1f}s"
                    )
                elapsed = now - self._storage_pose_fallback_start_s
                if elapsed >= self.storage_pose_fallback_hold_sec:
                    self._storage_pose_fallback_key = None
                    self._finish_storage_reverse(f"pose fallback left wall={pose_distance:.2f}m")
                    return
                vx, vy, omega = straight_reverse_command(
                    current_heading=float(self.world.robot_theta),
                    target_heading=self.storage_right_heading_rad,
                    distance_m=1.0,
                    max_speed=self.storage_pose_fallback_contact_speed,
                    heading_kp=self.storage_reverse_heading_kp,
                    omega_max=self.storage_reverse_omega_max,
                )
                self._drive(vx, vy, omega)
                return
            self._drive(0.0, 0.0, 0.0)
            self.get_logger().warn(
                "storage left wall is near by pose but has no fresh wall detection; "
                "holding",
                throttle_duration_sec=2.0,
            )
            return
        self._storage_pose_fallback_key = None

        heading_error = self._wrap_pi(
            self.storage_right_heading_rad - float(self.world.robot_theta)
        )
        if abs(heading_error) >= self.storage_reverse_realign_rad:
            self._drive(0.0, 0.0, 0.0)
            self._storage_route_phase = "face_right"
            self._reset_pulsed_heading()
            self._decide(
                f"STORAGE REVERSE RE-ALIGN error={math.degrees(heading_error):+.1f}deg"
            )
            return

        distance = observed if observed is not None else pose_distance
        vx, vy, omega = straight_reverse_command(
            current_heading=float(self.world.robot_theta),
            target_heading=self.storage_right_heading_rad,
            distance_m=max(0.0, distance - self.storage_left_wall_stop_m),
            max_speed=self.storage_reverse_speed,
            heading_kp=self.storage_reverse_heading_kp,
            omega_max=self.storage_reverse_omega_max,
        )
        self._drive(vx, vy, omega)

    # -------------------------------------------------------------- transitions
    def _step(self) -> None:
        """Condition-driven transition for the current state (one tick)."""
        self._update_global_seen_nodes()
        if self.state == "WAIT_FOR_ARM":
            self._step_wait_for_arm()

        elif self.state == "OPENING":
            self._step_opening()

        elif self.state == "WALL_INIT":
            self._step_wall_initialization()

        elif self.anchor_mission_enabled:
            self._step_anchor_mission()

        elif self.state == "SCAN":
            if self.zone_mission_enabled and self.zone_anchor_nav_enabled:
                if self._zone_anchor_reached_idx != self._zone_idx:
                    entry_anchor = self._zone_entry_anchor_point()
                    if entry_anchor is None:
                        self._drive(0.0, 0.0)
                        return
                    zx, zy = entry_anchor
                    d_zone = self._distance_to(zx, zy)
                    anchor_reached = d_zone is not None and d_zone <= self.zone_center_reach_tol_m
                    if not anchor_reached:
                        self._reset_zone_scan_timer()
                        if self._opening_zone_entry_pending and self._zone_idx == 0:
                            anchor_reached = self._step_opening_zone_entry(entry_anchor)
                        else:
                            transition_reached = self._step_zone_transition_entry(entry_anchor)
                            if transition_reached is None:
                                th = self.world.robot_theta if self.world is not None else 0.0
                                self._drive_toward(
                                    zx,
                                    zy,
                                    th,
                                    exclude_id=0,
                                    route_mode=self._next_travel_route_mode(),
                                    speed_limit_mps=self._zone_anchor_speed_limit(entry_anchor),
                                )
                            else:
                                anchor_reached = transition_reached
                        if not anchor_reached:
                            return
                    self._opening_zone_entry_pending = False
                    self._zone_anchor_reached_idx = self._zone_idx
                    self._decide(f"ZONE {self._active_zone_id()} ANCHOR REACHED")
                    if self._start_local_anchor_inspection():
                        return
                if not self._zone_is_stabilized():
                    self._drive(0.0, 0.0)
                    self._decide(f"ZONE {self._active_zone_id()} STABILIZE")
                    self._enter("ZONE_STABILIZE")
                    return
            if self.global_target_mode:
                ready, reason = self._global_navigation_ready()
                if not ready:
                    self._drive(0.0, 0.0, 0.0)
                    self.get_logger().info(
                        f"GLOBAL READY wait: {reason}",
                        throttle_duration_sec=1.0,
                    )
                    return
                phase_target_available = self._select_next_global_target() is not None
            else:
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

        elif self.state == LOCAL_ANCHOR_STATE:
            self._step_local_anchor_inspection()

        elif self.state == "ZONE_STABILIZE":
            self._drive(0.0, 0.0)
            stabilize_sec = self.zone_stabilize_sec if self.zone_stabilize_enabled else 0.0
            if self._time_in_state() >= stabilize_sec:
                pose_ready, pose_detail = self._local_anchor_pose_refresh_status()
                if not pose_ready:
                    self.get_logger().warn(
                        f"zone {self._active_zone_id()} waiting for post-local pose: "
                        f"{pose_detail}",
                        throttle_duration_sec=1.0,
                    )
                    return
                if self._local_anchor_pose_refresh_required:
                    self._local_anchor_pose_refresh_required = False
                    self.get_logger().info(
                        f"zone {self._active_zone_id()} post-local {pose_detail}"
                    )
                    self._decide(
                        f"ZONE {self._active_zone_id()} LOCAL POSE CONFIRMED"
                    )
                self._zone_stabilized_idx = self._zone_idx
                self._zone_stabilized_after_s = self._zone_stabilize_start_s
                self._reset_zone_scan_timer()
                self.get_logger().info(
                    f"zone {self._active_zone_id()} stabilized for {stabilize_sec:.1f}s -> SCAN")
                self._decide(f"ZONE {self._active_zone_id()} STABILIZED")
                self._enter("SCAN")

        elif self.state == "SELECT_TARGET":
            if not self._zone_ready_for_selection():
                self._enter("SCAN")
                return
            if self.global_target_mode:
                sel = self._select_next_global_target()
                if sel is not None and self._latch_mixed_target(sel[0], sel[1]):
                    self._enter("APPROACH")
                else:
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
            if self._reject_latched_set2_non_target():
                return
            if self.global_target_mode:
                self._maybe_global_retarget()
                if self._reject_latched_set1_non_target():
                    return
            elif not self.mixed_target_mode and not self._opportunistic_set2_active:
                opp = self._nearest_opportunistic_set2()
                if opp is not None:
                    self._opportunistic_set2_active = True
                    if self.set2_slot_enabled and (slot := self._slot_for_object(opp)) is not None:
                        self._latch_set2_slot(slot, decision_prefix="OPP SLOT INTERRUPT")
                        self._appr_tgt_xy = (slot.x, slot.y)
                    else:
                        self.current_target = opp
                        self._appr_tgt_xy = (opp.x, opp.y)
                    self._begin_set2_identity_visit()
                    self._appr_last_seen_s = self._now_s()
                    self._plan = None
                    self._reset_approach_motion()
                    self._latch_approach_goal(self._appr_tgt_xy)
                    self.get_logger().info(
                        f"APPROACH interrupt: opportunistic Set2 target #{opp.id} '{opp.fruit_label}'")
                    self._decide(f"OPP SET2 INTERRUPT {opp.fruit_label} #{opp.id}")
            if self.current_target is not None:
                fresh_latched = self._lookup_object(int(self.current_target.id))
                if fresh_latched is None and self.set1_label == "cube":
                    fresh_latched = self._nearest_object_at(
                        self._appr_tgt_xy,
                        self.zone_ambiguous_exclusion_radius_m,
                    )
                previous_unknown_count = self._plain_cube_unknown_count
                identity_action = self._plain_cube_approach_identity_action(fresh_latched)
                if identity_action == "hold":
                    self._drive(0.0, 0.0, 0.0)
                    if self._plain_cube_unknown_count != previous_unknown_count:
                        self._decide(
                            f"PLAIN CUBE AMBIGUOUS #{fresh_latched.id} "
                            f"{self._plain_cube_unknown_count}/"
                            f"{self.plain_cube_ambiguous_observations}; HOLD"
                        )
                    return
                if identity_action == "exclude" and fresh_latched is not None:
                    self._drive(0.0, 0.0, 0.0)
                    reason = fresh_latched.class_label or "unknown"
                    self._exclude_ambiguous_for_active_zone(
                        fresh_latched,
                        self._appr_tgt_xy,
                        reason,
                    )
                    self.current_target = None
                    self._appr_tgt_xy = None
                    self._enter("SCAN")
                    return
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
                    self._appr_last_seen_s = self._now_s()
                elif (self._appr_tgt_xy is None
                      or self._now_s() - self._appr_last_seen_s > self.approach_lost_grace_sec):
                    # The global map already supplied a fixed collision-safe stand-off goal.
                    # Preserve that route across temporary track/id churn and let ALIGN make
                    # the final Body-presence decision at the destination.  Restarting SCAN
                    # here destroys the route and alternates between lane-heading and
                    # object-facing pivots after every short perception dropout.
                    keep_latched_global_goal = keep_latched_global_approach(
                        global_target_mode=self.global_target_mode,
                        target_xy=self._appr_tgt_xy,
                        current_target=self.current_target,
                    )
                    if not keep_latched_global_goal:
                        self._enter("SCAN")            # no usable latched goal -> search again
                        return
            # Fresh observations keep the target identity alive, but the navigation goal cannot move
            # around the object after APPROACH has started.
            if (self._approach_standoff_xy is None
                    or self._approach_heading_latched is None):
                if not self._latch_approach_goal(self._appr_tgt_xy):
                    self._drive(0.0, 0.0, 0.0)
                    return
            sx, sy = self._approach_standoff_xy
            bearing = self._approach_heading_latched
            exclude = self.current_target.id if self.current_target is not None else 0
            d = self._distance_to(sx, sy)
            approach_body_target = self._fresh_approach_body_target()
            if d is not None:
                handled, heading_ready = self._step_approach_arrival(
                    d,
                    self.align_retry_heading_tol_rad,
                    self.direct_nav_omega_max,
                    body_target_visible=approach_body_target is not None,
                )
                if handled and not heading_ready:
                    return
                if (
                    heading_ready
                    and approach_body_target is None
                    and not self._ready_for_precise_align_retry(bearing)
                ):
                    return
            if d is not None and self._approach_motion_phase == "ready":
                # A frozen global route can outlive a transient world track. Before entering
                # ALIGN, verify that either the same physical map target or a spatially matching
                # Body object still exists; otherwise reselect in under a second, not 12 seconds.
                standoff_body = self._body_target_base(self._body_label())
                if standoff_body is None:
                    other = self._body_nearest_any()
                    if other is not None:
                        standoff_body = (
                            str(other[0]), float(other[1]), float(other[2]), 0.0
                        )
                if self._handle_absent_global_target(
                    self._now_s(), standoff_body, context="APPROACH stand-off"
                ):
                    return
                # Phase 2: classify at stand-off before spending time on final alignment. A
                # capture-time-bound target proceeds immediately; a bound non-target was already
                # rejected at the start of this state, and an untyped cube gets one short wait.
                latched_identity = self._latched_set2_identity()
                if latched_identity is not None and latched_identity.verdict == "target":
                    self.get_logger().info(
                        f"APPROACH: latched Body {latched_identity.label} "
                        "already identifies target -> ALIGN now"
                    )
                    self._enter("ALIGN")
                    return
                if self._set2_slot_mode() and self.set2_require_fruit_label:
                    self._drive(0.0, 0.0, 0.0)
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
                    self._drive(0.0, 0.0, 0.0)
                    if self._standoff_arrived_s is None:
                        self._standoff_arrived_s = self._now_s()
                    if self._now_s() - self._standoff_arrived_s < self.classify_standoff_sec:
                        return
                self._enter("ALIGN")
                return
            route_arrived = self._drive_toward(
                sx,
                sy,
                bearing,
                exclude_id=exclude,
                route_mode=getattr(
                    self, "_approach_route_mode", self._next_travel_route_mode()
                ),
                speed_limit_mps=(
                    self.global_approach_speed_mps if self.global_target_mode else None
                ),
            )
            if self._handle_global_approach_route_blocked(self._now_s()):
                return
            if route_arrived:
                self._step_approach_arrival(
                    0.0,
                    self.align_retry_heading_tol_rad,
                    self.direct_nav_omega_max,
                    body_target_visible=self._fresh_approach_body_target() is not None,
                )

        elif self.state == "ALIGN":
            if self._reject_latched_set2_non_target():
                return
            if self._reject_latched_set1_non_target():
                return
            self._step_align()

        elif self.state == "CLASSIFY":
            self._step_classify()

        elif self.state == "PICK":
            if self._pick_base_release_ready():
                self._enter("STORE_IN_TRAY")

        elif self.state == "STORE_IN_TRAY":
            self._do_store_in_tray()

        elif self.state == "WAIT_FOR_STORAGE":
            self._drive(0.0, 0.0, 0.0)

        elif self.state == "DRIVE_TO_STORAGE":
            self._step_drive_to_storage()

        elif self.state == "ALIGN_OVER_BIN":
            self._drive_storage_contact_hold()
            if self._time_in_state() >= self.align_settle_sec:
                self._enter("DUMP_ALL")

        elif self.state == "DUMP_ALL":
            self._drive_storage_contact_hold()
            if not self.dry_pick:
                self._publish_pick(False)   # release / dump tray
            if self._time_in_state() >= self.pick_duration_sec:
                self.get_logger().info("DUMP_ALL complete -> END")
                self._enter("END")

        elif self.state == "END":
            if self._storage_route_completed:
                self._drive_storage_contact_hold()
            else:
                self._drive(0.0, 0.0, 0.0)

    def _arm_phase_fresh(self) -> bool:
        return self._now_s() - self._arm_pick_phase_rx_s <= self.arm_pick_phase_max_age_sec

    def _arm_ready_for_pick(self) -> bool:
        if self.dry_pick or not self._arm_phase_fresh():
            return True
        return self._arm_pick_phase in {"IDLE", "COMPLETE"}

    def _pick_base_release_ready(self) -> bool:
        if (
            self.pick_release_on_lift_enabled
            and not self.dry_pick
            and arm_pick_lift_started(
                self._arm_pick_phase,
                phase_rx_s=self._arm_pick_phase_rx_s,
                pick_enter_s=self.state_enter_s,
                now_s=self._now_s(),
                max_age_sec=self.arm_pick_phase_max_age_sec,
            )
        ):
            self.get_logger().info(
                f"arm phase {self._arm_pick_phase}: release base immediately while arm finishes"
            )
            return True
        return self._time_in_state() >= self.pick_duration_sec

    def _step_wait_for_arm(self) -> None:
        self._drive(0.0, 0.0, 0.0)
        if self._pending_pick is None:
            self._enter("SELECT_TARGET")
            return
        if not self._arm_ready_for_pick():
            self.get_logger().info(
                f"next pick waits for arm IDLE (phase={self._arm_pick_phase})",
                throttle_duration_sec=1.0,
            )
            return
        set_type, label, detail = self._pending_pick
        self._pending_pick = None
        self._commit_pick(set_type, label, detail)

    def _commit_pick(self, set_type: int, label: str, detail: str) -> None:
        """Enter PICK for a confirmed target. In dry_pick mode, log instead of firing the arm."""
        if not self._arm_ready_for_pick():
            self._pending_pick = (int(set_type), str(label), str(detail))
            self._decide(f"WAIT ARM IDLE before PICK set{set_type} {label}")
            self._enter("WAIT_FOR_ARM")
            return
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
        self._set2_identity_latch = None
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
            self._clear_zone_entry_anchor()
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

    def _opening_heading_debug_fresh(self) -> bool:
        return bool(
            self._opening_heading_debug is not None
            and self._opening_heading_debug_s is not None
            and self._now_s() - self._opening_heading_debug_s <= 0.8
        )

    def _opening_position_debug_fresh(self) -> bool:
        return bool(
            self._opening_position_debug is not None
            and self._opening_position_debug_s is not None
            and self._now_s() - self._opening_position_debug_s <= 0.8
        )

    def _finish_opening_initialization(self) -> None:
        self._drive(0.0, 0.0, 0.0)
        self._opening_wall_heading_valid = True
        self._wall_translation_unlocked = True
        # Mapping enable also starts object-flow with a clean first frame in world_model. Until
        # this exact point neither provisional object tracks nor object odometry may affect pose.
        self._set_world_mapping_enabled(True, "opening wall pose initialization complete")
        self._opening_leg = "settle"
        self.state_enter_s = self._now_s()
        self.get_logger().info(
            "opening wall heading + x/y verified -> enable mapping/object-flow and start slot scan"
        )

    def _step_opening(self) -> None:
        """Run the configured opening motion, establish wall pose, then enable object-flow."""
        t = self._time_in_state()
        leg = self._opening_leg
        moving_exploration = bool(
            self.opening_explore_while_moving
            and not self.opening_wall_validation_enabled
            and self.global_target_mode
        )
        if moving_exploration:
            # Global tracks are motion compensated, so collect targets while entering the field.
            # Coverage accounting still ignores OPENING, preventing this scripted transit from
            # falsely marking unexplored patrol nodes as visited.
            self._set_world_mapping_enabled(True, "opening moving exploration")
        elif leg != "settle":
            self._set_world_mapping_enabled(False, "opening base motion")

        if leg == "wait":
            self._drive(0.0, 0.0)
            ready = (self._now_s() - self._node_start_s) >= self.startup_warmup_sec
            if ready:
                turn_direction = "CCW" if self.opening_turn_omega >= 0.0 else "CW"
                self.get_logger().info(
                    "opening start: forward -> right strafe -> "
                    f"{turn_direction} {abs(self.opening_turn_deg):.0f}deg"
                )
                self._opening_leg = "forward"
                self.state_enter_s = self._now_s()
        elif leg == "forward":
            if t < self.opening_forward_sec:
                self._drive(self.opening_speed, 0.0)
            else:
                self._drive(0.0, 0.0)
                self._opening_leg = "strafe_right"
                self.state_enter_s = self._now_s()
        elif leg == "strafe_right":
            if t < self.opening_strafe_right_sec:
                self._drive(0.0, -self.opening_strafe_speed)
            else:
                self._drive(0.0, 0.0)
                heading = float(self.world.robot_theta) if self.world is not None else None
                direction = 1.0 if self.opening_turn_omega >= 0.0 else -1.0
                self._opening_turn_target = (
                    self._wrap_pi(heading + direction * math.radians(abs(self.opening_turn_deg)))
                    if heading is not None
                    else None
                )
                self._opening_leg = "turn_cw"
                self.state_enter_s = self._now_s()
        elif leg == "turn_cw":
            target = self._opening_turn_target
            heading = float(self.world.robot_theta) if self.world is not None else None
            error = (
                self._wrap_pi(target - heading)
                if target is not None and heading is not None
                else None
            )
            reached = error is not None and abs(error) <= self.opening_turn_tolerance_rad
            target_duration = (
                math.radians(abs(self.opening_turn_deg)) / abs(self.opening_turn_omega)
                if abs(self.opening_turn_omega) > 1e-6
                else self.opening_turn_timeout_sec
            )
            timed_out = t >= min(
                self.opening_turn_timeout_sec,
                max(target_duration + 0.5, target_duration * 1.5),
            )
            if reached or timed_out:
                self._drive(0.0, 0.0)
                if timed_out and not reached:
                    self.get_logger().warn(
                        "opening relative CW turn timed out; ending the scripted opening turn"
                    )
                if moving_exploration:
                    self._opening_wall_heading_valid = False
                    self._wall_translation_unlocked = True
                    self._decide("OPENING ENTRY COMPLETE -> SELECT LIVE TARGETS")
                    self.get_logger().info(
                        "opening entry complete -> keep moving map and select live targets "
                        "without a stationary opening scan"
                    )
                    self._enter("SCAN")
                    return
                self._opening_heading_stable_count = 0
                self._opening_heading_checked_seq = self._opening_heading_debug_seq
                self._opening_leg = "heading_observe"
                self.state_enter_s = self._now_s()
            else:
                omega = self.opening_turn_omega
                if error is not None:
                    omega = math.copysign(abs(self.opening_turn_omega), error)
                self._drive(0.0, 0.0, omega)
        elif leg == "heading_observe":
            # HEADING_ONLY mode is published by tick(). It updates the estimated field yaw while
            # deliberately holding x/y and all relative/mapped geometry fixed.
            self._drive(0.0, 0.0, 0.0)
            if not self.opening_wall_validation_enabled:
                if t < self.opening_heading_observe_sec:
                    return
                self._opening_wall_heading_valid = False
                self._wall_translation_unlocked = True
                self._set_world_mapping_enabled(True, "opening wall validation disabled")
                self._opening_leg = "settle"
                self.state_enter_s = self._now_s()
                self.get_logger().info(
                    "opening wall validation disabled -> enable mapping/object-flow"
                )
                return
            if t < self.opening_heading_observe_sec or not self._opening_heading_debug_fresh():
                return
            self._opening_leg = "heading_verify"
            self._opening_heading_stable_count = 0
            self._opening_heading_checked_seq = self._opening_heading_debug_seq
            self.state_enter_s = self._now_s()
        elif leg == "heading_verify":
            self._drive(0.0, 0.0, 0.0)
            if not self._opening_heading_debug_fresh():
                return
            if self._opening_heading_debug_seq == self._opening_heading_checked_seq:
                return
            self._opening_heading_checked_seq = self._opening_heading_debug_seq
            raw_dth, _, segment_count, variance = self._opening_heading_debug
            stddev = math.sqrt(max(0.0, variance))
            wall_valid = (
                segment_count >= self.opening_wall_heading_min_segments
                and stddev <= self.opening_wall_heading_max_stddev_rad
                and abs(raw_dth) <= self.opening_wall_heading_tolerance_rad
            )
            if wall_valid:
                self._opening_heading_stable_count += 1
                if self._opening_heading_stable_count >= self.opening_heading_stable_frames:
                    self._opening_wall_heading_valid = True
                    self._opening_position_stable_count = 0
                    self._opening_position_checked_seq = self._opening_position_debug_seq
                    self._opening_leg = "position_observe"
                    self.state_enter_s = self._now_s()
                    self.get_logger().info(
                        "opening wall heading converged -> start two-axis wall x/y initialization"
                    )
            else:
                self._opening_heading_stable_count = 0
        elif leg == "position_observe":
            # Heading is frozen while fast wall translation pulls the provisional dead-reckoned
            # pose onto the observed field walls. Both axes must be observed and converge.
            self._drive(0.0, 0.0, 0.0)
            if t < self.opening_position_observe_sec or not self._opening_position_debug_fresh():
                return
            if self._opening_position_debug_seq == self._opening_position_checked_seq:
                return
            self._opening_position_checked_seq = self._opening_position_debug_seq
            dx, dy, confidence = self._opening_position_debug
            position_valid = (
                confidence >= self.opening_wall_position_min_confidence
                and abs(dx) <= self.opening_wall_position_tolerance_m
                and abs(dy) <= self.opening_wall_position_tolerance_m
            )
            if position_valid:
                self._opening_position_stable_count += 1
                if self._opening_position_stable_count >= self.opening_position_stable_frames:
                    self._finish_opening_initialization()
            else:
                self._opening_position_stable_count = 0
        else:  # settle / post-opening YOLO scan and t=18 release wait
            self._drive(0.0, 0.0)
            self._set_world_mapping_enabled(True, "post-wall-initialization YOLO/object-flow scan")
            scan_ready = self._relative_try_acquire()
            match_elapsed = self._now_s() - self._node_start_s
            release_ready = (
                self.opening_release_at_sec <= 0.0
                or match_elapsed >= self.opening_release_at_sec
            )
            if t >= self.opening_wait_after_turn_sec and release_ready and scan_ready:
                # The base has remained commanded-stop since the two-second snapshot.  Refresh
                # prior-only/empty geometry here so the t=18 hold itself cannot consume the
                # dead-reckoning budget before the first anchor drive even starts.
                self._relative_anchor_tracker.refresh_trusted_stationary_reference(
                    now_s=self._now_s()
                )
                self.get_logger().info(
                    f"wall-initialized pose + 2s slot scan ready at t={match_elapsed:.1f}s "
                    f"(release_at={self.opening_release_at_sec:.1f}s) -> start anchor driving"
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
        """Align using Body-guided translation only; ALIGN never commands angular motion."""
        now = self._now_s()
        if self._time_in_state() > self.align_timeout_sec:
            self._drive(0.0, 0.0)
            if self.anchor_mission_enabled and self._current_anchor_slot() is not None:
                self._anchor_defer_unconfirmed("align timeout")
                self._enter("ANCHOR_RETURN")
                return
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
                self._drive(self._pulse_vx, self._pulse_vy)
            else:
                self._drive(0.0, 0.0)
                if self.anchor_mission_enabled and self._current_anchor_slot() is not None:
                    # Every physical nudge invalidates the pre-motion pixel measurement.
                    # Keep the latched identity, but require three fresh post-motion frames.
                    self._reset_anchor_body_confirmation(clear_latched=False)
                self._align_phase = "settle"
                self._align_phase_start = now
            return
        if self._align_phase == "settle":
            self._drive(0.0, 0.0)                                # let the heavy base fully stop
            if now - self._align_phase_start >= self.align_settle_pulse_sec:
                self._align_phase = "measure"
            return
        if self._align_phase == "body_lost_lateral_sweep":
            if now - self._align_phase_start < self.align_body_lost_lateral_sweep_sec:
                self._drive(
                    0.0,
                    self._align_body_lost_sweep_strafe_sign
                    * self.align_body_lost_lateral_sweep_duty,
                    0.0,
                )
            else:
                self._drive(0.0, 0.0, 0.0)
                self._align_phase = "body_lost_lateral_settle"
                self._align_phase_start = now
            return
        if self._align_phase == "body_lost_lateral_settle":
            self._drive(0.0, 0.0, 0.0)
            if (
                now - self._align_phase_start
                >= self.align_body_lost_lateral_sweep_settle_sec
            ):
                self._reset_align_body_loss_confirmation()
                self._reset_align_body_presence()
                self._align_phase = "measure"
                self._align_phase_start = now
                self._decide("ALIGN LATERAL RECOVERY COMPLETE -> BODY REMEASURE")
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
                if self.anchor_mission_enabled and self._current_anchor_slot() is not None:
                    # Inspect from the widened view before any forward re-approach.  The
                    # relative lattice already predicts the reverse translation.
                    self._reset_anchor_body_confirmation(clear_latched=True)
                    self._align_phase = "measure"
                    return
                if self._set2_slot_mode() and self._same_slot_reapproach_after_body_lost():
                    return
                if self._set2_slot_mode() and self._current_slot() is not None:
                    self._retry_current_slot("body target lost")
                if self.global_target_mode:
                    # The backoff itself widened the Body view. Re-measure the same physical
                    # target instead of immediately selecting it again through a new route.
                    self._reset_align_body_loss_confirmation()
                    self._reset_align_body_presence()
                    self._align_phase = "measure"
                    self._align_phase_start = now
                    self._decide("ALIGN BODY REACQUIRE AFTER BACKOFF")
                    return
                self._enter("SELECT_TARGET")
            return

        # Measure only while stopped. APPROACH owns heading; ALIGN and every recovery phase
        # command translation only.
        candidate = self._body_target_base(self._body_label())
        if self.anchor_mission_enabled and self._current_anchor_slot() is not None:
            if not self._anchor_note_body_candidate(candidate):
                candidate = None
        # ALIGN is geometry-only. A moving/turning close-range view may temporarily call the
        # intended shape a cube, so label acceptance/rejection is deferred until the base has
        # fully stopped in CLASSIFY. Keep using the nearest slot-compatible Body point
        # for servoing.
        body_point = candidate
        if body_point is None:
            other = self._body_nearest_any()
            if other is not None:
                body_point = (str(other[0]), float(other[1]), float(other[2]), 0.0)

        if self._handle_absent_global_target(now, body_point, context="ALIGN"):
            self._reset_align_body_presence()
            self._align_phase = "measure"
            return

        presence_count, fresh_presence = self._update_align_body_presence(body_point)

        if body_point is None:
            self._drive(0.0, 0.0, 0.0)
            aligned_set2 = self._fresh_aligned_set2_body_target()
            if aligned_set2 is not None:
                label, confidence = aligned_set2
                self._reset_align_body_loss_confirmation()
                self._align_fail_count = 0
                self.get_logger().info(
                    f"ALIGN ok: fresh gripper-bound Body SigLIP '{label}' "
                    f"conf={confidence:.2f} despite transient YOLO loss -> classify"
                )
                self._decide(
                    f"ALIGN BODY SIGLIP {label} {confidence:.2f} AT GRIPPER -> CLASSIFY"
                )
                self._enter("CLASSIFY")
                return
            if (
                self.anchor_mission_enabled
                and self._current_anchor_slot() is not None
                and self._anchor_body_missing_count >= self.body_slot_confirm_frames
            ):
                self._anchor_start_body_backoff_or_defer(now, "current slot absent in Body")
                return
            target_still_recoverable = (
                self._align_target_still_in_world()
                or self._committed_set2_target_can_try_align()
                or self._committed_global_target_can_try_align()
            )
            if self.align_body_lost_backoff_enabled and target_still_recoverable:
                confirmed = self._align_body_loss_confirmed(now)
                if not confirmed:
                    missing_since = self._align_body_missing_since_s
                    elapsed = now - float(now if missing_since is None else missing_since)
                    self.get_logger().info(
                        f"ALIGN: Body missing {self._align_body_missing_frames}/"
                        f"{self.align_body_lost_confirm_frames}, "
                        f"{elapsed:.2f}/{self.align_body_lost_confirm_sec:.2f}s -> HOLD",
                        throttle_duration_sec=0.5,
                    )
                    return
                if (
                    self.global_target_mode
                    and self._align_body_lost_backoff_attempts
                    >= self._align_body_lost_backoff_limit()
                ):
                    if self._start_align_body_lost_lateral_sweep(now):
                        return
                    if self._same_slot_reapproach_after_body_lost():
                        return
                    self._defer_current_global_target("Body still absent after reacquire")
                    self.current_target = None
                    self._enter("SELECT_TARGET")
                    return
                self.get_logger().info(
                    "ALIGN: confirmed Body loss while target remains in world/wide "
                    "-> one straight backoff"
                )
                self._align_body_lost_backoff_attempts += 1
                self._reset_align_body_presence()
                self._reset_align_body_loss_confirmation()
                self._align_phase = "body_lost_backoff"
                self._align_phase_start = now
                return
            self._align_phase = "settle"
            self._align_phase_start = now
            return

        self._reset_align_body_loss_confirmation()
        _, obj_x, obj_y, _ = body_point
        ex = obj_x - self.grab_x
        ey = obj_y - self.grab_y
        if (
            hasattr(self, "_align_arrival_geometry_logged")
            and not self._align_arrival_geometry_logged
        ):
            expected = self._wide_align_expected_base()
            expected_text = (
                f" wide_expected=({expected[0]:.3f},{expected[1]:.3f})"
                if expected is not None
                else " wide_expected=none"
            )
            pose_text = (
                f" pose=({self.world.robot_x:.3f},{self.world.robot_y:.3f},"
                f"{math.degrees(self.world.robot_theta):+.1f}deg)"
                if self.world is not None
                else " pose=none"
            )
            self.get_logger().info(
                f"ALIGN ARRIVAL GEOMETRY body=({obj_x:.3f},{obj_y:.3f}) "
                f"fwd_err={ex * 100:+.1f}cm lat_err={ey * 100:+.1f}cm"
                f"{expected_text}{pose_text}"
            )
            self._align_arrival_geometry_logged = True
        ex_tol = self.align_fwd_tol
        aligned_axis = align_translation_axis(ex, ey, ex_tol, self.align_tol)
        if aligned_axis is None:
            self._drive(0.0, 0.0, 0.0)
            if fresh_presence:
                self.get_logger().info(
                    f"ALIGN BODY PRESENCE {presence_count}/{self.align_body_confirm_frames} "
                    f"fwd_err={ex * 100:+.1f}cm lat_err={ey * 100:+.1f}cm"
                )
            if presence_count < self.align_body_confirm_frames:
                return
            self.get_logger().info(
                f"ALIGN ok: Body x/y translation + presence "
                f"fwd_err={ex * 100:+.1f}cm lat_err={ey * 100:+.1f}cm -> classify"
            )
            self._align_fail_count = 0
            self.siglip_stamp_s = None
            self.shape_stamp_s = None
            self._enter("CLASSIFY")
            return

        if aligned_axis == "lateral":
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
            if obj_x <= self.grab_min_x and vx > 0.0:
                vx = 0.0
        self._reset_align_body_presence()
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
        fresh_body_read = self._attach_fresh_siglip_to_current_slot()

        if self.set2_require_fruit_label and slot.fruit_label:
            if slot.fruit_label != self.set2_label:
                if not fresh_body_read:
                    return
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
            if (
                fresh_body_read
                and tgt is not None
                and float(tgt.confidence) >= self.pick_track_conf
            ):
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

    def _step_classify_set1_final(self) -> None:
        """Verify a stopped, aligned Set1 object before allowing any arm motion."""
        now = self._now_s()
        phase_elapsed = now - self._classify_verify_phase_start_s
        use_wide = self._set1_final_uses_wide()
        source = "Wide" if use_wide else "Body"
        required = (
            int(self.classify_wide_confirm_frames)
            if use_wide
            else int(self.align_distractor_confirm_frames)
        )

        if self._classify_verify_phase == "settle":
            self._drive(0.0, 0.0, 0.0)
            if phase_elapsed >= self.classify_body_settle_sec:
                self._classify_verify_phase = "verify"
                self._classify_verify_phase_start_s = now
                self._reset_set1_final_classification()
                self.get_logger().info(
                    f"CLASSIFY: base settled -> start final {source} verification"
                )
            return

        if self._classify_verify_phase == "backoff":
            if phase_elapsed < self.classify_body_retry_backoff_sec:
                self._drive(-abs(self.align_body_lost_backoff_speed), 0.0, 0.0)
            else:
                self._drive(0.0, 0.0, 0.0)
                self._classify_verify_phase = "backoff_settle"
                self._classify_verify_phase_start_s = now
            return

        if self._classify_verify_phase == "backoff_settle":
            self._drive(0.0, 0.0, 0.0)
            if phase_elapsed >= self.classify_body_retry_settle_sec:
                self._classify_verify_phase = "verify"
                self._classify_verify_phase_start_s = now
                self._reset_set1_final_classification()
                self.get_logger().info(
                    f"CLASSIFY: retry backoff settled -> restart final {source} verification"
                )
            return

        self._drive(0.0, 0.0, 0.0)
        decision, observed_label, fresh_frame = self._update_set1_final_classification()
        if fresh_frame:
            other_best = max(self._classify_body_other_counts.values(), default=0)
            shown = observed_label or "none"
            self.get_logger().info(
                f"CLASSIFY FINAL: {source.lower()}='{shown}' target="
                f"{self._classify_body_target_count}/{required} "
                f"other_best={other_best}/{required} "
                f"frame={self._classify_body_frame_count}/{self.classify_body_max_frames}"
            )

        if decision == "target":
            if use_wide:
                wide_pick = self._wide_align_target_observation()
                confidence = float(wide_pick.confidence) if wide_pick is not None else 0.0
                evidence = (
                    f"final stopped Wide vote {self._classify_body_target_count}/"
                    f"{self._classify_body_frame_count} with Body grab presence "
                    f"conf={confidence:.2f}"
                )
            else:
                body_pick = self._fresh_body_set1_pick_candidate()
                confidence = float(body_pick[1]) if body_pick is not None else 0.0
                evidence = (
                    f"final stopped Body vote {self._classify_body_target_count}/"
                    f"{self._classify_body_frame_count} conf={confidence:.2f}"
                )
            self._commit_pick(
                1,
                self.set1_label,
                evidence,
            )
            return

        if decision == "distractor":
            obj_id = self.current_target.id if self.current_target is not None else 0
            self.get_logger().info(
                f"CLASSIFY FINAL: '{observed_label}' confirmed at grab -> skip target #{obj_id}"
            )
            self._decide(
                f"FINAL NON-TARGET {observed_label} {required}/"
                f"{self._classify_body_frame_count} -> SKIP #{obj_id}"
            )
            self._blacklist(obj_id)
            self.current_target = None
            self.set_type = 0
            self._align_fail_count = 0
            self._enter("SELECT_TARGET")
            return

        verify_timed_out = phase_elapsed >= self.classify_timeout_sec
        if decision != "inconclusive" and not verify_timed_out:
            return

        if self._classify_verify_retry_count == 0:
            self._classify_verify_retry_count = 1
            self._classify_verify_phase = "backoff"
            self._classify_verify_phase_start_s = now
            self.get_logger().info(
                "CLASSIFY FINAL: inconclusive -> short backoff and one stopped retry"
            )
            self._decide("FINAL VERIFY UNCERTAIN -> BACKOFF RETRY")
            return

        obj_id = self.current_target.id if self.current_target is not None else 0
        self.get_logger().warn(
            f"CLASSIFY FINAL: still inconclusive for target #{obj_id} -> "
            "reselect without blacklist"
        )
        self._decide(f"FINAL VERIFY UNCERTAIN -> RESELECT #{obj_id} (no blacklist)")
        self.current_target = None
        self.set_type = 0
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
        # Final Set2 pick authority is the fresh Body crop at the gripper, not the map's current
        # set_type.  A fruit-photo track can regress to set0 or disappear in the close-range blind
        # band after ALIGN even though the physical cube is still centred for pickup.  In that
        # case, accept only a NEW post-CLASSIFY SigLIP result whose encoded Body detection projects
        # to grab_x/grab_y.  Wide hints, stale approach frames, and neighbouring crops cannot pass
        # _fresh_set2_body_verdict().
        direct_body_verdict, direct_body_label, direct_body_confidence = (
            self._fresh_set2_body_verdict()
        )
        if (
            self.set2_require_fruit_label
            and direct_body_verdict == "target"
        ):
            track_state = (
                f"track=#{int(self.current_target.id)} set{int(self.current_target.set_type)}"
                if self.current_target is not None
                else "track=lost"
            )
            self._commit_pick(
                2,
                direct_body_label,
                f"fresh gripper Body SigLIP={direct_body_confidence:.2f} {track_state}",
            )
            return
        set2_visit = bool(
            self._opportunistic_set2_active
            or self.phase == 2
            or (
                self.current_target is not None
                and int(self.current_target.set_type) == 2
            )
        )
        committed_latch = self._latched_set2_identity() if set2_visit else None
        committed_cube = (
            self._fresh_committed_set2_cube_at_grab() if set2_visit else None
        )
        if (
            direct_body_verdict == "pending"
            and committed_cube is not None
            and not (
                committed_latch is not None
                and committed_latch.verdict == "non_target"
            )
        ):
            cube_label, cube_confidence, ex, ey = committed_cube
            target_id = int(self.current_target.id) if self.current_target is not None else 0
            self._commit_pick(
                2,
                self.set2_label,
                    f"committed {self.set2_label} position #{target_id}; fresh Body {cube_label} "
                f"conf={cube_confidence:.2f} grab_err=({ex*100:+.1f},{ey*100:+.1f})cm",
            )
            return
        if self.phase == 1 and not self._opportunistic_set2_active:
            self._step_classify_set1_final()
            return
        if self._reject_latched_set2_non_target():
            return
        latched_identity = committed_latch
        # ALIGN already established that this exact Set1 target was centred at the grab point. Near
        # grab_x the world-model track can temporarily disappear because its calibrated Body map
        # workspace starts farther away than the arm's grab point. In that narrow blind band, accept
        # only a NEW post-ALIGN Body frame that still sees the latched label at the same grab point.
        # This preserves the spatial safety gate without depending on the decaying world-map track.
        body_pick = self._fresh_body_set1_pick_candidate()
        if body_pick is not None:
            label, confidence, ex, ey = body_pick
            self._commit_pick(
                1,
                label,
                f"fresh body conf={confidence:.2f} "
                f"grab_err=({ex*100:+.1f},{ey*100:+.1f})cm",
            )
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
            tgt = (
                self._target_for_set2_latch(latched_identity)
                if latched_identity is not None
                else self._front_target(2)
            )
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
        # A wide/body-transit label may route us here, but it is NEVER pick authority.
        # Only a fresh post-ALIGN Body crop whose encoded pixel is at the gripper can approve
        # Set2. This is intentionally independent of the public world-model fruit_label.
        if direct_body_verdict != "pending":
            body_verdict = direct_body_verdict
            body_fruit_label = direct_body_label
            body_fruit_confidence = direct_body_confidence
        elif latched_identity is not None:
            body_verdict = latched_identity.verdict
            body_fruit_label = latched_identity.label
            body_fruit_confidence = latched_identity.confidence
        else:
            body_verdict, body_fruit_label, body_fruit_confidence = (
                self._fresh_set2_body_verdict()
            )
        set2_ok = set2_pick_allowed(
            tgt,
            pick_track_confidence=self.pick_track_conf,
            require_fruit_label=self.set2_require_fruit_label,
            body_verdict=body_verdict,
        )
        set2_pick_label = (
            body_fruit_label
            if body_verdict == "target"
            else "fruit_photo_cube"
        )

        if self._opportunistic_set2_active:
            if set2_ok:
                self._commit_pick(
                    2,
                    set2_pick_label,
                    f"spatial Body SigLIP={body_fruit_confidence:.2f} "
                    f"track={tgt.confidence:.2f}",
                )
                return
            if self.set2_require_fruit_label and body_verdict == "non_target":
                self.get_logger().info(
                    f"CLASSIFY: fresh Body fruit '{body_fruit_label}' "
                    f"!= {self.set2_label} -> reject"
                )
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
                detail = (
                    f"spatial Body SigLIP={body_fruit_confidence:.2f}"
                    if self.set2_require_fruit_label
                    else "fruit cube test"
                )
                self._commit_pick(2, set2_pick_label, f"track conf={tgt.confidence:.2f} ({detail})")
                return
            if set1_ok:
                self._skip_target(f"stray set1 '{tgt.class_label}' during Set2 phase")
                return
            # Reject only from a fresh, spatially-bound Body read. A wide routing hint is
            # deliberately positive-only and can never blacklist a possibly correct target.
            if self.set2_require_fruit_label and body_verdict == "non_target":
                self.get_logger().info(
                    f"CLASSIFY: fresh Body fruit '{body_fruit_label}' "
                    f"!= {self.set2_label} -> skip now"
                )
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
        running = self._run_started and self._competition_state == "RUNNING"
        if running:
            self._maybe_start_timed_storage()
            handled_stuck_escape = self._step_stuck_escape()
            if not handled_stuck_escape:
                self._update_stuck_watchdog()
                handled_stuck_escape = self._step_stuck_escape()
            if not handled_stuck_escape:
                self._step()
            if self.state not in {"OPENING", "WALL_INIT"}:
                self._set_world_mapping_enabled(True)
        else:
            self._reset_stuck_escape()
            self._drive(0.0, 0.0, 0.0)
            self._set_world_mapping_enabled(False, "competition not RUNNING")
        self.pub_mapping_enabled.publish(Bool(data=bool(self._world_mapping_enabled)))
        self._publish_track_birth_gate()
        self.pub_wall_fast.publish(
            Bool(data=bool(running and self._wall_fast_correction_requested()))
        )
        wall_mode = self._wall_correction_mode_requested() if running else "OFF"
        self.pub_wall_mode.publish(String(data=wall_mode))
        self.pub_wall_processing.publish(
            Bool(data=bool(running and self._wall_processing_requested()))
        )
        self._publish_match_time()

        # 2) publish mission state every tick
        msg = MissionState()
        msg.state = (
            LOCAL_ANCHOR_ALIGN_STATE
            if self.state == LOCAL_ANCHOR_STATE
            and self._local_anchor_fsm.state.startswith("ALIGN")
            else self.state
        )
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
