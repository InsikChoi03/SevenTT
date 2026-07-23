"""
ROS-independent local-anchor fruit inspection state machine.

The robot is assumed to be stopped at a freshly-created local anchor.  The FSM
only rotates while it builds and classifies the nearby fruit-cube inventory.
Optional translation pulses are reserved for one final dry ALIGN trial, after
all candidates have been inspected, so scan quality and ALIGN quality can be
tuned independently.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable


TAU = 2.0 * math.pi
TERMINAL_STATES = {"COMPLETE", "ABORTED", "FAULT"}


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % TAU - math.pi


def angle_error(target: float, current: float) -> float:
    """Return the shortest signed rotation from current to target."""
    return wrap_angle(float(target) - float(current))


def integrate_imu_yaw(current: float, physical_delta: float) -> float:
    """Accumulate a physical IMU yaw increment without pose/encoder attenuation."""
    return wrap_angle(float(current) + float(physical_delta))


@dataclass(frozen=True)
class LocalObservation:
    """One object observation expressed in the anchor frame."""

    x: float
    y: float
    label: str
    confidence: float


@dataclass
class Candidate:
    """Merged local inventory entry."""

    candidate_id: int
    x: float
    y: float
    label: str
    confidence: float
    hits: int = 1
    status: str = "UNINSPECTED"
    fruit_label: str = ""
    classification_hits: int = 0
    _label_scores: dict[str, float] = field(default_factory=dict, repr=False)
    _label_max_confidence: dict[str, float] = field(default_factory=dict, repr=False)

    @property
    def radius(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing(self) -> float:
        return math.atan2(self.y, self.x)


@dataclass(frozen=True)
class MotionCommand:
    """Command requested by the pure FSM for this tick."""

    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    request_pick: bool = False


@dataclass
class LocalAnchorConfig:
    """Parameters intentionally mirrored by local_anchor_test.yaml."""

    scan_step_rad: float = math.radians(45.0)
    scan_positions: int = 8
    turn_omega: float = 0.10
    turn_slow_omega: float = 0.07
    turn_slowdown_rad: float = math.radians(10.0)
    turn_pulse_sec: float = 0.24
    turn_burst_pause_sec: float = 0.18
    turn_settle_sec: float = 0.70
    turn_verify_sec: float = 0.60
    turn_verify_max_corrections: int = 4
    turn_correction_pulse_sec: float = 0.10
    turn_correction_settle_sec: float = 0.35
    turn_tolerance_rad: float = math.radians(5.0)
    max_turn_pulses: int = 60
    scan_observe_sec: float = 1.00
    single_lap_inspection: bool = True
    initial_inventory_observe_sec: float = 2.00
    face_observe_sec: float = 0.75
    candidate_radius_min_m: float = 0.12
    candidate_radius_max_m: float = 0.50
    candidate_merge_radius_m: float = 0.14
    candidate_merge_bearing_rad: float = math.radians(12.0)
    candidate_merge_radial_m: float = 0.10
    candidate_min_hits: int = 2
    inventory_max_candidates: int = 4
    fruit_cube_override_confidence: float = 0.60
    expected_candidates_min: int = 3
    expected_candidates_max: int = 4
    fruit_inspection_labels: tuple[str, ...] = (
        "fruit_photo_cube",
        "apple",
        "orange",
        "banana",
        "pineapple",
    )
    classify_min_confidence: float = 0.08
    classify_stable_frames: int = 2
    classify_timeout_sec: float = 4.0
    classify_hold_on_failure: bool = True
    visual_heading_enabled: bool = True
    visual_heading_min_confidence: float = 0.60
    visual_heading_center_x_px: float = 320.0
    visual_heading_center_tolerance_px: float = 40.0
    visual_heading_confirm_frames: int = 3
    visual_heading_coarse_gate_rad: float = math.radians(20.0)
    visual_heading_max_correction_rad: float = math.radians(20.0)
    target_fruit_label: str = "banana"
    enable_align: bool = False
    enable_pick: bool = False
    pick_duration_sec: float = 9.0
    grab_x_m: float = 0.19
    grab_y_m: float = 0.0
    grab_min_x_m: float = 0.17
    align_fwd_tolerance_m: float = 0.02
    align_lateral_tolerance_m: float = 0.02
    align_fwd_duty: float = 0.27
    align_fwd_pulse_sec: float = 0.15
    align_strafe_duty: float = 0.315
    align_strafe_pulse_sec: float = 0.30
    align_adaptive_steps_enabled: bool = True
    align_mid_error_m: float = 0.06
    align_fwd_mid_pulse_sec: float = 0.25
    align_strafe_mid_pulse_sec: float = 0.60
    align_settle_sec: float = 0.60
    align_target_timeout_sec: float = 2.0
    align_timeout_sec: float = 12.0


def efficient_candidate_order(
    candidates: Iterable[Candidate], current_yaw: float
) -> list[int]:
    """
    Order bearings along the shortest open arc that contains every candidate.

    The largest empty angular gap is excluded.  The two possible directions along
    the remaining arc are scored from the current yaw, and the cheaper direction
    is returned.  This prevents alternating left/right turns between candidates.
    """
    items = list(candidates)
    if len(items) <= 1:
        return [item.candidate_id for item in items]

    ordered = sorted(items, key=lambda item: item.bearing % TAU)
    angles = [item.bearing % TAU for item in ordered]
    gaps = [
        ((angles[(idx + 1) % len(angles)] - angles[idx]) % TAU, idx)
        for idx in range(len(angles))
    ]
    largest_gap, gap_start = max(gaps, key=lambda pair: pair[0])
    first = (gap_start + 1) % len(ordered)
    ccw = [ordered[(first + offset) % len(ordered)] for offset in range(len(ordered))]
    cw = list(reversed(ccw))
    span = TAU - largest_gap

    def cost(route: list[Candidate]) -> float:
        initial = abs(angle_error(route[0].bearing, current_yaw))
        return initial + span

    route = ccw if cost(ccw) <= cost(cw) else cw
    return [item.candidate_id for item in route]


class LocalAnchorFruitFsm:
    """Deterministic test FSM; hardware and ROS safety remain in its wrapper."""

    def __init__(self, config: LocalAnchorConfig) -> None:
        self.config = config
        self.state = "IDLE"
        self.state_enter_s = 0.0
        self.detail = "waiting for START"
        self.candidates: list[Candidate] = []
        self.route: list[int] = []
        self.route_index = 0
        self.scan_index = 0
        self.scan_targets: list[float] = []
        self._next_candidate_id = 1
        self._turn_target = 0.0
        self._turn_target_unwrapped: float | None = None
        self._turn_return_state = ""
        self._turn_pulses = 0
        self._pulse_sign = 0.0
        self._pulse_duration_sec = 0.0
        self._settle_duration_sec = 0.0
        self._settle_return_state = "TURN_MEASURE"
        self._visual_turn_hint = 0.0
        self._visual_correction_attempts = 0
        self._class_label = ""
        self._class_hits = 0
        self._align_target_id: int | None = None
        self._align_last_xy: tuple[float, float] | None = None
        self._align_last_seen_s = -math.inf
        self._align_start_s = 0.0
        self._align_pulse: MotionCommand = MotionCommand()
        self._align_pulse_sec = 0.0
        self._last_wrapped_yaw = 0.0
        self._unwrapped_yaw = 0.0
        # Camera-derived correction is deliberately separate from physical IMU lap
        # travel.  It may refine candidate-facing turns, but never shortens 360 deg.
        self._heading_correction = 0.0
        self._turn_uses_heading_correction = False
        self._visual_center_hits = 0
        self._visual_lock_count = 0

    @property
    def active_candidate(self) -> Candidate | None:
        if not self.route or self.route_index >= len(self.route):
            return None
        candidate_id = self.route[self.route_index]
        return next((c for c in self.candidates if c.candidate_id == candidate_id), None)

    def _candidate_by_id(self, candidate_id: int | None) -> Candidate | None:
        return next((c for c in self.candidates if c.candidate_id == candidate_id), None)

    def _enter(self, state: str, now: float, detail: str = "") -> None:
        self.state = state
        self.state_enter_s = float(now)
        if detail:
            self.detail = detail

    def complete_pick_on_lift(self, now: float) -> bool:
        """Release the base as soon as the external arm sequencer reaches LIFT."""
        if self.state != "PICK_WAIT":
            return False
        target = self._candidate_by_id(self._align_target_id)
        if target is not None:
            target.status = "PICKED"
        self._enter("COMPLETE", now, "arm reached LIFT; base released while arm finishes")
        return True

    def start(self, now: float, current_yaw: float = 0.0) -> None:
        """Reset all per-run state and begin the local inventory run."""
        self.candidates = []
        self.route = []
        self.route_index = 0
        self.scan_index = 0
        self._next_candidate_id = 1
        self._align_target_id = None
        self._align_last_xy = None
        self._align_last_seen_s = -math.inf
        current_yaw = wrap_angle(current_yaw)
        self._last_wrapped_yaw = current_yaw
        self._unwrapped_yaw = 0.0
        self._heading_correction = 0.0
        self._turn_uses_heading_correction = False
        self._visual_center_hits = 0
        self._visual_lock_count = 0
        self._turn_target_unwrapped = None
        if self.config.single_lap_inspection:
            self.scan_targets = [current_yaw]
            self._enter(
                "INITIAL_INVENTORY",
                now,
                "stationary wide inventory before one clockwise lap",
            )
            return
        step = abs(float(self.config.scan_step_rad))
        # Clockwise scan: REP-103 positive yaw is CCW.
        # Start at the first step and include the final 360-degree return.  The old
        # [0, 45, ..., 315] sequence observed the initial heading without moving and
        # then relied on candidate-facing turns, which made a scan look like several laps.
        self.scan_targets = [
            wrap_angle(float(current_yaw) - (idx + 1) * step)
            for idx in range(max(1, int(self.config.scan_positions)))
        ]
        self._begin_turn(self.scan_targets[0], "SCAN_OBSERVE", now, "scan position 1")

    def abort(self, now: float, reason: str) -> None:
        self._enter("ABORTED", now, reason)

    def fault(self, now: float, reason: str) -> None:
        self._enter("FAULT", now, reason)

    def _begin_turn(
        self,
        target: float,
        return_state: str,
        now: float,
        detail: str,
        *,
        unwrapped_target: float | None = None,
        use_heading_correction: bool = False,
    ) -> None:
        self._turn_target = wrap_angle(target)
        self._turn_target_unwrapped = unwrapped_target
        self._turn_return_state = return_state
        self._turn_uses_heading_correction = bool(use_heading_correction)
        self._turn_pulses = 0
        self._pulse_sign = 0.0
        self._pulse_duration_sec = 0.0
        self._settle_duration_sec = 0.0
        self._settle_return_state = "TURN_MEASURE"
        self._visual_turn_hint = 0.0
        self._visual_correction_attempts = 0
        self._visual_center_hits = 0
        self._enter("TURN_MEASURE", now, detail)

    def _begin_pulse(
        self,
        now: float,
        error: float,
        *,
        duration_sec: float,
        settle_sec: float,
        settle_return_state: str,
        detail: str,
    ) -> None:
        if self._turn_pulses >= max(1, int(self.config.max_turn_pulses)):
            self.fault(now, f"turn did not converge at target {self._turn_target:.3f} rad")
            return
        self._turn_pulses += 1
        self._pulse_sign = math.copysign(1.0, error)
        self._pulse_duration_sec = max(0.0, float(duration_sec))
        self._settle_duration_sec = max(0.0, float(settle_sec))
        self._settle_return_state = str(settle_return_state)
        self._enter("TURN_PULSE", now, detail)

    def _begin_burst_pulse(self, now: float, error: float) -> None:
        self._begin_pulse(
            now,
            error,
            duration_sec=self.config.turn_pulse_sec,
            settle_sec=self.config.turn_burst_pause_sec,
            settle_return_state="TURN_MEASURE",
            detail="rapid turn pulse",
        )

    def _begin_correction_pulse(self, now: float, error: float) -> None:
        self._begin_pulse(
            now,
            error,
            duration_sec=self.config.turn_correction_pulse_sec,
            settle_sec=self.config.turn_correction_settle_sec,
            settle_return_state="TURN_VERIFY",
            detail="turn correction pulse",
        )

    def _turn_error(self, current_yaw: float) -> float:
        if self._turn_target_unwrapped is None:
            measured_yaw = current_yaw
            if self._turn_uses_heading_correction:
                measured_yaw = wrap_angle(measured_yaw + self._heading_correction)
            return angle_error(self._turn_target, measured_yaw)
        measured_yaw = self._unwrapped_yaw
        if self._turn_uses_heading_correction:
            measured_yaw += self._heading_correction
        return self._turn_target_unwrapped - measured_yaw

    def _requires_visual_verify(self) -> bool:
        return (
            self.config.visual_heading_enabled
            and self._turn_uses_heading_correction
            and self._turn_return_state == "FACE_SETTLE"
            and self.active_candidate is not None
        )

    def _begin_visual_verify(self, now: float, detail: str) -> None:
        self._enter("TURN_VERIFY", now, detail)

    def _update_unwrapped_yaw(self, current_yaw: float) -> None:
        delta = angle_error(current_yaw, self._last_wrapped_yaw)
        self._unwrapped_yaw += delta
        self._last_wrapped_yaw = current_yaw

    def add_observations(self, observations: Iterable[LocalObservation]) -> None:
        """Merge observations only while stopped at a scan position."""
        if self.state not in {"SCAN_OBSERVE", "INITIAL_INVENTORY"}:
            return
        cfg = self.config
        for obs in observations:
            radius = math.hypot(float(obs.x), float(obs.y))
            if not (cfg.candidate_radius_min_m <= radius <= cfg.candidate_radius_max_m):
                continue
            best = None
            best_dist = float(cfg.candidate_merge_radius_m)
            for candidate in self.candidates:
                dist = math.hypot(candidate.x - obs.x, candidate.y - obs.y)
                bearing_close = abs(
                    angle_error(math.atan2(obs.y, obs.x), candidate.bearing)
                ) <= cfg.candidate_merge_bearing_rad
                radial_close = abs(radius - candidate.radius) <= cfg.candidate_merge_radial_m
                if (dist <= best_dist or (bearing_close and radial_close)) and (
                    best is None or dist < best_dist
                ):
                    best = candidate
                    best_dist = dist
            confidence = max(0.0, float(obs.confidence))
            if best is None:
                candidate = Candidate(
                    candidate_id=self._next_candidate_id,
                    x=float(obs.x),
                    y=float(obs.y),
                    label=str(obs.label),
                    confidence=confidence,
                )
                label = str(obs.label).strip().lower()
                candidate.label = label
                candidate._label_scores[label] = confidence
                candidate._label_max_confidence[label] = confidence
                self._next_candidate_id += 1
                self.candidates.append(candidate)
                continue
            total = float(best.hits + 1)
            best.x = (best.x * best.hits + float(obs.x)) / total
            best.y = (best.y * best.hits + float(obs.y)) / total
            best.hits += 1
            best.confidence = max(best.confidence, confidence)
            label = str(obs.label).strip().lower()
            best._label_scores[label] = best._label_scores.get(label, 0.0) + confidence
            best._label_max_confidence[label] = max(
                confidence,
                best._label_max_confidence.get(label, 0.0),
            )
            self._select_candidate_label(best)

    @staticmethod
    def _label_scores(candidate: Candidate) -> dict[str, float]:
        if candidate._label_scores:
            return dict(candidate._label_scores)
        return {candidate.label: max(0.01, candidate.confidence) * candidate.hits}

    @staticmethod
    def _label_max_confidences(candidate: Candidate) -> dict[str, float]:
        if candidate._label_max_confidence:
            return dict(candidate._label_max_confidence)
        return {candidate.label.strip().lower(): candidate.confidence}

    def _select_candidate_label(self, candidate: Candidate) -> None:
        """Prefer a confident fruit-cube view over ambiguous plain-cube votes."""
        scores = self._label_scores(candidate)
        maxima = self._label_max_confidences(candidate)
        has_cube = "cube" in scores
        fruit_confidence = maxima.get("fruit_photo_cube", 0.0)
        if has_cube and fruit_confidence >= self.config.fruit_cube_override_confidence:
            candidate.label = "fruit_photo_cube"
            candidate.confidence = max(candidate.confidence, fruit_confidence)
            return
        candidate.label = max(scores, key=scores.get)

    def _merge_candidate(self, target: Candidate, source: Candidate) -> None:
        """Merge two post-scan clusters using hit-weighted anchor coordinates."""
        scores = self._label_scores(target)
        source_scores = self._label_scores(source)
        maxima = self._label_max_confidences(target)
        source_maxima = self._label_max_confidences(source)
        total_hits = target.hits + source.hits
        target.x = (target.x * target.hits + source.x * source.hits) / total_hits
        target.y = (target.y * target.hits + source.y * source.hits) / total_hits
        target.hits = total_hits
        target.confidence = max(target.confidence, source.confidence)
        for label, score in source_scores.items():
            scores[label] = scores.get(label, 0.0) + score
        for label, confidence in source_maxima.items():
            maxima[label] = max(maxima.get(label, 0.0), confidence)
        target._label_scores = scores
        target._label_max_confidence = maxima
        self._select_candidate_label(target)

    def _consolidate_candidates(self, candidates: list[Candidate]) -> list[Candidate]:
        """Run a stronger one-shot NMS after the single 360-degree scan."""
        cfg = self.config
        merged: list[Candidate] = []
        for candidate in sorted(candidates, key=lambda item: item.hits, reverse=True):
            match = None
            match_distance = float("inf")
            for existing in merged:
                distance = math.hypot(existing.x - candidate.x, existing.y - candidate.y)
                bearing_close = abs(
                    angle_error(existing.bearing, candidate.bearing)
                ) <= cfg.candidate_merge_bearing_rad
                radial_close = (
                    abs(existing.radius - candidate.radius) <= cfg.candidate_merge_radial_m
                )
                if distance <= cfg.candidate_merge_radius_m or (
                    bearing_close and radial_close
                ):
                    if distance < match_distance:
                        match = existing
                        match_distance = distance
            if match is None:
                merged.append(candidate)
            else:
                self._merge_candidate(match, candidate)

        maximum = max(0, int(cfg.inventory_max_candidates))
        if maximum > 0 and len(merged) > maximum:
            merged = sorted(
                merged,
                key=lambda item: (item.hits, item.confidence),
                reverse=True,
            )[:maximum]
        merged.sort(key=lambda item: item.bearing)
        for new_id, candidate in enumerate(merged, start=1):
            candidate.candidate_id = new_id
        return merged

    def note_classification(
        self,
        label: str,
        confidence: float,
        face_visible: bool,
        now: float,
        *,
        is_target: bool = False,
    ) -> None:
        """Consume a fresh SigLIP result for the currently faced candidate.

        The upstream ``is_target`` gate already requires today's fruit label, a
        sufficient score margin, and a visible fruit face.  One such result is
        enough to begin the reversible ALIGN motion; ordinary target-label and
        non-target results retain the configured stable-frame accumulation.
        """
        if self.state != "CLASSIFY" or not face_visible:
            return
        if float(confidence) < float(self.config.classify_min_confidence):
            return
        clean = str(label).strip().lower()
        if not clean:
            return
        if clean == self._class_label:
            self._class_hits += 1
        else:
            self._class_label = clean
            self._class_hits = 1
        candidate = self.active_candidate
        if candidate is not None:
            candidate.classification_hits = self._class_hits
        target_label = self.config.target_fruit_label.strip().lower()
        if (
            bool(is_target)
            and clean == target_label
            and candidate is not None
            and self.config.enable_align
        ):
            candidate.fruit_label = clean
            candidate.status = "TARGET_FRUIT"
            self._begin_target_align(candidate, now)
            return
        if self._class_hits >= max(1, int(self.config.classify_stable_frames)):
            if candidate is not None:
                candidate.fruit_label = clean
                candidate.status = (
                    "TARGET_FRUIT"
                    if clean == target_label
                    else "NON_TARGET_FRUIT"
                )
                if candidate.status == "TARGET_FRUIT" and self.config.enable_align:
                    self._begin_target_align(candidate, now)
                    return
            self._advance_candidate(now)

    def _begin_target_align(self, candidate: Candidate, now: float) -> None:
        """Stop candidate inspection and immediately align the first target fruit."""
        self._align_target_id = candidate.candidate_id
        self._align_start_s = float(now)
        self._align_last_xy = None
        self._align_last_seen_s = -math.inf
        self.route = [candidate.candidate_id]
        self.route_index = 0
        candidate.status = "ALIGNING"
        self._enter(
            "ALIGN_SETTLE_INITIAL",
            now,
            f"target fruit {candidate.candidate_id} confirmed; begin ALIGN",
        )

    def note_visual_fruit_center(
        self,
        x_center_px: float | None,
        confidence: float,
        now: float,
        current_yaw: float,
        turn_hint_rad: float = 0.0,
    ) -> bool:
        """Lock a fruit-facing heading from stable body-camera centre observations.

        The visual lock is only an alternative completion condition for a current
        candidate-facing turn.  The raw IMU revolution counter is never modified.
        """
        cfg = self.config
        eligible = (
            cfg.visual_heading_enabled
            and self._turn_uses_heading_correction
            and self._turn_return_state == "FACE_SETTLE"
            and self.state
            in {
                "TURN_MEASURE",
                "TURN_CONTINUOUS",
                "TURN_VERIFY",
                "TURN_PULSE",
                "TURN_SETTLE",
            }
            and self.active_candidate is not None
            and x_center_px is not None
            and math.isfinite(float(x_center_px))
            and float(confidence) >= cfg.visual_heading_min_confidence
        )
        if not eligible:
            self._visual_center_hits = 0
            return False

        if math.isfinite(float(turn_hint_rad)) and abs(float(turn_hint_rad)) > 1e-6:
            self._visual_turn_hint = float(turn_hint_rad)

        current_yaw = wrap_angle(current_yaw)
        if self._turn_target_unwrapped is None:
            corrected = wrap_angle(current_yaw + self._heading_correction)
            coarse_error = angle_error(self._turn_target, corrected)
        else:
            corrected = self._unwrapped_yaw + self._heading_correction
            coarse_error = self._turn_target_unwrapped - corrected
        centered = abs(float(x_center_px) - cfg.visual_heading_center_x_px) <= (
            cfg.visual_heading_center_tolerance_px
        )
        if not centered or abs(coarse_error) > cfg.visual_heading_coarse_gate_rad:
            self._visual_center_hits = 0
            return False

        if self.state != "TURN_VERIFY":
            self._begin_visual_verify(now, "body centre reached; stopped for 3-frame verify")
        self._visual_center_hits += 1
        if self._visual_center_hits < max(1, int(cfg.visual_heading_confirm_frames)):
            return False

        if self._turn_target_unwrapped is None:
            proposed = angle_error(self._turn_target, current_yaw)
        else:
            proposed = self._turn_target_unwrapped - self._unwrapped_yaw
        delta = proposed - self._heading_correction
        if abs(delta) > cfg.visual_heading_max_correction_rad:
            self._visual_center_hits = 0
            return False

        self._heading_correction = proposed
        self._visual_lock_count += 1
        candidate = self.active_candidate
        candidate_id = candidate.candidate_id if candidate is not None else 0
        self._enter(
            "FACE_SETTLE",
            now,
            f"body-centered candidate {candidate_id}; visual heading locked",
        )
        return True

    def note_align_target(self, x: float, y: float, now: float) -> None:
        """Provide the latest body-camera target point in base_link metres."""
        if not self.state.startswith("ALIGN"):
            return
        if math.isfinite(x) and math.isfinite(y):
            self._align_last_xy = (float(x), float(y))
            self._align_last_seen_s = float(now)

    def _freeze_inventory(self, now: float, current_yaw: float) -> None:
        minimum = max(1, int(self.config.candidate_min_hits))
        stable = [candidate for candidate in self.candidates if candidate.hits >= minimum]
        self.candidates = self._consolidate_candidates(stable)
        fruit_labels = {label.strip().lower() for label in self.config.fruit_inspection_labels}
        fruit_candidates = []
        for candidate in self.candidates:
            if candidate.label.strip().lower() in fruit_labels:
                fruit_candidates.append(candidate)
            else:
                candidate.status = "IGNORED_NON_FRUIT"
        if self.config.single_lap_inspection:
            # Visit every fruit monotonically clockwise from the anchor heading.
            # A positive bearing is therefore reached near the end of the lap,
            # never by reversing counter-clockwise across zero.
            fruit_candidates.sort(key=lambda item: (-item.bearing) % TAU)
            self.route = [item.candidate_id for item in fruit_candidates]
        else:
            self.route = efficient_candidate_order(fruit_candidates, current_yaw)
        self.route_index = 0
        if not self.candidates:
            if self.config.single_lap_inspection:
                self._complete_single_lap(now, "no stable candidates")
            else:
                self._enter("COMPLETE", now, "360 scan complete: no stable candidates")
            return
        if not self.route:
            detail = f"{len(self.candidates)} objects, no fruit candidates"
            if self.config.single_lap_inspection:
                self._complete_single_lap(now, detail)
            else:
                self._enter("COMPLETE", now, f"360 scan complete: {detail}")
            return
        self._face_current_candidate(now)

    def _face_current_candidate(self, now: float) -> None:
        candidate = self.active_candidate
        if candidate is None:
            self._finish_inspection(now)
            return
        candidate.status = "FACING"
        self._class_label = ""
        self._class_hits = 0
        unwrapped_target = None
        if self.config.single_lap_inspection:
            clockwise_travel = (-candidate.bearing) % TAU
            unwrapped_target = -clockwise_travel
        self._begin_turn(
            candidate.bearing,
            "FACE_SETTLE",
            now,
            f"face candidate {candidate.candidate_id}",
            unwrapped_target=unwrapped_target,
            use_heading_correction=True,
        )

    def _advance_candidate(self, now: float) -> None:
        self.route_index += 1
        if self.route_index >= len(self.route):
            self._finish_inspection(now)
        else:
            self._face_current_candidate(now)

    def _finish_inspection(self, now: float) -> None:
        targets = [
            candidate
            for candidate in self.candidates
            if candidate.status == "TARGET_FRUIT"
        ]
        if self.config.single_lap_inspection:
            self._complete_single_lap(
                now,
                f"inspection complete: {len(targets)} target fruit(s)",
            )
            return
        if not self.config.enable_align or not targets:
            self._enter(
                "COMPLETE",
                now,
                f"inspection complete: {len(targets)} target fruit(s)",
            )
            return
        target = min(targets, key=lambda candidate: candidate.radius)
        self._align_target_id = target.candidate_id
        self._align_start_s = float(now)
        self._begin_turn(
            target.bearing,
            "ALIGN_SETTLE_INITIAL",
            now,
            f"face align target {target.candidate_id}",
        )

    def _complete_single_lap(self, now: float, result: str) -> None:
        """Finish at exactly one clockwise revolution from the start heading."""
        self._begin_turn(
            0.0,
            "LAP_COMPLETE",
            now,
            f"finish one 360-degree lap: {result}",
            unwrapped_target=-TAU,
        )

    def _start_align_pulse(self, now: float, x: float, y: float) -> None:
        cfg = self.config
        ex = float(x) - cfg.grab_x_m
        ey = float(y) - cfg.grab_y_m
        if abs(ex) <= cfg.align_fwd_tolerance_m and abs(ey) <= cfg.align_lateral_tolerance_m:
            target = self._candidate_by_id(self._align_target_id)
            if cfg.enable_pick:
                if target is not None:
                    target.status = "PICKING"
                self._enter("PICK_TRIGGER", now, "target aligned; trigger real pick")
            else:
                if target is not None:
                    target.status = "PICKED_DRY"
                self._enter("COMPLETE", now, "target aligned; dry pick complete")
            return
        # Move one axis per pulse.  Normalised excess makes unlike tolerances comparable.
        fwd_excess = max(0.0, abs(ex) - cfg.align_fwd_tolerance_m)
        lat_excess = max(0.0, abs(ey) - cfg.align_lateral_tolerance_m)
        if lat_excess >= fwd_excess:
            self._align_pulse = MotionCommand(vy=math.copysign(cfg.align_strafe_duty, ey))
            self._align_pulse_sec = (
                cfg.align_strafe_mid_pulse_sec
                if cfg.align_adaptive_steps_enabled and abs(ey) >= cfg.align_mid_error_m
                else cfg.align_strafe_pulse_sec
            )
        else:
            vx = math.copysign(cfg.align_fwd_duty, ex)
            if x < cfg.grab_min_x_m and vx > 0.0:
                self.fault(now, f"unsafe forward pulse blocked at target x={x:.3f}m")
                return
            self._align_pulse = MotionCommand(vx=vx)
            self._align_pulse_sec = (
                cfg.align_fwd_mid_pulse_sec
                if cfg.align_adaptive_steps_enabled and abs(ex) >= cfg.align_mid_error_m
                else cfg.align_fwd_pulse_sec
            )
        self._enter("ALIGN_PULSE", now, "align unit pulse")

    def tick(self, now: float, current_yaw: float) -> MotionCommand:
        """Advance the FSM and return the requested base command."""
        now = float(now)
        current_yaw = wrap_angle(current_yaw)
        self._update_unwrapped_yaw(current_yaw)
        cfg = self.config
        if self.state in {"IDLE", *TERMINAL_STATES}:
            return MotionCommand()

        if self.state == "TURN_MEASURE":
            error = self._turn_error(current_yaw)
            if abs(error) <= cfg.turn_tolerance_rad and not self._requires_visual_verify():
                self._enter(self._turn_return_state, now, self.detail)
            elif abs(error) <= cfg.turn_slowdown_rad:
                self._begin_visual_verify(now, "inside 10deg gate; stopped for IMU/body verify")
            else:
                self._begin_burst_pulse(now, error)
            return MotionCommand()

        if self.state == "TURN_VERIFY":
            if now - self.state_enter_s < cfg.turn_verify_sec:
                return MotionCommand()
            imu_error = self._turn_error(current_yaw)
            if abs(imu_error) > cfg.turn_tolerance_rad:
                self._begin_correction_pulse(now, imu_error)
                return MotionCommand()
            if not self._requires_visual_verify():
                self._enter(self._turn_return_state, now, self.detail)
                return MotionCommand()
            if self._visual_correction_attempts >= max(
                0, int(cfg.turn_verify_max_corrections)
            ):
                self._enter(
                    self._turn_return_state,
                    now,
                    "visual verify exhausted; accepting IMU heading",
                )
                return MotionCommand()
            error = self._visual_turn_hint
            if abs(error) <= 1e-6:
                error = self._turn_error(current_yaw)
            if abs(error) <= 1e-6:
                error = self._pulse_sign if abs(self._pulse_sign) > 0.0 else -1.0
            self._visual_correction_attempts += 1
            self._begin_correction_pulse(now, error)
            return MotionCommand()

        if self.state == "TURN_PULSE":
            if now - self.state_enter_s < self._pulse_duration_sec:
                return MotionCommand(omega=self._pulse_sign * abs(cfg.turn_omega))
            self._enter("TURN_SETTLE", now, self.detail)
            return MotionCommand()

        if self.state == "TURN_SETTLE":
            if now - self.state_enter_s >= self._settle_duration_sec:
                if self._settle_return_state == "TURN_VERIFY":
                    self._begin_visual_verify(now, "settled after precision correction")
                else:
                    self._enter("TURN_MEASURE", now, "measure after rapid pulse")
            return MotionCommand()

        if self.state == "INITIAL_INVENTORY":
            if now - self.state_enter_s >= cfg.initial_inventory_observe_sec:
                self._freeze_inventory(now, current_yaw)
            return MotionCommand()

        if self.state == "SCAN_OBSERVE":
            if now - self.state_enter_s >= cfg.scan_observe_sec:
                self.scan_index += 1
                if self.scan_index >= len(self.scan_targets):
                    self._freeze_inventory(now, current_yaw)
                else:
                    self._begin_turn(
                        self.scan_targets[self.scan_index],
                        "SCAN_OBSERVE",
                        now,
                        f"scan position {self.scan_index + 1}",
                    )
            return MotionCommand()

        if self.state == "FACE_SETTLE":
            if now - self.state_enter_s >= cfg.face_observe_sec:
                self._class_label = ""
                self._class_hits = 0
                self._enter("CLASSIFY", now, "waiting for stable fruit classification")
            return MotionCommand()

        if self.state == "CLASSIFY":
            if now - self.state_enter_s >= cfg.classify_timeout_sec:
                candidate = self.active_candidate
                if candidate is not None:
                    candidate.status = "WAITING_CLASSIFICATION"
                    self.detail = (
                        f"hold candidate {candidate.candidate_id}: "
                        f"SigLIP unresolved for {now - self.state_enter_s:.1f}s"
                    )
                if not cfg.classify_hold_on_failure:
                    self._advance_candidate(now)
            return MotionCommand()

        if self.state == "LAP_COMPLETE":
            self.scan_index = len(self.scan_targets)
            self._enter("COMPLETE", now, self.detail)
            return MotionCommand()

        if self.state == "ALIGN_SETTLE_INITIAL":
            if now - self.state_enter_s >= cfg.align_settle_sec:
                self._align_start_s = now
                self._enter("ALIGN_MEASURE", now, "waiting for body target")
            return MotionCommand()

        if self.state == "ALIGN_MEASURE":
            if now - self._align_start_s >= cfg.align_timeout_sec:
                self.fault(now, "ALIGN timeout")
                return MotionCommand()
            if now - self._align_last_seen_s > cfg.align_target_timeout_sec:
                if now - self.state_enter_s > cfg.align_target_timeout_sec:
                    self.fault(now, "body target lost during ALIGN")
                return MotionCommand()
            if self._align_last_xy is not None:
                self._start_align_pulse(now, *self._align_last_xy)
            return MotionCommand()

        if self.state == "ALIGN_PULSE":
            if now - self.state_enter_s < self._align_pulse_sec:
                return self._align_pulse
            self._enter("ALIGN_SETTLE", now, "settle after align pulse")
            return MotionCommand()

        if self.state == "ALIGN_SETTLE":
            if now - self.state_enter_s >= cfg.align_settle_sec:
                self._align_last_xy = None
                self._align_last_seen_s = -math.inf
                self._enter("ALIGN_MEASURE", now, "remeasure body target")
            return MotionCommand()

        if self.state == "PICK_TRIGGER":
            self._enter("PICK_WAIT", now, "real pick sequence running; base locked")
            return MotionCommand(request_pick=True)

        if self.state == "PICK_WAIT":
            if now - self.state_enter_s >= cfg.pick_duration_sec:
                target = self._candidate_by_id(self._align_target_id)
                if target is not None:
                    target.status = "PICKED"
                self._enter("COMPLETE", now, "one target fruit picked; test complete")
            return MotionCommand()

        self.fault(now, f"unknown state {self.state}")
        return MotionCommand()

    def summary(self) -> dict[str, object]:
        """Return JSON-serialisable diagnostics for logs and the status topic."""
        count = len(self.candidates)
        if self.config.single_lap_inspection:
            scan_completed_deg = min(360.0, abs(math.degrees(self._unwrapped_yaw)))
        else:
            scan_completed_deg = (
                min(self.scan_index, len(self.scan_targets))
                * math.degrees(abs(self.config.scan_step_rad))
            )
        turn_target = (
            self._turn_target
            if self._turn_target_unwrapped is None
            else self._turn_target_unwrapped
        )
        return {
            "state": self.state,
            "detail": self.detail,
            "scan_index": self.scan_index,
            "scan_positions": len(self.scan_targets),
            "scan_completed_deg": round(scan_completed_deg, 1),
            "route": list(self.route),
            "route_index": self.route_index,
            "turn_target_deg": round(math.degrees(turn_target), 2),
            "turn_pulses": self._turn_pulses,
            "heading_correction_deg": round(math.degrees(self._heading_correction), 2),
            "visual_center_hits": self._visual_center_hits,
            "visual_lock_count": self._visual_lock_count,
            "visual_correction_attempts": self._visual_correction_attempts,
            "candidate_radius_max_m": self.config.candidate_radius_max_m,
            "active_candidate": (
                self.active_candidate.candidate_id if self.active_candidate is not None else None
            ),
            "inventory_count": count,
            "inventory_count_ok": (
                self.config.expected_candidates_min
                <= count
                <= self.config.expected_candidates_max
            ),
            "expected_candidates": [
                self.config.expected_candidates_min,
                self.config.expected_candidates_max,
            ],
            "candidates": [
                {
                    "id": candidate.candidate_id,
                    "x": round(candidate.x, 4),
                    "y": round(candidate.y, 4),
                    "radius": round(candidate.radius, 4),
                    "bearing_deg": round(math.degrees(candidate.bearing), 2),
                    "label": candidate.label,
                    "confidence": round(candidate.confidence, 3),
                    "hits": candidate.hits,
                    "status": candidate.status,
                    "fruit_label": candidate.fruit_label,
                    "classification_hits": candidate.classification_hits,
                }
                for candidate in self.candidates
            ],
        }
