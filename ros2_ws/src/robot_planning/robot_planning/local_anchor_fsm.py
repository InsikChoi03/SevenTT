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
    turn_omega: float = 0.07
    turn_pulse_sec: float = 0.12
    turn_settle_sec: float = 0.70
    turn_tolerance_rad: float = math.radians(3.0)
    max_turn_pulses: int = 60
    scan_observe_sec: float = 1.00
    single_lap_inspection: bool = True
    initial_inventory_observe_sec: float = 2.00
    face_observe_sec: float = 2.00
    candidate_radius_min_m: float = 0.12
    candidate_radius_max_m: float = 0.40
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
    classify_stable_frames: int = 3
    classify_timeout_sec: float = 8.0
    classify_hold_on_failure: bool = True
    target_fruit_label: str = "banana"
    enable_align: bool = False
    grab_x_m: float = 0.19
    grab_y_m: float = 0.0
    grab_min_x_m: float = 0.17
    align_fwd_tolerance_m: float = 0.02
    align_lateral_tolerance_m: float = 0.02
    align_fwd_duty: float = 0.27
    align_fwd_pulse_sec: float = 0.15
    align_strafe_duty: float = 0.315
    align_strafe_pulse_sec: float = 0.30
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
    ) -> None:
        self._turn_target = wrap_angle(target)
        self._turn_target_unwrapped = unwrapped_target
        self._turn_return_state = return_state
        self._turn_pulses = 0
        self._pulse_sign = 0.0
        self._enter("TURN_MEASURE", now, detail)

    def _begin_pulse(self, now: float, error: float) -> None:
        if self._turn_pulses >= max(1, int(self.config.max_turn_pulses)):
            self.fault(now, f"turn did not converge at target {self._turn_target:.3f} rad")
            return
        self._turn_pulses += 1
        self._pulse_sign = math.copysign(1.0, error)
        self._enter("TURN_PULSE", now, self.detail)

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
    ) -> None:
        """Accumulate stable SigLIP results for the currently faced candidate."""
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
        if self._class_hits >= max(1, int(self.config.classify_stable_frames)):
            if candidate is not None:
                candidate.fruit_label = clean
                candidate.status = (
                    "TARGET_FRUIT"
                    if clean == self.config.target_fruit_label.strip().lower()
                    else "NON_TARGET_FRUIT"
                )
            self._advance_candidate(now)

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
            if target is not None:
                target.status = "PICKED_DRY"
            self._enter("COMPLETE", now, "target aligned; dry pick complete")
            return
        # Move one axis per pulse.  Normalised excess makes unlike tolerances comparable.
        fwd_excess = max(0.0, abs(ex) - cfg.align_fwd_tolerance_m)
        lat_excess = max(0.0, abs(ey) - cfg.align_lateral_tolerance_m)
        if lat_excess >= fwd_excess:
            self._align_pulse = MotionCommand(vy=math.copysign(cfg.align_strafe_duty, ey))
            self._align_pulse_sec = cfg.align_strafe_pulse_sec
        else:
            vx = math.copysign(cfg.align_fwd_duty, ex)
            if x < cfg.grab_min_x_m and vx > 0.0:
                self.fault(now, f"unsafe forward pulse blocked at target x={x:.3f}m")
                return
            self._align_pulse = MotionCommand(vx=vx)
            self._align_pulse_sec = cfg.align_fwd_pulse_sec
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
            if self._turn_target_unwrapped is None:
                error = angle_error(self._turn_target, current_yaw)
            else:
                error = self._turn_target_unwrapped - self._unwrapped_yaw
            if abs(error) <= cfg.turn_tolerance_rad:
                self._enter(self._turn_return_state, now, self.detail)
            else:
                self._begin_pulse(now, error)
            return MotionCommand()

        if self.state == "TURN_PULSE":
            if now - self.state_enter_s < cfg.turn_pulse_sec:
                return MotionCommand(omega=self._pulse_sign * abs(cfg.turn_omega))
            self._enter("TURN_SETTLE", now, self.detail)
            return MotionCommand()

        if self.state == "TURN_SETTLE":
            if now - self.state_enter_s >= cfg.turn_settle_sec:
                self._enter("TURN_MEASURE", now, self.detail)
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
