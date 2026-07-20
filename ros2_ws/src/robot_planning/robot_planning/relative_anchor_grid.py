"""Pose-free three-anchor tracking from wide-camera base-frame object observations.

The field map is intentionally not used here.  A two-second stationary acquisition
fits observations to a twelve-slot 50 cm virtual lattice in ``base_link``.  This can
bootstrap from K2/K3 observations when K1 is empty.  As the robot moves, current wide
detections update every anchor/slot through a rigid transform in the *current*
``base_link`` frame.

Empty and picked slots remain virtual landmarks in the template, so removing an
object never moves an anchor centre.  With no current landmark the last estimate is
retained only until ``stale_timeout_s``; callers must stop after that timeout.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _rotate(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


@dataclass(frozen=True)
class RelativeObservation:
    """One current wide-camera object position in ``base_link`` metres."""

    x: float
    y: float
    label: str = ""
    set_type: int = 0
    confidence: float = 0.0


@dataclass(frozen=True)
class RelativeSlotEvidence:
    """Latest accumulated identity evidence associated with one virtual slot."""

    anchor_id: int
    slot_index: int
    x: float
    y: float
    label: str
    set_type: int
    confidence: float
    hits: int
    last_seen_s: float


@dataclass(frozen=True)
class RelativeGridEstimate:
    """Current transform quality and active-anchor geometry in ``base_link``."""

    anchor_id: int
    center_x: float
    center_y: float
    slots: tuple[tuple[float, float], ...]
    matched_count: int
    residual_m: float
    age_s: float
    navigable: bool
    reason: str


@dataclass
class _Cluster:
    x: float
    y: float
    label: str
    set_type: int
    confidence: float
    hits: int = 1
    last_seen_s: float = 0.0


class RelativeAnchorGridTracker:
    """Track K1/K2/K3 as a local virtual lattice without a field-frame pose."""

    _OFFSETS = ((-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5), (0.5, 0.5))

    def __init__(
        self,
        *,
        spacing_m: float = 0.50,
        anchor_step_m: float = 1.0,
        route_unit_base: tuple[float, float] = (1.0, 0.0),
        association_radius_m: float = 0.22,
        cluster_radius_m: float = 0.10,
        stale_timeout_s: float = 0.60,
        dead_reckon_timeout_s: float = 12.0,
        prediction_stale_timeout_s: float = 0.60,
        smoothing_alpha: float = 0.65,
    ) -> None:
        self.spacing_m = max(1e-6, float(spacing_m))
        self.anchor_step_m = float(anchor_step_m)
        ux, uy = float(route_unit_base[0]), float(route_unit_base[1])
        un = math.hypot(ux, uy)
        if un <= 1e-9:
            raise ValueError("route_unit_base must be non-zero")
        self.route_unit = (ux / un, uy / un)
        self.association_radius = max(0.01, float(association_radius_m))
        self.cluster_radius = max(0.01, float(cluster_radius_m))
        self.stale_timeout = max(0.0, float(stale_timeout_s))
        self.dead_reckon_timeout = max(self.stale_timeout, float(dead_reckon_timeout_s))
        self.prediction_stale_timeout = max(
            0.05, float(prediction_stale_timeout_s)
        )
        self.alpha = min(1.0, max(0.0, float(smoothing_alpha)))

        self._clusters: list[_Cluster] = []
        self._locked = False
        self._template_slots: dict[tuple[int, int], tuple[float, float]] = {}
        self._template_centers: dict[int, tuple[float, float]] = {}
        # Rigid transform from acquisition base frame -> current base frame.
        self._yaw = 0.0
        self._tx = 0.0
        self._ty = 0.0
        self._last_update_s: float | None = None
        self._last_prediction_s: float | None = None
        self._last_frame_s: float | None = None
        self._matched_count = 0
        self._residual_m = math.inf
        self._evidence: dict[tuple[int, int], RelativeSlotEvidence] = {}
        self._excluded_slots: set[tuple[int, int]] = set()

    @property
    def locked(self) -> bool:
        return self._locked

    @property
    def acquisition_cluster_count(self) -> int:
        return len(self._clusters)

    def reset(self) -> None:
        self.__init__(
            spacing_m=self.spacing_m,
            anchor_step_m=self.anchor_step_m,
            route_unit_base=self.route_unit,
            association_radius_m=self.association_radius,
            cluster_radius_m=self.cluster_radius,
            stale_timeout_s=self.stale_timeout,
            dead_reckon_timeout_s=self.dead_reckon_timeout,
            prediction_stale_timeout_s=self.prediction_stale_timeout,
            smoothing_alpha=self.alpha,
        )

    def set_excluded_slots(self, slots: Iterable[tuple[int, int]]) -> None:
        """Exclude active/picked objects from rigid-fit landmarks without deleting slots."""
        excluded: set[tuple[int, int]] = set()
        for anchor_id, slot_index in slots:
            key = (int(anchor_id), int(slot_index))
            if key in self._template_slots or not self._locked:
                excluded.add(key)
        self._excluded_slots = excluded

    def predict_robot_motion(
        self,
        *,
        dx_body: float,
        dy_body: float,
        dtheta: float,
        now_s: float,
    ) -> bool:
        """Propagate stationary landmarks after a measured/commanded robot-frame delta.

        ``dx_body``/``dy_body`` describe robot translation in the old base frame and
        ``dtheta`` is the robot's CCW heading change.  The landmark transform therefore
        receives the inverse SE(2) motion.  Wide observations remain the correction source.
        """
        if not self._locked:
            return False
        try:
            dx = float(dx_body)
            dy = float(dy_body)
            yaw_delta = float(dtheta)
            now = float(now_s)
        except (TypeError, ValueError):
            return False
        if not all(math.isfinite(value) for value in (dx, dy, yaw_delta, now)):
            return False
        # One prediction update should be a short control/localizer interval, never a pose jump.
        if math.hypot(dx, dy) > 0.20 or abs(yaw_delta) > math.radians(30.0):
            return False
        self._yaw = _wrap(self._yaw - yaw_delta)
        self._tx, self._ty = _rotate(
            self._tx - dx,
            self._ty - dy,
            -yaw_delta,
        )
        self._last_prediction_s = now
        return True

    def refresh_trusted_stationary_reference(self, *, now_s: float) -> bool:
        """Refresh prior-only geometry after a commanded stationary hold without moving it."""
        if not self._locked:
            return False
        now = float(now_s)
        if not math.isfinite(now):
            return False
        self._last_update_s = now
        self._last_prediction_s = now
        return True

    @staticmethod
    def _clean(observations: Iterable[RelativeObservation]) -> list[RelativeObservation]:
        clean: list[RelativeObservation] = []
        for obs in observations:
            try:
                x, y = float(obs.x), float(obs.y)
                conf = float(obs.confidence)
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(conf)):
                continue
            if int(obs.set_type) not in {1, 2}:
                continue
            clean.append(
                RelativeObservation(x, y, str(obs.label), int(obs.set_type), conf)
            )
        # Suppress duplicate YOLO boxes for one physical object in the same frame.
        deduped: list[RelativeObservation] = []
        for obs in sorted(clean, key=lambda value: value.confidence, reverse=True):
            if any(math.hypot(obs.x - old.x, obs.y - old.y) < 0.07 for old in deduped):
                continue
            deduped.append(obs)
        return deduped

    def observe(self, observations: Iterable[RelativeObservation], *, now_s: float) -> None:
        """Consume one camera frame.  Before lock it builds stationary clusters."""
        now = float(now_s)
        obs = self._clean(observations)
        self._last_frame_s = now
        if not self._locked:
            used: set[int] = set()
            for value in obs:
                best_i = None
                best_d = self.cluster_radius
                for i, cluster in enumerate(self._clusters):
                    if i in used:
                        continue
                    distance = math.hypot(value.x - cluster.x, value.y - cluster.y)
                    if distance <= best_d:
                        best_i, best_d = i, distance
                if best_i is None:
                    self._clusters.append(
                        _Cluster(
                            value.x,
                            value.y,
                            value.label,
                            value.set_type,
                            value.confidence,
                            last_seen_s=now,
                        )
                    )
                    used.add(len(self._clusters) - 1)
                    continue
                used.add(best_i)
                cluster = self._clusters[best_i]
                n = cluster.hits + 1
                cluster.x = (cluster.x * cluster.hits + value.x) / n
                cluster.y = (cluster.y * cluster.hits + value.y) / n
                cluster.hits = n
                cluster.last_seen_s = now
                if value.confidence >= cluster.confidence:
                    cluster.label = value.label
                    cluster.set_type = value.set_type
                    cluster.confidence = value.confidence
            return
        self._update_locked(obs, now)

    def _offsets(self) -> tuple[tuple[float, float], ...]:
        return tuple(
            (sx * self.spacing_m, sy * self.spacing_m)
            for sx, sy in self._OFFSETS
        )

    @staticmethod
    def _greedy_assign(
        predicted: dict[tuple[int, int], tuple[float, float]],
        observations: list[RelativeObservation],
        radius: float,
    ) -> list[tuple[tuple[int, int], int, float]]:
        candidates: list[tuple[float, tuple[int, int], int]] = []
        for key, (px, py) in predicted.items():
            for index, obs in enumerate(observations):
                distance = math.hypot(px - obs.x, py - obs.y)
                if distance <= radius:
                    candidates.append((distance, key, index))
        assigned_keys: set[tuple[int, int]] = set()
        assigned_obs: set[int] = set()
        result: list[tuple[tuple[int, int], int, float]] = []
        for distance, key, index in sorted(candidates):
            if key in assigned_keys or index in assigned_obs:
                continue
            assigned_keys.add(key)
            assigned_obs.add(index)
            result.append((key, index, distance))
        return result

    @staticmethod
    def _square_equivalent_near(yaw: float, reference: float) -> float:
        return min(
            (yaw + k * math.pi * 0.5 for k in range(-4, 5)),
            key=lambda value: abs(_wrap(value - reference)),
        )

    def lock(
        self,
        *,
        expected_k1_center: tuple[float, float],
        expected_grid_yaw: float = 0.0,
        min_cluster_hits: int = 2,
        min_points: int = 2,
        center_gate_m: float = 0.80,
        allow_prior_fallback: bool = False,
        now_s: float = 0.0,
    ) -> bool:
        """Fit K1 and freeze a twelve-slot local template.

        The expected centre resolves the unavoidable two-adjacent-point ambiguity.  Normally it
        is only a coarse startup prior; ``allow_prior_fallback`` promotes the trusted opening pose
        to a virtual-grid reference when fewer than two physical objects exist.
        """
        if self._locked:
            return True
        qualified_clusters = [
            c
            for c in self._clusters
            if c.hits >= max(1, int(min_cluster_hits))
            and float(now_s) - c.last_seen_s <= self.stale_timeout
        ]
        observations = [
            RelativeObservation(c.x, c.y, c.label, c.set_type, c.confidence)
            for c in qualified_clusters
        ]
        required_points = max(2, int(min_points))
        if len(observations) < required_points:
            if not allow_prior_fallback:
                return False
            # The opening endpoint and heading define the lattice even when some or all physical
            # slots are empty.  Build all twelve virtual points from that trusted prior and attach
            # any stable observations that do exist; the remaining K1 slots stay explicitly empty.
            expected_x, expected_y = map(float, expected_k1_center)
            yaw = float(expected_grid_yaw)
            route_dx = self.route_unit[0] * self.anchor_step_m
            route_dy = self.route_unit[1] * self.anchor_step_m
            self._template_centers = {
                anchor_id: (
                    expected_x + (anchor_id - 1) * route_dx,
                    expected_y + (anchor_id - 1) * route_dy,
                )
                for anchor_id in (1, 2, 3)
            }
            self._template_slots = {}
            rotated_offsets = [_rotate(x, y, yaw) for x, y in self._offsets()]
            for anchor_id, center in self._template_centers.items():
                for index, (ox, oy) in enumerate(rotated_offsets, start=1):
                    self._template_slots[(anchor_id, index)] = (
                        center[0] + ox,
                        center[1] + oy,
                    )
            assigned = self._greedy_assign(
                self._template_slots, observations, self.association_radius
            )
            self._locked = True
            self._yaw = self._tx = self._ty = 0.0
            self._last_update_s = float(now_s)
            self._last_prediction_s = float(now_s)
            self._matched_count = len(assigned)
            self._residual_m = (
                math.sqrt(
                    sum(distance * distance for _, _, distance in assigned) / len(assigned)
                )
                if assigned else 0.0
            )
            for key, obs_index, _ in assigned:
                self._record_evidence(
                    key,
                    observations[obs_index],
                    float(now_s),
                    initial_hits=qualified_clusters[obs_index].hits,
                )
            return True

        offsets = self._offsets()
        yaw_candidates = [float(expected_grid_yaw)]
        for i, first in enumerate(observations):
            for second in observations[i + 1 :]:
                observed_angle = math.atan2(second.y - first.y, second.x - first.x)
                for oi, offset_a in enumerate(offsets):
                    for offset_b in offsets[oi + 1 :]:
                        template_angle = math.atan2(
                            offset_b[1] - offset_a[1], offset_b[0] - offset_a[0]
                        )
                        yaw_candidates.append(observed_angle - template_angle)
                        yaw_candidates.append(observed_angle - template_angle + math.pi)

        expected_x, expected_y = map(float, expected_k1_center)
        best = None
        for raw_yaw in yaw_candidates:
            yaw = self._square_equivalent_near(raw_yaw, float(expected_grid_yaw))
            rotated = [_rotate(x, y, yaw) for x, y in offsets]
            route_dx = self.route_unit[0] * self.anchor_step_m
            route_dy = self.route_unit[1] * self.anchor_step_m
            relative_slots = {
                (anchor_id, slot_index): (
                    (anchor_id - 1) * route_dx + ox,
                    (anchor_id - 1) * route_dy + oy,
                )
                for anchor_id in (1, 2, 3)
                for slot_index, (ox, oy) in enumerate(rotated, start=1)
            }
            for obs in observations:
                # The visible points do not have to belong to K1.  Matching against all twelve
                # virtual slots lets an empty K1 bootstrap from K2/K3 objects that the fisheye can
                # already see, while the expected K1 prior resolves the periodic-grid ambiguity.
                for _, (rel_x, rel_y) in relative_slots.items():
                    center = (obs.x - rel_x, obs.y - rel_y)
                    prior = math.hypot(center[0] - expected_x, center[1] - expected_y)
                    if prior > max(0.05, float(center_gate_m)):
                        continue
                    predicted = {
                        key: (center[0] + point[0], center[1] + point[1])
                        for key, point in relative_slots.items()
                    }
                    assigned = self._greedy_assign(
                        predicted, observations, self.association_radius
                    )
                    if len(assigned) < max(2, int(min_points)):
                        continue
                    residual = math.sqrt(
                        sum(distance * distance for _, _, distance in assigned) / len(assigned)
                    )
                    score = (len(assigned), -residual, -prior)
                    if best is None or score > best[0]:
                        best = (score, center, yaw, predicted, assigned)
        if best is None:
            return False

        _, center, yaw, _, assigned = best
        route_dx = self.route_unit[0] * self.anchor_step_m
        route_dy = self.route_unit[1] * self.anchor_step_m
        self._template_centers = {
            anchor_id: (
                center[0] + (anchor_id - 1) * route_dx,
                center[1] + (anchor_id - 1) * route_dy,
            )
            for anchor_id in (1, 2, 3)
        }
        self._template_slots = {}
        rotated_offsets = [_rotate(x, y, yaw) for x, y in offsets]
        for anchor_id, anchor_center in self._template_centers.items():
            for index, (ox, oy) in enumerate(rotated_offsets, start=1):
                self._template_slots[(anchor_id, index)] = (
                    anchor_center[0] + ox,
                    anchor_center[1] + oy,
                )

        self._locked = True
        self._yaw = self._tx = self._ty = 0.0
        self._last_update_s = float(now_s)
        self._last_prediction_s = float(now_s)
        self._matched_count = len(assigned)
        self._residual_m = -best[0][1]
        for key, obs_index, _ in assigned:
            self._record_evidence(
                key,
                observations[obs_index],
                float(now_s),
                initial_hits=qualified_clusters[obs_index].hits,
            )
        return True

    def _transform(self, point: tuple[float, float]) -> tuple[float, float]:
        x, y = _rotate(point[0], point[1], self._yaw)
        return x + self._tx, y + self._ty

    def _current_predicted(self) -> dict[tuple[int, int], tuple[float, float]]:
        return {
            key: self._transform(point)
            for key, point in self._template_slots.items()
            if key not in self._excluded_slots
        }

    @staticmethod
    def _rigid_fit(
        source: list[tuple[float, float]], target: list[tuple[float, float]]
    ) -> tuple[float, float, float, float]:
        sx = sum(p[0] for p in source) / len(source)
        sy = sum(p[1] for p in source) / len(source)
        tx = sum(p[0] for p in target) / len(target)
        ty = sum(p[1] for p in target) / len(target)
        a = b = 0.0
        for (px, py), (qx, qy) in zip(source, target):
            ux, uy = px - sx, py - sy
            vx, vy = qx - tx, qy - ty
            a += ux * vx + uy * vy
            b += ux * vy - uy * vx
        yaw = math.atan2(b, a)
        rsx, rsy = _rotate(sx, sy, yaw)
        return yaw, tx - rsx, ty - rsy, 0.0

    def _update_locked(self, observations: list[RelativeObservation], now_s: float) -> None:
        predicted = self._current_predicted()
        assigned = self._greedy_assign(predicted, observations, self.association_radius)
        if not assigned:
            self._matched_count = 0
            return

        if len(assigned) == 1:
            key, index, _ = assigned[0]
            template = self._template_slots[key]
            rx, ry = _rotate(template[0], template[1], self._yaw)
            new_yaw = self._yaw
            new_tx = observations[index].x - rx
            new_ty = observations[index].y - ry
        else:
            source = [self._template_slots[key] for key, _, _ in assigned]
            target = [(observations[index].x, observations[index].y) for _, index, _ in assigned]
            new_yaw, new_tx, new_ty, _ = self._rigid_fit(source, target)
            new_yaw = self._square_equivalent_near(new_yaw, self._yaw)

        # Reject a single bad frame instead of teleporting the entire virtual lattice.
        translation_jump = math.hypot(new_tx - self._tx, new_ty - self._ty)
        yaw_jump = abs(_wrap(new_yaw - self._yaw))
        translation_limit = 0.12 if len(assigned) == 1 else 0.30
        if translation_jump > translation_limit or yaw_jump > math.radians(35.0):
            self._matched_count = 0
            return
        fit_errors = []
        for key, index, _ in assigned:
            px, py = _rotate(*self._template_slots[key], new_yaw)
            fit_errors.append(
                math.hypot(
                    px + new_tx - observations[index].x,
                    py + new_ty - observations[index].y,
                )
            )
        fit_residual = math.sqrt(
            sum(error * error for error in fit_errors) / len(fit_errors)
        )
        if len(assigned) >= 2 and fit_residual > 0.10:
            self._matched_count = 0
            return
        alpha = self.alpha
        self._yaw = self._yaw + alpha * _wrap(new_yaw - self._yaw)
        self._tx = (1.0 - alpha) * self._tx + alpha * new_tx
        self._ty = (1.0 - alpha) * self._ty + alpha * new_ty
        self._last_update_s = now_s
        self._last_prediction_s = now_s
        self._matched_count = len(assigned)
        self._residual_m = math.sqrt(
            sum(distance * distance for _, _, distance in assigned) / len(assigned)
        )
        for key, index, _ in assigned:
            self._record_evidence(key, observations[index], now_s)

    def _record_evidence(
        self,
        key: tuple[int, int],
        observation: RelativeObservation,
        now_s: float,
        *,
        initial_hits: int = 1,
    ) -> None:
        previous = self._evidence.get(key)
        hits = max(1, int(initial_hits)) if previous is None else previous.hits + 1
        label = observation.label
        set_type = observation.set_type
        confidence = observation.confidence
        if previous is not None and previous.confidence > confidence:
            label = previous.label
            set_type = previous.set_type
            confidence = previous.confidence
        x, y = self._transform(self._template_slots[key])
        self._evidence[key] = RelativeSlotEvidence(
            key[0], key[1], x, y, label, set_type, confidence, hits, now_s
        )

    def anchor_point(self, anchor_id: int) -> tuple[float, float] | None:
        point = self._template_centers.get(int(anchor_id))
        return None if point is None else self._transform(point)

    def slot_point(self, anchor_id: int, slot_index: int) -> tuple[float, float] | None:
        point = self._template_slots.get((int(anchor_id), int(slot_index)))
        return None if point is None else self._transform(point)

    def slot_evidence(self, anchor_id: int, slot_index: int) -> RelativeSlotEvidence | None:
        evidence = self._evidence.get((int(anchor_id), int(slot_index)))
        if evidence is None:
            return None
        point = self.slot_point(anchor_id, slot_index)
        if point is None:
            return None
        return RelativeSlotEvidence(
            evidence.anchor_id,
            evidence.slot_index,
            point[0],
            point[1],
            evidence.label,
            evidence.set_type,
            evidence.confidence,
            evidence.hits,
            evidence.last_seen_s,
        )

    def evidence_for_anchor(
        self, anchor_id: int, *, min_hits: int = 1
    ) -> tuple[RelativeSlotEvidence, ...]:
        result = []
        for slot_index in range(1, 5):
            evidence = self.slot_evidence(anchor_id, slot_index)
            if evidence is not None and evidence.hits >= max(1, int(min_hits)):
                result.append(evidence)
        return tuple(result)

    def estimate(self, anchor_id: int, *, now_s: float) -> RelativeGridEstimate:
        center = self.anchor_point(anchor_id)
        slots = tuple(
            point
            for slot_index in range(1, 5)
            if (point := self.slot_point(anchor_id, slot_index)) is not None
        )
        if not self._locked or center is None or len(slots) != 4:
            return RelativeGridEstimate(
                int(anchor_id), 0.0, 0.0, (), 0, math.inf, math.inf, False, "not_locked"
            )
        age = math.inf if self._last_update_s is None else max(0.0, now_s - self._last_update_s)
        prediction_age = (
            math.inf
            if self._last_prediction_s is None
            else max(0.0, now_s - self._last_prediction_s)
        )
        visual_fresh = age <= self.stale_timeout
        dead_reckoning = (
            age <= self.dead_reckon_timeout
            and prediction_age <= self.prediction_stale_timeout
        )
        navigable = visual_fresh or dead_reckoning
        if visual_fresh:
            reason = "ok"
        elif dead_reckoning:
            reason = "dead_reckoning"
        else:
            reason = "stale_relative_landmarks"
        return RelativeGridEstimate(
            int(anchor_id), center[0], center[1], slots, self._matched_count,
            self._residual_m, age, navigable, reason
        )
