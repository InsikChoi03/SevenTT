"""Fixed three-anchor object-slot inventory for the main arena mission.

The inventory is deliberately ROS-free.  It owns twelve immutable field positions:
four 50 cm-grid object slots around each of K1, K2, and K3.  An anchor may be
snapshotted exactly once.  During that snapshot, observations are matched one-to-one
to the nearest fixed slots inside ``snapshot_radius_m``; matched slots become
``OCCUPIED`` and every unmatched slot at that anchor becomes ``EMPTY``.  Later
observations cannot create slots or change the locked snapshot.

Body inspection failures are held as ``UNCONFIRMED`` outside the normal queue.
After all other occupied slots are processed, each anchor may requeue those held
slots exactly once; a second failure remains visible but cannot create a loop.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Iterable, Mapping


UNMAPPED = "UNMAPPED"
EMPTY = "EMPTY"
OCCUPIED = "OCCUPIED"
UNCONFIRMED = "UNCONFIRMED"
CHECKED = "CHECKED"
PICKED = "PICKED"

SLOT_STATES = frozenset({UNMAPPED, EMPTY, OCCUPIED, UNCONFIRMED, CHECKED, PICKED})

# (anchor_id, anchor_x, anchor_y, ((slot_x, slot_y), ...))
# Slot order is stable and IDs are assigned K1.S1..K3.S4 -> 1..12.
ANCHOR_LAYOUT = (
    (1, -1.25, 0.75, ((-1.50, 0.50), (-1.00, 0.50), (-1.50, 1.00), (-1.00, 1.00))),
    (2, -1.25, -0.25, ((-1.50, -0.50), (-1.00, -0.50), (-1.50, 0.00), (-1.00, 0.00))),
    (3, -1.25, -1.25, ((-1.50, -1.50), (-1.00, -1.50), (-1.50, -1.00), (-1.00, -1.00))),
)


@dataclass(frozen=True)
class Anchor:
    """Fixed anchor position plus its one-shot snapshot state."""

    anchor_id: int
    x: float
    y: float
    snapshot_locked: bool = False
    snapshot_time_s: float | None = None
    final_retry_started: bool = False


@dataclass(frozen=True)
class SlotObservation:
    """ROS-independent field observation accepted by ``lock_anchor_snapshot``."""

    x: float
    y: float
    track_id: int = 0
    set_type: int = 0
    class_label: str = ""
    fruit_label: str = ""
    confidence: float = 0.0


@dataclass(frozen=True)
class AnchorSlot:
    """One fixed field slot.

    Instances are frozen so neither mission code nor a late camera observation can
    move ``x``/``y``.  State transitions replace the record held by the inventory.
    """

    slot_id: int
    anchor_id: int
    slot_index: int
    x: float
    y: float
    state: str = UNMAPPED
    track_id: int = 0
    set_type: int = 0
    class_label: str = ""
    fruit_label: str = ""
    confidence: float = 0.0
    observed_x: float | None = None
    observed_y: float | None = None
    target: bool | None = None
    snapshot_time_s: float | None = None
    last_transition_s: float | None = None

    @property
    def actionable(self) -> bool:
        return self.state == OCCUPIED

    @property
    def present(self) -> bool:
        return self.state in {OCCUPIED, UNCONFIRMED, CHECKED, PICKED}

    @property
    def resolved(self) -> bool:
        return self.state in {EMPTY, CHECKED, PICKED}


def _observation_value(observation: object, key: str, default: Any = None) -> Any:
    if isinstance(observation, Mapping):
        return observation.get(key, default)
    return getattr(observation, key, default)


def _normalise_observation(observation: object) -> SlotObservation | None:
    """Convert a dataclass, dict, or WorldModel-like object to a safe observation."""
    try:
        x = float(_observation_value(observation, "x"))
        y = float(_observation_value(observation, "y"))
    except (TypeError, ValueError):
        return None

    if not math.isfinite(x) or not math.isfinite(y):
        return None

    track_id = _observation_value(observation, "track_id", None)
    if track_id is None:
        track_id = _observation_value(observation, "id", 0)
    class_label = _observation_value(observation, "class_label", None)
    if class_label is None:
        class_label = _observation_value(observation, "label", "")
    try:
        confidence = float(_observation_value(observation, "confidence", 0.0))
    except (TypeError, ValueError):
        confidence = 0.0
    if not math.isfinite(confidence):
        confidence = 0.0

    try:
        return SlotObservation(
            x=x,
            y=y,
            track_id=int(track_id or 0),
            set_type=int(_observation_value(observation, "set_type", 0) or 0),
            class_label=str(class_label or ""),
            fruit_label=str(_observation_value(observation, "fruit_label", "") or ""),
            confidence=confidence,
        )
    except (TypeError, ValueError):
        return None


def slot_requires_body_confirmation(
    slot: AnchorSlot,
    *,
    set1_label: str,
    wide_low_confidence: float,
) -> bool:
    """Return whether a locked Wide slot needs a direct Body inspection.

    Wide is the immutable slot/occupancy authority.  Body is reserved for every
    fruit cube, Wide-uncertain observations, and the Set1 object that may be
    picked.  Unknown Wide identities are treated as uncertain rather than being
    silently accepted as non-targets.
    """
    threshold = float(wide_low_confidence)
    if not math.isfinite(threshold):
        raise ValueError("wide_low_confidence must be finite")
    if slot.state != OCCUPIED:
        return False
    if int(slot.set_type) == 2:
        return True
    if int(slot.set_type) != 1 or not str(slot.class_label):
        return True
    if float(slot.confidence) < threshold:
        return True
    return bool(set1_label and str(slot.class_label) == str(set1_label))


def body_observation_matches_slot(
    *,
    observed_xy: tuple[float, float],
    current_slot_id: int,
    slot_points: Mapping[int, tuple[float, float]],
    forward_tol_m: float,
    lateral_tol_m: float,
    bearing_tol_rad: float,
    unique_margin_m: float,
) -> bool:
    """Hard-bind one Body observation to the current immutable Wide slot.

    Coordinates are in ``base_link`` (x forward, y left).  A candidate must be
    close on both axes, agree in bearing, and be uniquely nearer to the current
    slot than to every other slot around the same anchor.  This prevents a visible
    rear-row object from shifting into an occluded front-row slot.
    """
    try:
        ox, oy = (float(observed_xy[0]), float(observed_xy[1]))
        expected = slot_points[int(current_slot_id)]
        ex, ey = float(expected[0]), float(expected[1])
        forward_tol = float(forward_tol_m)
        lateral_tol = float(lateral_tol_m)
        bearing_tol = float(bearing_tol_rad)
        unique_margin = float(unique_margin_m)
    except (KeyError, IndexError, TypeError, ValueError):
        return False
    values = (ox, oy, ex, ey, forward_tol, lateral_tol, bearing_tol, unique_margin)
    if not all(math.isfinite(value) for value in values):
        return False
    if min(forward_tol, lateral_tol, bearing_tol, unique_margin) < 0.0:
        return False
    valid_slot_points: dict[int, tuple[float, float]] = {}
    for slot_id, point in slot_points.items():
        try:
            px, py = float(point[0]), float(point[1])
            sid = int(slot_id)
        except (IndexError, TypeError, ValueError):
            continue
        if math.isfinite(px) and math.isfinite(py):
            valid_slot_points[sid] = (px, py)
    if len(valid_slot_points) != 4 or int(current_slot_id) not in valid_slot_points:
        return False
    if abs(ox - ex) > forward_tol or abs(oy - ey) > lateral_tol:
        return False
    if math.hypot(ex, ey) <= 1e-9 or math.hypot(ox, oy) <= 1e-9:
        return False
    observed_bearing = math.atan2(oy, ox)
    expected_bearing = math.atan2(ey, ex)
    bearing_error = math.atan2(
        math.sin(observed_bearing - expected_bearing),
        math.cos(observed_bearing - expected_bearing),
    )
    if abs(bearing_error) > bearing_tol:
        return False

    current_distance = math.hypot(ox - ex, oy - ey)
    other_distances: list[float] = []
    for slot_id, point in valid_slot_points.items():
        if int(slot_id) == int(current_slot_id):
            continue
        try:
            px, py = float(point[0]), float(point[1])
        except (IndexError, TypeError, ValueError):
            continue
        if math.isfinite(px) and math.isfinite(py):
            other_distances.append(math.hypot(ox - px, oy - py))
    if other_distances and min(other_distances) - current_distance < unique_margin:
        return False
    return True


class AnchorSlotInventory:
    """One-shot fixed-slot map for the K1 -> K2 -> K3 mission."""

    def __init__(
        self,
        *,
        snapshot_radius_m: float = 0.22,
        anchor_xy: Iterable[float] | None = None,
    ) -> None:
        radius = float(snapshot_radius_m)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("snapshot_radius_m must be finite and greater than zero")
        self.snapshot_radius_m = radius

        layout = ANCHOR_LAYOUT
        if anchor_xy is not None:
            coordinates = tuple(float(value) for value in anchor_xy)
            if len(coordinates) != 6 or not all(math.isfinite(value) for value in coordinates):
                raise ValueError("anchor_xy must contain three finite x,y pairs")
            offsets = ((-0.25, -0.25), (0.25, -0.25), (-0.25, 0.25), (0.25, 0.25))
            layout = tuple(
                (
                    index + 1,
                    coordinates[index * 2],
                    coordinates[index * 2 + 1],
                    tuple(
                        (
                            coordinates[index * 2] + offset_x,
                            coordinates[index * 2 + 1] + offset_y,
                        )
                        for offset_x, offset_y in offsets
                    ),
                )
                for index in range(3)
            )

        self._anchors: dict[int, Anchor] = {}
        self._slots: dict[int, AnchorSlot] = {}
        slot_id = 1
        for anchor_id, anchor_x, anchor_y, slot_xy in layout:
            self._anchors[anchor_id] = Anchor(anchor_id, anchor_x, anchor_y)
            for slot_index, (slot_x, slot_y) in enumerate(slot_xy, start=1):
                self._slots[slot_id] = AnchorSlot(
                    slot_id=slot_id,
                    anchor_id=anchor_id,
                    slot_index=slot_index,
                    x=slot_x,
                    y=slot_y,
                )
                slot_id += 1

    @property
    def anchors(self) -> tuple[Anchor, ...]:
        return tuple(self._anchors[k] for k in sorted(self._anchors))

    @property
    def slots(self) -> tuple[AnchorSlot, ...]:
        return tuple(self._slots[k] for k in sorted(self._slots))

    @property
    def all_anchors_locked(self) -> bool:
        return all(anchor.snapshot_locked for anchor in self._anchors.values())

    def get_anchor(self, anchor_id: int) -> Anchor:
        try:
            return self._anchors[int(anchor_id)]
        except (KeyError, TypeError, ValueError) as exc:
            raise KeyError(f"unknown anchor_id: {anchor_id}") from exc

    def get(self, slot_id: int) -> AnchorSlot:
        try:
            return self._slots[int(slot_id)]
        except (KeyError, TypeError, ValueError) as exc:
            raise KeyError(f"unknown slot_id: {slot_id}") from exc

    def slots_for_anchor(self, anchor_id: int) -> tuple[AnchorSlot, ...]:
        aid = self.get_anchor(anchor_id).anchor_id
        return tuple(slot for slot in self.slots if slot.anchor_id == aid)

    def is_anchor_locked(self, anchor_id: int) -> bool:
        return self.get_anchor(anchor_id).snapshot_locked

    # Concise mission-FSM integration names.  Keep the descriptive methods above as
    # backwards-readable aliases for tests and diagnostics.
    def anchor_slots(self, anchor_id: int) -> tuple[AnchorSlot, ...]:
        return self.slots_for_anchor(anchor_id)

    def anchor_locked(self, anchor_id: int) -> bool:
        return self.is_anchor_locked(anchor_id)

    def pending(self, anchor_id: int | None = None) -> tuple[AnchorSlot, ...]:
        selected = self.slots if anchor_id is None else self.slots_for_anchor(anchor_id)
        return tuple(slot for slot in selected if slot.state == OCCUPIED)

    def unconfirmed(self, anchor_id: int | None = None) -> tuple[AnchorSlot, ...]:
        selected = self.slots if anchor_id is None else self.slots_for_anchor(anchor_id)
        return tuple(slot for slot in selected if slot.state == UNCONFIRMED)

    def _best_one_to_one_matches(
        self,
        slots: tuple[AnchorSlot, ...],
        observations: tuple[SlotObservation, ...],
    ) -> dict[int, SlotObservation]:
        """Return a maximum-cardinality, minimum-distance bipartite assignment.

        There are only four slots per anchor.  A 4-bit dynamic program therefore
        handles arbitrarily many detections without greedy-order corner cases.
        """
        # mask -> (total_distance, ((slot_id, observation_index), ...))
        states: dict[int, tuple[float, tuple[tuple[int, int], ...]]] = {0: (0.0, ())}
        epsilon = 1e-12
        for observation_index, observation in enumerate(observations):
            next_states = dict(states)  # skipping this observation is always allowed
            for mask, (cost, pairs) in states.items():
                for local_index, slot in enumerate(slots):
                    bit = 1 << local_index
                    if mask & bit:
                        continue
                    distance = math.hypot(observation.x - slot.x, observation.y - slot.y)
                    if distance > self.snapshot_radius_m + epsilon:
                        continue
                    new_mask = mask | bit
                    candidate = (cost + distance, pairs + ((slot.slot_id, observation_index),))
                    previous = next_states.get(new_mask)
                    if previous is None or candidate < previous:
                        next_states[new_mask] = candidate
            states = next_states

        _, (_, pairs) = min(
            states.items(),
            key=lambda item: (-int(item[0]).bit_count(), item[1][0], item[1][1]),
        )
        return {slot_id: observations[index] for slot_id, index in pairs}

    def lock_anchor_snapshot(
        self,
        anchor_id: int,
        observations: Iterable[object],
        *,
        now_s: float = 0.0,
    ) -> tuple[AnchorSlot, ...]:
        """Lock one anchor's four fixed slots from the first accepted snapshot.

        Repeated calls are intentionally idempotent: once locked, no new observation
        is allowed to fill an ``EMPTY`` slot or replace an ``OCCUPIED`` slot.
        """
        anchor = self.get_anchor(anchor_id)
        if anchor.snapshot_locked:
            return self.slots_for_anchor(anchor.anchor_id)
        stamp = float(now_s)
        if not math.isfinite(stamp):
            raise ValueError("now_s must be finite")

        normalised = tuple(
            converted
            for observation in observations
            if (converted := _normalise_observation(observation)) is not None
        )
        anchor_slots = self.slots_for_anchor(anchor.anchor_id)
        matches = self._best_one_to_one_matches(anchor_slots, normalised)

        for slot in anchor_slots:
            observation = matches.get(slot.slot_id)
            if observation is None:
                updated = replace(
                    slot,
                    state=EMPTY,
                    snapshot_time_s=stamp,
                    last_transition_s=stamp,
                )
            else:
                updated = replace(
                    slot,
                    state=OCCUPIED,
                    track_id=observation.track_id,
                    set_type=observation.set_type,
                    class_label=observation.class_label,
                    fruit_label=observation.fruit_label,
                    confidence=observation.confidence,
                    observed_x=observation.x,
                    observed_y=observation.y,
                    snapshot_time_s=stamp,
                    last_transition_s=stamp,
                )
            self._slots[slot.slot_id] = updated

        self._anchors[anchor.anchor_id] = replace(
            anchor,
            snapshot_locked=True,
            snapshot_time_s=stamp,
        )
        return self.slots_for_anchor(anchor.anchor_id)

    def mark_checked(
        self,
        slot_id: int,
        *,
        target: bool | None = None,
        class_label: str | None = None,
        fruit_label: str | None = None,
        confidence: float | None = None,
        now_s: float | None = None,
    ) -> AnchorSlot:
        """Resolve an occupied slot after close inspection, without moving it."""
        slot = self.get(slot_id)
        if slot.state == CHECKED:
            return slot
        if slot.state != OCCUPIED:
            raise ValueError(f"slot {slot.slot_id} cannot transition {slot.state} -> {CHECKED}")
        stamp = slot.last_transition_s if now_s is None else float(now_s)
        conf = slot.confidence if confidence is None else float(confidence)
        if stamp is not None and not math.isfinite(stamp):
            raise ValueError("now_s must be finite")
        if not math.isfinite(conf):
            raise ValueError("confidence must be finite")
        updated = replace(
            slot,
            state=CHECKED,
            target=target if target is not None else slot.target,
            class_label=str(class_label) if class_label is not None else slot.class_label,
            fruit_label=str(fruit_label) if fruit_label is not None else slot.fruit_label,
            confidence=conf,
            last_transition_s=stamp,
        )
        self._slots[slot.slot_id] = updated
        return updated

    def mark_unconfirmed(
        self,
        slot_id: int,
        *,
        now_s: float | None = None,
    ) -> AnchorSlot:
        """Hold an occupied slot outside the normal queue without resolving it."""
        slot = self.get(slot_id)
        if slot.state == UNCONFIRMED:
            return slot
        if slot.state != OCCUPIED:
            raise ValueError(
                f"slot {slot.slot_id} cannot transition {slot.state} -> {UNCONFIRMED}"
            )
        stamp = slot.last_transition_s if now_s is None else float(now_s)
        if stamp is not None and not math.isfinite(stamp):
            raise ValueError("now_s must be finite")
        updated = replace(slot, state=UNCONFIRMED, last_transition_s=stamp)
        self._slots[slot.slot_id] = updated
        return updated

    def retry_unconfirmed_once(
        self,
        anchor_id: int,
        *,
        now_s: float | None = None,
    ) -> tuple[AnchorSlot, ...]:
        """Requeue held slots once, after the anchor's regular queue is exhausted."""
        anchor = self.get_anchor(anchor_id)
        held = self.unconfirmed(anchor.anchor_id)
        if anchor.final_retry_started or not held:
            return ()
        stamp = anchor.snapshot_time_s if now_s is None else float(now_s)
        if stamp is not None and not math.isfinite(stamp):
            raise ValueError("now_s must be finite")
        self._anchors[anchor.anchor_id] = replace(anchor, final_retry_started=True)
        retried: list[AnchorSlot] = []
        for slot in held:
            updated = replace(slot, state=OCCUPIED, last_transition_s=stamp)
            self._slots[slot.slot_id] = updated
            retried.append(updated)
        return tuple(retried)

    def mark_picked(self, slot_id: int, *, now_s: float | None = None) -> AnchorSlot:
        """Mark a present slot removed; EMPTY and UNMAPPED slots cannot be picked."""
        slot = self.get(slot_id)
        if slot.state == PICKED:
            return slot
        if slot.state not in {OCCUPIED, CHECKED}:
            raise ValueError(f"slot {slot.slot_id} cannot transition {slot.state} -> {PICKED}")
        stamp = slot.last_transition_s if now_s is None else float(now_s)
        if stamp is not None and not math.isfinite(stamp):
            raise ValueError("now_s must be finite")
        updated = replace(slot, state=PICKED, target=True, last_transition_s=stamp)
        self._slots[slot.slot_id] = updated
        return updated

    def counts(self, anchor_id: int | None = None) -> dict[str, int]:
        selected = self.slots if anchor_id is None else self.slots_for_anchor(anchor_id)
        by_state = {state: sum(slot.state == state for slot in selected) for state in SLOT_STATES}
        present = (
            by_state[OCCUPIED]
            + by_state[UNCONFIRMED]
            + by_state[CHECKED]
            + by_state[PICKED]
        )
        fruit_cubes = sum(slot.present and slot.set_type == 2 for slot in selected)
        shape_objects = sum(slot.present and slot.set_type == 1 for slot in selected)
        return {
            "total": len(selected),
            "unmapped": by_state[UNMAPPED],
            "empty": by_state[EMPTY],
            "occupied": by_state[OCCUPIED],
            "unconfirmed": by_state[UNCONFIRMED],
            "checked": by_state[CHECKED],
            "picked": by_state[PICKED],
            "present": present,
            "objects": present,
            "shape_objects": shape_objects,
            "fruit_cubes": fruit_cubes,
            "pending": by_state[OCCUPIED],
            "resolved": by_state[EMPTY] + by_state[CHECKED] + by_state[PICKED],
        }

    def debug_payload(
        self,
        *,
        current_anchor_id: int = 0,
        current_slot_id: int = 0,
    ) -> dict[str, Any]:
        """Return a JSON-serialisable payload for ``/planning/object_slots``."""
        return {
            "schema": "anchor_slots_v1",
            "enabled": True,
            "current_anchor_id": int(current_anchor_id),
            "current_slot_id": int(current_slot_id),
            "all_anchors_locked": self.all_anchors_locked,
            "counts": self.counts(),
            "anchors": [
                {
                    "id": anchor.anchor_id,
                    "x": round(anchor.x, 4),
                    "y": round(anchor.y, 4),
                    "state": "LOCKED" if anchor.snapshot_locked else UNMAPPED,
                    "snapshot_locked": anchor.snapshot_locked,
                    "snapshot_time_s": anchor.snapshot_time_s,
                    "final_retry_started": anchor.final_retry_started,
                    "counts": self.counts(anchor.anchor_id),
                }
                for anchor in self.anchors
            ],
            "slots": [
                {
                    "id": slot.slot_id,
                    "anchor_id": slot.anchor_id,
                    "slot_index": slot.slot_index,
                    "x": round(slot.x, 4),
                    "y": round(slot.y, 4),
                    "slot_state": slot.state,
                    "state": slot.state,
                    "track_id": slot.track_id,
                    "set_type": slot.set_type,
                    "class_label": slot.class_label,
                    "fruit": slot.fruit_label or "unknown",
                    "confidence": round(slot.confidence, 3),
                    "target": bool(slot.target),
                    "inspected": slot.state in {CHECKED, PICKED},
                    "non_target": slot.state == CHECKED and slot.target is False,
                    "picked": slot.state == PICKED,
                    "grid_locked": True,
                    "observed_x": slot.observed_x,
                    "observed_y": slot.observed_y,
                }
                for slot in self.slots
            ],
        }
