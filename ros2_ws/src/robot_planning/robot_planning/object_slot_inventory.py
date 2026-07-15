"""Stable field slots that survive short-lived world-model track IDs."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable


@dataclass
class ObjectSlot:
    """A persistent inspection target at a fixed field position."""

    slot_id: int
    x: float
    y: float
    set_type: int
    class_label: str = ""
    track_id: int | None = None
    physical_state: str = "candidate"
    fruit_label: str = ""
    fruit_confidence: float = 0.0
    inspected: bool = False
    target_confirmed: bool = False
    non_target: bool = False
    picked: bool = False
    confidence: float = 0.0
    n_obs: int = 0
    last_seen_s: float = 0.0
    last_seen_wide_s: float | None = None
    last_seen_body_s: float | None = None
    last_inspected_s: float | None = None
    retry_count: int = 0
    retry_after_s: float = 0.0
    grid_id: int | None = None
    grid_locked: bool = False

    @property
    def actionable(self) -> bool:
        return not self.picked and not self.non_target

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


class SlotInventory:
    """Merge changing tracks into persistent, position-based object slots."""

    def __init__(
        self,
        *,
        enabled_set_types: Iterable[int],
        merge_radius_m: float = 0.22,
        confirm_min_obs: int = 1,
    ) -> None:
        self.enabled_set_types = {int(v) for v in enabled_set_types}
        self.merge_radius_m = max(0.0, float(merge_radius_m))
        self.confirm_min_obs = max(1, int(confirm_min_obs))
        self._slots: dict[int, ObjectSlot] = {}
        self._track_to_slot: dict[int, int] = {}
        self._next_slot_id = 1

    @property
    def slots(self) -> tuple[ObjectSlot, ...]:
        return tuple(self._slots[k] for k in sorted(self._slots))

    def get(self, slot_id: int | None) -> ObjectSlot | None:
        if slot_id is None:
            return None
        return self._slots.get(int(slot_id))

    def slot_for_track(self, track_id: int) -> ObjectSlot | None:
        slot_id = self._track_to_slot.get(int(track_id))
        return self.get(slot_id)

    def link_track(self, slot_id: int, track_id: int) -> ObjectSlot | None:
        slot = self.get(slot_id)
        if slot is None or int(track_id) == 0:
            return slot
        slot.track_id = int(track_id)
        self._track_to_slot[int(track_id)] = slot.slot_id
        return slot

    def nearest_slot(
        self,
        x: float,
        y: float,
        *,
        set_type: int | None = None,
        max_distance_m: float | None = None,
    ) -> ObjectSlot | None:
        limit = self.merge_radius_m if max_distance_m is None else max(0.0, float(max_distance_m))
        best = None
        best_dist = limit
        for slot in self._slots.values():
            if set_type is not None and slot.set_type != int(set_type):
                continue
            dist = slot.distance_to(float(x), float(y))
            if dist <= best_dist:
                best = slot
                best_dist = dist
        return best

    def add_or_merge_observation(
        self,
        *,
        track_id: int,
        set_type: int,
        x: float,
        y: float,
        confidence: float,
        n_obs: int,
        class_label: str,
        source: str,
        seen_s: float,
        blacklisted: bool = False,
        allow_create: bool = True,
        fixed_xy: tuple[float, float] | None = None,
        grid_id: int | None = None,
        position_locked: bool = False,
    ) -> ObjectSlot | None:
        """Attach an observation by track ID first, then by field proximity.

        A confirmed slot's position does not follow per-frame track jitter. The only
        operation allowed to move confirmed slots is ``apply_transform``, which is a
        rigid correction of the complete field frame.
        """
        set_type = int(set_type)
        if set_type not in self.enabled_set_types:
            return None
        track_id = int(track_id)
        slot = self.slot_for_track(track_id) if track_id else None
        if slot is None:
            slot = self.nearest_slot(x, y, set_type=set_type)
        if slot is None:
            if blacklisted or not allow_create:
                return None
            slot_x, slot_y = fixed_xy if fixed_xy is not None else (float(x), float(y))
            slot = ObjectSlot(
                slot_id=self._next_slot_id,
                x=float(slot_x),
                y=float(slot_y),
                set_type=set_type,
                grid_id=grid_id,
                grid_locked=bool(position_locked),
            )
            self._slots[slot.slot_id] = slot
            self._next_slot_id += 1
        elif fixed_xy is not None and position_locked and not slot.grid_locked:
            slot.x = float(fixed_xy[0])
            slot.y = float(fixed_xy[1])
            slot.grid_id = grid_id
            slot.grid_locked = True

        # Candidates may settle before confirmation. Confirmed positions stay fixed.
        if not slot.grid_locked and slot.physical_state == "candidate" and slot.n_obs > 0:
            old_weight = max(1, slot.n_obs)
            new_weight = max(1, int(n_obs))
            denom = old_weight + new_weight
            slot.x = (slot.x * old_weight + float(x) * new_weight) / denom
            slot.y = (slot.y * old_weight + float(y) * new_weight) / denom

        slot.track_id = track_id or slot.track_id
        if track_id:
            self._track_to_slot[track_id] = slot.slot_id
        slot.class_label = str(class_label) or slot.class_label
        slot.confidence = max(slot.confidence, float(confidence))
        slot.n_obs = max(slot.n_obs, int(n_obs))
        slot.last_seen_s = max(slot.last_seen_s, float(seen_s))
        source = str(source)
        if "wide" in source:
            slot.last_seen_wide_s = float(seen_s)
        if "body" in source:
            slot.last_seen_body_s = float(seen_s)
        if not slot.picked and slot.n_obs >= self.confirm_min_obs:
            slot.physical_state = "confirmed_present"
        return slot

    def select_next_slot(
        self,
        *,
        robot_xy: tuple[float, float],
        now_s: float,
        min_confidence: float,
        seen_after_s: float = 0.0,
        predicate: Callable[[ObjectSlot], bool] | None = None,
    ) -> ObjectSlot | None:
        """Select every unresolved slot eventually; distance only determines order."""
        rx, ry = robot_xy
        candidates = []
        for slot in self._slots.values():
            if not slot.actionable or slot.retry_after_s > float(now_s):
                continue
            if slot.confidence < float(min_confidence) or slot.last_seen_s < float(seen_after_s):
                continue
            if predicate is not None and not predicate(slot):
                continue
            state_penalty = 0 if slot.physical_state == "confirmed_present" else 1
            candidates.append(
                (state_penalty, slot.retry_count, slot.distance_to(rx, ry), slot.slot_id, slot)
            )
        return min(candidates)[-1] if candidates else None

    def has_unresolved(
        self,
        *,
        min_confidence: float = 0.0,
        seen_after_s: float = 0.0,
        predicate: Callable[[ObjectSlot], bool] | None = None,
    ) -> bool:
        return any(
            slot.actionable
            and slot.confidence >= float(min_confidence)
            and slot.last_seen_s >= float(seen_after_s)
            and (predicate is None or predicate(slot))
            for slot in self._slots.values()
        )

    def attach_identity(
        self,
        slot_id: int,
        *,
        fruit_label: str,
        confidence: float,
        inspected_s: float,
    ) -> ObjectSlot | None:
        slot = self.get(slot_id)
        if slot is None or not fruit_label:
            return slot
        if not slot.fruit_label or float(confidence) >= slot.fruit_confidence:
            slot.fruit_label = str(fruit_label)
            slot.fruit_confidence = float(confidence)
        slot.last_inspected_s = float(inspected_s)
        return slot

    def mark_target_confirmed(self, slot_id: int, inspected_s: float) -> None:
        slot = self.get(slot_id)
        if slot is None:
            return
        slot.inspected = True
        slot.target_confirmed = True
        slot.non_target = False
        slot.last_inspected_s = float(inspected_s)

    def mark_non_target(self, slot_id: int, inspected_s: float) -> None:
        slot = self.get(slot_id)
        if slot is None:
            return
        slot.inspected = True
        slot.target_confirmed = False
        slot.non_target = True
        slot.last_inspected_s = float(inspected_s)

    def mark_picked(self, slot_id: int, inspected_s: float) -> None:
        slot = self.get(slot_id)
        if slot is None:
            return
        slot.inspected = True
        slot.target_confirmed = True
        slot.picked = True
        slot.non_target = False
        slot.physical_state = "picked_removed"
        slot.last_inspected_s = float(inspected_s)

    def mark_retry(
        self,
        slot_id: int,
        *,
        now_s: float,
        retry_limit: int,
        retry_cooldown_sec: float,
    ) -> None:
        slot = self.get(slot_id)
        if slot is None:
            return
        slot.inspected = False
        slot.retry_count += 1
        slot.retry_after_s = float(now_s) + max(0.0, float(retry_cooldown_sec))
        slot.last_inspected_s = float(now_s)
        if slot.retry_count >= max(1, int(retry_limit)):
            slot.physical_state = "lost_suspect"

    def apply_transform(
        self,
        tx: float,
        ty: float,
        dtheta: float,
        *,
        include_grid_locked: bool = False,
    ) -> None:
        """Apply a rigid field-frame correction to all persistent positions."""
        ct, st = math.cos(float(dtheta)), math.sin(float(dtheta))
        for slot in self._slots.values():
            if slot.grid_locked and not include_grid_locked:
                continue
            x, y = slot.x, slot.y
            slot.x = ct * x - st * y + float(tx)
            slot.y = st * x + ct * y + float(ty)

    def debug_dicts(self) -> list[dict]:
        return [
            {
                "id": slot.slot_id,
                "x": round(slot.x, 4),
                "y": round(slot.y, 4),
                "set_type": slot.set_type,
                "track_id": slot.track_id or 0,
                "state": slot.physical_state,
                "fruit": slot.fruit_label or "unknown",
                "inspected": slot.inspected,
                "target": slot.target_confirmed,
                "non_target": slot.non_target,
                "picked": slot.picked,
                "retries": slot.retry_count,
                "confidence": round(slot.confidence, 3),
                "grid_id": slot.grid_id or 0,
                "grid_locked": slot.grid_locked,
            }
            for slot in self.slots
        ]
