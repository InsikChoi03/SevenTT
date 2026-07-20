"""Regression tests for the fixed K1/K2/K3 anchor-slot inventory."""
import json
import math
from dataclasses import FrozenInstanceError

import pytest

from robot_planning.anchor_slot_inventory import (
    CHECKED,
    EMPTY,
    OCCUPIED,
    PICKED,
    UNCONFIRMED,
    UNMAPPED,
    AnchorSlot,
    AnchorSlotInventory,
    SlotObservation,
    body_observation_matches_slot,
    slot_requires_body_confirmation,
)


def test_layout_has_three_anchors_and_twelve_immutable_fixed_slots():
    inventory = AnchorSlotInventory()

    assert [(a.anchor_id, a.x, a.y) for a in inventory.anchors] == [
        (1, -1.25, 0.75),
        (2, -1.25, -0.25),
        (3, -1.25, -1.25),
    ]
    assert [(s.slot_id, s.anchor_id, s.slot_index, s.x, s.y) for s in inventory.slots] == [
        (1, 1, 1, -1.50, 0.50),
        (2, 1, 2, -1.00, 0.50),
        (3, 1, 3, -1.50, 1.00),
        (4, 1, 4, -1.00, 1.00),
        (5, 2, 1, -1.50, -0.50),
        (6, 2, 2, -1.00, -0.50),
        (7, 2, 3, -1.50, 0.00),
        (8, 2, 4, -1.00, 0.00),
        (9, 3, 1, -1.50, -1.50),
        (10, 3, 2, -1.00, -1.50),
        (11, 3, 3, -1.50, -1.00),
        (12, 3, 4, -1.00, -1.00),
    ]
    assert all(slot.state == UNMAPPED for slot in inventory.slots)

    with pytest.raises(FrozenInstanceError):
        inventory.get(1).x = 99.0


def test_first_snapshot_locks_occupied_and_empty_with_fixed_xy():
    inventory = AnchorSlotInventory(snapshot_radius_m=0.22)
    locked = inventory.lock_anchor_snapshot(
        1,
        [
            SlotObservation(-1.47, 0.52, track_id=11, set_type=1, class_label="cube", confidence=0.8),
            {"x": -0.98, "y": 1.03, "id": 12, "set_type": 2,
             "class_label": "fruit_photo_cube", "confidence": 0.9},
            {"x": 0.0, "y": 0.0, "id": 99},  # outside every K1 slot gate
        ],
        now_s=10.0,
    )

    assert [slot.state for slot in locked] == [OCCUPIED, EMPTY, EMPTY, OCCUPIED]
    assert (locked[0].x, locked[0].y) == (-1.50, 0.50)
    assert (locked[0].observed_x, locked[0].observed_y) == (-1.47, 0.52)
    assert locked[0].track_id == 11
    assert locked[3].track_id == 12
    assert inventory.is_anchor_locked(1)
    assert all(slot.state == UNMAPPED for slot in inventory.slots_for_anchor(2))


def test_config_anchor_xy_derives_four_fixed_quarter_offset_slots_and_alias_apis():
    inventory = AnchorSlotInventory(
        anchor_xy=[-1.25, 0.75, -1.25, -0.25, -1.25, -1.25]
    )
    inventory.lock_anchor_snapshot(1, [SlotObservation(-1.49, 0.51, track_id=7)])

    assert inventory.anchor_locked(1)
    assert inventory.anchor_slots(1) == inventory.slots_for_anchor(1)
    assert inventory.pending(1) == (inventory.get(1),)
    assert [(slot.x, slot.y) for slot in inventory.anchor_slots(2)] == [
        (-1.50, -0.50),
        (-1.00, -0.50),
        (-1.50, 0.00),
        (-1.00, 0.00),
    ]

    with pytest.raises(ValueError):
        AnchorSlotInventory(anchor_xy=[0.0, 1.0])


def test_snapshot_assignment_is_one_to_one_and_maximises_matches():
    inventory = AnchorSlotInventory(snapshot_radius_m=0.35)
    locked = inventory.lock_anchor_snapshot(
        1,
        [
            # This observation can fit S1 or S2.  The second can only fit S1, so
            # maximum-cardinality assignment must place this one at S2.
            SlotObservation(-1.30, 0.50, track_id=20),
            SlotObservation(-1.48, 0.50, track_id=21),
        ],
    )

    assert [slot.state for slot in locked] == [OCCUPIED, OCCUPIED, EMPTY, EMPTY]
    assert locked[0].track_id == 21
    assert locked[1].track_id == 20


def test_locked_snapshot_ignores_later_objects_and_never_creates_slots():
    inventory = AnchorSlotInventory()
    inventory.lock_anchor_snapshot(1, [SlotObservation(-1.50, 0.50, track_id=1)], now_s=1.0)
    before = inventory.slots_for_anchor(1)

    after = inventory.lock_anchor_snapshot(
        1,
        [
            SlotObservation(-1.00, 0.50, track_id=2),
            SlotObservation(-1.50, 1.00, track_id=3),
        ],
        now_s=99.0,
    )

    assert after == before
    assert [slot.state for slot in after] == [OCCUPIED, EMPTY, EMPTY, EMPTY]
    assert len(inventory.slots) == 12
    assert inventory.get_anchor(1).snapshot_time_s == 1.0


def test_checked_and_picked_transitions_preserve_position_and_reject_empty():
    inventory = AnchorSlotInventory()
    inventory.lock_anchor_snapshot(
        1,
        [
            SlotObservation(-1.50, 0.50, track_id=1),
            SlotObservation(-1.00, 0.50, track_id=2),
        ],
    )

    checked = inventory.mark_checked(
        1,
        target=False,
        class_label="dodecahedron",
        confidence=0.97,
        now_s=3.0,
    )
    picked = inventory.mark_picked(2, now_s=4.0)
    assert checked.state == CHECKED
    assert checked.target is False
    assert (checked.x, checked.y) == (-1.50, 0.50)
    assert picked.state == PICKED
    assert picked.target is True
    assert (picked.x, picked.y) == (-1.00, 0.50)

    assert inventory.mark_checked(1) is checked
    assert inventory.mark_picked(2) is picked
    with pytest.raises(ValueError):
        inventory.mark_checked(3)
    with pytest.raises(ValueError):
        inventory.mark_picked(4)


def test_counts_and_debug_payload_cover_all_states_and_are_json_serialisable():
    inventory = AnchorSlotInventory()
    inventory.lock_anchor_snapshot(
        1,
        [
            SlotObservation(-1.50, 0.50, track_id=1),
            SlotObservation(-1.00, 0.50, track_id=2),
            SlotObservation(-1.50, 1.00, track_id=3),
        ],
        now_s=5.0,
    )
    inventory.mark_checked(1, target=False)
    inventory.mark_picked(2)

    assert inventory.counts(1) == {
        "total": 4,
        "unmapped": 0,
        "empty": 1,
        "occupied": 1,
        "unconfirmed": 0,
        "checked": 1,
        "picked": 1,
        "present": 3,
        "objects": 3,
        "shape_objects": 0,
        "fruit_cubes": 0,
        "pending": 1,
        "resolved": 3,
    }
    assert inventory.counts() == {
        "total": 12,
        "unmapped": 8,
        "empty": 1,
        "occupied": 1,
        "unconfirmed": 0,
        "checked": 1,
        "picked": 1,
        "present": 3,
        "objects": 3,
        "shape_objects": 0,
        "fruit_cubes": 0,
        "pending": 1,
        "resolved": 3,
    }

    payload = inventory.debug_payload(current_anchor_id=1, current_slot_id=3)
    json.dumps(payload)
    assert payload["schema"] == "anchor_slots_v1"
    assert payload["current_anchor_id"] == 1
    assert payload["current_slot_id"] == 3
    assert len(payload["anchors"]) == 3
    assert len(payload["slots"]) == 12
    assert payload["anchors"][0]["snapshot_locked"] is True
    assert payload["anchors"][1]["snapshot_locked"] is False
    assert [slot["slot_state"] for slot in payload["slots"][:4]] == [
        CHECKED,
        PICKED,
        OCCUPIED,
        EMPTY,
    ]


def test_unconfirmed_slot_is_held_then_requeued_only_once():
    inventory = AnchorSlotInventory()
    inventory.lock_anchor_snapshot(
        1,
        [
            SlotObservation(
                -1.50,
                0.50,
                track_id=1,
                set_type=1,
                class_label="dodecahedron",
                confidence=0.9,
            )
        ],
    )

    held = inventory.mark_unconfirmed(1, now_s=1.0)

    assert held.state == UNCONFIRMED
    assert held.present
    assert not held.actionable
    assert not held.resolved
    assert inventory.pending(1) == ()
    assert inventory.unconfirmed(1) == (held,)
    assert inventory.counts(1) == {
        "total": 4,
        "unmapped": 0,
        "empty": 3,
        "occupied": 0,
        "unconfirmed": 1,
        "checked": 0,
        "picked": 0,
        "present": 1,
        "objects": 1,
        "shape_objects": 1,
        "fruit_cubes": 0,
        "pending": 0,
        "resolved": 3,
    }

    first_retry = inventory.retry_unconfirmed_once(1, now_s=2.0)
    assert [slot.slot_id for slot in first_retry] == [1]
    assert inventory.get(1).state == OCCUPIED
    assert inventory.get_anchor(1).final_retry_started

    inventory.mark_unconfirmed(1, now_s=3.0)
    assert inventory.retry_unconfirmed_once(1, now_s=4.0) == ()
    assert inventory.get(1).state == UNCONFIRMED
    assert inventory.pending(1) == ()
    assert (inventory.get(1).x, inventory.get(1).y) == (-1.50, 0.50)
    payload = inventory.debug_payload(current_anchor_id=1, current_slot_id=1)
    assert payload["anchors"][0]["final_retry_started"] is True
    assert payload["slots"][0]["slot_state"] == UNCONFIRMED


@pytest.mark.parametrize(
    ("set_type", "label", "confidence", "set1_label", "expected"),
    [
        (2, "fruit_photo_cube", 0.99, "dodecahedron", True),
        (1, "octahedron", 0.69, "dodecahedron", True),
        (1, "dodecahedron", 0.99, "dodecahedron", True),
        (1, "octahedron", 0.99, "dodecahedron", False),
        (1, "octahedron", 0.70, "dodecahedron", False),
        (0, "", 0.99, "dodecahedron", True),
    ],
)
def test_wide_trust_selects_only_slots_that_need_body(
    set_type, label, confidence, set1_label, expected
):
    slot = AnchorSlot(
        slot_id=1,
        anchor_id=1,
        slot_index=1,
        x=0.0,
        y=0.0,
        state=OCCUPIED,
        set_type=set_type,
        class_label=label,
        confidence=confidence,
    )
    assert slot_requires_body_confirmation(
        slot,
        set1_label=set1_label,
        wide_low_confidence=0.70,
    ) is expected


def test_body_slot_gate_accepts_only_current_unique_slot():
    slot_points = {1: (0.50, 0.00), 2: (1.00, 0.00), 3: (0.50, 0.50), 4: (1.00, 0.50)}
    assert body_observation_matches_slot(
        observed_xy=(0.58, 0.05),
        current_slot_id=1,
        slot_points=slot_points,
        forward_tol_m=0.12,
        lateral_tol_m=0.10,
        bearing_tol_rad=math.radians(10.0),
        unique_margin_m=0.08,
    )


def test_body_slot_gate_fails_closed_without_all_four_wide_slots():
    assert not body_observation_matches_slot(
        observed_xy=(0.50, 0.00),
        current_slot_id=1,
        slot_points={1: (0.50, 0.00)},
        forward_tol_m=0.12,
        lateral_tol_m=0.10,
        bearing_tol_rad=math.radians(10.0),
        unique_margin_m=0.08,
    )


@pytest.mark.parametrize(
    ("observed_xy", "forward_tol", "lateral_tol", "bearing_tol"),
    [
        ((0.63, 0.00), 0.12, 0.10, math.radians(10.0)),
        ((0.50, 0.11), 0.12, 0.10, math.pi),
        ((0.50, 0.09), 0.12, 0.10, math.radians(10.0)),
        ((1.00, 0.00), 0.60, 0.10, math.radians(10.0)),
        ((0.75, 0.00), 0.40, 0.10, math.pi),
    ],
)
def test_body_slot_gate_rejects_axis_bearing_rear_row_and_ambiguity(
    observed_xy, forward_tol, lateral_tol, bearing_tol
):
    slot_points = {1: (0.50, 0.00), 2: (1.00, 0.00), 3: (0.50, 0.50), 4: (1.00, 0.50)}
    assert not body_observation_matches_slot(
        observed_xy=observed_xy,
        current_slot_id=1,
        slot_points=slot_points,
        forward_tol_m=forward_tol,
        lateral_tol_m=lateral_tol,
        bearing_tol_rad=bearing_tol,
        unique_margin_m=0.08,
    )
