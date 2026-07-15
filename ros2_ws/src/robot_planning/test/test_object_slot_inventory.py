"""Regression tests for position-stable Set2 slot inventory behaviour."""
import math

from robot_planning.object_slot_inventory import SlotInventory


def _inventory():
    return SlotInventory(enabled_set_types={2}, merge_radius_m=0.22, confirm_min_obs=1)


def _observe(
    inv,
    track_id,
    x,
    y,
    *,
    set_type=2,
    blacklisted=False,
    seen_s=10.0,
    allow_create=True,
    fixed_xy=None,
    grid_id=None,
    position_locked=False,
):
    return inv.add_or_merge_observation(
        track_id=track_id,
        set_type=set_type,
        x=x,
        y=y,
        confidence=0.9,
        n_obs=4,
        class_label="fruit_photo_cube",
        source="wide",
        seen_s=seen_s,
        blacklisted=blacklisted,
        allow_create=allow_create,
        fixed_xy=fixed_xy,
        grid_id=grid_id,
        position_locked=position_locked,
    )


def test_track_id_churn_nearby_keeps_one_slot():
    inv = _inventory()
    first = _observe(inv, 12, 1.25, 1.00)
    second = _observe(inv, 18, 1.31, 0.95)
    assert first is second
    assert second.slot_id == 1
    assert second.track_id == 18
    assert len(inv.slots) == 1


def test_confirmed_slot_position_does_not_follow_track_jitter():
    inv = _inventory()
    slot = _observe(inv, 12, 1.25, 1.00)
    _observe(inv, 12, 1.48, 1.00)
    assert (slot.x, slot.y) == (1.25, 1.00)


def test_set1_and_new_blacklisted_tracks_do_not_create_slots():
    inv = _inventory()
    assert _observe(inv, 1, 0.0, 0.0, set_type=1) is None
    assert _observe(inv, 2, 0.0, 0.0, blacklisted=True) is None
    assert not inv.slots


def test_birth_gate_blocks_new_slot_but_keeps_existing_slot_updates():
    inv = _inventory()
    assert _observe(inv, 1, 0.0, 0.0, allow_create=False) is None
    assert not inv.slots

    slot = _observe(inv, 2, 0.0, 0.0)
    updated = _observe(inv, 2, 0.1, 0.0, seen_s=11.0, allow_create=False)
    assert updated is slot
    assert updated.last_seen_s == 11.0


def test_selection_excludes_non_target_and_picked_slots():
    inv = _inventory()
    a = _observe(inv, 1, 0.0, 0.0)
    b = _observe(inv, 2, 1.0, 0.0)
    c = _observe(inv, 3, 2.0, 0.0)
    inv.mark_non_target(a.slot_id, 11.0)
    inv.mark_picked(b.slot_id, 11.0)
    selected = inv.select_next_slot(
        robot_xy=(0.0, 0.0), now_s=12.0, min_confidence=0.5
    )
    assert selected is c


def test_retry_uses_cooldown_then_becomes_selectable_again():
    inv = _inventory()
    slot = _observe(inv, 1, 0.0, 0.0)
    inv.mark_retry(slot.slot_id, now_s=20.0, retry_limit=2, retry_cooldown_sec=3.0)
    assert inv.select_next_slot(robot_xy=(0.0, 0.0), now_s=22.9, min_confidence=0.5) is None
    assert inv.select_next_slot(robot_xy=(0.0, 0.0), now_s=23.0, min_confidence=0.5) is slot


def test_fresh_seen_gate_keeps_pre_anchor_observation_out():
    inv = _inventory()
    old = _observe(inv, 1, 0.0, 0.0, seen_s=10.0)
    fresh = _observe(inv, 2, 1.0, 0.0, seen_s=21.0)
    selected = inv.select_next_slot(
        robot_xy=(0.0, 0.0), now_s=22.0, min_confidence=0.5, seen_after_s=20.0
    )
    assert selected is fresh
    assert selected is not old


def test_field_transform_moves_slots_rigidly():
    inv = _inventory()
    slot = _observe(inv, 1, 1.0, 0.0)
    inv.apply_transform(0.5, -0.25, math.pi / 2.0)
    assert abs(slot.x - 0.5) < 1e-9
    assert abs(slot.y - 0.75) < 1e-9


def test_grid_locked_slot_uses_fixed_grid_position_and_ignores_observation_jitter():
    inv = _inventory()
    slot = _observe(
        inv,
        1,
        1.09,
        0.93,
        fixed_xy=(1.0, 1.0),
        grid_id=17,
        position_locked=True,
    )
    assert slot.grid_locked
    assert slot.grid_id == 17
    assert (slot.x, slot.y) == (1.0, 1.0)

    _observe(inv, 1, 1.19, 0.81, fixed_xy=(1.0, 1.0), grid_id=17, position_locked=True)
    assert (slot.x, slot.y) == (1.0, 1.0)


def test_field_transform_skips_grid_locked_slots_by_default():
    inv = _inventory()
    slot = _observe(
        inv,
        1,
        1.09,
        0.93,
        fixed_xy=(1.0, 1.0),
        grid_id=17,
        position_locked=True,
    )
    inv.apply_transform(0.5, -0.25, math.pi / 2.0)
    assert (slot.x, slot.y) == (1.0, 1.0)

    inv.apply_transform(0.5, -0.25, math.pi / 2.0, include_grid_locked=True)
    assert abs(slot.x + 0.5) < 1e-9
    assert abs(slot.y - 0.75) < 1e-9
