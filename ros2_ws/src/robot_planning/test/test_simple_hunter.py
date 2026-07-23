"""Unit tests for the ROS-free simple-hunter helpers."""
import math
from types import SimpleNamespace

from robot_planning.simple_hunter import (
    align_translation_axis,
    clamp_outward_field_velocity,
    is_strong_set1,
    is_strong_set2,
    position_banned,
    prune_bans,
    radial_standoff,
    select_body_candidate,
    select_hunt_target,
)


def obj(**kw):
    base = dict(
        id=1, class_label="cube", set_type=1, x=0.0, y=0.0, confidence=0.9,
        blacklisted=False, n_obs=5, source="wide", fruit_label="",
        fruit_confidence=0.0, fruit_label_source="", locked=False,
    )
    base.update(kw)
    return SimpleNamespace(**base)


# ------------------------------------------------------------------ strong gates
def test_strong_set1_accepts_high_conf_target():
    assert is_strong_set1(obj(confidence=0.85), "cube", 0.80, 2)


def test_strong_set1_rejects_low_conf_wrong_label_blacklist():
    assert not is_strong_set1(obj(confidence=0.7), "cube", 0.80, 2)
    assert not is_strong_set1(obj(class_label="octahedron"), "cube", 0.80, 2)
    assert not is_strong_set1(obj(blacklisted=True), "cube", 0.80, 2)
    assert not is_strong_set1(obj(n_obs=1), "cube", 0.80, 2)


def test_strong_set1_rejects_any_fruit_evidence():
    # PRINCIPLE: a fruit-labeled track IS a fruit cube — never a set1 shape,
    # regardless of what the class vote says
    assert not is_strong_set1(obj(fruit_label="apple"), "cube", 0.80, 2)
    assert not is_strong_set1(
        obj(class_label="icosahedron", fruit_label="apple"), "icosahedron", 0.80, 2)


def test_strong_set2_body_vs_wide_margin():
    fruit = dict(set_type=2, class_label="fruit_photo_cube", fruit_label="pineapple")
    assert is_strong_set2(
        obj(**fruit, fruit_confidence=0.55, fruit_label_source="body"),
        "pineapple", 0.50, 0.70, 0.50, 2)
    # same margin from wide is NOT enough
    assert not is_strong_set2(
        obj(**fruit, fruit_confidence=0.55, fruit_label_source="wide"),
        "pineapple", 0.50, 0.70, 0.50, 2)
    assert is_strong_set2(
        obj(**fruit, fruit_confidence=0.75, fruit_label_source="wide"),
        "pineapple", 0.50, 0.70, 0.50, 2)
    # wrong fruit never qualifies
    wrong = dict(fruit, fruit_label="banana")
    assert not is_strong_set2(
        obj(**wrong, fruit_confidence=0.9, fruit_label_source="body"),
        "pineapple", 0.50, 0.70, 0.50, 2)


def test_strong_set2_ignores_frozen_set_type():
    # PRINCIPLE: fruit evidence outranks the class vote — a track frozen as
    # set_type=1 "cube" with an attached pineapple label is still a set2 target
    frozen = obj(set_type=1, class_label="cube", fruit_label="pineapple",
                 fruit_confidence=0.6, fruit_label_source="body")
    assert is_strong_set2(frozen, "pineapple", 0.50, 0.70, 0.50, 2)


def test_fruit_conflict_action_decision_table():
    from robot_planning.simple_hunter import fruit_conflict_action
    # quota full -> worthless regardless of the read
    assert fruit_conflict_action(("pineapple", 0.9, True), "pineapple", 0.5, 0) == "reject_quota"
    # no read / weak read / face not visible -> hold and wait
    assert fruit_conflict_action(None, "pineapple", 0.5, 3) == "wait"
    assert fruit_conflict_action(("pineapple", 0.3, True), "pineapple", 0.5, 3) == "wait"
    assert fruit_conflict_action(("pineapple", 0.9, False), "pineapple", 0.5, 3) == "wait"
    # strong read of today's fruit -> switch to a set2 pick on the spot
    assert fruit_conflict_action(("pineapple", 0.6, True), "pineapple", 0.5, 3) == "switch"
    # strong read of a different fruit -> definitively not ours
    assert fruit_conflict_action(("banana", 0.8, True), "pineapple", 0.5, 3) == "reject_wrong"


# ------------------------------------------------------------------ selection
def kwargs(**over):
    base = dict(
        set1_label="cube", set2_label="pineapple",
        set1_remaining=4, set2_remaining=3,
        set1_min_conf=0.80, set2_min_margin_body=0.50, set2_min_margin_wide=0.70,
        set2_min_track_conf=0.50, min_obs=2,
        bans=[], now_s=100.0, ban_radius_m=0.22,
    )
    base.update(over)
    return base


def test_select_nearest_strong_target():
    objs = [
        obj(id=1, x=2.0, y=0.0, confidence=0.9),
        obj(id=2, x=0.5, y=0.0, confidence=0.9),
        obj(id=3, x=0.2, y=0.0, confidence=0.6),   # nearer but weak -> ignored
    ]
    got = select_hunt_target(objs, (0.0, 0.0), **kwargs())
    assert got is not None and got.id == 2


def test_select_respects_quota_and_bans():
    objs = [
        obj(id=1, x=0.5, y=0.0),
        obj(id=2, set_type=2, class_label="fruit_photo_cube", x=1.0, y=0.0,
            fruit_label="pineapple", fruit_confidence=0.6, fruit_label_source="body"),
    ]
    # set1 quota exhausted -> fruit wins even though farther
    got = select_hunt_target(objs, (0.0, 0.0), **kwargs(set1_remaining=0))
    assert got is not None and got.id == 2
    # banned position -> nothing
    got = select_hunt_target(
        [objs[0]], (0.0, 0.0), **kwargs(bans=[(0.5, 0.0, 0.0)]))
    assert got is None


def test_position_ban_expiry():
    bans = [(0.0, 0.0, 50.0), (1.0, 1.0, 0.0)]
    assert not position_banned(0.0, 0.0, bans, now_s=60.0, radius_m=0.22)  # expired
    assert position_banned(1.0, 1.0, bans, now_s=60.0, radius_m=0.22)      # permanent
    assert prune_bans(bans, 60.0) == [(1.0, 1.0, 0.0)]


# ------------------------------------------------------------------ align helpers
def test_align_axis_picks_worst_normalised_axis():
    assert align_translation_axis(0.0, 0.0, 0.03, 0.05) is None
    assert align_translation_axis(0.05, 0.0, 0.03, 0.05) == ("x", 0.05)
    assert align_translation_axis(0.0, -0.08, 0.03, 0.05) == ("y", -0.08)
    # x is 2x over tol, y only 1.2x -> x first
    axis = align_translation_axis(0.06, 0.06, 0.03, 0.05)
    assert axis == ("x", 0.06)


def test_select_body_candidate_nearest_to_grab():
    cands = [("cube", 0.60, 0.0, 0.9), ("cube", 0.22, 0.01, 0.8)]
    got = select_body_candidate(cands, (0.20, 0.0))
    assert got == ("cube", 0.22, 0.01, 0.8)
    assert select_body_candidate([("cube", 1.0, 0.0, 0.9)], (0.20, 0.0)) is None


# ------------------------------------------------------------------ motion helpers
def test_boundary_clamp_zeroes_outward_component():
    bounds = (-2.0, 2.0, -2.0, 2.0)
    # at the +x wall facing +x: forward command removed
    vx, vy = clamp_outward_field_velocity(0.2, 0.0, (1.9, 0.0, 0.0), bounds, 0.22, 0.03)
    assert vx == 0.0 and abs(vy) < 1e-9
    # same spot but driving backwards (inward) is untouched
    vx, vy = clamp_outward_field_velocity(-0.2, 0.0, (1.9, 0.0, 0.0), bounds, 0.22, 0.03)
    assert math.isclose(vx, -0.2)
    # facing +y at the +x wall: forward is not outward -> untouched
    vx, vy = clamp_outward_field_velocity(0.2, 0.0, (1.9, 0.0, math.pi / 2), bounds, 0.22, 0.03)
    assert math.isclose(vx, 0.2, abs_tol=1e-9)
    # centre of the field: nothing clamped
    vx, vy = clamp_outward_field_velocity(0.2, 0.1, (0.0, 0.0, 0.3), bounds, 0.22, 0.03)
    assert math.isclose(vx, 0.2) and math.isclose(vy, 0.1)


def test_rectify_collapses_staircase_to_single_L():
    from robot_planning.simple_hunter import rectify_rectilinear
    stair = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (1.0, 0.5), (1.0, 1.0)]
    out = rectify_rectilinear(stair, lambda a, b: True)
    assert out[0] == (0.0, 0.0) and out[-1] == (1.0, 1.0)
    assert len(out) == 3                     # start -> elbow -> end
    ex, ey = out[1]
    assert (ex, ey) in ((1.0, 0.0), (0.0, 1.0))


def test_rectify_keeps_staircase_when_L_blocked():
    from robot_planning.simple_hunter import rectify_rectilinear
    stair = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (1.0, 0.5)]

    def seg_free(a, b):
        # only allow the original 0.5 m hops: any longer leg is "blocked"
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) <= 0.5 + 1e-9

    out = rectify_rectilinear(stair, seg_free)
    assert out == stair                      # falls back to the raw route


def test_rectify_merges_collinear_runs():
    from robot_planning.simple_hunter import rectify_rectilinear
    line = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]
    out = rectify_rectilinear(line, lambda a, b: True)
    assert out == [(0.0, 0.0), (1.5, 0.0)]


def test_radial_standoff_on_robot_target_line():
    sx, sy = radial_standoff((0.0, 0.0), (1.0, 0.0), 0.375)
    assert math.isclose(sx, 0.625) and math.isclose(sy, 0.0)
    d = math.hypot(*(radial_standoff((0.3, -0.4), (1.2, 0.8), 0.375)[i] - (1.2, 0.8)[i]
                     for i in (0, 1)))
    assert math.isclose(d, 0.375, rel_tol=1e-6)
