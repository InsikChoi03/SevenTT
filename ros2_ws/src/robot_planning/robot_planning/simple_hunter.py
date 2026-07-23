"""ROS-free helpers for the simple "trust strong positives only" hunter mission.

Philosophy (deliberately simpler than mission_fsm_node):
  - Only a HIGH-confidence positive on the world map is a target. No slots, no zones,
    no anchors, no retry hierarchies, no multi-frame classify voting.
  - If a strong target exists -> drive straight to it, align, pick, move on.
  - If none exists -> sweep the nearest grid node the wide camera has not seen yet.

Pure functions over duck-typed objects (works with robot_interfaces/Object or any
namespace exposing the same attributes), unit-testable without rclpy.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

Point = tuple[float, float]


# --------------------------------------------------------------- position bans
def prune_bans(
    bans: list[tuple[float, float, float]], now_s: float
) -> list[tuple[float, float, float]]:
    """Drop expired (x, y, expire_s) bans. expire_s <= 0 means permanent."""
    return [b for b in bans if b[2] <= 0.0 or b[2] > now_s]


def position_banned(
    x: float,
    y: float,
    bans: Sequence[tuple[float, float, float]],
    now_s: float,
    radius_m: float,
) -> bool:
    for bx, by, expire_s in bans:
        if expire_s > 0.0 and expire_s <= now_s:
            continue
        if math.hypot(x - bx, y - by) <= radius_m:
            return True
    return False


# --------------------------------------------------------------- target gating
def is_strong_set1(
    obj,
    set1_label: str,
    min_conf: float,
    min_obs: int,
) -> bool:
    """A set1 shape track we trust enough to commit a full approach+pick to.

    A track carrying ANY fruit evidence is a fruit cube, never a set1 shape — a
    distant fruit cube often reads as a plain "cube", so the attached fruit_label
    outranks the class vote.
    """
    if bool(getattr(obj, "blacklisted", False)):
        return False
    if str(getattr(obj, "fruit_label", "") or ""):
        return False
    if int(obj.set_type) != 1 or str(obj.class_label) != str(set1_label):
        return False
    if float(obj.confidence) < float(min_conf) or int(obj.n_obs) < int(min_obs):
        return False
    return True


def is_strong_set2(
    obj,
    set2_label: str,
    min_margin_body: float,
    min_margin_wide: float,
    min_track_conf: float,
    min_obs: int,
) -> bool:
    """A fruit cube already SigLIP-typed as today's fruit with a strong margin.

    Wide fruit reads are routing-only evidence in the main stack, so a wide-sourced
    label needs a higher margin than a body-sourced one to count as a true positive.

    A track with a fruit_label attached IS a fruit cube, full stop — even when the
    class vote froze it as a plain "cube" (set_type 1). Identity at distance is
    unreliable; the attached fruit evidence is what we trust. So set_type is
    deliberately NOT checked here.
    """
    if bool(getattr(obj, "blacklisted", False)):
        return False
    if str(getattr(obj, "fruit_label", "") or "") != str(set2_label):
        return False
    if float(obj.confidence) < float(min_track_conf) or int(obj.n_obs) < int(min_obs):
        return False
    margin = float(getattr(obj, "fruit_confidence", 0.0))
    source = str(getattr(obj, "fruit_label_source", "") or "")
    need = float(min_margin_body) if source == "body" else float(min_margin_wide)
    return margin >= need


def select_hunt_target(
    objects: Iterable,
    robot_xy: Point,
    *,
    set1_label: str,
    set2_label: str,
    set1_remaining: int,
    set2_remaining: int,
    set1_min_conf: float,
    set2_min_margin_body: float,
    set2_min_margin_wide: float,
    set2_min_track_conf: float,
    min_obs: int,
    bans: Sequence[tuple[float, float, float]],
    now_s: float,
    ban_radius_m: float,
):
    """Nearest strong-positive target still needed by the quota, or None."""
    rx, ry = robot_xy
    best = None
    best_d = math.inf
    for obj in objects:
        strong1 = set1_remaining > 0 and is_strong_set1(obj, set1_label, set1_min_conf, min_obs)
        strong2 = set2_remaining > 0 and is_strong_set2(
            obj, set2_label, set2_min_margin_body, set2_min_margin_wide,
            set2_min_track_conf, min_obs,
        )
        if not (strong1 or strong2):
            continue
        x, y = float(obj.x), float(obj.y)
        if position_banned(x, y, bans, now_s, ban_radius_m):
            continue
        d = math.hypot(x - rx, y - ry)
        if d < best_d:
            best_d = d
            best = obj
    return best


# --------------------------------------------------------------- fruit conflict
def fruit_conflict_action(
    verdict: tuple[str, float, bool] | None,
    set2_label: str,
    min_margin: float,
    set2_remaining: int,
) -> str:
    """Decide what to do when the set1 "cube" target turns out to be a fruit cube.

    The ALIGN servo found a fruit_photo_cube face at the grab point, so this is NOT
    a plain cube and must never be picked as set1. verdict is the freshest body
    SigLIP read (label, margin, face_visible) or None.

    Returns one of:
      "switch"       -> it IS today's set2 fruit with a strong margin: pick it as set2
      "wait"         -> no trustworthy read yet: hold still (align timeout still bounds)
      "reject_quota" -> set2 quota already full: this object is worthless, leave
      "reject_wrong" -> strong read of a DIFFERENT fruit: leave for good
    """
    if int(set2_remaining) <= 0:
        return "reject_quota"
    if verdict is None:
        return "wait"
    label, margin, face_visible = verdict
    if not face_visible or float(margin) < float(min_margin):
        return "wait"
    return "switch" if str(label) == str(set2_label) else "reject_wrong"


# --------------------------------------------------------------- align helpers
def align_translation_axis(
    ex: float,
    ey: float,
    fwd_tol: float,
    lat_tol: float,
) -> tuple[str, float] | None:
    """Pick the worst out-of-tolerance axis ('x'|'y', signed error) or None if aligned.

    Same semantics as the field FSM: one axis per pulse, never both, never rotation.
    """
    x_bad = abs(ex) > fwd_tol
    y_bad = abs(ey) > lat_tol
    if not x_bad and not y_bad:
        return None
    if x_bad and y_bad:
        # normalise by tolerance so a 2x-over lateral error beats a 1.1x forward error
        return ("x", ex) if abs(ex) / fwd_tol >= abs(ey) / lat_tol else ("y", ey)
    return ("x", ex) if x_bad else ("y", ey)


def select_body_candidate(
    candidates: Sequence[tuple[str, float, float, float]],
    grab_xy: Point,
    max_dist_m: float = 0.50,
) -> tuple[str, float, float, float] | None:
    """Nearest (label, bx, by, conf) body detection to the grab point, within max_dist."""
    gx, gy = grab_xy
    best = None
    best_d = float(max_dist_m)
    for label, bx, by, conf in candidates:
        d = math.hypot(bx - gx, by - gy)
        if d <= best_d:
            best_d = d
            best = (label, bx, by, conf)
    return best


# --------------------------------------------------------------- route shaping
def rectify_rectilinear(pts, seg_free) -> list:
    """Collapse a 4-connected A* staircase into long runs with single L corners.

    The raw lane route alternates 0.5 m x/y hops on a diagonal, forcing a pivot at
    every node ("wiggle walk"). Diagonal string-pull is unsafe (unmapped objects sit
    on the lattice points it cuts across), but an axis-aligned L stays on the lane
    lines the whole way — safe AND straight. Greedy: from each kept point jump to
    the farthest later point reachable via a collision-free one-elbow L (or straight).
    `seg_free(a, b)` checks a straight leg against the known obstacles.
    """
    n = len(pts)
    if n <= 2:
        return list(pts)
    out = [pts[0]]
    i = 0
    while i < n - 1:
        jumped = False
        for j in range(n - 1, i + 1, -1):
            a, b = pts[i], pts[j]
            if abs(a[0] - b[0]) < 1e-9 or abs(a[1] - b[1]) < 1e-9:
                if seg_free(a, b):
                    out.append(b)
                    i = j
                    jumped = True
                    break
                continue
            for elbow in ((b[0], a[1]), (a[0], b[1])):
                if seg_free(a, elbow) and seg_free(elbow, b):
                    out.append(elbow)
                    out.append(b)
                    i = j
                    jumped = True
                    break
            if jumped:
                break
        if not jumped:
            out.append(pts[i + 1])
            i += 1
    return out


# --------------------------------------------------------------- motion helpers
def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp_outward_field_velocity(
    vx: float,
    vy: float,
    pose: tuple[float, float, float],
    bounds: tuple[float, float, float, float],
    margin_m: float,
    guard_m: float,
) -> tuple[float, float]:
    """Zero the outward field-frame translation component near a wall.

    Rotation is untouched. (vx, vy) are base_link; pose is (x, y, theta) field.
    """
    x, y, theta = pose
    xmin, xmax, ymin, ymax = bounds
    ct, st = math.cos(theta), math.sin(theta)
    fx = ct * vx - st * vy
    fy = st * vx + ct * vy
    lo_x, hi_x = xmin + margin_m + guard_m, xmax - margin_m - guard_m
    lo_y, hi_y = ymin + margin_m + guard_m, ymax - margin_m - guard_m
    if (x <= lo_x and fx < 0.0) or (x >= hi_x and fx > 0.0):
        fx = 0.0
    if (y <= lo_y and fy < 0.0) or (y >= hi_y and fy > 0.0):
        fy = 0.0
    return ct * fx + st * fy, -st * fx + ct * fy


def radial_standoff(robot_xy: Point, target_xy: Point, dist_m: float) -> Point:
    """Point at dist_m from the target on the robot->target line (FSM fallback rule)."""
    rx, ry = robot_xy
    tx, ty = target_xy
    bearing = math.atan2(ty - ry, tx - rx)
    return (tx - dist_m * math.cos(bearing), ty - dist_m * math.sin(bearing))
