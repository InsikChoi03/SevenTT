"""Regression tests for the stateless lane-graph waypoint planner (off-robot, pure math)."""
import math

from robot_planning.lane_planner import LanePlanner

S = 0.5
# a clean 50cm object grid on lattice phase (0,0), centres at multiples of 0.5 in [-1.5,1.5]
GRID = [(i * S, j * S) for i in range(-3, 4) for j in range(-3, 4)]


def _planner():
    return LanePlanner(spacing=0.5, bounds=(-2, 2, -2, 2), margin=0.22)


def _verify_clear(lp, start, path, obstacles):
    """Every segment of start+path must clear all obstacles by >= block_radius."""
    pts = [start] + list(path)
    for a, b in zip(pts, pts[1:]):
        d = lp._seg_min_clear(a, b, obstacles)
        assert d >= lp.block_r - 1e-9, f"segment {a}->{b} clearance {d:.3f} < {lp.block_r}"


def test_phase_inference():
    lp = _planner()
    ox, oy = lp.infer_origin(GRID)
    assert abs((ox % S)) < 1e-6 or abs((ox % S) - S) < 1e-6
    assert abs((oy % S)) < 1e-6 or abs((oy % S) - S) < 1e-6


def test_lane_nodes_are_midlines():
    lp = _planner()
    nodes = lp._lane_nodes(*lp.infer_origin(GRID))
    assert nodes
    for xy in nodes.values():
        d = min(math.hypot(xy[0] - ox, xy[1] - oy) for ox, oy in GRID)
        assert d >= lp.block_r


def test_straight_lane_is_direct():
    lp = _planner()
    p = lp.plan((-1.25, -1.25), (-0.25, -1.25), GRID)
    assert p is not None and len(p) == 1
    _verify_clear(lp, (-1.25, -1.25), p, GRID)


def test_corner_route_is_collision_free():
    lp = _planner()
    p = lp.plan((-1.25, -1.25), (1.25, 1.25), GRID)
    assert p is not None
    assert p[-1] == (1.25, 1.25)
    _verify_clear(lp, (-1.25, -1.25), p, GRID)


def test_object_drifted_into_lane_detours():
    lp = _planner()
    blocked = GRID + [(-1.25, 0.0)]
    p = lp.plan((-1.25, -1.25), (-1.25, 1.25), blocked)
    assert p is not None
    _verify_clear(lp, (-1.25, -1.25), p, blocked)
    assert any(abs(x - (-1.25)) > 1e-6 for x, _ in p)


def test_target_exclusion_makes_standoff_reachable():
    lp = _planner()
    target = (0.5, 0.0)
    standoff = (0.22, 0.0)
    obst = [o for o in GRID if o != target
            and math.hypot(o[0] - standoff[0], o[1] - standoff[1]) >= 0.30]
    p = lp.plan((-1.25, -1.25), standoff, obst)
    assert p is not None and p[-1] == standoff
    _verify_clear(lp, (-1.25, -1.25), p, obst)


def test_no_path_returns_none():
    lp = _planner()
    ring = GRID + [(-1.25 + lp.block_r, -1.25), (-1.25 - lp.block_r, -1.25),
                   (-1.25, -1.25 + lp.block_r), (-1.25, -1.25 - lp.block_r)]
    assert lp.plan((-1.25, -1.25), (1.25, 1.25), ring) is None


def test_coverage_visits_all_lane_nodes():
    lp = _planner()
    nodes = lp._lane_nodes(*lp.infer_origin(GRID))
    cov = lp.coverage_path((-1.75, -1.75), GRID)
    assert len(cov) == len(nodes)
    assert set(cov) == set(nodes.values())


def test_coverage_is_serpentine_by_row():
    lp = _planner()
    cov = lp.coverage_path((-1.75, -1.75), GRID)
    steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(cov, cov[1:])]
    assert all(s <= S + 1e-6 for s in steps)


def test_direct_shot_is_single_via():
    lp = _planner()
    p = lp.plan((-1.75, -1.75), (-1.75, 1.75), GRID)   # left wall lane, clear
    assert p == [(-1.75, 1.75)]


def test_simplify_disabled_keeps_four_connected_lane_vias():
    lp = LanePlanner(spacing=0.5, bounds=(-2, 2, -2, 2), margin=0.22, simplify=False)
    p = lp.plan((-1.25, -1.25), (1.25, 1.25), GRID)
    assert p is not None
    lane_vias = p[:-1]
    assert len(lane_vias) > 1
    for a, b in zip(lane_vias, lane_vias[1:]):
        dx = abs(b[0] - a[0])
        dy = abs(b[1] - a[1])
        assert (
            (abs(dx - S) < 1e-6 and dy < 1e-6)
            or (dx < 1e-6 and abs(dy - S) < 1e-6)
        )
