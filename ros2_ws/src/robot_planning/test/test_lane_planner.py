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


def _is_cardinal(a, b):
    return abs(a[0] - b[0]) < 1e-9 or abs(a[1] - b[1]) < 1e-9


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


def test_escape_segment_can_leave_only_the_obstacle_overlapping_start():
    lp = LanePlanner(block_radius=0.24)
    start = (0.20, 0.0)
    overlapping = (0.0, 0.0)

    assert lp.escape_segment_free(start, (0.40, 0.0), [overlapping])
    assert not lp.escape_segment_free(start, (0.00, 0.0), [overlapping])


def test_escape_segment_still_rejects_a_different_obstacle_ahead():
    lp = LanePlanner(block_radius=0.24)
    start = (0.20, 0.0)

    assert not lp.escape_segment_free(
        start,
        (0.60, 0.0),
        [(0.0, 0.0), (0.50, 0.0)],
    )


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


def test_grid_only_makes_start_and_goal_connectors_cardinal():
    lp = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        origin_mode="fixed",
        simplify=False,
    )
    start = (-1.40, -1.10)
    dest = (1.10, 1.40)
    path = lp.plan(start, dest, [], route_mode="grid_only")
    assert path is not None and path[-1] == dest
    assert all(_is_cardinal(a, b) for a, b in zip([start] + path, path))


def test_grid_only_ignores_legacy_direct_simplification():
    lp = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        origin_mode="fixed",
        simplify=True,
    )
    start = (-1.40, -1.10)
    dest = (1.10, 1.40)
    path = lp.plan(start, dest, [], route_mode="grid_only")
    assert path is not None and len(path) > 1
    assert all(_is_cardinal(a, b) for a, b in zip([start] + path, path))


def test_post_pick_entry_drives_directly_to_first_lane_then_stays_cardinal():
    lp = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        origin_mode="fixed",
        simplify=False,
    )
    start = (-1.40, -1.10)
    dest = (1.10, 1.40)

    path = lp.plan(start, dest, [], route_mode="post_pick_entry")

    assert path is not None and path[-1] == dest
    assert not _is_cardinal(start, path[0])
    assert all(_is_cardinal(a, b) for a, b in zip(path, path[1:]))
    _verify_clear(lp, start, path, [])


def test_object_approach_places_last_waypoint_on_lane_line():
    lp = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        origin_mode="fixed",
        simplify=False,
    )
    start = (-1.40, -1.10)
    stand_off = (1.10, 1.40)
    path = lp.plan(start, stand_off, [], route_mode="object_approach")
    assert path is not None and path[-1] == stand_off
    segments = list(zip([start] + path, path))
    assert all(_is_cardinal(a, b) for a, b in segments)
    assert path[-2] in ((stand_off[0], 1.25), (1.25, stand_off[1]))
    assert path[-2] != (1.25, 1.25)
    assert math.dist(*segments[-1]) <= 0.25


def test_object_approach_projects_recent_standoff_onto_nearest_lane():
    lp = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        origin_mode="fixed",
        simplify=False,
    )
    start = (-1.25, 0.75)
    stand_off = (-0.86, 0.62)

    path = lp.plan(start, stand_off, [], route_mode="object_approach")

    assert path is not None and path[-1] == stand_off
    assert path[-2] == (-0.86, 0.75)
    assert math.isclose(math.dist(path[-2], stand_off), 0.13, abs_tol=1e-9)


def test_object_standoff_is_one_of_four_lane_centres_around_target():
    lp = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        origin_mode="fixed",
        simplify=False,
    )
    start = (-1.25, -1.25)

    result = lp.plan_object_standoff(start, (0.0, 0.0), [])

    assert result is not None
    stand_off, path = result
    assert stand_off in {
        (-0.25, -0.25),
        (-0.25, 0.25),
        (0.25, -0.25),
        (0.25, 0.25),
    }
    assert path[-1] == stand_off
    assert all(_is_cardinal(a, b) for a, b in zip([start] + path, path))


def test_object_standoff_skips_a_blocked_lane_centre():
    lp = LanePlanner(
        spacing=0.5,
        bounds=(-2, 2, -2, 2),
        margin=0.22,
        block_radius=0.15,
        origin_mode="fixed",
        simplify=False,
    )
    blocked = (-0.25, -0.25)

    result = lp.plan_object_standoff((-1.25, -1.25), (0.0, 0.0), [blocked])

    assert result is not None
    stand_off, path = result
    assert stand_off != blocked
    assert path[-1] == stand_off


def test_orthogonal_connector_uses_unblocked_l_shape():
    lp = LanePlanner(block_radius=0.15)
    connector = lp._orthogonal_connector((0.0, 0.0), (1.0, 1.0), [(0.5, 0.0)])
    assert connector is not None
    assert connector[0] == [(0.0, 1.0), (1.0, 1.0)]


def test_orthogonal_connector_rejects_when_both_l_shapes_are_blocked():
    lp = LanePlanner(block_radius=0.15)
    connector = lp._orthogonal_connector(
        (0.0, 0.0), (1.0, 1.0), [(0.5, 0.0), (0.0, 0.5)]
    )
    assert connector is None
