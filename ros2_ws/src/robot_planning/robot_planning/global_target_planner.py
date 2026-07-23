"""Pure helpers for the map-driven GLOBAL target mode (no ROS imports).

Global mode replaces the Z1->Z4 zone tour: every CONFIRMED target already on the
world map (today's Set1 shape, or a Set2 cube SigLIP-typed as today's fruit) is
visited in minimum-total-path order, wherever it sits on the field. Only when
the map holds no candidate at all does the robot fall back to sweeping the fixed
0.5 m object-grid nodes its wide camera has not covered yet ("unseen" nodes).

Everything here is a pure function over plain tuples so the mission FSM stays
thin and the routing/coverage rules are unit-testable without rclpy.
"""
from __future__ import annotations

import math
from itertools import permutations
from typing import Hashable, Iterable, Sequence

Point = tuple[float, float]


def _path_length(start: Point, pts: Sequence[Point]) -> float:
    total = 0.0
    px, py = start
    for x, y in pts:
        total += math.hypot(x - px, y - py)
        px, py = x, y
    return total


def order_targets_min_path(
    start_xy: Point,
    targets: Iterable[tuple[Hashable, float, float]],
    brute_force_limit: int = 7,
) -> list[Hashable]:
    """Visit order (target keys) minimising the OPEN tour length from start.

    Exact for up to `brute_force_limit` targets (candidate counts are tiny: at
    most 7 objects remain pickable), greedy nearest-neighbour beyond that.
    """
    items = [(key, (float(x), float(y))) for key, x, y in targets]
    if not items:
        return []
    if len(items) == 1:
        return [items[0][0]]
    if len(items) <= brute_force_limit:
        best_order: tuple[int, ...] | None = None
        best_len = math.inf
        for perm in permutations(range(len(items))):
            length = _path_length(start_xy, [items[i][1] for i in perm])
            if length < best_len:
                best_len = length
                best_order = perm
        assert best_order is not None
        return [items[i][0] for i in best_order]
    remaining = list(items)
    order: list[Hashable] = []
    cx, cy = start_xy
    while remaining:
        j = min(
            range(len(remaining)),
            key=lambda i: math.hypot(remaining[i][1][0] - cx, remaining[i][1][1] - cy),
        )
        key, (cx, cy) = remaining.pop(j)
        order.append(key)
    return order


def grid_nodes(
    rows: int,
    cols: int,
    origin_x: float,
    origin_y: float,
    spacing: float,
) -> list[Point]:
    """Build the fixed field-grid coordinates (row-major index order) where cubes can sit."""
    return [
        (origin_x + col * spacing, origin_y + row * spacing)
        for row in range(max(0, rows))
        for col in range(max(0, cols))
    ]


def nodes_in_fov(
    pose: tuple[float, float, float],
    nodes: Sequence[Point],
    fov_rad: float,
    max_range_m: float,
    min_range_m: float = 0.0,
) -> set[int]:
    """Collect indices of grid nodes inside the forward camera wedge at `pose` (x, y, theta).

    A node counts as "seen" when it lies within the horizontal FOV cone and the
    trusted detection range — nearer than max_range_m (identity is unreliable
    further out) but beyond min_range_m (the base occludes the floor right under
    the robot).
    """
    x, y, theta = pose
    half = max(0.0, fov_rad) * 0.5
    seen: set[int] = set()
    for i, (nx, ny) in enumerate(nodes):
        dx, dy = nx - x, ny - y
        d = math.hypot(dx, dy)
        if d < min_range_m or d > max_range_m:
            continue
        bearing = math.atan2(dy, dx) - theta
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if abs(bearing) <= half:
            seen.add(i)
    return seen


def nodes_in_ellipse_fov(
    pose: tuple[float, float, float],
    nodes: Sequence[Point],
    forward_radius_m: float,
    lateral_radius_m: float,
    center_forward_m: float,
    min_range_m: float = 0.0,
) -> set[int]:
    """Collect grid nodes inside the calibrated, forward-offset wide-camera footprint."""
    x, y, theta = pose
    ct, st = math.cos(theta), math.sin(theta)
    forward_radius = max(1e-6, float(forward_radius_m))
    lateral_radius = max(1e-6, float(lateral_radius_m))
    min_range = max(0.0, float(min_range_m))
    seen: set[int] = set()
    for index, (node_x, node_y) in enumerate(nodes):
        dx, dy = node_x - x, node_y - y
        if math.hypot(dx, dy) < min_range:
            continue
        forward = ct * dx + st * dy
        lateral = -st * dx + ct * dy
        normalized = (
            ((forward - center_forward_m) / forward_radius) ** 2
            + (lateral / lateral_radius) ** 2
        )
        if normalized <= 1.0:
            seen.add(index)
    return seen


def best_observation_pose(
    robot_pose: tuple[float, float, float],
    nodes: Sequence[Point],
    seen: set[int],
    candidate_positions: Sequence[Point],
    forward_radius_m: float,
    lateral_radius_m: float,
    center_forward_m: float,
    min_range_m: float = 0.0,
) -> tuple[float, float, float, set[int]] | None:
    """Choose a safe viewpoint/heading that covers most unseen slots, then least travel."""
    unseen = [index for index in range(len(nodes)) if index not in seen]
    if not unseen:
        return None
    robot_x, robot_y, robot_theta = robot_pose
    best = None
    best_key = None
    for candidate_x, candidate_y in candidate_positions:
        headings = [
            math.atan2(nodes[index][1] - candidate_y, nodes[index][0] - candidate_x)
            for index in unseen
            if math.hypot(
                nodes[index][0] - candidate_x,
                nodes[index][1] - candidate_y,
            ) > 1e-6
        ]
        for heading in headings:
            covered = nodes_in_ellipse_fov(
                (candidate_x, candidate_y, heading),
                nodes,
                forward_radius_m,
                lateral_radius_m,
                center_forward_m,
                min_range_m,
            ) - seen
            if not covered:
                continue
            travel = math.hypot(candidate_x - robot_x, candidate_y - robot_y)
            turn = abs(math.atan2(
                math.sin(heading - robot_theta),
                math.cos(heading - robot_theta),
            ))
            key = (len(covered), -travel, -turn)
            if best_key is None or key > best_key:
                best_key = key
                best = (candidate_x, candidate_y, heading, covered)
    return best


def next_zigzag_observation_pose(
    nodes: Sequence[Point],
    seen: set[int],
    waypoints: Sequence[Point],
    start_index: int,
    forward_radius_m: float,
    lateral_radius_m: float,
    center_forward_m: float,
    min_range_m: float = 0.0,
    skip_fully_seen: bool = True,
) -> tuple[int, float, float, float, set[int]] | None:
    """Continue an ordered lawn-mower route.

    Dynamic coverage paths may skip waypoints whose view is already covered.  A configured
    geometric route can disable that shortcut so its connector points and turn order remain
    intact even while Wide frames mark nodes seen during travel.
    """
    count = len(waypoints)
    if count <= 0:
        return None
    for offset in range(count):
        index = (int(start_index) + offset) % count
        x, y = waypoints[index]
        if count == 1:
            heading = 0.0
        elif index + 1 < count:
            nx, ny = waypoints[index + 1]
            heading = math.atan2(ny - y, nx - x)
        else:
            px, py = waypoints[index - 1]
            heading = math.atan2(y - py, x - px)
        covered = nodes_in_ellipse_fov(
            (float(x), float(y), heading),
            nodes,
            forward_radius_m,
            lateral_radius_m,
            center_forward_m,
            min_range_m,
        ) - seen
        if covered or not skip_fully_seen:
            return index, float(x), float(y), heading, covered
    return None


def next_unseen_node(
    robot_xy: Point,
    nodes: Sequence[Point],
    seen: set[int],
) -> tuple[int, Point] | None:
    """Nearest grid node the camera has not covered yet, or None when all seen."""
    best: tuple[int, Point] | None = None
    best_d = math.inf
    rx, ry = robot_xy
    for i, (nx, ny) in enumerate(nodes):
        if i in seen:
            continue
        d = math.hypot(nx - rx, ny - ry)
        if d < best_d:
            best_d = d
            best = (i, (nx, ny))
    return best
