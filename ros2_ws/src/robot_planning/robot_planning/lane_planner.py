"""Stateless lane-graph waypoint planner for a 50 cm object grid + 40 cm robot.

Objects sit on a ~0.50 m lattice; a 0.40 m robot only fits centred on the LANE MIDLINES
(the 0.25 m-offset holes between four objects, ~1 cm clearance per side). So we do NOT
rasterise-and-inflate a fine occupancy grid (grid-snap error would spuriously block whole
lanes at the razor's edge). Instead:

  1. infer the lattice PHASE from the object coordinates (circular mean of coord mod s,
     robust to the 0/s wrap), with a quorum + fixed-origin fallback,
  2. build LANE NODES at the continuous hole-centres (max clearance by construction),
  3. run 4-connected A* whose edges are gated by exact point-to-SEGMENT distance to the
     float object coordinates (>= block_radius), with a clearance cost so equal-length
     routes prefer the roomiest lanes and thread a tight gate only as a last resort,
  4. connect the robot and destination according to the requested route policy: legacy direct
     connectors or cardinal L connectors whose elbow may lie anywhere on a lane line.

Pure Python (math only), ROS-free, unit-testable off-robot. All coordinates are field-frame
metres (REP-103), identical to the world model — no transforms.

The TARGET object (the thing being approached) must be excluded from `obstacles` by the
caller (e.g. drop it by id and drop anything within a small radius of the destination) so the
final leg into the stand-off is reachable; blacklisted-but-present objects stay in obstacles.
"""
from __future__ import annotations

import heapq
import math


def cardinal_segment_heading(
    start: tuple[float, float],
    end: tuple[float, float],
    axis_tolerance_m: float,
) -> float | None:
    """Return the field heading for an axis-aligned lane segment, else None."""
    dx = float(end[0]) - float(start[0])
    dy = float(end[1]) - float(start[1])
    if math.hypot(dx, dy) < 1e-6:
        return None
    tol = max(0.0, float(axis_tolerance_m))
    if abs(dy) <= tol and abs(dx) > abs(dy):
        return 0.0 if dx > 0.0 else math.pi
    if abs(dx) <= tol and abs(dy) > abs(dx):
        return math.pi / 2.0 if dy > 0.0 else -math.pi / 2.0
    return None


def _pt_seg_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Shortest distance from point (px,py) to segment a-b."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class LanePlanner:
    def __init__(
        self,
        *,
        spacing: float = 0.50,
        bounds: tuple[float, float, float, float] = (-2.0, 2.0, -2.0, 2.0),
        margin: float = 0.22,
        block_radius: float = 0.24,
        comfort_clear: float = 0.35,
        clearance_weight: float = 2.0,
        phase_tol: float = 0.08,
        phase_quorum: float = 0.70,
        min_objects: int = 3,
        origin_mode: str = "infer",
        origin_xy: tuple[float, float] = (0.0, 0.0),
        start_connect_k: int = 4,
        simplify: bool = True,
    ) -> None:
        self.s = float(spacing)
        xmin, xmax, ymin, ymax = bounds
        self.interior = (xmin + margin, xmax - margin, ymin + margin, ymax - margin)
        self.block_r = float(block_radius)
        self.comfort = float(comfort_clear)
        self.cw = float(clearance_weight)
        self.phase_tol = float(phase_tol)
        self.phase_quorum = float(phase_quorum)
        self.min_objects = int(min_objects)
        self.origin_mode = str(origin_mode)
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.k = int(start_connect_k)
        self.simplify = bool(simplify)

    # ------------------------------------------------------------ lattice phase
    def _circ_phase(self, coords: list[float]) -> float:
        """Circular mean of (coord mod s), robust to the 0<->s wraparound."""
        two_pi = 2.0 * math.pi
        sx = sum(math.sin(two_pi * c / self.s) for c in coords)
        cx = sum(math.cos(two_pi * c / self.s) for c in coords)
        if abs(sx) < 1e-9 and abs(cx) < 1e-9:
            return 0.0
        return (math.atan2(sx, cx) / two_pi * self.s) % self.s

    def _fixed_origin(self) -> tuple[float, float]:
        return (self.origin_xy[0] % self.s, self.origin_xy[1] % self.s)

    def infer_origin(self, objects: list[tuple[float, float]]) -> tuple[float, float]:
        """Infer the object-lattice phase (ox0,oy0) in [0,s). Falls back to the fixed origin
        when there are too few objects or they don't sit on a consistent lattice."""
        if self.origin_mode == "fixed" or len(objects) < self.min_objects:
            return self._fixed_origin()
        ox0 = self._circ_phase([o[0] for o in objects])
        oy0 = self._circ_phase([o[1] for o in objects])

        def frac_on(o0: float, coords: list[float]) -> float:
            good = sum(
                1 for c in coords
                if abs(((c - o0 + self.s / 2.0) % self.s) - self.s / 2.0) <= self.phase_tol
            )
            return good / max(1, len(coords))

        if (frac_on(ox0, [o[0] for o in objects]) >= self.phase_quorum
                and frac_on(oy0, [o[1] for o in objects]) >= self.phase_quorum):
            return (ox0, oy0)
        return self._fixed_origin()

    # ------------------------------------------------------------ lane nodes
    def _lane_nodes(self, ox0: float, oy0: float) -> dict[tuple[int, int], tuple[float, float]]:
        """Lane hole-centres = object lattice shifted half a cell, clipped to the interior."""
        xmin, xmax, ymin, ymax = self.interior
        base_x = ox0 + 0.5 * self.s   # first lane centre >= a lattice line + half-cell
        base_y = oy0 + 0.5 * self.s
        i0 = math.ceil((xmin - base_x) / self.s)
        i1 = math.floor((xmax - base_x) / self.s)
        j0 = math.ceil((ymin - base_y) / self.s)
        j1 = math.floor((ymax - base_y) / self.s)
        nodes: dict[tuple[int, int], tuple[float, float]] = {}
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                nodes[(i, j)] = (base_x + i * self.s, base_y + j * self.s)
        return nodes

    # ------------------------------------------------------------ collision / clearance
    def _seg_min_clear(self, a: tuple[float, float], b: tuple[float, float],
                       obstacles: list[tuple[float, float]]) -> float:
        if not obstacles:
            return float("inf")
        return min(_pt_seg_dist(ox, oy, a[0], a[1], b[0], b[1]) for ox, oy in obstacles)

    def _seg_free(self, a: tuple[float, float], b: tuple[float, float],
                  obstacles: list[tuple[float, float]]) -> bool:
        return self._seg_min_clear(a, b, obstacles) >= self.block_r

    def _edge_cost(self, a: tuple[float, float], b: tuple[float, float],
                   obstacles: list[tuple[float, float]]) -> float:
        """Base length + a penalty that grows as the lane narrows below comfort_clear, so A*
        prefers roomy lanes and only threads a barely-open gate as a costly last resort."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        clr = self._seg_min_clear(a, b, obstacles)
        pen = 0.0 if clr >= self.comfort else self.cw * (self.comfort - clr) / self.comfort
        return length * (1.0 + pen)

    # ------------------------------------------------------------ node connection
    def _nearest_clear_nodes(self, nodes, pt, obstacles, k):
        cand = sorted(nodes.items(), key=lambda kv: math.hypot(kv[1][0] - pt[0], kv[1][1] - pt[1]))
        out = []
        for ij, xy in cand:
            if self._seg_free(pt, xy, obstacles):
                out.append(ij)
                if len(out) >= k:
                    break
        return out

    def _nearest_clear_node(self, nodes, pt, obstacles):
        got = self._nearest_clear_nodes(nodes, pt, obstacles, 1)
        return got[0] if got else None

    def _orthogonal_connector(self, start, dest, obstacles):
        """Best collision-free cardinal connector, excluding start and ending at dest."""
        sx, sy = start
        dx, dy = dest
        if math.hypot(dx - sx, dy - sy) < 1e-9:
            return ([], 0.0)
        if abs(dx - sx) < 1e-9 or abs(dy - sy) < 1e-9:
            if not self._seg_free(start, dest, obstacles):
                return None
            return ([dest], self._edge_cost(start, dest, obstacles))

        candidates = []
        for elbow in ((dx, sy), (sx, dy)):
            if not self._seg_free(start, elbow, obstacles):
                continue
            if not self._seg_free(elbow, dest, obstacles):
                continue
            cost = (
                self._edge_cost(start, elbow, obstacles)
                + self._edge_cost(elbow, dest, obstacles)
            )
            candidates.append(([elbow, dest], cost))
        return min(candidates, key=lambda item: item[1]) if candidates else None

    def _endpoint_connectors(self, nodes, point, obstacles, *, from_point, allow_direct):
        """Map lane-node ids to (connector points, cost) for one route endpoint."""
        connectors = {}
        for ij, node in nodes.items():
            start, dest = (point, node) if from_point else (node, point)
            if allow_direct:
                if not self._seg_free(start, dest, obstacles):
                    continue
                connector = ([dest], self._edge_cost(start, dest, obstacles))
            else:
                connector = self._orthogonal_connector(start, dest, obstacles)
                if connector is None:
                    continue
            connectors[ij] = connector
        return connectors

    # ------------------------------------------------------------ A*
    def _astar(self, nodes, starts, goal_ij, obstacles, start_xy):
        gx, gy = nodes[goal_ij]
        gi, gj = goal_ij

        def h(ij):
            return (abs(ij[0] - gi) + abs(ij[1] - gj)) * self.s

        g_cost = {}
        came = {}
        pq = []
        for ij in starts:
            c = self._edge_cost(start_xy, nodes[ij], obstacles)
            if ij not in g_cost or c < g_cost[ij]:
                g_cost[ij] = c
                came[ij] = None
                heapq.heappush(pq, (c + h(ij), c, ij))
        while pq:
            _, c, ij = heapq.heappop(pq)
            if c > g_cost.get(ij, float("inf")):
                continue
            if ij == goal_ij:
                path = [ij]
                while came[ij] is not None:
                    ij = came[ij]
                    path.append(ij)
                path.reverse()
                return path
            i, j = ij
            for nij in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if nij not in nodes:
                    continue
                if not self._seg_free(nodes[ij], nodes[nij], obstacles):
                    continue
                nc = c + self._edge_cost(nodes[ij], nodes[nij], obstacles)
                if nc < g_cost.get(nij, float("inf")):
                    g_cost[nij] = nc
                    came[nij] = ij
                    heapq.heappush(pq, (nc + h(nij), nc, nij))
        return None

    def _astar_with_endpoint_connectors(
        self, nodes, start_connectors, goal_connectors, obstacles
    ):
        """Find the cheapest 4-connected lane path including both endpoint connectors."""
        g_cost = {}
        came = {}
        pq = []
        starts = sorted(start_connectors.items(), key=lambda item: item[1][1])[:self.k]
        for ij, (_, connector_cost) in starts:
            g_cost[ij] = connector_cost
            came[ij] = None
            heapq.heappush(pq, (connector_cost, ij))

        best_total = float("inf")
        best_goal = None
        while pq:
            cost, ij = heapq.heappop(pq)
            if cost > g_cost.get(ij, float("inf")) or cost >= best_total:
                continue
            if ij in goal_connectors:
                total = cost + goal_connectors[ij][1]
                if total < best_total:
                    best_total = total
                    best_goal = ij
            i, j = ij
            for nij in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if nij not in nodes or not self._seg_free(nodes[ij], nodes[nij], obstacles):
                    continue
                next_cost = cost + self._edge_cost(nodes[ij], nodes[nij], obstacles)
                if next_cost < g_cost.get(nij, float("inf")):
                    g_cost[nij] = next_cost
                    came[nij] = ij
                    heapq.heappush(pq, (next_cost, nij))

        if best_goal is None:
            return None
        path = [best_goal]
        while came[path[-1]] is not None:
            path.append(came[path[-1]])
        path.reverse()
        return path

    # ------------------------------------------------------------ simplify
    def _simplify(self, pts, obstacles):
        """Greedy string-pull: from each kept point jump to the FURTHEST later point still
        reachable by a collision-free straight segment (subsumes collinear-corner collapse)."""
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i = 0
        n = len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not self._seg_free(pts[i], pts[j], obstacles):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _drop_duplicate_points(self, pts):
        out = []
        for p in pts:
            if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-9:
                out.append(p)
        return out

    def plan_object_standoff(self, start, target, obstacles, route_mode="grid_only"):
        """Choose the cheapest reachable lane-hole centre surrounding an object."""
        sx, sy = float(start[0]), float(start[1])
        tx, ty = float(target[0]), float(target[1])
        obstacles = [(float(ox), float(oy)) for ox, oy in obstacles]
        ox0, oy0 = self.infer_origin(obstacles + [(tx, ty)])
        nodes = self._lane_nodes(ox0, oy0)
        if not nodes:
            return None

        lattice_i = int(round((tx - ox0) / self.s))
        lattice_j = int(round((ty - oy0) / self.s))
        candidate_ids = (
            (lattice_i - 1, lattice_j - 1),
            (lattice_i - 1, lattice_j),
            (lattice_i, lattice_j - 1),
            (lattice_i, lattice_j),
        )
        candidates = []
        for ij in candidate_ids:
            stand_off = nodes.get(ij)
            if stand_off is None:
                continue
            path = self.plan(
                (sx, sy), stand_off, obstacles, route_mode=route_mode
            )
            if not path:
                continue
            points = [(sx, sy)] + path
            route_cost = sum(
                self._edge_cost(a, b, obstacles)
                for a, b in zip(points, points[1:])
            )
            candidates.append((route_cost, stand_off[0], stand_off[1], path))

        if not candidates:
            return None
        _, stand_x, stand_y, path = min(candidates, key=lambda item: item[:3])
        return (stand_x, stand_y), path

    # ------------------------------------------------------------ public: plan
    def plan(self, start, dest, obstacles, route_mode="legacy"):
        """Collision-free via list from `start` to `dest` (field xy, metres). Returns a list of
        (x,y) ENDING at the exact dest (robot's own position is NOT included), or None if no lane
        route exists. `grid_only` makes both endpoint connectors cardinal; `post_pick_entry`
        connects the pickup pose directly to the selected first lane waypoint, then keeps every
        remaining segment cardinal. `object_approach` also keeps the final pre-stand-off waypoint
        on a lane line instead of forcing it to a 0.5 m grid intersection. `legacy` preserves the
        original direct endpoint behavior. `obstacles` must already EXCLUDE the target being
        approached."""
        sx, sy = float(start[0]), float(start[1])
        dx, dy = float(dest[0]), float(dest[1])
        obstacles = [(float(ox), float(oy)) for ox, oy in obstacles]
        if route_mode not in {"legacy", "grid_only", "post_pick_entry", "object_approach"}:
            raise ValueError(f"unsupported lane route mode: {route_mode}")
        if math.hypot(dx - sx, dy - sy) < 1e-6:
            return [(dx, dy)]
        # Direct shot already clear? then no vias needed. Disabled with simplify=false so tests can
        # observe the raw 4-connected lane route instead of a straight shortcut across the field.
        if (
            route_mode == "legacy"
            and self.simplify
            and self._seg_free((sx, sy), (dx, dy), obstacles)
        ):
            return [(dx, dy)]
        ox0, oy0 = self.infer_origin(obstacles)
        nodes = self._lane_nodes(ox0, oy0)
        if not nodes:
            return None
        if route_mode != "legacy":
            start_connectors = self._endpoint_connectors(
                nodes,
                (sx, sy),
                obstacles,
                from_point=True,
                allow_direct=route_mode == "post_pick_entry",
            )
            goal_connectors = self._endpoint_connectors(
                nodes,
                (dx, dy),
                obstacles,
                from_point=False,
                allow_direct=False,
            )
            if not start_connectors or not goal_connectors:
                return None
            path_ij = None
            selected_goal_ij = None
            for goal_ij, goal_connector in sorted(
                goal_connectors.items(), key=lambda item: item[1][1]
            ):
                path_ij = self._astar_with_endpoint_connectors(
                    nodes, start_connectors, {goal_ij: goal_connector}, obstacles
                )
                if path_ij is not None:
                    selected_goal_ij = goal_ij
                    break
            if path_ij is None or selected_goal_ij is None:
                return None
            pts = [(sx, sy)]
            pts.extend(start_connectors[path_ij[0]][0])
            pts.extend(nodes[ij] for ij in path_ij[1:])
            pts.extend(goal_connectors[selected_goal_ij][0])
            return self._drop_duplicate_points(pts)[1:]

        goal_ij = self._nearest_clear_node(nodes, (dx, dy), obstacles)
        starts = self._nearest_clear_nodes(nodes, (sx, sy), obstacles, self.k)
        if goal_ij is None or not starts:
            return None
        path_ij = self._astar(nodes, starts, goal_ij, obstacles, (sx, sy))
        if path_ij is None:
            return None
        pts = [(sx, sy)] + [nodes[ij] for ij in path_ij] + [(dx, dy)]
        pts = self._drop_duplicate_points(pts)
        if self.simplify:
            pts = self._simplify(pts, obstacles)
        return pts[1:]   # drop the robot's own start point

    # ------------------------------------------------------------ public: coverage sweep
    def coverage_path(self, start, obstacles):
        """Systematic boustrophedon (lawn-mower) ordering of the free lane nodes for SEARCH:
        sweep row by row, alternating direction, starting from the corner nearest `start`. The
        FSM drives to each waypoint (via plan() for collision-free travel) and aborts as soon as
        a phase target is found. Returns an ordered list of (x,y); [] if no lattice."""
        obstacles = [(float(ox), float(oy)) for ox, oy in obstacles]
        ox0, oy0 = self.infer_origin(obstacles)
        nodes = self._lane_nodes(ox0, oy0)
        if not nodes:
            return []
        sx, sy = float(start[0]), float(start[1])
        rows = sorted({j for (_, j) in nodes})
        cols = sorted({i for (i, _) in nodes})
        row_order = self._contiguous_from_nearest(rows, sy, lambda j: nodes[(cols[0], j)][1])
        seq = []
        forward = nodes[(cols[0], row_order[0])][0] <= sx   # first row sweeps away from the robot's side
        for j in row_order:
            row_nodes = sorted((nodes[(i, j)] for i in cols if (i, j) in nodes), key=lambda p: p[0])
            if not forward:
                row_nodes.reverse()
            seq.extend(row_nodes)
            forward = not forward
        return seq

    @staticmethod
    def _contiguous_from_nearest(rows, ref, y_of):
        """Return rows ordered as a contiguous sweep starting from the one nearest `ref`."""
        if not rows:
            return []
        start = min(rows, key=lambda j: abs(y_of(j) - ref))
        # sweep in the direction that covers the most first (toward the majority side)
        below = [j for j in rows if j < start]
        above = [j for j in rows if j > start]
        order = [start]
        # go toward the larger group first, then the other
        if len(above) >= len(below):
            order += sorted(above)
            order += sorted(below, reverse=True)
        else:
            order += sorted(below, reverse=True)
            order += sorted(above)
        return order
