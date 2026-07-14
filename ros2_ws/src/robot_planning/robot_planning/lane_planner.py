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
  4. string-pull the result to a few corner vias ending at the EXACT destination.

Pure Python (math only), ROS-free, unit-testable off-robot. All coordinates are field-frame
metres (REP-103), identical to the world model — no transforms.

The TARGET object (the thing being approached) must be excluded from `obstacles` by the
caller (e.g. drop it by id and drop anything within a small radius of the destination) so the
final leg into the stand-off is reachable; blacklisted-but-present objects stay in obstacles.
"""
from __future__ import annotations

import heapq
import math


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
        mode: str = "lane",
        taxi_final_direct_m: float = 0.35,
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
        self.mode = str(mode).strip().lower()
        self.taxi_final_direct_m = float(taxi_final_direct_m)

    def _taxi_mode(self) -> bool:
        return self.mode in ("taxi", "taxi_hybrid", "hybrid_taxi", "manhattan")

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

    def _simplify_taxi(self, pts):
        """Collapse only collinear runs. Unlike string-pull, this preserves right-angle lane turns."""
        dedup = []
        for p in pts:
            if not dedup or math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-9:
                dedup.append(p)
        pts = dedup
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        prev_dir = None
        for i in range(1, len(pts)):
            ax, ay = pts[i - 1]
            bx, by = pts[i]
            dx = bx - ax
            dy = by - ay
            if abs(dx) >= abs(dy):
                cur_dir = (1 if dx > 0 else -1 if dx < 0 else 0, 0)
            else:
                cur_dir = (0, 1 if dy > 0 else -1 if dy < 0 else 0)
            if prev_dir is not None and cur_dir != prev_dir:
                out.append(pts[i - 1])
            prev_dir = cur_dir
        out.append(pts[-1])
        return out

    def _nearest_clear_node_within(self, nodes, pt, obstacles, max_dist: float):
        cand = sorted(nodes.items(), key=lambda kv: math.hypot(kv[1][0] - pt[0], kv[1][1] - pt[1]))
        for ij, xy in cand:
            if math.hypot(xy[0] - pt[0], xy[1] - pt[1]) > max_dist:
                break
            if self._seg_free(xy, pt, obstacles):
                return ij
        return None

    # ------------------------------------------------------------ public: plan
    def plan(self, start, dest, obstacles):
        """Collision-free via list from `start` to `dest` (field xy, metres). Returns a list of
        (x,y) ENDING at the exact dest (robot's own position is NOT included), or None if no lane
        route exists (caller should then fall back to a direct goal). `obstacles` must already
        EXCLUDE the target being approached."""
        sx, sy = float(start[0]), float(start[1])
        dx, dy = float(dest[0]), float(dest[1])
        obstacles = [(float(ox), float(oy)) for ox, oy in obstacles]
        if math.hypot(dx - sx, dy - sy) < 1e-6:
            return [(dx, dy)]
        taxi = self._taxi_mode()
        # Direct shot already clear? In taxi-hybrid mode keep long travel on lane centres, but allow
        # the last short approach to the target/standoff.
        if self._seg_free((sx, sy), (dx, dy), obstacles) and (
                not taxi or math.hypot(dx - sx, dy - sy) <= self.taxi_final_direct_m):
            return [(dx, dy)]
        ox0, oy0 = self.infer_origin(obstacles)
        nodes = self._lane_nodes(ox0, oy0)
        if not nodes:
            return None
        if taxi:
            goal_ij = self._nearest_clear_node_within(
                nodes, (dx, dy), obstacles, max(self.taxi_final_direct_m, self.s * 0.5)
            )
        else:
            goal_ij = self._nearest_clear_node(nodes, (dx, dy), obstacles)
        starts = self._nearest_clear_nodes(nodes, (sx, sy), obstacles, self.k)
        if goal_ij is None or not starts:
            return None
        path_ij = self._astar(nodes, starts, goal_ij, obstacles, (sx, sy))
        if path_ij is None:
            return None
        pts = [(sx, sy)] + [nodes[ij] for ij in path_ij] + [(dx, dy)]
        pts = self._simplify_taxi(pts) if taxi else self._simplify(pts, obstacles)
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
