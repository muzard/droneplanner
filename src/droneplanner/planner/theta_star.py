"""Cost-aware Theta* any-angle planner (Software-Spec §5.3 v0).

Any-angle least-cost path over the composite cost surface (Preflight-Design §6A).
Theta* lets a vertex's parent be any earlier vertex, producing smooth non-blocky
routes instead of grid A*'s staircases.

The critical adaptation for a **cost surface** (as opposed to a binary-obstacle
grid): a straight-line shortcut to the grandparent is accepted only when its
*integrated* cost (mean cell cost x length) is no worse than routing through the
intermediate cell. Plain Theta*'s purely geometric line-of-sight test would
collapse to a straight line here, because with no hard obstacles every shortcut
is geometrically "clear" — so it would ignore the cost field entirely. The
cost-aware gate is what makes the route bend around exposed ridges toward
defilade while still cutting smooth corners through uniform-cost ground.

Distances are in grid cells (world/meter scaling is applied later from the grid
transform); only relative cost matters to the search result.
"""

from __future__ import annotations

import heapq
import math
from itertools import pairwise

import numpy as np

from droneplanner.raster_lines import bresenham

_NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class PlanningError(RuntimeError):
    """Raised when no path exists between start and goal."""


def _line_stats(
    cost: np.ndarray, obstacle: np.ndarray, ar: int, ac: int, br: int, bc: int
) -> tuple[bool, float]:
    """Return (blocked, integrated_cost) for the straight segment a->b.

    ``integrated_cost`` is mean cell cost x Euclidean length; ``blocked`` is True
    if the line crosses any obstacle cell.
    """
    cells = bresenham(ar, ac, br, bc)
    total = 0.0
    blocked = False
    for r, c in cells:
        if obstacle[r, c]:
            blocked = True
        total += float(cost[r, c])
    mean = total / len(cells)
    return blocked, math.hypot(br - ar, bc - ac) * mean


def plan_path(
    cost: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    obstacle: np.ndarray | None = None,
) -> list[tuple[int, int]]:
    """Plan an any-angle least-cost path of grid cells from ``start`` to ``goal``.

    ``obstacle`` (optional bool grid) marks impassable cells. Returns the list of
    any-angle vertices (turning points) including both endpoints. Raises
    ``PlanningError`` if the goal is unreachable.
    """
    rows, cols = cost.shape
    if obstacle is None:
        obstacle = np.zeros((rows, cols), dtype=bool)
    if obstacle[start] or obstacle[goal]:
        raise PlanningError("start or goal lies on an obstacle cell")

    c_min = max(float(cost.min()), 1e-6)

    def h(r: int, c: int) -> float:
        return math.hypot(goal[0] - r, goal[1] - c) * c_min

    g = np.full((rows, cols), math.inf, dtype=np.float64)
    parent = np.full((rows, cols, 2), -1, dtype=np.int32)
    closed = np.zeros((rows, cols), dtype=bool)

    g[start] = 0.0
    parent[start] = start
    heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(heap, (h(*start), counter, start))

    while heap:
        _, _, s = heapq.heappop(heap)
        if closed[s]:
            continue
        if s == goal:
            break
        closed[s] = True
        p = (int(parent[s][0]), int(parent[s][1]))

        for dr, dc in _NEIGHBORS:
            nr, nc = s[0] + dr, s[1] + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if closed[nr, nc] or obstacle[nr, nc]:
                continue

            # Grid-parent (short-edge) candidate.
            _, edge_sn = _line_stats(cost, obstacle, s[0], s[1], nr, nc)
            cand_g = g[s] + edge_sn
            cand_parent = s

            # Any-angle candidate: shortcut to s's parent, accepted only if it is
            # line-of-sight clear AND not more expensive than going via s.
            blocked_pn, edge_pn = _line_stats(cost, obstacle, p[0], p[1], nr, nc)
            if not blocked_pn:
                shortcut_g = g[p] + edge_pn
                if shortcut_g <= cand_g:
                    cand_g = shortcut_g
                    cand_parent = p

            if cand_g < g[nr, nc]:
                g[nr, nc] = cand_g
                parent[nr, nc] = cand_parent
                counter += 1
                heapq.heappush(heap, (cand_g + h(nr, nc), counter, (nr, nc)))

    if not math.isfinite(g[goal]):
        raise PlanningError("no path from start to goal")

    # Reconstruct the any-angle vertex chain.
    path: list[tuple[int, int]] = []
    node = goal
    while node != (int(parent[node][0]), int(parent[node][1])):
        path.append(node)
        node = (int(parent[node][0]), int(parent[node][1]))
    path.append(start)
    path.reverse()
    return _drop_collinear(path)


def _drop_collinear(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Remove redundant vertices that lie on a straight run (no turn)."""
    if len(path) <= 2:
        return path
    out = [path[0]]
    for i in range(1, len(path) - 1):
        a, b, c = out[-1], path[i], path[i + 1]
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if abs(cross) > 1e-9:  # a real turn at b — keep it
            out.append(b)
    out.append(path[-1])
    return out


def path_cost(cost: np.ndarray, path: list[tuple[int, int]]) -> float:
    """Total integrated cost of a vertex path (sum of any-angle edge costs)."""
    obstacle = np.zeros_like(cost, dtype=bool)
    return sum(_line_stats(cost, obstacle, a[0], a[1], b[0], b[1])[1] for a, b in pairwise(path))
