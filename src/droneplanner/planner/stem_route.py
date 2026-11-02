"""Route *between* mapped stems instead of reacting to them (TEST-PLAN N4).

WHAT THIS IS
The coarse corridor planner works on the DTM/DSM grid — 25 m cells on the demo AO
— and a 25 m cell cannot represent the 2.4 m gap between two 4.6 m-spaced trunks.
So the concealed centreline it produces is laid without any knowledge of where the
stems are, and measured against the dense manifest it passes **through** a trunk
(min clearance -0.033 m, 37 samples inside the airframe's requirement — against
the **37 strikes B2 actually flew**).

This pass fixes that without touching the concealment search: it keeps the coarse
route's shape and re-lays the *line* at stem resolution, inside the lateral
freedom the corridor already grants (``corridor_half_width_m``, A1).

WHY A SEPARATE PASS RATHER THAN A COST TERM
Both are needed and they answer different questions. A coarse cost term can only
say "this cell is cluttered" — it is the existing ``fields/stem_density.py``, and
N3 measured that routing on density cannot rescue the mission. Threading a
specific gap is a *geometry* problem at 0.5 m, and doing it at 25 m is not a
matter of tuning. Keeping it separate also keeps the concealment objective intact:
this pass may only move the line inside ground the corridor already blessed.

THE WINDOWED A*
A single fine grid over the whole route would be ~42 M cells. Instead the route is
refined in overlapping windows of a few vertices, each with its own local grid,
pinned at the window boundaries so the pieces compose. Within a window the search
is an 8-connected A* whose step cost is distance scaled by a clearance preference,
so it does not merely *clear* the stems — it prefers the middle of the gap. N4's
sweep is why: **5x the safety margin costs 1.3 % more distance**, so the right
policy is maximum clearance, not minimum.

🔴 **WHAT THIS DOES NOT DO.** It cannot invent lateral freedom. Where the corridor
grants 0 m — 32 of 282 vertices on the demo AO, and 2 of those have a trunk within
0.86 m — there is nothing to search, and that is N4's composition failure: *the
most concealed places are the least flyable*. ``min_half_width_m`` lets the caller
buy a working width at a stated concealment cost rather than failing silently, and
:class:`StemRouteReport` reports exactly how much corridor was borrowed.

⚠️ Every stem map is synthetic (see ``fields/stems``). This pass shows the planner
*can* use one; it does not show that one can be built.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

from droneplanner.fields.stems import StemMap

Point = tuple[float, float]

# Clearance the search actively prefers, beyond the hard requirement. N4: going
# from 0.50 m of air to 2.40 m costs 1.3 % more distance, so preferring the middle
# of a gap is nearly free and buys margin against a stem map that is only accurate
# to 1-2 m.
DEFAULT_PREFERRED_CLEARANCE_M = 2.4
# How hard to bend the path toward that preference, as a multiple of step length.
CLEARANCE_WEIGHT = 3.0


@dataclass(frozen=True)
class StemRouteReport:
    """What the refinement achieved, and what it had to borrow to achieve it."""

    windows: int
    windows_failed: int
    """Windows with no admissible path. Their original vertices are kept, so the
    route still exists — and is still unflyable there. Never silently dropped."""

    vertices_snapped: int
    """Window endpoints that sat inside the clearance requirement and had to be
    moved. Non-zero means the coarse route put a vertex where the aircraft cannot
    be — N4's composition failure, reported rather than papered over."""

    max_excursion_m: float
    """Furthest the refined line strays from the original centreline."""

    borrowed_beyond_corridor_m: float
    """How far the line goes outside the lateral freedom A1 granted. Zero means
    the concealment contract was honoured exactly; positive is a concealment cost
    the operator is paying for flyability."""

    length_ratio: float
    """Refined length / original length."""

    @property
    def clean(self) -> bool:
        return self.windows_failed == 0


def _astar(
    passable: np.ndarray,
    clearance: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    cell_m: float,
    preferred_m: float,
) -> list[tuple[int, int]] | None:
    """8-connected A* over a local grid, preferring the middle of a gap."""
    rows, cols = passable.shape
    # Penalty in [0, 1]: 0 where clearance meets the preference, 1 at zero gap.
    deficit = np.clip((preferred_m - clearance) / max(preferred_m, 1e-6), 0.0, 1.0)

    def h(rc: tuple[int, int]) -> float:
        return math.hypot(rc[0] - goal[0], rc[1] - goal[1]) * cell_m

    open_heap: list[tuple[float, tuple[int, int]]] = [(h(start), start)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    g: dict[tuple[int, int], float] = {start: 0.0}
    closed: set[tuple[int, int]] = set()
    steps = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    )  # fmt: skip

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        r0, c0 = cur
        for dr, dc, w in steps:
            r, c = r0 + dr, c0 + dc
            if not (0 <= r < rows and 0 <= c < cols) or not passable[r, c]:
                continue
            nxt = (r, c)
            if nxt in closed:
                continue
            step = w * cell_m * (1.0 + CLEARANCE_WEIGHT * float(deficit[r, c]))
            tentative = g[cur] + step
            if tentative < g.get(nxt, math.inf):
                g[nxt] = tentative
                came[nxt] = cur
                heapq.heappush(open_heap, (tentative + h(nxt), nxt))
    return None


def _segment_distance(px: np.ndarray, py: np.ndarray, a: Point, b: Point) -> np.ndarray:
    """Perpendicular distance from points to the segment a-b."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return np.hypot(px - ax, py - ay)
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / denom, 0.0, 1.0)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def refine_through_stems(
    pts: list[Point],
    stems: StemMap,
    *,
    required_m: float,
    corridor_half_width_m: list[float] | None = None,
    min_half_width_m: float = 4.0,
    max_half_width_m: float = 8.0,
    cell_m: float = 0.5,
    window_vertices: int = 4,
    preferred_m: float = DEFAULT_PREFERRED_CLEARANCE_M,
) -> tuple[list[Point], StemRouteReport]:
    """Re-lay ``pts`` through the gaps between ``stems``, inside the corridor.

    ``corridor_half_width_m`` is A1's per-vertex lateral freedom. Where it is
    narrower than ``min_half_width_m`` the search is given that minimum anyway —
    a route that cannot deviate cannot avoid a trunk standing on it — and the
    report says how much freedom was borrowed beyond what concealment granted.
    """
    n = len(pts)
    if n < 2 or len(stems) == 0:
        return list(pts), StemRouteReport(0, 0, 0, 0.0, 0.0, 1.0)
    if corridor_half_width_m is not None and len(corridor_half_width_m) != n:
        raise ValueError("corridor_half_width_m must be one value per vertex")
    granted = (
        [float(w) for w in corridor_half_width_m]
        if corridor_half_width_m is not None
        else [min_half_width_m] * n
    )
    # Clamped from BOTH sides, and the upper clamp is the load-bearing one.
    # A1 grants a median 50 m of freedom on the demo AO, and an A* given 50 m
    # does not refine the route — it *shortcuts* it, taking the straight chord
    # across every bend because that is shorter and still inside the union of the
    # corridors. The first run did exactly that: 287 vertices collapsed to 25, the
    # concealment shape was lost, and sub-clearance samples went UP (37 -> 52)
    # because the new line had never been checked against anything. The job here is
    # to dodge a trunk by a few metres, not to re-plan, so the search gets only as
    # much room as dodging needs.
    widths = [min(max(w, min_half_width_m), max_half_width_m) for w in granted]

    out: list[Point] = [pts[0]]
    windows = failed = snapped = 0
    # Carried in WORLD coordinates, never as a grid index: each window has its own
    # local origin, so an index from the previous window means a different place in
    # this one. (It raised IndexError immediately here; the same confusion has cost
    # this programme four silent bugs elsewhere.)
    pinned: Point | None = None
    step = max(2, window_vertices)
    for i in range(0, n - 1, step):
        j = min(i + step, n - 1)
        windows += 1
        seg = pts[i : j + 1]
        half = max(widths[i : j + 1])
        pad = half + 2.0
        xs = [p[0] for p in seg]
        ys = [p[1] for p in seg]
        x0, x1 = min(xs) - pad, max(xs) + pad
        y0, y1 = min(ys) - pad, max(ys) + pad
        nx = max(2, int((x1 - x0) / cell_m) + 1)
        ny = max(2, int((y1 - y0) / cell_m) + 1)
        gx = x0 + np.arange(nx) * cell_m
        gy = y0 + np.arange(ny) * cell_m
        mx, my = np.meshgrid(gx, gy)

        clearance = stems.clearance_at(mx, my)
        # A* only ever tests cell CENTRES, but the vehicle flies the segment
        # between them, which can pass closer to a trunk than either end does.
        # Inflate the requirement by the half-diagonal of one step so a move
        # between two passable centres is itself guaranteed to clear.
        cell_margin = cell_m * math.sqrt(2.0) / 2.0
        # Inside the corridor: within the granted half-width of the original line.
        near = np.full(mx.shape, np.inf)
        for a, b, wa, wb in zip(seg[:-1], seg[1:], widths[i:j], widths[i + 1 : j + 1], strict=True):
            d = _segment_distance(mx, my, a, b)
            near = np.minimum(near, d - max(wa, wb))
        passable = (clearance >= required_m + cell_margin) & (near <= 0.0)

        start = _rc_of(pinned if pinned is not None else pts[i], x0, y0, cell_m)
        goal = _rc_of(pts[j], x0, y0, cell_m)
        rows, cols = passable.shape
        start = (min(max(start[0], 0), rows - 1), min(max(start[1], 0), cols - 1))
        goal = (min(max(goal[0], 0), rows - 1), min(max(goal[1], 0), cols - 1))
        # A window endpoint can itself be inside the violation — on the demo AO two
        # zero-freedom vertices have a trunk within 0.86 m, which is exactly N4's
        # composition failure. Forcing such a cell "passable" would start the path
        # inside a trunk and quietly report success, so snap to the nearest cell
        # that really does clear and say that the vertex had to move.
        start, moved_s = _snap(start, passable)
        goal, moved_g = _snap(goal, passable)
        snapped += int(moved_s) + int(moved_g)
        path = _astar(passable, clearance, start, goal, cell_m, preferred_m)
        if path is None:
            failed += 1
            out.extend(pts[i + 1 : j + 1])
            pinned = None
            continue
        world = [(x0 + c * cell_m, y0 + r * cell_m) for r, c in path]
        # Simplify, then CHECK: the grid path is collision-free by construction,
        # a simplified polyline is not, because chords cut corners the search went
        # around. Tighten until it clears, and keep the raw path if nothing does.
        simplified = world
        for tol in (cell_m, cell_m / 2.0, cell_m / 4.0):
            candidate = _simplify(world, tol_m=tol)
            sx, sy = _dense(candidate, cell_m / 2.0)
            if float(stems.clearance_at(sx, sy).min()) >= required_m:
                simplified = candidate
                break
        out.extend(simplified[1:-1])
        out.append(simplified[-1])
        # Carry the (possibly snapped) endpoint into the next window so the two
        # halves join at the same point instead of teleporting back to the vertex.
        pinned = None if j >= n - 1 else simplified[-1]

    # --- what it cost -----------------------------------------------------------
    rx = np.array([p[0] for p in out])
    ry = np.array([p[1] for p in out])
    excursion = np.full(rx.shape, np.inf)
    beyond = np.full(rx.shape, np.inf)
    for k in range(n - 1):
        d = _segment_distance(rx, ry, pts[k], pts[k + 1])
        excursion = np.minimum(excursion, d)
        beyond = np.minimum(beyond, d - max(granted[k], granted[k + 1]))
    return out, StemRouteReport(
        windows=windows,
        windows_failed=failed,
        vertices_snapped=snapped,
        max_excursion_m=float(excursion.max()),
        borrowed_beyond_corridor_m=float(max(0.0, beyond.max())),
        length_ratio=_length(out) / max(_length(list(pts)), 1e-9),
    )


def _snap(rc: tuple[int, int], passable: np.ndarray) -> tuple[tuple[int, int], bool]:
    """Nearest passable cell to ``rc``, and whether it had to move.

    Used for window endpoints only. Falling back to ``rc`` when nothing is
    passable keeps the search well-formed; the window then fails and is reported.
    """
    if passable[rc]:
        return rc, False
    free = np.argwhere(passable)
    if free.size == 0:
        return rc, False
    d2 = (free[:, 0] - rc[0]) ** 2 + (free[:, 1] - rc[1]) ** 2
    best = free[int(np.argmin(d2))]
    return (int(best[0]), int(best[1])), True


def _dense(pts: list[Point], step_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Sample a polyline densely, for verifying a simplification did not cheat."""
    xs: list[float] = []
    ys: list[float] = []
    for a, b in pairwise(pts):
        d = math.dist(a, b)
        n = max(1, math.ceil(d / step_m))
        for k in range(n):
            f = k / n
            xs.append(a[0] + f * (b[0] - a[0]))
            ys.append(a[1] + f * (b[1] - a[1]))
    xs.append(pts[-1][0])
    ys.append(pts[-1][1])
    return np.asarray(xs), np.asarray(ys)


def _rc_of(p: Point, x0: float, y0: float, cell_m: float) -> tuple[int, int]:
    """(row, col) of a world point in a local window grid."""
    return (round((p[1] - y0) / cell_m), round((p[0] - x0) / cell_m))


def _length(pts: list[Point]) -> float:
    return sum(math.dist(a, b) for a, b in pairwise(pts))


def _simplify(pts: list[Point], tol_m: float) -> list[Point]:
    """Douglas-Peucker: drop vertices while staying within ``tol_m`` of the ORIGINAL.

    A 0.5 m grid path is one vertex per cell and the executor needs a polyline, not
    a raster trace. The bound has to be against the original polyline, not against
    each point's immediate neighbours: a collinearity filter drops a point that is
    within tolerance of the *already thinned* line, so the error compounds and the
    result can wander metres away. That is not a stylistic preference — the first
    version did exactly this and put the simplified line back through the trunks
    the A* had just avoided.
    """
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        lo, hi = stack.pop()
        if hi <= lo + 1:
            continue
        xs = np.array([p[0] for p in pts[lo + 1 : hi]])
        ys = np.array([p[1] for p in pts[lo + 1 : hi]])
        d = _segment_distance(xs, ys, pts[lo], pts[hi])
        k = int(np.argmax(d))
        if d[k] > tol_m:
            idx = lo + 1 + k
            keep[idx] = True
            stack.append((lo, idx))
            stack.append((idx, hi))
    return [p for p, k in zip(pts, keep, strict=True) if k]
