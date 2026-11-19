"""Flyability from fine voxels, aggregated to the corridor grid (SHORTFALLS item 2).

WHY THIS EXISTS
``pipeline._load_flyability`` used to voxelize the point cloud on the corridor
grid itself — 10-25 m columns. A column is occupied at a height if *any* return
in it is, so a single trunk fills a 25 m column at every trunk-space level and
the free window collapses to nothing. Measured on synthetic stands (trunks +
porous crowns 8-18 m + ground; the 25 m vs 0.5 m pair is pinned in
``tests/unit/test_forest_fine.py``):

| stems/ha | 25 m columns | 5 m | 1 m | **0.5 m** (fine) |
|---|---|---|---|---|
| 200 | **0 % flyable, 100 % NON_NAVIGABLE** | 58 % | 89 % | 90 % |
| 700 | **0 % flyable, 100 % NON_NAVIGABLE** | 10 % | 64 % | 74 % |

So the coarse field was not "optimistic" as SHORTFALLS used to say: with any
real point cloud it would have turned **every treed cell into an obstacle** and
sent the route out of the forest it is supposed to hide in. (It is optimistic in
one narrow sense — a free coarse cell reports clearance of at least one cell
size — but the occupancy collapse dominates.)

HOW
Each corridor cell that holds returns is voxelized on its own at
``fine_m`` (default :data:`FINE_VOXEL_M`, the spec's 0.25-0.5 m), with a
``halo_m`` border taken from its neighbours so the 3D clearance at the cell's
edge still sees a trunk just across it. :func:`build_flyability` runs on that
block unchanged; the inner block is then aggregated:

* class — FLYABLE when at least :data:`FLYABLE_SHARE` of the fine columns are
  flyable, NON_NAVIGABLE when fewer than :data:`NAVIGABLE_SHARE` are navigable
  at all, otherwise MARGINAL;
* clearance, free window, navigable band, best altitude — medians over the
  navigable fine columns.

Cells with no returns keep exactly what the coarse field gives them, so a
mission whose cloud covers part of the AO changes only where it has data.

⚠️ **Share, not connectivity.** A cell can be 90 % free and still be crossed by
an impassable line of trunks; the coarse grid cannot say whether the route gets
*through*. That is the stem router's job (``planner/stem_route.py``) and the
onboard planner's. The shares are judgement, not calibration.
"""

from __future__ import annotations

import math

import numpy as np

from droneplanner.forest.navigability import (
    FLYABLE,
    MARGINAL,
    NON_NAVIGABLE,
    FlyabilityConfig,
    FlyabilityField,
    build_flyability,
)
from droneplanner.forest.voxelize import voxelize
from droneplanner.model.grid import Grid

FINE_VOXEL_M = 0.5
"""Target horizontal voxel size; the actual one divides the corridor cell evenly."""

HALO_M = 3.0
"""Neighbour border voxelized with each cell: covers the flyable clearance
(airframe radius + state error + margin ≈ 1 m) several times over."""

FLYABLE_SHARE = 0.5
NAVIGABLE_SHARE = 0.2

_CLASS_COST = np.array([0.0, 0.5, 1.0], dtype=np.float32)


def build_flyability_fine(
    xyz: np.ndarray,
    grid: Grid,
    dtm: np.ndarray,
    config: FlyabilityConfig,
    *,
    voxel_size_z: float = 0.5,
    fine_m: float = FINE_VOXEL_M,
    halo_m: float = HALO_M,
) -> FlyabilityField:
    """Flyability on ``grid`` from fine voxels (see the module docstring).

    ``grid`` must be north-up (``b == d == 0``), as every planner grid is.
    """
    a, b, c0, d, e, f0 = grid.transform
    if b != 0.0 or d != 0.0 or e >= 0.0:
        raise ValueError("fine flyability needs a north-up grid")
    if fine_m <= 0.0:
        raise ValueError("fine_m must be positive")
    z_min = float(xyz[:, 2].min()) if len(xyz) else 0.0
    z_max = (float(xyz[:, 2].max()) if len(xyz) else 0.0) + voxel_size_z

    coarse_vox = voxelize(xyz, grid, z_min=z_min, z_max=z_max, voxel_size_z=voxel_size_z)
    coarse = build_flyability(coarse_vox, config, ground_level=coarse_vox.ground_level(dtm))
    if len(xyz) == 0:
        return coarse

    cell = a  # x cell size; e = -cell for a square grid
    n = max(1, round(cell / fine_m))
    fine = cell / n
    # The halo is drawn from the 8 neighbours only, so it cannot exceed a cell.
    h = min(math.ceil(halo_m / fine), n)
    side = n + 2 * h

    ci = np.floor((xyz[:, 0] - c0) / a).astype(np.int64)
    ri = np.floor((xyz[:, 1] - f0) / e).astype(np.int64)
    inside = (ri >= 0) & (ri < grid.rows) & (ci >= 0) & (ci < grid.cols)
    pts, ri, ci = xyz[inside], ri[inside], ci[inside]
    lin = ri * grid.cols + ci
    order = np.argsort(lin, kind="stable")
    pts, lin = pts[order], lin[order]
    cells, starts = np.unique(lin, return_index=True)
    ends = np.r_[starts[1:], len(lin)]
    span = {int(k): (int(s), int(t)) for k, s, t in zip(cells, starts, ends, strict=True)}

    klass = coarse.navigable_class.copy()
    clear = coarse.best_clearance_m.copy()
    window = coarse.trunk_window_m.copy()
    max_alt = coarse.max_flyable_altitude_m.copy()
    floor = coarse.nav_floor_m.copy()
    ceil = coarse.nav_ceil_m.copy()

    for k in cells:
        r, c = divmod(int(k), grid.cols)
        x_lo = c0 + c * cell - h * fine
        y_hi = f0 + r * e + h * fine
        block = []
        for rr in (r - 1, r, r + 1):
            for cc in (c - 1, c, c + 1):
                sl = span.get(rr * grid.cols + cc) if 0 <= rr < grid.rows else None
                if sl is not None and 0 <= cc < grid.cols:
                    block.append(pts[sl[0] : sl[1]])
        local_pts = np.vstack(block)
        local = Grid(
            transform=(fine, 0.0, x_lo, 0.0, -fine, y_hi),
            rows=side,
            cols=side,
            crs=grid.crs,
        )
        # Each fine column takes the DTM of the corridor cell it lies in.
        rows_c = np.clip(r + (np.arange(side) - h) // n, 0, grid.rows - 1)
        cols_c = np.clip(c + (np.arange(side) - h) // n, 0, grid.cols - 1)
        local_dtm = dtm[np.ix_(rows_c, cols_c)]
        vox = voxelize(local_pts, local, z_min=z_min, z_max=z_max, voxel_size_z=voxel_size_z)
        ff = build_flyability(vox, config, ground_level=vox.ground_level(local_dtm))
        inner = (slice(h, h + n), slice(h, h + n))
        cls = ff.navigable_class[inner]
        nav = cls != NON_NAVIGABLE
        share_fly = float(np.mean(cls == FLYABLE))
        share_nav = float(np.mean(nav))
        if share_fly >= FLYABLE_SHARE:
            klass[r, c] = FLYABLE
        elif share_nav < NAVIGABLE_SHARE:
            klass[r, c] = NON_NAVIGABLE
        else:
            klass[r, c] = MARGINAL
        pick = nav if nav.any() else np.ones_like(nav)
        clear[r, c] = float(np.median(ff.best_clearance_m[inner][pick])) if nav.any() else 0.0
        window[r, c] = float(np.median(ff.trunk_window_m[inner][pick]))
        max_alt[r, c] = float(np.median(ff.max_flyable_altitude_m[inner][pick]))
        lo = float(np.median(ff.nav_floor_m[inner][pick]))
        hi = float(np.median(ff.nav_ceil_m[inner][pick]))
        if lo > hi:
            lo = hi = 0.5 * (lo + hi)
        floor[r, c], ceil[r, c] = lo, hi

    return FlyabilityField(
        navigable_class=klass,
        cost=_CLASS_COST[klass],
        non_navigable=(klass == NON_NAVIGABLE),
        max_flyable_altitude_m=max_alt,
        best_clearance_m=clear,
        trunk_window_m=window,
        nav_floor_m=floor,
        nav_ceil_m=ceil,
    )
