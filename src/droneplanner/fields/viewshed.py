"""In-house line-of-sight viewshed (Software-Spec §5.1 ``viewshed.py``).

Cumulative visual exposure from a set of observers computed over the **DSM**, so
terrain *and* canopy occlude line-of-sight (Preflight-Design §3.1). This is the
shippable NumPy path — no GRASS/GPL dependency. A ridgeline penalty from the DTM
is added separately in ``fields.terrain`` and combined in the composite.

Algorithm: for each observer, cast rays to every grid-boundary cell and march
each ray outward tracking the running maximum elevation angle; a cell is visible
when its own elevation angle clears that running maximum. Exposure is the
fraction of observers that see each cell, in [0, 1].

VALIDATED 2026-07-28 against an independent implementation (TEST-PLAN A4,
``tests/unit/test_fields_viewshed_reference.py``), which tests each target cell
against the straight 3D segment aimed at *that* cell. Results worth knowing:

* the running-max-**angle** test is algebraically identical to a segment-height
  test on the same ray and cells, so the *only* approximation here is which ray
  and which cells get used;
* rays run to every boundary cell, and those Bresenham lines cover a rectangular
  grid **exhaustively — zero cells go untested**. (An earlier version of this
  docstring claimed radial rays "can miss interior cells". Measured: they do not.
  That was a guess, and it was wrong.)
* what remains is a **bearing error**: a cell is tested against a ray aimed at a
  boundary cell, not at itself, so the occluders considered are up to half a cell
  off-line. Cost measured on the demo AO: **1.0 % of cells disagree**, and mean
  exposure differs by **+0.004** — immaterial to any routing decision.
* the disagreements are **overwhelmingly conservative** (0.94 % of cells
  over-report exposure vs 0.07 % under-report), which is the safe direction for
  a concealment planner: it does not promise cover the terrain fails to provide.
  They concentrate where a sightline grazes a slope — exactly where sub-cell
  terrain detail, which neither model has, would decide the answer.

WHAT IS BEING SEEN — the target elevation (SHORTFALLS item 1)
The occluding surface (the DSM) and the *object being looked at* are two
different things, and conflating them measured concealment for the wrong object.
Until 2026-07-27 every caller used ``target_height_m=0``, i.e. asked "can an
observer see the DSM surface at this cell". The drone does not fly on the
surface: it cruises at ``cruise_agl_m`` above the terrain, or inside the trunk
band under the canopy. Those are radically different objects to hide —

* above the canopy the drone is a silhouette against sky, visible from far
  outside anything that can see the ground beneath it;
* inside the trunk band it is *more* concealed than the canopy top the old field
  scored, because the canopy between it and the observer occludes.

So the target elevation is now a first-class input: ``target_elev_m`` supplies an
absolute (AMSL) elevation per cell — for the drone, ``dtm + planned flight AGL``
— while occlusion still marches over the DSM. ``target_height_m`` remains for
objects that genuinely sit *on* the surface, which is what the laid **cable** is
(``fields.survivability``); the two are deliberately not the same field.

:func:`point_exposure` answers the same question for arbitrary 3D points, which
is how a *flown* trajectory is re-scored at the altitudes it will actually be at
(the raster is evaluated at the planned band altitude, which the climb-limited
smoother and PX4's front-loaded descent both depart from).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.raster_lines import bresenham


def _boundary_cells(rows: int, cols: int) -> list[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for c in range(cols):
        cells.add((0, c))
        cells.add((rows - 1, c))
    for r in range(rows):
        cells.add((r, 0))
        cells.add((r, cols - 1))
    return sorted(cells)


def _target_surface(
    dsm: np.ndarray, target_height_m: float, target_elev_m: np.ndarray | None
) -> np.ndarray:
    """Absolute elevation of the object being looked at, per cell."""
    if target_elev_m is None:
        return dsm + target_height_m
    if target_elev_m.shape != dsm.shape:
        raise ValueError("target_elev_m must have the same shape as the DSM")
    return target_elev_m


def single_viewshed(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    cell_size_m: float,
    observer_height_m: float = 2.0,
    target_height_m: float = 0.0,
    target_elev_m: np.ndarray | None = None,
) -> np.ndarray:
    """Boolean visibility grid for one observer (True = visible).

    ``target_elev_m`` (absolute elevation per cell) overrides ``target_height_m``
    (a uniform offset above the DSM) and is how a target *off* the surface — a
    drone at flight altitude — is scored. Occlusion always marches over the DSM.
    """
    rows, cols = dsm.shape
    r0, c0 = observer_rc
    target = _target_surface(dsm, target_height_m, target_elev_m)
    visible = np.zeros((rows, cols), dtype=bool)
    visible[r0, c0] = True
    eye = float(dsm[r0, c0]) + observer_height_m

    for rt, ct in _boundary_cells(rows, cols):
        max_angle = -math.inf
        for r, c in bresenham(r0, c0, rt, ct):
            if r == r0 and c == c0:
                continue
            dist = math.hypot(r - r0, c - c0) * cell_size_m
            if dist <= 0.0:
                continue
            # The running maximum covers *intervening* cells only: a cell's own
            # surface never occludes the object standing on (or flying over) it.
            terrain_angle = (float(dsm[r, c]) - eye) / dist
            view_angle = (float(target[r, c]) - eye) / dist
            if view_angle >= max_angle:
                visible[r, c] = True
            if terrain_angle > max_angle:
                max_angle = terrain_angle
    return visible


def cumulative_exposure(
    dsm: np.ndarray,
    grid: Grid,
    observers: list[Coordinate],
    observer_height_m: float = 2.0,
    target_height_m: float = 0.0,
    target_elev_m: np.ndarray | None = None,
) -> np.ndarray:
    """Fraction of observers that can see each cell, in [0, 1] (float32).

    With no observers, returns an all-zero field (nothing is exposed). See
    :func:`single_viewshed` for ``target_elev_m`` — pass it to score the drone at
    its planned flight altitude rather than the ground under it.
    """
    rows, cols = dsm.shape
    if not observers:
        return np.zeros((rows, cols), dtype=np.float32)

    accum = np.zeros((rows, cols), dtype=np.float32)
    for obs in observers:
        rc = grid.world_to_pixel(obs)
        accum += single_viewshed(
            dsm, rc, grid.cell_size_m, observer_height_m, target_height_m, target_elev_m
        ).astype(np.float32)
    return (accum / float(len(observers))).astype(np.float32)


def _point_visible(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    eye_elev_m: float,
    target_rc: tuple[int, int],
    target_elev_m: float,
    cell_size_m: float,
) -> bool:
    """Line of sight from one observer to one 3D point, occluded by the DSM."""
    r0, c0 = observer_rc
    rt, ct = target_rc
    max_angle = -math.inf
    for r, c in bresenham(r0, c0, rt, ct):
        if r == r0 and c == c0:
            continue
        dist = math.hypot(r - r0, c - c0) * cell_size_m
        if dist <= 0.0:
            continue
        if (r, c) == (rt, ct):
            return (target_elev_m - eye_elev_m) / dist >= max_angle
        angle = (float(dsm[r, c]) - eye_elev_m) / dist
        if angle > max_angle:
            max_angle = angle
    return True  # target shares the observer's cell


def point_exposure(
    dsm: np.ndarray,
    grid: Grid,
    observers: list[Coordinate],
    points: Sequence[tuple[float, float, float]],
    observer_height_m: float = 2.0,
) -> np.ndarray:
    """Fraction of observers that can see each ``(x, y, z_amsl)`` point.

    The raster :func:`cumulative_exposure` scores every cell at one assumed
    altitude; this scores specific 3D points, so a trajectory can be re-scored at
    the altitudes it is actually flown at. Cost is
    ``O(observers * points * ray_len)`` — cheap next to a full viewshed.
    """
    n = len(points)
    if n == 0 or not observers:
        return np.zeros(n, dtype=np.float32)

    accum = np.zeros(n, dtype=np.float32)
    cell = grid.cell_size_m
    for obs in observers:
        obs_rc = grid.world_to_pixel(obs)
        eye = float(dsm[obs_rc]) + observer_height_m
        for i, (x, y, z) in enumerate(points):
            rc = grid.world_to_pixel(Coordinate(x=x, y=y))
            if _point_visible(dsm, obs_rc, eye, rc, z, cell):
                accum[i] += 1.0
    return (accum / float(len(observers))).astype(np.float32)
