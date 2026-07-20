"""In-house line-of-sight viewshed (Software-Spec §5.1 ``viewshed.py``).

Cumulative visual exposure from a set of observers computed over the **DSM**, so
terrain *and* canopy occlude line-of-sight (Preflight-Design §3.1). This is the
shippable NumPy path — no GRASS/GPL dependency. A ridgeline penalty from the DTM
is added separately in ``fields.terrain`` and combined in the composite.

Algorithm: for each observer, cast rays to every grid-boundary cell and march
each ray outward tracking the running maximum elevation angle; a cell is visible
when its own elevation angle clears that running maximum. Exposure is the
fraction of observers that see each cell, in [0, 1].
"""

from __future__ import annotations

import math

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


def single_viewshed(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    cell_size_m: float,
    observer_height_m: float = 2.0,
    target_height_m: float = 0.0,
) -> np.ndarray:
    """Boolean visibility grid for one observer (True = visible)."""
    rows, cols = dsm.shape
    r0, c0 = observer_rc
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
            terrain_angle = (float(dsm[r, c]) - eye) / dist
            view_angle = (float(dsm[r, c]) + target_height_m - eye) / dist
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
) -> np.ndarray:
    """Fraction of observers that can see each cell, in [0, 1] (float32).

    With no observers, returns an all-zero field (nothing is exposed).
    """
    rows, cols = dsm.shape
    if not observers:
        return np.zeros((rows, cols), dtype=np.float32)

    accum = np.zeros((rows, cols), dtype=np.float32)
    for obs in observers:
        rc = grid.world_to_pixel(obs)
        accum += single_viewshed(
            dsm, rc, grid.cell_size_m, observer_height_m, target_height_m
        ).astype(np.float32)
    return (accum / float(len(observers))).astype(np.float32)
