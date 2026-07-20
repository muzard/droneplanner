"""Detect where a planned route crosses roads and emit crossing records (§5.4).

Walks the densified route cells, finds maximal runs over the on-road mask, and
emits one ``Crossing`` per run (method = OPEN — fly over). The planner already
minimizes on-road distance via a cost penalty, so each run should be a short,
near-perpendicular hop across open terrain.
"""

from __future__ import annotations

from itertools import pairwise

import numpy as np

from droneplanner.model.grid import Grid
from droneplanner.model.plan import Crossing, CrossingMethod
from droneplanner.raster_lines import bresenham


def _densify(cell_path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Expand any-angle vertices into the full sequence of traversed cells."""
    if not cell_path:
        return []
    cells: list[tuple[int, int]] = [cell_path[0]]
    for a, b in pairwise(cell_path):
        cells.extend(bresenham(a[0], a[1], b[0], b[1])[1:])
    return cells


def detect_crossings(
    cell_path: list[tuple[int, int]], road_mask: np.ndarray, grid: Grid
) -> tuple[Crossing, ...]:
    """Emit an OPEN crossing for each contiguous on-road run along the route."""
    crossings: list[Crossing] = []
    run: list[tuple[int, int]] = []

    def flush() -> None:
        if not run:
            return
        mid = run[len(run) // 2]
        crossings.append(
            Crossing(
                location=grid.pixel_to_world(mid[0], mid[1]),
                method=CrossingMethod.OPEN,
                feasible=True,
                needs_manipulation=False,
                cost=len(run) * grid.cell_size_m,
                label="open road crossing (fly over)",
            )
        )

    for rc in _densify(cell_path):
        if road_mask[rc]:
            run.append(rc)
        else:
            flush()
            run = []
    flush()
    return tuple(crossings)
