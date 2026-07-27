"""Unit tests for the corridor's lateral freedom (TEST-PLAN A1).

The requirement these encode (user, 2026-07-28):

> the corridors should still follow some sources of concealment (like a forest or
> a small depression inside it), so at least it couldn't expand outside of a
> forest patch

So a constant width is not acceptable: the corridor has to be bounded by the
*local extent of the cover*, and these tests are the proof that it is.
"""

from __future__ import annotations

import numpy as np

from droneplanner.fields.composite import CostSurface
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.planner.corridor_width import assess_corridor

CELL_M = 10.0
SIZE = 41
CAP = 200.0


def _grid(size: int = SIZE) -> Grid:
    return Grid(
        transform=(CELL_M, 0.0, 0.0, 0.0, -CELL_M, size * CELL_M),
        rows=size,
        cols=size,
        crs="EPSG:32633",
    )


def _surface(
    exposure: np.ndarray,
    *,
    flight_band: np.ndarray | None = None,
    obstacle: np.ndarray | None = None,
) -> CostSurface:
    grid = _grid(exposure.shape[0])
    zeros = np.zeros(exposure.shape, dtype=np.float32)
    components = {"exposure": exposure.astype(np.float32)}
    if flight_band is not None:
        components["flight_band"] = flight_band.astype(np.float32)
    return CostSurface(
        grid=grid,
        cost=np.ones(exposure.shape, dtype=np.float32),
        dtm=zeros,
        dsm=zeros,
        components=components,
        obstacle=obstacle,
    )


def _column_points(grid: Grid, col: int, rows: range) -> list[Coordinate]:
    return [grid.pixel_to_world(r, col) for r in rows]


def test_wide_open_cover_is_bounded_only_by_the_cap() -> None:
    """Nothing to run into: the corridor is whatever the operator allows."""
    exposure = np.zeros((SIZE, SIZE), dtype=np.float32)
    surface = _surface(exposure)
    report = assess_corridor(
        surface, _column_points(surface.grid, 20, range(10, 30)), max_half_width_m=CAP
    )
    assert report.min_half_width_m == CAP


def test_corridor_does_not_expand_into_more_exposed_ground() -> None:
    """Freedom ends where the concealment does — measured to the cell."""
    exposure = np.ones((SIZE, SIZE), dtype=np.float32)
    exposure[:, 18:23] = 0.0  # a 5-cell hidden strip, centre column 20
    surface = _surface(exposure)
    report = assess_corridor(
        surface, _column_points(surface.grid, 20, range(10, 30)), max_half_width_m=CAP
    )
    # Centre column to the first exposed column is 2 cells + 1 to reach it.
    assert report.min_half_width_m == max(report.half_width_m)
    assert 2 * CELL_M <= report.min_half_width_m <= 3 * CELL_M


def test_corridor_cannot_leave_the_forest_patch_even_where_it_is_unobserved() -> None:
    """The requirement, stated exactly.

    Here the ground *outside* the stand is just as unobserved as the ground
    inside it, so an exposure rule alone would let the corridor spill into the
    clearing. It must not: leaving the stand means climbing out of the cover the
    vertex was planned to fly inside, which is a different plan, not a deviation.
    """
    exposure = np.zeros((SIZE, SIZE), dtype=np.float32)  # nothing is observed
    band = np.zeros((SIZE, SIZE), dtype=np.float32)  # 0 = open
    band[:, 17:24] = 2.0  # a 7-cell stand, centre column 20
    surface = _surface(exposure, flight_band=band)
    report = assess_corridor(
        surface, _column_points(surface.grid, 20, range(10, 30)), max_half_width_m=CAP
    )
    assert 3 * CELL_M <= report.min_half_width_m <= 4 * CELL_M
    assert report.min_half_width_m < CAP


def test_a_route_outside_any_stand_is_not_constrained_by_canopy() -> None:
    """The patch rule binds only where the vertex is actually inside a patch."""
    exposure = np.zeros((SIZE, SIZE), dtype=np.float32)
    band = np.zeros((SIZE, SIZE), dtype=np.float32)
    band[:, 0:3] = 2.0  # a stand far from the route
    surface = _surface(exposure, flight_band=band)
    report = assess_corridor(
        surface, _column_points(surface.grid, 20, range(10, 30)), max_half_width_m=CAP
    )
    assert report.min_half_width_m == CAP


def test_corridor_stops_at_an_obstacle() -> None:
    exposure = np.zeros((SIZE, SIZE), dtype=np.float32)
    obstacle = np.zeros((SIZE, SIZE), dtype=bool)
    obstacle[:, 24:] = True
    surface = _surface(exposure, obstacle=obstacle)
    report = assess_corridor(
        surface, _column_points(surface.grid, 20, range(10, 30)), max_half_width_m=CAP
    )
    assert 3 * CELL_M <= report.min_half_width_m <= 4 * CELL_M


def test_an_already_exposed_vertex_keeps_the_freedom_it_has() -> None:
    """Crossing a clearing: there is no cover left to protect, so nothing to lose.

    Reporting zero freedom here would be wrong — the freedom is real, it is the
    concealment that is absent, and the exposure score says so separately.
    """
    exposure = np.ones((SIZE, SIZE), dtype=np.float32)
    surface = _surface(exposure)
    report = assess_corridor(
        surface, _column_points(surface.grid, 20, range(10, 30)), max_half_width_m=CAP
    )
    assert report.min_half_width_m == CAP
    assert report.mean_exposure == 1.0


def test_exposure_is_scored_over_the_swept_area_not_the_line() -> None:
    """The point of the whole module: score what the aircraft may fly, not the plan."""
    exposure = np.zeros((SIZE, SIZE), dtype=np.float32)
    exposure[:, 22:] = 1.0  # exposed ground just off the route's right shoulder
    surface = _surface(exposure)
    points = _column_points(surface.grid, 20, range(10, 30))
    centreline = float(np.mean([exposure[surface.grid.world_to_pixel(p)] for p in points]))
    report = assess_corridor(surface, points, max_half_width_m=CAP)

    assert centreline == 0.0  # the line is perfectly concealed...
    assert report.mean_exposure == 0.0  # ...and so is every cell it may use
    assert report.cells > len(points)  # but the corridor is an area, not a line


def test_widths_are_reported_per_vertex() -> None:
    exposure = np.zeros((SIZE, SIZE), dtype=np.float32)
    exposure[:, 30:] = 1.0
    surface = _surface(exposure)
    points = [surface.grid.pixel_to_world(20, c) for c in (5, 20, 28)]
    report = assess_corridor(surface, points, max_half_width_m=CAP)
    assert len(report.half_width_m) == 3
    # Closer to the exposed ground on the right => less freedom.
    assert report.half_width_m[0] > report.half_width_m[1] > report.half_width_m[2]
