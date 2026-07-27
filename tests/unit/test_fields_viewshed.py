"""Unit tests for the in-house line-of-sight viewshed (Software-Spec §5.1, §6.2)."""

from __future__ import annotations

import numpy as np

from droneplanner.fields.viewshed import cumulative_exposure, point_exposure, single_viewshed
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid


def _wall_dsm(size: int = 21, wall_col: int = 10, wall_h: float = 100.0) -> np.ndarray:
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, wall_col] = wall_h
    return dsm


def test_wall_blocks_line_of_sight() -> None:
    dsm = _wall_dsm()
    vis = single_viewshed(dsm, observer_rc=(10, 0), cell_size_m=10.0, observer_height_m=2.0)
    assert vis[10, 5]  # in front of the wall — visible
    assert vis[10, 10]  # the wall itself — visible
    assert not vis[10, 15]  # behind the wall — occluded


def test_observer_sees_open_far_slope_on_ridge() -> None:
    # A single ridge: observer on the crest sees down both slopes.
    size = 31
    dsm = np.zeros((size, size), dtype=np.float32)
    for c in range(size):
        dsm[:, c] = max(0.0, 50.0 - abs(c - 15) * 5.0)
    vis = single_viewshed(dsm, observer_rc=(15, 15), cell_size_m=10.0, observer_height_m=2.0)
    assert vis[15, 5]
    assert vis[15, 25]


def _flat_grid(size: int) -> Grid:
    transform = (10.0, 0.0, 0.0, 0.0, -10.0, size * 10.0)
    return Grid(transform=transform, rows=size, cols=size, crs="EPSG:32633")


def test_cumulative_exposure_no_observers_is_zero() -> None:
    size = 16
    dsm = np.zeros((size, size), dtype=np.float32)
    grid = _flat_grid(size)
    exp = cumulative_exposure(dsm, grid, observers=[])
    assert np.all(exp == 0.0)


def test_cumulative_exposure_flat_all_visible() -> None:
    size = 16
    dsm = np.zeros((size, size), dtype=np.float32)
    grid = _flat_grid(size)
    center = grid.pixel_to_world(8, 8)
    exp = cumulative_exposure(dsm, grid, observers=[Coordinate(x=center.x, y=center.y)])
    # On flat ground a single observer sees (almost) everything -> exposure ~1.
    assert exp.max() == 1.0
    assert exp.mean() > 0.9


# --- Target elevation: what is being looked at (SHORTFALLS item 1) ---------------


def test_flying_over_a_wall_is_visible_where_the_ground_behind_it_is_not() -> None:
    """The defect in one test: a drone is not the ground it flies over."""
    dsm = _wall_dsm(size=21, wall_col=10, wall_h=100.0)
    ground = single_viewshed(dsm, observer_rc=(10, 0), cell_size_m=10.0)
    # Fly at 300 m — well above the wall's shadow line at this range.
    aloft = single_viewshed(dsm, observer_rc=(10, 0), cell_size_m=10.0, target_elev_m=dsm + 300.0)
    assert not ground[10, 15]  # the ground behind the wall is concealed
    assert aloft[10, 15]  # a drone over that same ground is not


def test_raising_the_target_can_only_add_visibility() -> None:
    """Over a bare DSM, altitude is a pure cost in concealment terms."""
    size = 25
    yy, xx = np.mgrid[0:size, 0:size].astype(np.float64)
    dsm = (40.0 * np.sin(xx / 3.0) + 30.0 * np.cos(yy / 4.0)).astype(np.float32)
    low = single_viewshed(dsm, observer_rc=(3, 3), cell_size_m=10.0)
    high = single_viewshed(dsm, observer_rc=(3, 3), cell_size_m=10.0, target_elev_m=dsm + 30.0)
    assert np.all(high | ~low)  # visible-at-ground implies visible-at-altitude
    assert high.sum() > low.sum()


def test_flying_under_canopy_is_concealed_where_the_canopy_top_is_not() -> None:
    """The other direction, which is the whole premise of flying in forest.

    An observer on a hilltop overlooks a stand of 25 m trees: it sees the canopy
    roof, but not the drone in the trunk space beneath it. The demo AO cannot
    show this (its DSM has no canopy — SHORTFALLS), so it is pinned here.
    """
    size = 31
    dtm = np.zeros((size, size), dtype=np.float32)
    dsm = dtm.copy()
    dsm[:, 10:] = 25.0  # closed canopy from column 10 outward
    dtm[15, 0] = dsm[15, 0] = 100.0  # the observer's hilltop

    surface = single_viewshed(dsm, observer_rc=(15, 0), cell_size_m=10.0)
    trunk = single_viewshed(dsm, observer_rc=(15, 0), cell_size_m=10.0, target_elev_m=dtm + 8.0)
    assert surface[15, 25]  # the canopy roof at that cell: in plain view
    assert not trunk[15, 25]  # a drone at 8 m AGL under it: occluded by the canopy


def test_target_elev_shape_is_validated() -> None:
    dsm = np.zeros((8, 8), dtype=np.float32)
    try:
        single_viewshed(dsm, (0, 0), 10.0, target_elev_m=np.zeros((4, 4), dtype=np.float32))
    except ValueError as exc:
        assert "same shape" in str(exc)
    else:  # pragma: no cover - the call must raise
        raise AssertionError("expected a shape mismatch to raise")


# --- Point exposure: the same question asked of a 3D trajectory -----------------


def test_point_exposure_agrees_with_the_raster_at_the_same_elevation() -> None:
    """The trajectory re-score and the raster field must not disagree."""
    size = 21
    yy, xx = np.mgrid[0:size, 0:size].astype(np.float64)
    dsm = (30.0 * np.sin(xx / 2.5) + 20.0 * np.cos(yy / 3.0)).astype(np.float32)
    grid = _flat_grid(size)
    obs = grid.pixel_to_world(2, 2)
    observers = [Coordinate(x=obs.x, y=obs.y)]

    target = dsm + 30.0
    raster = cumulative_exposure(dsm, grid, observers, target_elev_m=target)

    cells = [(r, c) for r in range(0, size, 4) for c in range(0, size, 4)]
    pts = []
    for r, c in cells:
        w = grid.pixel_to_world(r, c)
        pts.append((w.x, w.y, float(target[r, c])))
    at_points = point_exposure(dsm, grid, observers, pts)

    for (r, c), got in zip(cells, at_points, strict=True):
        assert got == raster[r, c], f"cell {(r, c)} disagrees"


def test_point_exposure_no_observers_is_zero() -> None:
    dsm = np.zeros((8, 8), dtype=np.float32)
    grid = _flat_grid(8)
    assert np.all(point_exposure(dsm, grid, [], [(0.0, 0.0, 10.0)]) == 0.0)
    assert point_exposure(dsm, grid, [], []).size == 0
