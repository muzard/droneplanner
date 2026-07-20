"""Unit tests for the in-house line-of-sight viewshed (Software-Spec §5.1, §6.2)."""

from __future__ import annotations

import numpy as np

from droneplanner.fields.viewshed import cumulative_exposure, single_viewshed
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
