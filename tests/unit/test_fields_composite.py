"""Unit tests for survivability, snag, and the composite cost surface (§5.1, §5.3)."""

from __future__ import annotations

import numpy as np

from droneplanner.fields.composite import COST_FLOOR, build_cost_surface
from droneplanner.fields.snag import build_snag
from droneplanner.fields.survivability import build_survivability
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from tests.fixtures.synthetic import make_dem, make_mission


def _grid_for(size: int) -> Grid:
    transform = (10.0, 0.0, 500000.0, 0.0, -10.0, 5200000.0 + size * 10.0)
    return Grid(transform=transform, rows=size, cols=size, crs="EPSG:32633")


def test_snag_zero_without_landcover() -> None:
    s = build_snag((10, 10))
    assert np.all(s == 0.0)


def test_snag_from_landcover_lookup() -> None:
    lc = np.zeros((8, 8), dtype=np.int32)
    lc[0:4, :] = 5  # a "dense" class
    s = build_snag((8, 8), land_cover=lc, snag_lookup={5: 1.0})
    assert s[0, 0] == 1.0
    assert s[7, 7] == 0.0


def test_survivability_normalized() -> None:
    dem = make_dem("gaussian_hill", size=24)
    grid = _grid_for(24)
    obs = grid.pixel_to_world(4, 4)
    field = build_survivability(dem.data, dem.data, grid, [Coordinate(x=obs.x, y=obs.y)])
    assert field.min() >= 0.0 and field.max() <= 1.0
    assert field.shape == (24, 24)


def test_cost_surface_positive_and_has_components() -> None:
    dem = make_dem("ridge", size=24)
    grid = _grid_for(24)
    mission = make_mission(crs="EPSG:32633")
    surface = build_cost_surface(dem.data, dem.data, grid, mission)
    assert surface.cost.min() >= COST_FLOOR
    for key in ("exposure", "survivability", "snag", "ridge", "drainage", "cost_normalized"):
        assert key in surface.components
    assert surface.shape == (24, 24)
