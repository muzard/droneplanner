"""Unit tests for the road-crossing stub (Software-Spec §5.4)."""

from __future__ import annotations

import numpy as np

from droneplanner.crossings import ROAD_COST
from droneplanner.crossings.detect import detect_crossings
from droneplanner.crossings.roads import rasterize_roads
from droneplanner.model.grid import Grid
from droneplanner.model.plan import CrossingMethod


def _grid(size: int = 20, cell: float = 10.0) -> Grid:
    return Grid(
        transform=(cell, 0.0, 0.0, 0.0, -cell, size * cell), rows=size, cols=size, crs="EPSG:32633"
    )


def test_rasterize_roads_marks_line_cells() -> None:
    import geopandas as gpd
    from shapely.geometry import LineString

    grid = _grid()
    # A horizontal road across the middle of the 200 m x 200 m extent.
    road = LineString([(0.0, 100.0), (200.0, 100.0)])
    gdf = gpd.GeoDataFrame(geometry=[road], crs="EPSG:32633")
    mask = rasterize_roads(gdf, grid)
    assert mask.any()
    assert mask.shape == (20, 20)


def test_rasterize_empty_roads() -> None:
    import geopandas as gpd

    gdf = gpd.GeoDataFrame(geometry=[], crs="EPSG:32633")
    mask = rasterize_roads(gdf, _grid())
    assert not mask.any()


def test_detect_crossing_over_road() -> None:
    grid = _grid()
    road_mask = np.zeros((20, 20), dtype=bool)
    road_mask[10, :] = True  # a road along row 10
    # A vertical route crossing row 10 once.
    path = [(0, 5), (19, 5)]
    crossings = detect_crossings(path, road_mask, grid)
    assert len(crossings) == 1
    assert crossings[0].method is CrossingMethod.OPEN
    assert not crossings[0].needs_manipulation


def test_detect_no_crossing_when_off_road() -> None:
    grid = _grid()
    road_mask = np.zeros((20, 20), dtype=bool)
    road_mask[10, 15:] = True  # road only on the right
    path = [(0, 5), (9, 5)]  # route stays left, never touches the road
    assert detect_crossings(path, road_mask, grid) == ()


def test_road_cost_is_a_penalty() -> None:
    assert ROAD_COST > 1.0
