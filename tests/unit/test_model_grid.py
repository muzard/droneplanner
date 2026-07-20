"""Unit tests for the reference grid geometry (Software-Spec §4)."""

from __future__ import annotations

import pytest

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid

# North-up grid: 100 cols x 80 rows, 10 m cells, origin (1000, 5000) top-left.
_T = (10.0, 0.0, 1000.0, 0.0, -10.0, 5000.0)


def _grid() -> Grid:
    return Grid(transform=_T, rows=80, cols=100, crs="EPSG:32633")


def test_cell_size() -> None:
    assert _grid().cell_size_m == pytest.approx(10.0)


def test_pixel_to_world_center() -> None:
    g = _grid()
    # Cell (0,0) center is half a cell in from the top-left origin.
    c = g.pixel_to_world(0, 0)
    assert c.x == pytest.approx(1005.0)
    assert c.y == pytest.approx(4995.0)


def test_world_to_pixel_roundtrip() -> None:
    g = _grid()
    for row, col in [(0, 0), (10, 20), (79, 99), (40, 50)]:
        center = g.pixel_to_world(row, col)
        assert g.world_to_pixel(center) == (row, col)


def test_world_to_pixel_clamps_out_of_bounds() -> None:
    g = _grid()
    far = Coordinate(x=999999.0, y=-999999.0)
    row, col = g.world_to_pixel(far)
    assert 0 <= row < g.rows
    assert 0 <= col < g.cols


def test_contains() -> None:
    g = _grid()
    assert g.contains(g.pixel_to_world(40, 50))
    assert not g.contains(Coordinate(x=0.0, y=0.0))


def test_degenerate_transform_raises() -> None:
    g = Grid(transform=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), rows=5, cols=5, crs="EPSG:32633")
    with pytest.raises(ValueError, match="degenerate"):
        g.world_to_pixel(Coordinate(x=1.0, y=1.0))
