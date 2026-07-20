"""Unit tests for raster / point-cloud readers (Software-Spec §4).

These exercise the real GDAL/laspy backends on tiny synthetic data.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from droneplanner.io.pointcloud import read_pointcloud
from droneplanner.io.raster import read_raster, write_raster
from tests.fixtures.synthetic import make_dem, make_las_bytes


def test_raster_roundtrip(tmp_path: Path) -> None:
    grid = make_dem("gaussian_hill", size=16, cell_m=10.0)
    path = tmp_path / "dem.tif"
    write_raster(path, grid)
    back = read_raster(path)
    assert back.shape == grid.shape
    assert back.crs.upper().endswith("32633")
    np.testing.assert_allclose(back.data, grid.data, rtol=1e-5, atol=1e-3)
    assert back.transform[0] == pytest.approx(10.0)


def test_raster_peak_at_center() -> None:
    grid = make_dem("gaussian_hill", size=17)
    center = grid.data[8, 8]
    assert center == pytest.approx(grid.data.max())
    assert center > grid.data[0, 0]


def test_pointcloud_read(tmp_path: Path) -> None:
    path = tmp_path / "ao.las"
    path.write_bytes(make_las_bytes(n=150))
    pc = read_pointcloud(path)
    assert pc.count == 150
    assert pc.xyz.shape == (150, 3)
    assert pc.classification is not None
    assert int(pc.classification[0]) == 2  # ground
