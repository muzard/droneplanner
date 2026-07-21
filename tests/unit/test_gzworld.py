"""Tests for the Gazebo terrain-world generator (placement math + SDF)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from droneplanner.io.gzworld import build_world_sdf, compute_geometry
from droneplanner.io.raster import RasterGrid

_CELL = 25.0
_X0, _Y0 = 500_000.0, 5_200_000.0
_ROWS = _COLS = 100


def _dtm(min_elev: float = 50.0, relief: float = 80.0) -> RasterGrid:
    # North-up transform: (a, b, c, d, e, f) with negative e; (c, f) = top-left.
    data = np.linspace(min_elev, min_elev + relief, _ROWS * _COLS, dtype=np.float32)
    data = data.reshape(_ROWS, _COLS)
    transform = (_CELL, 0.0, _X0, 0.0, -_CELL, _Y0 + _ROWS * _CELL)
    return RasterGrid(data=data, transform=transform, crs="EPSG:32633", nodata=None)


def test_extents_come_from_the_raster() -> None:
    geom = compute_geometry(_dtm(), home_x=_X0, home_y=_Y0, home_elev_m=50.0)
    assert geom.size_x_m == _CELL * _COLS  # 2500 m
    assert geom.size_y_m == _CELL * _ROWS
    assert geom.relief_m == 80.0
    assert geom.min_elev_m == 50.0


def test_home_at_dtm_centre_needs_no_horizontal_offset() -> None:
    centre_x = _X0 + _CELL * _COLS / 2.0
    centre_y = _Y0 + _CELL * _ROWS / 2.0
    geom = compute_geometry(_dtm(), home_x=centre_x, home_y=centre_y, home_elev_m=50.0)
    assert abs(geom.offset_x_m) < 1e-6
    assert abs(geom.offset_y_m) < 1e-6


def test_offset_places_home_over_the_world_origin() -> None:
    # Home 500 m east / 300 m north of the DTM centre => the terrain model must
    # shift the opposite way so home ends up at (0, 0).
    centre_x = _X0 + _CELL * _COLS / 2.0
    centre_y = _Y0 + _CELL * _ROWS / 2.0
    geom = compute_geometry(
        _dtm(), home_x=centre_x + 500.0, home_y=centre_y + 300.0, home_elev_m=50.0
    )
    assert geom.offset_x_m == -500.0
    assert geom.offset_y_m == -300.0


def test_vertical_offset_puts_ground_at_home_on_z_zero() -> None:
    # Home sits 30 m above the DTM minimum -> shift the terrain down by 30 m.
    geom = compute_geometry(_dtm(min_elev=50.0), home_x=_X0, home_y=_Y0, home_elev_m=80.0)
    assert geom.offset_z_m == -30.0


def test_sdf_carries_home_spherical_coordinates_and_dem_uri(tmp_path: Path) -> None:
    dem = tmp_path / "dtm.tif"
    dem.write_bytes(b"")
    sdf = build_world_sdf(
        _dtm(),
        dem.resolve(),
        home_lat=46.95611629,
        home_lon=15.00410710,
        home_x=_X0,
        home_y=_Y0,
        home_elev_m=124.68,
        world_name="planner_terrain",
    )
    assert '<world name="planner_terrain">' in sdf
    # World origin must be the plan's home, or terrain and PX4_HOME_* disagree.
    assert "<latitude_deg>46.95611629</latitude_deg>" in sdf
    assert "<longitude_deg>15.00410710</longitude_deg>" in sdf
    assert "<elevation>124.68</elevation>" in sdf
    assert f"file://{dem.resolve()}" in sdf
    assert "<world_frame_orientation>ENU</world_frame_orientation>" in sdf
    # Heightmap extents come from the raster.
    assert "2500.00 2500.00 80.00" in sdf


def test_sdf_loads_the_sensor_system_plugins() -> None:
    """Without these, the vehicle spawns blind and PX4 never arms.

    In gz-sim a sensor on a model only produces data if the *world* loads the
    matching system plugin; omitting them yields "No valid data from Accel 0",
    "barometer 0 missing", "Found 0 compass".
    """
    sdf = build_world_sdf(
        _dtm(),
        Path("/dem.tif"),
        home_lat=46.9,
        home_lon=15.0,
        home_x=_X0,
        home_y=_Y0,
        home_elev_m=100.0,
    )
    for system in (
        "gz::sim::systems::Imu",
        "gz::sim::systems::AirPressure",
        "gz::sim::systems::Magnetometer",
        "gz::sim::systems::NavSat",
        "gz::sim::systems::Physics",
        "gz::sim::systems::Sensors",
    ):
        assert system in sdf, f"world is missing the {system} plugin"
