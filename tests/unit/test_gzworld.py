"""Tests for the Gazebo terrain-world generator (placement math + SDF)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from droneplanner.io.gzworld import (
    build_world_sdf,
    compute_geometry,
    normalise_dem,
    surface_z_at_home,
)
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


_BASE_WORLD = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <model name="ground_plane">
      <static>true</static>
      <link name="link"><collision name="collision"/></link>
    </model>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.3979</latitude_deg>
      <longitude_deg>8.5461</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


def _build(dem: Path = Path("/dem.tif")) -> str:
    return build_world_sdf(
        _dtm(),
        dem,
        _BASE_WORLD,
        home_lat=46.95611629,
        home_lon=15.00410710,
        home_x=_X0,
        home_y=_Y0,
        home_elev_m=124.68,
        world_name="planner_terrain",
    )


def test_injection_preserves_px4_world_config() -> None:
    """The reason we inject instead of hand-writing a world.

    A from-scratch world loses gz-sim's default plugin set and PX4's
    <magnetic_field>; the vehicle then flies blind and the EKF yaw diverges.
    """
    sdf = _build()
    assert "<magnetic_field>" in sdf  # without this the EKF attitude tumbles
    assert "adiabatic" in sdf
    assert "<real_time_update_rate>250</real_time_update_rate>" in sdf
    assert 'type="ode"' in sdf
    # Declaring ANY <plugin> would disable gz-sim's defaults - so declare none.
    assert "<plugin" not in sdf
    # The world name MUST match the .sdf file stem PX4 selects with
    # PX4_GZ_WORLD, or PX4 hangs on "Waiting for Gazebo world...".
    assert '<world name="planner_terrain">' in sdf
    assert '<world name="default">' not in sdf


def test_injection_replaces_ground_plane_with_terrain() -> None:
    sdf = _build()
    assert 'name="ground_plane"' not in sdf  # would floor the terrain at z=0
    assert '<model name="terrain">' in sdf
    assert "<heightmap>" in sdf


def test_visual_only_terrain_keeps_a_ground_plane_to_stand_on() -> None:
    """The --no-collision diagnostic must still leave collision geometry.

    Dropping both the terrain collision and PX4's ground_plane leaves a world
    with nothing to stand on: the vehicle falls through and PX4 reports
    "ekf2 missing data", never becoming ready.
    """
    sdf = build_world_sdf(
        _dtm(),
        Path("/dem.tif"),
        _BASE_WORLD,
        home_lat=46.9,
        home_lon=15.0,
        home_x=_X0,
        home_y=_Y0,
        home_elev_m=124.68,
        world_name="planner_nocol",
        collision=False,
    )
    root = ET.fromstring(sdf)
    world = root.find("world")
    assert world is not None
    names = {m.get("name") for m in world.findall("model")}
    assert "ground_plane" in names  # something to take off from
    assert "terrain" in names
    terrain = next(m for m in world.findall("model") if m.get("name") == "terrain")
    # The terrain itself must contribute no collision - that is the diagnostic.
    assert terrain.find(".//collision") is None
    assert terrain.find(".//visual") is not None


def test_injection_retargets_world_origin_to_plan_home() -> None:
    sdf = _build()
    assert "<latitude_deg>46.95611629</latitude_deg>" in sdf
    assert "<longitude_deg>15.00410710</longitude_deg>" in sdf
    assert "<elevation>124.68</elevation>" in sdf
    assert "47.3979" not in sdf  # PX4's Zurich default is gone


def test_injection_rejects_a_world_without_a_world_element() -> None:
    try:
        build_world_sdf(
            _dtm(),
            Path("/dem.tif"),
            "<sdf version='1.9'></sdf>",
            home_lat=0.0,
            home_lon=0.0,
            home_x=_X0,
            home_y=_Y0,
            home_elev_m=0.0,
            world_name="x",
        )
    except ValueError:
        pass
    else:  # pragma: no cover
        raise AssertionError("expected ValueError for a base SDF with no <world>")


def test_sdf_carries_home_spherical_coordinates_and_dem_uri(tmp_path: Path) -> None:
    dem = tmp_path / "dtm.tif"
    dem.write_bytes(b"")
    sdf = _build(dem.resolve())
    # World origin must be the plan's home, or terrain and PX4_HOME_* disagree.
    assert "<latitude_deg>46.95611629</latitude_deg>" in sdf
    assert "<longitude_deg>15.00410710</longitude_deg>" in sdf
    assert "<elevation>124.68</elevation>" in sdf
    assert f"file://{dem.resolve()}" in sdf
    assert "<world_frame_orientation>ENU</world_frame_orientation>" in sdf
    # Heightmap extents come from the raster.
    assert "2500.00 2500.00 80.00" in sdf


# --------------------------------------------------------------------------- #
# The tumble bug: vertical datum
#
# Terrain worlds tumbled because the vehicle spawned buried. PX4 spawns at world
# z = 0, so the terrain surface under home MUST be at world z = 0. Getting this
# wrong put it 40 m (real DEM) / 125 m (flat diagnostic) underground, which reads
# as a stable hover and then flings the vehicle the moment it translates.
# --------------------------------------------------------------------------- #


def test_surface_under_home_is_at_world_zero() -> None:
    """The regression guard for the tumble. Non-zero => buried or floating."""
    for min_elev, relief, home_elev in (
        (40.0, 110.0, 124.68),  # the demo AO that tumbled
        (0.0, 50.0, 25.0),
        (1200.0, 300.0, 1200.0),  # home at the DEM minimum
        (1200.0, 300.0, 1500.0),  # home at the DEM maximum
    ):
        geom = compute_geometry(
            _dtm(min_elev=min_elev, relief=relief),
            home_x=_X0,
            home_y=_Y0,
            home_elev_m=home_elev,
        )
        assert surface_z_at_home(geom, home_elev) == pytest.approx(0.0, abs=1e-9)


def test_normalise_dem_zero_bases_elevations() -> None:
    """Gazebo must be handed 0..relief, so raw-vs-rescaled interpretations agree."""
    dtm = _dtm(min_elev=40.0, relief=110.0)
    norm = normalise_dem(dtm)
    assert float(norm.data.min()) == pytest.approx(0.0)
    assert float(norm.data.max()) == pytest.approx(110.0)
    # Shape/placement must be untouched — only the values shift.
    assert norm.transform == dtm.transform
    assert norm.crs == dtm.crs
    assert norm.data.shape == dtm.data.shape


def test_normalise_dem_ignores_nodata_cells() -> None:
    dtm = _dtm(min_elev=40.0, relief=110.0)
    data = dtm.data.copy()
    data[0, 0] = -9999.0
    holed = replace(dtm, data=data, nodata=-9999.0)
    norm = normalise_dem(holed)
    valid = norm.data[norm.data != -9999.0]
    # The nodata sentinel must not become the floor (that would shift terrain by ~10 km).
    assert float(valid.min()) == pytest.approx(0.0)
    assert norm.data[0, 0] == -9999.0


def test_flat_dem_gets_a_usable_vertical_scale() -> None:
    """A perfectly flat DEM has relief 0; <size ... 0> is degenerate in Gazebo."""
    flat = _dtm(min_elev=100.0, relief=0.0)
    geom = compute_geometry(flat, home_x=_X0, home_y=_Y0, home_elev_m=100.0)
    assert geom.relief_m > 0.0
    # ...and the datum still lands on zero.
    assert surface_z_at_home(geom, 100.0) == pytest.approx(0.0, abs=1e-9)
