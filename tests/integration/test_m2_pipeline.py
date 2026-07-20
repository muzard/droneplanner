"""Integration: pipeline ingests lidar + roads for M2 flyability and crossings."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from droneplanner.io.raster import write_raster
from droneplanner.model.plan import CrossingMethod, FlyabilityClass
from droneplanner.pipeline import plan_mission
from tests.fixtures.synthetic import make_dem, make_mission


def _write_ground_canopy_las(path: Path, extent_m: float = 480.0) -> None:
    """Write a LAS with ground (z=0) + canopy (z=12) returns over the extent."""
    import laspy

    xs = np.arange(5.0, extent_m, 5.0)
    gx, gy = np.meshgrid(xs, xs)
    gx = gx.ravel()
    gy = gy.ravel()
    ground = np.column_stack([gx, gy, np.zeros_like(gx)])
    canopy = np.column_stack([gx, gy, np.full_like(gx, 12.0)])
    xyz = np.vstack([ground, canopy])

    header = laspy.LasHeader(point_format=3, version="1.4")
    las = laspy.LasData(header)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    las.write(path)


def _write_road(path: Path) -> None:
    import geopandas as gpd
    from shapely.geometry import LineString

    road = LineString([(0.0, 240.0), (480.0, 240.0)])  # horizontal, mid-extent
    gpd.GeoDataFrame(geometry=[road], crs="EPSG:32633").to_file(path, driver="GeoJSON")


def _mission_with_layers(tmp_path: Path):  # type: ignore[no-untyped-def]
    dtm = make_dem("flat", size=48, cell_m=10.0)
    dsm = make_dem("flat", size=48, cell_m=10.0)
    write_raster(tmp_path / "dtm.tif", dtm)
    write_raster(tmp_path / "dsm.tif", dsm)
    _write_ground_canopy_las(tmp_path / "ao.las")
    _write_road(tmp_path / "roads.geojson")
    return make_mission(
        crs="EPSG:32633",
        start={"x": 60.0, "y": 60.0},
        end={"x": 420.0, "y": 420.0},
        layers={
            "dtm": str(tmp_path / "dtm.tif"),
            "dsm": str(tmp_path / "dsm.tif"),
            "point_cloud": str(tmp_path / "ao.las"),
            "roads": str(tmp_path / "roads.geojson"),
        },
    )


def test_pipeline_m2_flyability_and_crossings(tmp_path: Path) -> None:
    mission = _mission_with_layers(tmp_path)
    results = plan_mission(mission, tmp_path / "out")
    plan = results[0].plan
    report = results[0].report

    # Forest flyability ran -> the M1 "not analyzed" flag is gone.
    codes = {f.code for f in report.findings}
    assert "flyability_not_analyzed" not in codes
    assert "analysis_not_performed" not in codes

    # Open, flyable forest -> every segment is FLYABLE and flies the trunk band.
    assert all(s.flyability is FlyabilityClass.FLYABLE for s in plan.segments)
    assert any(v.altitude_agl_m > 2.5 for v in plan.vertices)

    # The route crosses the mid-extent road -> at least one OPEN crossing.
    assert len(plan.crossings) >= 1
    assert all(c.method is CrossingMethod.OPEN for c in plan.crossings)
    assert "open_crossings" in codes
