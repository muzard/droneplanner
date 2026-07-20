"""I/O readers and writers (Software-Spec §4).

Heavy geospatial backends (rasterio/GDAL, geopandas, laspy) are imported lazily
inside each reader so the straight-line stub and its tests do not require them.
"""

from __future__ import annotations

from droneplanner.io.mavlink import plan_to_qgc, write_plan
from droneplanner.io.mission import MissionLoadError, load_mission
from droneplanner.io.pointcloud import PointCloud, read_pointcloud
from droneplanner.io.raster import RasterGrid, read_raster, write_raster
from droneplanner.io.report import (
    report_to_json,
    report_to_markdown,
    write_report,
)
from droneplanner.io.trajectory import trajectory_to_geojson, write_trajectory
from droneplanner.io.vector import read_vector, require_crs

__all__ = [
    "MissionLoadError",
    "PointCloud",
    "RasterGrid",
    "load_mission",
    "plan_to_qgc",
    "read_pointcloud",
    "read_raster",
    "read_vector",
    "report_to_json",
    "report_to_markdown",
    "require_crs",
    "trajectory_to_geojson",
    "write_plan",
    "write_raster",
    "write_report",
    "write_trajectory",
]
