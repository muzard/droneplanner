"""Geometry primitives and CRS helpers.

CRS discipline (Software-Spec §0): every geospatial input carries an explicit
CRS, and the planner does all distance/clearance math in a single *projected
metric* CRS (meters), never geographic degrees. These helpers enforce that.
"""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field
from pyproj import CRS, Transformer

_WGS84 = "EPSG:4326"


class Coordinate(BaseModel):
    """A 2D point in the mission's projected working CRS (meters)."""

    model_config = ConfigDict(frozen=True)

    x: float = Field(description="Easting in the working CRS (meters).")
    y: float = Field(description="Northing in the working CRS (meters).")

    def distance_to(self, other: Coordinate) -> float:
        """Planar Euclidean distance in meters (valid in a projected CRS)."""
        return math.hypot(self.x - other.x, self.y - other.y)


def validate_projected_crs(crs_str: str) -> str:
    """Return the normalized CRS string, raising if it is not projected.

    Geographic (degree-based) CRSs are rejected: distance and clearance math
    must run in meters.
    """
    try:
        crs = CRS.from_user_input(crs_str)
    except Exception as exc:  # pyproj raises CRSError; keep the message useful
        raise ValueError(f"unparseable CRS {crs_str!r}: {exc}") from exc
    if not crs.is_projected:
        raise ValueError(
            f"CRS {crs_str!r} is not a projected metric CRS; "
            "distance/clearance math requires meters (e.g. a UTM zone)."
        )
    return crs_str


def lonlat_to_crs(lon: float, lat: float, working_crs: str) -> Coordinate:
    """Transform a WGS84 lon/lat pair into the projected working CRS."""
    transformer = Transformer.from_crs(_WGS84, working_crs, always_xy=True)
    x, y = transformer.transform(lon, lat)
    return Coordinate(x=float(x), y=float(y))


def crs_to_lonlat(coord: Coordinate, working_crs: str) -> tuple[float, float]:
    """Transform a projected coordinate back to WGS84 (lon, lat)."""
    transformer = Transformer.from_crs(working_crs, _WGS84, always_xy=True)
    lon, lat = transformer.transform(coord.x, coord.y)
    return float(lon), float(lat)
