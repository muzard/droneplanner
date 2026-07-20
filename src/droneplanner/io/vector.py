"""Vector (GeoJSON / GeoPackage) reading for map layers (Software-Spec §4).

Lazily wraps ``geopandas``. Used from M1 for roads/rail/buildings/culverts;
provided here so the I/O boundary is complete in the scaffold.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import geopandas as gpd


def read_vector(path: str | Path) -> gpd.GeoDataFrame:
    """Read a vector file (GeoJSON, GeoPackage, ...) into a GeoDataFrame."""
    import geopandas as gpd

    return gpd.read_file(Path(path))


def require_crs(gdf: gpd.GeoDataFrame, expected_crs: str) -> gpd.GeoDataFrame:
    """Reproject ``gdf`` into ``expected_crs`` (CRS-discipline invariant)."""
    if gdf.crs is None:
        raise ValueError("vector layer has no CRS; every geospatial input must carry one")
    return gdf.to_crs(expected_crs)
