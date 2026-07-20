"""Road / linear-obstacle rasterization for the crossings stub (Software-Spec §5.4).

M1.1 treats road crossings as the spec's method 4 — **open crossing** (last
resort): roads are open, exposed terrain, so the drone simply flies over them and
the planner is nudged to spend as little on-road distance as possible (cross
roughly perpendicular) rather than routing to culverts/bridges. The fuller
feasibility-ranked crossing solver (culvert -> canopy bridge -> overhead span)
remains future work (Preflight-Design §5.2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from droneplanner.model.grid import Grid

if TYPE_CHECKING:
    import geopandas as gpd


def rasterize_roads(roads: gpd.GeoDataFrame, grid: Grid) -> np.ndarray:
    """Rasterize road geometries onto the grid as a boolean "on-road" mask."""
    from rasterio.features import rasterize
    from rasterio.transform import Affine

    gdf = roads.to_crs(grid.crs) if roads.crs is not None else roads
    a, b, c, d, e, f = grid.transform
    shapes = [(geom, 1) for geom in gdf.geometry if geom is not None and not geom.is_empty]
    if not shapes:
        return np.zeros((grid.rows, grid.cols), dtype=bool)
    burned = rasterize(
        shapes,
        out_shape=(grid.rows, grid.cols),
        transform=Affine(a, b, c, d, e, f),
        fill=0,
        all_touched=True,
        dtype="uint8",
    )
    return np.asarray(burned, dtype=bool)
