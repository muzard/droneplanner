"""GeoTIFF raster read/write (Software-Spec §4).

Thin ``rasterio`` wrappers returning aligned ``float32`` arrays plus their grid
metadata. ``rasterio`` is imported lazily so the M0 straight-line stub runs
(and unit-tests) without a GDAL stack present at import time.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class RasterGrid:
    """A single-band raster: array + the geotransform needed to place it."""

    data: np.ndarray  # float32, shape (rows, cols)
    transform: tuple[float, float, float, float, float, float]  # affine (a,b,c,d,e,f)
    crs: str
    nodata: float | None

    @property
    def shape(self) -> tuple[int, int]:
        return int(self.data.shape[0]), int(self.data.shape[1])


def read_raster(path: str | Path, band: int = 1) -> RasterGrid:
    """Read one band of a GeoTIFF as a ``float32`` ``RasterGrid``."""
    import rasterio  # lazy: heavy GDAL import

    with rasterio.open(Path(path)) as src:
        data = src.read(band).astype(np.float32)
        a = src.transform
        transform = (a.a, a.b, a.c, a.d, a.e, a.f)
        crs = src.crs.to_string() if src.crs is not None else ""
        nodata = None if src.nodata is None else float(src.nodata)
    return RasterGrid(data=data, transform=transform, crs=crs, nodata=nodata)


def write_raster(path: str | Path, grid: RasterGrid) -> None:
    """Write a ``RasterGrid`` to a single-band ``float32`` GeoTIFF."""
    import rasterio
    from rasterio.transform import Affine

    rows, cols = grid.shape
    a, b, c, d, e, f = grid.transform
    profile = {
        "driver": "GTiff",
        "dtype": "float32",
        "count": 1,
        "height": rows,
        "width": cols,
        "transform": Affine(a, b, c, d, e, f),
        "crs": grid.crs or None,
        "nodata": grid.nodata,
    }
    with rasterio.open(Path(path), "w", **profile) as dst:
        dst.write(grid.data.astype(np.float32), 1)
