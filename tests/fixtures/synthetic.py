"""Synthetic fixtures with known ground truth (Software-Spec §6.1).

Analytic terrain and minimal valid missions. The DEM generators produce terrain
whose TPI/slope/viewshed are hand-derivable, seeding the M1 field tests; for M0
they exercise the raster I/O round-trip.
"""

from __future__ import annotations

from typing import Any, Literal

import numpy as np

from droneplanner.io.raster import RasterGrid
from droneplanner.model.mission import Mission

_DEFAULT_CRS = "EPSG:32633"


def make_mission(**overrides: Any) -> Mission:
    """Build a minimal valid mission, with keyword overrides."""
    base: dict[str, Any] = {
        "name": "test-ao",
        "crs": _DEFAULT_CRS,
        "start": {"x": 500000.0, "y": 5200000.0},
        "end": {"x": 501000.0, "y": 5200800.0},
        "drone": {
            "rotor_span_m": 0.9,
            "mass_kg": 6.5,
            "min_turn_radius_m": 8.0,
            "climb_rate_max_ms": 3.0,
            "descent_rate_max_ms": 2.0,
            "cruise_speed_ms": 6.0,
        },
        "spool": {"length_m": 5000.0, "mass_kg": 1.2, "payout_side": "drone"},
        "seed": 7,
    }
    base.update(overrides)
    return Mission.model_validate(base)


def make_dem(
    kind: Literal["gaussian_hill", "ridge", "valley", "flat"] = "gaussian_hill",
    size: int = 32,
    cell_m: float = 10.0,
    crs: str = _DEFAULT_CRS,
) -> RasterGrid:
    """Generate an analytic DEM as a ``float32`` ``RasterGrid``.

    Grid origin (top-left) is (0, size*cell) with north-up (negative y step),
    matching a standard GeoTIFF geotransform.
    """
    rows = cols = size
    yy, xx = np.mgrid[0:rows, 0:cols].astype(np.float64)
    cx = cy = (size - 1) / 2.0

    if kind == "flat":
        z = np.zeros((rows, cols), dtype=np.float64)
    elif kind == "gaussian_hill":
        sigma = size / 6.0
        z = 100.0 * np.exp(-(((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * sigma**2)))
    elif kind == "ridge":
        sigma = size / 8.0
        z = 100.0 * np.exp(-((xx - cx) ** 2) / (2.0 * sigma**2))
    elif kind == "valley":
        sigma = size / 8.0
        z = -100.0 * np.exp(-((xx - cx) ** 2) / (2.0 * sigma**2))
    else:  # pragma: no cover - exhaustive Literal
        raise ValueError(f"unknown DEM kind {kind!r}")

    transform = (cell_m, 0.0, 0.0, 0.0, -cell_m, rows * cell_m)
    return RasterGrid(data=z.astype(np.float32), transform=transform, crs=crs, nodata=None)


def make_closed_canopy(size: int = 21, canopy_m: float = 20.0) -> tuple[np.ndarray, np.ndarray]:
    """``(dtm, dsm)`` for flat ground under a uniform closed canopy.

    The minimal scene for roadmap C1: with no relief, *every* occlusion in the
    answer is the canopy's, so a look-down observer's result isolates the crown
    layer instead of confounding it with terrain masking.
    """
    dtm = np.zeros((size, size), dtype=np.float32)
    dsm = np.full((size, size), canopy_m, dtype=np.float32)
    return dtm, dsm


def make_las_bytes(n: int = 200, crs: str = _DEFAULT_CRS) -> bytes:
    """Serialize a tiny synthetic LAS point cloud to bytes (for I/O tests)."""
    import io

    import laspy

    rng = np.random.default_rng(0)
    header = laspy.LasHeader(point_format=3, version="1.4")
    las = laspy.LasData(header)
    las.x = 500000.0 + rng.uniform(0, 100, n)
    las.y = 5200000.0 + rng.uniform(0, 100, n)
    las.z = rng.uniform(0, 30, n)
    las.classification = np.full(n, 2, dtype=np.uint8)  # ground
    buffer = io.BytesIO()
    las.write(buffer)
    return buffer.getvalue()
