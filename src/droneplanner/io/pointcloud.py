"""Point-cloud (LAZ/LAS 1.4) reading for the forest module (Software-Spec §4).

Lazily wraps ``laspy``. The forest voxelization/EDT pipeline (M2) consumes the
returned XYZ array + classification; this scaffold provides the reader and a
lightweight header summary so the I/O boundary is complete.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PointCloud:
    """Minimal point-cloud view: XYZ + optional classification codes."""

    xyz: np.ndarray  # shape (n, 3), float64 meters
    classification: np.ndarray | None  # shape (n,), uint8 or None
    crs: str | None

    @property
    def count(self) -> int:
        return int(self.xyz.shape[0])


def read_pointcloud(path: str | Path) -> PointCloud:
    """Read a LAS/LAZ file into a ``PointCloud`` (XYZ + classification)."""
    import laspy

    with laspy.open(Path(path)) as reader:
        las = reader.read()
    xyz = np.column_stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)]).astype(
        np.float64
    )
    classification = None
    if "classification" in las.point_format.dimension_names:
        classification = np.asarray(las.classification).astype(np.uint8)
    crs = None
    try:
        parsed = las.header.parse_crs()
        if parsed is not None:
            crs = parsed.to_string()
    except Exception:  # CRS VLR absent or unparseable — not fatal for reading points
        crs = None
    return PointCloud(xyz=xyz, classification=classification, crs=crs)
