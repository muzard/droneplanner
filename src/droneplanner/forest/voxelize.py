"""Point cloud -> 3D occupancy voxel grid (Software-Spec §5.2 step 1).

Bins lidar returns into a boolean occupancy grid aligned horizontally to a
reference ``Grid`` and stacked in vertical layers of ``voxel_size_z``. This is
the keystone the rest of the forest pipeline (segmentation, clearance,
navigability) consumes (Preflight-Design §4).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from droneplanner.model.grid import Grid


@dataclass(frozen=True)
class VoxelGrid:
    """A 3D boolean occupancy grid; z index 0 is the lowest layer."""

    occupancy: np.ndarray  # bool, shape (nz, rows, cols)
    grid: Grid  # horizontal georeferencing (rows, cols, transform, crs)
    z0: float  # world elevation (m) of the bottom of layer 0
    voxel_size_z: float

    @property
    def nz(self) -> int:
        return int(self.occupancy.shape[0])

    def level_height_m(self, level: int) -> float:
        """Height (m above z0) of the *center* of a z-level."""
        return (level + 0.5) * self.voxel_size_z

    def ground_level(self, ground_elev_m: np.ndarray) -> np.ndarray:
        """Per-column z-level index of the ground surface, from a DTM.

        ``z0`` is an *absolute* elevation (the cloud's minimum), so a level index
        means the same height everywhere only over flat ground. Downstream code
        that reports heights **above each column's ground** needs this offset;
        without it a column standing 30 m higher reads its own canopy 30 m too
        high, and a column standing above ``z0`` reads the void beneath it as
        navigable (SHORTFALLS 1b).
        """
        levels = np.floor((np.asarray(ground_elev_m) - self.z0) / self.voxel_size_z)
        return np.clip(levels, 0, max(self.nz - 1, 0)).astype(np.int64)


def voxelize(
    points_xyz: np.ndarray,
    grid: Grid,
    z_min: float,
    z_max: float,
    voxel_size_z: float = 0.5,
) -> VoxelGrid:
    """Voxelize an (n, 3) XYZ array into an occupancy grid over ``grid``."""
    if voxel_size_z <= 0:
        raise ValueError("voxel_size_z must be positive")
    nz = max(1, int(np.ceil((z_max - z_min) / voxel_size_z)))
    occ = np.zeros((nz, grid.rows, grid.cols), dtype=bool)
    if points_xyz.size == 0:
        return VoxelGrid(occupancy=occ, grid=grid, z0=z_min, voxel_size_z=voxel_size_z)

    a, b, c, d, e, f = grid.transform
    det = a * e - b * d
    x = points_xyz[:, 0] - c
    y = points_xyz[:, 1] - f
    col = (e * x - b * y) / det
    row = (-d * x + a * y) / det
    ci = np.floor(col).astype(np.int64)
    ri = np.floor(row).astype(np.int64)
    zi = np.floor((points_xyz[:, 2] - z_min) / voxel_size_z).astype(np.int64)

    ok = (ri >= 0) & (ri < grid.rows) & (ci >= 0) & (ci < grid.cols) & (zi >= 0) & (zi < nz)
    occ[zi[ok], ri[ok], ci[ok]] = True
    return VoxelGrid(occupancy=occ, grid=grid, z0=z_min, voxel_size_z=voxel_size_z)
