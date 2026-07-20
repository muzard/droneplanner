"""3D clearance via Euclidean Distance Transform (Software-Spec §5.2 step 3).

Every free voxel gets the distance (meters) to its nearest obstacle voxel — its
local clearance radius (Preflight-Design §4). Anisotropic voxels are handled by
passing per-axis sampling to the EDT.
"""

from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt

from droneplanner.forest.voxelize import VoxelGrid


def clearance_grid(voxels: VoxelGrid) -> np.ndarray:
    """Return a float32 (nz, rows, cols) grid of per-voxel clearance in meters.

    Occupied voxels have clearance 0. Free voxels hold the metric distance to the
    nearest occupied voxel, using the (z, y, x) voxel sizes as EDT sampling.
    """
    free = ~voxels.occupancy
    vz = voxels.voxel_size_z
    vxy = voxels.grid.cell_size_m
    # distance_transform_edt measures distance to the nearest zero (obstacle);
    # feed it the free mask so free voxels get distance to nearest occupied.
    dist = distance_transform_edt(free, sampling=(vz, vxy, vxy))
    return np.asarray(dist, dtype=np.float32)
