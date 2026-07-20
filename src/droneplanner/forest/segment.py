"""Vertical structure segmentation (Software-Spec §5.2 step 2, Preflight-Design §4).

Per column, split the profile into understory, trunk space, and canopy, and find
the tallest free vertical window above the understory — the room available to fly
(the "trunk-space window"). Heights are meters above each column's ground.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from droneplanner.forest.voxelize import VoxelGrid


@dataclass(frozen=True)
class VerticalProfile:
    """Per-column vertical-structure metrics (all 2D, shape rows x cols)."""

    understory_density: np.ndarray  # occupied fraction in [ground, understory_top]
    canopy_top_m: np.ndarray  # height of the highest occupied voxel (0 if none)
    free_window_m: np.ndarray  # tallest contiguous free run above understory
    free_window_base_level: np.ndarray  # int z-level where that run starts
    free_window_top_level: np.ndarray  # int z-level where that run ends (inclusive)


def segment_vertical(voxels: VoxelGrid, understory_top_m: float = 2.5) -> VerticalProfile:
    """Segment each column into understory / trunk space / canopy."""
    occ = voxels.occupancy  # (nz, rows, cols)
    nz, rows, cols = occ.shape
    vz = voxels.voxel_size_z
    understory_levels = max(1, round(understory_top_m / vz))

    understory_density = occ[:understory_levels].mean(axis=0).astype(np.float32)

    # Canopy top: highest occupied level per column (+1 -> top face), else 0.
    any_occ = occ.any(axis=0)
    highest = np.where(occ, np.arange(nz)[:, None, None], -1).max(axis=0)
    canopy_top_m = np.where(any_occ, (highest + 1) * vz, 0.0).astype(np.float32)

    free_window_m = np.zeros((rows, cols), dtype=np.float32)
    base_level = np.zeros((rows, cols), dtype=np.int32)
    top_level = np.zeros((rows, cols), dtype=np.int32)

    start = min(understory_levels, nz)
    for r in range(rows):
        for c in range(cols):
            col = occ[start:nz, r, c]
            best_len = best_base = best_top = 0
            run_start = None
            for i, occupied in enumerate(col):
                if not occupied:
                    if run_start is None:
                        run_start = i
                    run_len = i - run_start + 1
                    if run_len > best_len:
                        best_len = run_len
                        best_base = run_start + start
                        best_top = i + start
                else:
                    run_start = None
            free_window_m[r, c] = best_len * vz
            base_level[r, c] = best_base
            top_level[r, c] = best_top

    return VerticalProfile(
        understory_density=understory_density,
        canopy_top_m=canopy_top_m,
        free_window_m=free_window_m,
        free_window_base_level=base_level,
        free_window_top_level=top_level,
    )
