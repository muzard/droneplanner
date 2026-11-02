"""Vertical structure segmentation (Software-Spec §5.2 step 2, Preflight-Design §4).

Per column, split the profile into understory, trunk space, and canopy, and find
the tallest free vertical window above the understory — the room available to fly
(the "trunk-space window"). Heights are meters above each column's ground, which
requires ``ground_level``: a voxel level index is an *absolute* elevation (it
counts from ``VoxelGrid.z0``, the cloud minimum), so without the per-column ground
offset every derived height is wrong by that column's relief (SHORTFALLS 1b).
Omit it only for an already height-normalized cloud, where ``z0`` is the ground.
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
    free_window_base_level: np.ndarray  # int z-level where that run starts (absolute)
    free_window_top_level: np.ndarray  # int z-level where that run ends (absolute, inclusive)
    ground_level: np.ndarray  # int z-level of each column's ground (the height datum)


def segment_vertical(
    voxels: VoxelGrid,
    understory_top_m: float = 2.5,
    ground_level: np.ndarray | None = None,
) -> VerticalProfile:
    """Segment each column into understory / trunk space / canopy.

    ``ground_level`` is the per-column z-level of the ground surface
    (``VoxelGrid.ground_level(dtm)``). It is the datum every returned height is
    measured from. When omitted the grid is assumed already height-normalized
    (ground at level 0) — true for synthetic fixtures, false for any real cloud
    over relief.
    """
    occ = voxels.occupancy  # (nz, rows, cols)
    nz, rows, cols = occ.shape
    vz = voxels.voxel_size_z
    understory_levels = max(1, round(understory_top_m / vz))

    if ground_level is None:
        ground = np.zeros((rows, cols), dtype=np.int64)
    else:
        ground = np.clip(np.asarray(ground_level), 0, max(nz - 1, 0)).astype(np.int64)

    # Everything below is measured from each column's own ground level, so a
    # column standing above z0 no longer reads the void beneath it as understory
    # (or, worse, as the tallest free window available to fly through).
    levels = np.arange(nz)[:, None, None]
    above_ground = levels - ground[None]
    understory_band = (above_ground >= 0) & (above_ground < understory_levels)
    understory_density = ((occ & understory_band).sum(axis=0) / float(understory_levels)).astype(
        np.float32
    )

    # Canopy top: highest occupied level per column (+1 -> top face), else 0.
    any_occ = occ.any(axis=0)
    highest = np.where(occ, levels, -1).max(axis=0)
    canopy_top_m = np.where(any_occ, np.maximum(highest + 1 - ground, 0) * vz, 0.0).astype(
        np.float32
    )

    free_window_m = np.zeros((rows, cols), dtype=np.float32)
    base_level = np.zeros((rows, cols), dtype=np.int32)
    top_level = np.zeros((rows, cols), dtype=np.int32)

    for r in range(rows):
        for c in range(cols):
            start = min(int(ground[r, c]) + understory_levels, nz)
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
        ground_level=ground.astype(np.int32),
    )
