"""Unit tests for the M2 forest flyability pipeline (Software-Spec §5.2, §6.2)."""

from __future__ import annotations

import numpy as np

from droneplanner.forest.clearance import clearance_grid
from droneplanner.forest.navigability import (
    FLYABLE,
    NON_NAVIGABLE,
    FlyabilityConfig,
    build_flyability,
)
from droneplanner.forest.segment import segment_vertical
from droneplanner.forest.voxelize import VoxelGrid, voxelize
from droneplanner.model.grid import Grid

_VZ = 0.5


def _grid(size: int, cell: float = 0.5) -> Grid:
    return Grid(
        transform=(cell, 0.0, 0.0, 0.0, -cell, size * cell), rows=size, cols=size, crs="EPSG:32633"
    )


def _forest(size: int = 12, nz: int = 24, dense_block: bool = False) -> VoxelGrid:
    """Ground + canopy everywhere; optionally a dense (non-navigable) block."""
    occ = np.zeros((nz, size, size), dtype=bool)
    occ[0, :, :] = True  # ground
    occ[20:24, :, :] = True  # canopy (~10-12 m)
    if dense_block:
        occ[1:20, 4:8, 4:8] = True  # fill the trunk space in a block
    return VoxelGrid(occupancy=occ, grid=_grid(size), z0=0.0, voxel_size_z=_VZ)


def test_voxelize_bins_points() -> None:
    grid = _grid(10)
    # One point at world (2.25, 2.25) -> cell (row, col); z=3.0 -> level 6.
    pts = np.array([[2.25, 10 * 0.5 - 2.25, 3.0]])
    vox = voxelize(pts, grid, z_min=0.0, z_max=5.0, voxel_size_z=0.5)
    assert vox.occupancy.sum() == 1
    assert vox.occupancy[6].sum() == 1


def test_voxelize_empty() -> None:
    vox = voxelize(np.empty((0, 3)), _grid(8), z_min=0.0, z_max=4.0, voxel_size_z=0.5)
    assert not vox.occupancy.any()


def test_segment_free_window() -> None:
    vox = _forest()
    profile = segment_vertical(vox, understory_top_m=2.5)
    # Free window between understory top (2.5 m) and canopy base (10 m) ~ 7.5 m.
    assert profile.free_window_m[0, 0] > 6.0
    assert profile.canopy_top_m[0, 0] > 10.0


def test_clearance_zero_on_obstacle_and_positive_in_free() -> None:
    vox = _forest()
    clr = clearance_grid(vox)
    assert clr[0, 0, 0] == 0.0  # ground voxel is occupied
    assert clr[10, 6, 6] > 0.0  # mid-column free voxel has clearance


def test_open_forest_is_flyable() -> None:
    field = build_flyability(_forest(), FlyabilityConfig(r_drone=0.45))
    assert field.navigable_class[1, 1] == FLYABLE
    assert not field.non_navigable[1, 1]
    assert field.max_flyable_altitude_m[1, 1] > 2.5


def test_dense_block_is_non_navigable() -> None:
    field = build_flyability(_forest(dense_block=True), FlyabilityConfig(r_drone=0.45))
    assert field.navigable_class[5, 5] == NON_NAVIGABLE
    assert field.non_navigable[5, 5]
    # Open cells outside the block stay flyable.
    assert field.navigable_class[1, 1] == FLYABLE


def test_config_from_drone() -> None:
    from droneplanner.model.mission import DroneSpec

    drone = DroneSpec(
        rotor_span_m=1.0,
        mass_kg=5.0,
        min_turn_radius_m=8.0,
        climb_rate_max_ms=3.0,
        descent_rate_max_ms=2.0,
    )
    cfg = FlyabilityConfig.from_drone(drone, s_margin=0.4)
    assert cfg.r_drone == 0.5
    assert cfg.s_margin == 0.4
    assert cfg.flyable_clearance_m == 0.5 + cfg.e_state + 0.4
