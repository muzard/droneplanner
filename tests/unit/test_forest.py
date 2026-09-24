"""Unit tests for the M2 forest flyability pipeline (Software-Spec §5.2, §6.2)."""

from __future__ import annotations

import numpy as np
import pytest

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


# --- SHORTFALLS 1b: the navigable band must be AGL, not height above z0 -------
#
# A voxel level index counts from the cloud's global minimum elevation, so before
# the per-column ground datum landed, identical forest standing on different
# ground reported three different bands: 26.45 / 56.45 / 2.95 m "AGL". The last
# one is the worst — a column standing above z0 found the *void beneath it* and
# called it the tallest free window available to fly through.

_GROUNDS = (0.0, 30.0, 70.0)  # local ground elevation per column, metres
_CANOPY_BASE_AGL = 12.0
_CANOPY_TOP_AGL = 20.0


def _relief_cloud(seal_top: bool) -> tuple[np.ndarray, Grid, np.ndarray]:
    """Identical forest over rising ground; returns (points, grid, dtm).

    ``seal_top`` fills each column's canopy to the top of the vertical extent, so
    no column has open sky above it and the tallest free window is unambiguously
    the trunk space. That isolates the datum from the separate question of which
    window gets picked.
    """
    grid = _grid(len(_GROUNDS))
    z_top = max(_GROUNDS) + _CANOPY_TOP_AGL
    dtm = np.zeros((grid.rows, grid.cols), dtype=np.float64)
    pts: list[tuple[float, float, float]] = []
    for r in range(grid.rows):
        for c in range(grid.cols):
            g = _GROUNDS[c]
            dtm[r, c] = g
            w = grid.pixel_to_world(r, c)
            top = z_top if seal_top else g + _CANOPY_TOP_AGL
            zs = [g, *np.arange(g + _CANOPY_BASE_AGL, top + _VZ, _VZ)]
            pts += [(w.x, w.y, float(z)) for z in zs]
    return np.array(pts), grid, dtm


def _voxelize_relief(seal_top: bool) -> tuple[VoxelGrid, np.ndarray]:
    pts, grid, dtm = _relief_cloud(seal_top)
    z = pts[:, 2]
    vox = voxelize(pts, grid, z_min=float(z.min()), z_max=float(z.max()) + _VZ, voxel_size_z=_VZ)
    return vox, dtm


def test_ground_level_maps_elevations_to_voxel_levels() -> None:
    vox, dtm = _voxelize_relief(seal_top=True)
    levels = vox.ground_level(dtm)
    assert vox.z0 == 0.0
    assert list(levels[0]) == [int(g / _VZ) for g in _GROUNDS]
    # Ground below the cloud minimum clamps to the bottom rather than indexing
    # negatively into the occupancy array.
    assert vox.ground_level(np.full_like(dtm, -500.0)).min() == 0


def test_canopy_top_is_height_above_each_columns_own_ground() -> None:
    vox, dtm = _voxelize_relief(seal_top=False)
    profile = segment_vertical(vox, ground_level=vox.ground_level(dtm))
    # Identical 20 m stands over 0/30/70 m of relief must all read ~20 m.
    tops = profile.canopy_top_m[0]
    assert np.allclose(tops, _CANOPY_TOP_AGL + _VZ, atol=_VZ)
    assert tops.min() == tops.max()
    # Without the datum every column reads its own elevation back.
    naive = segment_vertical(vox).canopy_top_m[0]
    assert naive[2] - naive[0] == pytest.approx(_GROUNDS[2] - _GROUNDS[0])


def test_navigable_band_is_agl_not_height_above_cloud_minimum() -> None:
    vox, dtm = _voxelize_relief(seal_top=True)
    config = FlyabilityConfig(r_drone=0.45)
    field = build_flyability(vox, config, ground_level=vox.ground_level(dtm))

    floors, ceils = field.nav_floor_m[0], field.nav_ceil_m[0]
    assert floors.min() == floors.max(), f"identical forest, different floors: {floors}"
    assert ceils.min() == ceils.max(), f"identical forest, different ceilings: {ceils}"
    # Trunk space: understory top (2.5 m) + airframe radius, up to the canopy
    # base less the same radius.
    assert floors[0] == pytest.approx(config.understory_top_m + config.r_drone)
    assert ceils[0] == pytest.approx(_CANOPY_BASE_AGL - config.r_drone)

    # The regression this pins: without the datum, a column standing above z0
    # finds the void *beneath its own ground* and reports it as the navigable
    # band. Every raised column's ceiling lands just under its own terrain.
    naive = build_flyability(vox, config)
    naive_ceils = naive.nav_ceil_m[0]
    assert len(set(naive_ceils.tolist())) == len(_GROUNDS), "identical forest read alike by luck"
    for col, ground in enumerate(_GROUNDS[1:], start=1):
        assert naive_ceils[col] < ground, (
            f"column {col} band tops out at {naive_ceils[col]} m, under its {ground} m ground"
        )
