"""Navigability classification (Software-Spec §5.2 steps 4-5, Preflight-Design §4).

Combines 3D clearance with the vertical free window to classify each column
FLYABLE / MARGINAL / NON_NAVIGABLE against airframe-calibrated thresholds, and
exposes the 2.5D flyability cost + non-navigable mask + max-flyable-altitude band
that the corridor planner consumes. Thresholds are config (the §4 table is a
starting point — calibrate to the airframe in sim/flight).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from droneplanner.forest.clearance import clearance_grid
from droneplanner.forest.segment import segment_vertical
from droneplanner.forest.voxelize import VoxelGrid
from droneplanner.model.mission import DroneSpec
from droneplanner.model.plan import FlyabilityClass

# Integer class codes for the raster (map to FlyabilityClass via CLASS_TO_ENUM).
FLYABLE = 0
MARGINAL = 1
NON_NAVIGABLE = 2

CLASS_TO_ENUM = {
    FLYABLE: FlyabilityClass.FLYABLE,
    MARGINAL: FlyabilityClass.MARGINAL,
    NON_NAVIGABLE: FlyabilityClass.NON_NAVIGABLE,
}
_CLASS_COST = np.array([0.0, 0.5, 1.0], dtype=np.float32)


@dataclass(frozen=True)
class FlyabilityConfig:
    """Airframe-calibrated navigability thresholds (Preflight-Design §4)."""

    r_drone: float  # airframe half-width incl props (meters)
    e_state: float = 0.3  # expected state-estimation error (larger below canopy)
    s_margin: float = 0.3  # safety margin
    understory_top_m: float = 2.5
    flyable_window_m: float = 3.0
    marginal_window_m: float = 2.0

    @property
    def min_clearance_m(self) -> float:
        """Below this a cell is non-navigable (too tight for the airframe)."""
        return self.r_drone + self.e_state

    @property
    def flyable_clearance_m(self) -> float:
        return self.r_drone + self.e_state + self.s_margin

    @classmethod
    def from_drone(cls, drone: DroneSpec, **overrides: float) -> FlyabilityConfig:
        return cls(r_drone=drone.rotor_radius_m, **overrides)


@dataclass(frozen=True)
class FlyabilityField:
    """2.5D flyability products for the corridor planner."""

    navigable_class: np.ndarray  # int (0/1/2), shape rows x cols
    cost: np.ndarray  # float32 0..1
    non_navigable: np.ndarray  # bool
    max_flyable_altitude_m: np.ndarray  # float32, m above ground (best-clearance height)
    best_clearance_m: np.ndarray
    trunk_window_m: np.ndarray
    nav_floor_m: np.ndarray  # float32, bottom of the navigable altitude band (AGL)
    nav_ceil_m: np.ndarray  # float32, top of the navigable altitude band (AGL)


def build_flyability(voxels: VoxelGrid, config: FlyabilityConfig) -> FlyabilityField:
    """Classify navigability per column from a voxel occupancy grid."""
    profile = segment_vertical(voxels, understory_top_m=config.understory_top_m)
    clearance = clearance_grid(voxels)  # (nz, rows, cols)
    nz = clearance.shape[0]

    # Restrict the clearance search to each column's tallest free window.
    levels = np.arange(nz)[:, None, None]
    has_window = profile.free_window_m > 0.0
    band = (
        (levels >= profile.free_window_base_level[None])
        & (levels <= profile.free_window_top_level[None])
        & has_window[None]
    )
    banded = np.where(band, clearance, -1.0)
    best_clearance = np.where(has_window, banded.max(axis=0), 0.0).astype(np.float32)
    best_level = banded.argmax(axis=0)
    vz = voxels.voxel_size_z
    max_alt = np.where(has_window, (best_level + 0.5) * vz, 0.0).astype(np.float32)

    # Navigable altitude band = the free window, inset by the airframe radius so
    # the drone keeps clearance from the understory below and canopy above.
    floor = profile.free_window_base_level * vz + config.r_drone
    ceil = (profile.free_window_top_level + 1) * vz - config.r_drone
    mid = 0.5 * (profile.free_window_base_level + profile.free_window_top_level + 1) * vz
    invalid = (floor > ceil) | ~has_window
    nav_floor = np.where(invalid, mid, floor).astype(np.float32)
    nav_ceil = np.where(invalid, mid, ceil).astype(np.float32)

    window = profile.free_window_m
    non_nav = (best_clearance < config.min_clearance_m) | (window < config.marginal_window_m)
    flyable = (
        (best_clearance >= config.flyable_clearance_m)
        & (window >= config.flyable_window_m)
        & ~non_nav
    )
    klass = np.where(non_nav, NON_NAVIGABLE, np.where(flyable, FLYABLE, MARGINAL)).astype(np.int32)

    return FlyabilityField(
        navigable_class=klass,
        cost=_CLASS_COST[klass],
        non_navigable=(klass == NON_NAVIGABLE),
        max_flyable_altitude_m=max_alt,
        best_clearance_m=best_clearance,
        trunk_window_m=window.astype(np.float32),
        nav_floor_m=nav_floor,
        nav_ceil_m=nav_ceil,
    )


# The altitude band this field implies is assembled in ``fields/altitude.py``,
# which also handles the no-lidar case (a canopy ceiling from DSM - DTM). Keeping
# the policy there means the corridor planner and the cost surface read one
# definition — see that module's docstring.
