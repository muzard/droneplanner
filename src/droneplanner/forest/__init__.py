"""Forest flyability model (Software-Spec §5.2, milestone M2).

Pipeline (Preflight-Design §4): voxelize lidar -> segment the vertical profile ->
3D EDT clearance -> classify navigability (FLYABLE / MARGINAL / NON_NAVIGABLE) and
expose a 2.5D flyability cost + non-navigable mask + max-flyable-altitude band for
the corridor planner. The OMPL 3D kinodynamic refinement inside the corridor is
future work (see SHORTFALLS.md).
"""

from __future__ import annotations

from droneplanner.forest.clearance import clearance_grid
from droneplanner.forest.navigability import (
    CLASS_TO_ENUM,
    FLYABLE,
    MARGINAL,
    NON_NAVIGABLE,
    FlyabilityConfig,
    FlyabilityField,
    build_flyability,
)
from droneplanner.forest.segment import VerticalProfile, segment_vertical
from droneplanner.forest.voxelize import VoxelGrid, voxelize

__all__ = [
    "CLASS_TO_ENUM",
    "FLYABLE",
    "MARGINAL",
    "NON_NAVIGABLE",
    "FlyabilityConfig",
    "FlyabilityField",
    "VerticalProfile",
    "VoxelGrid",
    "build_flyability",
    "clearance_grid",
    "segment_vertical",
    "voxelize",
]
