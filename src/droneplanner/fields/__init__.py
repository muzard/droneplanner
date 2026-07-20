"""Derived 2.5D cost/constraint fields (Software-Spec §5.1).

M1 builders (each a pure function: input grids + config -> normalized field):
``terrain`` (slope, TPI, ridge/drainage), ``viewshed`` (cumulative visual
exposure / hideability), ``survivability`` (defilade / fragment exposure),
``snag`` (fiber snag risk), and ``composite`` (the weighted-sum cost surface the
planner minimizes). The forest voxel flyability field lands in M2.
"""

from __future__ import annotations

from droneplanner.fields.composite import CostSurface, build_cost_surface
from droneplanner.fields.snag import build_snag
from droneplanner.fields.survivability import build_survivability
from droneplanner.fields.terrain import (
    drainage_proxy,
    ridge_penalty,
    slope,
    slope_penalty,
    tpi,
)
from droneplanner.fields.viewshed import cumulative_exposure

__all__ = [
    "CostSurface",
    "build_cost_surface",
    "build_snag",
    "build_survivability",
    "cumulative_exposure",
    "drainage_proxy",
    "ridge_penalty",
    "slope",
    "slope_penalty",
    "tpi",
]
