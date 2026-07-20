"""Composite cost surface (Software-Spec §5.3 ``combine_objectives`` — weighted sum).

Fuses the normalized field layers into a single per-meter traversal-cost raster
the any-angle planner minimizes (Preflight-Design §6). Cost rises with drone
visual exposure, cable survivability cost, and snag risk, and falls under cover
(defilade). The individual layers are retained on the returned object so the
visualizer can show *why* the planner chose a route.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from droneplanner.fields.snag import build_snag
from droneplanner.fields.survivability import build_survivability
from droneplanner.fields.terrain import drainage_proxy, ridge_penalty, slope_penalty
from droneplanner.fields.util import normalize01
from droneplanner.fields.viewshed import cumulative_exposure
from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission

# Per-meter base movement cost, so a shorter route is preferred all else equal.
BASE_COST = 1.0
# Floor keeps every cell strictly positive (required by the search).
COST_FLOOR = 0.05
# How strongly the cover reward can discount a cell.
COVER_SCALE = 0.5


@dataclass(frozen=True)
class CostSurface:
    """Composite cost raster + its component layers (for search and viz)."""

    grid: Grid
    cost: np.ndarray  # float32, per-meter traversal cost, > 0
    dtm: np.ndarray
    dsm: np.ndarray
    components: dict[str, np.ndarray] = field(default_factory=dict)

    @property
    def shape(self) -> tuple[int, int]:
        return int(self.cost.shape[0]), int(self.cost.shape[1])


def build_cost_surface(
    dtm: np.ndarray, dsm: np.ndarray, grid: Grid, mission: Mission
) -> CostSurface:
    """Build the composite cost surface for a mission over a DTM/DSM."""
    weights = mission.weights
    observers = list(mission.threat.observers)

    exposure = cumulative_exposure(dsm, grid, observers)
    survivability = build_survivability(dtm, dsm, grid, observers)
    snag = build_snag((grid.rows, grid.cols))
    ridge = ridge_penalty(dtm)
    drainage = drainage_proxy(dtm)
    slope = slope_penalty(dtm, grid.cell_size_m)

    raw = (
        BASE_COST
        + weights.exposure * exposure
        + weights.survivability * survivability
        + weights.snag * snag
        - weights.cover * drainage * COVER_SCALE
    )
    cost = np.clip(raw, COST_FLOOR, None).astype(np.float32)

    components = {
        "exposure": exposure,
        "survivability": survivability,
        "snag": snag,
        "ridge": ridge,
        "drainage": drainage,
        "slope": slope,
        "cost_normalized": normalize01(cost),
    }
    return CostSurface(grid=grid, cost=cost, dtm=dtm, dsm=dsm, components=components)
