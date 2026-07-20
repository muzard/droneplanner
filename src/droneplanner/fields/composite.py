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

from droneplanner.crossings import ROAD_COST
from droneplanner.fields.snag import build_snag
from droneplanner.fields.survivability import build_survivability
from droneplanner.fields.terrain import drainage_proxy, ridge_penalty, slope_penalty
from droneplanner.fields.util import normalize01
from droneplanner.fields.viewshed import cumulative_exposure
from droneplanner.forest.navigability import FlyabilityField
from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission

# Per-meter base movement cost, so a shorter route is preferred all else equal.
BASE_COST = 1.0
# Floor keeps every cell strictly positive (required by the search).
COST_FLOOR = 0.05
# How strongly the terrain-defilade cover reward can discount a cell.
COVER_SCALE = 0.5
# Navigable forest is high-value cover/concealment, so FLYABLE cells are
# *rewarded* (cost discount); MARGINAL cells carry a navigability-risk penalty;
# NON_NAVIGABLE cells are obstacles (handled separately).
FOREST_COVER_REWARD = 0.5
FOREST_MARGINAL_PENALTY = 0.5


@dataclass(frozen=True)
class CostSurface:
    """Composite cost raster + its component layers (for search and viz)."""

    grid: Grid
    cost: np.ndarray  # float32, per-meter traversal cost, > 0
    dtm: np.ndarray
    dsm: np.ndarray
    components: dict[str, np.ndarray] = field(default_factory=dict)
    obstacle: np.ndarray | None = None  # bool; non-navigable cells the planner must avoid
    road_mask: np.ndarray | None = None  # bool; on-road cells (open crossings)
    flyability: FlyabilityField | None = None

    @property
    def shape(self) -> tuple[int, int]:
        return int(self.cost.shape[0]), int(self.cost.shape[1])


def build_cost_surface(
    dtm: np.ndarray,
    dsm: np.ndarray,
    grid: Grid,
    mission: Mission,
    *,
    road_mask: np.ndarray | None = None,
    flyability: FlyabilityField | None = None,
) -> CostSurface:
    """Build the composite cost surface for a mission over a DTM/DSM.

    ``road_mask`` (M1.1) adds an open-terrain penalty so the route minimizes
    on-road distance. ``flyability`` (M2) adds a marginal-flyability penalty and
    marks non-navigable cells as obstacles for the search.
    """
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

    components = {
        "exposure": exposure,
        "survivability": survivability,
        "snag": snag,
        "ridge": ridge,
        "drainage": drainage,
        "slope": slope,
    }
    if road_mask is not None:
        raw = raw + ROAD_COST * road_mask.astype(np.float32)
        components["roads"] = road_mask.astype(np.float32)
    if flyability is not None:
        klass = flyability.navigable_class
        # FLYABLE -> reward (cover), MARGINAL -> penalty, NON_NAVIGABLE -> obstacle.
        forest_term = np.select(
            [klass == 0, klass == 1],
            [-FOREST_COVER_REWARD, FOREST_MARGINAL_PENALTY],
            default=0.0,
        ).astype(np.float32)
        raw = raw + forest_term
        components["flyability"] = flyability.cost
        components["flyability_class"] = klass.astype(np.float32)

    cost = np.clip(raw, COST_FLOOR, None).astype(np.float32)
    components["cost_normalized"] = normalize01(cost)

    obstacle = flyability.non_navigable if flyability is not None else None
    return CostSurface(
        grid=grid,
        cost=cost,
        dtm=dtm,
        dsm=dsm,
        components=components,
        obstacle=obstacle,
        road_mask=road_mask,
        flyability=flyability,
    )
