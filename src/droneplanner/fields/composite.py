"""Composite cost surface (Software-Spec §5.3 ``combine_objectives`` — weighted sum).

Fuses the normalized field layers into a single per-meter traversal-cost raster
the any-angle planner minimizes (Preflight-Design §6). Cost rises with drone
visual exposure, cable survivability cost, and snag risk, and falls under cover
(defilade). The individual layers are retained on the returned object so the
visualizer can show *why* the planner chose a route.

Note the deliberate split between the two visibility layers: ``exposure`` is the
**drone** seen at its planned flight altitude, ``exposure_ground`` is the
**surface** (what the laid cable is exposed to, via ``survivability``). They are
different objects at different heights and are not interchangeable.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from droneplanner.crossings import ROAD_COST
from droneplanner.fields.altitude import band_raster, canopy_height, planned_flight_agl
from droneplanner.fields.snag import build_snag
from droneplanner.fields.stem_density import build_stem_clutter
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
    flight_agl_m: np.ndarray | None = None  # float32; altitude the exposure field assumes
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

    # Two viewsheds, two different objects — see SHORTFALLS item 1. The DRONE is
    # scored at the altitude it will fly (terrain + planned AGL: under the canopy
    # where there is canopy to get under, low over open ground); the CABLE lies on
    # the surface, so it keeps a ground-level viewshed. Scoring the drone at
    # ground level made the concealment metric — and the route it drives —
    # systematically wrong about the thing being hidden.
    chm = canopy_height(dtm, dsm)
    flight_agl = planned_flight_agl(chm, mission.cruise_agl_m, mission.altitude, flyability)
    flight_band = band_raster(chm, mission.cruise_agl_m, mission.altitude, flyability)
    exposure = cumulative_exposure(dsm, grid, observers, target_elev_m=dtm + flight_agl)
    ground_exposure = cumulative_exposure(dsm, grid, observers)

    survivability = build_survivability(dtm, dsm, grid, observers, openness=ground_exposure)
    snag = build_snag((grid.rows, grid.cols))
    ridge = ridge_penalty(dtm)
    drainage = drainage_proxy(dtm)
    slope = slope_penalty(dtm, grid.cell_size_m)

    # Stem clutter: how far apart the trunks are, and therefore how fast the
    # flight stack may go here (TEST-PLAN N3). Priced as extra *seconds per metre*
    # rather than as an abstract risk, because seconds are what the endurance
    # budget spends — so this term and the N1 check argue in the same currency.
    clutter = build_stem_clutter(
        chm,
        cruise_speed_ms=mission.drone.cruise_speed_ms,
        min_canopy_m=mission.altitude.min_canopy_m,
    )

    raw = (
        BASE_COST
        + weights.exposure * exposure
        + weights.survivability * survivability
        + weights.snag * snag
        + weights.clutter * clutter["clutter_time_penalty"]
        - weights.cover * drainage * COVER_SCALE
    )

    components = {
        "exposure": exposure,  # the drone, at flight altitude
        "exposure_ground": ground_exposure,  # the cable, on the surface
        "flight_agl": flight_agl,
        "canopy_height": chm,
        "flight_band": flight_band.astype(np.float32),
        "survivability": survivability,
        "snag": snag,
        "ridge": ridge,
        "drainage": drainage,
        "slope": slope,
        **clutter,
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
        flight_agl_m=flight_agl,
        components=components,
        obstacle=obstacle,
        road_mask=road_mask,
        flyability=flyability,
    )
