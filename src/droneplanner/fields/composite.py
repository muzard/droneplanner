"""Composite cost surface (Software-Spec §5.3 ``combine_objectives`` — weighted sum).

Fuses the normalized field layers into a single per-meter traversal-cost raster
the any-angle planner minimizes (Preflight-Design §6). Cost rises with drone
visual exposure, cable survivability cost, and snag risk, and falls under cover
(defilade). The individual layers are retained on the returned object so the
visualizer can show *why* the planner chose a route.

Note the deliberate split between the two visibility layers, which since roadmap
C2 answer **different questions** and not merely at different heights:

* ``exposure`` — the **drone** at its planned flight altitude, weighted by crown
  occlusion (C1) and by whether a 0.9 m airframe is resolvable at that slant range
  (C2). Read it as an approximate probability that a searching observer acquires
  the aircraft.
* ``ground_los`` — the **surface** the cable lies on, pure line of sight with no
  detection model, because a 0.28 mm fiber is not optically resolvable at any
  range. Read it as openness, never as a detection probability. It was called
  ``exposure_ground`` until C2; the name changed precisely so the two cannot be
  compared as if they were the same currency.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from droneplanner.crossings import ROAD_COST
from droneplanner.fields.altitude import (
    band_raster,
    canopy_height,
    concealment_mask,
    planned_flight_agl,
    skim_canopy,
)
from droneplanner.fields.snag import build_snag
from droneplanner.fields.stem_density import build_stem_clutter
from droneplanner.fields.survivability import build_survivability
from droneplanner.fields.terrain import drainage_proxy, ridge_penalty, slope_penalty
from droneplanner.fields.util import normalize01
from droneplanner.fields.viewshed import CrownMedium, TrunkSpace, cumulative_exposure
from droneplanner.forest.navigability import FlyabilityField
from droneplanner.io.stand_data import stand_density_raster
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
    # Which way each observer is facing (roadmap C3). Empty sectors reproduce the
    # all-round threat exactly, so this is inert until a mission states a posture.
    sectors = mission.threat.sectors
    vigilance = mission.threat.off_axis_vigilance

    # Two viewsheds, two different objects — see SHORTFALLS item 1. The DRONE is
    # scored at the altitude it will fly (terrain + planned AGL: under the canopy
    # where there is canopy to get under, low over open ground); the CABLE lies on
    # the surface, so it keeps a ground-level viewshed. Scoring the drone at
    # ground level made the concealment metric — and the route it drives —
    # systematically wrong about the thing being hidden.
    chm = canopy_height(dtm, dsm)
    # Observers at their vantage points (crown transmission only; inert otherwise).
    placed = vantage_mission(mission, chm, grid).threat
    observers = list(placed.observers)
    sectors = placed.sectors
    # Where concealment is required at all (Mission.covered_radius_m). Outside it
    # the drone flies over the canopy, and its exposure is not costed.
    covered = concealment_mask(grid, mission.start, mission.covered_radius_m)
    # Stem clutter needs the band (only stems the drone flies AMONG limit it), and
    # the band may now need the clutter: a stand denser than the onboard envelope
    # is flown OVER, not planned through (AltitudePolicy.over_canopy_when_unpassable).
    measured = (
        stand_density_raster(mission.layers.stand_data, grid)
        if mission.layers.stand_data is not None
        else None
    )

    def _clutter(band: np.ndarray) -> dict[str, np.ndarray]:
        return build_stem_clutter(
            chm,
            cruise_speed_ms=mission.drone.cruise_speed_ms,
            min_canopy_m=mission.altitude.min_canopy_m,
            among_stems=band >= 2,
            density_override_per_ha=mission.stem.density_per_ha,
            measured_per_ha=measured,
        )

    # Over-canopy legs clear the tallest canopy within the skim lookahead, not each
    # cell's own top (AltitudePolicy.skim_lookahead_m).
    skim = skim_canopy(chm, mission.altitude, grid.cell_size_m)
    flight_band = band_raster(
        chm, mission.cruise_agl_m, mission.altitude, flyability, covered=covered, skim=skim
    )
    clutter = _clutter(flight_band)
    forced_over = np.zeros(chm.shape, dtype=bool)
    enterable_zone = covered
    if mission.altitude.over_canopy_when_unpassable:
        forced_over = _regularise_forced(clutter["beyond_envelope"] > 0.5)
        if flyability is not None:
            # A lidar window is MEASURED free space; an inferred density must not
            # overrule it.
            forced_over &= ~(flyability.nav_ceil_m > 0.0)
        if covered is not None:
            forced_over &= covered
        if forced_over.any():
            zone_now = np.ones(chm.shape, dtype=bool) if covered is None else covered
            enterable_zone = zone_now & ~forced_over
            flight_band = band_raster(
                chm,
                mission.cruise_agl_m,
                mission.altitude,
                flyability,
                covered=enterable_zone,
                skim=skim,
            )
            clutter = _clutter(flight_band)
    flight_agl = planned_flight_agl(
        chm, mission.cruise_agl_m, mission.altitude, flyability, covered=enterable_zone, skim=skim
    )
    eye_m = mission.threat.observer_height_m
    exposure = cumulative_exposure(
        dsm,
        grid,
        observers,
        observer_height_m=eye_m,
        target_elev_m=dtm + flight_agl,
        canopy=mission.threat.canopy_occlusion,
        detection=mission.threat.detection,
        target_span_m=mission.drone.rotor_span_m,
        sectors=sectors,
        off_axis_vigilance=vigilance,
        trunk_space=trunk_space_for(mission, dtm, chm, clutter["stem_density_per_ha"]),
        gap_map=gap_map_for(mission, grid),
        medium=crown_medium_for(mission, dtm, dsm, clutter["stem_density_per_ha"], grid),
    )
    # The ground field gets NEITHER model, and both omissions are deliberate.
    # Canopy: its target *is* the DSM surface, so a cell's own crown is never
    # between the observer and it. Detection: the laid fiber is 0.25-0.28 mm and is
    # never optically resolvable at any range, so a visual acquisition model sized
    # for it would zero the field and delete cable survivability from the cost
    # surface. It therefore stays a **pure line-of-sight openness proxy** — and is
    # named `ground_los`, not `exposure_*`, so it cannot be read as a detection
    # probability now that `exposure` is one (roadmap C2; SHORTFALLS records this).
    ground_los = cumulative_exposure(
        dsm,
        grid,
        observers,
        observer_height_m=eye_m,
        sectors=sectors,
        off_axis_vigilance=vigilance,
    )

    # Sectors reach survivability twice, and deliberately so: the observer-bound
    # ones are already inside `ground_los` (they masked the viewshed), while the
    # bearing-only ones have no sensor to run a viewshed from and become the
    # terrain-masking openness term the formula has named since M0.
    survivability = build_survivability(
        dtm,
        dsm,
        grid,
        observers,
        openness=ground_los,
        sectors=sectors,
        off_axis_vigilance=vigilance,
    )
    snag = build_snag((grid.rows, grid.cols))
    ridge = ridge_penalty(dtm)
    drainage = drainage_proxy(dtm)
    slope = slope_penalty(dtm, grid.cell_size_m)

    # Stem clutter: how far apart the trunks are, and therefore how fast the
    # flight stack may go here (TEST-PLAN N3). Priced as extra *seconds per metre*
    # rather than as an abstract risk, because seconds are what the endurance
    # budget spends — so this term and the N1 check argue in the same currency.

    # Drone concealment terms apply only inside the concealment zone; the cable's
    # survivability applies everywhere, because the fiber stays on the ground.
    zone = np.ones(chm.shape, np.float32) if covered is None else covered.astype(np.float32)
    raw = (
        BASE_COST
        + weights.exposure * exposure * zone
        + weights.survivability * survivability
        + weights.snag * snag
        + weights.clutter * clutter["clutter_time_penalty"]
        + weights.cable_on_crowns * _crown_laydown(chm, flight_band, mission, grid)
        - weights.cover * drainage * COVER_SCALE * zone
    )

    components = {
        "exposure": exposure,  # the drone: occlusion x crown gap x detection range
        "ground_los": ground_los,  # the cable: line of sight only, no detection model
        "flight_agl": flight_agl,
        "canopy_height": chm,
        "skim_canopy": skim.astype(np.float32),
        "flight_band": flight_band.astype(np.float32),
        "covered": zone,
        # Cells inside the zone flown OVER because they cannot be flown under, and
        # the zone the altitude policy may enter — the corridor must use this one,
        # or it would plan under a stand the cost surface priced as flown over.
        "forced_over_canopy": forced_over.astype(np.float32),
        # Where the aircraft can get up through the canopy, or back down into the
        # trunk layer (planner/corridor.py places every under/over transition at one).
        "canopy_opening": _canopy_openings(mission, chm, grid),
        "enterable_zone": (
            np.ones(chm.shape, np.float32)
            if enterable_zone is None
            else enterable_zone.astype(np.float32)
        ),
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


FORCED_CLOSE_CELLS = 2
"""Gaps of up to this many cells between flown-over stands are flown over too."""


def _regularise_forced(mask: np.ndarray) -> np.ndarray:
    """Close small gaps and pad by one cell, so over/under does not porpoise.

    🔴 Why: along a canopy gradient the envelope edge (1200 stems/ha ≈ a 12.7 m
    canopy by inference) is crossed cell by cell, and a route that alternates
    under (~10 m AGL) and over (~16 m) on consecutive 25 m legs demands more
    climb than the airframe has and breaks the terrain guarantee. One decision
    per stand, with room to climb before it, is what an operator can read and a
    flight controller can fly.
    """
    from scipy.ndimage import binary_closing, binary_dilation

    if not mask.any():
        return mask
    square = np.ones((3, 3), dtype=bool)
    closed = binary_closing(mask, structure=square, iterations=FORCED_CLOSE_CELLS)
    out: np.ndarray = binary_dilation(closed | mask, structure=square)
    return out


OPENING_HEIGHT_M = 5.0
"""Canopy lower than this is an opening (matches tools/build_finnish_ao.py)."""


def _canopy_openings(mission: Mission, chm: np.ndarray, grid: Grid) -> np.ndarray:
    """1 where a climb out of / descent into the canopy has room, else 0.

    From ``layers.canopy_gaps`` (the 1 m canopy model's open columns) when given;
    otherwise only open ground, and the cells beside it (a stand edge), count —
    a DSM grid cannot see the strip roads and crown gaps a 1 m model can.
    """
    from scipy.ndimage import binary_dilation

    path = mission.layers.canopy_gaps
    if path is not None:
        from droneplanner.io.raster import read_raster

        r = read_raster(path)
        if r.shape != (grid.rows, grid.cols) or tuple(r.transform) != tuple(grid.transform):
            raise ValueError(f"{path}: canopy_gaps must be on the DTM grid (shape + transform)")
        return (r.data > 0.5).astype(np.float32)
    open_ground = chm < OPENING_HEIGHT_M
    out: np.ndarray = binary_dilation(open_ground, structure=np.ones((3, 3), dtype=bool))
    return out.astype(np.float32)


def _crown_laydown(
    chm: np.ndarray, flight_band: np.ndarray, mission: Mission, grid: Grid
) -> np.ndarray:
    """1 where the drone flies OVER canopy with no opening — the fiber settles on the
    crowns there (laydown/settle.py). 0 when ``weights.cable_on_crowns`` is 0."""
    from droneplanner.laydown.settle import CROWN_SETTLE_M

    if mission.weights.cable_on_crowns <= 0.0:
        return np.zeros(chm.shape, dtype=np.float32)
    over = flight_band < 2  # not flown among the stems
    opening = _canopy_openings(mission, chm, grid) > 0.5
    out: np.ndarray = (over & (chm >= CROWN_SETTLE_M) & ~opening).astype(np.float32)
    return out


def trunk_space_for(
    mission: Mission, dtm: np.ndarray, chm: np.ndarray, stems_per_ha: np.ndarray
) -> TrunkSpace | None:
    """The under-crown sightline inputs, or None when the mission leaves it off.

    One builder for the raster field and the flown re-score (planner/corridor.py),
    so the two cannot disagree about what an observer sees through the trunks."""
    occ = mission.threat.canopy_occlusion
    if not occ.trunk_space_sightlines:
        return None
    ratio = mission.altitude.crown_ratio or occ.trunk_space_crown_ratio
    return TrunkSpace(
        dtm=np.asarray(dtm, dtype=np.float32),
        crown_base_agl_m=(np.asarray(chm, dtype=np.float32) * (1.0 - ratio)),
        stems_per_m2=np.asarray(stems_per_ha, dtype=np.float32) / 10_000.0,
        trunk_diameter_m=occ.trunk_diameter_m,
        max_range_m=occ.trunk_space_max_range_m,
    )


def vantage_mission(mission: Mission, chm: np.ndarray, grid: Grid) -> Mission:
    """``mission`` with observers moved to their vantage points
    (``CanopyOcclusion.observer_vantage_m``), or unchanged.

    Only under crown transmission, where the eye is on the ground: an observer in
    vegetation taller than its eye moves to the nearest open cell within the radius;
    sectors bound to it move with it so the binding survives. One function for the
    cost surface and the flown re-score, so they cannot disagree."""
    occ = mission.threat.canopy_occlusion
    if not occ.crown_transmission or occ.observer_vantage_m <= 0.0:
        return mission
    from scipy.ndimage import distance_transform_edt

    from droneplanner.model.geometry import Coordinate

    blocked = np.asarray(chm) >= max(occ.crown_open_below_m, mission.threat.observer_height_m)
    if blocked.all() or not blocked.any():
        return mission
    dist, idx = distance_transform_edt(blocked, return_indices=True)

    def move(c: Coordinate) -> Coordinate:
        r, col = grid.world_to_pixel(c)
        if not blocked[r, col] or dist[r, col] * grid.cell_size_m > occ.observer_vantage_m:
            return c
        p = grid.pixel_to_world(int(idx[0, r, col]), int(idx[1, r, col]))
        return Coordinate(x=p.x, y=p.y)

    threat = mission.threat.model_copy(
        update={
            "observers": [move(o) for o in mission.threat.observers],
            "sectors": [
                s if s.observer is None else s.model_copy(update={"observer": move(s.observer)})
                for s in mission.threat.sectors
            ],
        }
    )
    return mission.model_copy(update={"threat": threat})


def crown_medium_for(
    mission: Mission,
    dtm: np.ndarray,
    dsm: np.ndarray,
    stems_per_ha: np.ndarray,
    grid: Grid,
) -> CrownMedium | None:
    """The canopy as a medium (``CanopyOcclusion.crown_transmission``), or None.

    Crown base from ``AltitudePolicy.crown_ratio`` (else the occlusion's own
    ratio), vertical gap probability from the measured map (else the scalar), stem
    density from the clutter field. One builder for the raster and the flown
    re-score, so they cannot disagree."""
    occ = mission.threat.canopy_occlusion
    if not occ.crown_transmission:
        return None
    ratio = mission.altitude.crown_ratio or occ.trunk_space_crown_ratio
    chm = np.clip(np.asarray(dsm, dtype=np.float64) - np.asarray(dtm, dtype=np.float64), 0.0, None)
    measured = gap_map_for(mission, grid)
    return CrownMedium.build(
        dtm,
        dsm,
        crown_base_agl_m=chm * (1.0 - ratio),
        gap=occ.sub_canopy_gap_fraction if measured is None else measured,
        stems_per_m2=np.asarray(stems_per_ha, dtype=np.float64) / 10_000.0,
        trunk_diameter_m=occ.trunk_diameter_m,
        open_below_m=occ.crown_open_below_m,
    )


def gap_map_for(mission: Mission, grid: Grid) -> np.ndarray | None:
    """Measured per-cell crown-gap probability (``layers.canopy_gap_fraction``), or
    None to use the scalar ``sub_canopy_gap_fraction``. Shared by the raster field and
    the flown re-score so they cannot disagree."""
    path = mission.layers.canopy_gap_fraction
    if path is None:
        return None
    from droneplanner.io.raster import read_raster

    r = read_raster(path)
    if r.shape != (grid.rows, grid.cols) or tuple(r.transform) != tuple(grid.transform):
        raise ValueError(f"{path}: canopy_gap_fraction must be on the DTM grid")
    out: np.ndarray = np.clip(r.data, 0.0, 1.0).astype(np.float32)
    return out
