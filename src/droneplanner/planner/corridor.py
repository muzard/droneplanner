"""2.5D corridor planner (Software-Spec §5.3, milestones M1/M2).

Ties the composite cost surface to the any-angle Theta* search and packages the
result as a ``Plan``: a concealment- and defilade-aware route with per-segment
exposure, flyability class, road crossings, and route-level objective scores.
When a forest flyability field is present (M2) the search treats non-navigable
cells as obstacles, segments carry their FLYABLE/MARGINAL class, and vertices
fly the max-flyable-altitude band. The OMPL 3D kinodynamic refinement inside the
corridor is future work (see SHORTFALLS.md).
"""

from __future__ import annotations

from itertools import pairwise

import numpy as np

from droneplanner.crossings.detect import detect_crossings
from droneplanner.fields.composite import CostSurface, build_cost_surface
from droneplanner.forest.navigability import CLASS_TO_ENUM, FlyabilityField
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission
from droneplanner.model.plan import (
    Crossing,
    FlightBand,
    FlyabilityClass,
    KinematicReport,
    Plan,
    RouteSegment,
    TrajectoryVertex,
)
from droneplanner.planner.kinematics import (
    KinematicLimits,
    fillet_path,
    smooth_altitudes,
)
from droneplanner.planner.terrain_profile import (
    FlownProfile,
    assess_clearance,
    densify_unsafe_descents,
)
from droneplanner.planner.theta_star import path_cost, plan_path
from droneplanner.raster_lines import bresenham

# A path cell counts as "exposed" when this fraction of observers can see it.
EXPOSED_THRESHOLD = 0.5

# Terrain clearance floor (m AGL) the flown descent profile must keep. Below this
# a leg is a CFIT finding, not a warning — see planner/terrain_profile.py.
MIN_TERRAIN_CLEARANCE_M = 5.0


def _segment_cells(a: tuple[int, int], b: tuple[int, int]) -> list[tuple[int, int]]:
    return bresenham(a[0], a[1], b[0], b[1])


def _mean_over(field: np.ndarray, cells: list[tuple[int, int]]) -> float:
    if not cells:
        return 0.0
    return float(np.mean([field[r, c] for r, c in cells]))


def _segment_flyability(
    flyability: FlyabilityField | None, cells: list[tuple[int, int]]
) -> FlyabilityClass:
    """Worst (highest-code) flyability class along a segment's cells."""
    if flyability is None:
        return FlyabilityClass.UNKNOWN
    worst = max(int(flyability.navigable_class[r, c]) for r, c in cells)
    return CLASS_TO_ENUM[worst]


def _altitude_band(
    flyability: FlyabilityField | None, rc: tuple[int, int], cruise_agl_m: float
) -> tuple[float, float, float, FlightBand]:
    """Return (preferred, floor, ceil, band) altitude for a cell.

    Without forest: a fixed cruise-AGL band. With forest: the navigable vertical
    window (altitude is free to vary within it to find/keep the gap).
    """
    if flyability is None:
        return cruise_agl_m, cruise_agl_m, cruise_agl_m, FlightBand.OPEN
    floor = float(flyability.nav_floor_m[rc])
    ceil = float(flyability.nav_ceil_m[rc])
    pref = float(flyability.max_flyable_altitude_m[rc])
    if ceil <= 0.0:
        return cruise_agl_m, cruise_agl_m, cruise_agl_m, FlightBand.OPEN
    return min(max(pref, floor), ceil), floor, ceil, FlightBand.UPPER_TRUNK


def plan_corridor(mission: Mission, surface: CostSurface, route_index: int = 0) -> Plan:
    """Plan the concealment-aware corridor route over a cost surface."""
    grid = surface.grid
    start_rc = grid.world_to_pixel(mission.start)
    goal_rc = grid.world_to_pixel(mission.end)

    cell_path = plan_path(surface.cost, start_rc, goal_rc, obstacle=surface.obstacle)
    exposure = surface.components["exposure"]
    flyability = surface.flyability
    speed = mission.drone.cruise_speed_ms

    # World coordinates: exact mission endpoints, cell centers for turning points.
    def world_of(i: int, rc: tuple[int, int]) -> Coordinate:
        if i == 0:
            return mission.start
        if i == len(cell_path) - 1:
            return mission.end
        return grid.pixel_to_world(rc[0], rc[1])

    coords = [world_of(i, rc) for i, rc in enumerate(cell_path)]

    # --- Kinematic feasibility pass (Software-Spec §3.6) ---
    limits = KinematicLimits.from_drone(mission.drone)

    def _blocked(x: float, y: float) -> bool:
        if surface.obstacle is None:
            return False
        return bool(surface.obstacle[grid.world_to_pixel(Coordinate(x=x, y=y))])

    filleted, turn_violations, reversals, tightest = fillet_path(
        [(c.x, c.y) for c in coords], limits, _blocked
    )

    def _ground_at(x: float, y: float) -> float:
        return float(surface.dtm[grid.world_to_pixel(Coordinate(x=x, y=y))])

    def _solve_altitudes(
        pts: list[tuple[float, float]],
    ) -> tuple[list[Coordinate], list[float], list[float], list[FlightBand], int, float, float]:
        """Band + climb-smooth a point list into per-vertex AGL and AMSL altitudes."""
        cs = [Coordinate(x=x, y=y) for x, y in pts]
        prefs, floors, ceils, bands = [], [], [], []
        for p in cs:
            pref, floor, ceil, band = _altitude_band(
                flyability, grid.world_to_pixel(p), mission.cruise_agl_m
            )
            prefs.append(pref)
            floors.append(floor)
            ceils.append(ceil)
            bands.append(band)
        dists = [a.distance_to(b) for a, b in pairwise(cs)]
        agl, climb_v, up, down = smooth_altitudes(prefs, floors, ceils, dists, limits)
        amsl = [a + _ground_at(p.x, p.y) for p, a in zip(cs, agl, strict=True)]
        return cs, agl, amsl, bands, climb_v, up, down

    _, _, amsl0, _, _, _, _ = _solve_altitudes(filleted)

    # --- Terrain clearance under the profile PX4 actually flies (SHORTFALLS 0) ---
    # PX4 front-loads a leg's descent instead of interpolating it along the ground
    # track, so a long leg with a big drop can reach its target altitude while
    # still over high ground. Split *only the legs that actually hit terrain*,
    # then re-solve altitudes: the inserted vertices are terrain-following for
    # free, because planner altitudes are AGL.
    # ⚠️ Splitting on a drop threshold instead of on the clearance check froze a
    # real SITL mission at 11/79 — see planner/terrain_profile.py.
    profile = FlownProfile.from_rates(limits.max_descent_rate_ms, limits.cruise_speed_ms)

    def _predicted_amsl(x: float, y: float) -> float:
        """AMSL an inserted vertex would get from the band + smoothing pass."""
        p = Coordinate(x=x, y=y)
        pref, floor, ceil, _band = _altitude_band(
            flyability, grid.world_to_pixel(p), mission.cruise_agl_m
        )
        return _ground_at(x, y) + min(max(pref, floor), ceil)

    densified, inserted = densify_unsafe_descents(
        filleted,
        amsl0,
        predicted_amsl=_predicted_amsl,
        ground_at=_ground_at,
        profile=profile,
        margin_agl_m=MIN_TERRAIN_CLEARANCE_M,
    )

    fpts, alts, amsl, bands, climb_violations, max_climb, max_descent = _solve_altitudes(densified)
    seg_dists = [a.distance_to(b) for a, b in pairwise(fpts)]
    clearance = assess_clearance(
        [(p.x, p.y) for p in fpts],
        amsl,
        _ground_at,
        profile,
        margin_agl_m=MIN_TERRAIN_CLEARANCE_M,
    )

    # Carry the terrain elevation under each vertex: AGL alone is not enough to
    # emit a correct .plan (the writer must convert to relative-to-home, and on
    # sloping ground those differ by the full relief).
    vertices = tuple(
        TrajectoryVertex(
            x=p.x,
            y=p.y,
            z=alt,
            altitude_agl_m=alt,
            ground_elev_m=float(surface.dtm[grid.world_to_pixel(p)]),
            flight_band=band,
            speed_hint_ms=speed,
        )
        for p, alt, band in zip(fpts, alts, bands, strict=True)
    )
    kinematics = KinematicReport(
        # Reversals are allowed (low payout speed mitigates snag), so they do not
        # break feasibility — turn-radius, climb-rate and terrain-clearance
        # violations do. Clearance is scored against the *flown* descent profile,
        # not linear interpolation (SHORTFALLS item 0).
        feasible=(turn_violations == 0 and climb_violations == 0 and clearance.clears),
        min_turn_radius_m=limits.min_turn_radius_m,
        tightest_turn_radius_m=tightest,
        turn_violations=turn_violations,
        reversals=reversals,
        max_climb_rate_ms=max_climb,
        max_descent_rate_ms=max_descent,
        climb_violations=climb_violations,
        min_clearance_agl_m=clearance.min_clearance_agl_m,
        clearance_violations=clearance.violations,
        descent_vertices_inserted=inserted,
    )
    trajectory_length = sum(seg_dists)

    seg_band = FlightBand.UPPER_TRUNK if flyability is not None else FlightBand.OPEN
    segments: list[RouteSegment] = []
    all_cells: list[tuple[int, int]] = []
    for i in range(len(cell_path) - 1):
        seg_cells = _segment_cells(cell_path[i], cell_path[i + 1])
        all_cells.extend(seg_cells if i == 0 else seg_cells[1:])
        seg_exposed = _mean_over(exposure, seg_cells) > EXPOSED_THRESHOLD
        segments.append(
            RouteSegment(
                index=i,
                start=coords[i],
                end=coords[i + 1],
                length_m=coords[i].distance_to(coords[i + 1]),
                flyability=_segment_flyability(flyability, seg_cells),
                flight_band=seg_band,
                exposed=seg_exposed,
            )
        )

    crossings: tuple[Crossing, ...] = ()
    if surface.road_mask is not None:
        crossings = detect_crossings(cell_path, surface.road_mask, grid)

    total_length = trajectory_length
    fiber_used = total_length * mission.fiber_slack_factor

    path_exposures = [float(exposure[r, c]) for r, c in all_cells]
    mean_exposure = float(np.mean(path_exposures)) if path_exposures else 0.0
    max_exposure = float(np.max(path_exposures)) if path_exposures else 0.0
    exposed_fraction = (
        float(np.mean([e > EXPOSED_THRESHOLD for e in path_exposures])) if path_exposures else 0.0
    )

    return Plan(
        mission_name=mission.name,
        crs=mission.crs,
        route_index=route_index,
        seed=mission.seed,
        vertices=tuple(vertices),
        segments=tuple(segments),
        crossings=crossings,
        total_length_m=total_length,
        fiber_used_m=fiber_used,
        objective_scores={
            "path_cost": path_cost(surface.cost, cell_path),
            "length_m": total_length,
            "mean_exposure": mean_exposure,
            "max_exposure": max_exposure,
            "exposed_fraction": exposed_fraction,
        },
        kinematics=kinematics,
    )


def plan_over_terrain(
    mission: Mission,
    dtm: np.ndarray,
    dsm: np.ndarray,
    grid: Grid,
    *,
    road_mask: np.ndarray | None = None,
    flyability: FlyabilityField | None = None,
    route_index: int = 0,
) -> tuple[Plan, CostSurface]:
    """Build the cost surface for a terrain and plan the corridor over it."""
    surface = build_cost_surface(
        dtm, dsm, grid, mission, road_mask=road_mask, flyability=flyability
    )
    plan = plan_corridor(mission, surface, route_index=route_index)
    return plan, surface


def straight_line_ground_truth(mission: Mission) -> list[Coordinate]:
    """The naive straight start->end line, for the viz to contrast against."""
    return [mission.start, mission.end]
