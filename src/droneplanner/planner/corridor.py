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

import math
from itertools import pairwise

import numpy as np

from droneplanner.crossings.detect import detect_crossings
from droneplanner.fields.altitude import altitude_band_at, canopy_height, concealment_mask
from droneplanner.fields.composite import CostSurface, build_cost_surface
from droneplanner.fields.stems import (
    StemMap,
    assess_stem_clearance,
    required_clearance_m,
)
from droneplanner.fields.viewshed import point_exposure
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
from droneplanner.planner.corridor_width import assess_corridor
from droneplanner.planner.endurance import route_budget
from droneplanner.planner.kinematics import (
    KinematicLimits,
    amsl_vertical_rates,
    fillet_path,
    smooth_altitudes,
)
from droneplanner.planner.stem_route import refine_through_stems
from droneplanner.planner.terrain_profile import (
    FlownProfile,
    assess_clearance,
    densify_unsafe_descents,
    sample_flown_profile,
)
from droneplanner.planner.theta_star import path_cost, plan_path
from droneplanner.raster_lines import bresenham

# A path cell counts as "exposed" when this fraction of observers can see it.
EXPOSED_THRESHOLD = 0.5

# Terrain clearance floor (m AGL) the flown descent profile must keep, used when a
# mission carries no AltitudePolicy. Below this a leg is a CFIT finding, not a
# warning — see planner/terrain_profile.py. The policy's ``min_agl_m`` is the same
# quantity seen from the other side (how low the drone may be *asked* to fly), so
# the two are one number: a route may not be planned lower than it is allowed to
# be flown.
MIN_TERRAIN_CLEARANCE_M = 5.0

# Samples per leg for re-scoring exposure along the flown 3D trajectory. Coarser
# than the clearance check's 32 on purpose: line-of-sight per sample costs a ray
# march per observer, and unlike a CFIT spike (a metres-wide terrain feature) an
# exposure average does not swing on sub-100 m detail.
EXPOSURE_SAMPLES_PER_LEG = 4


def _segment_cells(a: tuple[int, int], b: tuple[int, int]) -> list[tuple[int, int]]:
    return bresenham(a[0], a[1], b[0], b[1])


def _mean_over(field: np.ndarray, cells: list[tuple[int, int]]) -> float:
    if not cells:
        return 0.0
    return float(np.mean([field[r, c] for r, c in cells]))


def _subdivide(pts: list[tuple[float, float]], spacing_m: float) -> list[tuple[float, float]]:
    """Split legs longer than ``spacing_m`` so the commanded altitude can track terrain.

    **Subdivide only — never merge.** PX4 drives toward the next waypoint's
    altitude at its own vertical rate, so between distant waypoints the commanded
    profile is a straight line the ground is free to rise through: on the demo AO
    a 10 m AGL route flies **12 m underground** at 100 m spacing and clears by
    6.7 m at 25 m. Existing vertices are all kept, because they carry the fillet
    arcs that make the turn radius flyable — resampling *through* an arc would
    cut the corner it exists to round.

    The residual clearance loss after subdividing is terrain bulging between
    samples, so the useful floor on ``spacing_m`` is the DTM's cell size: finer
    than the data cannot buy accuracy it does not have.
    """
    if spacing_m <= 0.0 or len(pts) < 2:
        return list(pts)
    out: list[tuple[float, float]] = [pts[0]]
    for a, b in pairwise(pts):
        steps = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / spacing_m))
        for k in range(1, steps + 1):
            f = k / steps
            out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
    return out


def _track_weighted_mean(samples: list[tuple[float, float, float]], values: np.ndarray) -> float:
    """Mean of per-sample values weighted by the track length each sample covers.

    A plain mean would be wrong here: the sampler puts a fixed number of samples
    on every leg, and after filleting a route is mostly *short* arc legs, so
    corners would carry the metric. Weight each sample by half the distance to
    each neighbour (trapezoidal) and it becomes an average over the route.
    """
    n = len(samples)
    if n == 0:
        return 0.0
    if n == 1:
        return float(values[0])
    xy = np.asarray([(x, y) for x, y, _z in samples], dtype=float)
    seg = np.hypot(*(xy[1:] - xy[:-1]).T)
    weights = np.zeros(n, dtype=float)
    weights[:-1] += seg / 2.0
    weights[1:] += seg / 2.0
    if weights.sum() <= 0.0:  # a zero-length route
        return float(np.mean(values))
    return float(np.dot(weights, values) / weights.sum())


def _segment_flyability(
    flyability: FlyabilityField | None, cells: list[tuple[int, int]]
) -> FlyabilityClass:
    """Worst (highest-code) flyability class along a segment's cells."""
    if flyability is None:
        return FlyabilityClass.UNKNOWN
    worst = max(int(flyability.navigable_class[r, c]) for r, c in cells)
    return CLASS_TO_ENUM[worst]


def plan_corridor(
    mission: Mission,
    surface: CostSurface,
    route_index: int = 0,
    stems: StemMap | None = None,
) -> Plan:
    """Plan the concealment-aware corridor route over a cost surface."""
    grid = surface.grid
    start_rc = grid.world_to_pixel(mission.start)
    goal_rc = grid.world_to_pixel(mission.end)

    cell_path = plan_path(surface.cost, start_rc, goal_rc, obstacle=surface.obstacle)
    # Two objects, two fields (SHORTFALLS item 1): the drone is scored at its
    # flight altitude with a detection model, the cable on the ground it is laid
    # on with plain line of sight — see fields/composite.py for why they differ.
    exposure = surface.components["exposure"]
    ground_los = surface.components["ground_los"]
    flyability = surface.flyability
    speed = mission.drone.cruise_speed_ms
    clearance_floor = mission.altitude.min_agl_m

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

    # Terrain-following spacing: the route is flown low, so the commanded altitude
    # profile has to track the ground rather than span it. See ``_subdivide``.
    spaced = _subdivide(filleted, mission.waypoint_spacing_m)

    # --- Route BETWEEN the stems, if a stem map was supplied (TEST-PLAN N4) -----
    # The coarse search runs on 25 m cells and cannot see a 2.4 m gap, so its
    # centreline is laid with no knowledge of where the trunks are — measured
    # against the dense manifest it passes THROUGH one (min clearance -0.033 m,
    # 37 samples inside the airframe's requirement, against the 37 strikes B2
    # actually flew). This re-lays the line at 0.5 m inside the lateral freedom
    # the corridor already grants. It runs BEFORE the altitude solve so clearance,
    # kinematics and endurance all score the line that will really be flown.
    stem_report = None
    if stems is not None and len(stems) > 0:
        spaced, stem_report = refine_through_stems(
            spaced,
            stems,
            required_m=required_clearance_m(
                mission.drone.rotor_span_m, mission.stem.clearance_margin_m
            ),
            min_half_width_m=mission.stem.min_half_width_m,
            max_half_width_m=mission.stem.max_half_width_m,
        )

    def _ground_at(x: float, y: float) -> float:
        return float(surface.dtm[grid.world_to_pixel(Coordinate(x=x, y=y))])

    chm = canopy_height(surface.dtm, surface.dsm)
    covered = concealment_mask(grid, mission.start, mission.covered_radius_m)

    def _band_at(p: Coordinate) -> tuple[float, float, float, FlightBand]:
        return altitude_band_at(
            grid.world_to_pixel(p),
            chm=chm,
            target_agl_m=mission.cruise_agl_m,
            policy=mission.altitude,
            flyability=flyability,
            covered=covered,
        )

    def _solve_altitudes(
        pts: list[tuple[float, float]],
    ) -> tuple[list[Coordinate], list[float], list[float], list[FlightBand], int, float, float]:
        """Band + climb-smooth a point list into per-vertex AGL and AMSL altitudes."""
        cs = [Coordinate(x=x, y=y) for x, y in pts]
        prefs, floors, ceils, bands = [], [], [], []
        for p in cs:
            pref, floor, ceil, band = _band_at(p)
            prefs.append(pref)
            floors.append(floor)
            ceils.append(ceil)
            bands.append(band)
        dists = [a.distance_to(b) for a, b in pairwise(cs)]
        agl, climb_v, up, down = smooth_altitudes(prefs, floors, ceils, dists, limits)
        amsl = [a + _ground_at(p.x, p.y) for p, a in zip(cs, agl, strict=True)]
        return cs, agl, amsl, bands, climb_v, up, down

    _, _, amsl0, _, _, _, _ = _solve_altitudes(spaced)

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
        pref, floor, ceil, _band = _band_at(Coordinate(x=x, y=y))
        return _ground_at(x, y) + min(max(pref, floor), ceil)

    densified, inserted = densify_unsafe_descents(
        spaced,
        amsl0,
        predicted_amsl=_predicted_amsl,
        ground_at=_ground_at,
        profile=profile,
        margin_agl_m=clearance_floor,
    )

    fpts, alts, amsl, bands, climb_violations, max_climb, max_descent = _solve_altitudes(densified)
    seg_dists = [a.distance_to(b) for a, b in pairwise(fpts)]
    clearance = assess_clearance(
        [(p.x, p.y) for p in fpts],
        amsl,
        _ground_at,
        profile,
        margin_agl_m=clearance_floor,
    )

    # --- Does the mission close on endurance? (TEST-PLAN N1) ---
    # Scored on the FINAL vertices, after densification and altitude smoothing,
    # and in AMSL so that terrain-driven climbing is charged for — the kinematic
    # climb check runs in AGL and cannot see it (SHORTFALLS 0b).
    # Per-vertex speed cap from the local stem spacing (TEST-PLAN N3). The
    # aircraft cannot hold cruise through a stand whose gaps are tighter than its
    # reaction distance, so the budget is scored at the speed each leg can
    # actually be flown — not at a cruise figure the forest forbids.
    # Since 2026-09-23 the cap is the ONBOARD-PLANNING envelope, measured in the
    # droneflight harness, not the reactive-coherence limit: reactive avoidance
    # is dead in flight (N2), so pricing the mission on it priced a flight stack
    # that will not fly. `max_reactive_speed_ms` stays in the components for
    # comparison only.
    speed_cap = surface.components["max_speed_ms"]
    vertex_speeds = [float(min(speed, speed_cap[grid.world_to_pixel(p)])) for p in fpts]
    endurance = route_budget(
        [(p.x, p.y) for p in fpts], amsl, mission, speed_limits_ms=vertex_speeds
    )

    # --- Lateral freedom the plan grants, and what it costs (TEST-PLAN A1) ---
    # The flight stack follows this route approximately, so concealment scored on
    # the centreline describes a line it will not fly. The corridor's width is
    # whatever the local cover allows — it stops at the edge of a stand.
    corridor = assess_corridor(
        surface,
        list(fpts),
        max_half_width_m=mission.corridor_half_width_max_m,
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
            speed_hint_ms=max(vertex_speed, 0.1),
            corridor_half_width_m=half_width,
        )
        for p, alt, band, half_width, vertex_speed in zip(
            fpts, alts, bands, corridor.half_width_m, vertex_speeds, strict=True
        )
    )
    # The smoother constrains the *AGL* profile, but the airframe's climb limit
    # applies to true vertical motion. Over relief the two differ — a constant-AGL
    # route reports 0.00 m/s while genuinely climbing — so measure both and let
    # the report say when the flown rate exceeds the limit (SHORTFALLS 0b).
    max_climb_amsl, max_descent_amsl = amsl_vertical_rates(
        [v.ground_elev_m for v in vertices],
        [v.altitude_agl_m for v in vertices],
        seg_dists,
        limits.cruise_speed_ms,
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
        max_climb_rate_amsl_ms=max_climb_amsl,
        max_descent_rate_amsl_ms=max_descent_amsl,
        climb_violations=climb_violations,
        min_clearance_agl_m=clearance.min_clearance_agl_m,
        clearance_violations=clearance.violations,
        descent_vertices_inserted=inserted,
    )
    trajectory_length = sum(seg_dists)

    # Per-segment band, taken as the *least* concealed cell it crosses: a segment
    # that dips out of the stand for one cell is not a below-canopy segment.
    band_codes = surface.components["flight_band"]
    code_to_band = {
        0: FlightBand.OPEN,
        1: FlightBand.CANOPY_TOP,
        2: FlightBand.BELOW_CANOPY,
        3: FlightBand.UPPER_TRUNK,
    }

    def _segment_band(cells: list[tuple[int, int]]) -> FlightBand:
        return code_to_band[int(min(band_codes[r, c] for r, c in cells))]

    segments: list[RouteSegment] = []
    all_cells: list[tuple[int, int]] = []
    for i in range(len(cell_path) - 1):
        seg_cells = _segment_cells(cell_path[i], cell_path[i + 1])
        all_cells.extend(seg_cells if i == 0 else seg_cells[1:])
        segments.append(
            RouteSegment(
                index=i,
                start=coords[i],
                end=coords[i + 1],
                length_m=coords[i].distance_to(coords[i + 1]),
                flyability=_segment_flyability(flyability, seg_cells),
                flight_band=_segment_band(seg_cells),
                exposed=_mean_over(ground_los, seg_cells) > EXPOSED_THRESHOLD,
                drone_exposed=_mean_over(exposure, seg_cells) > EXPOSED_THRESHOLD,
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
    cable_los = [float(ground_los[r, c]) for r, c in all_cells]
    mean_cable_los = float(np.mean(cable_los)) if cable_los else 0.0

    # Re-score exposure along the trajectory the aircraft will actually fly. The
    # raster above assumes every cell is overflown at its *band* altitude; the
    # route departs from that twice over — the climb-limited smoother may not
    # reach the band, and PX4 front-loads a leg's descent (terrain_profile.py),
    # so on a descending leg it is higher than any interpolation says. This is
    # the same "re-score the final answer against what is actually flown" check
    # that terrain clearance uses, applied to concealment.
    flown_samples = sample_flown_profile(
        [(p.x, p.y) for p in fpts], amsl, profile, samples_per_leg=EXPOSURE_SAMPLES_PER_LEG
    )
    flown_exposure = point_exposure(
        surface.dsm,
        grid,
        list(mission.threat.observers),
        flown_samples,
        observer_height_m=mission.threat.observer_height_m,
        canopy=mission.threat.canopy_occlusion,
        detection=mission.threat.detection,
        target_span_m=mission.drone.rotor_span_m,
        sectors=mission.threat.sectors,
        # Must match build_cost_surface, or the flown re-score grades the route
        # against a different adversary than the one it was planned for.
        off_axis_vigilance=mission.threat.off_axis_vigilance,
    )
    mean_exposure_flown = _track_weighted_mean(flown_samples, flown_exposure)
    max_exposure_flown = float(np.max(flown_exposure)) if flown_exposure.size else 0.0

    # How much of the route is flown among stems denser than the onboard planner
    # has been shown to handle (fields/stem_density.ONBOARD_PLANNING_ENVELOPE).
    beyond = surface.components.get("beyond_envelope")
    beyond_m = 0.0
    if beyond is not None:
        for a, b in pairwise(fpts):
            if beyond[grid.world_to_pixel(a)] > 0.0:
                beyond_m += a.distance_to(b)

    stem_scores: dict[str, float] = {"below_canopy_beyond_envelope_m": beyond_m}
    if stems is not None and len(stems) > 0:
        sc = assess_stem_clearance(
            [(p.x, p.y) for p in fpts],
            stems,
            required_m=required_clearance_m(
                mission.drone.rotor_span_m, mission.stem.clearance_margin_m
            ),
        )
        stem_scores |= {
            "stem_min_clearance_m": sc.min_clearance_m,
            "stem_p05_clearance_m": sc.p05_clearance_m,
            "stem_sub_clearance_samples": float(sc.strikes),
            "stem_samples": float(sc.samples),
            "stem_mapped_fraction": sc.mapped_fraction,
            "stem_route_length_ratio": stem_report.length_ratio if stem_report else 1.0,
            "stem_windows_failed": float(stem_report.windows_failed if stem_report else 0),
            "stem_vertices_snapped": float(stem_report.vertices_snapped if stem_report else 0),
        }

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
            # Drone exposure at planned flight altitude, over the corridor cells.
            "mean_exposure": mean_exposure,
            "max_exposure": max_exposure,
            "exposed_fraction": exposed_fraction,
            # The same question asked of the flown 3D trajectory, and of the
            # cable on the ground beneath it. Divergence between these three is
            # informative, not noise — see the comment above.
            "mean_exposure_flown": mean_exposure_flown,
            "max_exposure_flown": max_exposure_flown,
            "mean_cable_los": mean_cable_los,
            # And of the CORRIDOR — the area the flight stack may actually use.
            # This is the honest number if the route is followed approximately.
            "mean_exposure_corridor": corridor.mean_exposure,
            "max_exposure_corridor": corridor.max_exposure,
            "corridor_half_width_min_m": corridor.min_half_width_m,
            "corridor_half_width_median_m": corridor.median_half_width_m,
            # Stem routing (N4). Scored on the FINAL line, after altitude solving
            # and CFIT densification, so it grades what is emitted — not what the
            # refinement pass happened to hand back.
            **stem_scores,
        },
        kinematics=kinematics,
        endurance=endurance,
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
    stems: StemMap | None = None,
) -> tuple[Plan, CostSurface]:
    """Build the cost surface for a terrain and plan the corridor over it."""
    surface = build_cost_surface(
        dtm, dsm, grid, mission, road_mask=road_mask, flyability=flyability
    )
    plan = plan_corridor(mission, surface, route_index=route_index, stems=stems)
    return plan, surface


def straight_line_ground_truth(mission: Mission) -> list[Coordinate]:
    """The naive straight start->end line, for the viz to contrast against."""
    return [mission.start, mission.end]
