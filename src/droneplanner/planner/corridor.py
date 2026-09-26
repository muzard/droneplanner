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
from collections.abc import Callable, Sequence
from itertools import pairwise

import numpy as np

from droneplanner.crossings.detect import detect_crossings
from droneplanner.fields.altitude import (
    _band_from_canopy,
    altitude_band_at,
    canopy_height,
    concealment_mask,
    under_canopy_ceiling,
)
from droneplanner.fields.composite import (
    CostSurface,
    build_cost_surface,
    crown_medium_for,
    gap_map_for,
    trunk_space_for,
)
from droneplanner.fields.stems import (
    StemMap,
    assess_stem_clearance,
    required_clearance_m,
)
from droneplanner.fields.viewshed import point_exposure
from droneplanner.forest.navigability import CLASS_TO_ENUM, FlyabilityField
from droneplanner.laydown.settle import cable_settle_lengths
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import AltitudePolicy, Mission
from droneplanner.model.plan import (
    CanopyClimb,
    Crossing,
    FlightBand,
    FlyabilityClass,
    KinematicReport,
    Plan,
    RouteSegment,
    TrajectoryVertex,
    commanded_speed_ms,
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
    # Where the altitude policy may go UNDER the canopy: the concealment zone minus
    # the stands the cost surface decided to fly over (over_canopy_when_unpassable).
    # Using the raw zone here would plan under a stand priced as flown over.
    enterable = surface.components.get("enterable_zone")
    zone_for_bands = (
        (np.ones(chm.shape, dtype=bool) if covered is None else covered.copy())
        if enterable is None
        else enterable > 0.5
    )

    def _band_at(p: Coordinate) -> tuple[float, float, float, FlightBand]:
        return altitude_band_at(
            grid.world_to_pixel(p),
            chm=chm,
            target_agl_m=mission.cruise_agl_m,
            policy=mission.altitude,
            flyability=flyability,
            covered=zone_for_bands,
            skim=skim,
        )

    speed_cap_raster = surface.components["max_speed_ms"]
    skim = surface.components.get("skim_canopy")

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
        _debounce_bands(
            cs,
            dists,
            prefs,
            floors,
            ceils,
            bands,
            chm=chm,
            grid=grid,
            zone=zone_for_bands,
            target_agl_m=mission.cruise_agl_m,
            policy=mission.altitude,
            skim=skim,
        )
        # The speed each leg will be COMMANDED (the .plan's speeds): the slower end's
        # stem-density cap, as the emitter and the endurance budget use.
        caps = [
            commanded_speed_ms(
                float(min(mission.drone.cruise_speed_ms, speed_cap_raster[grid.world_to_pixel(p)]))
            )
            for p in cs
        ]
        legs = [min(a, b) for a, b in pairwise(caps)]
        agl, climb_v, up, down = smooth_altitudes(prefs, floors, ceils, dists, limits, legs)
        amsl = [a + _ground_at(p.x, p.y) for p, a in zip(cs, agl, strict=True)]
        return cs, agl, amsl, bands, climb_v, up, down

    # --- Every under/over transition at a canopy OPENING (2026-09-25) ---
    # The aircraft cannot climb up through closed crowns, nor drop back into the
    # trunk layer through them. Where a transition has no opening beside it, the
    # over-canopy stretch is extended back (or forward) under the canopy to the
    # nearest one; the cells passed over are then flown over too.
    opening = surface.components.get("canopy_opening")
    if opening is not None:
        _, _, _, bands0, _, _, _ = _solve_altitudes(spaced)
        zone_for_bands &= ~_extend_to_openings(
            spaced, bands0, opening, grid, covered, max_search_m=OPENING_SEARCH_M
        )

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
    # Charged at the speed the .plan will COMMAND (commanded_speed_ms), not the raw
    # cap: the emitter floors to a step, and a budget on the unfloored cap ran 22 %
    # short in SITL (Askola measured zone, 2026-09-25).
    vertex_speeds = [
        commanded_speed_ms(float(min(speed, speed_cap[grid.world_to_pixel(p)]))) for p in fpts
    ]
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
        trunk_space=trunk_space_for(
            mission, surface.dtm, chm, surface.components["stem_density_per_ha"]
        ),
        gap_map=gap_map_for(mission, grid),
        medium=crown_medium_for(
            mission, surface.dtm, surface.dsm, surface.components["stem_density_per_ha"], grid
        ),
    )
    mean_exposure_flown = _track_weighted_mean(flown_samples, flown_exposure)
    max_exposure_flown = float(np.max(flown_exposure)) if flown_exposure.size else 0.0

    # How much of the route is flown among stems denser than the onboard planner
    # has been shown to handle (fields/stem_density.ONBOARD_PLANNING_ENVELOPE).
    beyond = surface.components.get("beyond_envelope")
    measured = surface.components.get("stem_density_measured")
    band = surface.components.get("flight_band")
    beyond_m = 0.0
    measured_m = 0.0
    for a, b in pairwise(fpts):
        cell = grid.world_to_pixel(a)
        if beyond is not None and beyond[cell] > 0.0:
            beyond_m += a.distance_to(b)
        if measured is not None and band is not None and measured[cell] > 0.0 and band[cell] >= 2:
            measured_m += a.distance_to(b)

    climbs = _canopy_climbs(vertices, chm, grid, covered, surface, mission)
    stem_scores: dict[str, float] = {
        "below_canopy_beyond_envelope_m": beyond_m,
        "below_canopy_density_measured_m": measured_m,
        "canopy_climb_in_zone_m": sum(c.length_m for c in climbs),
        **_escape_envelope(vertices, chm, grid, surface.components.get("stem_density_per_ha")),
        "below_canopy_in_crown_layer_m": _crown_layer_length(
            vertices,
            chm,
            grid,
            mission.altitude.crown_ratio or mission.threat.canopy_occlusion.trunk_space_crown_ratio,
        ),
        **_unopened_transitions(vertices, surface.components.get("canopy_opening"), grid),
        **cable_settle_lengths(
            vertices, chm, grid, surface.components.get("canopy_opening"), covered
        ),
    }
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
        canopy_climbs=climbs,
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


def _canopy_climbs(
    vertices: Sequence[TrajectoryVertex],
    chm: np.ndarray,
    grid: Grid,
    covered: np.ndarray | None,
    surface: CostSurface,
    mission: Mission,
) -> tuple[CanopyClimb, ...]:
    """Contiguous stretches inside the concealment zone flown OVER the canopy.

    The operator is told about each before launch (report finding
    ``canopy_climb_in_zone``): the aircraft leaves cover there, and a climb in the
    first few hundred metres gives the launch site away. Before 2026-09-25 a
    stand too short to enter was skimmed silently, and a stand too dense to fly was
    a NO-GO; the second is now flown over (``over_canopy_when_unpassable``), and
    both are listed.
    """
    forced = surface.components.get("forced_over_canopy")
    policy = mission.altitude
    runs: list[CanopyClimb] = []
    along = 0.0
    start: float | None = None
    length = max_agl = max_chm = 0.0
    reasons: set[str] = set()

    def _close() -> None:
        nonlocal start
        if start is not None and length > 0.0:
            order = ("dense_stand", "crowns_too_low", "stand_too_short")
            reason = next(r for r in order if r in reasons)
            runs.append(
                CanopyClimb(
                    start_m=start,
                    length_m=length,
                    max_agl_m=max_agl,
                    max_canopy_m=max_chm,
                    reason=reason,
                )
            )
        start = None

    for i, v in enumerate(vertices):
        seg = 0.0 if i == 0 else math.dist((vertices[i - 1].x, vertices[i - 1].y), (v.x, v.y))
        cell = grid.world_to_pixel(Coordinate(x=v.x, y=v.y))
        in_zone = covered is None or bool(covered[cell])
        over = in_zone and v.flight_band is FlightBand.CANOPY_TOP
        if over:
            if start is None:
                start, length, max_agl, max_chm, reasons = along, 0.0, 0.0, 0.0, set()
            length += seg
            max_agl = max(max_agl, v.altitude_agl_m)
            h = float(chm[cell])
            max_chm = max(max_chm, h)
            if forced is not None and forced[cell] > 0.5:
                reasons.add("dense_stand")
            elif (
                policy.crown_ratio is not None
                and h >= policy.min_canopy_m
                and h - policy.canopy_clearance_m >= policy.min_agl_m
            ):
                reasons.add("crowns_too_low")
            else:
                reasons.add("stand_too_short")
        else:
            _close()
        along += seg
    _close()
    return tuple(runs)


ESCAPE_RADIUS_M = 30.0
"""Mirror of droneflight ``LocalOffboardConfig.escape_radius_m`` (30 m since
2026-09-26): trunk tops within it set the escape height."""

ESCAPE_MARGIN_M = 3.0
"""Mirror of droneflight ``LocalOffboardConfig.escape_margin_m``: an in-flight
escape climbs to the local canopy top plus this."""


ESCAPE_LIKELY_ABOVE_PER_HA = 1000.0
"""Past this stand density an escape stops being hypothetical: GPS-denied SITL flew
~1000/ha with 0 escapes in 4 seed sets, and 1200 and 1500/ha with 1 in 4 and 1 in 3
(droneflight, 2026-09-26) — the local planner met gaps it could not thread."""


def _escape_envelope(
    vertices: Sequence[TrajectoryVertex],
    chm: np.ndarray,
    grid: Grid,
    density_per_ha: np.ndarray | None = None,
) -> dict[str, float]:
    """How high an in-flight ESCAPE CLIMB could take the aircraft on this route.

    The onboard stack climbs straight up over the trunks when it is blocked under
    the canopy (droneflight ``fly-local --escape-after``, default 20 s). That is a
    decision made in flight, but its worst case is knowable before launch: the
    tallest canopy within a cell of any below-canopy vertex, plus the margin. The
    report states it, so an escape is never a surprise (user, 2026-09-25)."""
    from scipy.ndimage import maximum_filter

    under = [
        i
        for i, v in enumerate(vertices)
        if v.flight_band in (FlightBand.BELOW_CANOPY, FlightBand.UPPER_TRUNK)
    ]
    if not under:
        return {}
    radius = max(1, math.ceil(ESCAPE_RADIUS_M / max(grid.cell_size_m, 1e-6)))
    local_top = maximum_filter(chm, size=2 * radius + 1)
    along = [0.0]
    for a, b in pairwise(vertices):
        along.append(along[-1] + math.dist((a.x, a.y), (b.x, b.y)))
    worst = max(
        float(local_top[grid.world_to_pixel(Coordinate(x=vertices[i].x, y=vertices[i].y))])
        for i in under
    )
    likely = 0.0
    if density_per_ha is not None:
        for i in under:
            if i + 1 >= len(along):
                continue
            cell = grid.world_to_pixel(Coordinate(x=vertices[i].x, y=vertices[i].y))
            if float(density_per_ha[cell]) > ESCAPE_LIKELY_ABOVE_PER_HA:
                likely += along[i + 1] - along[i]
    return {
        "escape_max_agl_m": worst + ESCAPE_MARGIN_M,
        "escape_first_m": along[under[0]],
        "escape_exposed_length_m": sum(
            along[i + 1] - along[i] for i in under if i + 1 < len(along)
        ),
        "escape_likely_length_m": likely,
    }


def _crown_layer_length(
    vertices: Sequence[TrajectoryVertex], chm: np.ndarray, grid: Grid, crown_ratio: float
) -> float:
    """Route length flown under the canopy but ABOVE the estimated crown base
    (canopy x (1 - crown_ratio)) — i.e. inside live crowns (2026-09-26)."""
    total = 0.0
    for a, b in pairwise(vertices):
        if a.flight_band not in (FlightBand.BELOW_CANOPY, FlightBand.UPPER_TRUNK):
            continue
        canopy = float(chm[grid.world_to_pixel(Coordinate(x=a.x, y=a.y))])
        if a.altitude_agl_m > canopy * (1.0 - crown_ratio) + 1e-6:
            total += math.dist((a.x, a.y), (b.x, b.y))
    return total


OPENING_SEARCH_M = 200.0
"""How far along the route an under/over transition may move to reach an opening."""

_UNDER = (FlightBand.BELOW_CANOPY, FlightBand.UPPER_TRUNK)


def _near_opening(opening: np.ndarray, cell: tuple[int, int]) -> bool:
    r, c = cell
    win = opening[max(r - 1, 0) : r + 2, max(c - 1, 0) : c + 2]
    return bool((win > 0.5).any())


def _extend_to_openings(
    pts: list[tuple[float, float]],
    bands: list[FlightBand],
    opening: np.ndarray,
    grid: Grid,
    covered: np.ndarray | None,
    *,
    max_search_m: float,
) -> np.ndarray:
    """Cells to fly OVER so every under<->over-canopy transition sits at an opening.

    A transition is a pair of consecutive vertices, one under the canopy and one
    not. It needs an opening within a cell of either vertex (open ground counts).
    If there is none, walk away from the transition through the UNDER side
    until a vertex near an opening is found (within ``max_search_m``) and return
    the cells in between, to be flown over as well; if none is found the
    transition is left where it is and the report says it has no opening.
    """
    cells = [grid.world_to_pixel(Coordinate(x=x, y=y)) for x, y in pts]
    force = np.zeros(opening.shape, dtype=bool)
    n = len(pts)

    def seg(i: int, j: int) -> float:
        return math.dist(pts[i], pts[j])

    for i in range(n - 1):
        a, b = bands[i], bands[i + 1]
        # Leaving the trunk layer (to skim a stand, or over a young stand flown at the
        # open target) crosses the crown layer at the edge; open ground is its own
        # opening, which the opening test sees.
        climb = a in _UNDER and b not in _UNDER
        descend = a not in _UNDER and b in _UNDER
        if not (climb or descend):
            continue
        if _near_opening(opening, cells[i]) or _near_opening(opening, cells[i + 1]):
            continue
        # Walk into the under side, away from the transition.
        k, step, walked = (i, -1, 0.0) if climb else (i + 1, 1, 0.0)
        path: list[int] = []
        while 0 <= k < n and bands[k] in _UNDER and walked <= max_search_m:
            if _near_opening(opening, cells[k]):
                for m in path:
                    force[cells[m]] = True
                break
            path.append(k)
            nxt = k + step
            if 0 <= nxt < n:
                walked += seg(k, nxt)
            k = nxt
    if covered is not None:
        force &= covered
    return force


def _unopened_transitions(
    vertices: Sequence[TrajectoryVertex], opening: np.ndarray | None, grid: Grid
) -> dict[str, float]:
    """Under/over-canopy transitions left WITHOUT an opening (none within reach).

    Scores: how many, and where the first one is along the route (for the
    report's ``canopy_transition_no_opening`` finding)."""
    if opening is None:
        return {}
    along = 0.0
    count = 0
    first = -1.0
    for a, b in pairwise(vertices):
        seg = math.dist((a.x, a.y), (b.x, b.y))
        if (a.flight_band in _UNDER) != (b.flight_band in _UNDER):
            ca = grid.world_to_pixel(Coordinate(x=a.x, y=a.y))
            cb = grid.world_to_pixel(Coordinate(x=b.x, y=b.y))
            if not (_near_opening(opening, ca) or _near_opening(opening, cb)):
                count += 1
                if first < 0:
                    first = along
        along += seg
    out = {"canopy_transitions_without_opening": float(count)}
    if count:
        out["canopy_transition_no_opening_first_m"] = first
    return out


MIN_DWELL_M = 50.0
"""Shortest stretch flown under, or out of, the canopy between two transitions."""

ENTRY_HYSTERESIS_M = 2.0
"""A stand this much below ``AltitudePolicy.min_canopy_m`` may still be flown under
to bridge a short excursion — if it physically has room under the crowns."""


def _debounce_bands(
    cs: Sequence[Coordinate],
    dists: Sequence[float],
    prefs: list[float],
    floors: list[float],
    ceils: list[float],
    bands: list[FlightBand],
    *,
    chm: np.ndarray,
    grid: Grid,
    zone: np.ndarray,
    target_agl_m: float,
    policy: AltitudePolicy,
    skim: np.ndarray | None = None,
) -> None:
    """Remove under/over excursions shorter than ``MIN_DWELL_M``, in place.

    🔴 SHORTFALLS M1 / the strict-xfail test: ``min_canopy_m`` is a hard per-cell
    threshold, so along a canopy gradient drifting across it by centimetres the
    band flipped every waypoint — 14.21 / 9.05 / 14.19 / 9.03 m AGL, 16.9 m/s of
    climb on a 2.46 m leg against a 3.0 m/s airframe. Two passes:

    1. a short run OUT of the canopy between two under-canopy runs is flown under
       too, where every cell is in the enterable zone, within
       ``ENTRY_HYSTERESIS_M`` of the threshold and has room under its crowns;
    2. a short run UNDER the canopy between two out-of-canopy runs is flown over.

    Lidar (``UPPER_TRUNK``) vertices are measured windows and are never touched.
    """
    n = len(bands)
    under = (FlightBand.BELOW_CANOPY,)

    def runs(pred: Callable[[int], bool]) -> list[tuple[int, int]]:
        out, i = [], 0
        while i < n:
            if pred(i):
                j = i
                while j + 1 < n and pred(j + 1):
                    j += 1
                out.append((i, j))
                i = j + 1
            else:
                i += 1
        return out

    def length(i: int, j: int) -> float:
        # Include the legs into and out of the run: that is the distance between
        # the transitions the run creates.
        lo, hi = max(i - 1, 0), min(j, n - 2)
        return float(sum(dists[lo : hi + 1]))

    def set_band(k: int, under_it: bool) -> None:
        h = float(chm[grid.world_to_pixel(cs[k])])
        pref, floor, ceil, band = _band_from_canopy(
            max(h, policy.min_canopy_m) if under_it else h,
            target_agl_m,
            policy,
            covered=under_it,
            skim_m=None if skim is None else float(skim[grid.world_to_pixel(cs[k])]),
        )
        if under_it:
            ceil = float(under_canopy_ceiling(np.asarray(h), policy))
            pref = min(max(target_agl_m, floor), ceil)
        prefs[k], floors[k], ceils[k], bands[k] = pref, floor, ceil, band

    # 1. bridge short excursions out of the canopy
    for i, j in runs(lambda k: bands[k] not in under and bands[k] is not FlightBand.UPPER_TRUNK):
        if i == 0 or j == n - 1 or bands[i - 1] not in under or bands[j + 1] not in under:
            continue
        if length(i, j) >= MIN_DWELL_M:
            continue
        ok = True
        for k in range(i, j + 1):
            cell = grid.world_to_pixel(cs[k])
            h = float(chm[cell])
            if not bool(zone[cell]) or h < policy.min_canopy_m - ENTRY_HYSTERESIS_M:
                ok = False
                break
            if float(under_canopy_ceiling(np.asarray(h), policy)) < policy.min_agl_m:
                ok = False
                break
        if ok:
            for k in range(i, j + 1):
                set_band(k, under_it=True)

    # 2. drop short islands under the canopy
    for i, j in runs(lambda k: bands[k] in under):
        if i == 0 or j == n - 1 or bands[i - 1] in under or bands[j + 1] in under:
            continue
        if length(i, j) >= MIN_DWELL_M:
            continue
        for k in range(i, j + 1):
            set_band(k, under_it=False)
