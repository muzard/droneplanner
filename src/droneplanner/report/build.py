"""Risk/feasibility report builder (Software-Spec §5.4, §5.5).

Turns a ``Mission`` + ``Plan`` into a ``RiskReport`` with a GO / REVIEW / NO-GO
recommendation. The recommendation is deterministic and conservative:

* **NO-GO** if the plan is infeasible on a hard constraint — the route needs
  more fiber than the spool holds, or a segment is classified NON_NAVIGABLE.
* **REVIEW** if the plan is feasible but carries reviewable risk — marginal
  forest legs, visually exposed cable, crossings that need aerial manipulation,
  or (as in the M0 stub) concealment/flyability analysis that was not performed.
* **GO** only if the plan is feasible and no reviewable risk remains.

Analysis coverage gates the recommendation: the M0 stub runs no analysis (caps
at REVIEW), M1 adds concealment/defilade but no forest flyability (still caps at
REVIEW until the M2 forest model), and only a fully-analyzed, low-risk plan can
reach GO. This keeps the human in the loop.
"""

from __future__ import annotations

import itertools
import math
from datetime import datetime

from droneplanner.fields.sectors import describe_threat_sectors
from droneplanner.fields.stem_density import ONBOARD_PLANNING_ENVELOPE
from droneplanner.model.mission import Mission
from droneplanner.model.plan import CrossingMethod, FlightBand, FlyabilityClass, Plan
from droneplanner.model.report import (
    CrossingReport,
    Finding,
    Recommendation,
    RiskReport,
    SegmentReport,
    Severity,
)
from droneplanner.planner.corridor import OPENING_SEARCH_M


def _decide(findings: list[Finding]) -> Recommendation:
    severities = {f.severity for f in findings}
    if Severity.CRITICAL in severities:
        return Recommendation.NO_GO
    if Severity.WARNING in severities:
        return Recommendation.REVIEW
    return Recommendation.GO


ESCAPE_AFTER_S = 20.0
ESCAPE_HOLD_M = 50.0
"""Mirrors of droneflight ``LocalOffboardConfig.escape_after_s`` / ``escape_hold_m``
— the report describes the onboard policy the plan will be flown under."""

LAUNCH_CLIMB_M = 200.0
"""A canopy climb starting this close to launch is called out first in the report:
the aircraft would rise to canopy height right after take-off, over its own
launch site (user, 2026-09-25: "so no surprises happen")."""


def build_report(
    mission: Mission,
    plan: Plan,
    *,
    generated_at: datetime | None = None,
    concealment_analyzed: bool = False,
    flyability_analyzed: bool = False,
) -> RiskReport:
    """Build the risk report for a plan.

    ``concealment_analyzed`` is True when the M1 viewshed/terrain fields drove
    the route; ``flyability_analyzed`` is True when the M2 forest model ran.
    ``generated_at`` is injected (the CLI passes the wall clock); left ``None``
    for deterministic/test runs so output is reproducible.
    """
    findings: list[Finding] = []

    # --- Hard feasibility: fiber budget ---
    if plan.fiber_used_m > mission.spool.length_m:
        findings.append(
            Finding(
                severity=Severity.CRITICAL,
                code="fiber_insufficient",
                message=(
                    f"route needs {plan.fiber_used_m:.0f} m of fiber but the spool holds "
                    f"{mission.spool.length_m:.0f} m"
                ),
            )
        )

    # --- Forest navigability ---
    non_navigable = [s for s in plan.segments if s.flyability is FlyabilityClass.NON_NAVIGABLE]
    marginal = [s for s in plan.segments if s.flyability is FlyabilityClass.MARGINAL]
    if non_navigable:
        findings.append(
            Finding(
                severity=Severity.CRITICAL,
                code="non_navigable_segments",
                message=f"{len(non_navigable)} segment(s) classified non-navigable",
            )
        )
    if marginal:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="marginal_segments",
                message=f"{len(marginal)} segment(s) classified marginal — review flyability",
            )
        )

    # --- Exposure: the cable on the ground, and the aircraft laying it ---
    # These are separate findings because they are separate objects at separate
    # heights (SHORTFALLS item 1). A route can lay cable in perfect defilade
    # while flying the aircraft skylined the whole way — before 2026-07-27 only
    # the ground was ever scored, so that plan looked concealed.
    exposed_segments = [s for s in plan.segments if s.exposed]
    exposed_length = sum(s.length_m for s in exposed_segments)
    if exposed_segments:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="exposed_cable",
                message=f"{exposed_length:.0f} m of cable is visually/fire exposed",
            )
        )

    # --- Below-canopy flight: concealment bought with an unverified assumption ---
    # The planner sees a canopy *top* (DSM - DTM). It cannot see trunks, branches
    # or understory, so a leg flown under the canopy is safe only if the aircraft
    # clears obstacles itself. That capability lives in the flight stack, not here,
    # and saying so is the difference between a concealed plan and a wishful one.
    below_canopy = [
        s
        for s in plan.segments
        if s.flight_band in (FlightBand.BELOW_CANOPY, FlightBand.UPPER_TRUNK)
    ]
    # Length from the VERTICES — where the aircraft actually flies. Segment bands
    # take a segment's least concealed cell, so once stands are flown over
    # (over_canopy_when_unpassable, padded by a cell) a segment merely touching one
    # read as "over" and Askola's 400 m plan reported 0 m under the canopy with 19
    # vertices under it (2026-09-25).
    under_bands = (FlightBand.BELOW_CANOPY, FlightBand.UPPER_TRUNK)
    below_canopy_length = sum(
        math.dist((a.x, a.y), (b.x, b.y))
        for a, b in itertools.pairwise(plan.vertices)
        if a.flight_band in under_bands and b.flight_band in under_bands
    )
    under_vertices = [v for v in plan.vertices if v.flight_band in under_bands]
    if below_canopy_length > 0.0 or below_canopy:
        lidar_verified = all(v.flight_band is FlightBand.UPPER_TRUNK for v in under_vertices)
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="below_canopy_flight",
                message=(
                    f"{below_canopy_length:.0f} m of the route is flown below the canopy"
                    + (
                        " inside a lidar-measured free window; obstacle clearance at flight "
                        "time still depends on onboard perception"
                        if lidar_verified
                        else " on canopy-top (DSM) evidence alone — trunks, branches and "
                        "understory are unmodelled. The route is a corridor to follow "
                        "approximately; clearing obstacles is the flight stack's job"
                    )
                ),
            )
        )

    drone_exposed_segments = [s for s in plan.segments if s.drone_exposed]
    drone_exposed_length = sum(s.length_m for s in drone_exposed_segments)
    if drone_exposed_segments:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="exposed_drone",
                message=(
                    f"{drone_exposed_length:.0f} m of the route is flown in view of threat "
                    "observers at the planned flight altitude — the aircraft is detectable "
                    "there even where the cable beneath it is concealed"
                ),
            )
        )

    # --- Threat sectors: what the plan assumed about where the enemy is looking ---
    # A sector is an intelligence assumption that *removes* cost from the surface,
    # so it has to be visible in the artifact that a human signs off. And a mission
    # that supplies one only for the stub path to discard it must not look like a
    # mission that supplied none — that is the failure mode C3 exists to close.
    threat_sectors = describe_threat_sectors(mission.threat.observers, mission.threat.sectors)
    if threat_sectors and not concealment_analyzed:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="threat_sectors_ignored",
                message=(
                    f"{len(threat_sectors)} threat sector(s) were supplied but no "
                    "concealment analysis ran, so they shaped nothing — this route is "
                    "not the one the stated threat posture implies"
                ),
            )
        )
    elif threat_sectors:
        findings.append(
            Finding(
                severity=Severity.INFO,
                code="threat_sectors_applied",
                message=(
                    f"{len(threat_sectors)} threat sector(s) narrowed the threat: "
                    + "; ".join(threat_sectors)
                    + ". Concealment here rests on the adversary facing where the "
                    "mission says they are facing"
                ),
            )
        )

    # --- Lateral freedom: where the route can be followed loosely, and where not ---
    # The flight stack follows the route approximately. Where the corridor pinches
    # to nothing the plan is relying on precise tracking it has no way to command,
    # so the operator should know before the aircraft finds out.
    # A vertex's 0.0 means "no corridor was computed" on a stub plan and "no
    # freedom here" on a planned one, so gate on whether the pass actually ran.
    scores = plan.objective_scores
    corridor_scored = "corridor_half_width_min_m" in scores
    corridor_min = scores.get("corridor_half_width_min_m", 0.0)
    corridor_median = scores.get("corridor_half_width_median_m", 0.0)
    pinched = [v for v in plan.vertices if v.corridor_half_width_m <= 0.0]
    if corridor_scored and pinched and len(plan.vertices) > 1:
        findings.append(
            Finding(
                severity=Severity.INFO,
                code="corridor_pinched",
                message=(
                    f"{len(pinched)}/{len(plan.vertices)} waypoints "
                    f"({len(pinched) / len(plan.vertices):.0%}) grant no lateral freedom — "
                    "the route runs at the edge of its cover there, so a deviation leaves "
                    "the concealment the plan was scored on. Elsewhere the corridor is "
                    f"{corridor_median:.0f} m (median half-width)"
                ),
            )
        )

    # --- Crossings needing unsolved aerial manipulation ---
    manipulation = [c for c in plan.crossings if c.needs_manipulation]
    if manipulation:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="manipulation_crossings",
                message=(
                    f"{len(manipulation)} crossing(s) require aerial manipulation "
                    "(unsolved R&D) — human decision required"
                ),
            )
        )
    open_crossings = [c for c in plan.crossings if c.method is CrossingMethod.OPEN]
    if open_crossings:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="open_crossings",
                message=f"{len(open_crossings)} open surface crossing(s) — mechanical/visual risk",
            )
        )

    # --- Kinematic / tether feasibility (Software-Spec §3.6) ---
    # --- Stem routing (TEST-PLAN N4) -------------------------------------------
    scores = plan.objective_scores
    if "stem_min_clearance_m" in scores:
        breaches = int(scores.get("stem_sub_clearance_samples", 0.0))
        min_gap = scores["stem_min_clearance_m"]
        if breaches > 0:
            findings.append(
                Finding(
                    severity=Severity.CRITICAL,
                    code="stem_clearance_breach",
                    message=(
                        f"{breaches} point(s) on the route pass within the airframe's "
                        f"clearance of a mapped stem (minimum gap {min_gap:.2f} m). The "
                        "route cannot be flown as drawn: reactive avoidance is measured "
                        "not to recover this (TEST-PLAN N2)"
                    ),
                )
            )
        else:
            findings.append(
                Finding(
                    severity=Severity.INFO,
                    code="stem_routed",
                    message=(
                        f"route threads the mapped stems with {min_gap:.2f} m minimum "
                        f"clearance for {scores.get('stem_route_length_ratio', 1.0):.3f}x "
                        f"the unrefined length, "
                        f"{100 * scores.get('stem_mapped_fraction', 0.0):.0f} % "
                        "of it inside the mapped area. ⚠️ The stem map is SYNTHETIC and was "
                        "generated around the route planned without it — this shows the "
                        "planner can use a stem map, not that the aircraft will miss real trees"
                    ),
                )
            )
        failed = int(scores.get("stem_windows_failed", 0.0))
        if failed > 0:
            findings.append(
                Finding(
                    severity=Severity.WARNING,
                    code="stem_route_blocked",
                    message=(
                        f"{failed} stretch(es) of route had no admissible path between the "
                        "mapped stems and kept their unrefined line. Widen the corridor "
                        "there or accept that those stretches rely on onboard avoidance"
                    ),
                )
            )
        snapped = int(scores.get("stem_vertices_snapped", 0.0))
        if snapped > 0:
            findings.append(
                Finding(
                    severity=Severity.INFO,
                    code="stem_vertex_moved",
                    message=(
                        f"{snapped} waypoint(s) sat inside the clearance requirement of a "
                        "mapped stem and were moved. The concealment search placed them "
                        "without seeing the trunks — the most concealed vertices are the "
                        "least flyable (TEST-PLAN N4 composition failure)"
                    ),
                )
            )

    if plan.kinematics is not None:
        kin = plan.kinematics
        if kin.reversals > 0:
            findings.append(
                Finding(
                    severity=Severity.INFO,
                    code="tether_reversal",
                    message=(
                        f"{kin.reversals} route reversal(s) — acceptable at low payout speed; "
                        "the flight layer must slow through them to avoid slackening/snagging"
                    ),
                )
            )
        if kin.turn_violations > 0:
            findings.append(
                Finding(
                    severity=Severity.WARNING,
                    code="turn_radius",
                    message=(
                        f"{kin.turn_violations} corner(s) tighter than the "
                        f"{kin.min_turn_radius_m:.0f} m min turn radius "
                        f"(tightest achievable {kin.tightest_turn_radius_m:.1f} m)"
                    ),
                )
            )
        if kin.climb_violations > 0:
            findings.append(
                Finding(
                    severity=Severity.WARNING,
                    code="climb_rate",
                    message=(
                        f"{kin.climb_violations} altitude change(s) exceed the climb/descent limit "
                        "to stay in the navigable band"
                    ),
                )
            )
        # The altitude smoother works in AGL, so terrain-driven vertical motion is
        # invisible to `climb_violations`. Measure the true rate and say so when the
        # commanded profile is steeper than the airframe will fly.
        #
        # 🔴 FLOWN 2026-09-14 (`sim-logs/mvp/16_53_04.ulg`): this route predicted
        # 2.56 m/s of true descent and PX4 flew a maximum of **1.51 m/s**, with
        # ZERO samples over 2.0 — so the warning does NOT mean a limit is violated.
        # PX4 clamps descent at `MPC_Z_VEL_MAX_DN` (1.5 in SITL) and spreads a
        # descent across neighbouring legs, so a per-leg rate is the wrong frame
        # (the same thing RUNBOOK §4.9 found for B1's wp23). The climb side went the
        # other way: predicted 2.17, flown **2.82**, because PX4 sawtooths around
        # each terrain-following step and so exceeds the per-leg average.
        # The per-leg model is therefore wrong in BOTH directions, exactly like the
        # tether model before its PX4 prior. What the number really indicates is
        # **tracking lag** — the vehicle cannot hold a profile this steep and will
        # fall behind it — and the consequence of that lag is terrain clearance,
        # which `terrain_profile.py` already guarantees separately (predicted
        # 6.59 m here, flown 7.05 m, 0 samples under 5 m).
        # WARNING, not a feasibility break: constraining the AMSL rate changes which
        # routes are produced, which is an operator decision (SHORTFALLS 0b).
        over_climb = kin.max_climb_rate_amsl_ms > mission.drone.climb_rate_max_ms
        over_descent = kin.max_descent_rate_amsl_ms > mission.drone.descent_rate_max_ms
        if over_climb or over_descent:
            findings.append(
                Finding(
                    severity=Severity.WARNING,
                    code="vertical_rate_amsl_exceeded",
                    message=(
                        f"true vertical motion reaches {kin.max_climb_rate_amsl_ms:.2f} m/s up / "
                        f"{kin.max_descent_rate_amsl_ms:.2f} m/s down against a "
                        f"{mission.drone.climb_rate_max_ms:.2f} / "
                        f"{mission.drone.descent_rate_max_ms:.2f} m/s airframe. The altitude "
                        f"smoother constrains the AGL profile only, which reports "
                        f"{kin.max_climb_rate_ms:.2f} / {kin.max_descent_rate_ms:.2f} m/s here — "
                        "terrain-following motion is invisible to it. This is a TRACKING-LAG "
                        "indicator, not a violation: PX4 clamps descent at MPC_Z_VEL_MAX_DN "
                        "and spreads it across legs, so it flies a shallower profile than "
                        "commanded and falls behind rather than exceeding a limit (measured "
                        "2026-09-14: 2.56 m/s predicted, 1.51 m/s flown). Terrain clearance "
                        "under the lagging profile is guaranteed separately"
                    ),
                )
            )

        # CFIT: clearance is scored against the front-loaded descent profile PX4
        # actually flies, not linear interpolation. This is the check that would
        # have caught the 2026-07-24 terrain crash — it is NO-GO, not a warning.
        if kin.clearance_violations > 0:
            findings.append(
                Finding(
                    severity=Severity.CRITICAL,
                    code="terrain_clearance",
                    message=(
                        f"{kin.clearance_violations} leg(s) descend into terrain: minimum "
                        f"{kin.min_clearance_agl_m:.1f} m AGL under the flown descent profile. "
                        "PX4 front-loads a leg's descent instead of interpolating it along the "
                        "ground track, so clearance that holds at every vertex can still fly the "
                        "aircraft into a falling slope"
                    ),
                )
            )
        if kin.descent_vertices_inserted > 0:
            findings.append(
                Finding(
                    severity=Severity.INFO,
                    code="descent_densified",
                    message=(
                        f"{kin.descent_vertices_inserted} vertex/vertices inserted to split the "
                        "leg(s) whose flown descent hit terrain, so the autopilot cannot descend "
                        f"ahead of it (min clearance now {kin.min_clearance_agl_m:.1f} m AGL). "
                        "Each inserted vertex is one PX4 must accept vertically to sequence past"
                    ),
                )
            )

    # --- Endurance: can the airframe fly this before the battery runs out? ---
    # A hard constraint in the same class as the fiber budget and terrain
    # clearance, and it became one only when TEST-PLAN B2 measured the forest's
    # speed ceiling: route length is fixed, so every metre per second the trees
    # take away is paid in flight time.
    end = plan.endurance
    if end is not None and end.stated:
        if end.exceeds_raw:
            findings.append(
                Finding(
                    severity=Severity.CRITICAL,
                    code="endurance_exceeded",
                    message=(
                        f"route needs {end.energy_time_s / 60:.1f} min of flight but the "
                        f"airframe has {(end.endurance_s or 0.0) / 60:.1f} min of endurance — "
                        f"it cannot be flown at {end.min_sustainable_speed_ms:.2f} m/s or "
                        "slower. Shorten the route, raise cruise speed, or fit a bigger "
                        "battery"
                    ),
                )
            )
        elif not end.closes:
            findings.append(
                Finding(
                    severity=Severity.CRITICAL,
                    code="endurance_reserve_breached",
                    message=(
                        f"route needs {end.energy_time_s / 60:.1f} min against a usable "
                        f"budget of {(end.usable_endurance_s or 0.0) / 60:.1f} min "
                        f"({end.energy_fraction:.0%} of it) — it fits the battery only by "
                        "spending the reserve, which is not a margin the plan may assume"
                    ),
                )
            )
        else:
            findings.append(
                Finding(
                    severity=Severity.INFO,
                    code="endurance_budget",
                    message=(
                        f"flight time {end.flight_time_s / 60:.1f} min, "
                        f"{end.energy_fraction:.0%} of the usable budget; the route needs at "
                        f"least {end.min_sustainable_speed_ms:.2f} m/s to get home"
                    ),
                )
            )
    elif end is not None:
        # No endurance stated: report what the route demands rather than judging
        # it. A default here would manufacture a NO-GO out of an assumption.
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="endurance_unknown",
                message=(
                    f"no airframe endurance stated, so the route cannot be checked: it "
                    f"demands {end.required_endurance_s / 60:.1f} min including reserve "
                    f"({end.flight_time_s / 60:.1f} min of flight). Set "
                    "drone.endurance_s to make this a hard check"
                ),
            )
        )
    # --- Stem clutter: WHY the route is slow, and how much to trust that ---
    # The endurance finding above can be driven entirely by this speed cap, so the
    # operator must see that its density input is a forestry relation applied to a
    # canopy height model — not a stem inventory, which this program has never
    # ingested. It is stated next to the verdict it produces, not buried.
    if end is not None and end.speed_capped_legs > 0:
        measured_m = float(plan.objective_scores.get("below_canopy_density_measured_m", 0.0))
        measured_note = (
            f"{measured_m:.0f} m of below-canopy flight is priced from MEASURED stand "
            "data (layers.stand_data, Metsäkeskus stemcount). "
            if measured_m > 0.0
            else ""
        )
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="stem_speed_capped",
                message=(
                    f"{end.speed_capped_legs} leg(s) are slowed below cruise by stem density, "
                    f"giving an effective {end.effective_speed_ms:.2f} m/s against a "
                    f"{mission.drone.cruise_speed_ms:.2f} m/s airframe cruise, at the speeds "
                    "the onboard local planner flew clean in the harness. "
                    f"{measured_note}Unless the "
                    "mission states stem.density_per_ha, density elsewhere is inferred from "
                    "canopy height by a self-thinning relation (fields/stem_density.py), NOT "
                    "measured; the ordering (taller stands are sparser) is more "
                    "trustworthy than the absolute density"
                ),
            )
        )

    beyond_m = float(plan.objective_scores.get("below_canopy_beyond_envelope_m", 0.0))
    if beyond_m > 0.0:
        findings.append(
            Finding(
                severity=Severity.CRITICAL,
                code="stem_density_beyond_envelope",
                message=(
                    f"{beyond_m:.0f} m of the route is flown below the canopy through stands "
                    "denser than the onboard planner has been shown to fly "
                    f"({ONBOARD_PLANNING_ENVELOPE[-1][0]:.0f} stems/ha at "
                    f"{ONBOARD_PLANNING_ENVELOPE[-1][1]:.1f} m/s, "
                    "fields/stem_density.ONBOARD_PLANNING_ENVELOPE). The flight time charged "
                    "there is an extrapolation. Shrink the concealment zone "
                    "(covered_radius_m), route round the stand, or state a measured density "
                    "(stem.density_per_ha, layers.stand_data)"
                ),
            )
        )

    climbs = plan.canopy_climbs
    if climbs:
        why = {
            "dense_stand": "stand denser than the onboard envelope",
            "crowns_too_low": "crowns reach the floor, no trunk space",
            "stand_too_short": "stand too short to fly under",
        }
        first = climbs[0]
        lead = ""
        if first.start_m <= LAUNCH_CLIMB_M:
            lead = (
                f"FIRST CLIMB {first.start_m:.0f} m FROM LAUNCH: the aircraft rises to "
                f"{first.max_agl_m:.0f} m AGL over {first.max_canopy_m:.0f} m canopy almost "
                "immediately after take-off, which exposes the launch site. "
            )
        listed = "; ".join(
            f"{c.start_m:.0f} m from launch, {c.length_m:.0f} m long, up to "
            f"{c.max_agl_m:.0f} m AGL ({why.get(c.reason, c.reason)})"
            for c in climbs[:6]
        )
        more = (
            f"; and {len(climbs) - 6} more (report.json canopy_climbs)" if len(climbs) > 6 else ""
        )
        total = sum(c.length_m for c in climbs)
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="canopy_climb_in_zone",
                message=(
                    f"{lead}Inside the concealment zone the route leaves cover and flies over "
                    f"the canopy {len(climbs)} time(s), {total:.0f} m in all: {listed}{more}. "
                    "Where flying under is unpassable the planner goes over rather than "
                    "through (AltitudePolicy.over_canopy_when_unpassable); exposure there is "
                    "costed at the over-canopy altitude. To avoid a climb, move the launch "
                    "site, shrink covered_radius_m, or route round the stand."
                ),
            )
        )

    unopened = int(plan.objective_scores.get("canopy_transitions_without_opening", 0.0))
    if unopened:
        at = plan.objective_scores.get("canopy_transition_no_opening_first_m", 0.0)
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="canopy_transition_no_opening",
                message=(
                    f"{unopened} climb(s) out of or descent(s) into the canopy have no "
                    f"canopy opening within {OPENING_SEARCH_M:.0f} m of route (first at "
                    f"{at:.0f} m from launch): the aircraft would have to push through "
                    "closed crowns there. Openings come from layers.canopy_gaps (the 1 m "
                    "canopy model's open columns) or open ground. Re-route, move the "
                    "concealment zone, or accept the risk."
                ),
            )
        )

    crowns = plan.objective_scores.get("cable_on_crowns_m", 0.0)
    if crowns > 0.0:
        zone_crowns = plan.objective_scores.get("cable_on_crowns_in_zone_m", 0.0)
        trunk = plan.objective_scores.get("cable_in_trunk_space_m", 0.0)
        ground = plan.objective_scores.get("cable_on_ground_m", 0.0)
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="cable_on_crowns",
                message=(
                    f"{crowns:.0f} m of fiber is laid OVER treed canopy with no opening to "
                    f"fall through ({zone_crowns:.0f} m of it inside the concealment zone): "
                    "it settles on the crowns, not the ground — wind-exposed, abrading on "
                    "branches as payout continues (snag is the tether's real break "
                    "mechanism) and easy to cut. The rest: "
                    f"{trunk:.0f} m dropped through trunk space, {ground:.0f} m on open "
                    f"ground; it hangs off a crown edge into a stand at "
                    f"{plan.objective_scores.get('cable_crown_hangs', 0.0):.0f} place(s) "
                    "(snag points). Bounded model (laydown/settle.py, roadmap C7): the crown "
                    "length is what certainly does not reach the ground, not a calibrated "
                    "prediction."
                ),
            )
        )

    esc = plan.objective_scores.get("escape_max_agl_m")
    if esc is not None:
        esc_from = plan.objective_scores.get("escape_first_m", 0.0)
        span = plan.objective_scores.get("escape_exposed_length_m", 0.0)
        likely = plan.objective_scores.get("escape_likely_length_m", 0.0)
        where = (
            f" {likely:.0f} m of that runs through stands denser than 1000 stems/ha, "
            "where SITL needed an escape on about one traverse in four — expect one there."
            if likely > 0.0
            else ""
        )
        findings.append(
            Finding(
                severity=Severity.INFO,
                code="escape_climb_policy",
                message=(
                    f"In flight, if the aircraft is blocked under the canopy for "
                    f"{ESCAPE_AFTER_S:.0f} s it climbs STRAIGHT UP over the trees "
                    f"(droneflight fly-local --escape-after), holds that height for "
                    f"{ESCAPE_HOLD_M:.0f} m of route, then descends. On this route that can "
                    f"happen anywhere along the {span:.0f} m flown under the canopy, from "
                    f"{esc_from:.0f} m after launch, and would take it up to {esc:.0f} m AGL "
                    "(tallest nearby canopy + 3 m) — visible above the crowns."
                    + where
                    + " Turn it off with --escape-after 0 to stop and report instead."
                ),
            )
        )

    hidden_eyes = int(plan.objective_scores.get("observers_in_vegetation", 0.0))
    if hidden_eyes:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="observers_in_vegetation",
                message=(
                    f"{hidden_eyes} observer(s) stand in vegetation taller than their eye "
                    f"({mission.threat.observer_height_m:.0f} m). Under crown transmission the "
                    "eye is on the ground and looks out THROUGH it, so these observers see "
                    "little (demo AO: exposed cells 1335 -> 6 when its observers in 4-5 m "
                    "scrub moved off the scrub top). A real observer picks a vantage point — "
                    "place observers where they would actually stand to look."
                ),
            )
        )

    in_crowns = plan.objective_scores.get("below_canopy_in_crown_layer_m", 0.0)
    if in_crowns > 0.0 and mission.altitude.crown_ratio is None:
        default_ratio = mission.threat.canopy_occlusion.trunk_space_crown_ratio
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="flight_in_crown_layer",
                message=(
                    f"{in_crowns:.0f} m of below-canopy flight is planned above the estimated "
                    f"crown base (canopy x {1.0 - default_ratio:.2f}) — inside live crowns: "
                    "branches the SITL trunks "
                    "do not have, and only part of the crown overhead for cover (Askola: 12 m "
                    "AGL exceeded the 0.15 exposure cap against a close airborne sensor on "
                    "2 of 12 bearings; under the crown base it passed all 12). Set "
                    "altitude.crown_ratio to fly under the crown base."
                ),
            )
        )

    if end is not None and end.sharp_corners > 0:
        findings.append(
            Finding(
                severity=Severity.INFO,
                code="endurance_sharp_corners",
                message=(
                    f"{end.sharp_corners} corner(s) exceed 20 deg, where PX4 decelerates on "
                    "its own — the flight-time estimate does not charge for that and is "
                    "optimistic here"
                ),
            )
        )

    # --- Analysis coverage: how far up the M0->M2 ladder did we get? ---
    if not concealment_analyzed:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="analysis_not_performed",
                message=(
                    "no concealment/flyability analysis performed (straight-line stub route); "
                    "do not execute without review"
                ),
            )
        )
    elif not flyability_analyzed:
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="flyability_not_analyzed",
                message=(
                    "concealment/defilade analyzed, but forest flyability was not "
                    "(no lidar/voxel model); route assumes open flight — confirm before execution"
                ),
            )
        )

    recommendation = _decide(findings)

    return RiskReport(
        mission_name=mission.name,
        route_index=plan.route_index,
        recommendation=recommendation,
        generated_at=generated_at,
        total_length_m=plan.total_length_m,
        fiber_used_m=plan.fiber_used_m,
        fiber_available_m=mission.spool.length_m,
        exposed_length_m=exposed_length,
        drone_exposed_length_m=drone_exposed_length,
        below_canopy_length_m=below_canopy_length,
        corridor_half_width_min_m=corridor_min,
        corridor_half_width_median_m=corridor_median,
        canopy_climbs=plan.canopy_climbs,
        threat_sectors=threat_sectors,
        non_navigable_count=len(non_navigable),
        marginal_count=len(marginal),
        manipulation_crossings=len(manipulation),
        kinematics=plan.kinematics,
        endurance=plan.endurance,
        segments=tuple(
            SegmentReport(
                index=s.index,
                length_m=s.length_m,
                flyability=s.flyability,
                exposed=s.exposed,
                drone_exposed=s.drone_exposed,
            )
            for s in plan.segments
        ),
        crossings=tuple(
            CrossingReport(
                method=c.method,
                feasible=c.feasible,
                needs_manipulation=c.needs_manipulation,
                label=c.label,
            )
            for c in plan.crossings
        ),
        objective_scores=dict(plan.objective_scores),
        findings=tuple(findings),
    )
