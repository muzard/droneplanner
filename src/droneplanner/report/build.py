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

from datetime import datetime

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


def _decide(findings: list[Finding]) -> Recommendation:
    severities = {f.severity for f in findings}
    if Severity.CRITICAL in severities:
        return Recommendation.NO_GO
    if Severity.WARNING in severities:
        return Recommendation.REVIEW
    return Recommendation.GO


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
    below_canopy_length = sum(s.length_m for s in below_canopy)
    if below_canopy:
        lidar_verified = all(s.flight_band is FlightBand.UPPER_TRUNK for s in below_canopy)
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
        findings.append(
            Finding(
                severity=Severity.WARNING,
                code="stem_speed_capped",
                message=(
                    f"{end.speed_capped_legs} leg(s) are slowed below cruise by modelled "
                    f"stem spacing, giving an effective {end.effective_speed_ms:.2f} m/s "
                    f"against a {plan.vertices[0].speed_hint_ms:.2f} m/s airframe cruise. "
                    "Stem density is derived from canopy height by a self-thinning "
                    "relation (fields/stem_density.py), NOT measured — no stem inventory "
                    "has been ingested. The ordering (taller stands are sparser) is more "
                    "trustworthy than the absolute density"
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
