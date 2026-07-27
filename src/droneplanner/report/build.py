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
from droneplanner.model.plan import CrossingMethod, FlyabilityClass, Plan
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

    # --- Cable exposure ---
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
                        f"{kin.descent_vertices_inserted} vertex/vertices inserted to split "
                        "big-drop legs so the autopilot cannot descend ahead of the terrain "
                        f"(min clearance now {kin.min_clearance_agl_m:.1f} m AGL)"
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
        non_navigable_count=len(non_navigable),
        marginal_count=len(marginal),
        manipulation_crossings=len(manipulation),
        kinematics=plan.kinematics,
        segments=tuple(
            SegmentReport(
                index=s.index,
                length_m=s.length_m,
                flyability=s.flyability,
                exposed=s.exposed,
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
