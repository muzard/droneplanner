"""Risk report writer — ``report.json`` (machine) + ``report.md`` (human).

Software-Spec §5.4 / §5.5: this is the human-in-the-loop artifact carrying the
GO / REVIEW / NO-GO recommendation.
"""

from __future__ import annotations

import json
from pathlib import Path

from droneplanner.model.report import RiskReport, Severity

_SEVERITY_ICON = {
    Severity.INFO: "",
    Severity.WARNING: "⚠️",
    Severity.CRITICAL: "⛔",
}


def report_to_json(report: RiskReport) -> str:
    """Serialize the report to deterministic, pretty JSON."""
    return json.dumps(report.model_dump(mode="json"), indent=2, sort_keys=True) + "\n"


def report_to_markdown(report: RiskReport) -> str:
    """Render the report as a human-readable Markdown document."""
    lines: list[str] = []
    lines.append(f"# Flight plan risk report — {report.mission_name}")
    lines.append("")
    lines.append(f"**Recommendation: {report.recommendation.value}**")
    if report.generated_at is not None:
        lines.append("")
        lines.append(f"_Generated {report.generated_at.isoformat()}_")
    lines.append("")

    lines.extend(_operator_briefing(report))

    lines.append("## Summary")
    lines.append("")
    lines.append("| Metric | Value |")
    lines.append("| --- | ---: |")
    lines.append(f"| Route index | {report.route_index} |")
    lines.append(f"| Total route length | {report.total_length_m:.1f} m |")
    lines.append(f"| Fiber used | {report.fiber_used_m:.1f} m |")
    lines.append(f"| Fiber available | {report.fiber_available_m:.1f} m |")
    lines.append(f"| Exposed cable length | {report.exposed_length_m:.1f} m |")
    lines.append(f"| Exposed flight length (drone) | {report.drone_exposed_length_m:.1f} m |")
    lines.append(f"| Flown below canopy | {report.below_canopy_length_m:.1f} m |")
    lines.append(
        f"| Corridor half-width (median / min) | "
        f"{report.corridor_half_width_median_m:.0f} / {report.corridor_half_width_min_m:.0f} m |"
    )
    lines.append(f"| Non-navigable segments | {report.non_navigable_count} |")
    lines.append(f"| Marginal segments | {report.marginal_count} |")
    lines.append(f"| Crossings needing manipulation | {report.manipulation_crossings} |")
    lines.append("")

    # Sectors get their own section rather than a findings line: they are an
    # intelligence *assumption* that removed cost from the surface, so the person
    # signing this off should see exactly what was assumed (roadmap C3).
    if report.threat_sectors:
        lines.append("## Threat sectors applied")
        lines.append("")
        for sector in report.threat_sectors:
            lines.append(f"- {sector}")
        lines.append("")

    if report.kinematics is not None:
        k = report.kinematics
        lines.append("## Kinematic feasibility")
        lines.append("")
        lines.append(f"**Trajectory {'feasible' if k.feasible else 'INFEASIBLE'}**")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("| --- | ---: |")
        lines.append(f"| Min turn radius | {k.min_turn_radius_m:.1f} m |")
        lines.append(f"| Tightest turn achieved | {k.tightest_turn_radius_m:.1f} m |")
        lines.append(f"| Turn-radius violations | {k.turn_violations} |")
        lines.append(f"| Reversals | {k.reversals} |")
        lines.append(f"| Max climb rate (AGL profile) | {k.max_climb_rate_ms:.2f} m/s |")
        lines.append(f"| Max descent rate (AGL profile) | {k.max_descent_rate_ms:.2f} m/s |")
        lines.append(f"| Max climb rate (true vertical) | {k.max_climb_rate_amsl_ms:.2f} m/s |")
        lines.append(f"| Max descent rate (true vertical) | {k.max_descent_rate_amsl_ms:.2f} m/s |")
        lines.append(f"| Climb-rate violations | {k.climb_violations} |")
        lines.append("")

    if report.endurance is not None:
        e = report.endurance
        lines.append("## Endurance budget")
        lines.append("")
        if not e.stated:
            verdict = "NOT CHECKED — no airframe endurance stated"
        elif e.exceeds_raw:
            verdict = "INFEASIBLE — outruns the battery"
        elif not e.closes:
            verdict = "INFEASIBLE — fits only by spending the reserve"
        else:
            verdict = "Closes"
        lines.append(f"**{verdict}**")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("| --- | ---: |")
        lines.append(f"| Flight time | {e.flight_time_s / 60:.1f} min |")
        lines.append(f"| — cruise | {e.cruise_time_s / 60:.1f} min |")
        lines.append(f"| — vertical-rate bound | {e.vertical_bound_time_s / 60:.1f} min |")
        lines.append(f"| — waypoint overhead | {e.waypoint_overhead_s / 60:.1f} min |")
        lines.append(f"| — route ends | {e.terminal_overhead_s / 60:.1f} min |")
        lines.append(f"| Energy-equivalent time | {e.energy_time_s / 60:.1f} min |")
        if e.stated:
            lines.append(f"| Usable budget | {(e.usable_endurance_s or 0.0) / 60:.1f} min |")
            lines.append(f"| Budget consumed | {e.energy_fraction:.0%} |")
        lines.append(
            f"| Endurance required (incl. reserve) | {e.required_endurance_s / 60:.1f} min |"
        )
        # The number the forest speed ceiling has to be compared against: below
        # this the aircraft cannot get home, above it the trees are unavoidable.
        if e.min_sustainable_speed_ms > 0.0:
            lines.append(f"| Min sustainable cruise | {e.min_sustainable_speed_ms:.2f} m/s |")
        lines.append(f"| Vertical travel commanded | {e.vertical_travel_m:.0f} m |")
        lines.append("")

    if report.objective_scores:
        lines.append("## Objective scores")
        lines.append("")
        lines.append("| Objective | Score |")
        lines.append("| --- | ---: |")
        for key in sorted(report.objective_scores):
            lines.append(f"| {key} | {report.objective_scores[key]:.4f} |")
        lines.append("")

    lines.append("## Findings")
    lines.append("")
    if report.findings:
        for finding in report.findings:
            icon = _SEVERITY_ICON.get(finding.severity, "")
            prefix = f"{icon} " if icon else ""
            lines.append(
                f"- {prefix}**{finding.severity.value}** ({finding.code}): {finding.message}"
            )
    else:
        lines.append("- None.")
    lines.append("")

    if report.crossings:
        lines.append("## Crossings")
        lines.append("")
        lines.append("| Method | Feasible | Needs manipulation | Label |")
        lines.append("| --- | :---: | :---: | --- |")
        for crossing in report.crossings:
            lines.append(
                f"| {crossing.method.value} | {'yes' if crossing.feasible else 'no'} | "
                f"{'yes' if crossing.needs_manipulation else 'no'} | {crossing.label or ''} |"
            )
        lines.append("")

    return "\n".join(lines) + "\n"


def write_report(json_path: str | Path, md_path: str | Path, report: RiskReport) -> None:
    """Write both ``report.json`` and ``report.md``."""
    Path(json_path).write_text(report_to_json(report), encoding="utf-8")
    Path(md_path).write_text(report_to_markdown(report), encoding="utf-8")


_BRIEFING = (
    ("canopy_climb_in_zone", "Planned climbs out of cover"),
    ("canopy_transition_no_opening", "Climbs/descents with no canopy opening"),
    ("flight_in_crown_layer", "Flying inside the crowns"),
    ("escape_climb_policy", "If it gets stuck"),
    ("cable_on_crowns", "Where the fiber lands"),
    ("open_crossings", "Road crossings (fiber across a road)"),
    ("stem_density_beyond_envelope", "Stands beyond the demonstrated envelope"),
)


def _operator_briefing(report: RiskReport) -> list[str]:
    """What the aircraft will VISIBLY do, before launch — first thing in the report.

    User, 2026-09-25: the operator must know beforehand "so no surprises happen and
    the drone jumps up to height right from the launch site". The same findings are
    in the full list below; this pulls the ones about visible behaviour and cable
    placement to the top, in plain order."""
    by_code = {f.code: f for f in report.findings}
    items = [(title, by_code[code]) for code, title in _BRIEFING if code in by_code]
    out = ["## Operator briefing", ""]
    if report.canopy_climbs:
        out.append("| Climb | From launch | Length | Up to | Why |")
        out.append("| ---: | ---: | ---: | ---: | --- |")
        for i, c in enumerate(report.canopy_climbs, 1):
            out.append(
                f"| {i} | {c.start_m:.0f} m | {c.length_m:.0f} m | {c.max_agl_m:.0f} m AGL "
                f"| {c.reason.replace('_', ' ')} |"
            )
        out.append("")
    if not items:
        out.append("_No climbs out of cover, escape policy or cable-placement warnings._")
        out.append("")
        return out
    for title, f in items:
        out.append(f"- **{title}** ({f.severity.value}): {f.message}")
    out.append("")
    return out
