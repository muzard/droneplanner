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
        lines.append(f"| Max climb rate | {k.max_climb_rate_ms:.2f} m/s |")
        lines.append(f"| Max descent rate | {k.max_descent_rate_ms:.2f} m/s |")
        lines.append(f"| Climb-rate violations | {k.climb_violations} |")
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
