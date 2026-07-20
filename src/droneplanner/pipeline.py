"""End-to-end planning pipeline: mission -> plan(s) + report -> output files.

This is the orchestration the CLI calls. In M0 it runs the straight-line stub;
later milestones swap ``_plan_route`` for the real corridor/3D planners without
changing the output contract or the written artifacts.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from droneplanner.io.mavlink import write_plan
from droneplanner.io.mission import load_mission
from droneplanner.io.report import write_report
from droneplanner.io.trajectory import write_trajectory
from droneplanner.logging_config import get_logger
from droneplanner.model.mission import Mission
from droneplanner.model.plan import Plan
from droneplanner.model.report import RiskReport
from droneplanner.planner.stub import straight_line_plan
from droneplanner.report.build import build_report

_log = get_logger(__name__)


@dataclass(frozen=True)
class PlanResult:
    """A planned route paired with its report and the files written for it."""

    plan: Plan
    report: RiskReport
    trajectory_path: Path
    plan_path: Path
    report_json_path: Path
    report_md_path: Path


def _seed_everything(seed: int) -> None:
    """Seed all RNGs from the single master seed (Software-Spec §0 determinism)."""
    random.seed(seed)
    np.random.seed(seed % (2**32))


def _plan_route(mission: Mission, route_index: int) -> Plan:
    """Plan a single route. M0 = straight-line stub; M1+ swaps this in place."""
    return straight_line_plan(mission, route_index=route_index)


def plan_mission(
    mission: Mission,
    out_dir: str | Path,
    *,
    generated_at: datetime | None = None,
) -> list[PlanResult]:
    """Plan a mission's ``k`` routes and write all output artifacts.

    Returns one ``PlanResult`` per route. Determinism: identical mission + seed
    yields byte-identical trajectory/plan/report (``generated_at`` aside).
    """
    _seed_everything(mission.seed)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    results: list[PlanResult] = []
    multi = mission.k_routes > 1
    for route_index in range(mission.k_routes):
        plan = _plan_route(mission, route_index)
        report = build_report(mission, plan, generated_at=generated_at)

        suffix = f".route{route_index}" if multi else ""
        trajectory_path = out / f"trajectory{suffix}.geojson"
        plan_path = out / f"mission{suffix}.plan"
        report_json_path = out / f"report{suffix}.json"
        report_md_path = out / f"report{suffix}.md"

        write_trajectory(trajectory_path, plan)
        write_plan(plan_path, plan)
        write_report(report_json_path, report_md_path, report)

        _log.info(
            "planned route %d/%d: %.0f m, %s",
            route_index + 1,
            mission.k_routes,
            plan.total_length_m,
            report.recommendation.value,
        )
        results.append(
            PlanResult(
                plan=plan,
                report=report,
                trajectory_path=trajectory_path,
                plan_path=plan_path,
                report_json_path=report_json_path,
                report_md_path=report_md_path,
            )
        )
    return results


def plan_from_file(
    mission_path: str | Path,
    out_dir: str | Path,
    *,
    seed_override: int | None = None,
    generated_at: datetime | None = None,
) -> list[PlanResult]:
    """Load a mission YAML and plan it end-to-end."""
    mission = load_mission(mission_path)
    if seed_override is not None:
        mission = mission.model_copy(update={"seed": seed_override})
    return plan_mission(mission, out_dir, generated_at=generated_at)
