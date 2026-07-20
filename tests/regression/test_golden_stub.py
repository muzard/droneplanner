"""Golden-plan regression test (Software-Spec §6.4).

Serializes a plan + report for a fixed mission and compares to stored snapshots,
guarding against silent planner drift. Regenerate deliberately (reviewed in PRs):

    UPDATE_GOLDEN=1 pytest tests/regression -q
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from droneplanner.io.report import report_to_json
from droneplanner.io.trajectory import trajectory_to_geojson
from droneplanner.planner.stub import straight_line_plan
from droneplanner.report.build import build_report
from tests.fixtures.synthetic import make_mission

GOLDEN_DIR = Path(__file__).parent / "golden"


def _fixed_mission() -> object:
    return make_mission(
        name="golden-ao",
        start={"x": 500000.0, "y": 5200000.0},
        end={"x": 503000.0, "y": 5204000.0},
        waypoint_spacing_m=250.0,
        cruise_agl_m=30.0,
        fiber_slack_factor=1.05,
        seed=123,
    )


def _artifacts() -> dict[str, str]:
    mission = _fixed_mission()
    plan = straight_line_plan(mission)  # type: ignore[arg-type]
    report = build_report(mission, plan, generated_at=None)  # type: ignore[arg-type]
    trajectory = json.dumps(trajectory_to_geojson(plan), indent=2, sort_keys=True) + "\n"
    return {"trajectory.geojson": trajectory, "report.json": report_to_json(report)}


def test_golden_snapshots_match() -> None:
    artifacts = _artifacts()
    if os.environ.get("UPDATE_GOLDEN"):
        GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
        for name, content in artifacts.items():
            (GOLDEN_DIR / name).write_text(content, encoding="utf-8")

    for name, content in artifacts.items():
        golden = GOLDEN_DIR / name
        assert golden.is_file(), f"missing golden {name}; run UPDATE_GOLDEN=1 pytest"
        assert content == golden.read_text(encoding="utf-8"), (
            f"{name} drifted from golden snapshot; review and rerun UPDATE_GOLDEN=1 if intended"
        )


def test_golden_plan_is_reproducible() -> None:
    assert _artifacts() == _artifacts()
