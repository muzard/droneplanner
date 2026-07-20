"""End-to-end integration: mission.yaml -> plan + report (Software-Spec §6.5).

The M0 acceptance criterion: ``droneplanner plan`` runs end-to-end returning a
straight-line stub plan + report.
"""

from __future__ import annotations

import json
from pathlib import Path

from typer.testing import CliRunner

from droneplanner.cli import app
from droneplanner.pipeline import plan_from_file

runner = CliRunner()


def test_cli_plan_end_to_end(example_mission_path: Path, tmp_path: Path) -> None:
    out = tmp_path / "out"
    result = runner.invoke(
        app,
        ["plan", "--mission", str(example_mission_path), "--out", str(out)],
    )
    assert result.exit_code == 0, result.output
    assert "REVIEW" in result.output

    # All four output artifacts exist.
    for name in ("trajectory.geojson", "mission.plan", "report.json", "report.md"):
        assert (out / name).is_file(), f"missing {name}"

    # Report JSON is schema-conformant and sane.
    report = json.loads((out / "report.json").read_text())
    assert report["recommendation"] == "REVIEW"
    assert report["fiber_available_m"] == 5000.0
    assert report["total_length_m"] > 0

    # .plan is valid QGC JSON.
    plan_doc = json.loads((out / "mission.plan").read_text())
    assert plan_doc["fileType"] == "Plan"
    assert len(plan_doc["mission"]["items"]) >= 2


def test_cli_version() -> None:
    result = runner.invoke(app, ["version"])
    assert result.exit_code == 0
    assert result.output.strip()


def test_cli_missing_mission_file(tmp_path: Path) -> None:
    result = runner.invoke(
        app, ["plan", "--mission", str(tmp_path / "nope.yaml"), "--out", str(tmp_path / "o")]
    )
    assert result.exit_code != 0


def test_cli_bad_mission_reports_error(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\ncrs: EPSG:4326\nstart: {x: 0, y: 0}\nend: {x: 1, y: 1}\n")
    result = runner.invoke(app, ["plan", "--mission", str(bad), "--out", str(tmp_path / "o")])
    assert result.exit_code == 2
    assert "error:" in result.output


def test_pipeline_seed_override(example_mission_path: Path, tmp_path: Path) -> None:
    results = plan_from_file(example_mission_path, tmp_path / "o", seed_override=999)
    assert len(results) == 1
    assert results[0].plan.seed == 999


def test_pipeline_k_routes_writes_suffixed_files(
    tmp_path: Path, example_mission_path: Path
) -> None:
    from droneplanner.io.mission import load_mission
    from droneplanner.pipeline import plan_mission

    mission = load_mission(example_mission_path).model_copy(update={"k_routes": 2})
    results = plan_mission(mission, tmp_path / "o")
    assert len(results) == 2
    assert (tmp_path / "o" / "trajectory.route0.geojson").is_file()
    assert (tmp_path / "o" / "trajectory.route1.geojson").is_file()
