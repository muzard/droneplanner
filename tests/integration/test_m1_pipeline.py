"""Integration: the pipeline runs M1 when DTM/DSM layers are supplied (§5.3, §7)."""

from __future__ import annotations

from pathlib import Path

from droneplanner.io.raster import write_raster
from droneplanner.pipeline import plan_mission
from tests.fixtures.synthetic import make_dem, make_mission


def _write_terrain(tmp_path: Path) -> tuple[Path, Path]:
    dtm = make_dem("ridge", size=48, cell_m=10.0)
    dsm = make_dem("ridge", size=48, cell_m=10.0)
    dtm_path = tmp_path / "dtm.tif"
    dsm_path = tmp_path / "dsm.tif"
    write_raster(dtm_path, dtm)
    write_raster(dsm_path, dsm)
    return dtm_path, dsm_path


def test_pipeline_runs_m1_with_layers(tmp_path: Path) -> None:
    dtm_path, dsm_path = _write_terrain(tmp_path)
    # Endpoints inside the 480 m x 480 m raster extent (origin at 0, 480).
    mission = make_mission(
        crs="EPSG:32633",
        start={"x": 60.0, "y": 60.0},
        end={"x": 420.0, "y": 420.0},
        threat={"observers": [{"x": 240.0, "y": 240.0}]},
        layers={"dtm": str(dtm_path), "dsm": str(dsm_path)},
    )
    results = plan_mission(mission, tmp_path / "out")
    assert len(results) == 1
    report = results[0].report

    codes = {f.code for f in report.findings}
    # M1 performed concealment analysis, so the stub "not performed" flag is gone,
    # but forest flyability (M2) has not run yet.
    assert "analysis_not_performed" not in codes
    assert "flyability_not_analyzed" in codes

    # The planned route has real turning points from the search.
    assert len(results[0].plan.vertices) >= 2
    assert (tmp_path / "out" / "trajectory.geojson").is_file()


def test_pipeline_stub_without_layers(tmp_path: Path) -> None:
    mission = make_mission()  # no layers -> straight-line stub
    results = plan_mission(mission, tmp_path / "out")
    codes = {f.code for f in results[0].report.findings}
    assert "analysis_not_performed" in codes
