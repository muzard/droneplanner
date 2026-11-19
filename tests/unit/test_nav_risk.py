"""Fix availability along the route (roadmap P2)."""

from __future__ import annotations

import numpy as np
import pytest

from droneplanner.fields.stems import StemMap
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.planner.nav_risk import DRIFT_FRACTION, assess_fix_availability

CELL = 10.0
N = 200  # 2 km square


def _grid() -> Grid:
    return Grid(transform=(CELL, 0.0, 0.0, 0.0, -CELL, N * CELL), rows=N, cols=N, crs="EPSG:3067")


def _line(x0: float, x1: float, y: float = 1000.0) -> list[Coordinate]:
    return [Coordinate(x=x0, y=y), Coordinate(x=x1, y=y)]


def _forest_between(x0: float, x1: float) -> np.ndarray:
    chm = np.zeros((N, N))
    chm[:, int(x0 / CELL) : int(x1 / CELL)] = 18.0
    return chm


def test_open_ground_everywhere_has_no_gap() -> None:
    s = assess_fix_availability(_line(5.0, 1995.0), np.zeros((N, N)), _grid())
    assert s["nav_max_unfixed_m"] <= 5.0 + 1e-9
    assert s["nav_unfixed_fraction"] == 0.0
    assert s["nav_runs_over_budget"] == 0.0


def test_a_forest_block_is_one_unfixed_run_and_drift_scales_with_it() -> None:
    s = assess_fix_availability(_line(5.0, 1995.0), _forest_between(500.0, 1500.0), _grid())
    assert s["nav_max_unfixed_m"] == pytest.approx(1000.0, abs=CELL)
    assert s["nav_max_unfixed_start_m"] == pytest.approx(500.0, abs=CELL)
    assert s["nav_max_drift_m"] == pytest.approx(s["nav_max_unfixed_m"] * DRIFT_FRACTION)
    assert s["nav_runs_over_budget"] == 1.0  # 20 m of drift against 15 m
    assert s["nav_unfixed_fraction"] == pytest.approx(1000.0 / 1990.0, abs=0.01)


def test_launching_inside_the_forest_counts_from_launch() -> None:
    s = assess_fix_availability(_line(5.0, 1995.0), _forest_between(0.0, 600.0), _grid())
    assert s["nav_max_unfixed_start_m"] == 0.0
    assert s["nav_max_unfixed_m"] == pytest.approx(600.0, abs=CELL)
    assert s["nav_runs_over_budget"] == 0.0  # 12 m < 15 m


def test_a_stem_map_supplies_fixes_under_the_canopy() -> None:
    chm = _forest_between(500.0, 1500.0)
    xs = np.arange(700.0, 1300.0, 5.0)
    stems = StemMap(x=xs, y=np.full(len(xs), 1003.0), radius_m=np.full(len(xs), 0.15))
    s = assess_fix_availability(_line(5.0, 1995.0), chm, _grid(), stems=stems)
    assert s["nav_max_unfixed_m"] == pytest.approx(200.0, abs=2 * CELL)
    assert s["nav_runs_over_budget"] == 0.0


def test_budget_is_reported_and_degenerate_routes_are_empty() -> None:
    s = assess_fix_availability(_line(5.0, 5.0), np.zeros((N, N)), _grid(), drift_budget_m=5.0)
    assert s["nav_max_unfixed_m"] == 0.0
    assert s["nav_drift_budget_m"] == 5.0
    assert assess_fix_availability([], np.zeros((N, N)), _grid())["nav_max_unfixed_m"] == 0.0


def test_the_report_carries_the_gap() -> None:
    from droneplanner.demo import build_demo
    from droneplanner.planner.corridor import plan_over_terrain
    from droneplanner.report.build import build_report

    mission, dtm, dsm, grid = build_demo(size=90)
    plan, _ = plan_over_terrain(mission, dtm, dsm, grid)
    assert "nav_max_unfixed_m" in plan.objective_scores
    report = build_report(mission, plan, concealment_analyzed=True, flyability_analyzed=True)
    nav = [f for f in report.findings if f.code in {"nav_fix_gap", "nav_fix_availability"}]
    assert len(nav) == 1
    over = plan.objective_scores["nav_max_drift_m"] > plan.objective_scores["nav_drift_budget_m"]
    assert (nav[0].code == "nav_fix_gap") == over
