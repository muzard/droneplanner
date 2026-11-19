"""Launch / end-point exposure: where the crew stands (roadmap P3)."""

from __future__ import annotations

import numpy as np

from droneplanner.demo import build_demo
from droneplanner.model.geometry import Coordinate
from droneplanner.planner.corridor import plan_over_terrain
from droneplanner.planner.ground_points import GROUND_POINT_EXPOSED, score_ground_points
from droneplanner.report.build import build_report


def _bare_demo_with_observer_near_start(offset_m: float):  # type: ignore[no-untyped-def]
    mission, dtm, _dsm, grid = build_demo(size=90)
    dtm = np.full_like(dtm, 50.0)  # flat, bare: the launch site is open ground
    dsm = dtm.copy()
    near = Coordinate(x=mission.start.x + offset_m, y=mission.start.y)
    threat = mission.threat.model_copy(update={"observers": (near,), "sectors": ()})
    return mission.model_copy(update={"threat": threat}), dtm, dsm, grid


def test_an_open_launch_site_near_an_observer_is_flagged() -> None:
    mission, dtm, dsm, grid = _bare_demo_with_observer_near_start(300.0)
    plan, _ = plan_over_terrain(mission, dtm, dsm, grid)
    s = plan.objective_scores
    assert s["launch_crew_los"] == 1.0
    assert s["launch_crew_exposure"] > GROUND_POINT_EXPOSED
    report = build_report(mission, plan, concealment_analyzed=True, flyability_analyzed=True)
    hit = [f for f in report.findings if f.code == "launch_site_exposed"]
    assert hit and "LAUNCH SITE" in hit[0].message


def test_detection_and_line_of_sight_are_scored_separately() -> None:
    """The scorer is asked twice: with the range term and without it."""
    mission, dtm, dsm, grid = _bare_demo_with_observer_near_start(0.0)
    far = Coordinate(x=mission.start.x, y=mission.start.y + 1.0)
    # Score directly with a stub scorer: in sight (1.0) but undetected (0.0).
    got = score_ground_points(
        [mission.start, far], dtm, grid, lambda pts, det: np.full(len(pts), 0.0 if det else 1.0)
    )
    assert got == [(0.0, 1.0), (0.0, 1.0)]


def test_a_wall_between_observer_and_launch_hides_the_crew() -> None:
    mission, dtm, dsm, grid = _bare_demo_with_observer_near_start(300.0)
    r, c = grid.world_to_pixel(mission.start)
    walled = dsm.copy()
    walled[:, c + 2 : c + 4] = 80.0  # a 30 m wall of forest east of the start
    plan, _ = plan_over_terrain(mission, dtm, walled, grid)
    assert plan.objective_scores["launch_crew_los"] == 0.0
    assert plan.objective_scores["launch_crew_exposure"] == 0.0
    report = build_report(mission, plan, concealment_analyzed=True, flyability_analyzed=True)
    assert not any(f.code.startswith("launch_site") for f in report.findings)


def test_no_points_scores_nothing() -> None:
    mission, dtm, _dsm, grid = build_demo(size=30)
    assert score_ground_points([], dtm, grid, lambda pts, det: np.zeros(len(pts))) == []


def test_range_hidden_site_gets_an_info_not_a_warning() -> None:
    mission, dtm, dsm, grid = _bare_demo_with_observer_near_start(300.0)
    plan, _ = plan_over_terrain(mission, dtm, dsm, grid)
    scores = dict(plan.objective_scores) | {"launch_crew_exposure": 0.0, "launch_crew_los": 1.0}
    report = build_report(
        mission,
        plan.model_copy(update={"objective_scores": scores}),
        concealment_analyzed=True,
        flyability_analyzed=True,
    )
    codes = {f.code for f in report.findings}
    assert "launch_site_in_line_of_sight" in codes
    assert "launch_site_exposed" not in codes
