"""Unit tests for the M1 corridor planner (Software-Spec §5.3)."""

from __future__ import annotations

import numpy as np

from droneplanner.demo import build_demo
from droneplanner.model.plan import FlightBand
from droneplanner.planner.corridor import plan_corridor, plan_over_terrain
from droneplanner.raster_lines import bresenham


def test_corridor_endpoints_match_mission() -> None:
    mission, dtm, dsm, grid = build_demo(size=90)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid)
    assert plan.start == mission.start
    assert plan.end == mission.end
    assert plan.total_length_m > 0.0


def test_corridor_reduces_exposure_vs_straight_line() -> None:
    mission, dtm, dsm, grid = build_demo(size=110)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid)
    exposure = surface.components["exposure"]
    s = grid.world_to_pixel(mission.start)
    g = grid.world_to_pixel(mission.end)
    naive_mean = float(np.mean([exposure[r, c] for r, c in bresenham(*s, *g)]))
    assert plan.objective_scores["mean_exposure"] < naive_mean


def test_corridor_objective_scores_present() -> None:
    mission, dtm, dsm, grid = build_demo(size=80)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid)
    for key in ("path_cost", "length_m", "mean_exposure", "max_exposure", "exposed_fraction"):
        assert key in plan.objective_scores


def test_corridor_flight_band_open() -> None:
    mission, dtm, dsm, grid = build_demo(size=80)
    plan = plan_corridor(mission, plan_over_terrain(mission, dtm, dsm, grid)[1])
    assert all(v.flight_band is FlightBand.OPEN for v in plan.vertices)


def test_corridor_is_deterministic() -> None:
    mission, dtm, dsm, grid = build_demo(size=80)
    a, _ = plan_over_terrain(mission, dtm, dsm, grid)
    b, _ = plan_over_terrain(mission, dtm, dsm, grid)
    assert a.model_dump() == b.model_dump()
