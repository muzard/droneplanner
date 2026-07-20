"""Unit tests for the M1 corridor planner (Software-Spec §5.3)."""

from __future__ import annotations

from itertools import pairwise

import numpy as np

from droneplanner.demo import build_demo, demo_flyability
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


def test_corridor_trajectory_is_kinematically_smoothed() -> None:
    import math

    mission, dtm, dsm, grid = build_demo(size=110)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid)
    assert plan.kinematics is not None
    assert plan.kinematics.feasible
    assert plan.kinematics.tightest_turn_radius_m <= mission.drone.min_turn_radius_m + 1e-6

    # After filleting, no corner deflects more than a gentle per-arc-step angle.
    vs = [(v.x, v.y) for v in plan.vertices]
    worst = 0.0
    for a, b, c in zip(vs, vs[1:], vs[2:], strict=False):
        u1 = (b[0] - a[0], b[1] - a[1])
        u2 = (c[0] - b[0], c[1] - b[1])
        l1, l2 = math.hypot(*u1), math.hypot(*u2)
        if l1 == 0 or l2 == 0:
            continue
        dot = (u1[0] * u2[0] + u1[1] * u2[1]) / (l1 * l2)
        worst = max(worst, math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
    assert worst < 60.0


def test_corridor_altitude_varies_with_canopy() -> None:
    mission, dtm, dsm, grid = build_demo(size=120)
    fly = demo_flyability(grid, mission.drone)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid, flyability=fly)
    alts = {round(v.altitude_agl_m, 1) for v in plan.vertices}
    assert len(alts) > 1  # altitude varies to follow the shifting navigable band
    assert plan.kinematics is not None and plan.kinematics.climb_violations == 0


def test_corridor_avoids_non_navigable_forest() -> None:
    mission, dtm, dsm, grid = build_demo(size=120)
    fly = demo_flyability(grid, mission.drone)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid, flyability=fly)

    vs = [grid.world_to_pixel(v.coordinate) for v in plan.vertices]
    for a, b in pairwise(vs):
        for r, c in bresenham(a[0], a[1], b[0], b[1]):
            assert not fly.non_navigable[r, c]
    # M2 flies the trunk band, not the M1 constant AGL.
    assert all(v.flight_band is FlightBand.UPPER_TRUNK for v in plan.vertices)
