"""Unit tests for the M1 corridor planner (Software-Spec §5.3)."""

from __future__ import annotations

from itertools import pairwise

import numpy as np
import pytest

from droneplanner.demo import build_demo, demo_flyability
from droneplanner.fields.altitude import canopy_height
from droneplanner.model.plan import FlightBand
from droneplanner.planner.corridor import _track_weighted_mean, plan_corridor, plan_over_terrain
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
    for key in (
        "path_cost",
        "length_m",
        "mean_exposure",
        "max_exposure",
        "exposed_fraction",
        "mean_exposure_flown",
        "max_exposure_flown",
        "mean_exposure_cable",
    ):
        assert key in plan.objective_scores


def test_drone_and_cable_exposure_are_scored_separately() -> None:
    """SHORTFALLS item 1, end to end: the aircraft is the exposed object, not the cable.

    Under canopy the two genuinely differ — the cable lies on ground the observers
    can see over, the drone is beneath the crowns. Before the fix both numbers
    were the ground's.
    """
    mission, dtm, dsm, grid = build_demo(size=90)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid)
    scores = plan.objective_scores
    assert scores["mean_exposure"] != scores["mean_exposure_cable"]
    assert not np.array_equal(surface.components["exposure"], surface.components["exposure_ground"])


def test_flown_trajectory_exposure_tracks_the_raster_metric() -> None:
    """The re-score at flown altitudes must corroborate the raster, not replace it.

    They are computed differently on purpose (raster: corridor cells at the band
    altitude; flown: the filleted 3D trajectory under PX4's front-loaded descent).
    A large gap would mean the field is scored at an altitude the route does not
    fly — the thing this item exists to prevent.
    """
    mission, dtm, dsm, grid = build_demo(size=90)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid)
    flown = plan.objective_scores["mean_exposure_flown"]
    raster = plan.objective_scores["mean_exposure"]
    assert 0.0 <= flown <= 1.0
    assert abs(flown - raster) < 0.15


def test_flown_exposure_is_weighted_by_track_length_not_by_sample_count() -> None:
    """Corners must not carry the metric.

    The sampler puts a fixed number of samples on every leg, and a filleted route
    is mostly short arc legs, so an unweighted mean would report the corners.
    """
    values = np.array([1.0, 1.0, 0.0], dtype=np.float32)
    samples = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (999.0, 0.0, 0.0)]  # two close, one far
    weighted = _track_weighted_mean(samples, values)
    assert weighted < 0.51, "the long final leg must dominate, not the tight pair"
    assert weighted < float(np.mean(values))
    # Degenerate inputs stay finite.
    assert _track_weighted_mean([], np.array([])) == 0.0
    assert _track_weighted_mean([(0.0, 0.0, 0.0)], np.array([0.25])) == 0.25
    assert _track_weighted_mean([(0.0, 0.0, 0.0)] * 2, np.array([0.2, 0.4])) == pytest.approx(0.3)


def test_corridor_flies_under_the_canopy_where_there_is_canopy() -> None:
    """The route hides *inside* the stands, not above them (SHORTFALLS item 1c).

    Without a lidar model the ceiling comes from the canopy height (DSM - DTM):
    tall stands are entered, clearings are crossed at the open target altitude.
    """
    mission, dtm, dsm, grid = build_demo(size=80)
    plan = plan_corridor(mission, plan_over_terrain(mission, dtm, dsm, grid)[1])
    bands = {v.flight_band for v in plan.vertices}
    assert FlightBand.BELOW_CANOPY in bands
    assert bands <= {FlightBand.BELOW_CANOPY, FlightBand.CANOPY_TOP, FlightBand.OPEN}

    # Every vertex is either under the crowns or clear above them — never inside.
    chm = canopy_height(dtm, dsm)
    for v in plan.vertices:
        canopy = float(chm[grid.world_to_pixel(v.coordinate)])
        if canopy < mission.altitude.min_canopy_m:
            continue
        under = v.altitude_agl_m <= canopy - mission.altitude.canopy_clearance_m
        over = v.altitude_agl_m >= canopy + mission.altitude.canopy_clearance_m
        assert under or over, (
            f"vertex sits in the crowns: {v.altitude_agl_m:.1f} m in {canopy:.1f} m"
        )


def test_altitude_is_low_not_a_fixed_cruise_height() -> None:
    """Regression on the posture itself: a 30 m constant AGL is what made the
    concealment fake, and it is what the planner emitted before 2026-07-27."""
    mission, dtm, dsm, grid = build_demo(size=80)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid)
    alts = [v.altitude_agl_m for v in plan.vertices]
    assert max(alts) < 30.0
    assert len(set(round(a, 1) for a in alts)) > 1  # it tracks the canopy, not a constant


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


def test_corridor_clears_terrain_under_the_flown_descent_profile() -> None:
    """SHORTFALLS item 0 regression: the planner's own clearance guarantee must
    hold against PX4's front-loaded descent, not just linear interpolation.

    A plan that passed the old linear check flew into the ground in SITL on
    2026-07-24. On the demo AO the flown profile put the route 6.6 m *underground*
    before the descent-densification pass; it must now clear.
    """
    mission, dtm, dsm, grid = build_demo(size=120)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid)

    kin = plan.kinematics
    assert kin is not None
    assert kin.clearance_violations == 0
    assert kin.min_clearance_agl_m > 5.0
    assert kin.feasible


def test_clearance_is_scored_against_the_flown_profile_not_linear_interp() -> None:
    """The check must be the pessimistic one — if these ever agree, the flown
    profile has silently stopped being modelled."""
    from droneplanner.model.geometry import Coordinate
    from droneplanner.planner.terrain_profile import FlownProfile, assess_clearance

    mission, dtm, dsm, grid = build_demo(size=120)
    plan, _surface = plan_over_terrain(mission, dtm, dsm, grid)

    def ground(x: float, y: float) -> float:
        return float(dtm[grid.world_to_pixel(Coordinate(x=x, y=y))])

    pts = [(v.x, v.y) for v in plan.vertices]
    amsl = [v.altitude_amsl_m for v in plan.vertices]
    linear = assess_clearance(
        pts, amsl, ground, FlownProfile(descent_gradient=0.0), margin_agl_m=5.0
    )
    assert plan.kinematics is not None
    assert plan.kinematics.min_clearance_agl_m < linear.min_clearance_agl_m
