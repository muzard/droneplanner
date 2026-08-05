"""Unit tests for the flight-time / endurance budget (TEST-PLAN N1).

The question N1 exists to answer: TEST-PLAN B2 measured a **speed ceiling imposed
by the forest** (~3.5 m/s at 107 stems/ha, 1.6-2.5 at real managed-forest
density), and route length is fixed, so the whole cost of that cap lands on
flight time. These tests pin that the budget is calibrated against the one flight
that has actually happened, and that it fails loudly rather than optimistically.
"""

from __future__ import annotations

import math

import pytest

from droneplanner.planner.endurance import (
    B1_TERMINAL_OVERHEAD_S,
    B1_WAYPOINT_OVERHEAD_S,
    EnduranceModel,
    assess_endurance,
)

# B1, the only route flown end to end: sim-logs/canopy/21_38_21.ulg, AUTO_MISSION
# phase. 282 waypoints, commanded 6.0 m/s, COMPLETED 282/282.
B1_FLOWN_TIME_S = 956.0
B1_LENGTH_M = 5299.0
B1_WAYPOINTS = 282


def _straight(n: int, length_m: float, *, amsl: float = 100.0) -> tuple[list, list]:
    """``n`` collinear vertices spanning ``length_m``, at constant altitude."""
    step = length_m / (n - 1)
    pts = [(i * step, 0.0) for i in range(n)]
    return pts, [amsl] * n


def _model(**kw: float | None) -> EnduranceModel:
    base: dict = {
        "cruise_speed_ms": 6.0,
        "climb_rate_ms": 3.0,
        "descent_rate_ms": 2.0,
        "endurance_s": 1800.0,
    }
    base.update(kw)
    return EnduranceModel(**base)


class TestB1Calibration:
    """The model must reproduce the one flight it was calibrated against.

    ⚠️ This is a *consistency* check, not a validation: both overhead constants
    were fitted to this same 956.0 s, so passing is very nearly an identity. Its
    job is to catch the constants or the arithmetic drifting apart, and to make
    the provenance of the numbers executable. The first independent test is the
    B2b speed sweep at 3.5 and 2.0 m/s.
    """

    def test_reproduces_the_flown_time_within_one_percent(self) -> None:
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model())
        assert rep.flight_time_s == pytest.approx(B1_FLOWN_TIME_S, rel=0.01)

    def test_naive_length_over_cruise_is_measurably_optimistic(self) -> None:
        """The whole reason the overheads exist: ``length/cruise`` is 7.6 % low."""
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model())
        naive = B1_LENGTH_M / 6.0
        assert naive < rep.flight_time_s
        assert rep.flight_time_s - naive == pytest.approx(
            B1_WAYPOINT_OVERHEAD_S * B1_WAYPOINTS + B1_TERMINAL_OVERHEAD_S, rel=1e-6
        )

    def test_waypoint_overhead_is_paid_per_waypoint_not_per_metre(self) -> None:
        """Densifying a route costs time even though it does not lengthen it.

        This is what makes the 25 m below-canopy posture expensive: B1 pays the
        acceptance-radius overhead 282 times over the same 5.3 km.
        """
        sparse_pts, sparse_amsl = _straight(30, B1_LENGTH_M)
        dense_pts, dense_amsl = _straight(282, B1_LENGTH_M)
        sparse = assess_endurance(sparse_pts, sparse_amsl, _model())
        dense = assess_endurance(dense_pts, dense_amsl, _model())
        assert dense.flight_time_s > sparse.flight_time_s
        assert dense.cruise_time_s == pytest.approx(sparse.cruise_time_s, rel=1e-9)
        assert dense.waypoint_overhead_s - sparse.waypoint_overhead_s == pytest.approx(
            B1_WAYPOINT_OVERHEAD_S * (282 - 30), rel=1e-9
        )


class TestVerticalTerm:
    def test_a_leg_whose_descent_outlasts_its_cruise_takes_the_longer_time(self) -> None:
        """A 40 m drop over a 100 m leg cannot be flown in the 16.7 s cruise takes."""
        pts = [(0.0, 0.0), (100.0, 0.0)]
        amsl = [140.0, 100.0]
        rep = assess_endurance(pts, amsl, _model())
        assert rep.vertical_bound_time_s == pytest.approx(40.0 / 2.0 - 100.0 / 6.0, rel=1e-9)

    def test_shallow_terrain_following_steps_never_bind(self) -> None:
        """The demo route's ~1 m steps over 25 m legs: horizontal always wins."""
        pts = [(25.0 * i, 0.0) for i in range(50)]
        amsl = [100.0 + (i % 2) for i in range(50)]
        rep = assess_endurance(pts, amsl, _model())
        assert rep.vertical_bound_time_s == 0.0

    def test_vertical_travel_counts_both_directions(self) -> None:
        pts = [(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)]
        amsl = [100.0, 130.0, 100.0]
        rep = assess_endurance(pts, amsl, _model())
        assert rep.vertical_travel_m == pytest.approx(60.0, rel=1e-9)


class TestBudgetVerdicts:
    def test_the_demo_route_closes_at_planned_cruise(self) -> None:
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model(cruise_speed_ms=6.0))
        assert rep.closes
        assert not rep.exceeds_raw_endurance
        assert rep.energy_fraction < 1.0

    def test_the_forest_capped_speed_breaks_the_reserve(self) -> None:
        """3.5 m/s is B2's measured cap at 107 stems/ha — the density actually flown."""
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model(cruise_speed_ms=3.5))
        assert not rep.closes
        assert rep.energy_fraction > 1.0

    def test_real_forest_density_outruns_the_whole_battery(self) -> None:
        """1.6-2.5 m/s is the cap at 400-1000 stems/ha. Not a reserve question."""
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        for cruise in (2.5, 1.6):
            rep = assess_endurance(pts, amsl, _model(cruise_speed_ms=cruise))
            assert rep.exceeds_raw_endurance, cruise

    def test_reserve_and_raw_failures_are_distinguishable(self) -> None:
        """Eating the reserve is a command decision; outrunning the battery is not."""
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model(cruise_speed_ms=3.5))
        assert not rep.closes
        assert not rep.exceeds_raw_endurance


class TestMinSustainableSpeed:
    """The number B2's forest ceiling has to be compared against."""

    def test_flying_at_the_floor_exactly_spends_the_budget(self) -> None:
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        floor = assess_endurance(pts, amsl, _model()).min_sustainable_speed_ms
        at_floor = assess_endurance(pts, amsl, _model(cruise_speed_ms=floor))
        assert at_floor.energy_fraction == pytest.approx(1.0, rel=0.02)

    def test_the_floor_is_independent_of_the_speed_it_was_computed_at(self) -> None:
        """It is a property of the route and the battery, not of the query."""
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        floors = [
            assess_endurance(pts, amsl, _model(cruise_speed_ms=v)).min_sustainable_speed_ms
            for v in (6.0, 5.0, 4.0)
        ]
        assert max(floors) - min(floors) < 0.05

    def test_the_demo_floor_sits_above_the_forest_ceiling(self) -> None:
        """N1's actual finding, pinned so a regression cannot quietly reopen it.

        The endurance floor exceeds B2's measured cap at 107 stems/ha (3.5) and
        at real managed-forest density (1.6-2.5), so there is **no feasible
        cruise speed** for below-canopy flight through real forest on a 30 min
        airframe. Only the sparsest arm (50 stems/ha, 5.4 m/s) leaves a window
        open.

        This route is level, so it pays no climb surcharge and floors at
        3.85 m/s; the real demo route, which climbs, floors at **4.00 m/s**. The
        conclusion is the same either way, which is the point of asserting the
        inequalities rather than the number.
        """
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        floor = assess_endurance(pts, amsl, _model()).min_sustainable_speed_ms
        assert 3.8 < floor < 4.1
        assert floor > 3.5  # 107 stems/ha — flown in B2
        assert floor > 2.5  # 400 stems/ha
        assert floor < 5.4  # 50 stems/ha — the one density that still works


class TestUnstatedEndurance:
    """No endurance given: report what the route demands, never invent a verdict."""

    def test_no_endurance_reports_requirement_without_judging(self) -> None:
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model(endurance_s=None))
        assert not rep.stated
        assert not rep.closes
        assert not rep.exceeds_raw_endurance
        assert math.isinf(rep.energy_fraction)
        assert rep.required_endurance_s > rep.flight_time_s

    def test_required_endurance_includes_the_reserve(self) -> None:
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model(endurance_s=None, reserve_fraction=0.25))
        assert rep.required_endurance_s == pytest.approx(rep.energy_time_s / 0.75, rel=1e-9)


class TestEnergy:
    def test_climbing_costs_more_than_hovering(self) -> None:
        model = _model()
        assert model.climb_power_ratio(0.0) == 1.0
        assert model.climb_power_ratio(3.0) > 1.0
        assert model.climb_power_ratio(6.0) > model.climb_power_ratio(3.0)

    def test_energy_time_exceeds_flight_time_only_when_climbing(self) -> None:
        level_pts, level_amsl = _straight(50, 1000.0)
        level = assess_endurance(level_pts, level_amsl, _model())
        assert level.energy_time_s == pytest.approx(level.flight_time_s, rel=1e-9)

        climb_amsl = [100.0 + 2.0 * i for i in range(50)]
        climbing = assess_endurance(level_pts, climb_amsl, _model())
        assert climbing.energy_time_s > climbing.flight_time_s

    def test_overshoot_factor_scales_the_climb_charge(self) -> None:
        """PX4 flew 3.6x the vertical travel B1's plan commanded; that costs energy."""
        pts, _ = _straight(50, 1000.0)
        amsl = [100.0 + 2.0 * i for i in range(50)]
        plain = assess_endurance(pts, amsl, _model(vertical_overshoot=1.0))
        overshoot = assess_endurance(pts, amsl, _model(vertical_overshoot=3.63))
        assert overshoot.energy_time_s > plain.energy_time_s


class TestCorners:
    def test_shallow_planner_routes_report_no_sharp_corners(self) -> None:
        """The demo route's sharpest vertex is 6.7 deg over 282 waypoints."""
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        assert assess_endurance(pts, amsl, _model()).sharp_corners == 0

    def test_a_right_angle_is_flagged_as_uncharged_time(self) -> None:
        """Where PX4's own corner deceleration would bite, say so rather than
        silently under-estimating — the model deliberately omits that term."""
        pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]
        amsl = [100.0] * 3
        assert assess_endurance(pts, amsl, _model()).sharp_corners == 1


class TestValidation:
    def test_mismatched_inputs_are_rejected(self) -> None:
        with pytest.raises(ValueError, match="same length"):
            assess_endurance([(0.0, 0.0)], [1.0, 2.0], _model())

    @pytest.mark.parametrize(
        "kw",
        [
            {"cruise_speed_ms": 0.0},
            {"climb_rate_ms": 0.0},
            {"descent_rate_ms": -1.0},
            {"reserve_fraction": 1.0},
            {"endurance_s": 0.0},
        ],
    )
    def test_degenerate_models_are_rejected(self, kw: dict) -> None:
        with pytest.raises(ValueError):
            _model(**kw)

    def test_a_single_vertex_costs_nothing(self) -> None:
        rep = assess_endurance([(0.0, 0.0)], [100.0], _model())
        assert rep.flight_time_s == pytest.approx(B1_WAYPOINT_OVERHEAD_S, rel=1e-9)
        assert rep.terminal_overhead_s == 0.0


class TestForestSpeedCaps:
    """Per-vertex stem-spacing caps (TEST-PLAN N3) feeding the budget.

    Scoring a forest route at a cruise speed the trees forbid is the same class of
    error as scoring clearance under a descent profile PX4 does not fly.
    """

    def test_uncapped_route_is_unchanged_by_passing_cruise_as_the_limit(self) -> None:
        pts, amsl = _straight(50, 2000.0)
        plain = assess_endurance(pts, amsl, _model())
        capped = assess_endurance(pts, amsl, _model(), speed_limits_ms=[6.0] * 50)
        assert capped.flight_time_s == pytest.approx(plain.flight_time_s, rel=1e-9)
        assert capped.speed_capped_legs == 0
        assert capped.effective_speed_ms == pytest.approx(6.0, rel=1e-6)

    def test_a_slow_stand_stretches_flight_time_and_is_counted(self) -> None:
        """10 slow vertices cap **11** legs: a slow vertex slows both legs touching
        it, which is the "slower of its endpoints" rule seen from the other side."""
        pts, amsl = _straight(50, 2000.0)
        limits = [6.0] * 50
        limits[20:30] = [1.5] * 10
        rep = assess_endurance(pts, amsl, _model(), speed_limits_ms=limits)
        assert rep.speed_capped_legs == 11
        assert rep.effective_speed_ms < 6.0
        assert rep.flight_time_s > assess_endurance(pts, amsl, _model()).flight_time_s

    def test_a_leg_is_charged_at_the_slower_of_its_endpoints(self) -> None:
        """The aircraft must already have slowed on arrival, not at the vertex."""
        pts = [(0.0, 0.0), (600.0, 0.0)]
        amsl = [100.0, 100.0]
        rep = assess_endurance(pts, amsl, _model(), speed_limits_ms=[6.0, 2.0])
        assert rep.cruise_time_s == pytest.approx(300.0, rel=1e-9)

    def test_the_demo_concealed_route_does_not_close_once_stems_are_priced(self) -> None:
        """N1 + N3 together: 1.58 m/s effective over 5.3 km is 57 min, not 16."""
        pts, amsl = _straight(B1_WAYPOINTS, B1_LENGTH_M)
        rep = assess_endurance(pts, amsl, _model(), speed_limits_ms=[1.58] * B1_WAYPOINTS)
        assert rep.exceeds_raw_endurance
        assert rep.flight_time_s / 60.0 > 50.0

    def test_mismatched_limit_length_is_rejected(self) -> None:
        pts, amsl = _straight(10, 100.0)
        with pytest.raises(ValueError, match="speed_limits_ms"):
            assess_endurance(pts, amsl, _model(), speed_limits_ms=[6.0] * 9)
