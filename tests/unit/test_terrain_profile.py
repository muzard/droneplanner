"""Unit tests for the flown-descent clearance model (SHORTFALLS item 0).

The defect these guard against is a real CFIT: on 2026-07-24 a plan that was
clear at every vertex *and* along every linearly-interpolated leg flew into the
ground at 0.8 m AGL, because PX4 front-loads a leg's descent instead of spreading
it along the ground track.
"""

from __future__ import annotations

import math

import pytest

from droneplanner.planner.terrain_profile import (
    FlownProfile,
    assess_clearance,
    densify_descending_legs,
)

# The airframe in configs/mission.example.yaml: 2 m/s descent at 6 m/s cruise.
DEMO_PROFILE = FlownProfile.from_rates(descent_rate_ms=2.0, cruise_speed_ms=6.0)


class TestFlownProfile:
    def test_descent_is_front_loaded_not_linear(self) -> None:
        """The measured SITL case: leg wp50->wp51, 765 m, -40.5 m."""
        length, start, end = 765.0, 100.0, 59.5
        at_17pct = DEMO_PROFILE.altitude_amsl_at(
            length * 0.17, length_m=length, start_amsl=start, end_amsl=end
        )
        linear = start + (end - start) * 0.17
        # Front-loaded: 0.17*765 = 130 m along, at 1/3 m down per m = 43.3 m of
        # drop available, which is the whole 40.5 m -> already at target.
        assert at_17pct == pytest.approx(end)
        assert linear == pytest.approx(93.1, abs=0.1)
        assert at_17pct < linear - 30.0, "front-loaded descent must be far below linear"

    def test_descent_never_undershoots_the_target(self) -> None:
        alt = DEMO_PROFILE.altitude_amsl_at(700.0, length_m=765.0, start_amsl=100.0, end_amsl=59.5)
        assert alt == pytest.approx(59.5)

    def test_climb_is_linear_conservative_not_front_loaded(self) -> None:
        """Prompt climbing only adds clearance, so assuming it would be optimistic."""
        alt = DEMO_PROFILE.altitude_amsl_at(100.0, length_m=1000.0, start_amsl=50.0, end_amsl=150.0)
        assert alt == pytest.approx(60.0)

    def test_gradient_is_rate_over_speed(self) -> None:
        assert DEMO_PROFILE.descent_gradient == pytest.approx(2.0 / 6.0)

    def test_zero_length_leg_returns_the_target(self) -> None:
        assert DEMO_PROFILE.altitude_amsl_at(0.0, length_m=0.0, start_amsl=9.0, end_amsl=4.0) == 4.0

    def test_along_distance_is_clamped_to_the_leg(self) -> None:
        far = DEMO_PROFILE.altitude_amsl_at(1e9, length_m=100.0, start_amsl=50.0, end_amsl=100.0)
        assert far == pytest.approx(100.0)

    def test_rejects_nonpositive_cruise_speed(self) -> None:
        with pytest.raises(ValueError, match="cruise_speed_ms"):
            FlownProfile.from_rates(descent_rate_ms=2.0, cruise_speed_ms=0.0)


class TestAssessClearance:
    def test_catches_the_falling_slope_linear_interpolation_calls_safe(self) -> None:
        """THE regression test: clear at both vertices and clear under linear
        interpolation, but the flown profile puts it underground."""
        # A 900 m leg descending 60 m over ground that falls only at the far end:
        # terrain holds high for the first half, then drops away.
        pts = [(0.0, 0.0), (900.0, 0.0)]
        amsl = [130.0, 70.0]  # 30 m AGL at each end

        def ground(x: float, _y: float) -> float:
            return 100.0 if x < 450.0 else 100.0 - (x - 450.0) * (60.0 / 450.0)

        flown = assess_clearance(pts, amsl, ground, DEMO_PROFILE, margin_agl_m=5.0)
        linear = assess_clearance(
            pts, amsl, ground, FlownProfile(descent_gradient=0.0), margin_agl_m=5.0
        )

        assert linear.clears, "linear interpolation must consider this leg safe"
        assert linear.min_clearance_agl_m > 5.0
        assert not flown.clears, "the flown profile must catch it"
        assert flown.min_clearance_agl_m < 5.0
        assert flown.violations == 1
        assert flown.worst_leg_index == 0

    def test_level_flight_over_flat_ground_clears(self) -> None:
        pts = [(0.0, 0.0), (500.0, 0.0), (1000.0, 0.0)]
        amsl = [30.0, 30.0, 30.0]
        rep = assess_clearance(pts, amsl, lambda _x, _y: 0.0, DEMO_PROFILE, margin_agl_m=5.0)
        assert rep.clears
        assert rep.min_clearance_agl_m == pytest.approx(30.0)

    def test_reports_the_worst_leg(self) -> None:
        pts = [(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)]
        amsl = [30.0, 30.0, 30.0]

        def ground(x: float, _y: float) -> float:
            return 20.0 if x > 100.0 else 0.0

        rep = assess_clearance(pts, amsl, ground, DEMO_PROFILE, margin_agl_m=5.0)
        assert rep.worst_leg_index == 1
        assert rep.min_clearance_agl_m == pytest.approx(10.0)

    def test_margin_is_carried_on_the_report(self) -> None:
        rep = assess_clearance(
            [(0.0, 0.0), (10.0, 0.0)], [30.0, 30.0], lambda *_: 0.0, DEMO_PROFILE, margin_agl_m=7.5
        )
        assert rep.margin_agl_m == 7.5

    def test_degenerate_routes(self) -> None:
        rep = assess_clearance([(0.0, 0.0)], [30.0], lambda *_: 0.0, DEMO_PROFILE, margin_agl_m=5.0)
        assert rep.clears
        assert rep.min_clearance_agl_m == math.inf
        assert rep.worst_leg_index == -1

    def test_rejects_mismatched_inputs(self) -> None:
        with pytest.raises(ValueError, match="same length"):
            assess_clearance([(0.0, 0.0)], [1.0, 2.0], lambda *_: 0.0, DEMO_PROFILE, margin_agl_m=5)

    def test_rejects_too_coarse_sampling(self) -> None:
        with pytest.raises(ValueError, match="samples_per_leg"):
            assess_clearance(
                [(0.0, 0.0), (1.0, 0.0)],
                [1.0, 1.0],
                lambda *_: 0.0,
                DEMO_PROFILE,
                margin_agl_m=5.0,
                samples_per_leg=1,
            )


class TestDensifyDescendingLegs:
    def test_splits_a_big_drop_into_capped_sub_legs(self) -> None:
        pts, inserted = densify_descending_legs(
            [(0.0, 0.0), (900.0, 0.0)], [130.0, 70.0], max_drop_m=25.0
        )
        # 60 m drop / 25 m cap -> 3 sub-legs -> 2 inserted vertices.
        assert inserted == 2
        assert pts == [(0.0, 0.0), (300.0, 0.0), (600.0, 0.0), (900.0, 0.0)]

    def test_capped_sub_legs_restore_clearance_on_the_failing_case(self) -> None:
        """Densification must actually fix the leg the flown profile rejects."""
        amsl = [130.0, 70.0]

        def ground(x: float, _y: float) -> float:
            return 100.0 if x < 450.0 else 100.0 - (x - 450.0) * (60.0 / 450.0)

        pts, _ = densify_descending_legs([(0.0, 0.0), (900.0, 0.0)], amsl, max_drop_m=25.0)
        # Inserted vertices are terrain-following, as the planner's AGL bands make them.
        dense_amsl = [ground(x, y) + 30.0 for x, y in pts]
        dense_amsl[0], dense_amsl[-1] = amsl[0], amsl[-1]
        rep = assess_clearance(pts, dense_amsl, ground, DEMO_PROFILE, margin_agl_m=5.0)
        assert rep.clears
        assert rep.min_clearance_agl_m > 5.0

    def test_leaves_small_drops_and_level_legs_alone(self) -> None:
        pts = [(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)]
        out, inserted = densify_descending_legs(pts, [30.0, 20.0, 20.0], max_drop_m=25.0)
        assert inserted == 0
        assert out == pts

    def test_never_splits_a_climb(self) -> None:
        """A front-loaded climb only gains clearance; the check assumes linear."""
        out, inserted = densify_descending_legs(
            [(0.0, 0.0), (900.0, 0.0)], [70.0, 200.0], max_drop_m=25.0
        )
        assert inserted == 0
        assert out == [(0.0, 0.0), (900.0, 0.0)]

    def test_zero_length_leg_is_not_split(self) -> None:
        out, inserted = densify_descending_legs(
            [(5.0, 5.0), (5.0, 5.0)], [130.0, 70.0], max_drop_m=25.0
        )
        assert inserted == 0
        assert out == [(5.0, 5.0), (5.0, 5.0)]

    def test_is_deterministic(self) -> None:
        args = ([(0.0, 0.0), (900.0, 30.0)], [130.0, 40.0])
        a = densify_descending_legs(*args, max_drop_m=17.0)
        b = densify_descending_legs(*args, max_drop_m=17.0)
        assert a == b

    def test_rejects_bad_arguments(self) -> None:
        with pytest.raises(ValueError, match="max_drop_m"):
            densify_descending_legs([(0.0, 0.0)], [1.0], max_drop_m=0.0)
        with pytest.raises(ValueError, match="same length"):
            densify_descending_legs([(0.0, 0.0)], [1.0, 2.0], max_drop_m=5.0)
