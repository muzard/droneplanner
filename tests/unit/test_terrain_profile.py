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
    _settle_margin_m,
    assess_clearance,
    densify_unsafe_descents,
    sample_flown_profile,
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


class TestSampleFlownProfile:
    """The sampler both the clearance check and the concealment metric consume."""

    def test_samples_span_the_route_without_duplicating_shared_vertices(self) -> None:
        pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]
        amsl = [50.0, 50.0, 50.0]
        samples = sample_flown_profile(pts, amsl, DEMO_PROFILE, samples_per_leg=4)
        assert len(samples) == 4 * 2 + 1  # not 2 * (4 + 1): shared vertices appear once
        assert samples[0] == (0.0, 0.0, 50.0)
        assert samples[-1] == (100.0, 100.0, 50.0)

    def test_altitudes_are_the_front_loaded_ones_not_interpolated(self) -> None:
        """Mid-leg on a descent the aircraft is already down — that is the point."""
        pts = [(0.0, 0.0), (900.0, 0.0)]
        amsl = [130.0, 70.0]
        mid = sample_flown_profile(pts, amsl, DEMO_PROFILE, samples_per_leg=2)[1]
        assert mid[0] == pytest.approx(450.0)
        assert mid[2] == pytest.approx(70.0)  # 0.333 gradient: down after 180 m
        assert mid[2] < 100.0  # linear interpolation would say 100.0

    def test_degenerate_routes_are_returned_as_is(self) -> None:
        assert sample_flown_profile([(1.0, 2.0)], [7.0], DEMO_PROFILE) == [(1.0, 2.0, 7.0)]
        with pytest.raises(ValueError, match="same length"):
            sample_flown_profile([(0.0, 0.0), (1.0, 1.0)], [0.0], DEMO_PROFILE)
        with pytest.raises(ValueError, match="samples_per_leg"):
            sample_flown_profile(
                [(0.0, 0.0), (1.0, 1.0)], [0.0, 1.0], DEMO_PROFILE, samples_per_leg=1
            )


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


class TestDensifyUnsafeDescents:
    """The mechanism. Every test here is a lesson paid for in SITL."""

    # A 900 m leg descending 60 m over ground that holds high then falls away —
    # the shape that put the aircraft in the ground on 2026-07-24.
    FAILING_LEG = ([(0.0, 0.0), (900.0, 0.0)], [130.0, 70.0])

    @staticmethod
    def _falling_slope(x: float, _y: float) -> float:
        return 100.0 if x < 450.0 else 100.0 - (x - 450.0) * (60.0 / 450.0)

    @classmethod
    def _terrain_following(cls, x: float, y: float) -> float:
        return cls._falling_slope(x, y) + 30.0

    def _densify(self, pts: list[tuple[float, float]], amsl: list[float], **kw: object):
        return densify_unsafe_descents(
            pts,
            amsl,
            predicted_amsl=self._terrain_following,
            ground_at=self._falling_slope,
            profile=DEMO_PROFILE,
            margin_agl_m=5.0,
            **kw,  # type: ignore[arg-type]
        )

    def test_splits_a_leg_the_clearance_check_rejects(self) -> None:
        pts, inserted = self._densify(*self.FAILING_LEG)
        assert inserted >= 1
        assert len(pts) == 2 + inserted

    def test_the_split_actually_restores_clearance(self) -> None:
        """Splitting is pointless unless the guarantee then passes."""
        pts, _ = self._densify(*self.FAILING_LEG)
        amsl = [self._terrain_following(x, y) for x, y in pts]
        amsl[0], amsl[-1] = self.FAILING_LEG[1]
        rep = assess_clearance(pts, amsl, self._falling_slope, DEMO_PROFILE, margin_agl_m=5.0)
        assert rep.clears
        assert rep.min_clearance_agl_m > 5.0

    def test_does_NOT_split_a_leg_that_already_clears(self) -> None:
        """THE regression test for the 2026-07-27 SITL failure.

        The first version split on ``drop > threshold`` regardless of whether the
        leg was safe. On the demo AO that inserted a vertex into a leg already
        clear at +9.0 m AGL; PX4 missed it by 1.58 m vertically (> the 0.8 m
        ``NAV_MC_ALT_RAD``), never marked it reached, and the mission froze at
        11/79 while the trajectory generator slid past it into the ground.
        A big drop is not by itself a reason to insert anything.
        """
        # 60 m of descent over flat, low ground: a huge drop that is never unsafe.
        flat = lambda _x, _y: 0.0  # noqa: E731
        pts, inserted = densify_unsafe_descents(
            [(0.0, 0.0), (900.0, 0.0)],
            [130.0, 70.0],
            predicted_amsl=lambda _x, _y: 30.0,
            ground_at=flat,
            profile=DEMO_PROFILE,
            margin_agl_m=5.0,
        )
        assert inserted == 0
        assert pts == [(0.0, 0.0), (900.0, 0.0)]

    def test_inserts_as_few_vertices_as_possible(self) -> None:
        """Every inserted vertex is a new chance for PX4 to refuse to sequence."""
        _, inserted = self._densify(*self.FAILING_LEG)
        assert inserted == 1

    def test_placement_maximises_the_level_run_before_arrival(self) -> None:
        """Placement is scored by settle margin, not taken as the midpoint.

        Measured 2026-07-24 over 51 accepted arrivals: PX4 holds altitude to
        0.35 m mean / 0.64 m p95 when it reaches a waypoint level. The vertex it
        missed by 1.58 m had only ~84 m of level run.
        """
        pts, _ = self._densify(*self.FAILING_LEG)
        amsl = [self._terrain_following(x, y) for x, y in pts]
        amsl[0], amsl[-1] = self.FAILING_LEG[1]
        runs = [
            _settle_margin_m(math.dist(pts[k], pts[k + 1]), amsl[k] - amsl[k + 1], DEMO_PROFILE)
            for k in range(len(pts) - 1)
        ]
        assert min(runs) > 84.0, "must beat the level run of the vertex PX4 refused"

    def test_never_splits_a_climb(self) -> None:
        """A front-loaded climb only gains clearance; the check assumes linear."""
        out, inserted = self._densify([(0.0, 0.0), (900.0, 0.0)], [70.0, 200.0])
        assert inserted == 0
        assert out == [(0.0, 0.0), (900.0, 0.0)]

    def test_zero_length_leg_is_not_split(self) -> None:
        out, inserted = self._densify([(5.0, 5.0), (5.0, 5.0)], [130.0, 70.0])
        assert inserted == 0
        assert out == [(5.0, 5.0), (5.0, 5.0)]

    def test_leaves_the_route_alone_when_nothing_fails(self) -> None:
        pts = [(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)]
        out, inserted = densify_unsafe_descents(
            pts,
            [130.0, 129.0, 128.0],
            predicted_amsl=lambda _x, _y: 130.0,
            ground_at=lambda _x, _y: 0.0,
            profile=DEMO_PROFILE,
            margin_agl_m=5.0,
        )
        assert inserted == 0
        assert out == pts

    def test_is_deterministic(self) -> None:
        a = self._densify(*self.FAILING_LEG)
        b = self._densify(*self.FAILING_LEG)
        assert a == b

    def test_rejects_bad_arguments(self) -> None:
        with pytest.raises(ValueError, match="same length"):
            self._densify([(0.0, 0.0)], [1.0, 2.0])
        with pytest.raises(ValueError, match="max_inserts_per_leg"):
            self._densify(*self.FAILING_LEG, max_inserts_per_leg=0)
