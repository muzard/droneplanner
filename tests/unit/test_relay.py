"""Unit tests for relay-leg splitting (TEST-PLAN N1 way out #1, planner/relay.py)."""

from __future__ import annotations

import math
from itertools import pairwise

import pytest

from droneplanner.planner.endurance import EnduranceModel, assess_endurance
from droneplanner.planner.relay import (
    balance_legs,
    legs_required_for_endurance,
    split_into_legs,
)


def _model(endurance_s: float | None = 1800.0, cruise: float = 6.0) -> EnduranceModel:
    return EnduranceModel(
        cruise_speed_ms=cruise,
        climb_rate_ms=3.0,
        descent_rate_ms=2.0,
        endurance_s=endurance_s,
        reserve_fraction=0.2,
    )


def _straight(n: int, spacing: float = 25.0) -> tuple[list[tuple[float, float]], list[float]]:
    """A level straight route of ``n`` vertices — no climb, no corners."""
    pts = [(i * spacing, 0.0) for i in range(n)]
    return pts, [100.0] * n


def test_a_route_that_fits_is_one_leg_with_no_relays() -> None:
    pts, amsl = _straight(20)  # 475 m — trivially inside a 30 min budget
    plan = split_into_legs(pts, amsl, _model())
    assert plan.feasible
    assert plan.leg_count == 1
    assert plan.relay_indices == []
    assert plan.legs[0].start_index == 0
    assert plan.legs[0].end_index == len(pts) - 1


def test_a_long_route_splits_and_every_leg_closes() -> None:
    pts, amsl = _straight(600)  # ~15 km at 6 m/s: well over one battery
    model = _model()
    whole = assess_endurance(pts, amsl, model)
    assert not whole.closes, "fixture must not fit one battery or it tests nothing"

    plan = split_into_legs(pts, amsl, model)
    assert plan.feasible
    assert plan.leg_count > 1
    assert all(leg.closes for leg in plan.legs)


def test_legs_tile_the_route_sharing_the_relay_vertex() -> None:
    """A relay point is flown *to* by one leg and *from* by the next, not skipped."""
    pts, amsl = _straight(600)
    plan = split_into_legs(pts, amsl, _model())
    assert plan.legs[0].start_index == 0
    assert plan.legs[-1].end_index == len(pts) - 1
    for prev, nxt in pairwise(plan.legs):
        assert prev.end_index == nxt.start_index
    assert plan.relay_indices == [leg.end_index for leg in plan.legs[:-1]]


def test_greedy_split_is_minimal() -> None:
    """No smaller leg count exists — one fewer must leave some leg over budget."""
    pts, amsl = _straight(600)
    model = _model()
    plan = split_into_legs(pts, amsl, model)
    with pytest.raises(ValueError, match="cannot cover this route"):
        balance_legs(pts, amsl, model, leg_count=plan.leg_count - 1)


def test_a_gap_no_battery_can_cross_is_reported_not_truncated() -> None:
    """Two vertices 500 km apart cannot be relayed — say so, don't return a stub."""
    pts = [(0.0, 0.0), (10.0, 0.0), (500_000.0, 0.0)]
    amsl = [100.0, 100.0, 100.0]
    plan = split_into_legs(pts, amsl, _model())
    assert not plan.feasible
    assert plan.blocking_index == 1


def test_unstated_endurance_cannot_be_split() -> None:
    """With no endurance in the mission there is no budget to split against."""
    pts, amsl = _straight(20)
    plan = split_into_legs(pts, amsl, _model(endurance_s=None))
    assert not plan.feasible
    assert plan.blocking_index == 0


def test_balancing_keeps_the_leg_count_and_buys_back_the_margin() -> None:
    """Greedy maxes every leg; balancing spreads the same route over the same legs."""
    pts, amsl = _straight(600)
    model = _model()
    greedy = split_into_legs(pts, amsl, model)
    fair = balance_legs(pts, amsl, model)

    assert fair.feasible
    assert fair.leg_count == greedy.leg_count, "balancing must not cost a ground stop"
    assert fair.worst_energy_fraction < greedy.worst_energy_fraction
    # Greedy lands its non-final legs on the limit by construction.
    assert greedy.worst_energy_fraction == pytest.approx(1.0, abs=0.02)
    # Balanced legs are within a few percent of each other.
    fractions = [leg.endurance.energy_fraction for leg in fair.legs]
    assert max(fractions) - min(fractions) < 0.1


def test_more_legs_than_the_minimum_are_allowed_and_lighten_each_one() -> None:
    pts, amsl = _straight(600)
    model = _model()
    minimum = split_into_legs(pts, amsl, model).leg_count
    roomier = balance_legs(pts, amsl, model, leg_count=minimum + 2)
    assert roomier.leg_count <= minimum + 2
    assert roomier.worst_energy_fraction < balance_legs(pts, amsl, model).worst_energy_fraction


def test_a_bigger_battery_buys_fewer_relays() -> None:
    pts, amsl = _straight(600)
    model = _model()
    counts = [
        legs_required_for_endurance(pts, amsl, model, minutes * 60.0).leg_count
        for minutes in (30.0, 60.0, 120.0)
    ]
    assert counts == sorted(counts, reverse=True)
    assert counts[0] > counts[-1]


def test_stem_speed_caps_shorten_the_legs_they_apply_to() -> None:
    """The forest's speed cap is what makes below-canopy legs short — it must bind."""
    pts, amsl = _straight(600)
    model = _model()
    fast = split_into_legs(pts, amsl, model)
    slow = split_into_legs(pts, amsl, model, speed_limits_ms=[1.5] * len(pts))
    assert slow.leg_count > fast.leg_count
    assert all(leg.endurance.speed_capped_legs > 0 for leg in slow.legs)


def test_leg_length_matches_the_geometry_it_covers() -> None:
    pts, amsl = _straight(600)
    plan = split_into_legs(pts, amsl, _model())
    for leg in plan.legs:
        expected = sum(math.dist(pts[i], pts[i + 1]) for i in range(leg.start_index, leg.end_index))
        assert leg.length_m == pytest.approx(expected)
    total = sum(leg.length_m for leg in plan.legs)
    assert total == pytest.approx(sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)))


def test_mismatched_input_lengths_are_rejected() -> None:
    pts, amsl = _straight(10)
    with pytest.raises(ValueError, match="same length"):
        split_into_legs(pts, amsl[:-1], _model())
    with pytest.raises(ValueError, match="same length"):
        split_into_legs(pts, amsl, _model(), speed_limits_ms=[3.0] * 3)
