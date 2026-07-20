"""Unit tests for the M0 straight-line stub planner (Software-Spec §7)."""

from __future__ import annotations

import pytest

from droneplanner.planner.stub import straight_line_plan
from tests.fixtures.synthetic import make_mission


def test_stub_endpoints_match_mission() -> None:
    m = make_mission()
    plan = straight_line_plan(m)
    assert plan.start == m.start
    assert plan.end == m.end


def test_stub_length_equals_straight_distance() -> None:
    m = make_mission()
    plan = straight_line_plan(m)
    assert plan.total_length_m == pytest.approx(m.start.distance_to(m.end))


def test_stub_vertices_sampled_at_spacing() -> None:
    m = make_mission(waypoint_spacing_m=100.0)
    plan = straight_line_plan(m)
    # start->end is ~1281 m -> ceil(1281/100)=13 intervals -> 14 vertices.
    assert len(plan.vertices) == 14
    assert len(plan.segments) == 13


def test_stub_segments_are_contiguous() -> None:
    m = make_mission()
    plan = straight_line_plan(m)
    for a, b in zip(plan.segments, plan.segments[1:], strict=False):
        assert a.end == b.start


def test_stub_fiber_uses_slack_factor() -> None:
    m = make_mission(fiber_slack_factor=1.1)
    plan = straight_line_plan(m)
    assert plan.fiber_used_m == pytest.approx(plan.total_length_m * 1.1)


def test_stub_altitude_is_cruise_agl() -> None:
    m = make_mission(cruise_agl_m=42.0)
    plan = straight_line_plan(m)
    assert all(v.altitude_agl_m == 42.0 for v in plan.vertices)
    assert all(v.z == 42.0 for v in plan.vertices)


def test_stub_is_deterministic() -> None:
    m = make_mission()
    assert straight_line_plan(m).model_dump() == straight_line_plan(m).model_dump()


def test_stub_route_index_recorded() -> None:
    m = make_mission()
    assert straight_line_plan(m, route_index=2).route_index == 2


def test_stub_short_hop_has_single_segment() -> None:
    m = make_mission(
        start={"x": 0.0, "y": 0.0},
        end={"x": 10.0, "y": 0.0},
        crs="EPSG:32633",
        waypoint_spacing_m=100.0,
    )
    plan = straight_line_plan(m)
    assert len(plan.segments) == 1
    assert len(plan.vertices) == 2
