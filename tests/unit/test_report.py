"""Unit tests for the report builder and its GO/REVIEW/NO-GO logic (§5.4)."""

from __future__ import annotations

from droneplanner.io.report import report_to_json, report_to_markdown
from droneplanner.model.plan import (
    Crossing,
    CrossingMethod,
    FlyabilityClass,
    Plan,
    RouteSegment,
    TrajectoryVertex,
)
from droneplanner.model.report import Recommendation, Severity
from droneplanner.planner.stub import straight_line_plan
from droneplanner.report.build import build_report
from tests.fixtures.synthetic import make_mission


def test_stub_report_is_review_not_go() -> None:
    # No analysis layers -> the stub can never certify concealment -> REVIEW.
    m = make_mission()
    plan = straight_line_plan(m)
    report = build_report(m, plan)
    assert report.recommendation is Recommendation.REVIEW
    codes = {f.code for f in report.findings}
    assert "analysis_not_performed" in codes


def test_fiber_insufficient_is_no_go() -> None:
    m = make_mission(spool={"length_m": 100.0, "mass_kg": 1.0})
    plan = straight_line_plan(m)  # ~1.28 km route >> 100 m spool
    report = build_report(m, plan)
    assert report.recommendation is Recommendation.NO_GO
    assert any(f.code == "fiber_insufficient" for f in report.findings)
    assert any(f.severity is Severity.CRITICAL for f in report.findings)


def _plan_with(segments: tuple[RouteSegment, ...], **overrides: object) -> Plan:
    v = TrajectoryVertex(x=0, y=0, z=30, altitude_agl_m=30, speed_hint_ms=5)
    base = dict(
        mission_name="t",
        crs="EPSG:32633",
        seed=0,
        vertices=(v, v),
        segments=segments,
        total_length_m=sum(s.length_m for s in segments),
        fiber_used_m=sum(s.length_m for s in segments),
    )
    base.update(overrides)
    return Plan(**base)  # type: ignore[arg-type]


def test_non_navigable_segment_is_no_go() -> None:
    # Give the mission the analysis layers so the only finding is the hard one.
    m = make_mission(layers={"dsm": "x.tif", "dtm": "y.tif", "point_cloud": "z.laz"})
    seg = RouteSegment(
        index=0,
        start=m.start,
        end=m.end,
        length_m=50.0,
        flyability=FlyabilityClass.NON_NAVIGABLE,
    )
    report = build_report(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.NO_GO
    assert report.non_navigable_count == 1


def test_marginal_segment_is_review() -> None:
    m = make_mission(layers={"dsm": "x.tif", "dtm": "y.tif", "point_cloud": "z.laz"})
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.MARGINAL
    )
    report = build_report(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.REVIEW
    assert report.marginal_count == 1


def test_all_flyable_with_layers_is_go() -> None:
    m = make_mission(layers={"dsm": "x.tif", "dtm": "y.tif", "point_cloud": "z.laz"})
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    report = build_report(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.GO
    assert report.findings == ()


def test_exposed_cable_is_review() -> None:
    m = make_mission(layers={"dsm": "x.tif", "dtm": "y.tif", "point_cloud": "z.laz"})
    seg = RouteSegment(
        index=0,
        start=m.start,
        end=m.end,
        length_m=50.0,
        flyability=FlyabilityClass.FLYABLE,
        exposed=True,
    )
    report = build_report(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.REVIEW
    assert report.exposed_length_m == 50.0


def test_manipulation_crossing_flagged_and_review() -> None:
    m = make_mission(layers={"dsm": "x.tif", "dtm": "y.tif", "point_cloud": "z.laz"})
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    crossing = Crossing(
        location=m.start,
        method=CrossingMethod.OVERHEAD_SPAN,
        needs_manipulation=True,
    )
    report = build_report(m, _plan_with((seg,), crossings=(crossing,)))
    assert report.recommendation is Recommendation.REVIEW
    assert report.manipulation_crossings == 1


def test_open_crossing_is_review() -> None:
    m = make_mission(layers={"dsm": "x.tif", "dtm": "y.tif", "point_cloud": "z.laz"})
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    crossing = Crossing(location=m.start, method=CrossingMethod.OPEN)
    report = build_report(m, _plan_with((seg,), crossings=(crossing,)))
    assert report.recommendation is Recommendation.REVIEW
    assert any(f.code == "open_crossings" for f in report.findings)


def test_report_serialization_roundtrips() -> None:
    m = make_mission()
    report = build_report(m, straight_line_plan(m))
    js = report_to_json(report)
    assert '"recommendation": "REVIEW"' in js
    md = report_to_markdown(report)
    assert "Recommendation: REVIEW" in md
    assert "# Flight plan risk report" in md
    assert "## Findings" in md
