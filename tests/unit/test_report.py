"""Unit tests for the report builder and its GO/REVIEW/NO-GO logic (§5.4)."""

from __future__ import annotations

from droneplanner.io.report import report_to_json, report_to_markdown
from droneplanner.model.plan import (
    Crossing,
    CrossingMethod,
    FlyabilityClass,
    KinematicReport,
    Plan,
    RouteSegment,
    TrajectoryVertex,
)
from droneplanner.model.report import Recommendation, Severity
from droneplanner.planner.stub import straight_line_plan
from droneplanner.report.build import build_report
from tests.fixtures.synthetic import make_mission


def _analyzed(mission, plan):  # type: ignore[no-untyped-def]
    """build_report as if concealment + flyability analysis both ran."""
    return build_report(mission, plan, concealment_analyzed=True, flyability_analyzed=True)


def test_stub_report_is_review_not_go() -> None:
    # No analysis performed -> the stub can never certify concealment -> REVIEW.
    m = make_mission()
    plan = straight_line_plan(m)
    report = build_report(m, plan)
    assert report.recommendation is Recommendation.REVIEW
    assert "analysis_not_performed" in {f.code for f in report.findings}


def test_concealment_only_is_review_flyability_flag() -> None:
    # M1: concealment analyzed but forest flyability not -> REVIEW with its flag.
    m = make_mission()
    plan = straight_line_plan(m)
    report = build_report(m, plan, concealment_analyzed=True, flyability_analyzed=False)
    assert report.recommendation is Recommendation.REVIEW
    assert "flyability_not_analyzed" in {f.code for f in report.findings}


def test_fiber_insufficient_is_no_go() -> None:
    m = make_mission(spool={"length_m": 100.0, "mass_kg": 1.0})
    plan = straight_line_plan(m)  # ~1.28 km route >> 100 m spool
    report = _analyzed(m, plan)
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
    m = make_mission()
    seg = RouteSegment(
        index=0,
        start=m.start,
        end=m.end,
        length_m=50.0,
        flyability=FlyabilityClass.NON_NAVIGABLE,
    )
    report = _analyzed(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.NO_GO
    assert report.non_navigable_count == 1


def test_marginal_segment_is_review() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.MARGINAL
    )
    report = _analyzed(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.REVIEW
    assert report.marginal_count == 1


def test_all_flyable_fully_analyzed_is_go() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    report = _analyzed(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.GO
    assert report.findings == ()


def test_exposed_cable_is_review() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0,
        start=m.start,
        end=m.end,
        length_m=50.0,
        flyability=FlyabilityClass.FLYABLE,
        exposed=True,
    )
    report = _analyzed(m, _plan_with((seg,)))
    assert report.recommendation is Recommendation.REVIEW
    assert report.exposed_length_m == 50.0


def test_manipulation_crossing_flagged_and_review() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    crossing = Crossing(
        location=m.start, method=CrossingMethod.OVERHEAD_SPAN, needs_manipulation=True
    )
    report = _analyzed(m, _plan_with((seg,), crossings=(crossing,)))
    assert report.recommendation is Recommendation.REVIEW
    assert report.manipulation_crossings == 1


def test_open_crossing_is_review() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    crossing = Crossing(location=m.start, method=CrossingMethod.OPEN)
    report = _analyzed(m, _plan_with((seg,), crossings=(crossing,)))
    assert report.recommendation is Recommendation.REVIEW
    assert any(f.code == "open_crossings" for f in report.findings)


def _kin(**kw: object) -> KinematicReport:
    base = dict(feasible=True, min_turn_radius_m=8.0, tightest_turn_radius_m=8.0)
    base.update(kw)
    return KinematicReport(**base)  # type: ignore[arg-type]


def test_reversal_is_informational_not_no_go() -> None:
    # Reversals are allowed at low payout speed: surfaced, but not a blocker.
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    plan = _plan_with((seg,), kinematics=_kin(feasible=True, reversals=2))
    report = build_report(m, plan, concealment_analyzed=True, flyability_analyzed=True)
    assert report.recommendation is Recommendation.GO
    reversal = next(f for f in report.findings if f.code == "tether_reversal")
    assert reversal.severity is Severity.INFO
    assert report.kinematics is not None and report.kinematics.reversals == 2


def test_turn_and_climb_violations_are_review() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    plan = _plan_with(
        (seg,),
        kinematics=_kin(
            feasible=False, turn_violations=2, tightest_turn_radius_m=3.0, climb_violations=1
        ),
    )
    report = build_report(m, plan, concealment_analyzed=True, flyability_analyzed=True)
    assert report.recommendation is Recommendation.REVIEW
    codes = {f.code for f in report.findings}
    assert "turn_radius" in codes
    assert "climb_rate" in codes


def test_feasible_kinematics_no_findings() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    plan = _plan_with((seg,), kinematics=_kin(feasible=True))
    report = build_report(m, plan, concealment_analyzed=True, flyability_analyzed=True)
    assert report.recommendation is Recommendation.GO


def test_report_markdown_shows_kinematics() -> None:
    m = make_mission()
    seg = RouteSegment(
        index=0, start=m.start, end=m.end, length_m=50.0, flyability=FlyabilityClass.FLYABLE
    )
    report = build_report(
        m,
        _plan_with((seg,), kinematics=_kin()),
        concealment_analyzed=True,
        flyability_analyzed=True,
    )
    assert "Kinematic feasibility" in report_to_markdown(report)


def test_report_serialization_roundtrips() -> None:
    m = make_mission()
    report = build_report(m, straight_line_plan(m))
    js = report_to_json(report)
    assert '"recommendation": "REVIEW"' in js
    md = report_to_markdown(report)
    assert "Recommendation: REVIEW" in md
    assert "# Flight plan risk report" in md
    assert "## Findings" in md
