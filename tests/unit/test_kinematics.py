"""Unit tests for the kinematic feasibility pass (Software-Spec §3.6)."""

from __future__ import annotations

import math

from droneplanner.model.mission import DroneSpec
from droneplanner.planner.kinematics import (
    KinematicLimits,
    fillet_path,
    smooth_altitudes,
)


def _limits(min_radius: float = 10.0, **kw: float) -> KinematicLimits:
    base = dict(
        min_turn_radius_m=min_radius,
        max_climb_rate_ms=3.0,
        max_descent_rate_ms=2.0,
        cruise_speed_ms=6.0,
    )
    base.update(kw)
    return KinematicLimits(**base)  # type: ignore[arg-type]


def _max_deflection(pts: list[tuple[float, float]]) -> float:
    worst = 0.0
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        u1 = (b[0] - a[0], b[1] - a[1])
        u2 = (c[0] - b[0], c[1] - b[1])
        l1, l2 = math.hypot(*u1), math.hypot(*u2)
        if l1 == 0 or l2 == 0:
            continue
        dot = (u1[0] * u2[0] + u1[1] * u2[1]) / (l1 * l2)
        worst = max(worst, math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
    return worst


def test_limits_from_drone() -> None:
    drone = DroneSpec(
        rotor_span_m=0.9,
        mass_kg=6.5,
        min_turn_radius_m=8.0,
        climb_rate_max_ms=3.0,
        descent_rate_max_ms=2.0,
        cruise_speed_ms=6.0,
    )
    limits = KinematicLimits.from_drone(drone)
    assert limits.min_turn_radius_m == 8.0
    assert limits.max_climb_rate_ms == 3.0


def test_fillet_straight_line_unchanged() -> None:
    pts = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)]
    out, tv, rev, tight = fillet_path(pts, _limits())
    assert tv == 0 and rev == 0
    assert out[0] == pts[0] and out[-1] == pts[-1]


def test_fillet_rounds_right_angle() -> None:
    pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]
    out, tv, rev, tight = fillet_path(pts, _limits(min_radius=10.0))
    assert tv == 0 and rev == 0
    assert tight == 10.0  # desired radius fit
    assert len(out) > len(pts)  # arc points inserted
    # The 90-degree corner is distributed into gentler turns.
    assert _max_deflection(out) < 45.0
    assert out[0] == pts[0] and out[-1] == pts[-1]


def test_fillet_flags_tight_corner_on_short_legs() -> None:
    pts = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]  # 5 m legs, 90-degree turn
    _out, tv, _rev, tight = fillet_path(pts, _limits(min_radius=10.0))
    assert tv == 1  # 10 m radius cannot fit on 5 m legs
    assert tight < 10.0


def test_fillet_flags_reversal() -> None:
    pts = [(0.0, 0.0), (100.0, 0.0), (0.0, 0.001)]  # ~180-degree turn
    _out, _tv, rev, _tight = fillet_path(pts, _limits())
    assert rev == 1


def test_fillet_skips_blocked_arc() -> None:
    pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]

    def blocked(x: float, y: float) -> bool:
        return 88.0 < x < 112.0 and -12.0 < y < 12.0  # covers the corner arc

    out, tv, _rev, _tight = fillet_path(pts, _limits(min_radius=10.0), blocked)
    assert tv >= 1
    assert (100.0, 0.0) in out  # corner kept sharp


def test_smooth_altitudes_constant_band() -> None:
    alts, viol, up, down = smooth_altitudes(
        [5.0, 5.0, 5.0], [0.0, 0.0, 0.0], [10.0, 10.0, 10.0], [100.0, 100.0], _limits()
    )
    assert alts == [5.0, 5.0, 5.0]
    assert viol == 0 and up == 0.0 and down == 0.0


def test_smooth_altitudes_follows_pref_when_feasible() -> None:
    alts, viol, up, _down = smooth_altitudes(
        [2.0, 8.0, 2.0], [0.0, 0.0, 0.0], [10.0, 10.0, 10.0], [100.0, 100.0], _limits()
    )
    assert viol == 0
    assert alts[1] == 8.0  # climb of 6 m over 100 m is well within limits


def test_smooth_altitudes_flags_infeasible_climb() -> None:
    # Navigable band jumps 10 m up over 1 m horizontal — impossible at 1 m/s.
    alts, viol, _up, _down = smooth_altitudes(
        [0.0, 10.0],
        [0.0, 10.0],
        [1.0, 11.0],
        [1.0],
        _limits(max_climb_rate_ms=1.0, cruise_speed_ms=10.0),
    )
    assert viol == 1
    assert alts[1] >= 10.0  # stays in the navigable band despite the violation
