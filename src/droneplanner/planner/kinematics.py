"""Tether/airframe kinematic feasibility pass (Software-Spec §3.6).

The any-angle Theta* route is geometrically optimal but not necessarily
executable: its corners can be sharper than ``min_turn_radius``, it could reverse
(slackening the fiber), and — once altitude is a free variable — its climb between
waypoints could exceed the airframe's climb/descent rate. This pass makes the
route flyable and reports what it could not fix:

* **Turn filleting** — replace each corner with a circular arc of radius up to
  ``min_turn_radius`` tangent to both legs, sampled into waypoints. Corners too
  tight for the desired radius (short legs) are filleted with the largest radius
  that fits and flagged; near-reversals are flagged and left sharp.
* **Altitude smoothing** — choose a *continuous* altitude within each waypoint's
  navigable band (forest: the vertical free window; open: constant cruise AGL)
  that respects the climb/descent-rate limits. Altitude is free to vary to find
  the navigable gap; it is not pinned to a fixed band.

All geometry is in the projected world CRS (meters).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from droneplanner.model.mission import DroneSpec

_STRAIGHT_EPS_RAD = math.radians(1.0)

Point = tuple[float, float]
BlockedFn = Callable[[float, float], bool]


@dataclass(frozen=True)
class KinematicLimits:
    """Airframe/tether limits used by the feasibility pass."""

    min_turn_radius_m: float
    max_climb_rate_ms: float
    max_descent_rate_ms: float
    cruise_speed_ms: float
    reversal_angle_deg: float = 150.0
    arc_samples: int = 4

    @classmethod
    def from_drone(cls, drone: DroneSpec, **overrides: float) -> KinematicLimits:
        base = {
            "min_turn_radius_m": drone.min_turn_radius_m,
            "max_climb_rate_ms": drone.climb_rate_max_ms,
            "max_descent_rate_ms": drone.descent_rate_max_ms,
            "cruise_speed_ms": drone.cruise_speed_ms,
        }
        base.update(overrides)
        return cls(**base)  # type: ignore[arg-type]


@dataclass(frozen=True)
class KinematicSummary:
    """What the feasibility pass found (feeds the risk report)."""

    tightest_turn_radius_m: float
    turn_violations: int
    reversals: int
    max_climb_rate_ms: float
    max_descent_rate_ms: float
    climb_violations: int

    @property
    def feasible(self) -> bool:
        # Reversals are allowed at low payout speed, so they do not break
        # feasibility — only turn-radius and climb-rate violations do.
        return self.turn_violations == 0 and self.climb_violations == 0


def _unit(a: Point, b: Point) -> tuple[float, float, float]:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        return 0.0, 0.0, 0.0
    return dx / length, dy / length, length


def fillet_path(
    pts: list[Point], limits: KinematicLimits, is_blocked: BlockedFn | None = None
) -> tuple[list[Point], int, int, float]:
    """Round corners to <= ``min_turn_radius`` arcs.

    Returns (new_points, turn_violations, reversals, tightest_radius_m). A corner
    is a turn_violation if the desired radius did not fit; a reversal if its
    deflection exceeds ``reversal_angle_deg`` (left sharp). Arcs that would enter
    a blocked cell are skipped (corner kept sharp, counted as a turn_violation).
    """
    if len(pts) <= 2:
        return list(pts), 0, 0, limits.min_turn_radius_m

    out: list[Point] = [pts[0]]
    turn_violations = 0
    reversals = 0
    tightest = limits.min_turn_radius_m

    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        u1x, u1y, len_in = _unit(a, b)
        u2x, u2y, len_out = _unit(b, c)
        if len_in == 0.0 or len_out == 0.0:
            out.append(b)
            continue

        cross = u1x * u2y - u1y * u2x
        dot = max(-1.0, min(1.0, u1x * u2x + u1y * u2y))
        delta = math.atan2(abs(cross), dot)  # deflection 0..pi
        if delta < _STRAIGHT_EPS_RAD:
            out.append(b)
            continue

        is_reversal = math.degrees(delta) >= limits.reversal_angle_deg
        tan_half = math.tan(delta / 2.0)
        max_t = 0.5 * min(len_in, len_out)
        t_desired = limits.min_turn_radius_m * tan_half
        if t_desired <= max_t:
            radius, tan_t = limits.min_turn_radius_m, t_desired
        else:
            radius, tan_t = max_t / tan_half, max_t
            turn_violations += 1
        tightest = min(tightest, radius)

        if is_reversal:
            reversals += 1
        if is_reversal or radius < 0.1:
            out.append(b)  # cannot round a reversal / degenerate corner
            continue

        p1 = (b[0] - u1x * tan_t, b[1] - u1y * tan_t)
        p2 = (b[0] + u2x * tan_t, b[1] + u2y * tan_t)
        turn_sign = 1.0 if cross > 0 else -1.0
        n1 = (-u1y * turn_sign, u1x * turn_sign)  # inward normal toward the arc center
        center = (p1[0] + n1[0] * radius, p1[1] + n1[1] * radius)
        start_ang = math.atan2(p1[1] - center[1], p1[0] - center[0])

        arc = [p1]
        for k in range(1, limits.arc_samples):
            ang = start_ang + turn_sign * delta * (k / limits.arc_samples)
            arc.append((center[0] + radius * math.cos(ang), center[1] + radius * math.sin(ang)))
        arc.append(p2)

        if is_blocked is not None and any(is_blocked(x, y) for x, y in arc):
            out.append(b)
            turn_violations += 1
            continue
        out.extend(arc)

    out.append(pts[-1])
    return out, turn_violations, reversals, tightest


def smooth_altitudes(
    prefs: list[float],
    floors: list[float],
    ceils: list[float],
    seg_dists: list[float],
    limits: KinematicLimits,
) -> tuple[list[float], int, float, float]:
    """Pick a climb-feasible altitude in each [floor, ceil] band near ``prefs``.

    Prioritizes staying inside the navigable band; where the climb/descent limit
    cannot also be met, it clamps to the band and counts a climb_violation.
    Returns (altitudes, climb_violations, max_climb_rate, max_descent_rate).
    """
    n = len(prefs)
    if n == 0:
        return [], 0, 0.0, 0.0
    up = limits.max_climb_rate_ms / limits.cruise_speed_ms  # meters up per horizontal meter
    down = limits.max_descent_rate_ms / limits.cruise_speed_ms

    alts = [min(max(prefs[0], floors[0]), ceils[0])]
    climb_violations = 0
    max_climb = max_descent = 0.0
    for i in range(1, n):
        d = seg_dists[i - 1]
        prev = alts[-1]
        lo_reach, hi_reach = prev - down * d, prev + up * d
        band_lo, band_hi = floors[i], ceils[i]
        target = min(max(prefs[i], band_lo), band_hi)
        # Intersect climb-reachable interval with the navigable band.
        lo = max(lo_reach, band_lo)
        hi = min(hi_reach, band_hi)
        if lo <= hi:
            z = min(max(target, lo), hi)
        else:
            z = min(max(prefs[i], band_lo), band_hi)  # stay navigable; break the climb limit
            climb_violations += 1
        alts.append(z)

        rate = abs(z - prev) / d * limits.cruise_speed_ms if d > 0 else 0.0
        if z >= prev:
            max_climb = max(max_climb, rate)
        else:
            max_descent = max(max_descent, rate)
    return alts, climb_violations, max_climb, max_descent
