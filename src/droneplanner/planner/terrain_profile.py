"""The altitude profile PX4 actually flies along a leg, and the clearance it implies.

WHY THIS EXISTS (SHORTFALLS item 0 — a CFIT defect proven in SITL)
The clearance guarantee used to assume altitude is interpolated **linearly with
ground distance** between waypoints. PX4 does not do that. It drives toward the
next waypoint's altitude at its own vertical rate, so a long leg with a big drop
reaches the target altitude in the first fraction of the leg and then flies level.
Over a *falling* slope that puts the aircraft into the ground while the plan still
reads a healthy AGL at every vertex and along every linearly-interpolated leg.

Measured 2026-07-24 (`sim-logs/terrain/17_12_10.ulg`), leg wp50→wp51, 765 m,
-40.5 m: at 17 % along the leg linear interpolation predicts 27.0 m AGL; the
aircraft was at **0.8 m AGL** — it had burned 33 of the 40 m of descent in the
first sixth of the leg, while still over the high end of the slope. It flew into
the ground and the mission stalled at 51/77.

Two behaviours, deliberately **asymmetric** — this is not an oversight:

* **Descent is front-loaded.** Measured, above. This is the hazard, so it is
  modelled explicitly.
* **Climb is modelled as linear**, not front-loaded. The same flight climbed a
  +62 m leg without incident, which suggests PX4 climbs promptly too — but prompt
  climbing only *adds* clearance, so assuming it would make this check optimistic
  on the strength of an unvalidated behaviour. Linear is the conservative choice
  and stays conservative if PX4 ever climbs lazily.

The fix has two halves and they are deliberately separate:

* :func:`densify_descending_legs` is the *mechanism* — it splits a big-drop leg so
  the autopilot can never get more than one short sub-leg ahead of the terrain.
  Because planner altitudes are **AGL** (terrain-relative), an inserted vertex
  picks up a terrain-following altitude from the normal band/smoothing pass; the
  densifier itself is purely geometric.
* :func:`assess_clearance` is the *guarantee* — it re-scores the final altitudes
  against the flown profile and real terrain. Densification is a heuristic; this
  is what the risk report is allowed to trust, so a plan cannot silently claim
  clearance it does not have.

All geometry is in the projected world CRS (meters); altitudes here are **AMSL**,
because clearance is a question about terrain, not about the AGL bookkeeping.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

Point = tuple[float, float]
GroundFn = Callable[[float, float], float]

# Terrain is sampled this densely along each leg when scoring clearance. The
# defect it exists to catch appeared at 17 % along a 765 m leg, so the sampling
# must be fine enough to land inside the front-loaded portion of a long leg.
DEFAULT_SAMPLES_PER_LEG = 32


@dataclass(frozen=True)
class FlownProfile:
    """How the autopilot distributes an altitude change along a leg.

    ``descent_gradient`` is metres of altitude given up per horizontal metre
    travelled — the airframe's descent rate divided by its cruise speed.
    """

    descent_gradient: float

    @classmethod
    def from_rates(cls, descent_rate_ms: float, cruise_speed_ms: float) -> FlownProfile:
        if cruise_speed_ms <= 0.0:
            raise ValueError("cruise_speed_ms must be positive")
        if descent_rate_ms < 0.0:
            raise ValueError("descent_rate_ms must not be negative")
        return cls(descent_gradient=descent_rate_ms / cruise_speed_ms)

    def altitude_amsl_at(
        self, along_m: float, *, length_m: float, start_amsl: float, end_amsl: float
    ) -> float:
        """Commanded AMSL altitude ``along_m`` metres into a leg.

        Descending: front-loaded at ``descent_gradient``, then level at the
        target. Climbing: linear in ground distance (see the module docstring for
        why the two directions differ).
        """
        if length_m <= 0.0:
            return end_amsl
        s = min(max(along_m, 0.0), length_m)
        if end_amsl >= start_amsl:
            return start_amsl + (end_amsl - start_amsl) * (s / length_m)
        return max(end_amsl, start_amsl - self.descent_gradient * s)


@dataclass(frozen=True)
class ClearanceReport:
    """Terrain clearance of a route scored against the profile PX4 flies."""

    min_clearance_agl_m: float
    """Lowest AGL anywhere along the route under the flown profile."""

    violations: int
    """Legs whose flown profile drops below ``margin_agl_m``."""

    worst_leg_index: int
    """Leg (vertex index of its start) holding ``min_clearance_agl_m``; -1 if none."""

    margin_agl_m: float
    """The clearance floor this was scored against."""

    @property
    def clears(self) -> bool:
        return self.violations == 0


def _leg_length(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def assess_clearance(
    pts: list[Point],
    amsl: list[float],
    ground_at: GroundFn,
    profile: FlownProfile,
    *,
    margin_agl_m: float,
    samples_per_leg: int = DEFAULT_SAMPLES_PER_LEG,
) -> ClearanceReport:
    """Score every leg's flown altitude profile against the terrain beneath it.

    This is the authoritative clearance check — the one the risk report may
    trust. It samples real terrain rather than assuming anything about the
    route's shape.
    """
    if len(pts) != len(amsl):
        raise ValueError("pts and amsl must be the same length")
    if samples_per_leg < 2:
        raise ValueError("samples_per_leg must be at least 2")
    if len(pts) < 2:
        return ClearanceReport(
            min_clearance_agl_m=math.inf,
            violations=0,
            worst_leg_index=-1,
            margin_agl_m=margin_agl_m,
        )

    worst = math.inf
    worst_leg = -1
    violations = 0
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        length = _leg_length(a, b)
        leg_worst = math.inf
        for k in range(samples_per_leg + 1):
            frac = k / samples_per_leg
            x = a[0] + (b[0] - a[0]) * frac
            y = a[1] + (b[1] - a[1]) * frac
            flown = profile.altitude_amsl_at(
                length * frac, length_m=length, start_amsl=amsl[i], end_amsl=amsl[i + 1]
            )
            leg_worst = min(leg_worst, flown - ground_at(x, y))
        if leg_worst < margin_agl_m:
            violations += 1
        if leg_worst < worst:
            worst, worst_leg = leg_worst, i
    return ClearanceReport(
        min_clearance_agl_m=worst,
        violations=violations,
        worst_leg_index=worst_leg,
        margin_agl_m=margin_agl_m,
    )


def densify_descending_legs(
    pts: list[Point], amsl: list[float], *, max_drop_m: float
) -> tuple[list[Point], int]:
    """Split legs that descend more than ``max_drop_m`` into equal sub-legs.

    The front-loaded profile can be below the terrain-following one by at most the
    leg's own drop, so capping the per-sub-leg drop caps how far the autopilot can
    get ahead of the terrain. Returns ``(points, inserted_count)``.

    Only *descending* legs are split: a front-loaded climb gains clearance, and a
    linear climb is what the check already assumes.
    """
    if max_drop_m <= 0.0:
        raise ValueError("max_drop_m must be positive")
    if len(pts) != len(amsl):
        raise ValueError("pts and amsl must be the same length")
    if len(pts) < 2:
        return list(pts), 0

    out: list[Point] = [pts[0]]
    inserted = 0
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        drop = amsl[i] - amsl[i + 1]
        if drop > max_drop_m and _leg_length(a, b) > 0.0:
            n = math.ceil(drop / max_drop_m)
            for k in range(1, n):
                frac = k / n
                out.append((a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac))
                inserted += 1
        out.append(b)
    return out, inserted
