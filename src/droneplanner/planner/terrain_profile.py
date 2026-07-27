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

* :func:`assess_clearance` is the *guarantee* — it re-scores the final altitudes
  against the flown profile and real terrain. This is what the risk report is
  allowed to trust, so a plan cannot silently claim clearance it does not have.
* :func:`densify_unsafe_descents` is the *mechanism* — it splits a leg the check
  rejects, so the autopilot cannot get ahead of the terrain. Because planner
  altitudes are **AGL** (terrain-relative), an inserted vertex picks up a
  terrain-following altitude from the normal band/smoothing pass.

**The mechanism is driven BY the guarantee, and that is load-bearing.** The first
version split every leg whose drop exceeded a threshold, without asking whether
that leg was actually unsafe. On the demo AO it split one that was already clear
at +9.0 m AGL, and that single gratuitous vertex **froze a real SITL mission at
11/79** on 2026-07-27 — 40 legs before the fix was ever exercised. See
:func:`densify_unsafe_descents` for the mechanism and why placement matters.

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


# Candidate insertion positions along a leg, as a fraction of its length. Coarse
# on purpose: a finer grid buys nothing (the clearance surface is smooth) and a
# denser search would make the placement sensitive to DEM sampling noise.
_CANDIDATE_FRACTIONS: tuple[float, ...] = tuple(round(0.10 + 0.05 * k, 2) for k in range(17))


def _settle_margin_m(length_m: float, drop_m: float, profile: FlownProfile) -> float:
    """Metres of LEVEL flight before a sub-leg's endpoint.

    The autopilot gives up ``drop_m`` at ``descent_gradient``, so it needs
    ``drop_m / gradient`` metres to get down; whatever is left of the sub-leg is
    flown level, which is when its altitude tracking settles. Negative means the
    vehicle is still descending when it arrives.
    """
    if drop_m <= 0.0:
        return length_m
    if profile.descent_gradient <= 0.0:
        return length_m
    return length_m - drop_m / profile.descent_gradient


def densify_unsafe_descents(
    pts: list[Point],
    amsl: list[float],
    *,
    predicted_amsl: Callable[[float, float], float],
    ground_at: GroundFn,
    profile: FlownProfile,
    margin_agl_m: float,
    max_inserts_per_leg: int = 3,
    samples_per_leg: int = DEFAULT_SAMPLES_PER_LEG,
) -> tuple[list[Point], int]:
    """Split only the legs whose flown profile actually hits terrain.

    ⚠️ **Two lessons from SITL are baked in here — do not "simplify" either away.**

    1. **Split only legs that FAIL the clearance check**, never every leg over a
       drop threshold. The first version split on ``drop > cruise_agl - margin``,
       which on the demo AO also split a leg that was already clear at +9.0 m AGL.
       That gratuitous vertex **froze the mission at 11/79** on 2026-07-27: PX4
       needs both horizontal *and* vertical acceptance (``NAV_MC_ALT_RAD``, 0.8 m
       by default), the vehicle passed it 1.58 m off vertically, the sequencer
       never advanced, and the trajectory generator slid past it for 565 s until
       it touched down. **An unreachable waypoint is worse than the CFIT it was
       meant to prevent** — it fails 40 legs earlier. Every inserted vertex is a
       new chance to fail this way, so insert as few as possible.
    2. **Place the vertex where the vehicle has LEVELLED OFF**, not at the
       geometric midpoint. Measured over 51 accepted arrivals (2026-07-24):
       PX4 tracks altitude to 0.35 m mean / 0.64 m p95 at a waypoint it reaches
       level — including a 27.8 m descent over 251 m, tracked to 0.22 m *unsplit*.
       The midpoint of that same leg was missed by 1.58 m. So placement is scored
       by :func:`_settle_margin_m`, the level-flight run before the endpoint.

    ``predicted_amsl(x, y)`` must return the AMSL altitude an inserted vertex
    would receive from the normal band + smoothing pass, so placement is scored
    against the altitude the vertex will actually get.

    Returns ``(points, inserted_count)``. Only *descending* legs are considered: a
    front-loaded climb gains clearance, and a linear climb is what the check
    already assumes.
    """
    if len(pts) != len(amsl):
        raise ValueError("pts and amsl must be the same length")
    if max_inserts_per_leg < 1:
        raise ValueError("max_inserts_per_leg must be at least 1")
    if len(pts) < 2:
        return list(pts), 0

    out: list[Point] = [pts[0]]
    inserted = 0
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        length = _leg_length(a, b)
        leg_clears = assess_clearance(
            [a, b],
            [amsl[i], amsl[i + 1]],
            ground_at,
            profile,
            margin_agl_m=margin_agl_m,
            samples_per_leg=samples_per_leg,
        ).clears
        if leg_clears or length <= 0.0 or amsl[i] <= amsl[i + 1]:
            out.append(b)
            continue

        placement = _best_placement(
            a,
            b,
            amsl[i],
            amsl[i + 1],
            predicted_amsl=predicted_amsl,
            ground_at=ground_at,
            profile=profile,
            margin_agl_m=margin_agl_m,
            max_inserts=max_inserts_per_leg,
            samples_per_leg=samples_per_leg,
        )
        out.extend(placement)
        inserted += len(placement)
        out.append(b)
    return out, inserted


def _best_placement(
    a: Point,
    b: Point,
    amsl_a: float,
    amsl_b: float,
    *,
    predicted_amsl: Callable[[float, float], float],
    ground_at: GroundFn,
    profile: FlownProfile,
    margin_agl_m: float,
    max_inserts: int,
    samples_per_leg: int,
) -> list[Point]:
    """Interior vertices for one failing leg — fewest first, then most settled.

    Tries 1 insert before 2, and among placements that clear, picks the one whose
    *worst* sub-leg has the longest level-flight run. Returns ``[]`` when nothing
    helps, leaving the leg intact for :func:`assess_clearance` to fail loudly —
    silently shipping unreachable vertices is the failure mode this whole module
    exists to avoid.
    """

    def at(frac: float) -> Point:
        return (a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac)

    def clears(fracs: tuple[float, ...]) -> tuple[bool, float]:
        pts = [a, *(at(f) for f in fracs), b]
        alts = [amsl_a, *(predicted_amsl(*at(f)) for f in fracs), amsl_b]
        rep = assess_clearance(
            pts,
            alts,
            ground_at,
            profile,
            margin_agl_m=margin_agl_m,
            samples_per_leg=samples_per_leg,
        )
        worst_settle = min(
            _settle_margin_m(_leg_length(pts[k], pts[k + 1]), alts[k] - alts[k + 1], profile)
            for k in range(len(pts) - 1)
        )
        return rep.clears, worst_settle

    for n in range(1, max_inserts + 1):
        best: tuple[float, tuple[float, ...]] | None = None
        for fracs in _candidate_sets(n):
            ok, settle = clears(fracs)
            if ok and (best is None or settle > best[0]):
                best = (settle, fracs)
        if best is not None:
            return [at(f) for f in best[1]]
    return []


def _candidate_sets(n: int) -> list[tuple[float, ...]]:
    """Insertion-fraction combinations to try for ``n`` interior vertices."""
    if n == 1:
        return [(f,) for f in _CANDIDATE_FRACTIONS]
    # Beyond one vertex the search space explodes and the marginal gain is small,
    # so fall back to even spacing — a leg needing 2+ inserts is already unusual.
    return [tuple((k + 1) / (n + 1) for k in range(n))]
