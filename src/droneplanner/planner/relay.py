"""Split a route that does not fit the battery into legs that do (relay sorties).

WHY THIS EXISTS
TEST-PLAN N1 closed the speed window: the concealed below-canopy route needs
57.1 min against a 30 min airframe, and N3 showed routing cannot recover it
(0 of 19 272 treed cells clear the 4.00 m/s endurance floor). The programme's own
list of ways out has four entries, and three of them are expensive — buy a bigger
airframe, abandon concealment, or build a stem map nothing can produce yet. The
fourth, **"shorter legs with relay points"**, is the one `CLAUDE.md` calls
*"cheapest, and it is a concept-of-operations change, not a software one"* — and
it is the only one that had never been costed. This module costs it.

WHAT A LEG IS, AND WHAT IT IS NOT
A leg is a contiguous run of the **already-planned, already-concealed** route that
one battery can fly. Splitting changes nothing about where the route goes, so the
concealment the route was scored on is preserved exactly — that is the whole
attraction of this option compared with `weights.clutter`, which buys a closing
mission by leaving the forest and paying 2.7x the exposure.

What splitting *does* cost is real and is not modelled here:

* **A relay point is a place on the ground where someone must be**, to swap a
  battery or hand over to the next aircraft. :func:`split_into_legs` reports where
  those points fall; whether they are reachable and concealed is a question for
  the operator and for the exposure raster, not for this function.
* **Every leg pays its own terminal overhead** (climb-out and approach, 19.1 s
  measured on B1) and every relay implies a **fiber splice or a spool handover**.
  The splice loss and the turnaround time are CONOPS numbers the programme has
  never stated, so total mission duration is deliberately *not* reported — only
  per-leg flight time, which is what the battery constrains.

THE SPLIT IS GREEDY AND THAT IS OPTIMAL HERE
Energy consumption is monotone in leg length (every added vertex adds distance and
one waypoint overhead, both non-negative), so "extend the current leg as far as it
still closes, then cut" yields the minimum number of legs. The extension point is
found by binary search rather than linear extension, which keeps a 282-vertex
route at a few hundred endurance evaluations instead of tens of thousands.

🔴 **A route can be unsplittable.** If a single *pair* of adjacent vertices already
exceeds the budget, no number of relays helps — the aircraft cannot cross that gap
on one battery at the speed the forest permits. :attr:`RelayPlan.feasible` says so
rather than returning a silently truncated answer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from droneplanner.planner.endurance import (
    EnduranceModel,
    EnduranceReport,
    Point,
    assess_endurance,
)


@dataclass(frozen=True)
class RelayLeg:
    """One battery's worth of the route, as a closed interval of vertex indices."""

    start_index: int
    end_index: int
    """Inclusive. ``end_index`` of leg *n* equals ``start_index`` of leg *n+1*:
    the relay point is flown to by one leg and departed by the next, so it is a
    shared vertex, not a gap."""

    length_m: float
    endurance: EnduranceReport

    @property
    def vertex_count(self) -> int:
        return self.end_index - self.start_index + 1

    @property
    def closes(self) -> bool:
        return self.endurance.closes


@dataclass(frozen=True)
class RelayPlan:
    """The whole route expressed as relay legs."""

    legs: list[RelayLeg]
    relay_indices: list[int]
    """Interior vertices where a relay happens — the split points. Empty when the
    route already fits one battery."""

    feasible: bool
    """False when some adjacent vertex pair alone exceeds the budget, which no
    amount of relaying can fix."""

    blocking_index: int | None = None
    """First vertex whose own outgoing leg cannot close, when ``feasible`` is
    False."""

    @property
    def leg_count(self) -> int:
        return len(self.legs)

    @property
    def worst_energy_fraction(self) -> float:
        return max((leg.endurance.energy_fraction for leg in self.legs), default=0.0)


def _fits(report: EnduranceReport, energy_cap: float) -> bool:
    """Does this leg fit inside ``energy_cap`` of usable endurance?"""
    return report.stated and report.energy_fraction <= energy_cap


def _leg_report(
    pts: list[Point],
    amsl: list[float],
    model: EnduranceModel,
    speed_limits_ms: list[float] | None,
    lo: int,
    hi: int,
) -> EnduranceReport:
    """Endurance of the sub-route ``pts[lo:hi + 1]``, scored as its own sortie."""
    limits = speed_limits_ms[lo : hi + 1] if speed_limits_ms is not None else None
    return assess_endurance(pts[lo : hi + 1], amsl[lo : hi + 1], model, speed_limits_ms=limits)


def split_into_legs(
    pts: list[Point],
    amsl: list[float],
    model: EnduranceModel,
    *,
    speed_limits_ms: list[float] | None = None,
    energy_cap: float = 1.0,
) -> RelayPlan:
    """Cut a route into the fewest legs that each fit one battery.

    Each leg is scored by :func:`endurance.assess_endurance` as a standalone
    sortie — its own terminal overhead, its own waypoint count, its own climb
    surcharge — because that is what a relay actually is. ``speed_limits_ms`` is
    the per-vertex stem-density cap; supplying it is what makes the answer honest
    about below-canopy legs, exactly as it is for the whole-route budget.

    ``energy_cap`` is the fraction of *usable* endurance a leg may spend. The
    default 1.0 minimises leg count, which is what you want to know first — but it
    also lands every leg except the last exactly on the limit, with the remainder
    dumped into a short tail. :func:`balance_legs` is the operational answer.
    """
    n = len(pts)
    if n != len(amsl):
        raise ValueError("pts and amsl must be the same length")
    if speed_limits_ms is not None and len(speed_limits_ms) != n:
        raise ValueError("speed_limits_ms must be the same length as pts")
    if n < 2:
        return RelayPlan(legs=[], relay_indices=[], feasible=True)

    legs: list[RelayLeg] = []
    relays: list[int] = []
    start = 0
    while start < n - 1:
        # A leg must advance at least one vertex. If even that does not close,
        # relaying cannot help: the gap itself is too long for one battery.
        if not _fits(_leg_report(pts, amsl, model, speed_limits_ms, start, start + 1), energy_cap):
            return RelayPlan(legs=legs, relay_indices=relays, feasible=False, blocking_index=start)

        # Energy is monotone in the endpoint, so binary-search the furthest one
        # that still closes instead of walking vertex by vertex.
        lo, hi = start + 1, n - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if _fits(_leg_report(pts, amsl, model, speed_limits_ms, start, mid), energy_cap):
                lo = mid
            else:
                hi = mid - 1
        end = lo
        report = _leg_report(pts, amsl, model, speed_limits_ms, start, end)
        length = sum(math.dist(pts[i], pts[i + 1]) for i in range(start, end))
        legs.append(RelayLeg(start_index=start, end_index=end, length_m=length, endurance=report))
        if end < n - 1:
            relays.append(end)
        start = end

    return RelayPlan(legs=legs, relay_indices=relays, feasible=True)


def legs_required_for_endurance(
    pts: list[Point],
    amsl: list[float],
    model: EnduranceModel,
    endurance_s: float,
    *,
    speed_limits_ms: list[float] | None = None,
) -> RelayPlan:
    """:func:`split_into_legs` against a hypothetical airframe endurance.

    The trade the operator actually faces is *relays versus battery*: a longer
    endurance buys fewer ground stops. Sweeping this answers "what would a 45-minute
    airframe be worth?" without touching the mission file.
    """
    hypothetical = replace(model, endurance_s=endurance_s)
    return split_into_legs(pts, amsl, hypothetical, speed_limits_ms=speed_limits_ms)


def balance_legs(
    pts: list[Point],
    amsl: list[float],
    model: EnduranceModel,
    *,
    speed_limits_ms: list[float] | None = None,
    leg_count: int | None = None,
) -> RelayPlan:
    """Split into ``leg_count`` legs that share the load as evenly as possible.

    :func:`split_into_legs` minimises the *number* of legs, which is the right
    first question and the wrong thing to fly: maximising every leg lands each one
    exactly on the endurance limit and dumps the remainder into a short final hop.
    On the demo route that is 2392 m / 2546 m / 229 m, with two legs at **1.00** of
    usable endurance — no margin at all for wind, a diversion, or a relay that is
    not exactly where the map says.

    This minimises the **worst** leg instead, by binary-searching the tightest
    energy cap that still fits ``leg_count`` legs. ``leg_count`` defaults to the
    minimum :func:`split_into_legs` found, so the usual call costs nothing extra
    in ground stops and buys back the whole margin.
    """
    minimum = split_into_legs(pts, amsl, model, speed_limits_ms=speed_limits_ms)
    if not minimum.feasible:
        return minimum
    target = minimum.leg_count if leg_count is None else leg_count
    if target < minimum.leg_count:
        raise ValueError(
            f"{target} legs cannot cover this route; the minimum is {minimum.leg_count}"
        )

    # The leg count is monotone non-increasing in the cap, so binary-search it.
    # 1e-3 of usable endurance is well under a second on any plausible airframe.
    lo, hi = 0.0, 1.0
    best = minimum
    for _ in range(40):
        mid = (lo + hi) / 2.0
        trial = split_into_legs(pts, amsl, model, speed_limits_ms=speed_limits_ms, energy_cap=mid)
        if trial.feasible and trial.leg_count <= target:
            best = trial
            hi = mid
        else:
            lo = mid
        if hi - lo < 1e-3:
            break
    return best
