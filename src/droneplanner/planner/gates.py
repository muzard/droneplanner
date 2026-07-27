"""Reduce a dense route to the few points that actually decide it — its *gates*.

WHY THIS EXISTS (TEST-PLAN A2)
The flight stack does not need 282 waypoints; per the user (2026-07-27) the plan
should read more like *"enter the forest patch here and exit here"* and leave the
executor to work out the middle. This module turns that from a slogan into a
decidable test.

**The decision rule falls straight out of the corridor (A1).** The corridor is
the region the aircraft may use without losing the concealment the route was
scored on. So a straight line between two gates is an acceptable substitute for
the planned path between them **exactly when that straight stays inside the
corridor** — nothing about concealment, patch boundaries or obstacles needs
restating here, because the corridor mask already encodes all three.

The reduction is therefore a greedy forward sweep: from the current gate, reach
as far along the route as the straight stays admissible, drop a gate there, and
repeat. Gates land where they must — at the mouth of a stand, at the lip of a
depression, wherever the cover forces a turn — rather than at a fixed spacing.

⚠️ **A gate route is NOT a drop-in replacement for the dense one.** The dense
waypoints exist for a second reason the corridor knows nothing about: they keep
each leg's *altitude step* small, which is what made the low terrain-following
route sequence cleanly in SITL (TEST-PLAN B1 — 8.56 m max step, worst vertical
error 1.817 m; the sparse route it replaced had 18-40 m steps and stalled three
missions). Reducing to gates puts the long legs back, so the altitude between
gates has to come from **somewhere else** — an executor that follows terrain
itself. Emitted into PX4 mission mode, which interpolates altitude between
waypoints, a gate route reintroduces exactly the CFIT this project already
crashed an aircraft on. :func:`assess_gate_altitude_risk` measures that cost so
the trade is explicit rather than discovered in flight.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.raster_lines import bresenham


@dataclass(frozen=True)
class GateReduction:
    """A dense route expressed as the few points that decide it."""

    indices: tuple[int, ...]
    """Indices into the original vertex list, always including first and last."""

    gates: tuple[Coordinate, ...]
    max_deviation_m: float
    """Furthest the gate-to-gate straights stray from the original route."""

    @property
    def count(self) -> int:
        return len(self.indices)


def _segment_inside(grid: Grid, mask: np.ndarray, a: Coordinate, b: Coordinate) -> bool:
    """Does the straight from ``a`` to ``b`` stay inside the admissible mask?"""
    ra, ca = grid.world_to_pixel(a)
    rb, cb = grid.world_to_pixel(b)
    return all(mask[r, c] for r, c in bresenham(ra, ca, rb, cb))


def _point_to_segment_m(p: Coordinate, a: Coordinate, b: Coordinate) -> float:
    abx, aby = b.x - a.x, b.y - a.y
    denom = abx * abx + aby * aby
    if denom <= 0.0:
        return p.distance_to(a)
    t = max(0.0, min(1.0, ((p.x - a.x) * abx + (p.y - a.y) * aby) / denom))
    return p.distance_to(Coordinate(x=a.x + abx * t, y=a.y + aby * t))


def reduce_to_gates(vertices: list[Coordinate], corridor: np.ndarray, grid: Grid) -> GateReduction:
    """Greedily drop every vertex the corridor makes unnecessary.

    ``corridor`` is the swept mask from :func:`planner.corridor_width.assess_corridor`.
    A vertex survives only where the straight past it would leave that region.
    """
    if len(vertices) < 2:
        return GateReduction(
            indices=tuple(range(len(vertices))),
            gates=tuple(vertices),
            max_deviation_m=0.0,
        )

    keep = [0]
    i = 0
    while i < len(vertices) - 1:
        # Reach as far as the straight stays admissible; never fewer than one
        # step, or a route pinched to zero corridor would not terminate.
        furthest = i + 1
        for j in range(len(vertices) - 1, i, -1):
            if _segment_inside(grid, corridor, vertices[i], vertices[j]):
                furthest = j
                break
        keep.append(furthest)
        i = furthest

    deviation = 0.0
    for a_idx, b_idx in pairwise(keep):
        a, b = vertices[a_idx], vertices[b_idx]
        for k in range(a_idx, b_idx + 1):
            deviation = max(deviation, _point_to_segment_m(vertices[k], a, b))

    return GateReduction(
        indices=tuple(keep),
        gates=tuple(vertices[k] for k in keep),
        max_deviation_m=deviation,
    )


@dataclass(frozen=True)
class GateAltitudeRisk:
    """What reducing to gates costs in commanded-altitude fidelity.

    The corridor decides where the aircraft may fly *horizontally*. It says
    nothing about altitude, and altitude is the reason the dense route existed.
    """

    max_step_m: float
    """Largest altitude step a gate leg demands, if altitude is flown linearly."""

    max_leg_m: float
    dense_max_step_m: float
    """The same quantity on the route before reduction — the baseline that flew."""


def assess_gate_altitude_risk(
    vertices: list[Coordinate], altitudes_amsl: list[float], indices: tuple[int, ...]
) -> GateAltitudeRisk:
    """Compare the altitude steps a gate route demands against the dense one's."""
    dense_step = max(
        (abs(b - a) for a, b in pairwise(altitudes_amsl)),
        default=0.0,
    )
    step = 0.0
    leg = 0.0
    for a_idx, b_idx in pairwise(indices):
        step = max(step, abs(altitudes_amsl[b_idx] - altitudes_amsl[a_idx]))
        leg = max(leg, vertices[a_idx].distance_to(vertices[b_idx]))
    return GateAltitudeRisk(max_step_m=step, max_leg_m=leg, dense_max_step_m=dense_step)
