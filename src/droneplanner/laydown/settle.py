"""Where the paid-out fiber settles — ON_GROUND, IN_TRUNK_SPACE or ON_CROWNS.

WHY (Concealment-Rework-Roadmap C7, built minimally 2026-09-25)
The drone is exposed for a sortie; the fiber stays for the operation. Flying
OVER the canopy drapes it across the crowns instead of dropping it into trunk
space: wind-exposed, abrading on branches as payout continues (the tether
model's real break mechanism is snag, not speed), easy to cut and harder to
hide. Two things made that stop being hypothetical on 2026-09-25: the MVP flies
over the canopy everywhere outside the concealment zone, and
``over_canopy_when_unpassable`` climbs over stands inside it.

MODEL — bounded, and honest about it
Per leg of the flown route, from its flight bands, the canopy under it and the
canopy-opening raster (``canopy_opening``; ``layers.canopy_gaps``):

* released UNDER the canopy (both ends in the trunk layer)  -> ``IN_TRUNK_SPACE``
  (it falls to the ground through trunk space — may still catch low branches);
* released above a treed cell (canopy >= ``CROWN_SETTLE_M``) with no opening
  -> ``ON_CROWNS``;
* otherwise (open ground, or an opening it can fall through) -> ``ON_GROUND``.

Not modelled: crown closure beyond "an opening exists in this cell", wind drift of
the falling span, and branches catching fiber released in trunk space. The span
hanging between crown top and ground where the route passes between the two is
COUNTED (``cable_crown_hangs``) but not shaped. Treat
``ON_CROWNS`` as the length that certainly does not reach the ground, not as a
prediction of where it ends up. Needs field calibration (Preflight-Design §8).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.plan import FlightBand, TrajectoryVertex

CROWN_SETTLE_M = 3.0
"""Vegetation at least this tall catches a fiber released above it."""

_UNDER = (FlightBand.BELOW_CANOPY, FlightBand.UPPER_TRUNK)


def cable_settle_lengths(
    vertices: Sequence[TrajectoryVertex],
    chm: np.ndarray,
    grid: Grid,
    opening: np.ndarray | None,
    covered: np.ndarray | None = None,
) -> dict[str, float]:
    """Metres of laid fiber in each settle class, plus ON_CROWNS inside the zone."""
    out = {
        "cable_in_trunk_space_m": 0.0,
        "cable_on_crowns_m": 0.0,
        "cable_on_ground_m": 0.0,
        "cable_on_crowns_in_zone_m": 0.0,
        "cable_crown_hangs": 0.0,
    }
    prev_class: str | None = None

    def classed(kind: str) -> None:
        # A change between ON_CROWNS and IN_TRUNK_SPACE is a span hanging off a
        # crown edge down into the stand — where payout tension drags it across
        # branches (snag is the tether's real break mechanism).
        nonlocal prev_class
        if prev_class is not None and {prev_class, kind} == {"crowns", "trunk"}:
            out["cable_crown_hangs"] += 1.0
        prev_class = kind

    for a, b in pairwise(vertices):
        length = math.dist((a.x, a.y), (b.x, b.y))
        if length <= 0.0:
            continue
        if a.flight_band in _UNDER and b.flight_band in _UNDER:
            out["cable_in_trunk_space_m"] += length
            classed("trunk")
            continue
        mid = Coordinate(x=(a.x + b.x) / 2.0, y=(a.y + b.y) / 2.0)
        cell = grid.world_to_pixel(mid)
        treed = float(chm[cell]) >= CROWN_SETTLE_M
        has_opening = opening is not None and float(opening[cell]) > 0.5
        if treed and not has_opening:
            out["cable_on_crowns_m"] += length
            if covered is None or bool(covered[cell]):
                out["cable_on_crowns_in_zone_m"] += length
            classed("crowns")
        else:
            out["cable_on_ground_m"] += length
            classed("ground")
    return out
