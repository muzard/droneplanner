"""Where along the route can a GPS-denied aircraft fix its position? (roadmap P2)

WHY THIS EXISTS
The flight stack navigates GPS-denied on VIO, which drifts *coherently* (~2 % of
distance flown, droneflight ``nav.py``), bounded only by position fixes. Every
fix source it has measured fails in the same place: optical terrain matching
(VTRN, ``droneflight/correction.py``) declined **76.5 %** of fixes for
``no_features`` on the forested arm, because canopy is low-texture — and the
forest is the aircraft's own cover, so it is most lost where it is most
concealed. The stem-map localiser (0.03 m) works exactly there, but needs a stem
map, and ``forest/als_model.py`` showed the open national lidar cannot make one.

Until now the planner priced concealment, endurance and clearance but not this:
a route could spend 2 km under closed canopy with no fix opportunity and the
report said nothing. This module says it.

THE MODEL — deliberately coarse, and conservative in the documented direction
* A sample can take a **fix** where the ground is open (canopy below
  :data:`FIX_OPEN_CANOPY_M`): VTRN has texture to match there. Treed cells starve
  it whether the aircraft flies under the crowns or over them.
* Where a **stem map** is supplied, samples within :data:`STEM_FIX_RADIUS_M` of a
  mapped stem can fix too (the stem localiser).
* Between fixes, drift grows at :data:`DRIFT_FRACTION` of distance flown — the
  rate the flight stack's error model uses, starting from a fix at launch.
* An unfixed run whose drift exceeds ``drift_budget_m`` (default
  :data:`DEFAULT_DRIFT_BUDGET_M`, the 15 m corridor droneflight's acceptance
  arms were scored against) is a WARNING.

⚠️ "Open ground permits a fix" is optimistic: a stale or featureless basemap
(fields, snow, a changed battlespace) declines fixes too, and an accepted fix
can be wrong. "Closed canopy permits none" is pessimistic for over-canopy flight
above *textured* canopy. Report-only: the run length is a property of the whole
path, not of a cell, so it cannot be a per-cell cost term without changing the
search; routing to fixes is the operator's call for now.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise

import numpy as np

from droneplanner.fields.stems import StemMap
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid

DRIFT_FRACTION = 0.02
"""VIO drift per metre flown (droneflight ``nav.py``, Tech-Selection §2.1)."""

DEFAULT_DRIFT_BUDGET_M = 15.0
"""Corridor the GPS-denied acceptance arms were scored against (droneflight)."""

FIX_OPEN_CANOPY_M = 3.0
"""Canopy below this is open enough for nadir terrain matching."""

STEM_FIX_RADIUS_M = 10.0
"""A mapped stem this close means the stem localiser sees enough of the map."""

SAMPLE_STEP_M = 5.0


def assess_fix_availability(
    points: Sequence[Coordinate],
    chm: np.ndarray,
    grid: Grid,
    *,
    stems: StemMap | None = None,
    drift_budget_m: float = DEFAULT_DRIFT_BUDGET_M,
) -> dict[str, float]:
    """Scores for the fix opportunities along a polyline (see module docstring)."""
    out = {
        "nav_max_unfixed_m": 0.0,
        "nav_max_unfixed_start_m": 0.0,
        "nav_max_drift_m": 0.0,
        "nav_unfixed_fraction": 0.0,
        "nav_runs_over_budget": 0.0,
        "nav_drift_budget_m": drift_budget_m,
    }
    xs: list[float] = []
    ys: list[float] = []
    along: list[float] = []
    total = 0.0
    for a, b in pairwise(points):
        seg = a.distance_to(b)
        n = max(1, math.ceil(seg / SAMPLE_STEP_M))
        for k in range(n):
            t = k / n
            xs.append(a.x + t * (b.x - a.x))
            ys.append(a.y + t * (b.y - a.y))
            along.append(total + t * seg)
        total += seg
    if not points or total <= 0.0:
        return out
    xs.append(points[-1].x)
    ys.append(points[-1].y)
    along.append(total)

    fix = np.empty(len(xs), dtype=bool)
    for i, (x, y) in enumerate(zip(xs, ys, strict=True)):
        fix[i] = float(chm[grid.world_to_pixel(Coordinate(x=x, y=y))]) < FIX_OPEN_CANOPY_M
    if stems is not None and len(stems) > 0:
        d, _ = stems.tree.query(np.c_[xs, ys])
        fix |= d <= STEM_FIX_RADIUS_M
    fix[0] = True  # the aircraft knows where it is at launch

    unfixed = 0.0
    last_fix = 0.0
    over = 0
    for i in range(1, len(along)):
        step = along[i] - along[i - 1]
        if not fix[i]:
            unfixed += step
        if fix[i] or i == len(along) - 1:
            run = along[i] - last_fix
            if run > out["nav_max_unfixed_m"]:
                out["nav_max_unfixed_m"] = run
                out["nav_max_unfixed_start_m"] = last_fix
            if run * DRIFT_FRACTION > drift_budget_m:
                over += 1
            last_fix = along[i]
    out["nav_max_drift_m"] = out["nav_max_unfixed_m"] * DRIFT_FRACTION
    out["nav_unfixed_fraction"] = unfixed / total
    out["nav_runs_over_budget"] = float(over)
    return out
