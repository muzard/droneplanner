"""Individual stem positions as a planner input (TEST-PLAN N4).

WHY THIS EXISTS
Reactive avoidance is dead in the flight stack: four independent negatives, ending
with N2, where slowing from 6.0 to 2.0 m/s made strikes *worse* (34 -> 41) and
every candidate cause — speed, angular resolution, sensor range, field of view,
sensor aim — was eliminated by measurement. N4 then measured the alternative and
found it cheap: a collision-free path between *mapped* stems costs **+1.6 %
distance** in a +/-25 m corridor and **+2.9 %** at 3.0 m clearance, because the
forest is 99.9 % free area. It is not a maze; it is open space with thin obstacles,
and reactive avoidance fails there from late commitment on local information.

So the planner needs to see stems. This module is that input.

⚠️ **EVERY STEM MAP THE PROGRAMME HAS IS SYNTHETIC.** They come from
``io/forest_models.generate_trunks``, which *invents* trunks where the planner
believed there was canopy. No real ``.laz`` has ever been ingested, no individual
stem has ever been detected, and ALS crown-centroid stem positions carry ~1-2 m
error against the **2.4 m** clearance budget N4 derived. Routing between mapped
stems is therefore demonstrated *in simulation against the same file that places
the obstacles* — which is the right way to test the planner and says nothing yet
about whether a real map can be built. Treat a clearance number from this module
as "the planner can use a stem map", never as "the aircraft will miss the trees".

⚠️ **The synthetic manifests are also route-shaped.** ``generate_trunks`` places
stems in a band around *the route that was planned without them*, so a re-planned
route that leaves the band finds open space that a real forest would have filled.
:func:`mapped_fraction` measures how much of a route relies on that, and any
clearance claim must be quoted next to it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from droneplanner.model.geometry import Coordinate


@dataclass(frozen=True)
class StemMap:
    """Stem centres in world coordinates, plus their radii.

    The manifest ``io/forest_models`` writes is **home-relative** (east/north
    metres from the mission start) because that is the frame Gazebo and the flight
    stack share. The planner works in world CRS coordinates, so the home offset is
    applied on load and never carried around afterwards — a manifest read in the
    wrong frame is the third instance of the absolute-vs-home-relative mismatch
    this programme has already hit (``reactive.py`` put a forest 5.2e6 m away and
    every run returned clean at exactly 0.000 m clearance).
    """

    x: np.ndarray
    y: np.ndarray
    radius_m: np.ndarray

    def __post_init__(self) -> None:
        if not (len(self.x) == len(self.y) == len(self.radius_m)):
            raise ValueError("stem arrays must be the same length")

    def __len__(self) -> int:
        return len(self.x)

    @property
    def tree(self) -> cKDTree:
        """Spatial index over stem centres. Built once per map, cached."""
        cached = getattr(self, "_tree", None)
        if cached is None:
            cached = cKDTree(np.c_[self.x, self.y])
            object.__setattr__(self, "_tree", cached)
        return cached

    def clearance_at(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Distance from each point to the nearest stem **surface**, in metres.

        Surface, not centre: a 0.15 m trunk that the centre-distance calls 0.30 m
        clear leaves only 0.15 m of air. Negative means inside a trunk.
        """
        if len(self) == 0:
            return np.full(np.shape(xs), np.inf, dtype=np.float64)
        dist, idx = self.tree.query(np.c_[np.ravel(xs), np.ravel(ys)], k=1)
        gap: np.ndarray = (dist - self.radius_m[idx]).reshape(np.shape(xs))
        return gap

    def bounds(self) -> tuple[float, float, float, float]:
        """(min_x, min_y, max_x, max_y) of the mapped area."""
        return (
            float(self.x.min()),
            float(self.y.min()),
            float(self.x.max()),
            float(self.y.max()),
        )


def load_stem_manifest(path: str | Path, home: Coordinate) -> StemMap:
    """Read a ``*_trunks.csv`` manifest, converting home-relative to world.

    One file feeds the planner, the Gazebo world and the perception oracle, which
    is deliberate: generating the obstacles twice would be a second chance for
    them to disagree.
    """
    raw = np.genfromtxt(Path(path), delimiter=",", names=True)
    if raw.size == 0:
        return StemMap(np.empty(0), np.empty(0), np.empty(0))
    east = np.atleast_1d(raw["east_m"]).astype(float)
    north = np.atleast_1d(raw["north_m"]).astype(float)
    radius = np.atleast_1d(raw["radius_m"]).astype(float)
    return StemMap(x=east + home.x, y=north + home.y, radius_m=radius)


def required_clearance_m(rotor_span_m: float, margin_m: float = 0.5) -> float:
    """Air gap a path must keep from a stem surface.

    Half the rotor span plus a margin. N4's sweep is the calibration: 1.10 m of
    centre-clearance (0.50 m of air) costs 1.016x the planned distance and 3.00 m
    (2.40 m of air) costs 1.029x — **5x the safety margin for 1.3 % more
    distance** — so plan at the *maximum* clearance the field allows, not the
    minimum. The cliff is at 4.00 m, where the 4.6 m mean spacing asserts itself
    and there is no path at all.
    """
    return rotor_span_m / 2.0 + margin_m


def sample_polyline(
    pts: list[tuple[float, float]], step_m: float = 0.5
) -> tuple[np.ndarray, np.ndarray]:
    """Walk a polyline at ~``step_m``, returning (xs, ys) including both ends.

    Clearance must be scored along the *line*, not at the vertices: the whole
    point of a stem map is the gap between waypoints, and a 25 m leg sampled only
    at its ends can pass straight through a trunk unnoticed.
    """
    if len(pts) < 2:
        arr = np.array(pts, dtype=float).reshape(-1, 2)
        return arr[:, 0], arr[:, 1]
    xs: list[float] = []
    ys: list[float] = []
    for (x0, y0), (x1, y1) in pairwise(pts):
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, math.ceil(d / step_m))
        for k in range(n):
            f = k / n
            xs.append(x0 + f * (x1 - x0))
            ys.append(y0 + f * (y1 - y0))
    xs.append(pts[-1][0])
    ys.append(pts[-1][1])
    return np.asarray(xs), np.asarray(ys)


@dataclass(frozen=True)
class StemClearanceReport:
    """How close a route comes to the mapped stems."""

    min_clearance_m: float
    """Smallest air gap to a stem surface anywhere along the line. Negative means
    the route passes through a trunk."""

    strikes: int
    """Samples whose clearance is below the airframe's requirement."""

    samples: int
    mapped_fraction: float
    """Fraction of the route inside the stem map's footprint. A route that scores
    well by leaving the mapped area has not been shown to be safe — the manifests
    are generated in a band around the *original* route, so outside it there is
    open space a real forest would have filled."""

    p05_clearance_m: float


def assess_stem_clearance(
    pts: list[tuple[float, float]],
    stems: StemMap,
    *,
    required_m: float,
    step_m: float = 0.5,
) -> StemClearanceReport:
    """Score a planned line against a stem map — the N4 acceptance measurement."""
    xs, ys = sample_polyline(pts, step_m)
    if len(stems) == 0 or xs.size == 0:
        return StemClearanceReport(math.inf, 0, int(xs.size), 0.0, math.inf)
    clear = stems.clearance_at(xs, ys)
    min_x, min_y, max_x, max_y = stems.bounds()
    inside = (xs >= min_x) & (xs <= max_x) & (ys >= min_y) & (ys <= max_y)
    return StemClearanceReport(
        min_clearance_m=float(clear.min()),
        strikes=int((clear < required_m).sum()),
        samples=int(clear.size),
        mapped_fraction=float(inside.mean()),
        p05_clearance_m=float(np.percentile(clear, 5)),
    )
