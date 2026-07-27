"""How much lateral freedom the route can grant without giving up its concealment.

WHY THIS EXISTS (TEST-PLAN A1)
The flight stack does not track the planned polyline to the metre — the route is
a corridor to follow approximately, and obstacle avoidance happens onboard
("enter the forest patch here and exit here", user 2026-07-27). But every
concealment number in this planner is scored **along the centreline**, i.e. along
a line the aircraft will not fly. A plan that promises cover on a line and grants
50 m of freedom around it is promising something it has not measured.

So the corridor becomes an explicit output: each vertex carries the half-width it
is granting, and exposure is re-scored over the swept area rather than the line.

THE CONSTRAINT THAT SHAPES IT (user, 2026-07-28)
> the corridors should still follow some sources of concealment (like a forest or
> a small depression inside it), so at least it couldn't expand outside of a
> forest patch

A constant width cannot express that. Instead the half-width at a point is its
**distance to the nearest cell that would lose the concealment it currently
has** — so the corridor naturally stops at the edge of a stand, at the lip of a
depression, or at an obstacle, and is wide only where there is genuinely room to
wander without being seen. Three conditions, all measured from the fields the
route was planned on:

* **never get more exposed** — a deviation may not enter ground more visible
  than the vertex it left. Note this is *relative*, not a fixed threshold: an
  absolute "exposure <= 0.5" rule would let a corridor drawn around a vertex
  nobody can see reach into ground one observer watches, which is exactly the
  promise-what-was-not-measured problem this module exists to fix.
* **stay in the patch** — a vertex flown *under canopy* may not deviate into a
  cell that is not under canopy, even if that cell happens to be unobserved.
  Leaving the stand changes the flight regime (the aircraft would have to climb
  over what it was hiding inside), so it is not a deviation, it is a different
  plan. This is the clause that answers the requirement above, and it binds
  independently of exposure.
* **stay navigable** — never into an obstacle.

Where the centreline is **already exposed** — crossing a clearing, say — the
first clause has nothing left to protect, so the width falls back to the other
two. That is deliberate: the corridor reports the freedom that exists, and on an
exposed crossing the freedom is real even though the concealment is not. The
exposure score says the latter separately.

⚠️ **Resolution bounds this.** Widths come from a distance transform of the
corridor grid, so they cannot be finer than a cell — on the 25 m demo AO a
"20 m corridor" is one cell of slack, and the numbers should be read as
quantised. A real 1-2 m AO would resolve this properly.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from droneplanner.fields.composite import CostSurface
from droneplanner.model.geometry import Coordinate

# Codes emitted by ``fields.altitude.band_raster``: 2 = below canopy, 3 = inside a
# lidar-measured trunk window. Both mean "flying inside a stand".
_UNDER_CANOPY_CODES = (2, 3)


@dataclass(frozen=True)
class CorridorReport:
    """The lateral freedom a route grants, and what it costs in concealment."""

    half_width_m: tuple[float, ...]
    """Per-vertex corridor half-width."""

    mean_exposure: float
    """Drone exposure averaged over the swept corridor, not the centreline."""

    max_exposure: float
    cells: int
    """Corridor area, in grid cells — the sample size behind the two means above."""

    swept: np.ndarray
    """Boolean mask of the cells the flight stack may use.

    This is the corridor as a *region*, which is what makes gate reduction
    decidable: a straight line between two gates is an acceptable substitute for
    the planned path between them exactly when it stays inside this mask.
    """

    @property
    def min_half_width_m(self) -> float:
        return min(self.half_width_m, default=0.0)

    @property
    def median_half_width_m(self) -> float:
        return float(np.median(self.half_width_m)) if self.half_width_m else 0.0


def under_canopy_mask(surface: CostSurface) -> np.ndarray:
    """Cells where the drone is flying *inside* a stand, and navigable."""
    bands = surface.components.get("flight_band")
    if bands is None:
        return np.zeros(surface.cost.shape, dtype=bool)
    mask = np.isin(bands.astype(np.int32), _UNDER_CANOPY_CODES)
    if surface.obstacle is not None:
        mask = mask & ~surface.obstacle
    return mask


def _admissible_radius(mask: np.ndarray, cell_size_m: float, unbounded_m: float) -> np.ndarray:
    """Metres a cell may be left in any direction while staying inside ``mask``.

    The distance transform gives the distance **to** the nearest excluded cell,
    so a corridor of exactly that radius would reach it. Stop one cell short: a
    cell whose neighbour is excluded has no freedom at all, which is the correct
    answer and not what the raw transform says.

    ``unbounded_m`` is returned where the mask excludes nothing — the constraint
    does not bind there, and 0 would claim the opposite.
    """
    if not mask.any():
        return np.zeros(mask.shape, dtype=np.float32)
    if mask.all():
        return np.full(mask.shape, unbounded_m, dtype=np.float32)
    edt: np.ndarray = ndimage.distance_transform_edt(mask)
    return np.maximum((edt - 1.0) * cell_size_m, 0.0).astype(np.float32)


def _distance_to_worse_exposure(
    exposure: np.ndarray, cell_size_m: float, unbounded_m: float
) -> dict[float, np.ndarray]:
    """For each exposure level present, metres to the nearest MORE exposed cell.

    Admissibility has to be **relative to where you are**, not an absolute
    threshold. With two observers an absolute "exposure <= 0.5" admits cells that
    one observer can see — so a corridor drawn around a vertex that *nobody* can
    see would happily reach into ground that is watched, which is precisely the
    promise-what-was-not-measured problem this module exists to fix.

    Exposure is a fraction of observers, so it takes few distinct values and one
    distance transform per level covers the whole route.
    """
    return {
        float(level): _admissible_radius(exposure <= level, cell_size_m, unbounded_m)
        for level in np.unique(exposure)
    }


def assess_corridor(
    surface: CostSurface,
    points: list[Coordinate],
    *,
    max_half_width_m: float,
) -> CorridorReport:
    """Score the lateral freedom around a route, and the exposure it implies."""
    grid = surface.grid
    exposure = surface.components["exposure"]
    under_canopy = under_canopy_mask(surface)
    cap = max_half_width_m

    navigable = (
        ~surface.obstacle if surface.obstacle is not None else np.ones(exposure.shape, dtype=bool)
    )
    d_worse = _distance_to_worse_exposure(exposure, grid.cell_size_m, cap)
    d_canopy = _admissible_radius(under_canopy, grid.cell_size_m, cap)
    d_navigable = _admissible_radius(navigable, grid.cell_size_m, cap)

    widths: list[float] = []
    swept = np.zeros(exposure.shape, dtype=bool)
    rows, cols = exposure.shape
    for p in points:
        rc = grid.world_to_pixel(p)
        # Never deviate into ground more exposed than where you already are...
        width = float(d_worse[float(exposure[rc])][rc])
        # ...never leave the stand you are hiding inside...
        if under_canopy[rc]:
            width = min(width, float(d_canopy[rc]))
        # ...and never into an obstacle.
        width = min(width, float(d_navigable[rc]), cap)
        widths.append(width)

        # Mark the swept cells so exposure can be scored over the area flown,
        # not the line planned.
        radius_cells = int(width / grid.cell_size_m)
        r0, c0 = rc
        rlo, rhi = max(0, r0 - radius_cells), min(rows, r0 + radius_cells + 1)
        clo, chi = max(0, c0 - radius_cells), min(cols, c0 + radius_cells + 1)
        rr, cc = np.ogrid[rlo:rhi, clo:chi]
        within = ((rr - r0) ** 2 + (cc - c0) ** 2) * grid.cell_size_m**2 <= width**2
        swept[rlo:rhi, clo:chi] |= within

    if not swept.any():  # degenerate: zero-width everywhere, fall back to the line
        for p in points:
            swept[grid.world_to_pixel(p)] = True
    values = exposure[swept]
    return CorridorReport(
        half_width_m=tuple(widths),
        mean_exposure=float(values.mean()),
        max_exposure=float(values.max()),
        cells=int(swept.sum()),
        swept=swept,
    )
