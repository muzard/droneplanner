"""Built-in procedural demo area of operations.

Lets the M1 planner run — and the visualizer/interactive explorer show it — with
zero input data. The terrain is a small analytic scene: two observer hills and a
ridge with an off-center saddle between start and end, so the naive straight line
crosses an exposed crest and the concealment-aware planner has an interesting
detour to find (through the saddle / lower-exposure flank).

This is synthetic scenery for demonstration only; real runs supply a DTM/DSM.
"""

from __future__ import annotations

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import DroneSpec, Mission, SpoolSpec, ThreatModel, Weights

DEMO_CRS = "EPSG:32633"
_X0 = 500_000.0
_Y0 = 5_200_000.0


def _gauss(
    xx: np.ndarray, yy: np.ndarray, cx: float, cy: float, amp: float, sig: float
) -> np.ndarray:
    return amp * np.exp(-(((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * sig**2)))


def demo_terrain(size: int = 160, cell_m: float = 25.0) -> tuple[np.ndarray, np.ndarray, Grid]:
    """Return (dtm, dsm, grid) for the analytic demo AO."""
    rows = cols = size
    yy, xx = np.mgrid[0:rows, 0:cols].astype(np.float64)

    dtm = np.full((rows, cols), 40.0)
    dtm += 0.12 * (xx + yy)  # gentle regional tilt

    # Two observer hills flanking the centre.
    dtm += _gauss(xx, yy, cx=0.42 * cols, cy=0.30 * rows, amp=95.0, sig=0.10 * size)
    dtm += _gauss(xx, yy, cx=0.62 * cols, cy=0.70 * rows, amp=85.0, sig=0.10 * size)

    # A ridge along the anti-diagonal, with a low saddle offset toward one end.
    d_anti = (xx + yy - size) / np.sqrt(2.0)  # signed distance to anti-diagonal
    ridge = 70.0 * np.exp(-(d_anti**2) / (2.0 * (0.06 * size * cell_m / cell_m) ** 2))
    saddle = 1.0 - 0.75 * np.exp(
        -((xx - 0.30 * cols) ** 2 + (yy - 0.70 * rows) ** 2) / (2 * (0.12 * size) ** 2)
    )
    dtm += ridge * saddle

    # Flanking valleys (defilade) to reward.
    dtm -= _gauss(xx, yy, cx=0.25 * cols, cy=0.78 * rows, amp=25.0, sig=0.11 * size)
    dtm -= _gauss(xx, yy, cx=0.78 * cols, cy=0.25 * rows, amp=25.0, sig=0.11 * size)

    dtm = dtm.astype(np.float32)
    dsm = dtm.copy()  # no canopy in the demo; DSM == DTM

    transform = (cell_m, 0.0, _X0, 0.0, -cell_m, _Y0 + rows * cell_m)
    grid = Grid(transform=transform, rows=rows, cols=cols, crs=DEMO_CRS)
    return dtm, dsm, grid


def demo_mission(grid: Grid, weights: Weights | None = None) -> Mission:
    """A demo mission on the demo grid: opposite-corner start/end, two observers."""
    start = grid.pixel_to_world(grid.rows - 12, 12)
    end = grid.pixel_to_world(12, grid.cols - 12)
    observers = (
        grid.pixel_to_world(int(0.30 * grid.rows), int(0.42 * grid.cols)),
        grid.pixel_to_world(int(0.70 * grid.rows), int(0.62 * grid.cols)),
    )
    return Mission(
        name="demo-ao",
        crs=grid.crs,
        start=Coordinate(x=start.x, y=start.y),
        end=Coordinate(x=end.x, y=end.y),
        drone=DroneSpec(
            rotor_span_m=0.9,
            mass_kg=6.5,
            min_turn_radius_m=8.0,
            climb_rate_max_ms=3.0,
            descent_rate_max_ms=2.0,
            cruise_speed_ms=6.0,
        ),
        spool=SpoolSpec(length_m=8000.0, mass_kg=1.6),
        threat=ThreatModel(observers=observers),
        weights=weights or Weights(),
        cruise_agl_m=30.0,
        seed=1,
    )


def build_demo(size: int = 160) -> tuple[Mission, np.ndarray, np.ndarray, Grid]:
    """Convenience: (mission, dtm, dsm, grid) for the demo AO."""
    dtm, dsm, grid = demo_terrain(size=size)
    mission = demo_mission(grid)
    return mission, dtm, dsm, grid
