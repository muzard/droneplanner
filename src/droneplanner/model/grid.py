"""Reference grid: the shared 2.5D raster geometry all fields align to.

Every cost/constraint field is a ``float32`` array on this grid, and the planner
works in (row, col) space, converting to/from the mission's projected world CRS
through the grid's affine transform. Keeping one grid for all fields is what
makes the composite cost surface well-defined (Software-Spec §4 "aligned grid,
shared transform").
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from droneplanner.model.geometry import Coordinate


@dataclass(frozen=True)
class Grid:
    """An aligned raster grid with an affine world<->pixel transform.

    ``transform`` is ``(a, b, c, d, e, f)`` mapping pixel (col, row) to world:
    ``world_x = a*col + b*row + c``, ``world_y = d*col + e*row + f``. North-up
    grids have ``b == d == 0`` and ``e < 0``.
    """

    transform: tuple[float, float, float, float, float, float]
    rows: int
    cols: int
    crs: str

    @property
    def cell_size_m(self) -> float:
        """Nominal cell size (meters); mean of the two axis scales."""
        a, b, d, e = self.transform[0], self.transform[1], self.transform[3], self.transform[4]
        sx = math.hypot(a, d)
        sy = math.hypot(b, e)
        return (sx + sy) / 2.0

    def pixel_to_world(self, row: float, col: float) -> Coordinate:
        """World coordinate of a pixel's *center* (row, col may be fractional)."""
        a, b, c, d, e, f = self.transform
        cc, rr = col + 0.5, row + 0.5
        return Coordinate(x=a * cc + b * rr + c, y=d * cc + e * rr + f)

    def world_to_pixel(self, coord: Coordinate) -> tuple[int, int]:
        """(row, col) cell index containing ``coord`` (floored, clamped in-grid)."""
        a, b, c, d, e, f = self.transform
        det = a * e - b * d
        if det == 0.0:
            raise ValueError("degenerate grid transform (zero determinant)")
        wx, wy = coord.x - c, coord.y - f
        col = (e * wx - b * wy) / det
        row = (-d * wx + a * wy) / det
        ri = min(max(math.floor(row), 0), self.rows - 1)
        ci = min(max(math.floor(col), 0), self.cols - 1)
        return ri, ci

    def contains(self, coord: Coordinate) -> bool:
        """True if ``coord`` falls inside the grid extent."""
        a, b, c, d, e, f = self.transform
        det = a * e - b * d
        wx, wy = coord.x - c, coord.y - f
        col = (e * wx - b * wy) / det
        row = (-d * wx + a * wy) / det
        return 0.0 <= col < self.cols and 0.0 <= row < self.rows
