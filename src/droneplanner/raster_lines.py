"""Integer line rasterization shared by the viewshed and the planner.

Both the in-house line-of-sight viewshed (``fields.viewshed``) and the
cost-aware any-angle planner (``planner.theta_star``) need the sequence of grid
cells a straight segment passes through. Kept dependency-free (pure Python) so it
stays in the shippable path.
"""

from __future__ import annotations


def bresenham(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
    """Bresenham line from (r0, c0) to (r1, c1) inclusive, in grid cells."""
    cells: list[tuple[int, int]] = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells
