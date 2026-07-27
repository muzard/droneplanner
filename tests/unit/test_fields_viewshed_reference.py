"""Cross-validation of the in-house viewshed against an independent algorithm.

TEST-PLAN item A4. The production viewshed (``fields/viewshed.py``) is homegrown
and, until 2026-07-28, had never been checked against anything — while every
concealment number in the program rests on it.

**The reference here is a different algorithm, not a refactor.** Production casts
a ray from the observer to every grid-*boundary* cell and marches it outward
tracking a running maximum elevation angle. The reference takes each target cell
independently and asks whether the straight 3D segment from the observer's eye to
*that* cell ever passes below the surface.

Note the two are algebraically identical *given the same ray and the same cells*:

    angle(T) >= angle(i)
    (tz - eye)/d_T >= (dsm_i - eye)/d_i
    eye + (tz - eye) * (d_i/d_T) >= dsm_i          <- the segment test

which is proved numerically in :func:`test_angle_test_equals_segment_test`. So
every disagreement is attributable to *which ray* and *which cells* were used —
that is the approximation, and these tests bound it.
"""

from __future__ import annotations

import math

import numpy as np

from droneplanner.fields.viewshed import _boundary_cells, single_viewshed
from droneplanner.raster_lines import bresenham

OBSERVER_HEIGHT_M = 2.0


def reference_visible(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    target_rc: tuple[int, int],
    target_elev_m: float,
) -> bool:
    """Independent line-of-sight: straight segment to *this* target, sub-cell march."""
    r0, c0 = observer_rc
    rt, ct = target_rc
    if (r0, c0) == (rt, ct):
        return True
    eye = float(dsm[r0, c0]) + OBSERVER_HEIGHT_M
    dr, dc = rt - r0, ct - c0
    steps = max(2, math.ceil(math.hypot(dr, dc) / 0.25))
    for k in range(1, steps):
        f = k / steps
        ri, ci = round(r0 + dr * f), round(c0 + dc * f)
        if (ri, ci) in ((r0, c0), (rt, ct)):
            continue  # a cell never occludes what stands on it
        if eye + (target_elev_m - eye) * f < float(dsm[ri, ci]):
            return False
    return True


def reference_viewshed(dsm: np.ndarray, observer_rc: tuple[int, int]) -> np.ndarray:
    rows, cols = dsm.shape
    return np.array(
        [
            [reference_visible(dsm, observer_rc, (r, c), float(dsm[r, c])) for c in range(cols)]
            for r in range(rows)
        ],
        dtype=bool,
    )


def _flat(size: int) -> np.ndarray:
    return np.zeros((size, size), dtype=np.float32)


def _wall(size: int) -> np.ndarray:
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, size // 2] = 100.0
    return dsm


def _hill(size: int) -> np.ndarray:
    yy, xx = np.mgrid[0:size, 0:size].astype(np.float64)
    c = (size - 1) / 2.0
    sigma = size / 6.0
    return (100.0 * np.exp(-(((xx - c) ** 2 + (yy - c) ** 2) / (2 * sigma**2)))).astype(np.float32)


def _rough(size: int) -> np.ndarray:
    rng = np.random.default_rng(3)
    return (rng.random((size, size)) * 40.0).astype(np.float32)


def test_angle_test_equals_segment_test() -> None:
    """The running-max-angle test IS a segment test — so only ray choice can differ."""
    eye, dist_t, dist_i = 102.0, 250.0, 240.0
    for target_z in (8.0, 25.0, 60.0, 130.0):
        for obstacle_z in (0.0, 25.0, 25.5, 99.0):
            by_angle = (target_z - eye) / dist_t >= (obstacle_z - eye) / dist_i
            by_segment = eye + (target_z - eye) * (dist_i / dist_t) >= obstacle_z
            assert by_angle == by_segment, (target_z, obstacle_z)


def test_every_cell_is_actually_tested() -> None:
    """The module used to warn that radial rays 'can miss interior cells'. Measured: they do not.

    Rays run to *every* boundary cell, and Bresenham lines to a full perimeter
    cover a rectangular grid exhaustively. Zero cells go untested from any
    observer position — so an unvisited-cell default of "concealed" is not a
    failure mode this implementation has.
    """
    size = 41
    for obs in [(size // 2, size // 2), (0, 0), (size // 2, 2), (size - 1, size - 3)]:
        covered = np.zeros((size, size), dtype=bool)
        for rt, ct in _boundary_cells(size, size):
            for r, c in bresenham(obs[0], obs[1], rt, ct):
                covered[r, c] = True
        assert covered.all(), f"observer {obs} leaves {(~covered).sum()} cells untested"


def test_matches_the_reference_exactly_on_flat_ground() -> None:
    """No occlusion anywhere: the answer cannot depend on discretisation."""
    size = 41
    for name, obs in [("centre", (size // 2, size // 2)), ("corner", (0, 0))]:
        dsm = _flat(size)
        prod = single_viewshed(dsm, obs, cell_size_m=10.0)
        assert np.array_equal(prod, reference_viewshed(dsm, obs)), name


def test_a_wall_casts_the_same_shadow_in_both() -> None:
    """The shadow is the physically meaningful claim, and it is exact.

    ⚠️ The wall's *own* extreme corner cells are NOT exact, and the reason is
    worth keeping: the sightline to the top of the far corner grazes past an
    equally tall wall cell slightly nearer to the observer, so the reference
    blocks it — while Bresenham's line to that boundary cell steps around the
    blocker and never tests it. The reference is right there. Production calls
    the cell visible, which over-reports exposure: the conservative direction.
    """
    size = 41
    wall_col = size // 2
    dsm = _wall(size)
    obs = (size // 2, 2)
    prod = single_viewshed(dsm, obs, cell_size_m=10.0)
    ref = reference_viewshed(dsm, obs)

    behind = np.zeros_like(prod)
    behind[:, wall_col + 1 :] = True
    assert not prod[behind].any(), "production sees through the wall"
    assert not ref[behind].any(), "reference sees through the wall"

    off_wall = np.ones_like(prod)
    off_wall[:, wall_col] = False
    assert np.array_equal(prod[off_wall], ref[off_wall])

    disagree = np.argwhere(prod != ref)
    assert {tuple(d) for d in disagree} <= {(0, wall_col), (size - 1, wall_col)}
    for r, c in disagree:
        assert prod[r, c] and not ref[r, c]  # conservative, never the reverse


def test_smooth_terrain_agrees_and_errs_toward_over_reporting_exposure() -> None:
    """On smooth terrain the two discretisations differ — bounded, and in the safe direction.

    Disagreements concentrate where a sightline grazes a slope, i.e. where
    sub-cell terrain detail neither model has would decide the answer. What
    matters for a concealment planner is the *direction*: reporting a cell
    visible when the reference hides it is conservative; the reverse would
    promise concealment the terrain does not provide.
    """
    size = 41
    for name, dsm, obs in [("hill", _hill(size), (2, 2)), ("rough", _rough(size), (1, 1))]:
        prod = single_viewshed(dsm, obs, cell_size_m=10.0)
        ref = reference_viewshed(dsm, obs)
        agreement = float((prod == ref).mean())
        over = int((prod & ~ref).sum())  # production says visible, reference hides
        under = int((~prod & ref).sum())  # production says hidden, reference sees — dangerous
        assert agreement > 0.90, f"{name}: only {agreement:.2%} agreement"
        assert over >= under, f"{name}: {under} dangerous vs {over} conservative disagreements"
