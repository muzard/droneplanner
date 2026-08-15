"""In-house line-of-sight viewshed (Software-Spec §5.1 ``viewshed.py``).

Cumulative visual exposure from a set of observers computed over the **DSM**, so
terrain *and* canopy occlude line-of-sight (Preflight-Design §3.1). This is the
shippable NumPy path — no GRASS/GPL dependency. A ridgeline penalty from the DTM
is added separately in ``fields.terrain`` and combined in the composite.

Algorithm: for each observer, cast rays to every grid-boundary cell and march
each ray outward tracking the running maximum elevation angle; a cell is visible
when its own elevation angle clears that running maximum. Exposure is the
fraction of observers that see each cell, in [0, 1].

VALIDATED 2026-07-28 against an independent implementation (TEST-PLAN A4,
``tests/unit/test_fields_viewshed_reference.py``), which tests each target cell
against the straight 3D segment aimed at *that* cell. Results worth knowing:

* the running-max-**angle** test is algebraically identical to a segment-height
  test on the same ray and cells, so the *only* approximation here is which ray
  and which cells get used;
* rays run to every boundary cell, and those Bresenham lines cover a rectangular
  grid **exhaustively — zero cells go untested**. (An earlier version of this
  docstring claimed radial rays "can miss interior cells". Measured: they do not.
  That was a guess, and it was wrong.)
* what remains is a **bearing error**: a cell is tested against a ray aimed at a
  boundary cell, not at itself, so the occluders considered are up to half a cell
  off-line. Cost measured on the demo AO: **1.0 % of cells disagree**, and mean
  exposure differs by **+0.004** — immaterial to any routing decision.
* the disagreements are **overwhelmingly conservative** (0.94 % of cells
  over-report exposure vs 0.07 % under-report), which is the safe direction for
  a concealment planner: it does not promise cover the terrain fails to provide.
  They concentrate where a sightline grazes a slope — exactly where sub-cell
  terrain detail, which neither model has, would decide the answer.

WHAT IS BEING SEEN — the target elevation (SHORTFALLS item 1)
The occluding surface (the DSM) and the *object being looked at* are two
different things, and conflating them measured concealment for the wrong object.
Until 2026-07-27 every caller used ``target_height_m=0``, i.e. asked "can an
observer see the DSM surface at this cell". The drone does not fly on the
surface: it cruises at ``cruise_agl_m`` above the terrain, or inside the trunk
band under the canopy. Those are radically different objects to hide —

* above the canopy the drone is a silhouette against sky, visible from far
  outside anything that can see the ground beneath it;
* inside the trunk band it is *more* concealed than the canopy top the old field
  scored, because the canopy between it and the observer occludes.

So the target elevation is now a first-class input: ``target_elev_m`` supplies an
absolute (AMSL) elevation per cell — for the drone, ``dtm + planned flight AGL``
— while occlusion still marches over the DSM. ``target_height_m`` remains for
objects that genuinely sit *on* the surface, which is what the laid **cable** is
(``fields.survivability``); the two are deliberately not the same field.

:func:`point_exposure` answers the same question for arbitrary 3D points, which
is how a *flown* trajectory is re-scored at the altitudes it will actually be at
(the raster is evaluated at the planned band altitude, which the climb-limited
smoother and PX4's front-loaded descent both depart from).

VISIBILITY IS A WEIGHT, NOT A BOOLEAN (roadmap C1, 2026-08-11)
A cell's own surface never occludes the object standing on it — which is right
for a drone *over* the canopy and wrong for one *under* it. The consequence was
measured: from a 300 m observer the model called 1570 m of a 3244 m below-canopy
route visible while the aircraft was beneath the crowns, so **sub-canopy
concealment against a look-down observer could not be represented at all**. That
is precisely the threat the below-canopy posture exists to defeat.

:class:`~droneplanner.model.mission.CanopyOcclusion` fixes it by making
visibility a **weight in [0, 1]**: a target under its own crown, on a sightline
steep enough that the ray must cross the crown layer, is seen only through the
stand's gap fraction. Shallow sightlines are untouched — they already cross many
crown cells, and the DSM march occludes them correctly. Pass ``canopy=None`` (the
default at this level) for the pre-C1 boolean geometry, or
:func:`single_viewshed_bool` when the question really is "does a sightline
exist" — which is what the A4 reference validation asks.

AND WHETHER IT IS RESOLVABLE AT THAT RANGE (roadmap C2, 2026-08-11)
There was no range term of any kind: a cell 3 km from an observer scored exactly
like one at 300 m. :class:`~droneplanner.model.mission.DetectionModel` supplies
one — an acquisition angle (3 arcmin, the empirical figure) times an optic
magnification gives a **slant** range beyond which a target of a given span is not
acquired. For this programme's 0.9 m airframe unaided that is **1031 m**.

The three reductions multiply: occlusion x crown gap x range. So "exposure" is now
an approximate *probability that a searching observer acquires the aircraft*, not
"a sightline exists somewhere". ⚠️ **Visual acquisition only** — not radar, not
acoustics (300-500 m for small quadcopters, and largely independent of flight
band), not thermal.

⚠️ The ground field (``fields.composite`` ``ground_los``) deliberately gets **no**
detection model: its target is the laid fiber, which at 0.25-0.28 mm is never
optically resolvable at any range, so a visual model applied to it would be
meaningless — it stays a pure line-of-sight openness proxy and is named for it.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import CanopyOcclusion, DetectionModel
from droneplanner.raster_lines import bresenham


def _boundary_cells(rows: int, cols: int) -> list[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for c in range(cols):
        cells.add((0, c))
        cells.add((rows - 1, c))
    for r in range(rows):
        cells.add((r, 0))
        cells.add((r, cols - 1))
    return sorted(cells)


def _target_surface(
    dsm: np.ndarray, target_height_m: float, target_elev_m: np.ndarray | None
) -> np.ndarray:
    """Absolute elevation of the object being looked at, per cell."""
    if target_elev_m is None:
        return dsm + target_height_m
    if target_elev_m.shape != dsm.shape:
        raise ValueError("target_elev_m must have the same shape as the DSM")
    return target_elev_m


def _range_terms(detection: DetectionModel | None, target_span_m: float) -> tuple[float, float]:
    """``(full_weight_m, zero_weight_m)``, resolved once per viewshed.

    ``(inf, inf)`` disables the gate, which is the pre-C2 behaviour: line of sight
    at any range counts as exposure.
    """
    if detection is None:
        return math.inf, math.inf
    zero = detection.max_range_m(target_span_m)
    return (zero if detection.falloff == "hard" else zero / 2.0), zero


def _range_weight(range_m: float, full_m: float, zero_m: float) -> float:
    """1.0 out to ``full_m``, linear to 0.0 at ``zero_m``, 0 beyond."""
    if range_m <= full_m:
        return 1.0
    if range_m >= zero_m:
        return 0.0
    return (zero_m - range_m) / (zero_m - full_m)


def _canopy_terms(canopy: CanopyOcclusion | None) -> tuple[float, float]:
    """``(gap_fraction, tan(max_elev))``. ``None`` disables the sub-canopy model.

    The threshold is compared as a **tangent** against the sightline's rise/run,
    which is what the march already carries — monotone in the angle, and it saves
    an ``atan`` per cell on the hot loop.
    """
    if canopy is None:
        return 1.0, math.inf
    return (
        canopy.sub_canopy_gap_fraction,
        math.tan(math.radians(canopy.sub_canopy_max_elev_deg)),
    )


def _sub_canopy_weight(
    view_angle: float,
    target_elev: float,
    surface_elev: float,
    eye_elev: float,
    gap: float,
    steep_tan: float,
) -> float:
    """1.0, unless the target is under its own crown on a steep **look-down**.

    Three conditions, and all of them are needed for the ray to have crossed the
    crown layer rather than arrived under it through trunk space:

    * ``target_elev < surface_elev`` — the target really is beneath its own crown;
    * ``eye_elev > surface_elev`` — the observer is *above* that crown, so the ray
      enters the column from the top. Without this the model also attenuates a
      trunk-space-to-trunk-space sightline, where nothing is in the way, and that
      **overstates** concealment — the one direction a concealment planner must
      never err in;
    * the sightline is steeper than the threshold, so it punches through the crown
      near the target instead of grazing in from the side. ``view_angle`` is
      signed (negative looking down) and the first two conditions already force it
      negative, so the magnitude is what is tested.
    """
    if target_elev < surface_elev and eye_elev > surface_elev and abs(view_angle) > steep_tan:
        return gap
    return 1.0


def single_viewshed(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    cell_size_m: float,
    observer_height_m: float = 2.0,
    target_height_m: float = 0.0,
    target_elev_m: np.ndarray | None = None,
    canopy: CanopyOcclusion | None = None,
    detection: DetectionModel | None = None,
    target_span_m: float = 0.0,
) -> np.ndarray:
    """Per-cell visibility **weight** in [0, 1] for one observer (float32).

    ``target_elev_m`` (absolute elevation per cell) overrides ``target_height_m``
    (a uniform offset above the DSM) and is how a target *off* the surface — a
    drone at flight altitude — is scored. Occlusion always marches over the DSM.

    Three independent things can reduce the weight, and they multiply:

    * occlusion — the DSM march, 0 or 1 (roadmap pre-history);
    * ``canopy`` — a cell's own crown hiding a target beneath it (C1);
    * ``detection`` — whether ``target_span_m`` is resolvable at that **slant**
      range (C2).

    With both models absent the result is 0.0/1.0 and identical to the original
    boolean field.
    """
    rows, cols = dsm.shape
    r0, c0 = observer_rc
    target = _target_surface(dsm, target_height_m, target_elev_m)
    visible = np.zeros((rows, cols), dtype=np.float32)
    visible[r0, c0] = 1.0
    eye = float(dsm[r0, c0]) + observer_height_m
    gap, steep_tan = _canopy_terms(canopy)
    full_m, zero_m = _range_terms(detection, target_span_m)

    for rt, ct in _boundary_cells(rows, cols):
        max_angle = -math.inf
        for r, c in bresenham(r0, c0, rt, ct):
            if r == r0 and c == c0:
                continue
            dist = math.hypot(r - r0, c - c0) * cell_size_m
            if dist <= 0.0:
                continue
            if dist > zero_m:
                # Horizontal range only grows along a ray and slant range is never
                # less, so every remaining cell scores zero: stop marching. This is
                # what keeps a detection-limited viewshed cheap on a large AO.
                break
            # The running maximum covers *intervening* cells only: a cell's own
            # surface never occludes the object standing on (or flying over) it.
            # For an object *under* that surface the crown still does — which is
            # what _sub_canopy_weight restores.
            surface = float(dsm[r, c])
            target_elev = float(target[r, c])
            terrain_angle = (surface - eye) / dist
            view_angle = (target_elev - eye) / dist
            if view_angle >= max_angle:
                slant = math.hypot(dist, target_elev - eye)
                visible[r, c] = _range_weight(slant, full_m, zero_m) * _sub_canopy_weight(
                    view_angle, target_elev, surface, eye, gap, steep_tan
                )
            if terrain_angle > max_angle:
                max_angle = terrain_angle
    return visible


def single_viewshed_bool(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    cell_size_m: float,
    observer_height_m: float = 2.0,
    target_height_m: float = 0.0,
    target_elev_m: np.ndarray | None = None,
) -> np.ndarray:
    """Does *any* line of sight reach each cell (the pre-C1 boolean contract).

    Line-of-sight geometry is a yes/no question and the A4 reference
    implementation answers it that way; the weighted field answers a different
    one — how likely a look down that line is to succeed.
    """
    weights = single_viewshed(
        dsm, observer_rc, cell_size_m, observer_height_m, target_height_m, target_elev_m
    )
    return weights > 0.0


def cumulative_exposure(
    dsm: np.ndarray,
    grid: Grid,
    observers: list[Coordinate],
    observer_height_m: float = 2.0,
    target_height_m: float = 0.0,
    target_elev_m: np.ndarray | None = None,
    canopy: CanopyOcclusion | None = None,
    detection: DetectionModel | None = None,
    target_span_m: float = 0.0,
) -> np.ndarray:
    """Mean per-observer visibility weight for each cell, in [0, 1] (float32).

    With no observers, returns an all-zero field (nothing is exposed). See
    :func:`single_viewshed` for ``target_elev_m`` — pass it to score the drone at
    its planned flight altitude rather than the ground under it — for ``canopy``,
    which lets a stand hide what flies beneath it, and for ``detection``, which
    asks whether the thing is resolvable at that range at all.
    """
    rows, cols = dsm.shape
    if not observers:
        return np.zeros((rows, cols), dtype=np.float32)

    accum = np.zeros((rows, cols), dtype=np.float32)
    for obs in observers:
        rc = grid.world_to_pixel(obs)
        accum += single_viewshed(
            dsm,
            rc,
            grid.cell_size_m,
            observer_height_m,
            target_height_m,
            target_elev_m,
            canopy,
            detection,
            target_span_m,
        )
    return (accum / float(len(observers))).astype(np.float32)


def _point_weight(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    eye_elev_m: float,
    target_rc: tuple[int, int],
    target_elev_m: float,
    cell_size_m: float,
    canopy: CanopyOcclusion | None = None,
    detection: DetectionModel | None = None,
    target_span_m: float = 0.0,
) -> float:
    """Visibility weight in [0, 1] from one observer to one 3D point.

    The single-point twin of :func:`single_viewshed`, including its sub-canopy and
    detection-range rules — a trajectory re-score and the raster field must not
    disagree about whether a crown is in the way or whether the target is
    resolvable at that range.
    """
    r0, c0 = observer_rc
    rt, ct = target_rc
    gap, steep_tan = _canopy_terms(canopy)
    full_m, zero_m = _range_terms(detection, target_span_m)
    max_angle = -math.inf
    for r, c in bresenham(r0, c0, rt, ct):
        if r == r0 and c == c0:
            continue
        dist = math.hypot(r - r0, c - c0) * cell_size_m
        if dist <= 0.0:
            continue
        if (r, c) == (rt, ct):
            view_angle = (target_elev_m - eye_elev_m) / dist
            if view_angle < max_angle:
                return 0.0
            slant = math.hypot(dist, target_elev_m - eye_elev_m)
            return _range_weight(slant, full_m, zero_m) * _sub_canopy_weight(
                view_angle, target_elev_m, float(dsm[r, c]), eye_elev_m, gap, steep_tan
            )
        angle = (float(dsm[r, c]) - eye_elev_m) / dist
        if angle > max_angle:
            max_angle = angle
    return 1.0  # target shares the observer's cell


def point_exposure(
    dsm: np.ndarray,
    grid: Grid,
    observers: list[Coordinate],
    points: Sequence[tuple[float, float, float]],
    observer_height_m: float = 2.0,
    canopy: CanopyOcclusion | None = None,
    detection: DetectionModel | None = None,
    target_span_m: float = 0.0,
) -> np.ndarray:
    """Mean per-observer visibility weight for each ``(x, y, z_amsl)`` point.

    The raster :func:`cumulative_exposure` scores every cell at one assumed
    altitude; this scores specific 3D points, so a trajectory can be re-scored at
    the altitudes it is actually flown at. Cost is
    ``O(observers * points * ray_len)`` — cheap next to a full viewshed.
    """
    n = len(points)
    if n == 0 or not observers:
        return np.zeros(n, dtype=np.float32)

    accum = np.zeros(n, dtype=np.float32)
    cell = grid.cell_size_m
    for obs in observers:
        obs_rc = grid.world_to_pixel(obs)
        eye = float(dsm[obs_rc]) + observer_height_m
        for i, (x, y, z) in enumerate(points):
            rc = grid.world_to_pixel(Coordinate(x=x, y=y))
            accum[i] += _point_weight(
                dsm, obs_rc, eye, rc, z, cell, canopy, detection, target_span_m
            )
    return (accum / float(len(observers))).astype(np.float32)
