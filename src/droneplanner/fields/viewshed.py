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
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

from droneplanner.fields.sectors import (
    element_bearing_weight,
    element_mask,
    resolve_threat_elements,
)
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import (
    DEFAULT_OFF_AXIS_VIGILANCE,
    CanopyOcclusion,
    DetectionModel,
    ThreatSector,
)
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


def _range_weights(slant: np.ndarray, full_m: float, zero_m: float) -> np.ndarray:
    """:func:`_range_weight` over an array."""
    if math.isinf(full_m):
        return np.ones_like(slant, dtype=np.float64)
    ramp = (zero_m - slant) / max(zero_m - full_m, 1e-12)
    return np.where(slant <= full_m, 1.0, np.where(slant >= zero_m, 0.0, ramp))


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
    sector_mask: np.ndarray | None = None,
    trunk_space: TrunkSpace | None = None,
    gap_map: np.ndarray | None = None,
    medium: CrownMedium | None = None,
) -> np.ndarray:
    """Per-cell visibility **weight** in [0, 1] for one observer (float32).

    ``target_elev_m`` (absolute elevation per cell) overrides ``target_height_m``
    (a uniform offset above the DSM) and is how a target *off* the surface — a
    drone at flight altitude — is scored. Occlusion always marches over the DSM.

    Four independent things can reduce the weight, and they multiply:

    * occlusion — the DSM march, 0 or 1 (roadmap pre-history);
    * ``canopy`` — a cell's own crown hiding a target beneath it (C1);
    * ``detection`` — whether ``target_span_m`` is resolvable at that **slant**
      range (C2);
    * ``sector_mask`` — whether the observer is searching that bearing at all
      (C3; build it with ``fields.sectors.element_mask``).

    With all of them absent the result is 0.0/1.0 and identical to the original
    boolean field.

    The sector mask is applied to the finished field and **not** to the march,
    which is the whole point: occlusion is geometry and does not care where an
    observer is looking, so terrain outside the arc must still raise the running
    horizon for cells inside it.
    """
    rows, cols = dsm.shape
    r0, c0 = observer_rc
    target = _target_surface(dsm, target_height_m, target_elev_m)
    visible = np.zeros((rows, cols), dtype=np.float32)
    visible[r0, c0] = 1.0
    eye = _eye_elev(dsm, medium, observer_rc, observer_height_m)
    gap, steep_tan = _canopy_terms(canopy)
    full_m, zero_m = _range_terms(detection, target_span_m)
    if medium is not None:
        # Crown transmission replaces the horizon march, the sub-canopy rule and the
        # trunk-space pass: all three are special cases of one line integral.
        visible = _medium_viewshed(
            medium, dsm, observer_rc, eye, target, cell_size_m, full_m, zero_m
        )
        return _apply_sector(visible, sector_mask, dsm.shape)

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
                cell_gap = gap if gap_map is None else float(gap_map[r, c])
                visible[r, c] = _range_weight(slant, full_m, zero_m) * _sub_canopy_weight(
                    view_angle, target_elev, surface, eye, cell_gap, steep_tan
                )
            if terrain_angle > max_angle:
                max_angle = terrain_angle
    if trunk_space is not None:
        # Under-crown sightlines the horizon march cannot represent: per target,
        # within range, only where that beats what the march already found.
        reach = math.ceil(min(trunk_space.max_range_m, zero_m) / cell_size_m)
        for r in range(max(r0 - reach, 0), min(r0 + reach + 1, rows)):
            for c in range(max(c0 - reach, 0), min(c0 + reach + 1, cols)):
                if visible[r, c] >= 1.0:
                    continue
                t = _trunk_space_weight(
                    trunk_space, dsm, (r0, c0), eye, (r, c), float(target[r, c]), cell_size_m
                )
                if t > 0.0:
                    slant = math.hypot(
                        math.hypot(r - r0, c - c0) * cell_size_m, float(target[r, c]) - eye
                    )
                    w = t * _range_weight(slant, full_m, zero_m)
                    if w > visible[r, c]:
                        visible[r, c] = w
    return _apply_sector(visible, sector_mask, dsm.shape)


def _eye_elev(
    dsm: np.ndarray,
    medium: CrownMedium | None,
    observer_rc: tuple[int, int],
    observer_height_m: float,
) -> float:
    """Where the observer's eye is.

    With the DSM as a wall the eye has to sit on the surface (``dsm + h``): moved
    down into a stand it would be blind to everything, including a drone in the same
    stand's trunk space. Under crown transmission the canopy is a medium the eye can
    look through, so the observer stands on the TERRAIN (``dtm + h``) — a 2 m
    dismount in a 22 m stand is at 2 m, not in the treetops (SHORTFALLS).
    """
    ground = dsm if medium is None else medium.dtm
    return float(ground[observer_rc]) + observer_height_m


def _apply_sector(
    visible: np.ndarray, sector_mask: np.ndarray | None, shape: tuple[int, ...]
) -> np.ndarray:
    if sector_mask is None:
        return visible
    if sector_mask.shape != shape:
        raise ValueError("sector_mask must have the same shape as the DSM")
    out: np.ndarray = visible * sector_mask.astype(np.float32)
    return out


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


@dataclass(frozen=True)
class TrunkSpace:
    """What a sightline under the crowns needs (``CanopyOcclusion.trunk_space_sightlines``)."""

    dtm: np.ndarray
    crown_base_agl_m: np.ndarray
    stems_per_m2: np.ndarray
    trunk_diameter_m: float = 0.25
    max_range_m: float = 300.0
    open_observer_m: float = 2.0
    """An observer counts as standing on open ground if the canopy at its cell is lower."""


def _trunk_space_weight(
    ts: TrunkSpace,
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    eye_elev_m: float,
    target_rc: tuple[int, int],
    target_elev_m: float,
    cell_size_m: float,
) -> float:
    """Transmittance of a sightline that passes under the crowns, or 0.

    Walk the line cell by cell: above the canopy is clear; under the crown base the
    trunks attenuate it (Poisson stems: exp(-n d L)); in the crown layer, or below
    the terrain, it is blocked. Only for observers on open ground, targets below
    their own canopy, within ``max_range_m``.
    """
    r0, c0 = observer_rc
    rt, ct = target_rc
    if float(dsm[r0, c0]) - float(ts.dtm[r0, c0]) >= ts.open_observer_m:
        return 0.0
    if target_elev_m >= float(dsm[rt, ct]):
        return 0.0  # above its canopy: the ordinary march already scores it
    total = math.hypot(rt - r0, ct - c0) * cell_size_m
    if total <= 0.0 or total > ts.max_range_m:
        return 0.0
    transmit = 1.0
    prev_dist = 0.0
    for r, c in bresenham(r0, c0, rt, ct):
        if r == r0 and c == c0:
            continue
        dist = math.hypot(r - r0, c - c0) * cell_size_m
        h = eye_elev_m + (target_elev_m - eye_elev_m) * (dist / total)
        ground = float(ts.dtm[r, c])
        if h < ground:
            return 0.0
        top = float(dsm[r, c])
        if h < top:
            if h > ground + float(ts.crown_base_agl_m[r, c]):
                return 0.0  # through the crown layer
            transmit *= math.exp(
                -float(ts.stems_per_m2[r, c]) * ts.trunk_diameter_m * (dist - prev_dist)
            )
        prev_dist = dist
    return transmit


@dataclass(frozen=True)
class CrownMedium:
    """The canopy as an attenuating medium (``CanopyOcclusion.crown_transmission``).

    Per cell: ``crown_k_per_m`` in the crown layer [crown base, DSM top] and
    ``trunk_k_per_m`` from the crown base down to the ground; zero where there is
    no canopy. Build it with ``fields.composite.crown_medium_for``.
    """

    dtm: np.ndarray
    crown_base_m: np.ndarray
    """Absolute (AMSL) crown-base elevation."""
    crown_k_per_m: np.ndarray
    trunk_k_per_m: np.ndarray

    @classmethod
    def build(
        cls,
        dtm: np.ndarray,
        dsm: np.ndarray,
        crown_base_agl_m: np.ndarray,
        gap: np.ndarray | float,
        stems_per_m2: np.ndarray | float,
        trunk_diameter_m: float = 0.25,
        open_below_m: float = 2.0,
    ) -> CrownMedium:
        """Calibrate the crown attenuation so a VERTICAL pass through a cell's whole
        crown layer transmits exactly its gap probability.

        A slant path of length ``s`` through crown depth ``D`` then transmits
        ``gap^(s/D)`` (turbid medium). For clumped conifer crowns the effective
        clumping index rises off nadir, so real slant gaps are *smaller* than this
        predicts — the model errs toward exposure, the safe direction here.
        """
        dtm = np.asarray(dtm, dtype=np.float64)
        dsm = np.asarray(dsm, dtype=np.float64)
        chm = np.clip(dsm - dtm, 0.0, None)
        treed = chm >= open_below_m
        base_agl = np.clip(np.asarray(crown_base_agl_m, dtype=np.float64), 0.0, chm)
        depth = np.maximum(chm - base_agl, 0.5)
        p = np.clip(np.broadcast_to(np.asarray(gap, dtype=np.float64), chm.shape), 1e-4, 1.0)
        crown_k = np.where(treed, -np.log(p) / depth, 0.0)
        stems = np.broadcast_to(np.asarray(stems_per_m2, dtype=np.float64), chm.shape)
        trunk_k = np.where(treed, stems * trunk_diameter_m, 0.0)
        return cls(
            dtm=dtm,
            crown_base_m=dtm + base_agl,
            crown_k_per_m=crown_k,
            trunk_k_per_m=trunk_k,
        )


SAMPLES_PER_CELL = 2
"""Pieces per cell of horizontal run in :func:`crown_transmittance` (each piece
looks up its midpoint's cell)."""


def _layer_length(
    h_a: np.ndarray, h_b: np.ndarray, lo: np.ndarray, hi: np.ndarray, length: np.ndarray
) -> np.ndarray:
    """3D length of a straight piece (heights ``h_a`` -> ``h_b``, length ``length``)
    lying between ``lo`` and ``hi``: exact for a linear height, so a steep line's
    short crossing of a crown layer is resolved however few pieces there are."""
    top = np.maximum(h_a, h_b)
    bot = np.minimum(h_a, h_b)
    span = top - bot
    overlap = np.clip(np.minimum(top, hi) - np.maximum(bot, lo), 0.0, None)
    flat = span < 1e-9
    inside_flat = (bot >= lo) & (bot < hi)
    frac = np.where(flat, inside_flat.astype(np.float64), overlap / np.where(flat, 1.0, span))
    out: np.ndarray = frac * length
    return out


def crown_transmittance(
    medium: CrownMedium,
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    eye_elev_m: float,
    target_r: np.ndarray,
    target_c: np.ndarray,
    target_elev_m: np.ndarray,
    cell_size_m: float,
) -> np.ndarray:
    """Transmittance in [0, 1] of the straight sightline to each target.

    Each line is cut into ``SAMPLES_PER_CELL`` pieces per cell of its own horizontal
    run (so a target's answer does not depend on which other targets share the
    call — the raster and the flown re-score agree exactly). Each piece takes its
    midpoint's cell; the lengths it spends in that cell's crown layer and trunk
    space are integrated exactly from the heights at its two ends. Any piece dipping
    below its ground blocks the line.
    """
    r0, c0 = observer_rc
    tr = np.asarray(target_r, dtype=np.float64).ravel()
    tc = np.asarray(target_c, dtype=np.float64).ravel()
    tz = np.asarray(target_elev_m, dtype=np.float64).ravel()
    out = np.ones(tr.shape, dtype=np.float64)
    if tr.size == 0:
        return out
    run = np.hypot(tr - r0, tc - c0)
    pieces = np.maximum(np.ceil(run * SAMPLES_PER_CELL), 1.0)
    rows, cols = dsm.shape
    # Distance bands keep short lines from looping over the longest one's pieces.
    band_edges = [0.0, 8.0, 32.0, 128.0, 512.0, math.inf]
    for lo, hi in pairwise(band_edges):
        sel = np.flatnonzero((run > lo) & (run <= hi))
        if sel.size == 0:
            continue
        k_i = pieces[sel]
        dr = tr[sel] - r0
        dc = tc[sel] - c0
        dz = tz[sel] - eye_elev_m
        piece_len = np.hypot(run[sel] * cell_size_m, dz) / k_i
        tau = np.zeros(sel.size)
        blocked = np.zeros(sel.size, dtype=bool)
        for i in range(int(k_i.max())):
            live = i < k_i
            f_a = np.minimum(i / k_i, 1.0)
            f_b = np.minimum((i + 1) / k_i, 1.0)
            f_m = 0.5 * (f_a + f_b)
            rr = np.clip(np.rint(r0 + f_m * dr).astype(np.int64), 0, rows - 1)
            cc = np.clip(np.rint(c0 + f_m * dc).astype(np.int64), 0, cols - 1)
            h_a = eye_elev_m + f_a * dz
            h_b = eye_elev_m + f_b * dz
            ground = medium.dtm[rr, cc]
            base = medium.crown_base_m[rr, cc]
            top = dsm[rr, cc]
            blocked |= live & (np.minimum(h_a, h_b) < ground)
            crown = _layer_length(h_a, h_b, base, top, piece_len)
            trunks = _layer_length(h_a, h_b, ground, np.minimum(base, top), piece_len)
            tau += np.where(
                live,
                crown * medium.crown_k_per_m[rr, cc] + trunks * medium.trunk_k_per_m[rr, cc],
                0.0,
            )
        out[sel] = np.where(blocked, 0.0, np.exp(-tau))
    return out


def _medium_viewshed(
    medium: CrownMedium,
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    eye: float,
    target: np.ndarray,
    cell_size_m: float,
    full_m: float,
    zero_m: float,
) -> np.ndarray:
    """:func:`single_viewshed` under ``crown_transmission``: transmittance x range."""
    rows, cols = dsm.shape
    r0, c0 = observer_rc
    rr, cc = np.mgrid[0:rows, 0:cols]
    horiz = np.hypot(rr - r0, cc - c0) * cell_size_m
    slant = np.hypot(horiz, target - eye)
    rng = _range_weights(slant, full_m, zero_m)
    visible = np.zeros((rows, cols), dtype=np.float32)
    live = np.flatnonzero((rng > 0.0).ravel())
    t = crown_transmittance(
        medium,
        dsm,
        observer_rc,
        eye,
        rr.ravel()[live],
        cc.ravel()[live],
        target.ravel()[live],
        cell_size_m,
    )
    visible.ravel()[live] = (t * rng.ravel()[live]).astype(np.float32)
    visible[r0, c0] = 1.0
    return visible


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
    sectors: Sequence[ThreatSector] = (),
    off_axis_vigilance: float = DEFAULT_OFF_AXIS_VIGILANCE,
    trunk_space: TrunkSpace | None = None,
    gap_map: np.ndarray | None = None,
    medium: CrownMedium | None = None,
) -> np.ndarray:
    """Mean per-observer visibility weight for each cell, in [0, 1] (float32).

    ``medium`` (``CanopyOcclusion.crown_transmission``) scores every sightline by
    transmittance through the canopy instead of the DSM horizon, and supersedes
    ``trunk_space`` and ``gap_map`` (both are folded into it).

    ``gap_map`` replaces ``canopy.sub_canopy_gap_fraction`` per cell with a measured
    value (``layers.canopy_gap_fraction``).

    With no observers, returns an all-zero field (nothing is exposed). See
    :func:`single_viewshed` for ``target_elev_m`` — pass it to score the drone at
    its planned flight altitude rather than the ground under it — for ``canopy``,
    which lets a stand hide what flies beneath it, and for ``detection``, which
    asks whether the thing is resolvable at that range at all.

    ``sectors`` (C3) says which way each observer is facing. Sectors that name an
    observer restrict it; sectors that name none are threat *bearings* with no
    sensor and are ignored here — they belong to
    :func:`~droneplanner.fields.sectors.openness_to_threat_sectors`. A sector
    naming an unlisted point adds that observer rather than dropping the sector.
    """
    rows, cols = dsm.shape
    if not observers and not any(s.observer is not None for s in sectors):
        return np.zeros((rows, cols), dtype=np.float32)

    views = resolve_threat_elements(observers, sectors, off_axis_vigilance)
    accum = np.zeros((rows, cols), dtype=np.float32)
    for group in views:
        rc = grid.world_to_pixel(group[0].observer)
        # Max over one observer's elements, mean over observers: an observer
        # watching two arcs through two optics is still one observer.
        seen = np.zeros((rows, cols), dtype=np.float32)
        for element in group:
            np.maximum(
                seen,
                single_viewshed(
                    dsm,
                    rc,
                    grid.cell_size_m,
                    observer_height_m,
                    target_height_m,
                    target_elev_m,
                    canopy,
                    detection if element.detection is None else element.detection,
                    target_span_m,
                    element_mask(grid, element),
                    trunk_space,
                    gap_map,
                    medium,
                ),
                out=seen,
            )
        accum += seen
    return (accum / float(len(views))).astype(np.float32)


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
    trunk_space: TrunkSpace | None = None,
    gap_map: np.ndarray | None = None,
) -> float:
    """Visibility weight in [0, 1] from one observer to one 3D point.

    The single-point twin of :func:`single_viewshed`, including its sub-canopy and
    detection-range rules — a trajectory re-score and the raster field must not
    disagree about whether a crown is in the way or whether the target is
    resolvable at that range.
    """
    base = _march_point_weight(
        dsm,
        observer_rc,
        eye_elev_m,
        target_rc,
        target_elev_m,
        cell_size_m,
        canopy,
        detection,
        target_span_m,
        gap_map,
    )

    if trunk_space is None or base >= 1.0:
        return base
    full_m, zero_m = _range_terms(detection, target_span_m)
    r0, c0 = observer_rc
    rt, ct = target_rc
    total = math.hypot(rt - r0, ct - c0) * cell_size_m
    if total > min(trunk_space.max_range_m, zero_m):
        return base
    t = _trunk_space_weight(
        trunk_space, dsm, observer_rc, eye_elev_m, target_rc, target_elev_m, cell_size_m
    )
    if t <= 0.0:
        return base
    slant = math.hypot(total, target_elev_m - eye_elev_m)
    return max(base, t * _range_weight(slant, full_m, zero_m))


def _medium_point(
    transmit: float,
    observer_rc: tuple[int, int],
    eye_elev_m: float,
    target_rc: tuple[int, int],
    target_elev_m: float,
    cell_size_m: float,
    detection: DetectionModel | None,
    target_span_m: float,
) -> float:
    """Point twin of :func:`_medium_viewshed`: transmittance x range weight."""
    if target_rc == observer_rc:
        return 1.0
    full_m, zero_m = _range_terms(detection, target_span_m)
    horiz = math.hypot(target_rc[0] - observer_rc[0], target_rc[1] - observer_rc[1])
    slant = math.hypot(horiz * cell_size_m, target_elev_m - eye_elev_m)
    return float(transmit) * _range_weight(slant, full_m, zero_m)


def _march_point_weight(
    dsm: np.ndarray,
    observer_rc: tuple[int, int],
    eye_elev_m: float,
    target_rc: tuple[int, int],
    target_elev_m: float,
    cell_size_m: float,
    canopy: CanopyOcclusion | None = None,
    detection: DetectionModel | None = None,
    target_span_m: float = 0.0,
    gap_map: np.ndarray | None = None,
) -> float:
    """The DSM-march half of :func:`_point_weight`."""
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
            cell_gap = gap if gap_map is None else float(gap_map[r, c])
            return _range_weight(slant, full_m, zero_m) * _sub_canopy_weight(
                view_angle, target_elev_m, float(dsm[r, c]), eye_elev_m, cell_gap, steep_tan
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
    sectors: Sequence[ThreatSector] = (),
    off_axis_vigilance: float = DEFAULT_OFF_AXIS_VIGILANCE,
    trunk_space: TrunkSpace | None = None,
    gap_map: np.ndarray | None = None,
    medium: CrownMedium | None = None,
) -> np.ndarray:
    """Mean per-observer visibility weight for each ``(x, y, z_amsl)`` point.

    The raster :func:`cumulative_exposure` scores every cell at one assumed
    altitude; this scores specific 3D points, so a trajectory can be re-scored at
    the altitudes it is actually flown at. Cost is
    ``O(observers * points * ray_len)`` — cheap next to a full viewshed.

    ``sectors`` applies the identical binding and bearing test the raster uses,
    so the flown re-score and the field cannot disagree about which way an
    observer faces.
    """
    n = len(points)
    if n == 0 or (not observers and not any(s.observer is not None for s in sectors)):
        return np.zeros(n, dtype=np.float32)

    views = resolve_threat_elements(observers, sectors, off_axis_vigilance)
    accum = np.zeros(n, dtype=np.float32)
    cell = grid.cell_size_m
    for group in views:
        obs_rc = grid.world_to_pixel(group[0].observer)
        eye = _eye_elev(dsm, medium, obs_rc, observer_height_m)
        through = None
        if medium is not None:
            # One vectorised pass per observer; the per-element loop then only
            # applies range and attention, exactly as the raster does.
            rcs = [grid.world_to_pixel(Coordinate(x=x, y=y)) for x, y, _ in points]
            through = crown_transmittance(
                medium,
                dsm,
                obs_rc,
                eye,
                np.array([r for r, _ in rcs]),
                np.array([c for _, c in rcs]),
                np.array([z for _, _, z in points]),
                cell,
            )
        for i, (x, y, z) in enumerate(points):
            rc = grid.world_to_pixel(Coordinate(x=x, y=y))
            seen = 0.0
            for element in group:
                # A sector scales the weight rather than gating it: off-axis the
                # observer is less attentive, not blind (C3 + off_axis_vigilance).
                attention = element_bearing_weight(element, x, y)
                if attention <= 0.0:
                    continue
                det = detection if element.detection is None else element.detection
                if through is not None:
                    seen = max(
                        seen,
                        attention
                        * _medium_point(through[i], obs_rc, eye, rc, z, cell, det, target_span_m),
                    )
                    continue
                seen = max(
                    seen,
                    attention
                    * _point_weight(
                        dsm,
                        obs_rc,
                        eye,
                        rc,
                        z,
                        cell,
                        canopy,
                        detection if element.detection is None else element.detection,
                        target_span_m,
                        trunk_space,
                        gap_map,
                    ),
                )
            accum[i] += seen
    return (accum / float(len(views))).astype(np.float32)
