"""Threat fire sectors — where the adversary is actually looking (roadmap C3).

``ThreatSector`` has been in the mission contract since M0 and was **read by
nothing**: ``build_survivability`` took observer *points*, so the premise "the
enemy is over there and we are flying away from them" could not be expressed to
the planner at all. That is why the demo AO ended up with observers flanking the
route, and why the below-canopy posture was tuned against that geometry.

Roadmap C2 promoted this from housekeeping to the live question. Once exposure
became a *detection probability* with a range term, unaided exposure on the demo
AO fell to **0.000 for every flight band** — the route simply stays outside
1031 m — while a 12x observation optic restored the entire pre-C2 picture to
three decimals. So the answer no longer turns on altitude; it turns on **whether
a magnified optic is searching that bearing**, which is exactly what a sector
says. :class:`~droneplanner.model.mission.ThreatSector` therefore carries an
optional ``detection`` override: a spotting scope watching one arc while
dismounts hold the rest is a different threat from either alone, and it is a
statement about the adversary, not a global constant.

Two kinds of sector, and they feed different fields because they answer
different questions:

* **with an observer** — that observer only searches those azimuths, so its
  viewshed contribution is masked to them. This reduces both the drone
  ``exposure`` field and the cable's ``ground_los``: an observer facing away
  sees neither the aircraft nor the cable.
* **without an observer** — a threat *bearing* with no located sensor, which is
  what Preflight-Design §5.1 means by ``openness_to_threat_sectors``: score each
  cell by how much of the threat arc it is masked from by terrain. It feeds
  cable survivability, where "masked from the expected threat azimuths by
  intervening terrain" covers direct fire and near-miss fragmentation as well as
  observation.

AZIMUTH CONVENTION
Compass bearings: 0 deg = grid north (+y), 90 deg = east (+x), increasing
clockwise. An arc sweeps **clockwise** from ``azimuth_start_deg`` to
``azimuth_end_deg``, so 350->20 wraps through north and covers 30 deg while
20->350 covers 330 deg. A degenerate arc (start == end, and the 0->360 that
normalises to it) is read as the **full circle**, never as a zero-width sector:
the safe reading of an ambiguous input is the one that cannot silently erase a
threat.

⚠️ **A sector never adds concealment the terrain does not provide** — it narrows
where an observer is *attending*, which is an intelligence assumption. An
adversary who turns around invalidates the plan, and nothing here models that.
Sectors are for expressing a known posture, not for manufacturing cover.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import (
    DetectionModel,
    ThreatSector,
)

# How far away a bearing-only threat is assumed to observe/engage from. Terrain
# beyond this is not credited with masking the cell, because a threat can stand
# in front of it. There is no principled value — it is a range assumption, and
# the openness field is sensitive to it, so it is a named argument everywhere.
DEFAULT_HORIZON_M = 2000.0
# Azimuth sampling step for the bearing-only openness field. 5 deg over a 90 deg
# sector is 19 rays; finer buys little because the DSM is 25 m here.
DEFAULT_AZIMUTH_STEP_DEG = 5.0


@dataclass(frozen=True)
class ThreatElement:
    """One observer's search commitment: an arc set plus the optic behind it.

    An observer with no sectors gets a single element with empty ``arcs``, which
    means the full circle and reproduces the pre-C3 field exactly. An observer
    named by several sectors gets one element **per distinct detection model**,
    because that is the grouping a viewshed can be computed once for; arcs that
    share an optic are unioned into one element.
    """

    observer: Coordinate
    arcs: tuple[tuple[float, float], ...] = ()
    detection: DetectionModel | None = None
    labels: tuple[str, ...] = ()
    off_axis_vigilance: float = 0.0
    """Weight applied off-axis, as a fraction of the on-axis weight. A sector
    concentrates attention; it does not blind the observer everywhere else
    (``ThreatModel.off_axis_vigilance``)."""

    @property
    def restricted(self) -> bool:
        """True when this element's attention is unevenly distributed at all.

        An element whose off-axis weight is 1.0 searches every bearing equally,
        so it is *not* restricted however many arcs it names — treating it as
        restricted would build a uniform weight raster for nothing.
        """
        return bool(self.arcs) and self.off_axis_vigilance < 1.0


def arc_span_deg(start_deg: float, end_deg: float) -> float:
    """Clockwise width of an arc in degrees; a degenerate arc is the full 360."""
    span = (end_deg - start_deg) % 360.0
    return 360.0 if span == 0.0 else span


def arc_contains(azimuth_deg: float, start_deg: float, end_deg: float) -> bool:
    """Is ``azimuth_deg`` inside the clockwise arc ``start -> end``?"""
    span = arc_span_deg(start_deg, end_deg)
    if span >= 360.0:
        return True
    return ((azimuth_deg - start_deg) % 360.0) <= span


def bearing_deg(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """Compass bearing (0 = +y, clockwise) from one world point to another."""
    return math.degrees(math.atan2(to_x - from_x, to_y - from_y)) % 360.0


def _views_of(
    observer: Coordinate,
    bound: Sequence[ThreatSector],
    off_axis_vigilance: float = 0.0,
) -> tuple[ThreatElement, ...]:
    """One observer's elements: the full circle, or one element per distinct optic."""
    if not bound:
        return (ThreatElement(observer=observer),)
    # Arcs watched through the same detection model share a viewshed; arcs watched
    # through different ones cannot, so they become separate elements.
    groups: list[tuple[DetectionModel | None, list[ThreatSector]]] = []
    for sector in bound:
        for detection, members in groups:
            if detection == sector.detection:
                members.append(sector)
                break
        else:
            groups.append((sector.detection, [sector]))
    return tuple(
        ThreatElement(
            observer=observer,
            arcs=tuple((s.azimuth_start_deg, s.azimuth_end_deg) for s in members),
            detection=detection,
            labels=tuple(s.label for s in members if s.label is not None),
            off_axis_vigilance=off_axis_vigilance,
        )
        for detection, members in groups
    )


def resolve_threat_elements(
    observers: Sequence[Coordinate],
    sectors: Sequence[ThreatSector] = (),
    off_axis_vigilance: float = 0.0,
) -> tuple[tuple[ThreatElement, ...], ...]:
    """Bind sectors to observers: one inner tuple **per observer**, in order.

    Exposure is a **mean over observers** of a **max over that observer's
    elements** — a cell seen through any of an observer's arcs is seen by that
    observer, and one observer holding two optics is not two threats. The nesting
    is what carries that distinction, so callers must not flatten it.

    Rules, all chosen so a supplied sector can never be silently dropped:

    * an observer named by no sector keeps the full circle, which reproduces the
      pre-C3 field exactly (including the denominator, if a mission happens to
      list the same point twice);
    * an observer named by a sector searches **only** its arcs — the unrestricted
      element is replaced, not kept alongside it;
    * a sector naming a point that is not in ``observers`` contributes as an
      **additional** sector-restricted observer rather than being ignored.
      Dropping it would delete a threat the mission author declared, and a
      concealment planner must never err toward promising cover;
    * sectors with no observer are not elements at all — they are bearings, and
      :func:`openness_to_threat_sectors` is where they land.
    """
    bound: dict[tuple[float, float], list[ThreatSector]] = {}
    extra: list[Coordinate] = []
    listed = {(o.x, o.y) for o in observers}
    for sector in sectors:
        if sector.observer is None:
            continue
        key = (sector.observer.x, sector.observer.y)
        if key not in bound:
            bound[key] = []
            if key not in listed:
                extra.append(sector.observer)
        bound[key].append(sector)

    return tuple(
        _views_of(observer, bound.get((observer.x, observer.y), []), off_axis_vigilance)
        for observer in list(observers) + extra
    )


def element_mask(grid: Grid, element: ThreatElement) -> np.ndarray | None:
    """Per-cell attention weight for ``element``, or ``None`` when it is uniform.

    **This is a weight in [0, 1], not a boolean mask.** Cells on a searched bearing
    get 1.0; every other cell gets ``element.off_axis_vigilance`` rather than zero,
    because a search sector says where an observer is *concentrating*, not where
    they are blind (user, 2026-09-14: "realistically observers will be looking
    around, even by accident"). With vigilance 0.0 this reproduces the original
    hard mask exactly.

    The weight is applied to the *result* of a viewshed, never to its march:
    occlusion is geometry and does not care where the observer is looking, so
    intervening terrain must still raise the running horizon.
    """
    if not element.restricted:
        return None
    a, b, c, d, e, f = grid.transform
    rr, cc = np.indices((grid.rows, grid.cols), dtype=np.float64)
    xs = a * (cc + 0.5) + b * (rr + 0.5) + c
    ys = d * (cc + 0.5) + e * (rr + 0.5) + f
    azimuth = np.degrees(np.arctan2(xs - element.observer.x, ys - element.observer.y)) % 360.0

    on_axis = np.zeros((grid.rows, grid.cols), dtype=bool)
    for start, end in element.arcs:
        span = arc_span_deg(start, end)
        if span >= 360.0:
            return None
        on_axis |= ((azimuth - start) % 360.0) <= span
    # The observer's own cell has no bearing from itself; leave it fully weighted
    # so a sector cannot make an observer invisible to itself.
    r0, c0 = grid.world_to_pixel(element.observer)
    on_axis[r0, c0] = True
    return np.where(on_axis, 1.0, element.off_axis_vigilance).astype(np.float32)


def element_bearing_weight(element: ThreatElement, x: float, y: float) -> float:
    """``element``'s attention weight on the bearing of world point ``(x, y)``.

    The single-point twin of :func:`element_mask` — the flown re-score and the
    raster must not disagree about which way an observer is facing, *or* about how
    much they see when facing elsewhere.
    """
    if not element.restricted:
        return 1.0
    if (x, y) == (element.observer.x, element.observer.y):
        return 1.0
    azimuth = bearing_deg(element.observer.x, element.observer.y, x, y)
    on_axis = any(arc_contains(azimuth, start, end) for start, end in element.arcs)
    return 1.0 if on_axis else element.off_axis_vigilance


def element_sees_bearing(element: ThreatElement, x: float, y: float) -> bool:
    """Can ``element`` see that bearing at all? (weight strictly above zero)"""
    return element_bearing_weight(element, x, y) > 0.0


def bearing_only_sectors(sectors: Sequence[ThreatSector]) -> tuple[ThreatSector, ...]:
    """The sectors that name no observer — threat *directions*, not sensors."""
    return tuple(s for s in sectors if s.observer is None)


def _sector_azimuths(
    sectors: Sequence[ThreatSector], step_deg: float
) -> tuple[tuple[float, ...], float]:
    """Sampled azimuths across every sector, deduplicated, plus the step used."""
    sampled: list[float] = []
    seen: set[int] = set()
    for sector in sectors:
        span = arc_span_deg(sector.azimuth_start_deg, sector.azimuth_end_deg)
        steps = max(1, math.ceil(span / step_deg))
        for i in range(steps + 1):
            azimuth = (sector.azimuth_start_deg + span * i / steps) % 360.0
            # Overlapping sectors must not double-weight a bearing, so dedupe on
            # a 1e-3 deg grid rather than on float identity.
            key = round(azimuth * 1000.0) % 360_000
            if key not in seen:
                seen.add(key)
                sampled.append(azimuth)
    return tuple(sampled), step_deg


def _pixel_direction(grid: Grid, azimuth_deg: float) -> tuple[float, float]:
    """Unit (drow, dcol) step for a compass azimuth on this grid."""
    a, b, _c, d, e, _f = grid.transform
    det = a * e - b * d
    if det == 0.0:
        raise ValueError("degenerate grid transform (zero determinant)")
    dx = math.sin(math.radians(azimuth_deg))
    dy = math.cos(math.radians(azimuth_deg))
    dcol = (e * dx - b * dy) / det
    drow = (-d * dx + a * dy) / det
    norm = math.hypot(drow, dcol)
    if norm == 0.0:
        raise ValueError("degenerate grid transform (zero-length direction)")
    return drow / norm, dcol / norm


def _shifted_max(dsm: np.ndarray, drow: int, dcol: int, into: np.ndarray) -> None:
    """``into = max(into, dsm shifted by (drow, dcol))``; off-grid contributes nothing."""
    rows, cols = dsm.shape
    if abs(drow) >= rows or abs(dcol) >= cols:
        return
    src_r = slice(max(drow, 0), rows + min(drow, 0))
    src_c = slice(max(dcol, 0), cols + min(dcol, 0))
    dst_r = slice(max(-drow, 0), rows + min(-drow, 0))
    dst_c = slice(max(-dcol, 0), cols + min(-dcol, 0))
    np.maximum(into[dst_r, dst_c], dsm[src_r, src_c], out=into[dst_r, dst_c])


def openness_to_threat_sectors(
    dsm: np.ndarray,
    grid: Grid,
    sectors: Sequence[ThreatSector],
    *,
    target_height_m: float = 0.0,
    horizon_m: float = DEFAULT_HORIZON_M,
    azimuth_step_deg: float = DEFAULT_AZIMUTH_STEP_DEG,
) -> np.ndarray:
    """Fraction of the threat arc a cell is **not** terrain-masked from, in [0, 1].

    Software-Spec §5.1's ``openness_to_threat_sectors``, for sectors that name a
    bearing but no sensor. For each sampled azimuth the cell is masked when any
    surface within ``horizon_m`` along that bearing stands higher than the cell's
    own target elevation — i.e. a threat somewhere out there, at unknown range,
    is treated as being **at the horizon**. That is the reverse-slope/defilade
    question Preflight-Design §5.1 asks, and it is deliberately not a viewshed:
    there is no observer to run one from.

    Consequences worth knowing before trusting the number:

    * the test is a *sign* test on elevation, so it is scale-free in distance —
      what it is sensitive to is ``horizon_m``, because a ridge just outside the
      horizon stops counting as cover. There is no principled default;
    * the DSM carries the canopy, so a stand masks the ground behind it exactly
      as a landform does;
    * flat ground scores 1.0 (fully open) everywhere, which is correct and also
      means the field contributes nothing on a flat AO.

    With no sectors the field is all zeros — nothing to be open *to*.
    """
    rows, cols = dsm.shape
    if not sectors:
        return np.zeros((rows, cols), dtype=np.float32)

    azimuths, _ = _sector_azimuths(sectors, azimuth_step_deg)
    target = dsm.astype(np.float64) + target_height_m
    steps = max(1, round(horizon_m / grid.cell_size_m))
    open_count = np.zeros((rows, cols), dtype=np.float32)

    for azimuth in azimuths:
        drow, dcol = _pixel_direction(grid, azimuth)
        highest = np.full((rows, cols), -np.inf, dtype=np.float64)
        last: tuple[int, int] | None = None
        for k in range(1, steps + 1):
            offset = (round(k * drow), round(k * dcol))
            if offset == last:
                continue  # near-axis rays repeat a cell; sample it once
            last = offset
            _shifted_max(dsm, offset[0], offset[1], highest)
        open_count += (highest <= target).astype(np.float32)

    return (open_count / float(len(azimuths))).astype(np.float32)


def describe_threat_sectors(
    observers: Sequence[Coordinate], sectors: Sequence[ThreatSector]
) -> tuple[str, ...]:
    """One line per sector saying how it bound, for ``report.json``.

    A mission that supplies sectors and gets them ignored is the failure mode
    this exists to make impossible, so every sector appears here — including the
    ones that introduced an observer the mission did not list.
    """
    listed = {(o.x, o.y) for o in observers}
    lines: list[str] = []
    for i, sector in enumerate(sectors):
        label = sector.label or f"sector[{i}]"
        arc = (
            f"{sector.azimuth_start_deg:.0f}-{sector.azimuth_end_deg:.0f} deg "
            f"({arc_span_deg(sector.azimuth_start_deg, sector.azimuth_end_deg):.0f} deg wide)"
        )
        optic = (
            ""
            if sector.detection is None
            else f", {sector.detection.optic_magnification:.0f}x optic"
        )
        if sector.observer is None:
            lines.append(f"{label}: {arc} bearing-only -> cable openness field{optic}")
        elif (sector.observer.x, sector.observer.y) in listed:
            lines.append(
                f"{label}: {arc} masking the observer at "
                f"({sector.observer.x:.0f}, {sector.observer.y:.0f}){optic}"
            )
        else:
            lines.append(
                f"{label}: {arc} from an UNLISTED observer at "
                f"({sector.observer.x:.0f}, {sector.observer.y:.0f}) — "
                f"added as a threat observer{optic}"
            )
    return tuple(lines)
