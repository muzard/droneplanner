"""Cable survivability field (Software-Spec §5.1 ``survivability.py``).

Cable cost oriented to the *threat fire sectors* rather than to observers
(Preflight-Design §5.1):

    survivability = w1*openness_to_threat + w2*positive_TPI
                    + w3*chokepoint_exposure + w4*(1 - veg_cover)

Minimizing it routes the cable through defiladed, vegetated, non-chokepoint
ground. "Threat" is a passive placement cost only - never targeting logic
(Software-Spec §8). Returns a normalized 0..1 field.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from droneplanner.fields.sectors import (
    DEFAULT_HORIZON_M,
    bearing_only_sectors,
    openness_to_threat_sectors,
)
from droneplanner.fields.terrain import ridge_penalty, slope_penalty
from droneplanner.fields.util import normalize01
from droneplanner.fields.viewshed import cumulative_exposure
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import DEFAULT_OFF_AXIS_VIGILANCE, ThreatSector


@dataclass(frozen=True)
class SurvivabilityWeights:
    """Sub-weights of the survivability formula (Preflight-Design §5.1)."""

    openness: float = 1.0
    positive_tpi: float = 0.7
    chokepoint: float = 0.5
    exposure_no_cover: float = 0.4


def build_survivability(
    dtm: np.ndarray,
    dsm: np.ndarray,
    grid: Grid,
    threat_observers: list[Coordinate],
    veg_cover: np.ndarray | None = None,
    weights: SurvivabilityWeights | None = None,
    tpi_radius_cells: int = 5,
    openness: np.ndarray | None = None,
    sectors: Sequence[ThreatSector] = (),
    horizon_m: float = DEFAULT_HORIZON_M,
    off_axis_vigilance: float = DEFAULT_OFF_AXIS_VIGILANCE,
) -> np.ndarray:
    """Build the normalized 0..1 cable-survivability cost field.

    ``openness`` lets the caller pass a precomputed **ground-level** viewshed so
    the expensive field is not built twice. It must stay ground-level: unlike the
    drone (SHORTFALLS item 1, now scored at flight altitude) the cable really
    does lie on the surface, so target height 0 is the right question for it.

    ``sectors`` (roadmap C3) restores the ``w1*openness_to_threat_sectors`` term
    this formula has named since M0 and never had. Only the sectors that name **no
    observer** are used here — a bearing the threat is expected from, with no
    located sensor to run a viewshed from. Sectors that do name an observer are
    already accounted for in ``openness``, because they mask that observer's
    viewshed upstream.

    The two are combined by **maximum**, not by mean: they are two ways of being
    exposed to the same threat, and a cell an observer can see is exposed whether
    or not terrain masks the bearing. Averaging them would let one dilute the
    other, which on a concealment planner means promising cover that is not there.
    """
    w = weights or SurvivabilityWeights()

    # Openness to the threat: what the threat observers can see of the ground.
    if openness is None:
        openness = cumulative_exposure(
            dsm, grid, threat_observers, sectors=sectors, off_axis_vigilance=off_axis_vigilance
        )
    bearings = bearing_only_sectors(sectors)
    if bearings:
        openness = np.maximum(
            openness,
            openness_to_threat_sectors(dsm, grid, bearings, horizon_m=horizon_m),
        )

    # Ridgelines/crests are both exposed and un-maskable for a cable.
    pos_tpi = ridge_penalty(dtm, tpi_radius_cells)

    # v0 chokepoint proxy: steep, pinched terrain (Preflight-Design §5.1 flags a
    # fuller chokepoint model as future work).
    chokepoint = slope_penalty(dtm, grid.cell_size_m)

    # Bare (uncovered) ground; 0 everywhere when no land cover is supplied.
    if veg_cover is not None:
        no_cover = normalize01(1.0 - normalize01(veg_cover))
    else:
        no_cover = np.zeros_like(pos_tpi)

    raw = (
        w.openness * openness
        + w.positive_tpi * pos_tpi
        + w.chokepoint * chokepoint
        + w.exposure_no_cover * no_cover
    )
    return normalize01(raw)
