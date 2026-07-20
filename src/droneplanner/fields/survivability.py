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

from dataclasses import dataclass

import numpy as np

from droneplanner.fields.terrain import ridge_penalty, slope_penalty
from droneplanner.fields.util import normalize01
from droneplanner.fields.viewshed import cumulative_exposure
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid


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
) -> np.ndarray:
    """Build the normalized 0..1 cable-survivability cost field."""
    w = weights or SurvivabilityWeights()

    # Openness to the threat: what the threat observers can see of the ground.
    openness = cumulative_exposure(dsm, grid, threat_observers)

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
