"""Fiber snag-risk field (Software-Spec §5.1 ``snag.py``).

v0 proxy: obstacle density at the predicted settle height derived from land
cover (Preflight-Design §3.4). The full model uses forest voxels (M2); until
then, a land-cover class -> snag-risk lookup, or an all-zero field when no land
cover is supplied. Returns a normalized 0..1 field.
"""

from __future__ import annotations

import numpy as np

from droneplanner.fields.util import normalize01


def build_snag(
    shape: tuple[int, int],
    land_cover: np.ndarray | None = None,
    snag_lookup: dict[int, float] | None = None,
) -> np.ndarray:
    """Build the normalized 0..1 snag-risk field.

    ``land_cover`` is a categorical class raster; ``snag_lookup`` maps each class
    to a snag-risk weight. Unmapped classes -> 0. No land cover -> all zeros.
    """
    if land_cover is None or snag_lookup is None:
        return np.zeros(shape, dtype=np.float32)

    risk = np.zeros(shape, dtype=np.float32)
    for cls, weight in snag_lookup.items():
        risk[land_cover == cls] = weight
    return normalize01(risk)
