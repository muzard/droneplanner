"""Terrain indices from the DTM (Software-Spec §5.1 ``terrain.py``).

Topographic Position Index (TPI), slope, and a drainage proxy — the covert-route
"terrain masking / hideability" primitives (Preflight-Design §3.1, §5.1).

Pure functions: bare-earth DTM array + cell size -> field array. TPI keeps its
sign (ridge > 0, valley < 0); ``slope`` and the ``*_penalty`` helpers return
normalized 0..1 cost layers.
"""

from __future__ import annotations

import numpy as np
from scipy.ndimage import uniform_filter

from droneplanner.fields.util import normalize01


def slope(dtm: np.ndarray, cell_size_m: float) -> np.ndarray:
    """Slope in degrees from the DTM gradient."""
    dz_dy, dz_dx = np.gradient(dtm.astype(np.float64), cell_size_m)
    return np.asarray(np.degrees(np.arctan(np.hypot(dz_dx, dz_dy))), dtype=np.float32)


def tpi(dtm: np.ndarray, radius_cells: int = 5) -> np.ndarray:
    """Topographic Position Index: cell elevation minus its neighborhood mean.

    Positive on ridges/crests (exposed), negative in valleys/draws (defiladed).
    Sign is preserved — callers decide how to penalize it.
    """
    window = 2 * radius_cells + 1
    local_mean = uniform_filter(dtm.astype(np.float64), size=window, mode="nearest")
    return np.asarray(dtm.astype(np.float64) - local_mean, dtype=np.float32)


def ridge_penalty(dtm: np.ndarray, radius_cells: int = 5) -> np.ndarray:
    """Normalized 0..1 penalty for positive TPI (ridgelines/crests)."""
    t = tpi(dtm, radius_cells)
    positive = np.clip(t, 0.0, None)
    return normalize01(positive)


def drainage_proxy(dtm: np.ndarray, radius_cells: int = 5) -> np.ndarray:
    """Normalized 0..1 "in a draw/valley" score from negative TPI.

    A lightweight stand-in for full hydrological drainage-line extraction
    (Preflight-Design §5.1): valleys and reverse slopes, which mask a cable from
    both observation and near-miss fragmentation, score high.
    """
    t = tpi(dtm, radius_cells)
    negative = np.clip(-t, 0.0, None)
    return normalize01(negative)


def slope_penalty(dtm: np.ndarray, cell_size_m: float) -> np.ndarray:
    """Normalized 0..1 slope-steepness penalty."""
    return normalize01(slope(dtm, cell_size_m))
