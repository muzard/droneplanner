"""Shared helpers for field builders."""

from __future__ import annotations

import numpy as np


def normalize01(arr: np.ndarray) -> np.ndarray:
    """Scale an array to [0, 1]. A constant array maps to all zeros."""
    a = arr.astype(np.float32)
    lo = float(np.nanmin(a))
    hi = float(np.nanmax(a))
    if hi - lo <= 1e-12:
        return np.zeros_like(a)
    return ((a - lo) / (hi - lo)).astype(np.float32)
