"""Unit tests for terrain indices (Software-Spec §5.1, §6.2).

Analytic terrain with hand-derivable TPI signs and slope.
"""

from __future__ import annotations

import numpy as np

from droneplanner.fields.terrain import (
    drainage_proxy,
    ridge_penalty,
    slope,
    slope_penalty,
    tpi,
)
from tests.fixtures.synthetic import make_dem


def test_tpi_positive_on_ridge_crest() -> None:
    dem = make_dem("ridge", size=41)
    t = tpi(dem.data, radius_cells=4)
    center_col = 41 // 2
    # The crest (ridge center) sits above its neighborhood -> positive TPI.
    assert t[20, center_col] > 0.0
    # A cell out on the flank is below the local mean -> negative.
    assert t[20, center_col + 8] < 0.0


def test_tpi_negative_in_valley_floor() -> None:
    dem = make_dem("valley", size=41)
    t = tpi(dem.data, radius_cells=4)
    assert t[20, 20] < 0.0


def test_ridge_penalty_normalized() -> None:
    dem = make_dem("ridge", size=32)
    r = ridge_penalty(dem.data)
    assert r.min() >= 0.0
    assert r.max() <= 1.0
    assert r.dtype == np.float32


def test_drainage_proxy_high_in_valley() -> None:
    dem = make_dem("valley", size=41)
    d = drainage_proxy(dem.data, radius_cells=4)
    assert d.min() >= 0.0 and d.max() <= 1.0
    # Valley floor should score high on the "in a draw" proxy.
    assert d[20, 20] > 0.5


def test_slope_flat_is_zero() -> None:
    dem = make_dem("flat", size=16)
    s = slope(dem.data, cell_size_m=10.0)
    assert np.allclose(s, 0.0)


def test_slope_penalty_ranges() -> None:
    dem = make_dem("gaussian_hill", size=32)
    sp = slope_penalty(dem.data, cell_size_m=10.0)
    assert sp.min() >= 0.0 and sp.max() <= 1.0
