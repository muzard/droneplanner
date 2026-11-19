"""Fine-voxel flyability aggregated to the corridor grid (SHORTFALLS item 2)."""

from __future__ import annotations

import numpy as np
import pytest

from droneplanner.fields.stems import StemMap
from droneplanner.forest.fine import build_flyability_fine
from droneplanner.forest.navigability import (
    FLYABLE,
    NON_NAVIGABLE,
    FlyabilityConfig,
    build_flyability,
)
from droneplanner.forest.synthetic_scan import simulate_scan
from droneplanner.forest.voxelize import voxelize
from droneplanner.model.grid import Grid

SIDE = 100.0
CFG = FlyabilityConfig(r_drone=0.45)


def _grid(cell: float) -> Grid:
    k = int(SIDE / cell)
    return Grid(transform=(cell, 0.0, 0.0, 0.0, -cell, SIDE), rows=k, cols=k, crs="EPSG:3067")


def _stand(stems_per_ha: float, seed: int = 0) -> np.ndarray:
    """Trunks to 8 m, porous crowns 8-18 m, ground returns."""
    rng = np.random.default_rng(seed)
    n = round(stems_per_ha * SIDE * SIDE / 10_000.0)
    xy = rng.uniform(2.0, SIDE - 2.0, (n, 2))
    stems = StemMap(x=xy[:, 0], y=xy[:, 1], radius_m=rng.uniform(0.1, 0.2, n))
    trunks = simulate_scan(stems, returns_per_stem_m2=50.0, seed=seed + 1)
    trunks = trunks[trunks[:, 2] < 8.0]
    m = 200_000
    crowns = np.c_[rng.uniform(0, SIDE, m), rng.uniform(0, SIDE, m), rng.uniform(8, 18, m)]
    ground = np.c_[rng.uniform(0, SIDE, 50_000), rng.uniform(0, SIDE, 50_000), np.zeros(50_000)]
    return np.vstack([trunks, crowns, ground])


def test_coarse_columns_turn_a_flyable_stand_into_an_obstacle() -> None:
    """The defect: one trunk fills a 25 m column at every trunk-space level."""
    pts = _stand(200)
    g = _grid(25.0)
    vox = voxelize(pts, g, z_min=0.0, z_max=19.0)
    coarse = build_flyability(vox, CFG, ground_level=np.zeros((g.rows, g.cols), dtype=int))
    assert np.all(coarse.navigable_class == NON_NAVIGABLE)

    fine = build_flyability_fine(pts, g, np.zeros((g.rows, g.cols)), CFG)
    assert np.mean(fine.navigable_class == FLYABLE) >= 0.75
    flyable = fine.navigable_class == FLYABLE
    # The band sits in the trunk space: above the understory, under the crowns.
    assert np.all(fine.nav_floor_m[flyable] >= CFG.understory_top_m)
    # Crowns in the fixture are porous (~0.25 returns per voxel), so a window can
    # reach a metre or so into them; the band still tops out near the crown base.
    assert np.all(fine.nav_ceil_m[flyable] <= 10.0)
    assert np.all(fine.best_clearance_m[flyable] >= CFG.flyable_clearance_m)


def test_a_solid_block_stays_non_navigable() -> None:
    xs = np.arange(0.05, 25.0, 0.1)
    gx, gy = np.meshgrid(xs, xs)
    zs = np.arange(0.0, 20.0, 0.25)
    wall = np.array([(x, y, z) for z in zs for x, y in zip(gx.ravel()[::7], gy.ravel()[::7])])
    g = _grid(25.0)
    pts = wall + np.array([25.0, 25.0, 0.0])  # fill corridor cell (row 2, col 1)
    fine = build_flyability_fine(pts, g, np.zeros((g.rows, g.cols)), CFG)
    r, c = 2, 1
    assert fine.navigable_class[r, c] == NON_NAVIGABLE
    assert fine.non_navigable[r, c]


def test_cells_without_returns_match_the_coarse_field() -> None:
    pts = _stand(200)
    pts = pts[(pts[:, 0] < 50.0) & (pts[:, 1] > 50.0)]  # data only in the NW quarter
    g = _grid(25.0)
    dtm = np.zeros((g.rows, g.cols))
    vox = voxelize(pts, g, z_min=float(pts[:, 2].min()), z_max=float(pts[:, 2].max()) + 0.5)
    coarse = build_flyability(vox, CFG, ground_level=vox.ground_level(dtm))
    fine = build_flyability_fine(pts, g, dtm, CFG)
    empty = ~np.zeros((g.rows, g.cols), dtype=bool)
    empty[:2, :2] = False
    assert np.array_equal(fine.navigable_class[empty], coarse.navigable_class[empty])
    assert np.array_equal(fine.nav_ceil_m[empty], coarse.nav_ceil_m[empty])


def test_empty_cloud_and_bad_grids() -> None:
    g = _grid(25.0)
    dtm = np.zeros((g.rows, g.cols))
    out = build_flyability_fine(np.empty((0, 3)), g, dtm, CFG)
    assert out.navigable_class.shape == (g.rows, g.cols)
    with pytest.raises(ValueError, match="north-up"):
        build_flyability_fine(
            np.empty((0, 3)),
            Grid(transform=(25.0, 0.0, 0.0, 0.0, 25.0, 0.0), rows=4, cols=4, crs="EPSG:3067"),
            dtm,
            CFG,
        )
    with pytest.raises(ValueError, match="fine_m"):
        build_flyability_fine(np.empty((0, 3)), g, dtm, CFG, fine_m=0.0)
