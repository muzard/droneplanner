"""Sightlines under the crowns (CanopyOcclusion.trunk_space_sightlines, 2026-09-25).

The DSM march treats the canopy as a wall; an observer on open ground at a forest
edge can nonetheless see a drone flying through the trunk space inside it."""

from __future__ import annotations

import math

import numpy as np

from droneplanner.fields.viewshed import TrunkSpace, _point_weight, single_viewshed

CELL = 10.0


def _field(stems_ha: float = 400.0) -> tuple[np.ndarray, TrunkSpace]:
    # One row: open ground (cols 0-2), then a 20 m stand with its crown base at 10 m.
    n = 12
    dtm = np.zeros((1, n), dtype=np.float32)
    dsm = dtm.copy()
    dsm[0, 3:] = 20.0
    ts = TrunkSpace(
        dtm=dtm,
        crown_base_agl_m=np.where(dsm > 0, 10.0, 0.0).astype(np.float32),
        stems_per_m2=np.where(dsm > 0, stems_ha / 1e4, 0.0).astype(np.float32),
        trunk_diameter_m=0.25,
    )
    return dsm, ts


def test_a_drone_in_trunk_space_is_seen_from_the_edge_attenuated_by_the_trunks() -> None:
    dsm, ts = _field()
    eye = 2.0  # standing at col 0, open ground
    # Drone at col 7 (70 m out, 40 m into the stand) flying at 6 m AGL.
    without = _point_weight(dsm, (0, 0), eye, (0, 7), 6.0, CELL)
    with_ts = _point_weight(dsm, (0, 0), eye, (0, 7), 6.0, CELL, trunk_space=ts)
    assert without == 0.0  # the canopy is a wall to the march
    # ~45 m of trunk space at 0.04 stems/m2 x 0.25 m: exp(-0.01 x 45) ~ 0.64
    assert 0.5 < with_ts < 0.75
    denser = _point_weight(dsm, (0, 0), eye, (0, 7), 6.0, CELL, trunk_space=_field(1200.0)[1])
    assert denser < with_ts  # more stems, less seen
    assert math.isclose(
        _point_weight(dsm, (0, 0), eye, (0, 7), 12.0, CELL, trunk_space=ts), 0.0
    )  # at 12 m the line runs through the crown layer


def test_an_observer_inside_the_stand_is_not_covered_and_the_raster_agrees() -> None:
    dsm, ts = _field()
    assert _point_weight(dsm, (0, 5), 22.0, (0, 9), 6.0, CELL, trunk_space=ts) == 0.0
    target = np.full(dsm.shape, 6.0, dtype=np.float32)
    raster = single_viewshed(dsm, (0, 0), CELL, 2.0, target_elev_m=target, trunk_space=ts)
    for c in range(1, dsm.shape[1]):
        point = _point_weight(dsm, (0, 0), 2.0, (0, c), 6.0, CELL, trunk_space=ts)
        assert math.isclose(float(raster[0, c]), point, rel_tol=1e-5, abs_tol=1e-6), c


def test_a_measured_gap_map_replaces_the_scalar_gap_fraction_per_cell() -> None:
    """layers.canopy_gap_fraction (2026-09-26): the look-down gap probability is the
    measured value of the cell the target is under, not the literature 0.10."""
    from droneplanner.model.mission import CanopyOcclusion

    n = 6
    dsm = np.full((1, n), 20.0, dtype=np.float32)
    canopy = CanopyOcclusion(sub_canopy_gap_fraction=0.10)
    gaps = np.full((1, n), 0.10, dtype=np.float32)
    gaps[0, 5] = 0.40
    # A 300 m sensor looking steeply down at a drone 6 m up under the crown at col 5.
    scalar = _point_weight(dsm, (0, 0), 320.0, (0, 5), 6.0, CELL, canopy=canopy)
    measured = _point_weight(dsm, (0, 0), 320.0, (0, 5), 6.0, CELL, canopy=canopy, gap_map=gaps)
    assert math.isclose(scalar, 0.10, rel_tol=1e-6)
    assert math.isclose(measured, 0.40, rel_tol=1e-6)
