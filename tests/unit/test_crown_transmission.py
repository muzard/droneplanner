"""Sightlines scored by transmittance through the canopy
(CanopyOcclusion.crown_transmission, 2026-09-26).

The DSM march treats every crown BETWEEN observer and target as a solid wall, so a
drone at 5-7 m under a 20 m canopy, seen on a 26 degree look-down that clips the
neighbouring crown, scored exactly 0 — physically it crosses ~2 cells of crown, and
gap^2 gets through."""

from __future__ import annotations

import math

import numpy as np

from droneplanner.fields.viewshed import (
    CrownMedium,
    crown_transmittance,
    point_exposure,
    single_viewshed,
)
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import CanopyOcclusion

CELL = 10.0


def _stand(
    n: int = 40, canopy: float = 20.0, base: float = 10.0, gap: float = 0.1, stems_ha: float = 0.0
) -> tuple[np.ndarray, CrownMedium]:
    dtm = np.zeros((n, n))
    dsm = dtm + canopy
    medium = CrownMedium.build(
        dtm, dsm, crown_base_agl_m=np.full((n, n), base), gap=gap, stems_per_m2=stems_ha / 1e4
    )
    return dsm, medium


def _t(
    medium: CrownMedium,
    dsm: np.ndarray,
    obs: tuple[int, int],
    eye: float,
    tgt: tuple[int, int],
    z: float,
) -> float:
    return float(
        crown_transmittance(
            medium, dsm, obs, eye, np.array([tgt[0]]), np.array([tgt[1]]), np.array([z]), CELL
        )[0]
    )


def test_a_vertical_pass_through_the_whole_crown_transmits_the_gap_fraction() -> None:
    dsm, medium = _stand(gap=0.1)
    # Observer one cell away, 50 km up: the sightline is vertical to 2e-4 rad.
    t = _t(medium, dsm, (5, 5), 50_000.0, (5, 6), 5.0)
    assert math.isclose(t, 0.1, rel_tol=0.01)


def test_a_slant_path_is_attenuated_by_its_length_in_the_crown() -> None:
    dsm, medium = _stand(gap=0.1)
    # 45 degrees: the path crosses sqrt(2) crown depths -> gap^sqrt(2).
    t = _t(medium, dsm, (5, 5), 5.0 + 200.0, (5, 25), 5.0)
    assert math.isclose(t, 0.1 ** math.sqrt(2.0), rel_tol=0.05)


def test_the_binary_wall_scored_zero_where_the_medium_lets_light_through() -> None:
    """The artifact the model exists to fix: under a neighbour's crown on a 26 degree
    look-down the march scores exactly 0."""
    dsm, medium = _stand(n=60, gap=0.2)
    canopy = CanopyOcclusion(sub_canopy_gap_fraction=0.2)
    target = np.full(dsm.shape, 6.0)
    eye_h = 300.0
    obs = (30, 0)
    d = 60  # cells -> 600 m, atan(314 / 600) = 27.6 deg
    binary = single_viewshed(dsm, obs, CELL, eye_h, target_elev_m=target, canopy=canopy)
    medium_v = single_viewshed(
        dsm, obs, CELL, eye_h, target_elev_m=target, canopy=canopy, medium=medium
    )
    assert binary[30, d - 1] == 0.0
    # Crown path = 10 m / sin(27.6 deg) = 21.6 m -> 0.2^2.16 ~ 0.031
    assert 0.02 < medium_v[30, d - 1] < 0.045


def test_terrain_still_blocks_and_open_sky_is_clear() -> None:
    n = 30
    dtm = np.zeros((n, n))
    dtm[:, 10] = 100.0  # a ridge
    dsm = dtm.copy()
    medium = CrownMedium.build(
        dtm, dsm, crown_base_agl_m=np.zeros((n, n)), gap=0.1, stems_per_m2=0.0
    )
    assert _t(medium, dsm, (5, 0), 2.0, (5, 20), 5.0) == 0.0  # behind the ridge
    assert _t(medium, dsm, (5, 0), 2.0, (5, 8), 5.0) == 1.0  # in front, no canopy


def test_under_the_crowns_only_the_trunks_attenuate() -> None:
    n = 20
    dtm = np.zeros((1, n))
    dsm = dtm.copy()
    dsm[0, 3:] = 20.0  # open ground cols 0-2, then a stand with crowns from 10 m
    medium = CrownMedium.build(
        dtm,
        dsm,
        crown_base_agl_m=np.where(dsm > 0, 10.0, 0.0),
        gap=0.1,
        stems_per_m2=np.where(dsm > 0, 400.0 / 1e4, 0.0),
    )
    t = _t(medium, dsm, (0, 0), 2.0, (0, 12), 5.0)
    # ~95 m of trunk space (from the edge at 25 m to the target at 120 m) at
    # 0.04/m2 x 0.25 m -> exp(-0.95) ~ 0.39
    assert 0.3 < t < 0.5


def test_point_and_raster_agree() -> None:
    dsm, medium = _stand(n=30, gap=0.15, stems_ha=600.0)
    dsm = dsm.copy()
    dsm[:, :5] = 0.0  # open ground where the observer stands
    medium = CrownMedium.build(
        np.zeros_like(dsm),
        dsm,
        crown_base_agl_m=np.where(dsm > 0, 10.0, 0.0),
        gap=0.15,
        stems_per_m2=np.where(dsm > 0, 0.06, 0.0),
    )
    grid = Grid(transform=(CELL, 0.0, 0.0, 0.0, -CELL, 300.0), rows=30, cols=30, crs="EPSG:3067")
    obs_rc = (15, 1)
    observer = grid.pixel_to_world(*obs_rc)
    target = np.full(dsm.shape, 6.0)
    raster = single_viewshed(dsm, obs_rc, CELL, 150.0, target_elev_m=target, medium=medium)
    cells = [(15, 10), (3, 20), (27, 29), (15, 2)]
    pts = []
    for r, c in cells:
        p = grid.pixel_to_world(r, c)
        pts.append((p.x, p.y, 6.0))
    pe = point_exposure(
        dsm,
        grid,
        [Coordinate(x=observer.x, y=observer.y)],
        pts,
        observer_height_m=150.0,
        off_axis_vigilance=1.0,
        medium=medium,
    )
    for (r, c), v in zip(cells, pe, strict=True):
        assert math.isclose(float(raster[r, c]), float(v), rel_tol=1e-5, abs_tol=1e-6), (r, c)


def test_an_observer_inside_a_stand_stands_on_the_ground_and_sees_through_the_trunks() -> None:
    """SHORTFALLS 'the observer's eye sits on the DSM': a 2 m dismount in a 20 m stand
    was modelled at 22 m, in the treetops, where the binary wall blinds it to a drone
    in the same stand's trunk space. Under crown transmission it stands on the DTM."""
    n = 30
    dtm = np.zeros((1, n))
    dsm = dtm + 20.0
    medium = CrownMedium.build(
        dtm, dsm, crown_base_agl_m=np.full((1, n), 10.0), gap=0.1, stems_per_m2=600.0 / 1e4
    )
    target = np.full((1, n), 5.0)
    binary = single_viewshed(dsm, (0, 0), CELL, 2.0, target_elev_m=target)
    seen = single_viewshed(dsm, (0, 0), CELL, 2.0, target_elev_m=target, medium=medium)
    assert binary[0, 5] == 0.0  # eye on the crowns, the neighbours' crowns are a wall
    # 50 m of trunk space at 0.06 stems/m2 x 0.25 m -> exp(-0.75) ~ 0.47
    assert math.isclose(float(seen[0, 5]), math.exp(-0.75), rel_tol=0.05)
    assert seen[0, 20] < seen[0, 10] < seen[0, 5]  # the trunks add up with distance
    over = single_viewshed(
        dsm, (0, 0), CELL, 2.0, target_elev_m=np.full((1, n), 23.0), medium=medium
    )
    assert over[0, 5] < seen[0, 5]  # a canopy-top drone is seen up through the crowns
