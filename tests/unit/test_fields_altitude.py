"""Unit tests for the flight-altitude policy (SHORTFALLS item 1c).

The defect these guard against: the planner flew a fixed 30 m above *bare
terrain*, which in a 22 m stand is 8 m over the crowns. The cost surface scored
that as concealed forest flight. It was not — the drone was above the canopy the
whole way.
"""

from __future__ import annotations

import numpy as np

from droneplanner.fields.altitude import (
    altitude_band_at,
    band_raster,
    canopy_height,
    planned_flight_agl,
)
from droneplanner.forest.navigability import FlyabilityConfig, build_flyability
from droneplanner.forest.voxelize import VoxelGrid
from droneplanner.model.grid import Grid
from droneplanner.model.mission import AltitudePolicy
from droneplanner.model.plan import FlightBand

POLICY = AltitudePolicy()  # floor 5 m, canopy clearance 3 m, min canopy 8 m
TARGET = 10.0


def _band(chm_m: float) -> tuple[float, float, float, FlightBand]:
    chm = np.full((1, 1), chm_m, dtype=np.float32)
    return altitude_band_at((0, 0), chm=chm, target_agl_m=TARGET, policy=POLICY)


def test_canopy_height_is_dsm_minus_dtm_clamped() -> None:
    dtm = np.array([[10.0, 20.0]], dtype=np.float32)
    dsm = np.array([[32.0, 19.0]], dtype=np.float32)  # second cell: DSM below DTM
    assert np.array_equal(canopy_height(dtm, dsm), np.array([[22.0, 0.0]], dtype=np.float32))


def test_mature_stand_is_flown_UNDER_the_canopy() -> None:
    """The whole point: 22 m of trees means fly at 10 m, not 30 m."""
    pref, floor, ceil, band = _band(22.0)
    assert band is FlightBand.BELOW_CANOPY
    assert pref == TARGET  # the target fits under the crowns
    assert ceil == 22.0 - POLICY.canopy_clearance_m  # 3 m of headroom kept
    assert floor == POLICY.min_agl_m


def test_a_stand_too_tight_to_enter_is_skimmed_not_entered() -> None:
    """9 m canopy: only 6 m of usable space, under a 5 m floor + 3 m clearance."""
    pref, _floor, _ceil, band = _band(9.0)
    assert band is FlightBand.CANOPY_TOP
    assert pref == 9.0 + POLICY.canopy_clearance_m  # over the top, not into it


def test_open_ground_holds_the_target_and_is_not_called_cover() -> None:
    pref, _floor, _ceil, band = _band(2.0)  # scrub, below min_canopy
    assert band is FlightBand.OPEN
    assert pref == TARGET


def test_the_floor_wins_over_a_lower_target() -> None:
    chm = np.full((1, 1), 30.0, dtype=np.float32)
    pref, floor, _ceil, _band = altitude_band_at((0, 0), chm=chm, target_agl_m=1.0, policy=POLICY)
    assert pref == floor == POLICY.min_agl_m


def test_tall_canopy_lets_the_drone_go_as_low_as_the_floor() -> None:
    """'As low as possible' is bounded by the floor, not by the canopy."""
    pref, _floor, _ceil, band = altitude_band_at(
        (0, 0),
        chm=np.full((1, 1), 40.0, dtype=np.float32),
        target_agl_m=POLICY.min_agl_m,
        policy=POLICY,
    )
    assert band is FlightBand.BELOW_CANOPY
    assert pref == POLICY.min_agl_m


def _mixed_chm() -> np.ndarray:
    """A canopy raster spanning every case: open, scrub, tight, mature, towering."""
    return np.array(
        [[0.0, 2.0, 7.9, 8.0], [9.0, 13.0, 22.0, 40.0], [8.1, 11.0, 30.0, 5.0]], dtype=np.float32
    )


def test_raster_matches_the_per_cell_band_everywhere() -> None:
    """The raster and the per-vertex band must agree cell for cell.

    The cost surface evaluates the viewshed at ``planned_flight_agl`` while the
    corridor planner flies ``altitude_band_at``. If they drift apart, concealment
    is scored at an altitude the drone never occupies — the exact class of bug
    (an offline model disagreeing with what is actually flown) this project keeps
    paying for. So they are one definition, pinned here.
    """
    chm = _mixed_chm()
    raster = planned_flight_agl(chm, TARGET, POLICY)
    for r in range(chm.shape[0]):
        for c in range(chm.shape[1]):
            pref, _floor, _ceil, _band = altitude_band_at(
                (r, c), chm=chm, target_agl_m=TARGET, policy=POLICY
            )
            assert raster[r, c] == np.float32(pref), f"cell {(r, c)} disagrees"


def _flyability(size: int = 4):  # type: ignore[no-untyped-def]
    """A lidar-derived field with a real free window (ground + canopy at 10-12 m)."""
    nz = 26
    occ = np.zeros((nz, size, size), dtype=bool)
    occ[0] = True  # ground
    occ[20:] = True  # canopy
    grid = Grid(transform=(1.0, 0.0, 0.0, 0.0, -1.0, size), rows=size, cols=size, crs="EPSG:32633")
    voxels = VoxelGrid(occupancy=occ, grid=grid, z0=0.0, voxel_size_z=0.5)
    return build_flyability(voxels, FlyabilityConfig(r_drone=0.45))


def test_a_lidar_voxel_window_overrides_the_canopy_top_guess() -> None:
    """Measured free space beats a ceiling inferred from the canopy top."""
    field = _flyability()
    chm = np.full((4, 4), 12.0, dtype=np.float32)
    pref, _floor, _ceil, band = altitude_band_at(
        (1, 1), chm=chm, target_agl_m=TARGET, policy=POLICY, flyability=field
    )
    assert band is FlightBand.UPPER_TRUNK
    assert pref != TARGET  # it came from the voxel window, not the policy
    raster = planned_flight_agl(chm, TARGET, POLICY, field)
    assert raster[1, 1] == np.float32(pref)


def test_band_raster_codes_each_case() -> None:
    codes = band_raster(_mixed_chm(), TARGET, POLICY)
    assert codes[0, 0] == 0  # bare ground -> open
    assert codes[0, 1] == 0  # 2 m scrub -> still open, nothing to hide under
    assert codes[1, 0] == 1  # 9 m canopy -> too tight to enter, skim the top
    assert codes[1, 2] == 2  # 22 m canopy -> below canopy
    # A lidar window promotes every covered cell to the measured trunk band.
    lidar = band_raster(np.full((4, 4), 12.0, dtype=np.float32), TARGET, POLICY, _flyability())
    assert np.all(lidar == 3)


class TestConcealmentZone:
    """Mission.covered_radius_m — cover only for the first part (user, 2026-09-14)."""

    def test_outside_the_zone_a_tall_stand_is_flown_over(self) -> None:
        from droneplanner.fields.altitude import _band_from_canopy

        inside = _band_from_canopy(20.0, 10.0, POLICY, covered=True)
        outside = _band_from_canopy(20.0, 10.0, POLICY, covered=False)
        assert inside[3] == FlightBand.BELOW_CANOPY
        assert outside[3] == FlightBand.CANOPY_TOP
        assert outside[0] >= 20.0 + POLICY.canopy_clearance_m

    def test_mask_is_radial_about_the_start(self) -> None:
        from droneplanner.fields.altitude import concealment_mask
        from droneplanner.model.geometry import Coordinate

        grid = Grid(
            transform=(10.0, 0.0, 0.0, 0.0, -10.0, 100.0), rows=10, cols=10, crs="EPSG:32633"
        )
        mask = concealment_mask(grid, Coordinate(x=5.0, y=95.0), 30.0)
        assert mask is not None
        assert mask[0, 0] and mask[0, 3] and not mask[0, 4] and not mask[9, 9]
        assert concealment_mask(grid, Coordinate(x=5.0, y=95.0), None) is None

    def test_raster_and_point_agree_under_a_zone(self) -> None:
        chm = np.full((1, 2), 20.0, dtype=np.float32)
        covered = np.array([[True, False]])
        agl = planned_flight_agl(chm, 10.0, POLICY, covered=covered)
        codes = band_raster(chm, 10.0, POLICY, covered=covered)
        assert codes[0, 0] == 2 and codes[0, 1] == 1
        assert agl[0, 0] < 20.0 < agl[0, 1]


CROWNED = AltitudePolicy(crown_ratio=0.5, crown_clearance_m=1.0)


def test_crown_ratio_holds_the_ceiling_under_the_crown_base() -> None:
    """🔴 Askola, 2026-09-25: a 15 m, spruce-dominated stand flown at 10.5-12 m —
    inside the live crowns. With a crown ratio the ceiling is the crown base less
    its clearance, not the canopy top less 3 m."""
    chm = np.full((1, 1), 22.0, dtype=np.float32)
    pref, _floor, ceil, band = altitude_band_at(
        (0, 0), chm=chm, target_agl_m=TARGET, policy=CROWNED
    )
    assert band is FlightBand.BELOW_CANOPY
    assert ceil == 22.0 * 0.5 - 1.0  # crown base 11 m, not canopy top 22 - 3 = 19 m
    assert pref == TARGET  # 10 m still fits under 10.0 -- the target equals the ceiling
    pref, _floor, ceil, band = altitude_band_at(
        (0, 0), chm=np.full((1, 1), 15.0, dtype=np.float32), target_agl_m=12.0, policy=CROWNED
    )
    assert band is FlightBand.BELOW_CANOPY
    assert pref == ceil == 6.5  # pushed down out of the crowns, still over the 5 m floor


def test_a_stand_whose_crowns_reach_the_floor_is_not_entered() -> None:
    """Crown base 0.5 x 11 m = 5.5 m, less 1 m: no trunk space above a 5 m floor."""
    chm = np.full((1, 1), 11.0, dtype=np.float32)
    _pref, _floor, _ceil, band = altitude_band_at(
        (0, 0),
        chm=chm,
        target_agl_m=TARGET,
        policy=AltitudePolicy(min_canopy_m=8.0, crown_ratio=0.5),
    )
    assert band is FlightBand.CANOPY_TOP


def test_crowned_raster_matches_the_per_cell_band_everywhere() -> None:
    chm = _mixed_chm()
    raster = planned_flight_agl(chm, TARGET, CROWNED)
    codes = band_raster(chm, TARGET, CROWNED)
    for r in range(chm.shape[0]):
        for c in range(chm.shape[1]):
            pref, _floor, _ceil, band = altitude_band_at(
                (r, c), chm=chm, target_agl_m=TARGET, policy=CROWNED
            )
            assert raster[r, c] == np.float32(pref), f"cell {(r, c)} disagrees"
            assert (codes[r, c] == 2) == (band is FlightBand.BELOW_CANOPY)


def test_over_canopy_skims_the_tallest_canopy_ahead_with_headroom_above() -> None:
    """Real canopy is jagged cell to cell; over it, clear the tallest canopy within the
    lookahead, and leave headroom so a short clearing is not a dive (2026-09-25)."""
    from droneplanner.fields.altitude import OVER_HEADROOM_M, skim_canopy

    chm = np.array([[10.0, 25.0, 10.0, 10.0, 10.0, 10.0]], dtype=np.float32)
    policy = AltitudePolicy(skim_lookahead_m=20.0)  # 2 cells at 10 m
    skim = skim_canopy(chm, policy, cell_size_m=10.0)
    assert list(skim[0]) == [25.0, 25.0, 25.0, 25.0, 10.0, 10.0]
    pref, floor, ceil, band = altitude_band_at(
        (0, 3),
        chm=chm,
        target_agl_m=TARGET,
        policy=policy,
        covered=np.zeros_like(chm, bool),
        skim=skim,
    )
    assert band is FlightBand.CANOPY_TOP
    assert floor == pref == 25.0 + policy.canopy_clearance_m
    assert ceil == pref + OVER_HEADROOM_M
    raster = planned_flight_agl(chm, TARGET, policy, covered=np.zeros_like(chm, bool), skim=skim)
    assert raster[0, 3] == np.float32(pref)
