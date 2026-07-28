"""Unit tests for the Gazebo forest generator (TEST-PLAN B2)."""

from __future__ import annotations

import numpy as np

from droneplanner.io.forest_models import (
    generate_trunks,
    nearest_trunk_spacing_m,
    trunk_manifest_csv,
    trunks_to_sdf,
)
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid

CELL_M = 5.0
SIZE = 40


def _grid() -> Grid:
    return Grid(
        transform=(CELL_M, 0.0, 0.0, 0.0, -CELL_M, SIZE * CELL_M),
        rows=SIZE,
        cols=SIZE,
        crs="EPSG:32633",
    )


def _scene(canopy_cols: slice = slice(10, 30)) -> tuple[np.ndarray, np.ndarray, list[Coordinate]]:
    chm = np.zeros((SIZE, SIZE), dtype=np.float32)
    chm[:, canopy_cols] = 18.0
    dtm = np.zeros((SIZE, SIZE), dtype=np.float32)
    grid = _grid()
    route = [grid.pixel_to_world(r, 20) for r in range(5, 35)]
    return chm, dtm, route


def _generate(**kwargs: object):  # type: ignore[no-untyped-def]
    chm, dtm, route = _scene()
    grid = _grid()
    defaults = dict(
        home=Coordinate(x=0.0, y=0.0),
        home_elev_m=0.0,
        band_m=20.0,
        stems_per_ha=400.0,
        max_stems=10_000,
        seed=7,
    )
    defaults.update(kwargs)
    return generate_trunks(chm, dtm, grid, route, **defaults)  # type: ignore[arg-type]


def test_trunks_only_stand_where_there_is_canopy() -> None:
    """A trunk in a clearing would be scenery the planner never believed in."""
    trunks, _ = _generate()
    grid = _grid()
    chm, _dtm, _route = _scene()
    assert trunks
    for t in trunks:
        r, c = grid.world_to_pixel(Coordinate(x=t.east_m, y=t.north_m))
        assert chm[r, c] > 0.0


def test_density_is_reported_and_honoured() -> None:
    trunks, stats = _generate()
    assert stats.band_area_ha > 0
    assert abs(stats.effective_stems_per_ha - 400.0) < 40.0
    assert not stats.subsampled
    assert len(trunks) == stats.trunks


def test_the_cap_lowers_density_and_says_so_rather_than_narrowing_the_band() -> None:
    """A hole in the forest would let the aircraft escape it — that must not happen."""
    wide, wide_stats = _generate(max_stems=10_000)
    capped, capped_stats = _generate(max_stems=200)
    assert capped_stats.subsampled
    assert len(capped) == 200
    assert capped_stats.effective_stems_per_ha < wide_stats.effective_stems_per_ha
    # Same footprint: the band is unchanged, only the density within it.
    assert capped_stats.band_area_ha == wide_stats.band_area_ha
    assert capped_stats.max_route_distance_m <= wide_stats.max_route_distance_m + CELL_M


def test_generation_is_deterministic() -> None:
    a, _ = _generate()
    b, _ = _generate()
    assert a == b
    c, _ = _generate(seed=8)
    assert c != a


def test_trunks_sit_on_the_terrain_not_at_the_world_origin() -> None:
    """Base z is the terrain surface relative to home, or trunks float or sink."""
    chm, _dtm, route = _scene()
    dtm = np.full((SIZE, SIZE), 130.0, dtype=np.float32)  # 30 m above a home at 100 m
    trunks, _ = generate_trunks(
        chm, dtm, _grid(), route, home=Coordinate(x=0.0, y=0.0), home_elev_m=100.0, seed=1
    )
    assert trunks
    assert all(abs(t.base_z_m - 30.0) < 1e-6 for t in trunks)


def test_manifest_and_sdf_describe_the_same_trees() -> None:
    """The whole point: perception must sense what the physics engine will hit."""
    trunks, _ = _generate(max_stems=25)
    csv = trunk_manifest_csv(trunks).strip().splitlines()
    sdf = trunks_to_sdf(trunks)
    assert len(csv) == len(trunks) + 1  # header
    # ONE model, N collisions — not N models. See trunks_to_sdf: 4000 separate
    # models made Gazebo hang before it ever published the world.
    assert sdf.count("<model name=") == 1
    assert sdf.count("<collision name=") == len(trunks)
    first = trunks[0]
    assert f"{first.east_m:.3f},{first.north_m:.3f}" in csv[1]
    # A collision's pose lifts the cylinder centre to base + height/2.
    centre_z = first.base_z_m + first.height_m / 2
    assert f"{first.east_m:.3f} {first.north_m:.3f} {centre_z:.3f}" in sdf


def test_visuals_are_off_by_default_because_sitl_runs_headless() -> None:
    trunks, _ = _generate(max_stems=10)
    assert "<visual" not in trunks_to_sdf(trunks)
    assert trunks_to_sdf(trunks, visuals=True).count("<visual name=") == len(trunks)


def test_no_canopy_means_no_forest() -> None:
    chm = np.zeros((SIZE, SIZE), dtype=np.float32)
    dtm = np.zeros((SIZE, SIZE), dtype=np.float32)
    grid = _grid()
    route = [grid.pixel_to_world(r, 20) for r in range(5, 35)]
    trunks, stats = generate_trunks(
        chm, dtm, grid, route, home=Coordinate(x=0.0, y=0.0), home_elev_m=0.0
    )
    assert trunks == []
    assert stats.trunks == 0


def test_spacing_reports_the_gap_the_airframe_has_to_fly_through() -> None:
    """Density is the headline; spacing is what avoidance actually meets."""
    dense, _ = _generate(stems_per_ha=1000.0)
    sparse, _ = _generate(stems_per_ha=100.0)
    assert nearest_trunk_spacing_m(dense) < nearest_trunk_spacing_m(sparse)
    assert nearest_trunk_spacing_m([]) == float("inf")
