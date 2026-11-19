"""Unit tests for survivability, snag, and the composite cost surface (§5.1, §5.3)."""

from __future__ import annotations

import numpy as np

from droneplanner.fields.composite import COST_FLOOR, build_cost_surface
from droneplanner.fields.snag import build_snag
from droneplanner.fields.survivability import build_survivability
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from tests.fixtures.synthetic import make_dem, make_mission


def _grid_for(size: int) -> Grid:
    transform = (10.0, 0.0, 500000.0, 0.0, -10.0, 5200000.0 + size * 10.0)
    return Grid(transform=transform, rows=size, cols=size, crs="EPSG:32633")


def test_snag_zero_without_landcover() -> None:
    s = build_snag((10, 10))
    assert np.all(s == 0.0)


def test_snag_from_landcover_lookup() -> None:
    lc = np.zeros((8, 8), dtype=np.int32)
    lc[0:4, :] = 5  # a "dense" class
    s = build_snag((8, 8), land_cover=lc, snag_lookup={5: 1.0})
    assert s[0, 0] == 1.0
    assert s[7, 7] == 0.0


def test_survivability_normalized() -> None:
    dem = make_dem("gaussian_hill", size=24)
    grid = _grid_for(24)
    obs = grid.pixel_to_world(4, 4)
    field = build_survivability(dem.data, dem.data, grid, [Coordinate(x=obs.x, y=obs.y)])
    assert field.min() >= 0.0 and field.max() <= 1.0
    assert field.shape == (24, 24)


def test_cost_surface_positive_and_has_components() -> None:
    dem = make_dem("ridge", size=24)
    grid = _grid_for(24)
    mission = make_mission(crs="EPSG:32633")
    surface = build_cost_surface(dem.data, dem.data, grid, mission)
    assert surface.cost.min() >= COST_FLOOR
    for key in (
        "exposure",
        "ground_los",
        "flight_agl",
        "survivability",
        "snag",
        "ridge",
        "drainage",
        "cost_normalized",
    ):
        assert key in surface.components
    assert surface.shape == (24, 24)


def _mission_with_observer(grid: Grid, **overrides: object) -> object:
    obs = grid.pixel_to_world(2, 2)
    return make_mission(
        crs="EPSG:32633",
        threat={"observers": [{"x": obs.x, "y": obs.y}]},
        **overrides,
    )


def test_exposure_is_scored_at_flight_altitude_not_on_the_ground() -> None:
    """SHORTFALLS item 1: the drone and the ground under it are different objects.

    Over a bare DSM, lifting the target to cruise altitude can only reveal more,
    and here it must actually reveal more — otherwise the field is still
    measuring the ground.
    """
    dem = make_dem("ridge", size=24)
    grid = _grid_for(24)
    mission = _mission_with_observer(grid)
    surface = build_cost_surface(dem.data, dem.data, grid, mission)  # type: ignore[arg-type]

    drone = surface.components["exposure"]
    ground = surface.components["ground_los"]
    assert np.all(drone >= ground)  # nothing hides by climbing over bare terrain
    assert drone.mean() > ground.mean()  # and the ridge stops hiding the aircraft
    # DSM == DTM here: no canopy to get under, so the target altitude stands.
    assert np.all(surface.flight_agl_m == mission.cruise_agl_m)  # type: ignore[attr-defined]


def _canopy_mission(grid: Grid, **threat: object) -> object:
    """A mission whose one observer overlooks a stand from directly above it."""
    obs = grid.pixel_to_world(2, 2)
    return make_mission(
        crs="EPSG:32633",
        cruise_agl_m=10.0,
        altitude={"min_canopy_m": 12.0, "canopy_clearance_m": 3.0},
        threat={"observers": [{"x": obs.x, "y": obs.y}], **threat},
    )


def test_canopy_occlusion_reaches_the_cost_surface() -> None:
    """C1 must not be inert in the product it was written for.

    The gap fraction has to move the drone's exposure field, and it only can when
    the sensor is high enough to look *down* into the stand — which is why this
    test also pins ``observer_height_m``. With the pre-2026-08-11 hard-coded 2 m
    eye the planner could not represent that threat, and C1 changed nothing at all.
    """
    size = 24
    grid = _grid_for(size)
    dtm = np.zeros((size, size), dtype=np.float32)
    dsm = np.full((size, size), 20.0, dtype=np.float32)  # closed 20 m canopy
    dsm[0:4, 0:4] = 0.0  # ...with a clearing for the observer to stand in

    def exposure(gap: float, height_m: float) -> float:
        mission = _canopy_mission(
            grid, observer_height_m=height_m, canopy_occlusion={"sub_canopy_gap_fraction": gap}
        )
        surface = build_cost_surface(dtm, dsm, grid, mission)  # type: ignore[arg-type]
        return float(surface.components["exposure"].mean())

    # A sensor 500 m up looks down steeply: the crowns must start to matter.
    assert exposure(0.0, 500.0) < exposure(0.10, 500.0) < exposure(1.0, 500.0)
    # A dismount in the clearing is *under* the crown line, so its sightline into
    # the stand arrives through trunk space and no crown is in the way. The gap
    # fraction must not touch it — attenuating that ray would invent concealment.
    assert exposure(0.0, 2.0) == exposure(1.0, 2.0)


def test_cable_survivability_is_unaffected_by_the_drone_altitude() -> None:
    """The cable lies on the ground, so its openness term must not move with cruise AGL."""
    dem = make_dem("ridge", size=24)
    grid = _grid_for(24)
    low = build_cost_surface(
        dem.data, dem.data, grid, _mission_with_observer(grid, cruise_agl_m=5.0)
    )  # type: ignore[arg-type]
    high = build_cost_surface(
        dem.data, dem.data, grid, _mission_with_observer(grid, cruise_agl_m=200.0)
    )  # type: ignore[arg-type]

    assert np.array_equal(low.components["survivability"], high.components["survivability"])
    assert np.array_equal(low.components["ground_los"], high.components["ground_los"])
    assert high.components["exposure"].mean() > low.components["exposure"].mean()


def test_crown_transmission_reaches_the_cost_surface_and_is_off_by_default() -> None:
    """CanopyOcclusion.crown_transmission (2026-09-26): with it on, a look-down sensor
    sees the drone under the neighbouring crowns (the binary wall scored those 0); off,
    the field is bit-identical to what it was."""
    size = 24
    grid = _grid_for(size)
    dtm = np.zeros((size, size), dtype=np.float32)
    dsm = np.full((size, size), 20.0, dtype=np.float32)
    dsm[0:4, 0:4] = 0.0

    def field(**occ: object) -> np.ndarray:
        mission = _canopy_mission(grid, observer_height_m=300.0, canopy_occlusion=occ)
        surface = build_cost_surface(dtm, dsm, grid, mission)  # type: ignore[arg-type]
        out: np.ndarray = surface.components["exposure"]
        return out

    default = field(sub_canopy_gap_fraction=0.10)
    explicit_off = field(sub_canopy_gap_fraction=0.10, crown_transmission=False)
    on = field(sub_canopy_gap_fraction=0.10, crown_transmission=True)
    assert np.array_equal(default, explicit_off)
    assert not np.array_equal(default, on)
    assert float(on.max()) <= 1.0
