"""Where the fiber settles (roadmap C7, minimal): trunk space, crowns or ground."""

from __future__ import annotations

import numpy as np

from droneplanner.laydown.settle import cable_settle_lengths
from droneplanner.model.grid import Grid
from droneplanner.model.plan import FlightBand, TrajectoryVertex


def _v(x: float, band: FlightBand) -> TrajectoryVertex:
    return TrajectoryVertex(
        x=x,
        y=5.0,
        z=10.0,
        altitude_agl_m=10.0,
        ground_elev_m=0.0,
        flight_band=band,
        speed_hint_ms=3.0,
    )


def test_released_under_the_canopy_drops_through_over_it_stays_on_the_crowns() -> None:
    grid = Grid(transform=(10.0, 0.0, 0.0, 0.0, -10.0, 10.0), rows=1, cols=4, crs="EPSG:32633")
    chm = np.array([[20.0, 20.0, 20.0, 0.0]], dtype=np.float32)  # forest, then a clearing
    opening = np.array([[0.0, 0.0, 0.0, 1.0]], dtype=np.float32)  # only the clearing
    verts = [
        _v(5.0, FlightBand.BELOW_CANOPY),
        _v(15.0, FlightBand.BELOW_CANOPY),  # 0-1: under -> trunk space
        _v(25.0, FlightBand.CANOPY_TOP),  # 1-2: over closed crowns
        _v(35.0, FlightBand.OPEN),  # 2-3: over the opening / clearing -> ground
    ]
    got = cable_settle_lengths(verts, chm, grid, opening)
    assert got["cable_in_trunk_space_m"] == 10.0
    assert got["cable_on_ground_m"] == 10.0
    assert got["cable_on_crowns_m"] == 10.0
    assert got["cable_on_crowns_in_zone_m"] == 10.0  # no zone given = everywhere


def test_without_an_opening_raster_only_open_ground_catches_it() -> None:
    grid = Grid(transform=(10.0, 0.0, 0.0, 0.0, -10.0, 10.0), rows=1, cols=2, crs="EPSG:32633")
    chm = np.array([[15.0, 15.0]], dtype=np.float32)
    verts = [_v(5.0, FlightBand.CANOPY_TOP), _v(15.0, FlightBand.CANOPY_TOP)]
    got = cable_settle_lengths(verts, chm, grid, None)
    assert got["cable_on_crowns_m"] == 10.0 and got["cable_on_ground_m"] == 0.0
