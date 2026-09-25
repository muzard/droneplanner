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


def test_the_cable_weight_steers_the_uncovered_route_onto_open_ground() -> None:
    """Opt-in ``weights.cable_on_crowns``: outside the concealment zone the aircraft
    flies over the canopy, so the fiber lies on the crowns — unless the route takes
    the clearing next to the forest."""
    from droneplanner.fields.composite import build_cost_surface
    from droneplanner.planner.corridor import plan_corridor
    from tests.fixtures.synthetic import make_mission

    cell, n = 25.0, 31
    grid = Grid(transform=(cell, 0.0, 0.0, 0.0, -cell, n * cell), rows=n, cols=n, crs="EPSG:32633")
    dtm = np.zeros((n, n), dtype=np.float32)
    dsm = np.full((n, n), 20.0, dtype=np.float32)  # closed forest
    dsm[n // 2 - 4, :] = 0.0  # a cleared ride 100 m off the straight line
    start, end = grid.pixel_to_world(n // 2, 1), grid.pixel_to_world(n // 2, n - 2)
    base = make_mission(
        name="ride",
        start={"x": start.x, "y": start.y},
        end={"x": end.x, "y": end.y},
        waypoint_spacing_m=cell,
        covered_radius_m=50.0,
    )

    def crowns(w: float) -> float:
        m = base.model_copy(
            update={"weights": base.weights.model_copy(update={"cable_on_crowns": w})}
        )
        plan = plan_corridor(m, build_cost_surface(dtm, dsm, grid, m))
        return plan.objective_scores["cable_on_crowns_m"]

    assert crowns(0.0) > 400.0
    assert crowns(2.0) < 0.5 * crowns(0.0)
