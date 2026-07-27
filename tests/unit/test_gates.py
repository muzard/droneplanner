"""Unit tests for gate reduction (TEST-PLAN A2).

"Enter the forest patch here and exit here" — the claim under test is that a
dense route can be replaced by a handful of gates without giving up the
concealment it was scored on, and the decision rule is that a gate-to-gate
straight must stay inside the A1 corridor.
"""

from __future__ import annotations

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.planner.gates import assess_gate_altitude_risk, reduce_to_gates

CELL_M = 10.0
SIZE = 41


def _grid() -> Grid:
    return Grid(
        transform=(CELL_M, 0.0, 0.0, 0.0, -CELL_M, SIZE * CELL_M),
        rows=SIZE,
        cols=SIZE,
        crs="EPSG:32633",
    )


def _straight_route(grid: Grid, col: int = 20) -> list[Coordinate]:
    return [grid.pixel_to_world(r, col) for r in range(5, 36)]


def test_a_straight_route_reduces_to_its_endpoints() -> None:
    """31 waypoints down one column carry no shape at all."""
    grid = _grid()
    route = _straight_route(grid)
    corridor = np.ones((SIZE, SIZE), dtype=bool)
    red = reduce_to_gates(route, corridor, grid)
    assert red.count == 2
    assert red.indices == (0, len(route) - 1)
    assert red.max_deviation_m < CELL_M


def test_first_and_last_vertices_are_always_gates() -> None:
    grid = _grid()
    route = _straight_route(grid)
    red = reduce_to_gates(route, np.ones((SIZE, SIZE), dtype=bool), grid)
    assert red.indices[0] == 0
    assert red.indices[-1] == len(route) - 1
    assert red.gates[0] == route[0]
    assert red.gates[-1] == route[-1]


def test_a_gate_is_forced_where_the_straight_would_leave_the_corridor() -> None:
    """A dog-leg around an obstruction cannot be flown straight through it."""
    grid = _grid()
    # Route goes down column 15, steps across to column 25, continues down.
    route = [grid.pixel_to_world(r, 15) for r in range(5, 20)]
    route += [grid.pixel_to_world(20, c) for c in range(15, 26)]
    route += [grid.pixel_to_world(r, 25) for r in range(21, 36)]

    corridor = np.zeros((SIZE, SIZE), dtype=bool)
    for p in route:  # the corridor hugs the route, so the shortcut is excluded
        r, c = grid.world_to_pixel(p)
        corridor[r, c] = True

    red = reduce_to_gates(route, corridor, grid)
    assert 3 <= red.count <= 6, f"expected gates at the dog-leg, got {red.count}"
    # The corner itself must survive — cutting it would leave the corridor.
    corner = grid.world_to_pixel(grid.pixel_to_world(20, 15))
    assert any(grid.world_to_pixel(g) == corner for g in red.gates)


def test_a_zero_width_corridor_keeps_every_vertex() -> None:
    """Degenerate but must terminate, and must not silently invent freedom."""
    grid = _grid()
    route = _straight_route(grid)
    red = reduce_to_gates(route, np.zeros((SIZE, SIZE), dtype=bool), grid)
    assert red.count == len(route)


def test_degenerate_routes() -> None:
    grid = _grid()
    corridor = np.ones((SIZE, SIZE), dtype=bool)
    assert reduce_to_gates([], corridor, grid).count == 0
    single = [grid.pixel_to_world(10, 10)]
    assert reduce_to_gates(single, corridor, grid).count == 1


def test_deviation_reports_the_route_shape_that_was_discarded() -> None:
    """A bulging route replaced by a chord: the bulge is what the gates give up."""
    grid = _grid()
    route = [grid.pixel_to_world(5, 20), grid.pixel_to_world(20, 24), grid.pixel_to_world(35, 20)]
    red = reduce_to_gates(route, np.ones((SIZE, SIZE), dtype=bool), grid)
    assert red.count == 2
    assert 3 * CELL_M < red.max_deviation_m < 5 * CELL_M


def test_gate_reduction_makes_the_altitude_steps_much_larger() -> None:
    """The cost that decides whether gates are usable at all.

    Dense waypoints keep each leg's altitude step small, which is what let the
    low route sequence cleanly in SITL (B1). Gates put the long legs back, so the
    altitude has to come from an executor that follows terrain itself.
    """
    grid = _grid()
    route = _straight_route(grid)
    altitudes = [100.0 + 2.0 * i for i in range(len(route))]  # 2 m per leg
    red = reduce_to_gates(route, np.ones((SIZE, SIZE), dtype=bool), grid)
    risk = assess_gate_altitude_risk(route, altitudes, red.indices)

    assert risk.dense_max_step_m == 2.0
    assert risk.max_step_m == 2.0 * (len(route) - 1)  # the whole climb in one leg
    assert risk.max_step_m > 10 * risk.dense_max_step_m
