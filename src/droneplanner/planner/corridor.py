"""2.5D corridor planner (Software-Spec §5.3 v0, milestone M1).

Ties the composite cost surface to the any-angle Theta* search and packages the
result as a ``Plan``: a concealment- and defilade-aware route with per-segment
exposure and route-level objective scores. No forest (3D) refinement yet — that
is M2 — so flyability is left UNKNOWN and altitude is constant-AGL terrain
following.
"""

from __future__ import annotations

import numpy as np

from droneplanner.fields.composite import CostSurface, build_cost_surface
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission
from droneplanner.model.plan import (
    FlightBand,
    FlyabilityClass,
    Plan,
    RouteSegment,
    TrajectoryVertex,
)
from droneplanner.planner.theta_star import path_cost, plan_path
from droneplanner.raster_lines import bresenham

# A path cell counts as "exposed" when this fraction of observers can see it.
EXPOSED_THRESHOLD = 0.5


def _segment_cells(a: tuple[int, int], b: tuple[int, int]) -> list[tuple[int, int]]:
    return bresenham(a[0], a[1], b[0], b[1])


def _mean_over(field: np.ndarray, cells: list[tuple[int, int]]) -> float:
    if not cells:
        return 0.0
    return float(np.mean([field[r, c] for r, c in cells]))


def plan_corridor(mission: Mission, surface: CostSurface, route_index: int = 0) -> Plan:
    """Plan the M1 concealment-aware corridor route over a cost surface."""
    grid = surface.grid
    start_rc = grid.world_to_pixel(mission.start)
    goal_rc = grid.world_to_pixel(mission.end)

    cell_path = plan_path(surface.cost, start_rc, goal_rc)
    exposure = surface.components["exposure"]
    agl = mission.cruise_agl_m  # M1 flies constant AGL (terrain following is M2)
    speed = mission.drone.cruise_speed_ms

    # World coordinates: exact mission endpoints, cell centers for turning points.
    def world_of(i: int, rc: tuple[int, int]) -> Coordinate:
        if i == 0:
            return mission.start
        if i == len(cell_path) - 1:
            return mission.end
        return grid.pixel_to_world(rc[0], rc[1])

    coords = [world_of(i, rc) for i, rc in enumerate(cell_path)]

    vertices = tuple(
        TrajectoryVertex(
            x=coord.x,
            y=coord.y,
            z=agl,
            altitude_agl_m=agl,
            flight_band=FlightBand.OPEN,
            speed_hint_ms=speed,
        )
        for coord in coords
    )

    segments: list[RouteSegment] = []
    all_cells: list[tuple[int, int]] = []
    for i in range(len(cell_path) - 1):
        seg_cells = _segment_cells(cell_path[i], cell_path[i + 1])
        all_cells.extend(seg_cells if i == 0 else seg_cells[1:])
        seg_exposed = _mean_over(exposure, seg_cells) > EXPOSED_THRESHOLD
        segments.append(
            RouteSegment(
                index=i,
                start=coords[i],
                end=coords[i + 1],
                length_m=coords[i].distance_to(coords[i + 1]),
                flyability=FlyabilityClass.UNKNOWN,
                flight_band=FlightBand.OPEN,
                exposed=seg_exposed,
            )
        )

    total_length = sum(s.length_m for s in segments)
    fiber_used = total_length * mission.fiber_slack_factor

    path_exposures = [float(exposure[r, c]) for r, c in all_cells]
    mean_exposure = float(np.mean(path_exposures)) if path_exposures else 0.0
    max_exposure = float(np.max(path_exposures)) if path_exposures else 0.0
    exposed_fraction = (
        float(np.mean([e > EXPOSED_THRESHOLD for e in path_exposures])) if path_exposures else 0.0
    )

    return Plan(
        mission_name=mission.name,
        crs=mission.crs,
        route_index=route_index,
        seed=mission.seed,
        vertices=vertices,
        segments=tuple(segments),
        crossings=(),
        total_length_m=total_length,
        fiber_used_m=fiber_used,
        objective_scores={
            "path_cost": path_cost(surface.cost, cell_path),
            "length_m": total_length,
            "mean_exposure": mean_exposure,
            "max_exposure": max_exposure,
            "exposed_fraction": exposed_fraction,
        },
    )


def plan_over_terrain(
    mission: Mission,
    dtm: np.ndarray,
    dsm: np.ndarray,
    grid: Grid,
    route_index: int = 0,
) -> tuple[Plan, CostSurface]:
    """Build the cost surface for a terrain and plan the corridor over it."""
    surface = build_cost_surface(dtm, dsm, grid, mission)
    plan = plan_corridor(mission, surface, route_index=route_index)
    return plan, surface


def straight_line_ground_truth(mission: Mission) -> list[Coordinate]:
    """The naive straight start->end line, for the viz to contrast against."""
    return [mission.start, mission.end]
