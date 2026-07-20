"""M0 straight-line stub planner (Software-Spec §7, milestone M0).

Produces a deterministic straight-line 3D trajectory from start to end at a
constant cruise AGL, sampled at the mission's waypoint spacing. It performs no
terrain, viewshed, or forest analysis — the report (see ``report.build``) makes
that limitation explicit so nothing is executed blind. M1 replaces this with the
Theta* corridor search over the composite cost surface.
"""

from __future__ import annotations

import math

from droneplanner.model.geometry import Coordinate
from droneplanner.model.mission import Mission
from droneplanner.model.plan import (
    FlightBand,
    FlyabilityClass,
    Plan,
    RouteSegment,
    TrajectoryVertex,
)


def _sample_points(start: Coordinate, end: Coordinate, spacing_m: float) -> list[Coordinate]:
    """Sample the start->end segment at ``spacing_m``, always including endpoints."""
    total = start.distance_to(end)
    n_intervals = max(1, math.ceil(total / spacing_m))
    points: list[Coordinate] = []
    for i in range(n_intervals + 1):
        t = i / n_intervals
        points.append(
            Coordinate(
                x=start.x + (end.x - start.x) * t,
                y=start.y + (end.y - start.y) * t,
            )
        )
    return points


def straight_line_plan(mission: Mission, route_index: int = 0) -> Plan:
    """Build the deterministic straight-line stub plan for a mission."""
    points = _sample_points(mission.start, mission.end, mission.waypoint_spacing_m)
    speed = mission.drone.cruise_speed_ms
    agl = mission.cruise_agl_m

    vertices = tuple(
        TrajectoryVertex(
            x=p.x,
            y=p.y,
            z=agl,
            altitude_agl_m=agl,
            flight_band=FlightBand.UNKNOWN,
            speed_hint_ms=speed,
        )
        for p in points
    )

    segments = tuple(
        RouteSegment(
            index=i,
            start=points[i],
            end=points[i + 1],
            length_m=points[i].distance_to(points[i + 1]),
            flyability=FlyabilityClass.UNKNOWN,
            flight_band=FlightBand.UNKNOWN,
            exposed=False,
        )
        for i in range(len(points) - 1)
    )

    total_length = sum(s.length_m for s in segments)
    fiber_used = total_length * mission.fiber_slack_factor

    return Plan(
        mission_name=mission.name,
        crs=mission.crs,
        route_index=route_index,
        seed=mission.seed,
        vertices=vertices,
        segments=segments,
        crossings=(),
        total_length_m=total_length,
        fiber_used_m=fiber_used,
        objective_scores={"length_m": total_length},
    )
