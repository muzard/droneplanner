"""Plan output data contract (Software-Spec §5.3 — the integration boundary).

This is what the flight stack consumes. Everything downstream (trajectory
GeoJSON, MAVLink ``.plan``, laydown polyline) is a serialization of a ``Plan``.
"""

from __future__ import annotations

import math
from enum import StrEnum

from pydantic import BaseModel, ConfigDict, Field

from droneplanner.model.geometry import Coordinate


class FlightBand(StrEnum):
    """Altitude band the drone occupies along a segment (Preflight-Design §4)."""

    CANOPY_TOP = "canopy_top"
    UPPER_TRUNK = "upper_trunk"
    BELOW_CANOPY = "below_canopy"
    OPEN = "open"
    UNKNOWN = "unknown"


class FlyabilityClass(StrEnum):
    """Forest navigability class (Preflight-Design §4 threshold table)."""

    FLYABLE = "flyable"
    MARGINAL = "marginal"
    NON_NAVIGABLE = "non_navigable"
    UNKNOWN = "unknown"


class CrossingMethod(StrEnum):
    """Road/linear-obstacle crossing method, ranked by automation feasibility."""

    CULVERT = "culvert"  # existing masked crossing — preferred, no manipulation
    CANOPY_BRIDGE = "canopy_bridge"  # opposing crowns meet overhead
    OVERHEAD_SPAN = "overhead_span"  # needs aerial manipulation — flag only
    OPEN = "open"  # last resort


class TrajectoryVertex(BaseModel):
    """One vertex of the 3D flight track with per-vertex flight attributes."""

    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float = Field(description="Altitude AGL (meters) — same value as altitude_agl_m.")
    altitude_agl_m: float = Field(description="Height above the ground *beneath this vertex*.")
    ground_elev_m: float = Field(
        default=0.0,
        description=(
            "Terrain elevation (AMSL) under this vertex, sampled from the DTM. Needed to "
            "convert terrain-relative AGL into the relative-to-home altitude a QGC .plan "
            "carries; 0.0 when no terrain was supplied (the M0 stub)."
        ),
    )
    flight_band: FlightBand = FlightBand.UNKNOWN
    speed_hint_ms: float = Field(gt=0)
    corridor_half_width_m: float = Field(
        default=0.0,
        ge=0,
        description=(
            "How far the flight stack may deviate laterally here without losing the "
            "concealment this vertex was planned for. Bounded by the local extent of "
            "the cover — it stops at the edge of a stand or a depression — so it is "
            "small in a thin screen of trees and large in the middle of one. 0.0 "
            "means it was not computed (the M0 stub), not that freedom is zero."
        ),
    )

    @property
    def altitude_amsl_m(self) -> float:
        """Absolute altitude above sea level."""
        return self.ground_elev_m + self.altitude_agl_m

    @property
    def coordinate(self) -> Coordinate:
        return Coordinate(x=self.x, y=self.y)


class RouteSegment(BaseModel):
    """A segment between two consecutive vertices, with per-segment classes."""

    model_config = ConfigDict(frozen=True)

    index: int = Field(ge=0)
    start: Coordinate
    end: Coordinate
    length_m: float = Field(ge=0)
    flyability: FlyabilityClass = FlyabilityClass.UNKNOWN
    flight_band: FlightBand = FlightBand.UNKNOWN
    exposed: bool = Field(
        default=False,
        description="The laid CABLE is visually exposed (viewshed of the ground surface).",
    )
    drone_exposed: bool = Field(
        default=False,
        description=(
            "The DRONE is visually exposed while flying this segment (viewshed evaluated at "
            "the planned flight altitude, not at ground level). Distinct from ``exposed``: a "
            "cable hidden in a draw can still be laid by an aircraft skylined above it."
        ),
    )


class Crossing(BaseModel):
    """A road/linear-obstacle crossing node with chosen method + flags."""

    model_config = ConfigDict(frozen=True)

    location: Coordinate
    method: CrossingMethod
    feasible: bool = True
    needs_manipulation: bool = Field(
        default=False, description="True if method requires unsolved aerial manipulation."
    )
    cost: float = Field(default=0.0, ge=0)
    label: str | None = None


class KinematicReport(BaseModel):
    """Tether/airframe feasibility of the trajectory (Software-Spec §3.6)."""

    model_config = ConfigDict(frozen=True)

    feasible: bool
    min_turn_radius_m: float = Field(ge=0)
    tightest_turn_radius_m: float = Field(ge=0)
    turn_violations: int = Field(default=0, ge=0)
    reversals: int = Field(default=0, ge=0)
    max_climb_rate_ms: float = Field(default=0.0, ge=0)
    max_descent_rate_ms: float = Field(default=0.0, ge=0)
    climb_violations: int = Field(default=0, ge=0)
    min_clearance_agl_m: float = Field(
        default=math.inf,
        description=(
            "Lowest AGL along the route under the altitude profile PX4 actually flies "
            "(front-loaded descent), not under linear interpolation. Can be negative — "
            "that means the plan flies underground. See planner/terrain_profile.py."
        ),
    )
    clearance_violations: int = Field(
        default=0,
        ge=0,
        description="Legs whose flown descent profile drops below the clearance floor.",
    )
    descent_vertices_inserted: int = Field(
        default=0,
        ge=0,
        description=(
            "Vertices added to split big-drop legs so the autopilot cannot get ahead "
            "of the terrain."
        ),
    )


class Plan(BaseModel):
    """A single planned route (one of ``k`` for redundancy)."""

    model_config = ConfigDict(frozen=True)

    mission_name: str
    crs: str
    route_index: int = Field(default=0, ge=0)
    seed: int = Field(ge=0)
    vertices: tuple[TrajectoryVertex, ...]
    segments: tuple[RouteSegment, ...] = ()
    crossings: tuple[Crossing, ...] = ()
    total_length_m: float = Field(ge=0)
    fiber_used_m: float = Field(ge=0)
    objective_scores: dict[str, float] = Field(default_factory=dict)
    kinematics: KinematicReport | None = None

    @property
    def start(self) -> Coordinate:
        return self.vertices[0].coordinate

    @property
    def end(self) -> Coordinate:
        return self.vertices[-1].coordinate
