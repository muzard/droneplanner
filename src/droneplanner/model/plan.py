"""Plan output data contract (Software-Spec §5.3 — the integration boundary).

This is what the flight stack consumes. Everything downstream (trajectory
GeoJSON, MAVLink ``.plan``, laydown polyline) is a serialization of a ``Plan``.
"""

from __future__ import annotations

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
    altitude_agl_m: float
    flight_band: FlightBand = FlightBand.UNKNOWN
    speed_hint_ms: float = Field(gt=0)

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
    exposed: bool = Field(default=False, description="Visually exposed per the viewshed field.")


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

    @property
    def start(self) -> Coordinate:
        return self.vertices[0].coordinate

    @property
    def end(self) -> Coordinate:
        return self.vertices[-1].coordinate
