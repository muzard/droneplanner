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
    max_climb_rate_ms: float = Field(
        default=0.0,
        ge=0,
        description=(
            "Peak rate of change of the AGL profile. This is what the altitude "
            "smoother constrains, and it is NOT the airframe's vertical rate — over "
            "rising ground a constant-AGL route has 0.00 m/s here while climbing "
            "steadily. See max_climb_rate_amsl_ms (SHORTFALLS 0b)."
        ),
    )
    max_descent_rate_ms: float = Field(default=0.0, ge=0, description="AGL profile; see above.")
    max_climb_rate_amsl_ms: float = Field(
        default=0.0,
        ge=0,
        description=(
            "Peak TRUE vertical rate (ground elevation + AGL), which is what the "
            "airframe's climb limit actually applies to. Measured and reported only "
            "— the smoother still constrains the AGL profile, so a route can be "
            "emitted with this over the limit (raises vertical_rate_amsl_exceeded)."
        ),
    )
    max_descent_rate_amsl_ms: float = Field(
        default=0.0, ge=0, description="Peak true vertical descent rate; see above."
    )
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


class EnduranceBudget(BaseModel):
    """Whether the route fits the airframe's endurance (TEST-PLAN N1).

    Separate from ``KinematicReport`` on purpose: that answers "is this shape
    flyable", this answers "is it flyable *before the battery runs out*". They
    fail independently — a perfectly smooth, perfectly cleared route is still
    infeasible if the speed its forest demands makes it too slow to get home.
    """

    model_config = ConfigDict(frozen=True)

    flight_time_s: float = Field(ge=0)
    cruise_time_s: float = Field(default=0.0, ge=0)
    vertical_bound_time_s: float = Field(
        default=0.0,
        ge=0,
        description="Extra time on legs where the climb/descent rate, not cruise, binds.",
    )
    waypoint_overhead_s: float = Field(
        default=0.0,
        ge=0,
        description=(
            "PX4's decelerate-in/accelerate-out at each acceptance radius, 0.161 s per "
            "waypoint measured on B1. At 25 m spacing this is paid 282 times."
        ),
    )
    terminal_overhead_s: float = Field(default=0.0, ge=0)
    energy_time_s: float = Field(
        default=0.0, ge=0, description="Flight time plus the climb power surcharge."
    )
    energy_fraction: float = Field(
        default=0.0,
        ge=0,
        description="Fraction of USABLE endurance consumed; > 1.0 breaks into the reserve.",
    )
    required_endurance_s: float = Field(
        default=0.0, ge=0, description="Endurance the route demands, reserve included."
    )
    endurance_s: float | None = None
    usable_endurance_s: float | None = None
    min_sustainable_speed_ms: float = Field(
        default=0.0,
        ge=0,
        description=(
            "Slowest cruise that still gets home. Endurance floors speed from below "
            "while obstacle avoidance caps it from above (TEST-PLAN B2); when the "
            "floor exceeds the ceiling no feasible speed exists."
        ),
    )
    sharp_corners: int = Field(
        default=0,
        ge=0,
        description=(
            "Vertices past 20 deg, where PX4's own corner deceleration would add time "
            "this model does not charge for. Zero on every planner route measured so "
            "far (demo max: 6.7 deg)."
        ),
    )
    vertical_travel_m: float = Field(default=0.0, ge=0)
    effective_speed_ms: float = Field(
        default=0.0,
        ge=0,
        description=(
            "Route length / cruise time — the speed the forest actually permits, "
            "after per-vertex stem-spacing caps (TEST-PLAN N3). Below the airframe's "
            "cruise speed wherever the stands are too tight to dodge at cruise."
        ),
    )
    speed_capped_legs: int = Field(
        default=0, ge=0, description="Legs the stems slowed below cruise speed."
    )

    @property
    def stated(self) -> bool:
        return self.endurance_s is not None

    @property
    def closes(self) -> bool:
        """Fits inside the usable budget with the reserve intact."""
        return self.stated and self.energy_fraction <= 1.0

    @property
    def exceeds_raw(self) -> bool:
        """Outruns the whole battery, reserve spent."""
        return self.endurance_s is not None and self.energy_time_s > self.endurance_s


class CanopyClimb(BaseModel):
    """A stretch inside the concealment zone flown OVER the canopy, not under it.

    The operator must know these before launch: the aircraft leaves cover and
    climbs to canopy height there, and a climb in the first few hundred metres
    exposes the launch site itself."""

    model_config = ConfigDict(frozen=True)

    start_m: float = Field(ge=0, description="Along-route distance from launch.")
    length_m: float = Field(ge=0)
    max_agl_m: float = Field(description="Highest planned AGL in the stretch.")
    max_canopy_m: float = Field(ge=0, description="Tallest canopy under the stretch.")
    reason: str = Field(
        description=(
            "'dense_stand' (denser than the onboard envelope), 'crowns_too_low' "
            "(crown base leaves no trunk space above the floor, AltitudePolicy.crown_ratio) "
            "or 'stand_too_short' (below AltitudePolicy.min_canopy_m, or no room under it)."
        )
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
    endurance: EnduranceBudget | None = None
    canopy_climbs: tuple[CanopyClimb, ...] = ()

    @property
    def start(self) -> Coordinate:
        return self.vertices[0].coordinate

    @property
    def end(self) -> Coordinate:
        return self.vertices[-1].coordinate


SPEED_STEP_MS = 0.5
"""Commanded leg speeds are floored to this step, so a speed that wobbles by
centimetres per second along a stand does not emit a change item at every vertex."""

SLOW_SPEED_STEP_MS = 0.1
"""Below one ``SPEED_STEP_MS`` the step is this instead."""


def commanded_speed_ms(cap_ms: float) -> float:
    """The speed the ``.plan`` commands for a leg whose cap is ``cap_ms`` — never
    above the cap.

    🔴 Two defects this closes (2026-09-25, Askola measured zone). The emitter
    floored to 0.5 m/s while the endurance model charged the raw cap, so a
    0.76 m/s leg was flown at 0.5 and charged at 0.76 (SITL: 21.3 min flown vs
    17.5 planned). And ``max(0.5, ...)`` turned a 0.4 m/s cap (1200 stems/ha)
    into 0.5 — faster than the planner's own limit. Both the emitter and the
    endurance budget now call this.
    """
    if cap_ms >= SPEED_STEP_MS:
        return math.floor(cap_ms / SPEED_STEP_MS + 1e-9) * SPEED_STEP_MS
    return max(
        SLOW_SPEED_STEP_MS, math.floor(cap_ms / SLOW_SPEED_STEP_MS + 1e-9) * SLOW_SPEED_STEP_MS
    )
