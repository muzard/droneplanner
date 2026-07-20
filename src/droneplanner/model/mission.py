"""Mission input data contract (Software-Spec §4).

A ``Mission`` is the single validated entry point for a planning run. It is
loaded from YAML by ``io.mission`` (which also handles lon/lat -> working-CRS
coordinate conversion) and is otherwise treated as immutable.
"""

from __future__ import annotations

from enum import StrEnum
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from droneplanner.model.geometry import Coordinate, validate_projected_crs


class PayoutSide(StrEnum):
    """Where the fiber spool pays out from (Technology-Selection §2.2)."""

    DRONE = "drone"
    GROUND = "ground"


class DroneSpec(BaseModel):
    """Airframe geometry + kinematic limits that constrain the plan."""

    model_config = ConfigDict(frozen=True)

    rotor_span_m: float = Field(gt=0, description="Tip-to-tip width incl. props (meters).")
    mass_kg: float = Field(gt=0)
    min_turn_radius_m: float = Field(gt=0, description="Min horizontal turn radius (meters).")
    climb_rate_max_ms: float = Field(gt=0)
    descent_rate_max_ms: float = Field(gt=0)
    cruise_speed_ms: float = Field(default=5.0, gt=0)

    @property
    def rotor_radius_m(self) -> float:
        """Airframe half-width including props (the clearance term ``r_drone``)."""
        return self.rotor_span_m / 2.0


class SpoolSpec(BaseModel):
    """Fiber spool: the mass budget driver and the hard length limit."""

    model_config = ConfigDict(frozen=True)

    length_m: float = Field(gt=0, description="Available fiber length (meters).")
    mass_kg: float = Field(gt=0)
    payout_side: PayoutSide = PayoutSide.DRONE


class ThreatSector(BaseModel):
    """A threat fire sector: an azimuth range, optionally from an observer point.

    "Threat" enters the planner solely as a passive survivability cost on cable
    placement (Software-Spec §8) — never as targeting logic.
    """

    model_config = ConfigDict(frozen=True)

    azimuth_start_deg: float = Field(ge=0, le=360)
    azimuth_end_deg: float = Field(ge=0, le=360)
    observer: Coordinate | None = None
    label: str | None = None


class ThreatModel(BaseModel):
    """Observer positions (viewshed sources) + threat fire sectors."""

    model_config = ConfigDict(frozen=True)

    observers: tuple[Coordinate, ...] = ()
    sectors: tuple[ThreatSector, ...] = ()


class Weights(BaseModel):
    """Operator-tunable objective weights (Preflight-Design §3)."""

    model_config = ConfigDict(frozen=True)

    exposure: float = Field(default=1.0, ge=0, description="Drone visual-exposure penalty.")
    cover: float = Field(default=1.0, ge=0, description="Reward for terrain/canopy cover.")
    length: float = Field(default=0.5, ge=0, description="Path-length penalty.")
    survivability: float = Field(default=1.0, ge=0, description="Cable survivability penalty.")
    snag: float = Field(default=0.5, ge=0, description="Fiber snag-risk penalty.")


class LayerPaths(BaseModel):
    """Optional paths to input map layers (consumed from M1 onward).

    Absent layers are fine for the M0 stub; the planner degrades gracefully and
    the report flags what analysis it could not perform.
    """

    model_config = ConfigDict(frozen=True)

    dtm: Path | None = Field(default=None, description="Bare-earth DTM GeoTIFF.")
    dsm: Path | None = Field(default=None, description="Surface DSM GeoTIFF.")
    chm: Path | None = Field(default=None, description="Canopy height model GeoTIFF.")
    land_cover: Path | None = None
    point_cloud: Path | None = Field(default=None, description="ALS/UAV lidar LAZ/LAS 1.4.")
    roads: Path | None = None
    rail: Path | None = None
    buildings: Path | None = None
    culverts: Path | None = None


class Mission(BaseModel):
    """Validated mission definition — the planner's sole input contract."""

    model_config = ConfigDict(frozen=True)

    name: str = Field(min_length=1)
    crs: str = Field(description="Projected metric working CRS, e.g. 'EPSG:32633'.")
    start: Coordinate
    end: Coordinate
    drone: DroneSpec
    spool: SpoolSpec
    threat: ThreatModel = ThreatModel()
    weights: Weights = Weights()
    layers: LayerPaths = LayerPaths()
    k_routes: int = Field(default=1, ge=1, description="Survivability redundancy (k routes).")
    seed: int = Field(default=0, ge=0, description="Master seed for reproducible runs.")

    # Planner defaults (tunable per mission; kept here so the contract is complete).
    waypoint_spacing_m: float = Field(default=100.0, gt=0)
    cruise_agl_m: float = Field(default=30.0, gt=0, description="Default cruise altitude AGL.")
    fiber_slack_factor: float = Field(
        default=1.0, ge=1.0, description="Fiber-used / track-length ratio (>=1)."
    )

    @field_validator("crs")
    @classmethod
    def _crs_projected(cls, v: str) -> str:
        return validate_projected_crs(v)

    @model_validator(mode="after")
    def _endpoints_distinct(self) -> Mission:
        if self.start.distance_to(self.end) <= 0.0:
            raise ValueError("mission start and end must be distinct points")
        return self
