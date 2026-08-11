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
    endurance_s: float | None = Field(
        default=None,
        gt=0,
        description=(
            "Usable HOVER endurance carrying the fiber payload (seconds). Left None "
            "when the airframe is not yet chosen: the planner then reports the "
            "endurance the route *demands* instead of judging it, because a silent "
            "default would manufacture a NO-GO out of an assumption. Charging level "
            "flight at the hover rate is conservative for a multirotor — induced "
            "power falls with forward speed long before parasitic drag grows."
        ),
    )
    energy_reserve_fraction: float = Field(
        default=0.20,
        ge=0.0,
        lt=1.0,
        description=(
            "Battery held back from the plan's budget. A route that fits the full "
            "battery but eats this is REVIEW, not NO-GO — spending the reserve is a "
            "command decision, outrunning the battery is arithmetic."
        ),
    )

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


class CanopyOcclusion(BaseModel):
    """Whether a cell's own canopy may hide something flying *beneath* it.

    ``fields.viewshed`` marches over the DSM and deliberately excludes a cell's
    own surface from its occluders — correct for an aircraft *above* the surface,
    wrong for one *under* it. Without this the model sees straight down through
    20 m of crown: from a 300 m observer it scored **1570 m of a 3244 m
    below-canopy route as visible while the aircraft was physically beneath the
    canopy**. That is the one threat geometry where being under the canopy is the
    entire point, and the planner could not represent it at all.

    Near-horizontal sightlines are left alone: they already cross many crown
    cells, which the DSM march occludes correctly.
    """

    model_config = ConfigDict(frozen=True)

    sub_canopy_gap_fraction: float = Field(
        default=0.10,
        ge=0.0,
        le=1.0,
        description=(
            "Probability a sub-canopy target is seen through a crown gap. Closed "
            "Nordic stands run 5-20 %, so this is a configuration knob and not a "
            "constant: it is the dominant uncertainty in any below-canopy "
            "concealment claim, and no gap-fraction measurement for these stands "
            "has ever been ingested. 0.0 gives the fully-hidden bound that brackets "
            "a sensitivity analysis; 1.0 restores the pre-fix behaviour."
        ),
    )
    sub_canopy_max_elev_deg: float = Field(
        default=10.0,
        ge=0.0,
        le=90.0,
        description=(
            "Only sightlines steeper than this get the gap-fraction treatment. "
            "Below it the DSM march already models intervening canopy correctly, so "
            "the shallow path is left exactly as it was — this fix is about "
            "look-down observers, which are the ones the old model could not see."
        ),
    )


class ThreatModel(BaseModel):
    """Observer positions (viewshed sources) + threat fire sectors."""

    model_config = ConfigDict(frozen=True)

    observers: tuple[Coordinate, ...] = ()
    sectors: tuple[ThreatSector, ...] = ()
    canopy_occlusion: CanopyOcclusion = CanopyOcclusion()
    observer_height_m: float = Field(
        default=2.0,
        ge=0.0,
        description=(
            "Eye/mast/aperture height of every observer, above the surface they "
            "stand on. 2 m is a dismount, 12-30 m a radar mast, 300-1000 m an "
            "airborne sensor — and the industry guidance is explicit that raising a "
            "sensor from 2 to 12 m dramatically extends its detection horizon, so "
            "assume the adversary has read it.\n\n"
            "Until 2026-08-11 this was hard-coded at 2.0 in ``build_cost_surface``, "
            "which meant **the planner could not route against an elevated sensor at "
            "all** — and that is exactly the geometry where the canopy earns its "
            "keep, so :class:`CanopyOcclusion` had nothing to act on."
        ),
    )


class AltitudePolicy(BaseModel):
    """How low to fly, and what stops it going lower (Preflight-Design §4, §"primary posture").

    The drone hides by being **under the canopy**, not by being high over it: at a
    fixed 30 m AGL it clears a 22 m stand by 8 m and is a silhouette against sky.
    So the planner targets ``Mission.cruise_agl_m`` and this policy supplies the
    two things that stop it: a floor (understory, rotor wash, terrain error) and a
    ceiling (stay under the crowns) derived from the canopy height model.

    ⚠️ **Flying under the canopy is a bet on onboard perception.** This offline
    planner sees a canopy *top* (DSM - DTM); it cannot see trunks, branches or
    understory, and at DSM resolution it never will. The route is a corridor to
    follow approximately — local avoidance is the flight stack's job (camera /
    lidar), and where that is unavailable a below-canopy leg is not verified.
    """

    model_config = ConfigDict(frozen=True)

    min_agl_m: float = Field(
        default=5.0,
        gt=0,
        description="Hard floor above terrain; also the terrain-clearance margin.",
    )
    canopy_clearance_m: float = Field(
        default=3.0, gt=0, description="Vertical gap kept below the canopy top."
    )
    min_canopy_m: float = Field(
        default=12.0,
        gt=0,
        description=(
            "Only stands at least this tall are entered. A crude proxy for canopy base "
            "height, which a DSM cannot measure: mature open-structured stands have "
            "usable trunk space, young regen and thickets of the same footprint do not. "
            "Lidar replaces this guess with a measured free window."
        ),
    )


class Weights(BaseModel):
    """Operator-tunable objective weights (Preflight-Design §3)."""

    model_config = ConfigDict(frozen=True)

    exposure: float = Field(default=1.0, ge=0, description="Drone visual-exposure penalty.")
    cover: float = Field(default=1.0, ge=0, description="Reward for terrain/canopy cover.")
    length: float = Field(default=0.5, ge=0, description="Path-length penalty.")
    survivability: float = Field(default=1.0, ge=0, description="Cable survivability penalty.")
    snag: float = Field(default=0.5, ge=0, description="Fiber snag-risk penalty.")
    clutter: float = Field(
        default=0.0,
        ge=0,
        description=(
            "How hard to route AWAY from stands whose stem spacing forces the flight "
            "stack to slow down (TEST-PLAN N3), priced as extra seconds per metre — "
            "the currency the endurance budget spends.\n\n"
            "**Defaults to 0.0 deliberately.** Measuring the clutter is "
            "unconditional: the speed cap always feeds the endurance budget, so an "
            "unflyable route is always reported as one. *Trading concealment for it* "
            "is the operator's call, because on the demo AO the trade is brutal and "
            "bimodal — any weight at all drives the route out of the forest, cutting "
            "flight time 57.1 -> 16.3 min while raising drone exposure 0.150 -> 0.403 "
            "(2.7x). Defaulting this above zero would silently abandon the "
            "concealment posture the whole program exists for."
        ),
    )


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
    altitude: AltitudePolicy = AltitudePolicy()
    layers: LayerPaths = LayerPaths()
    k_routes: int = Field(default=1, ge=1, description="Survivability redundancy (k routes).")
    seed: int = Field(default=0, ge=0, description="Master seed for reproducible runs.")

    # Planner defaults (tunable per mission; kept here so the contract is complete).
    waypoint_spacing_m: float = Field(
        default=25.0,
        gt=0,
        description=(
            "Max spacing between emitted waypoints. This is what makes low flight "
            "possible: PX4 drives toward the next waypoint's altitude at its own "
            "vertical rate, so between distant waypoints the commanded profile cannot "
            "track the ground. Measured on the demo AO, a 10 m AGL route clears terrain "
            "at 25 m spacing and flies 12 m underground at the old 100 m."
        ),
    )
    cruise_agl_m: float = Field(
        default=10.0,
        gt=0,
        description=(
            "Target height above terrain. The planner flies this low wherever the "
            "AltitudePolicy floor and the canopy ceiling allow — it is a target to be "
            "clamped, not a cruise altitude to be held. Was 30 m until 2026-07-27, "
            "which put the aircraft above the canopy it was supposed to be hiding in."
        ),
    )
    fiber_slack_factor: float = Field(
        default=1.0, ge=1.0, description="Fiber-used / track-length ratio (>=1)."
    )
    corridor_half_width_max_m: float = Field(
        default=50.0,
        gt=0,
        description=(
            "Most lateral freedom the plan will ever grant the flight stack. The "
            "actual per-vertex width is whatever the local concealment allows — it "
            "stops at the edge of a stand or a depression — and this only caps it, "
            "because beyond some width 'follow this route' stops meaning anything."
        ),
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
