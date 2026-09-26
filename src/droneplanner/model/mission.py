"""Mission input data contract (Software-Spec §4).

A ``Mission`` is the single validated entry point for a planning run. It is
loaded from YAML by ``io.mission`` (which also handles lon/lat -> working-CRS
coordinate conversion) and is otherwise treated as immutable.
"""

from __future__ import annotations

import math
from enum import StrEnum
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from droneplanner.model.geometry import Coordinate, validate_projected_crs

_ARCMIN_RAD = math.pi / (180.0 * 60.0)


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
    trunk_space_sightlines: bool = Field(
        default=False,
        description=(
            "Let a near-horizontal sightline pass UNDER the crowns, attenuated by the "
            "trunks, instead of treating the canopy as a wall (2026-09-25). An observer "
            "standing on open ground can see a drone flying through a stand's trunk "
            "space from its edge; the DSM march cannot. Transmittance exp(-stems/m^2 x "
            "trunk_diameter_m x distance) under the crown base, blocked in the crown "
            "layer, only within trunk_space_max_range_m. Off by default: it raises "
            "below-canopy exposure near forest edges and moves every historical figure. "
            "Observers standing inside a stand (eye on the crowns in this model) are "
            "not covered — see SHORTFALLS."
        ),
    )
    trunk_diameter_m: float = Field(default=0.25, gt=0.0, le=2.0)
    trunk_space_max_range_m: float = Field(default=300.0, gt=0.0)
    trunk_space_crown_ratio: float = Field(
        default=0.5,
        gt=0.0,
        lt=1.0,
        description="Crown base = canopy x (1 - this) when AltitudePolicy.crown_ratio is unset.",
    )
    crown_transmission: bool = Field(
        default=False,
        description=(
            "Score every sightline by TRANSMITTANCE through the vegetation it crosses "
            "instead of the binary DSM horizon (2026-09-26). Terrain (DTM) still blocks; "
            "the canopy (DSM - DTM) becomes a medium: in the crown layer each cell "
            "attenuates by its vertical gap probability per crown depth "
            "(gap^(path/depth), turbid medium calibrated at nadir), under the crown base "
            "the trunks do (exp(-stems/m^2 x trunk_diameter_m x path)). Fixes the binary "
            "wall that scored a drone at 5-7 m under a neighbouring crown as exactly 0. "
            "Replaces the sub-canopy steepness rule and trunk_space_sightlines (both "
            "become special cases), and the observer stands on the TERRAIN (dtm + observer "
            "height), not on the crowns. Off by default: it moves every historical figure."
        ),
    )
    crown_open_below_m: float = Field(
        default=2.0,
        ge=0.0,
        description="Crown transmission: a cell with less canopy than this has no crown.",
    )


class DetectionModel(BaseModel):
    """When a target is close enough to be *resolved*, not merely in line of sight.

    The viewshed answers "does a sightline exist". It said nothing about whether a
    0.9 m object is resolvable down that line, and had **no range term at all** — a
    cell 3 km from an observer scored exactly like one at 300 m. Measured
    consequence: the median exposed sample on every study arm sits at **1.3-1.6
    arcmin**, two to five times below the angle at which observers actually
    acquired real sUAS in trials, and against a far more favourable background
    (sky, not forest). Most of the "exposure" the planner has been routing away
    from is at ranges where nobody was going to see this aircraft at any altitude.

    ⚠️ This models **acquisition by a searching human eye or optic** and nothing
    else. It is not a radar model, not an acoustic one, and not a thermal one —
    acoustic C-UAS ranges (300-500 m for small quadcopters) are of the same order
    as the unaided visual threshold and are essentially independent of flight band.
    """

    model_config = ConfigDict(frozen=True)

    threshold_arcmin: float = Field(
        default=3.0,
        gt=0.0,
        description=(
            "Angular size at which the airframe is acquired. 3' is the empirical "
            "figure and the agreement across independent trials is good: DJI Mavic "
            "Pro (0.335 m) identified at 310 m = 3.7'; Inspire 2 (0.6 m) at 698 m = "
            "2.95'; AeroVironment Wasp (0.72 m) at 760 m = 3.26'. NTSB's 'reasonable "
            "chance of detection' figure is 12', which is far more forgiving, so 3.0 "
            "is the **pessimistic, adversary-favouring** default: it grants the "
            "observer roughly 4x the range NTSB would."
        ),
    )
    optic_magnification: float = Field(
        default=1.0,
        ge=1.0,
        description=(
            "Observation optic in front of the eye. This is the input that can erase "
            "the range argument entirely: at 12x the NTSB threshold reaches 3.1 km, "
            "which covers a whole AO — so a tripod-mounted spotting scope searching "
            "the right bearing is a different threat from a dismount, and the "
            "difference belongs to the operator, not to a constant."
        ),
    )
    falloff: Literal["hard", "ramp"] = Field(
        default="ramp",
        description=(
            "'ramp' holds full weight to half the acquisition range and falls "
            "linearly to zero at it — detection degrades, it does not switch off. "
            "'hard' is the step function, for sensors quoted with a stated range."
        ),
    )

    def max_range_m(self, target_span_m: float) -> float:
        """Range at which ``target_span_m`` subtends exactly ``threshold_arcmin``.

        For this programme's 0.9 m airframe unaided at 3', that is **1031 m**.
        """
        return target_span_m * self.optic_magnification / (self.threshold_arcmin * _ARCMIN_RAD)


class ThreatSector(BaseModel):
    """A threat fire sector: an azimuth range, optionally from an observer point.

    "Threat" enters the planner solely as a passive survivability cost on cable
    placement (Software-Spec §8) — never as targeting logic.

    Azimuths are compass bearings (0 = grid north, clockwise) and the arc sweeps
    **clockwise** from start to end, so 350->20 is a 30 deg arc through north.
    A degenerate arc (start == end) reads as the full circle — see
    ``fields.sectors``, which is where the geometry lives.

    With an ``observer`` this restricts *that observer's* viewshed to the arc.
    Without one it is a threat **bearing** with no located sensor, which feeds
    the cable's ``openness_to_threat_sectors`` field instead (Software-Spec
    §5.1). The two are different questions, so they drive different layers.
    """

    model_config = ConfigDict(frozen=True)

    azimuth_start_deg: float = Field(ge=0, le=360)
    azimuth_end_deg: float = Field(ge=0, le=360)
    observer: Coordinate | None = None
    label: str | None = None
    detection: DetectionModel | None = Field(
        default=None,
        description=(
            "Optic searching *this* arc, overriding ``ThreatModel.detection``. This "
            "is the input roadmap C2 left dangling: once exposure became a detection "
            "probability, unaided exposure on the demo AO went to 0.000 for every "
            "flight band while a 12x optic restored the whole pre-C2 picture — so the "
            "live question is not how high to fly but whether magnified optics are "
            "searching that bearing. A spotting scope on one arc with dismounts "
            "elsewhere is a real posture and now expressible. None inherits the "
            "mission-wide model."
        ),
    )


# Detection weight outside a stated search arc, as a fraction of on-axis. A sector
# concentrates attention; it does not blind the observer elsewhere (user,
# 2026-09-14). This is the default EVERYWHERE - this model field and the viewshed
# signatures alike - so a call site that forgets to pass the mission's value errs
# toward a MORE attentive adversary. The opposite default (0.0, a hard mask)
# shipped for one afternoon and immediately desynchronised the flown re-score from
# the raster, which is precisely the failure mode C3 warned about.
DEFAULT_OFF_AXIS_VIGILANCE = 0.25


class ThreatModel(BaseModel):
    """Observer positions (viewshed sources) + threat fire sectors."""

    model_config = ConfigDict(frozen=True)

    observers: tuple[Coordinate, ...] = ()
    sectors: tuple[ThreatSector, ...] = ()
    canopy_occlusion: CanopyOcclusion = CanopyOcclusion()
    detection: DetectionModel = DetectionModel()
    off_axis_vigilance: float = Field(
        default=DEFAULT_OFF_AXIS_VIGILANCE,
        ge=0.0,
        le=1.0,
        description=(
            "Detection weight on bearings OUTSIDE an observer's stated search "
            "arcs, as a fraction of the on-axis weight. A sector says where "
            "attention is concentrated, not where the observer is blind - per "
            "the user (2026-09-14), 'realistically observers will be looking "
            "around, even by accident'. "
            "0.0 is the pre-2026-09-14 behaviour: a hard mask, an adversary who "
            "physically cannot see off-axis. That is the optimistic direction, "
            "and a concealment planner must not err that way - it lets the route "
            "hug ground the enemy is merely less likely to be watching and score "
            "it as unobservable. 1.0 collapses the sector back to an all-round "
            "threat. WARNING: the default 0.25 is an ASSUMPTION, not a "
            "measurement - roughly 'three-quarters of attention on the stated "
            "arc'. No trial data backs it. Sweep it before quoting any sectored "
            "exposure number."
        ),
    )
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


class StemPolicy(BaseModel):
    """How much room to leave around a mapped stem, and how far the line may move.

    ``clearance_margin_m`` is added to the airframe half-width to get the air gap
    the refined route must keep. The default 0.5 m is the *geometric* minimum and
    is deliberately conservative about nothing: N4's sweep showed **5x the margin
    costs 1.3 % more distance** (0.50 m of air -> 1.016x, 2.40 m -> 1.029x), with
    the cliff at 3.40 m where the 4.6 m mean spacing runs out.

    🔴 The number that should drive this is not geometry, it is **tracking error**.
    PX4 mission mode held cross-track to mean 0.42 m but **max 2.28 m** over B1, so
    a route planned with 0.95 m of air can still be flown into a trunk by ordinary
    tracking error. Raise the margin to cover the tracking the executor actually
    achieves; the distance cost of doing so is close to free.
    """

    model_config = ConfigDict(frozen=True)

    clearance_margin_m: float = Field(default=0.5, ge=0.0)
    min_half_width_m: float = Field(
        default=4.0,
        ge=0.0,
        description=(
            "Lateral room the refinement may use even where the concealment "
            "corridor grants none. A route that cannot deviate cannot avoid a "
            "trunk standing on it; spending this is a concealment cost and is "
            "reported as such."
        ),
    )
    max_half_width_m: float = Field(
        default=8.0,
        gt=0.0,
        description=(
            "Upper clamp, and the load-bearing one. Given A1's median 50 m "
            "corridor an A* does not refine the route, it SHORTCUTS it — taking "
            "the straight chord across every bend, which lost the concealment "
            "shape and made clearance worse the first time this ran."
        ),
    )
    density_per_ha: float | None = Field(
        default=None,
        gt=0.0,
        description=(
            "Stated stand density, stems/ha, replacing the canopy-height inference "
            "(fields/stem_density.py) on every treed cell. That inference is forestry "
            "literature fitted to one municipality, and on the demo AO it says 640-1460 "
            "stems/ha while the Gazebo forest is 107 — so a mission flown in SITL "
            "must state the density of the world it flies, or its speed and "
            "endurance are priced for a different forest. Operationally this is "
            "the place for a stand-inventory figure."
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
    crown_ratio: float | None = Field(
        default=None,
        gt=0,
        lt=1,
        description=(
            "Live-crown length / tree height. When set, the below-canopy ceiling is "
            "also held under the crown BASE (height x (1 - crown_ratio)) by "
            "crown_clearance_m, not just under the canopy top — so the aircraft flies "
            "in trunk space, not through the crowns. None = the canopy-top ceiling "
            "only (every baseline before 2026-09-25)."
        ),
    )
    crown_clearance_m: float = Field(
        default=1.0, ge=0, description="Vertical gap kept below the crown base."
    )
    skim_lookahead_m: float = Field(
        default=40.0,
        ge=0,
        description=(
            "Over the canopy, fly at the TALLEST canopy within this distance plus the "
            "clearance — not each cell's own top. Real canopy is jagged at 16 m cells "
            "(10-20 m steps); skimming it cell by cell demanded 4-5 m/s of climb at "
            "cruise and could reach a taller stand below its crowns. ~40 m is the ground "
            "a 20 m climb takes at 3 m/s and 6 m/s. 0 = the old per-cell skim."
        ),
    )
    over_canopy_when_unpassable: bool = Field(
        default=True,
        description=(
            "Inside the concealment zone, fly OVER a stand that cannot be flown under "
            "(denser than the onboard planner has been shown to fly) instead of "
            "planning a NO-GO leg through it. Every such climb is listed in the report "
            "(canopy_climbs, finding canopy_climb_in_zone) so the operator knows before "
            "launch. False = the old behaviour: plan under it and raise "
            "stem_density_beyond_envelope."
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
    cable_on_crowns: float = Field(
        default=0.0,
        ge=0,
        description=(
            "How hard to route AWAY from flying over closed canopy, where the laid "
            "fiber settles on the crowns instead of the ground (laydown/settle.py, "
            "roadmap C7): a per-metre penalty on cells flown over canopy >= 3 m with "
            "no opening. 0 (default) keeps every route unchanged; the report measures "
            "the crown length either way (finding cable_on_crowns). Whether the cable's "
            "survival is worth the route change is the operator's call."
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
    stand_data: Path | None = Field(
        default=None,
        description=(
            "Finnish Forest Centre stand GeoPackage (Metsävarakuviot, CC BY 4.0). "
            "Its per-stand stemcount replaces the canopy-height density inference "
            "wherever a stand covers a treed cell (io/stand_data.py). A planning "
            "prior only: stem.density_per_ha, if stated, still wins."
        ),
    )
    roads: Path | None = None
    canopy_gap_fraction: Path | None = Field(
        default=None,
        description=(
            "Raster on the DTM grid: per-cell probability a look-down sightline finds a "
            "gap in the crowns (fraction of the 1 m canopy model open below the flight "
            "band; tools/build_finnish_ao.py writes gapfrac.tif). Replaces "
            "threat.canopy_occlusion.sub_canopy_gap_fraction cell by cell. Askola "
            "(2026-09-26): median 0.04-0.16 by stand height — the literature 0.10 is the "
            "median, but a quarter of tall cells exceed ~0.20-0.25."
        ),
    )
    canopy_gaps: Path | None = Field(
        default=None,
        description=(
            "Raster on the DTM grid, 1 where a cell holds a canopy OPENING a climb out "
            "of (or descent back into) the trunk layer can use — tools/build_finnish_ao.py "
            "derives it from the 1 m canopy model. Without it, open ground next to a "
            "transition is the only opening assumed."
        ),
    )
    stem_map: Path | None = Field(
        default=None,
        description=(
            "Individual stem positions as a `*_trunks.csv` manifest (east_m, "
            "north_m, base_z_m, radius_m, height_m -- HOME-RELATIVE, as written by "
            "`droneplanner gz-world --forest-dsm`). Supplying it turns on the "
            "stem-routing refinement (TEST-PLAN N4): the planner re-lays the line "
            "between the mapped trunks at 0.5 m resolution instead of leaving them "
            "to reactive avoidance, which is dead in flight.\n\n"
            "WARNING: every manifest that exists is SYNTHETIC and is generated in a "
            "band around the route planned WITHOUT it. No real stem map has ever "
            "been ingested, and ALS stem positions carry 1-2 m of error against a "
            "2.4 m clearance budget. This input demonstrates the planner can use a "
            "stem map; it does not demonstrate that one can be built."
        ),
    )
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
    stem: StemPolicy = StemPolicy()
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
    covered_radius_m: float | None = Field(
        default=None,
        gt=0,
        description=(
            "Concealment is required only within this distance of the start (user, "
            "2026-09-14: 'realistically only the first part has to be flown under "
            "cover'). Inside it the drone flies below the canopy where it can and "
            "drone exposure is costed; beyond it the drone flies over the canopy or "
            "in the open, with no drone-exposure or cover term, and no stem-clutter "
            "speed limit because it is above the stems. The cable's survivability "
            "term applies everywhere — the fiber stays on the ground after the "
            "aircraft has gone. None = the whole route (the pre-2026-09-23 "
            "behaviour). Radial from the start rather than along the route so the "
            "zone is a property of the ground, fixed before any route exists."
        ),
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
