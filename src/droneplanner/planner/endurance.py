"""Can the airframe actually fly this route before the battery runs out?

WHY THIS EXISTS (TEST-PLAN N1)
Nothing in either project had an endurance, energy or time model. That was
survivable while every plan assumed 6.0 m/s cruise, which flies the 5.3 km demo
route in ~16 minutes. It stopped being survivable when TEST-PLAN B2 measured a
**speed ceiling imposed by the forest**: reactive avoidance collides unless
``stem spacing / required reaction distance`` stays above ~1, which caps cruise at
~3.5 m/s at 107 stems/ha and 1.6-2.5 m/s at the 400-1000 stems/ha of real managed
forest. Route length is fixed by the mission, so the whole cost of that cap lands
on flight time — and a multirotor carrying 5 km of fiber does not have unlimited
time.

So this module answers one question the planner previously could not: **given the
route it just built and the speed it intends to fly, does the mission close?**

THE MODEL, AND EXACTLY HOW MUCH OF IT IS MEASURED
Flight time is not ``length / cruise``. Scored against the one route that has been
flown end to end — B1, 282 waypoints, 5299 m, commanded 6.0 m/s, COMPLETED 282/282
(``sim-logs/canopy/21_38_21.ulg``, AUTO_MISSION phase) — the naive estimate is
**7.6 % low**: 883 s against a flown **956.0 s**. That deficit decomposes cleanly,
and the decomposition is what this module implements:

* **Cruise** — ``leg length / speed``, the dominant term.
* **Vertical rate** — a leg whose altitude step needs longer than its horizontal
  run takes the longer of the two. Inert on the demo route (its steps are ~1 m
  over 25 m legs) but not on routes with big drops, which is precisely the shape
  ``terrain_profile.py`` exists to police.
* **Per-waypoint overhead — 0.161 s, MEASURED.** Excluding both route ends, B1
  covered 5148 m in 903.5 s against an ideal 858.0 s: 45.5 s spread over 282
  waypoints. PX4 decelerates into every acceptance radius and re-accelerates out,
  and at 25 m spacing it does that 282 times.
* **Terminal overhead — 19.1 s, MEASURED.** B1's first 100 m took 32.1 s (3.12 m/s
  mean, still climbing) and its last 100 m took 20.4 s, against 16.7 s each at
  cruise. Dominated by the initial climb, so it is modelled as a fixed allowance
  rather than something that scales with cruise.

⚠️ **This is a CALIBRATION, not a prediction, and the distinction matters.** Both
overhead constants were fitted to the same 956.0 s they reproduce, so "the model
matches B1 to 0.9 %" is very nearly an identity. **One flight, one route, one
cruise speed (6.0 m/s).** Whether the per-waypoint overhead is a fixed time or a
fixed *fraction* is UNRESOLVED — exactly the open question
``droneflight/px4_profile.py`` carries about its corner prior, for the same reason
(everything was measured at one speed). The B2b speed sweep at 6.0 / 3.5 / 2.0 m/s
is the first independent test of it. Fortunately the answer is robust to that
uncertainty: the overhead is 6.8 % of flight time at 6.0 m/s and 2.4 % at 2.0 m/s,
so it cannot move the go/no-go conclusion either way.

WHAT IS DELIBERATELY *NOT* MODELLED
**Corner deceleration.** PX4 slows into corners on its own, steeply — 4.96 m/s at
a 20 deg bend, 0.46 m/s at 160 deg (``droneflight/px4_profile.py``, measured). It
is omitted here because on planner routes it never fires: the any-angle search
plus the kinematic fillet pass keep every vertex shallow, and the demo route's
**sharpest corner is 6.7 deg** across all 282 waypoints, where PX4's own cap is
still ~6.0 m/s. Importing that table to model an effect measured at zero would
duplicate a calibration across the repo boundary for nothing. :func:`assess_endurance`
counts corners sharp enough to matter and reports them, so a route that ever grows
real corners says so instead of being silently under-estimated.

ENERGY, AND WHY THE TIME BUDGET IS THE CONSERVATIVE ONE
``DroneSpec.endurance_s`` is *hover* endurance. For a multirotor, hovering is close
to the worst case for level flight in the 0-8 m/s band — induced power falls with
forward speed well before parasitic drag grows — so **charging level flight at the
hover rate is conservative**, and no unbenched aerodynamic coefficient is needed to
say so.

The one term that is *not* conservative is climbing, which costs more than hover.
It is charged through :attr:`EnduranceModel.climb_power_ratio` using the standard
momentum-theory result that a climb costs roughly ``v_climb / (2 * v_induced_hover)``
above hover. And it is charged against **measured** vertical travel, not commanded:
B1's plan commands 283.7 m of altitude change and the aircraft flew **~1030 m**
(stable from 200 Hz down to 1 Hz resampling, 0.9 vertical-velocity sign changes per
leg, 4.04 m mean peak-to-peak over 5 s windows). PX4 overshoots each small
terrain-following step and sawtooths around it, so a nap-of-the-earth route does
**3.6x** the vertical work its plan asks for. ⚠️ That factor is one flight at one
waypoint spacing, and it is almost certainly spacing-dependent — smaller commanded
steps mean proportionally larger overshoot.

All geometry is in the projected world CRS (meters).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from droneplanner.model.mission import Mission
from droneplanner.model.plan import EnduranceBudget

# --- Constants measured from B1 (sim-logs/canopy/21_38_21.ulg, MISSION phase) ---
# See the module docstring for the decomposition. Named rather than inlined so a
# future flight can move them in one place, and so the provenance travels with
# the value.

B1_WAYPOINT_OVERHEAD_S = 0.161
"""Time lost per waypoint to PX4's decelerate-in/accelerate-out (45.5 s / 282 wp)."""

B1_TERMINAL_OVERHEAD_S = 19.1
"""Fixed cost of the route's two ends: initial climb + accel, final decel."""

B1_VERTICAL_OVERSHOOT = 3.63
"""Flown vertical travel / commanded vertical travel (~1030 m vs 283.7 m)."""

DEFAULT_RESERVE_FRACTION = 0.20
"""Battery held back from the budget. Standard practice, not measured here."""

DEFAULT_INDUCED_VELOCITY_MS = 8.0
"""Hover induced velocity; sets how expensive climbing is relative to hovering.

Typical for a small multirotor at 5-15 kg/m^2 disc loading. Only affects the
*climb* surcharge, which is a few percent — it is not load-bearing.
"""

SHARP_CORNER_DEG = 20.0
"""Deflection at which PX4's own corner deceleration starts to bite (measured:
4.96 m/s at 20 deg against a 6.0 m/s command). Routes below this are unaffected."""

Point = tuple[float, float]


@dataclass(frozen=True)
class EnduranceModel:
    """Airframe time/energy parameters for the budget."""

    cruise_speed_ms: float
    climb_rate_ms: float
    descent_rate_ms: float
    endurance_s: float | None = None
    """Usable *hover* endurance with the fiber payload. ``None`` = never stated."""

    reserve_fraction: float = DEFAULT_RESERVE_FRACTION
    waypoint_overhead_s: float = B1_WAYPOINT_OVERHEAD_S
    terminal_overhead_s: float = B1_TERMINAL_OVERHEAD_S
    vertical_overshoot: float = B1_VERTICAL_OVERSHOOT
    induced_velocity_ms: float = DEFAULT_INDUCED_VELOCITY_MS

    def __post_init__(self) -> None:
        if self.cruise_speed_ms <= 0.0:
            raise ValueError("cruise_speed_ms must be positive")
        if self.climb_rate_ms <= 0.0 or self.descent_rate_ms <= 0.0:
            raise ValueError("climb/descent rates must be positive")
        if not 0.0 <= self.reserve_fraction < 1.0:
            raise ValueError("reserve_fraction must be in [0, 1)")
        if self.endurance_s is not None and self.endurance_s <= 0.0:
            raise ValueError("endurance_s must be positive when given")

    @property
    def usable_endurance_s(self) -> float | None:
        """Endurance after the reserve is held back — the budget a plan may spend."""
        if self.endurance_s is None:
            return None
        return self.endurance_s * (1.0 - self.reserve_fraction)

    def climb_power_ratio(self, climb_rate_ms: float) -> float:
        """Power to climb at ``climb_rate_ms``, as a multiple of hover power.

        Momentum theory: induced velocity in a climb is
        ``v_c/2 + sqrt((v_c/2)^2 + v_h^2)``, and power scales with it. Reduces to
        1.0 in a hover and grows roughly linearly at the climb rates a laying
        drone uses.
        """
        if climb_rate_ms <= 0.0:
            return 1.0
        half = climb_rate_ms / 2.0
        v_i = half + math.hypot(half, self.induced_velocity_ms)
        return v_i / self.induced_velocity_ms


@dataclass(frozen=True)
class EnduranceReport:
    """Whether the route fits the airframe's endurance, and where the time goes."""

    flight_time_s: float
    cruise_time_s: float
    """Time set by horizontal speed."""

    vertical_bound_time_s: float
    """Extra time on legs where the climb/descent rate, not cruise, is the limit."""

    waypoint_overhead_s: float
    terminal_overhead_s: float

    energy_time_s: float
    """Flight time plus the climb surcharge — what the battery actually pays."""

    energy_fraction: float
    """Fraction of *usable* endurance consumed, including the climb surcharge.
    ``inf`` when no endurance was stated."""

    required_endurance_s: float
    """Endurance this route demands, reserve included — what the airframe must have."""

    usable_endurance_s: float | None
    endurance_s: float | None
    min_sustainable_speed_ms: float
    """Slowest cruise that still fits the budget. ``0.0`` when no endurance given.

    Endurance floors cruise speed **from below**, which is what makes this the
    number to compare against B2's forest ceiling: obstacle avoidance caps speed
    from above (~3.5 m/s at 107 stems/ha, 1.6-2.5 at real density) while getting
    home caps it from below. **If the floor is above the ceiling there is no
    feasible speed at all** — and that is a statement about the concept of
    operations, not about tuning.
    """

    sharp_corners: int
    """Vertices deflecting more than ``SHARP_CORNER_DEG``, where PX4's own corner
    deceleration would add time this model does not charge for."""

    vertical_travel_m: float
    """Commanded sum of |altitude change|, before the overshoot factor."""

    effective_speed_ms: float = 0.0
    """Route length / cruise time — the speed the forest actually permits.

    Equals ``cruise_speed_ms`` on an unobstructed route; below it wherever stem
    spacing caps the flight stack (TEST-PLAN N3).
    """

    speed_capped_legs: int = 0
    """Legs the stems slowed below the airframe's cruise speed."""

    @property
    def stated(self) -> bool:
        """Was an endurance budget supplied at all?"""
        return self.endurance_s is not None

    @property
    def closes(self) -> bool:
        """Does the route fit inside the usable budget (reserve intact)?"""
        return self.stated and self.energy_fraction <= 1.0

    @property
    def exceeds_raw_endurance(self) -> bool:
        """Does it not even fit the *full* battery, reserve spent?

        The distinction from :attr:`closes` is operationally real: a route that
        eats into the reserve is a command decision, one that outruns the whole
        battery is arithmetic.
        """
        if self.endurance_s is None:
            return False
        return self.energy_time_s > self.endurance_s


def _deflection_deg(a: Point, b: Point, c: Point) -> float:
    ax, ay = b[0] - a[0], b[1] - a[1]
    bx, by = c[0] - b[0], c[1] - b[1]
    la, lb = math.hypot(ax, ay), math.hypot(bx, by)
    if la == 0.0 or lb == 0.0:
        return 0.0
    dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
    return math.degrees(math.acos(dot))


def assess_endurance(
    pts: list[Point],
    amsl: list[float],
    model: EnduranceModel,
    *,
    speed_limits_ms: list[float] | None = None,
) -> EnduranceReport:
    """Score a route's flight time and energy against the airframe's endurance.

    This is the *guarantee*, in the same shape as
    :func:`terrain_profile.assess_clearance`: it re-scores the finished route
    rather than trusting an assumption made while building it, and a breach is
    CRITICAL. ``amsl`` is the per-vertex absolute altitude, so the vertical term
    sees terrain-driven climbing — unlike the kinematic climb-rate check, which
    runs in AGL and is blind to it (SHORTFALLS 0b).

    ``speed_limits_ms`` is the per-vertex cap the local stem spacing imposes
    (TEST-PLAN N3, ``fields/stem_density.py``). Supplying it is what makes the
    budget honest about below-canopy flight: scoring a forest route at a cruise
    speed the trees forbid is the same class of error as scoring clearance under
    a descent profile PX4 does not fly. A leg is charged at the slower of its two
    endpoints, because the aircraft must already have slowed on arrival.
    """
    if len(pts) != len(amsl):
        raise ValueError("pts and amsl must be the same length")
    if speed_limits_ms is not None and len(speed_limits_ms) != len(pts):
        raise ValueError("speed_limits_ms must be the same length as pts")

    cruise_time = 0.0
    vertical_extra = 0.0
    climb_m = 0.0
    descend_m = 0.0
    distance_m = 0.0
    speed_capped_legs = 0

    for i in range(len(pts) - 1):
        length = math.dist(pts[i], pts[i + 1])
        distance_m += length
        dz = amsl[i + 1] - amsl[i]
        if dz >= 0.0:
            climb_m += dz
            rate = model.climb_rate_ms
        else:
            descend_m += -dz
            rate = model.descent_rate_ms

        speed = model.cruise_speed_ms
        if speed_limits_ms is not None:
            capped = min(speed_limits_ms[i], speed_limits_ms[i + 1])
            if capped < speed:
                speed = capped
                speed_capped_legs += 1
        speed = max(speed, 0.01)

        t_horizontal = length / speed
        t_vertical = abs(dz) / rate
        cruise_time += t_horizontal
        vertical_extra += max(0.0, t_vertical - t_horizontal)

    waypoint_overhead = model.waypoint_overhead_s * len(pts)
    terminal = model.terminal_overhead_s if len(pts) > 1 else 0.0
    flight_time = cruise_time + vertical_extra + waypoint_overhead + terminal

    sharp = sum(
        1
        for i in range(1, len(pts) - 1)
        if _deflection_deg(pts[i - 1], pts[i], pts[i + 1]) > SHARP_CORNER_DEG
    )

    # --- Energy ---------------------------------------------------------------
    # Level flight is charged at the hover rate (conservative for a multirotor,
    # see the module docstring). Climbing is charged its momentum-theory
    # surcharge, against the vertical travel PX4 actually flies rather than the
    # much smaller amount the plan commands.
    flown_climb_m = climb_m * model.vertical_overshoot
    climb_time = flown_climb_m / model.climb_rate_ms if model.climb_rate_ms > 0 else 0.0
    climb_time = min(climb_time, flight_time)
    surcharge = model.climb_power_ratio(model.climb_rate_ms) - 1.0
    energy_time = flight_time + climb_time * surcharge

    usable = model.usable_endurance_s
    if usable is None or usable <= 0.0:
        energy_fraction = math.inf
        min_sustainable = 0.0
    else:
        energy_fraction = energy_time / usable
        # Slowest cruise that still fits. Everything except the cruise term is
        # speed-independent to within the caveat in the module docstring, so
        # invert on the cruise term alone and keep the same energy surcharge.
        fixed = flight_time - cruise_time
        overhead_ratio = energy_time / flight_time if flight_time > 0.0 else 1.0
        budget_for_cruise = usable / overhead_ratio - fixed
        min_sustainable = distance_m / budget_for_cruise if budget_for_cruise > 0.0 else math.inf

    required = energy_time / (1.0 - model.reserve_fraction)

    return EnduranceReport(
        flight_time_s=flight_time,
        cruise_time_s=cruise_time,
        vertical_bound_time_s=vertical_extra,
        waypoint_overhead_s=waypoint_overhead,
        terminal_overhead_s=terminal,
        energy_time_s=energy_time,
        energy_fraction=energy_fraction,
        required_endurance_s=required,
        usable_endurance_s=usable,
        endurance_s=model.endurance_s,
        min_sustainable_speed_ms=min_sustainable,
        sharp_corners=sharp,
        vertical_travel_m=climb_m + descend_m,
        effective_speed_ms=(distance_m / cruise_time if cruise_time > 0.0 else 0.0),
        speed_capped_legs=speed_capped_legs,
    )


def model_from_mission(mission: Mission) -> EnduranceModel:
    """The airframe's budget parameters, straight off the mission contract."""
    return EnduranceModel(
        cruise_speed_ms=mission.drone.cruise_speed_ms,
        climb_rate_ms=mission.drone.climb_rate_max_ms,
        descent_rate_ms=mission.drone.descent_rate_max_ms,
        endurance_s=mission.drone.endurance_s,
        reserve_fraction=mission.drone.energy_reserve_fraction,
    )


def route_budget(
    pts: list[Point],
    amsl: list[float],
    mission: Mission,
    *,
    speed_limits_ms: list[float] | None = None,
) -> EnduranceBudget:
    """Score a route and return the serializable budget the ``Plan`` carries.

    One helper for both planners: the endurance question needs only length,
    waypoint count, altitude and speed, so the M0 stub can answer it exactly as
    well as the terrain-aware corridor planner. Leaving it unscored on the stub
    path would put a silent ``null`` in the report where a real number belongs.
    """
    report = assess_endurance(
        pts, amsl, model_from_mission(mission), speed_limits_ms=speed_limits_ms
    )
    return EnduranceBudget(
        flight_time_s=report.flight_time_s,
        cruise_time_s=report.cruise_time_s,
        vertical_bound_time_s=report.vertical_bound_time_s,
        waypoint_overhead_s=report.waypoint_overhead_s,
        terminal_overhead_s=report.terminal_overhead_s,
        energy_time_s=report.energy_time_s,
        energy_fraction=(report.energy_fraction if math.isfinite(report.energy_fraction) else 0.0),
        required_endurance_s=report.required_endurance_s,
        endurance_s=report.endurance_s,
        usable_endurance_s=report.usable_endurance_s,
        min_sustainable_speed_ms=(
            report.min_sustainable_speed_ms
            if math.isfinite(report.min_sustainable_speed_ms)
            else 0.0
        ),
        sharp_corners=report.sharp_corners,
        vertical_travel_m=report.vertical_travel_m,
        effective_speed_ms=report.effective_speed_ms,
        speed_capped_legs=report.speed_capped_legs,
    )
