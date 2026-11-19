"""How fast can the flight stack go here? Stem density from the canopy model.

WHY THIS EXISTS (TEST-PLAN N3)
The cost surface had no obstacle-density term of any kind. ``fields/snag.py`` is
land-cover-driven and returns all zeros without land cover, which the demo AO has
none of, so nothing in the planner knew that one stand is harder to fly through
than another. That was tolerable while the route was scored on concealment and
length. TEST-PLAN B2 and N1 made it the central question:

* **B2** measured that reactive avoidance is coherent only while
  ``mean stem spacing / required reaction distance`` stays above ~1. Required
  distance is linear in speed, spacing is not, so **stem spacing sets a local
  speed ceiling** (5.4 m/s at 50 stems/ha, 3.5 at 107, 1.6-2.5 at the 400-1000 of
  real managed forest).
* **N1** measured the other side: on a 30 min airframe the 5.3 km demo route
  needs at least **4.00 m/s** to get home at all.

So speed is squeezed from both ends, and the only lever that keeps the
below-canopy concealment posture is to route where the stems are *thinner*. That
needs a spatially varying density estimate, and this module derives one from data
the planner already has — the canopy height model, ``DSM - DTM`` — exactly the
trick that produced the below-canopy posture itself.

THE RELATION, AND HOW MUCH TO TRUST IT
Stand density falls as stands grow: competition kills stems, so a tall stand has
fewer, bigger trees than a short one of the same footprint. That is the
self-thinning rule (Reineke: ``N ~ D^-1.605``), combined with height-diameter
allometry (``D ~ H^1.2..1.4``), giving roughly ``N ~ H^-1.5..-2.1``.

It is implemented as a **power law between two stated anchors** rather than a
fitted curve, so the assumption is visible and replaceable:
:data:`REFERENCE_HEIGHT_M` / :data:`REFERENCE_STEMS_PER_HA` and
:data:`THINNING_EXPONENT`.

⚠️ **This is forestry literature, not a measurement of any real AO.** No stem
inventory has ever been ingested by this program, and the sim forest
(``io/forest_models.py``) places stems at a *uniform operator-set* density, so it
cannot validate a spatially varying one either. Treat the absolute stems/ha as
order-of-magnitude and the *ordering* — taller stands are sparser — as the part
that carries weight. Species, site index, and above all whether the stand has been
thinned move the constant by more than the height term does.

⚠️ **Mean spacing is the right predictor only because B2 measured it that way.** A
reactive layer needs *a* gap, not the average gap, and a Poisson field has gaps
wider than its mean. B2's strike counts nonetheless tracked mean spacing over two
densities and four speeds, so the mean is used here and the caveat is recorded.

WHAT THIS BUYS, MEASURED
On the demo AO it buys almost nothing, and that is the finding rather than a bug:
its 12-22 m stands map to ~640-1460 stems/ha (2026-09-24 Askola fit; ~520-1290 on the
literature anchor this was written with), whereas
clearing the 4.00 m/s endurance floor needs **5.21 m / 92 stems/ha**. The whole
available range sits an order of magnitude inside the infeasible region, so no
amount of routing between stands recovers the mission. Proving the term *works*
therefore needs a stand structure the demo does not contain, which is what the
unit tests supply.

FINNISH MANAGED FOREST ("talousmetsä") — CHECKED 2026-09-24
Most Finnish forest is managed for wood: Luke's 2025 statistics put **13 %** of
forest and poorly productive land under protection (6 % in the south, 20 % in the
north), so the rest is talousmetsä, thinned to published targets. Those targets
were the best available prior for stems/ha; the anchors above are now FITTED to
real stand data (Askola), and the Askola medians are in the right-hand column:

====================================  =========  ==============  =============
development class (VMI11 share of      height      stems/ha         Askola median
production forest land)                              (Tapio)         (Metsäkeskus)
====================================  =========  ==============  =============
seedling stand, taimikko (19 %)        < 7 m      2000+ small      open ground (*)
young thinning stand (36 %)            7-16 m     1500-2500 before, 1671 @ 13 m
                                                  700-1200 after
                                                  first thinning
advanced thinning stand (30 %)         16-22 m    ~500-900         706 @ 22 m
mature stand, uudistuskypsä (13 %)     20-28 m    ~400-700         494 @ 26 m
====================================  =========  ==============  =============

(*) below ``AltitudePolicy.min_canopy_m`` the planner flies over, not under.
Sources: Luke, *Suomen metsät 2009-2013* (LuoBio 59/2017), §4.2.1 class shares;
Tapio, *Metsänhoidon suositukset*, first-thinning targets by species and site
(pine 900-1200, spruce 700-1200, silver birch 700-800 stems/ha after thinning at
12-17 m dominant height); later-thinning and mature figures are the ranges those
targets thin down to, not a table value.

🔑 **What it means for the aircraft.** The stands tall enough to fly *under* —
advanced thinning and mature, ~43 % of production forest land — sit at roughly
**400-900 stems/ha**: inside the onboard planner's envelope since 2026-09-24
(:data:`ONBOARD_PLANNING_ENVELOPE`, 2.0-1.25 m/s across that range, SITL-anchored),
though flown in SITL only at 200 and 400. Young thinning stands are denser
still and their live crowns reach down into the 5-10 m band the aircraft would fly
in; a trunk-only stem count flatters them. Thinned talousmetsä is *easier* than an
unmanaged stand of the same height (fewer stems, less understory after pre-harvest
clearing), which is the one favourable fact here.
"""

from __future__ import annotations

import numpy as np

# --- The self-thinning anchors. Change these, not the formula. ---------------

REFERENCE_HEIGHT_M = 20.0
"""Canopy height the reference density is quoted at."""

REFERENCE_STEMS_PER_HA = 730.0
"""Stems/ha of a managed stand at :data:`REFERENCE_HEIGHT_M`.

MEASURED 2026-09-24 on real Finnish stand data: the Metsäkeskus Metsävarakuviot
download for Askola (southern Finland, CC BY 4.0), present-day dominant storey,
4 900+ stands with dominant height 12-32 m, area-weighted log-log fit:
**N = 43 026 · H^-1.36 → 727 stems/ha at 20 m**. The literature value it replaces
(600) under-predicted the measured median by 29 % (IQR of the ratio 0.98-1.66).
Medians by height: 1308 @ 13 m, 1015 @ 16 m, 723 @ 20 m, 637 @ 22 m, 574 @ 26 m.
⚠️ One municipality, thinned talousmetsä; the ±30 % stem-count uncertainty of the
source (``io/stand_data.py``) applies. Where stand data exists, use it directly
(``layers.stand_data``) — this relation is the fallback.
"""

THINNING_EXPONENT = 1.36
"""``N ~ H^-exponent``, fitted on the same Askola stands (see above). Reineke plus
height-diameter allometry gives 1.5-2.1 for unmanaged self-thinning; thinned
managed stands come out flatter, because thinning — not competition — sets their
density."""

MIN_MODELLED_HEIGHT_M = 2.0
"""Below this a "stand" is scrub, not stems the model describes."""

MAX_STEMS_PER_HA = 5000.0
"""Cap on the extrapolation into very short canopy, where the power law blows up."""

# --- Geometry shared with droneflight's maneuver model -----------------------

POISSON_SPACING_COEFF = 50.0
"""``mean nearest-neighbour spacing = COEFF / sqrt(stems_per_ha)``.

The Poisson nearest-neighbour result (``0.5 / sqrt(density per m^2)``, converted
from hectares). Checked against the real B2 trunk manifests, which are generated
by an independent path: 50 stems/ha predicts 7.07 m against a measured 7.0, and
107 stems/ha predicts 4.83 m against a measured 4.6 (the manifest clusters
slightly, so it runs a little tighter than Poisson).
"""

REACTION_TIME_S = 1.3025
"""Seconds of reaction the airframe needs per metre-per-second of speed.

``required_react_distance_m = speed * REACTION_TIME_S`` at the B2 lateral
clearance of 1.10 m (airframe half-width + margin + trunk radius), under PX4's
``MPC_ACC_HOR`` 3.0 / ``MPC_JERK_AUTO`` 4.0. Taken from
``droneflight.maneuver.required_react_distance_m``, which is the authority:
5.21 m at 4.0 m/s, 7.82 m at 6.0 m/s. It is quoted rather than imported because
the two projects are decoupled at the ``.plan`` boundary — the number is pinned by
``test_stem_density.py`` against those measured values so it cannot drift silently.
"""


def stem_density_per_ha(chm: np.ndarray, *, min_canopy_m: float = 2.0) -> np.ndarray:
    """Stems per hectare implied by canopy height (self-thinning power law).

    Cells with canopy below ``min_canopy_m`` are open ground and get 0.0 — no
    stems, hence no speed limit from this term.
    """
    height = np.asarray(chm, dtype=np.float32)
    treed = height >= max(min_canopy_m, MIN_MODELLED_HEIGHT_M)
    safe = np.maximum(height, MIN_MODELLED_HEIGHT_M)
    density = REFERENCE_STEMS_PER_HA * (safe / REFERENCE_HEIGHT_M) ** (-THINNING_EXPONENT)
    density = np.clip(density, 0.0, MAX_STEMS_PER_HA)
    return np.where(treed, density, 0.0).astype(np.float32)


def mean_stem_spacing_m(stems_per_ha: np.ndarray) -> np.ndarray:
    """Mean nearest-neighbour spacing — the gap avoidance actually has to thread.

    Open cells (zero density) get ``inf``: unobstructed, no spacing limit.
    """
    density = np.asarray(stems_per_ha, dtype=np.float32)
    with np.errstate(divide="ignore"):
        spacing = POISSON_SPACING_COEFF / np.sqrt(density)
    return np.where(density > 0.0, spacing, np.inf).astype(np.float32)


def max_reactive_speed_ms(
    spacing_m: np.ndarray, *, cruise_speed_ms: float, reaction_time_s: float = REACTION_TIME_S
) -> np.ndarray:
    """Fastest speed at which reactive avoidance stays coherent, capped at cruise.

    Inverts B2's finding directly: the dodge needs ``speed * reaction_time`` metres
    of warning, and the stems only give ``spacing_m``.
    """
    if cruise_speed_ms <= 0.0:
        raise ValueError("cruise_speed_ms must be positive")
    if reaction_time_s <= 0.0:
        raise ValueError("reaction_time_s must be positive")
    spacing = np.asarray(spacing_m, dtype=np.float32)
    # Typed local: numpy 2.4's stubs (CI's Python 3.11 job) call this chain Any.
    capped: np.ndarray = np.minimum(spacing / reaction_time_s, cruise_speed_ms).astype(np.float32)
    return capped


HARNESS_ENVELOPE: tuple[tuple[float, float], ...] = (
    (107.0, 6.0),
    (200.0, 5.0),
    (400.0, 4.0),
    (500.0, 3.0),
    (600.0, 3.0),
    (800.0, 2.5),
    (1000.0, 1.5),
    (1200.0, 0.75),
)
"""What the offline harness flies clean — an UPPER BOUND, not a planning number.
See :data:`ONBOARD_PLANNING_ENVELOPE` for why the planner does not use it."""

ONBOARD_PLANNING_ENVELOPE: tuple[tuple[float, float], ...] = (
    (107.0, 3.6),
    (200.0, 3.0),
    (400.0, 2.0),
    (500.0, 1.5),
    (600.0, 1.5),
    (800.0, 1.5),
    (1000.0, 1.0),
    (1200.0, 0.5),
)
"""``(stems/ha, speed m/s)`` the planner prices below-canopy flight at: the
harness envelope **anchored to SITL**, which is the arbiter.

DECISION 2026-09-24. GPS-denied SITL on the final flight stack (planner fixes
incl. "steer, don't brake", VTRN fixes bled over 30 m), swept over four EV noise
seed sets (0/1, 2/3, 4/5, 6/7) — one run is one noise draw:

==============  ============================  =============================
stand / speed   clean seed sets (min air)     contact
==============  ============================  =============================
200/ha 3 m/s    **4/4** (+0.18 m)             —
200/ha 4 m/s    4/5                           **0/1**
400/ha 2 m/s    **4/4** (+0.28 m)             —
400/ha 3 m/s    —                             **4/5 (2 contacts)**
400/ha 3 m/s†   3/4 (+0.03 m on 4/5)          **0/1 (graze -0.04 m)**
600/ha 1.5 m/s  **4/4** (⚠️ +0.03 m on 6/7)    —
600/ha 2 m/s    4/4 (+0.12 m)                 —
800/ha 1.0 m/s‡ **4/4** (+0.66 m)             —
800/ha 1.5 m/s‡ **4/4** (+0.29 m)             — (one earlier run at RTF 0.2 voided)
~1000/ha 0.5§   **4/4** (+0.33 m)             — (0/3 before memory clearing)
~1000/ha 1.0§   **4/4** (+0.27 m)             — (`askolam230`, RTF 1.0, 0 escapes)
1200/ha 1.0¶    4/4 strike-free (⚠️ +0.07 m)   — (**1 escape** in 4)
1500/ha 0.5¶    **4/4** (+0.52 m)             — (1 escape in 4, a FALSE trigger)
==============  ============================  =============================

¶ 2026-09-26, `forest_mvp1200n` / `forest_mvp1500n` (MVP route, stand over the first
~550 m, ±15 m band), memory clearing + escape climb + sensing at target height on.
1200/ha air +0.33 / +0.50 / +0.48 / **+0.07 m** — the thin one (seed set 6/7) is in
ordinary flight at 11.4 m AGL and 0.95 m/s, approaching the dead end that then
triggered its escape. The pre-registered bar was 4/4 with > 0.15 m of air, so **the
1200/ha anchor is not raised to 1.0**. 1500/ha at 0.5: air +0.52 / +0.78 / +0.58 / +0.52 m
(seed set 6/7 re-flown after a first run at RTF 0.11 was voided) — **4/4**, so **1200/ha
→ 0.5** (was 0.4, discounted from the harness): a *denser* stand flew clean at that speed
with ≥ 0.52 m of air on every seed set. **No 1500 anchor is added**, deliberately:
:func:`beyond_envelope` is "denser than the last anchor", so a 1500 anchor would move the
over-canopy fallback from 1200 to 1500/ha and plan *under* stands where genuine dead ends
occur — what the user asked the fallback to prevent (2026-09-25).
🔑 **Passability, not speed, is what breaks first past ~1000/ha — but less often than
first read.** One traverse at 1200/ha (h1g) hit a real dead end: no path for 20 s, then
the escape climb. The 1500/ha escape (h5e) was a **false trigger** — 0 blocked steps, a
12.7 m detour at 0.5 m/s tripping "20 s without progress" (droneflight now waits for a
30 m detour when the planner has a path). So genuine dead ends above 1000/ha: **1 in 8
traverses**; at ~1000/ha, 0 in 4. Revisit the 1500 anchor after a re-fly with the fixed
trigger — until then the fallback stays at 1200.

‡ 2026-09-25, `forest_mvp800n` (1100 trunks, 800/ha over the first 900 m, ±15 m
band so the VM holds real time), memory clearing on: +1.11 to +1.35 m surface.
The 800/ha anchor (1.25) is commanded as 1.0, so this is what the planner sends.
† 2026-09-25, with memory clearing (droneflight PR #5): still not clean — the anchor
stays 2.0.
§ 2026-09-25, Askola's MEASURED zone (Metsäkeskus stands, ~1000-1200 stems/ha,
1.5 m median spacing, real MML terrain): the 1000/ha anchor below (0.75) is
commanded as 0.5 (``model.plan.commanded_speed_ms``), which is what flew. The
onboard stack stalled 3/3 there until it cleared memory the scan sees through
(droneflight PR #5); with it 4/4 reached the goal, 21.2-21.5 min against 20.9
planned.

(Air = trunk-surface clearance from ``b2-score`` minus the 0.45 m airframe half-width.)
So **200/ha → 3.0, 400/ha → 2.0, 600/ha → 1.5, 800/ha → 1.5, ~1000/ha → 1.0 and
1200/ha → 0.5 are measured** (the last bounded by 1500/ha at 0.5, 2026-09-26;
2026-09-25: 800/ha raised 1.25 → 1.5, ~1000/ha 0.75 → 1.0).
600/ha also flew 2 m/s strike-free, but at 600/ha the margin is thin at *both*
speeds — a 1.9 m median gap threaded by a 0.9 m airframe — so it is not raised on
that evidence.
The remaining anchors are the harness discounted by the SITL/harness ratio at the
nearest validated density (3/5 = 0.6 at <= 200, 2/4 = 0.5 at >= 400).
The harness passes 4-5 m/s at 200/ha: it is necessary, not sufficient.
Overturn an anchor with a four-seed SITL sweep (``~/sweep.sh`` on the VM), never
one run. Before "steer, don't brake" the same
3 m/s grazed on seed set 4/5 three times out of three.

The harness measurement behind it (:data:`HARNESS_ENVELOPE`):

MEASURED 2026-09-24 with ``droneflight/tools/envelope_sweep.py``: the harness
(`reactive.fly_with_local_planner`) flying the measured PX4 offboard response
(0.3 s dead time + 0.4 s lag, drawn per seed from inside the fit), 10 Hz control,
oracle depth sensor, 90 deg FOV, 30 m range, truth navigation, 1200 m of Poisson
forest (trunk radii 0.08-0.25 m), seeds 10-13. An anchor is the fastest speed at
which **all four seeds reach the goal with 0 strikes and > 0.15 m of air**:

========  =====  ==============  ============================================
stems/ha  m/s    min air gap     next step up / notes
========  =====  ==============  ============================================
107       6.0    +0.55 m         airframe cruise (+0.81 m over the 5.2 km manifest)
200       5.0    +0.41 m         not tried above 5
400       4.0    +0.28 m
500       3.0    +0.27 m
600       3.0    +0.34 m
800       2.5    +0.36 m
1000      1.5    +0.26 m         ⚠️ a seed STALLED at 1.0 (no strike): stalls exist here
1200      0.75   +0.38 m         not tried faster; 1.0 and 1.5 failed before the fixes
========  =====  ==============  ============================================

The previous envelope (2026-09-23: 107 -> 6, 200 -> 3, 400 -> 2, 800 fails) was
measured before four planner defects were fixed — see ``droneflight/CLAUDE.md``,
"the loop found four more defects": prediction from the wrong dynamics model,
a speed-blind pursuit lookahead that deadlocked, and no momentum limit. The
same fixes took two GPS-denied SITL contacts to a clean flight (fix5).

⚠️ The harness is necessary, not sufficient: it did not reproduce the SITL
contacts the fixes came from, and it passes 5 m/s at 200/ha where SITL fails.
Past 1200 the extrapolation (speed inversely proportional to density) rests on
nothing, and :func:`beyond_envelope` says so.

Replaces :func:`max_reactive_speed_ms` as the planner's speed cap: the reactive
policy it modelled is dead in flight (N2, 34-41 strikes), so capping speed by
*its* coherence limit priced the mission on a flight stack nobody will fly.
"""


def max_planned_speed_ms(stems_per_ha: np.ndarray, *, cruise_speed_ms: float) -> np.ndarray:
    """Fastest demonstrated-clean speed for onboard planning, capped at cruise.

    Linear in density between the :data:`ONBOARD_PLANNING_ENVELOPE` anchors, and
    cruise at or below the first anchor (and on open ground). **Past the last
    anchor it is an extrapolation, not a measurement**: speed falls in inverse
    proportion to density (floored at 0.3 m/s). That keeps the cost surface
    preferring the sparser of two unflyable stands and keeps the flight time
    finite; :func:`beyond_envelope` marks those cells so nobody mistakes the
    number for a demonstrated one.
    """
    density = np.asarray(stems_per_ha, dtype=np.float32)
    # Open ground (0 stems/ha) is cruise, and a stand sparser than the first
    # anchor interpolates up to it — not clamped to the first anchor's speed,
    # which slowed every open cell once that anchor fell below cruise.
    xs = np.array([0.0] + [d for d, _ in ONBOARD_PLANNING_ENVELOPE], dtype=np.float32)
    ys = np.array([cruise_speed_ms] + [v for _, v in ONBOARD_PLANNING_ENVELOPE], dtype=np.float32)
    last_d, last_v = ONBOARD_PLANNING_ENVELOPE[-1]
    inside = np.interp(density, xs, ys)
    with np.errstate(divide="ignore"):
        beyond = np.maximum(last_v * last_d / np.maximum(density, 1e-6), 0.3)
    speed: np.ndarray = np.where(density > last_d, beyond, inside).astype(np.float32)
    capped: np.ndarray = np.minimum(speed, np.float32(cruise_speed_ms)).astype(np.float32)
    return capped


def beyond_envelope(stems_per_ha: np.ndarray) -> np.ndarray:
    """Cells denser than the onboard planner has been shown to fly."""
    limit = ONBOARD_PLANNING_ENVELOPE[-1][0]
    out: np.ndarray = np.asarray(stems_per_ha, dtype=np.float32) > limit
    return out


def traversal_time_penalty(max_speed_ms: np.ndarray, *, cruise_speed_ms: float) -> np.ndarray:
    """Extra flight time per metre, relative to flying the same metre at cruise.

    This is the term the cost surface wants, because **time is the currency the
    endurance budget spends** (N1). A cell that caps the aircraft at half cruise
    costs twice the seconds per metre, so it is penalised 1.0; a cell that allows
    full cruise is free. Bounded so a near-zero-speed cell cannot dominate the
    entire objective with an unbounded number.
    """
    if cruise_speed_ms <= 0.0:
        raise ValueError("cruise_speed_ms must be positive")
    speed = np.maximum(np.asarray(max_speed_ms, dtype=np.float32), 1e-3)
    penalty = cruise_speed_ms / speed - 1.0
    return np.clip(penalty, 0.0, 10.0).astype(np.float32)


def build_stem_clutter(
    chm: np.ndarray,
    *,
    cruise_speed_ms: float,
    min_canopy_m: float = 2.0,
    among_stems: np.ndarray | None = None,
    density_override_per_ha: float | None = None,
    measured_per_ha: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """All four rasters at once — density, spacing, speed cap, time penalty.

    ``among_stems`` marks the cells the drone actually flies *below the canopy*.
    Elsewhere it is above the crowns and the stems cannot limit its speed, so the
    cap is cruise there. Before 2026-09-23 the cap applied to every treed cell,
    including ones flown over — inert on the demo AO, where the treed and the
    below-canopy cells coincide, but wrong as soon as a route goes over a stand.
    """
    density = stem_density_per_ha(chm, min_canopy_m=min_canopy_m)
    measured = np.zeros(density.shape, dtype=bool)
    if measured_per_ha is not None and density_override_per_ha is None:
        # Stand data (io/stand_data.py) replaces the inference where a stand
        # covers a treed cell. The canopy model still decides treed vs open: it is
        # the newer of the two, and a stand felled since its inventory is open.
        measured = (density > 0.0) & np.isfinite(measured_per_ha) & (measured_per_ha > 0.0)
        density = np.where(measured, measured_per_ha, density).astype(np.float32)
    if density_override_per_ha is not None:
        # An operator's stated density replaces the canopy-height inference on
        # every treed cell (open cells stay open).
        density = np.where(density > 0.0, np.float32(density_override_per_ha), 0.0).astype(
            np.float32
        )
    spacing = mean_stem_spacing_m(density)
    reactive = max_reactive_speed_ms(spacing, cruise_speed_ms=cruise_speed_ms)
    speed = max_planned_speed_ms(density, cruise_speed_ms=cruise_speed_ms)
    outside = beyond_envelope(density)
    if among_stems is not None:
        cruise = np.float32(cruise_speed_ms)
        reactive = np.where(among_stems, reactive, cruise).astype(np.float32)
        speed = np.where(among_stems, speed, cruise).astype(np.float32)
        outside = outside & among_stems
    return {
        "stem_density_per_ha": density,
        "stem_spacing_m": np.where(np.isfinite(spacing), spacing, 0.0).astype(np.float32),
        "max_reactive_speed_ms": reactive,
        "max_speed_ms": speed,
        "beyond_envelope": outside.astype(np.float32),
        "stem_density_measured": measured.astype(np.float32),
        "clutter_time_penalty": traversal_time_penalty(speed, cruise_speed_ms=cruise_speed_ms),
    }
