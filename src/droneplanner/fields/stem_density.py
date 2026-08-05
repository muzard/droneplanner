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
its 12-22 m stands map to ~520-1290 stems/ha, i.e. **1.4-2.2 m spacing**, whereas
clearing the 4.00 m/s endurance floor needs **5.21 m / 92 stems/ha**. The whole
available range sits an order of magnitude inside the infeasible region, so no
amount of routing between stands recovers the mission. Proving the term *works*
therefore needs a stand structure the demo does not contain, which is what the
unit tests supply.
"""

from __future__ import annotations

import numpy as np

# --- The self-thinning anchors. Change these, not the formula. ---------------

REFERENCE_HEIGHT_M = 20.0
"""Canopy height the reference density is quoted at."""

REFERENCE_STEMS_PER_HA = 600.0
"""Stems/ha of a mature managed stand at :data:`REFERENCE_HEIGHT_M`.

Mid-range for managed temperate/boreal forest. TEST-PLAN B2 independently calls
400 stems/ha "the *low* end of real managed forest", which brackets this from
below.
"""

THINNING_EXPONENT = 1.5
"""``N ~ H^-exponent``. Reineke plus height-diameter allometry gives 1.5-2.1;
the low end is used because it is the conservative choice *for this planner* —
it predicts less density variation between stands, so it claims less benefit from
routing between them."""

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
    return np.minimum(spacing / reaction_time_s, cruise_speed_ms).astype(np.float32)


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
    chm: np.ndarray, *, cruise_speed_ms: float, min_canopy_m: float = 2.0
) -> dict[str, np.ndarray]:
    """All four rasters at once — density, spacing, speed cap, time penalty."""
    density = stem_density_per_ha(chm, min_canopy_m=min_canopy_m)
    spacing = mean_stem_spacing_m(density)
    speed = max_reactive_speed_ms(spacing, cruise_speed_ms=cruise_speed_ms)
    return {
        "stem_density_per_ha": density,
        "stem_spacing_m": np.where(np.isfinite(spacing), spacing, 0.0).astype(np.float32),
        "max_reactive_speed_ms": speed,
        "clutter_time_penalty": traversal_time_penalty(speed, cruise_speed_ms=cruise_speed_ms),
    }
