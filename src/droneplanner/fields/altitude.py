"""Where the drone flies vertically — the band the route and the viewshed share.

WHY THIS EXISTS (SHORTFALLS item 1c — "the concealment was fake")
The planner used to fly a fixed 30 m above **bare terrain**. In a 22 m stand that
is 8 m over the crowns: the cost surface rewarded forest as cover while the
aircraft flew above it, silhouetted against sky. Concealment came from the map,
not from the flight. Preflight-Design is explicit that the posture is
*canopy-top skimming / upper-trunk-space flight*, and the data to do it was
already ingested — the canopy height model is just ``DSM - DTM``.

So altitude is now **surface-relative, and as low as the constraints allow**:

* **target** — ``Mission.cruise_agl_m``, a height to be clamped, not held;
* **floor** — ``AltitudePolicy.min_agl_m``: understory, rotor wash, DTM error;
* **ceiling** — ``canopy_top - canopy_clearance_m`` wherever the canopy is tall
  enough to be worth flying under, so the drone stays *inside* the stand.

Three bands come out of it, and they mean different things to the risk report:
``BELOW_CANOPY`` (concealed, and betting on onboard obstacle avoidance),
``CANOPY_TOP`` (skimming a stand too short to enter), and ``OPEN`` (no cover —
concealment here can only come from terrain masking).

⚠️ **This module decides where the drone *is*, not whether it can get there.** A
DSM sees the canopy top and nothing beneath it — no trunks, no branches, no
understory. Committing a leg below the canopy is therefore a bet on the flight
stack's perception (camera/lidar avoidance), which is why the report raises a
finding for it rather than treating a below-canopy plan as verified. Where a
lidar voxel model *is* available, ``FlyabilityField`` overrides this policy with
a measured free window (and see SHORTFALLS 1b — that band's datum is wrong today).
"""

from __future__ import annotations

import numpy as np

from droneplanner.forest.navigability import FlyabilityField
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import AltitudePolicy
from droneplanner.model.plan import FlightBand


def canopy_height(dtm: np.ndarray, dsm: np.ndarray) -> np.ndarray:
    """Canopy height model (m above ground), clamped non-negative."""
    chm: np.ndarray = np.maximum(dsm - dtm, 0.0).astype(np.float32)
    return chm


def concealment_mask(grid: Grid, start: Coordinate, radius_m: float | None) -> np.ndarray | None:
    """Cells where concealment is required (``Mission.covered_radius_m``), or None = all."""
    if radius_m is None:
        return None
    a, b, c, d, e, f = grid.transform
    rows, cols = np.mgrid[0 : grid.rows, 0 : grid.cols].astype(np.float64)
    cx = a * (cols + 0.5) + b * (rows + 0.5) + c
    cy = d * (cols + 0.5) + e * (rows + 0.5) + f
    mask: np.ndarray = np.hypot(cx - start.x, cy - start.y) <= radius_m
    return mask


def _band_from_canopy(
    chm_m: float, target_agl_m: float, policy: AltitudePolicy, *, covered: bool = True
) -> tuple[float, float, float, FlightBand]:
    """``(preferred, floor, ceil, band)`` AGL for one cell, from its canopy height.

    Either **under** the canopy with clearance or **over** it with the same
    clearance — never inside it. Holding a target altitude regardless of canopy is
    how the drone ended up 1 m above the crowns of a stand it was not entering.
    """
    floor = policy.min_agl_m
    target = max(target_agl_m, floor)

    # Outside the concealment zone there is no reason to be among the stems:
    # going over is faster (no clutter speed limit) and needs no avoidance.
    if covered and chm_m >= policy.min_canopy_m:
        ceil = chm_m - policy.canopy_clearance_m
        if ceil >= floor:  # room to get under it and still clear the ground
            return min(target, ceil), floor, ceil, FlightBand.BELOW_CANOPY

    # Not going under: go over, with the same clearance the crowns get from below.
    # The vegetation raises the *floor* here — leaving it at ``min_agl_m`` let the
    # altitude smoother pick a height inside the crowns of a stand it had already
    # decided not to enter.
    over_floor = max(floor, chm_m + policy.canopy_clearance_m)
    pref = max(target, over_floor)
    band = FlightBand.CANOPY_TOP if pref > target else FlightBand.OPEN
    return pref, over_floor, pref, band


def altitude_band_at(
    rc: tuple[int, int],
    *,
    chm: np.ndarray,
    target_agl_m: float,
    policy: AltitudePolicy,
    flyability: FlyabilityField | None = None,
    covered: np.ndarray | None = None,
) -> tuple[float, float, float, FlightBand]:
    """Return ``(preferred, floor, ceil, band)`` altitude AGL for one cell.

    A lidar voxel model wins where it has a usable window: it measured the free
    space, this policy only infers a ceiling from the canopy top. Outside the
    concealment zone (``covered`` False) neither applies — the drone goes over.
    """
    in_zone = True if covered is None else bool(covered[rc])
    if flyability is not None and in_zone:
        ceil = float(flyability.nav_ceil_m[rc])
        if ceil > 0.0:
            floor = float(flyability.nav_floor_m[rc])
            pref = float(flyability.max_flyable_altitude_m[rc])
            return min(max(pref, floor), ceil), floor, ceil, FlightBand.UPPER_TRUNK
    return _band_from_canopy(float(chm[rc]), target_agl_m, policy, covered=in_zone)


def planned_flight_agl(
    chm: np.ndarray,
    target_agl_m: float,
    policy: AltitudePolicy,
    flyability: FlyabilityField | None = None,
    covered: np.ndarray | None = None,
) -> np.ndarray:
    """Per-cell height AGL the planner would fly — :func:`altitude_band_at`, rastered.

    The cost surface evaluates the viewshed at this altitude while the corridor
    planner flies ``altitude_band_at``; a unit test pins them cell-for-cell. If
    they drift, concealment is scored at an altitude the drone never occupies.
    """
    floor = policy.min_agl_m
    target = max(target_agl_m, floor)
    ceil = chm - policy.canopy_clearance_m
    enterable = (chm >= policy.min_canopy_m) & (ceil >= floor)
    zone = np.ones(chm.shape, dtype=bool) if covered is None else covered
    enterable = enterable & zone

    under_alt = np.minimum(target, ceil)
    over_alt = np.maximum(target, chm + policy.canopy_clearance_m)
    agl = np.where(enterable, under_alt, over_alt)
    if flyability is not None:
        banded = np.minimum(
            np.maximum(flyability.max_flyable_altitude_m, flyability.nav_floor_m),
            flyability.nav_ceil_m,
        )
        agl = np.where((flyability.nav_ceil_m > 0.0) & zone, banded, agl)
    return agl.astype(np.float32)


def band_raster(
    chm: np.ndarray,
    target_agl_m: float,
    policy: AltitudePolicy,
    flyability: FlyabilityField | None = None,
    covered: np.ndarray | None = None,
) -> np.ndarray:
    """Per-cell :class:`FlightBand` as an int code, for reporting and viz.

    0 = open, 1 = canopy-top skim, 2 = below canopy, 3 = lidar-confirmed trunk.
    """
    target = max(target_agl_m, policy.min_agl_m)
    zone = np.ones(chm.shape, dtype=bool) if covered is None else covered
    enterable = (
        (chm >= policy.min_canopy_m)
        & ((chm - policy.canopy_clearance_m) >= policy.min_agl_m)
        & zone
    )
    # "Skim" means the canopy is what sets the altitude, not the target.
    skimming = ~enterable & ((chm + policy.canopy_clearance_m) > target)
    codes = np.where(enterable, 2, np.where(skimming, 1, 0)).astype(np.int32)
    if flyability is not None:
        codes = np.where((flyability.nav_ceil_m > 0.0) & zone, 3, codes).astype(np.int32)
    return codes
