# Known shortfalls & gaps (pre-simulation review)

Status: **M1 (2.5D concealment) + M1.1 (crossings stub) + M2 (forest flyability) +
kinematic feasibility pass implemented.**
This is an honest inventory of what is *not* yet done or *not* trustworthy, focused
on what will bite when the planner's output is handed to PX4 SITL / Gazebo (M4).

Severity: 🔴 blocks trustworthy sim results · 🟠 material accuracy gap · 🟡 minor / later.

## Recently closed

- ✅ **Altitude-frame CFIT bug at the `.plan` boundary** (2026-07-21, found by the
  first real-terrain run). The writer emitted the planner's terrain-relative AGL
  directly as a **relative-to-home** altitude; over the demo AO's 73.5 m of relief
  that put **57 of 77 waypoints below ground** (min clearance −66 m), and
  `plannedHomePosition[2]` was an AGL value rather than ground AMSL. Fixed:
  `TrajectoryVertex.ground_elev_m` + conversion in `io/mavlink.py`; clearance is
  now a constant 30 m AGL, 0 waypoints underground. *It hid behind two degenerate
  layers — the no-terrain stub is flat, and Gazebo's default world is flat — a
  reminder that green tests over synthetic flat inputs prove very little.*
- ✅ **`droneplanner demo-ao`** exports the procedural AO as real GeoTIFF layers +
  a mission YAML, so `plan` produces a genuine cost-aware route (tortuosity 1.068,
  6 turns >5°) instead of the straight-line stub — and exercises the same
  ingestion path real DTM/DSM data will use.

- ✅ **Kinematic / tether feasibility** (`planner/kinematics.py`): corners are
  filleted to arcs ≤ `min_turn_radius`, altitude is smoothed to respect the
  climb/descent-rate limits, and reversals are detected. The report carries a
  `KinematicReport` and flags `tether_reversal` (NO-GO), `turn_radius`, and
  `climb_rate` (REVIEW). *Remaining:* it's a post-search pass (not baked into the
  search), and fillet arcs are checked only against the non-navigable mask, not
  against exposure/high-cost cells (a rounded corner could bulge into a slightly
  more exposed cell).
- ✅ **Altitude is a free variable** within the navigable band (not pinned to a
  fixed "fly high" assumption); the planner varies height to follow the gap,
  climb-limited.
- ✅ **Forest is treated as high-value cover** (`fields/composite.py`): FLYABLE
  forest is rewarded (concealment/cover), MARGINAL penalized, NON_NAVIGABLE is an
  obstacle.

---

## Top priorities before sims

1. 🔴 **Exposure is evaluated at ground level, not at flight altitude**
   (`fields/viewshed.py`, `target_height_m=0`). The viewshed asks "can an
   observer see this ground cell", but the drone flies at cruise AGL (M1) or the
   trunk band (M2). Concealment is measured for the wrong object, so the
   `mean_exposure` metric and the route it drives are systematically off. *Fix:
   evaluate the viewshed at the planned flight altitude per cell.*
   *Note (user, later versions):* the fuller fix — using **lidar to determine
   cover** in the visibility model — belongs to the online/perception side, not
   this offline route planner; treat it as a later-version enhancement.
2. 🟠 **Forest voxel resolution is tied to the corridor grid**
   (`pipeline._load_flyability`). The spec wants **0.25–0.5 m** voxels for
   clearance near the airframe scale; we voxelize at the DTM/DSM horizontal
   resolution (often ≥1 m) co-registered to the corridor grid. Horizontal
   clearance is therefore coarse — **flyability can read optimistic** (a gap
   narrower than a cell is invisible). *Fix: voxelize at fine resolution, compute
   clearance/navigability there, then aggregate to the corridor grid.*
3. 🟠 **Performance will not scale to a real 5 km AO.** The viewshed
   (`O(observers × perimeter × ray_len)`) and Theta\* are pure Python, sized for
   the ~120–160-cell demo. The §6.7 budget (5 km route / 25 km² / 1 m raster /
   0.5 m voxel in <5 min) is **not met** at real resolution. *Fix: Numba /
   vectorized viewshed, and a coarser search grid with fine local refinement.*

---

## M1 — concealment / cost surface

- 🟠 **Land cover / vegetation is not ingested.** `LayerPaths.land_cover` exists
  in the contract but is unwired: `snag` is identically 0 and the survivability
  `(1 − veg_cover)` term is 0. Canopy is one of the strongest real concealment
  and snag drivers — currently the only "cover" reward comes from terrain
  defilade (drainage/negative TPI), not vegetation. (`fields/composite.py`,
  `fields/snag.py`, `fields/survivability.py`.)
- 🟠 **Threat *sectors* are ignored.** `ThreatModel.sectors` (azimuth ranges,
  optionally without an observer point) are accepted but never used;
  survivability "openness to threat" uses observer *points* only. Sector-only
  threat definitions have no effect on the plan. (`fields/survivability.py`.)
- 🟡 **Cost weights are uncalibrated.** The composite blend (`BASE_COST`,
  `COVER_SCALE`, `FOREST_COVER_REWARD`, `FOREST_MARGINAL_PENALTY`, `ROAD_COST`,
  the survivability sub-weights) is heuristic; there is no cross-AO normalization
  or tuning study.
- 🟡 **Viewshed is an approximation.** Radial rays to boundary cells can miss
  interior cells; no earth-curvature/refraction (negligible <5 km) and fixed
  2 m observer height.
- 🟡 **`k_routes > 1` emits identical routes.** Spatial-diversity penalization is
  M3; today extra routes are duplicates.

## M1.1 — crossings (closed as an open-terrain stub, per direction)

- ✅ Roads are treated as method 4 (open crossing): rasterized into an on-road
  penalty so the planner minimizes on-road distance, and each route/road
  intersection is flagged as an OPEN crossing in the report.
- 🟡 **Only OPEN crossings.** Culvert/underpass/bridge routing, natural canopy
  bridges, and overhead spans (§5.4 methods 1–3) are intentionally deferred.
- 🟡 **Rail / buildings / culverts layers are not ingested;** only `roads`.
- 🟡 **Crossing geometry is coarse** (grid-cell resolution, `all_touched`); the
  penalty nudges toward short on-road distance but does not explicitly enforce a
  perpendicular crossing.

## M2 — forest flyability

- ✅ Implemented: voxelize → vertical segmentation → 3D EDT clearance →
  navigability classification (FLYABLE / MARGINAL / NON_NAVIGABLE) → 2.5D
  flyability cost + non-navigable obstacle mask + max-flyable-altitude band,
  wired into the corridor search and the report (non-navigable → NO-GO,
  marginal → REVIEW).
- 🔴 **No 3D kinodynamic refinement (OMPL).** The spec's part (B) — BIT\*/RRT\*
  sampling in the navigable voxel free-space honoring tether kinematics — is
  **not built**. OMPL's Python bindings are not pip-installable (require a
  source build), so this needs a packaging decision or an in-house 3D planner.
  Today M2 keeps routes in flyable tubes only at the **2.5D** corridor level.
- 🟠 **No explicit connected-corridor check** (spec §5.2 step 6). Connectivity is
  implicit (Theta\* routes around non-navigable cells) and the altitude profile is
  now climb-continuous (the kinematic smoother), but there is no hard guarantee a
  *consistent navigable altitude band* threads a segment — the smoother will keep
  the drone in-band or flag a climb violation, rather than proving a tube exists.
- 🟠 **LAS point classification is unused** in voxelization (`forest/voxelize.py`
  bins raw XYZ). Ground vs vegetation returns are needed for a correct DTM and
  understory density; today occupancy is undifferentiated.
- 🟡 **Thresholds are uncalibrated** (`FlyabilityConfig` uses the §4 table
  starting points — must be calibrated to the airframe in sim/flight).
- 🟡 **Lidar currency / DSM-vs-canopy coupling** is not validated; below-canopy
  concealment credit assumes the DSM includes canopy.

## Cross-cutting

- 🔴 **No real-AO validation.** Everything is exercised on synthetic fixtures and
  the procedural demo. The spec's small real lidar tile + DEM (§6.1) is not yet
  wired in, so field correctness on real data is unverified.
- 🟠 **Reference-data fidelity is assumed, not modeled — the corridor and the
  flight-time fixes share one shaky input.** The whole corridor is built from
  geospatial reference (DTM/DSM, lidar, land cover). That reference is **uneven in
  fidelity globally** and **perishable in conflict zones** (new craters, rubble,
  defoliation, construction, seasonal canopy). Two consequences, one root cause:
  (a) the planned corridor may not match the present world; (b) the flight stack's
  **VTRN drift-correction leans on the same class of basemap** and fails exactly
  where this drone hides (low-texture forest canopy) or where the map is stale.
  The flight side now quantifies this in `droneflight/correction.py` +
  [`droneflight/NAV-FIDELITY.md`](../droneflight/NAV-FIDELITY.md). **Planner-side
  fix (unbuilt):** ingest a per-cell **basemap matchability / currency** layer and
  add a *navigation-risk* term to `fields/composite.py`, so the route balances
  concealment against staying in reach of trustworthy optical fixes — otherwise a
  maximally-concealed route is a maximally-blind one. Ties into the M2 "lidar
  currency / DSM-vs-canopy coupling" 🟡 and land-cover (currently unwired) items.
- 🟠 **Cable-laydown prediction (M3) is absent** — where the fiber settles, and
  its per-segment survivability, are not modeled; the report's survivability is a
  placement heuristic, not a laydown result.
- 🟢 Determinism, CRS discipline, license hygiene, kinematic feasibility, and the
  human-in-the-loop REVIEW-cap all hold.

---

## Suggested sequence

1. Flight-altitude viewshed — makes the concealment metric honest.
2. Wire land cover + threat sectors — completes the M1 concealment inputs.
3. Fine-resolution voxelization + aggregation — makes flyability trustworthy.
4. Then SITL/Gazebo export (M4) with a real AO; treat OMPL 3D refinement and
   laydown (M3) as parallel tracks.
