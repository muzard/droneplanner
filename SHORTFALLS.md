# Known shortfalls & gaps (pre-simulation review)

Status: **M1 (2.5D concealment) + M1.1 (crossings stub) + M2 (forest flyability) +
kinematic feasibility pass implemented.**
This is an honest inventory of what is *not* yet done or *not* trustworthy, focused
on what will bite when the planner's output is handed to PX4 SITL / Gazebo (M4).

Severity: 🔴 blocks trustworthy sim results · 🟠 material accuracy gap · 🟡 minor / later.

## Recently closed

- ✅ **CFIT on descending legs — FIXED 2026-07-27** (was item 0, 🔴🔴; the full
  diagnosis is kept below under "Top priorities" for the record). Clearance was
  guaranteed against linear altitude interpolation; PX4 front-loads a leg's
  descent, so on the demo AO the route sat **6.6 m underground** at its worst
  point and it crashed the aircraft in SITL on 2026-07-24.

  **Fix = fix option (a) driven by a check that cannot be fooled**
  (`planner/terrain_profile.py`, wired in `planner/corridor.py`):
  * `assess_clearance` re-scores the **final** altitudes against real terrain
    under the *flown* profile (front-loaded descent; climb deliberately modelled
    as linear, since prompt climbing only adds clearance and is unvalidated).
    This is the guarantee, so a plan cannot silently claim clearance it lacks.
  * `densify_unsafe_descents` splits **only the legs that check rejects**, placing
    each inserted vertex where the settle margin (level-flight run before arrival)
    is longest. Planner altitudes are AGL, so an inserted vertex picks up a
    terrain-following altitude from the normal band + smoothing pass for free.

  🔴 **v1 of this fix caused a worse crash than the bug — read this before
  touching it (2026-07-27, `sim-logs/cfit/`).** v1 split on a *drop threshold*
  (`drop > cruise_agl − 5 m`) rather than on the clearance check, so on the demo
  AO it also split leg 10 — **already clear at +9.0 m AGL**. PX4 needs both
  horizontal *and* vertical acceptance (`NAV_MC_ALT_RAD`, 0.8 m); the vehicle
  passed that gratuitous vertex 1.58 m off vertically, the sequencer **froze at
  11/79**, and the trajectory generator slid past it for 565 s, descending, until
  it touched down 29 m below the plan's lowest altitude. **An unreachable
  waypoint is worse than the CFIT it prevents — it fails 40 legs earlier.**
  Two rules came out of it, both encoded in the module and its tests:
  1. **Insert only where the check fails.** A big drop is not by itself unsafe:
     PX4 flew leg 10's full 27.8 m drop over 251 m *unsplit* with **0.22 m** of
     vertical error. Splitting it made tracking **7× worse**.
  2. **Insert as few as possible, placed where the vehicle arrives level.** Over
     51 accepted arrivals (2026-07-24) PX4 held altitude to **0.35 m mean /
     0.64 m p95**; the vertex it refused had ~84 m of level run, the one now
     inserted has **327 m**.

  | demo AO | vertices | min AGL (flown profile) | legs <5 m | feasible |
  |---|---|---|---|---|
  | before | 77 | **−6.6 m** | 1 | ❌ |
  | v1 (drop threshold) | 79 | +11.9 m | 0 | ✅ *but froze SITL at 11/79* |
  | now (check-driven) | **78** | **+9.0 m** | 0 | ✅ |

  Cost: **1 vertex**. `KinematicReport` now carries `min_clearance_agl_m`,
  `clearance_violations` and `descent_vertices_inserted`; a breach is a
  **CRITICAL `terrain_clearance` finding → NO-GO**, and it breaks `feasible`.
  ⚠️ The residual margin is real and should be read: the same fixed route scores
  **23.5 m** under the old linear assumption but **11.9 m** under the flown
  profile, so front-loading still costs ~12 m of clearance after densification.
  ⚠️ Also note the old **−32.9 m / 5 legs** figure came from a cruder worst-case
  ("vehicle sits at the leg's lower altitude throughout"); the calibrated descent
  gradient says −6.6 m / 1 leg. Still CFIT, still fixed — but −32.9 m was an upper
  bound, not a measurement.
  ✅ **THE CLEARANCE FIX IS VERIFIED IN FLIGHT — SIX RUNS, 2026-07-27**
  (`sim-logs/cfit/` + `cfit/cfit-rep/`). Same AO, same leg that crashed the
  aircraft on 2026-07-24 at 0.8 m AGL. Minimum AGL per run, including the two
  that stalled: **11.97 / 11.94 / 11.96 / 11.91 / 11.96 / 11.91 m** — a **5 cm
  spread** against a predicted 9.02 m floor, with **0 of ~250 000 samples below
  5 m**. Four of the six flew the route end to end (78/78). **Terrain clearance is
  closed.**

  **And the mechanism is confirmed, not just the outcome.** Splitting the leg
  shrinks the commanded altitude *step*, and PX4's descent gradient scales with
  the step — measured on the same terrain:

  | leg | commanded step | flown descent gradient | min AGL |
  |---|---|---|---|
  | 2026-07-24, unsplit | −40.5 m in one go | **0.220** | 0.8 m → crash |
  | 2026-07-27, split | −18.3 m | **0.040** | **11.97 m** |

  So the front-loading the model assumes (0.333) is real and conservative — the
  worst observed is 0.220 — and the fix works for exactly the reason claimed.

  🟠 **Residual, NOT a planner defect but a real operational cost: an inserted
  vertex is mid-descent, so PX4's vertical acceptance window decides whether the
  mission sequences past it — and it is a RATE, not a switch.** Seven flights,
  outcome ordering perfectly by the vertical error at that vertex (every
  completion ≤0.841 m, every stall more):

  | ulog | `NAV_MC_ALT_RAD` | \|alt err\| at the vertex | outcome |
  |---|---|---|---|
  | `00_49_06` | 0.8 | 1.580 m (wp11, v1's spurious insert) | stall 11/79 |
  | `01_36_02` | 0.8 | 0.873 m (wp51) | stall 51/78 |
  | `03_28_04` | 2.0 | 0.669 m | COMPLETE |
  | `02_34_36` | 2.0 | 0.716 m | COMPLETE |
  | `03_11_41` | 2.0 | 0.721 m | COMPLETE |
  | `02_05_36` | 2.0 | 0.841 m | COMPLETE |
  | `02_52_58` | 2.0 | **1.190 m** | **stall 51/78** |

  **Stall rate 2/2 at 0.8 → 1/5 at 2.0.** The error varies run to run, so the
  window must cover the tail, not the median:
  `droneflight/configs/px4/terrain_following.params` is now **4.0** (3.4× the
  worst error seen, still 3× under the 11.9 m measured clearance). It is an
  *acceptance tolerance*, not a clearance change — the aircraft flies no lower,
  and every original waypoint tracks to ≤0.37 m so they are unaffected.

  💡 **If that is judged too loose, the alternative is measured:** drop the
  inserted vertex and instead raise the failing leg's **end altitude by 14 m**,
  which clears the same leg to 5.4 m with no new waypoint and no sequencing risk.
  It costs concealment — **30 m → 44 m AGL** over that stretch — so it is a
  survivability trade, not a free fix, and it is the user's call.

  ⚠️ **Two ULog audit traps cost a confidently wrong conclusion here
  ("it's nondeterministic, the param is irrelevant") — do not repeat them.**
  * **A param set at the `pxh>` prompt after boot appears in the ULog's
    `changed_parameters`, not `initial_parameters`.** Diffing only the boot
    snapshot made two differently-configured flights look identical.
  * **Score altitude error against `vehicle_local_position_groundtruth`.**
    `vehicle_local_position.z` is relative to the EKF local origin, not the
    mission's home datum, and `ref_alt` is NaN in these logs — comparing it to a
    plan altitude yields ~0 m error and explains nothing. Truth is only a proxy
    for what PX4 actually tests, but it predicts all three outcomes correctly.

  The deeper argument still stands: every inserted vertex buys a sequencing
  dependency, which is another reason to want the min-spacing emission filter
  (cross-cutting) that would make most insertions unnecessary.
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

0. ✅ **CFIT on descending legs — FIXED 2026-07-27, see "Recently closed" for the
   fix and its measured before/after. The diagnosis below is kept as the record
   of what the defect was, because it is the clearest example in this project of
   an offline model quietly disagreeing with the autopilot.**

   🔴🔴 **(historical) Terrain clearance assumed an altitude profile PX4 does not
   fly. PROVEN IN SITL: it crashed the aircraft (2026-07-24).**

   The emitted plan is correct at every vertex (all 77 waypoints 29.4–33.3 m AGL)
   *and* along every leg **if** altitude is interpolated linearly with ground
   distance (min 22.4 m AGL). PX4 does not do that. It descends toward the next
   waypoint's altitude **at its own vertical rate**, front-loading the descent:

   | leg wp50→wp51, 765 m, −40.5 m altitude | at 17 % along |
   |---|---|
   | linear interp (what the planner assumes) | +14.4 m → **27.0 m AGL** |
   | what PX4 actually flew | −11.9 m → **0.8 m AGL** |

   It burned 33 of the 40 m of descent in the first sixth of the leg, while still
   over the *high* end of a falling slope, and flew into the ground — the mission
   stalled at 51/77 and the watchdog aborted it.

   Scoring the same plan with a worst-case model (vehicle sits at the **lower** of
   the leg's two altitudes for the whole leg):

   | clearance model | min AGL | legs <5 m | legs <0 m |
   |---|---|---|---|
   | linear interp (assumed) | 22.4 m | 0 | 0 |
   | descend-early (PX4, observed) | **−32.9 m** | **5** | **3** |

   *Fix options:* (a) **densify vertices on legs with large |Δalt|** so the
   commanded profile is forced to track terrain — the standard terrain-following
   answer and probably the cheapest; (b) plan clearance against the descend-early
   worst case rather than linear interpolation; (c) cap the per-leg altitude
   change so the vehicle cannot get far ahead of the terrain.

   ⚠️ The **climb** side of that worst-case model is *unvalidated* — this flight
   passed the +62 m climb leg (wp40→41) without incident, so PX4 appears to climb
   promptly too. Only the descending legs (8 in this plan) are a confirmed hazard;
   the "3 legs underground" figure is an upper bound.

   ⚠️ Note this is **not** the 2026-07-21 altitude-frame CFIT bug (AGL emitted
   verbatim instead of relative-to-home). That one is fixed and still fixed — the
   vertices are all correct here. This is a *trajectory* defect: clearance is
   guaranteed for a path the autopilot never flies. **Flat-world SITL could never
   have found it** (constant altitude ⇒ no descent).

0b. 🟠 **The climb/descent-rate check runs in AGL, so terrain-driven vertical
   motion is invisible to it** (found 2026-07-27 while fixing item 0; **flagged,
   deliberately not changed**). `smooth_altitudes` picks and constrains altitudes
   in the **AGL** band, but the airframe's climb/descent limit applies to actual
   vertical motion (**AMSL**). On a constant-30 m-AGL route over the demo AO's
   73.5 m of relief the report says:

   | | max climb | max descent |
   |---|---|---|
   | `KinematicReport` (AGL) | **0.00 m/s** | **0.00 m/s** |
   | actually flown (AMSL) | 0.83 m/s | 0.93 m/s |
   | airframe limit | 3.0 | 2.0 |

   Here it hides nothing — the real rates are well inside the limits — but the
   check is *structurally* blind: on steeper terrain, or with a tighter descent
   limit, it would pass a plan the airframe cannot fly. It is the same
   AGL-vs-AMSL confusion that produced item 0, in a different place.

   **Not fixed here because it is a design decision, not a bug fix**: constraining
   in AMSL would change routes and could make terrain-following plans infeasible
   wherever terrain slope alone exceeds the climb rate — at which point the
   planner must either climb off the terrain (losing concealment) or route around
   the slope. That trade is the user's call. Item 0's fix is unaffected: it
   computes in AMSL throughout.

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

- 🟠 **Planner output geometry defeats the flight stack's tether speed policy.**
  The speed policy (`droneflight/speed_policy.py`) slows on **per-waypoint bend
  angle** (>30° taper, >150° reversal). But planner output is smooth at exactly
  that granularity: any-angle Theta* minimises turns, and the kinematic fillet
  then spreads whatever remains into arcs of many shallow (<25°) vertices — so
  the policy barely fires on real routes. Measured 2026-07-26: a slalom **forced**
  through an obstacle comb still peaked at **24° per vertex** (0 waypoints over
  30°); the flown `ev_test.plan` peaked at 16°. This is the same root as the
  duplicate-waypoint clusters (droneflight CLAUDE.md) — both smear a turn below
  the policy's trigger. **Consequence:** SITL §3 (tether policy in the loop) can
  only be tested with a *synthetic* turn-rich course
  (`droneflight/sitl/make_slalom_plan.py`), not planner output. **Fix candidates:**
  (a) trigger the policy on **path curvature / turn-rate over a window**, not
  per-vertex angle, so it matches how the planner actually emits geometry; or
  (b) emit a *speed profile* from the planner's kinematic pass (it already knows
  the fillet radii and where it flagged reversals) and carry it in the `.plan`,
  rather than re-deriving speed from geometry downstream.
  🛑 **DEPRIORITISED 2026-07-26 — do neither fix yet.** Acceptance §3 flew the
  synthetic course and the policy **did nothing measurable in the loop**: peak
  tether turn transient 0.3150 N (policy) vs 0.3164 N (control, cruise
  everywhere) — **−0.4 %** — for **2.19× flight time**, with cross-track slightly
  *worse*. Two reasons, both invisible to the offline model: PX4 already
  decelerates into corners on its own (control arm commanded 6.0 m/s at all 32
  waypoints, actually flew them at 4.4 m/s at ≤35° bends down to **0.51 m/s** at
  ≥140° reversals), and slowing further makes PX4 corner *tighter* (5.3 m vs
  13.6 m radius at the transient peak), which cancels the `v²` gain. Both fixes
  above would only make a **miscalibrated** policy fire more often. Worse, the
  offline model over-predicts 7–18× at bends ≥100° and **under-predicts 2–5× at
  bends ≤35°**, i.e. it is optimistic exactly where the real in-loop peak lives
  (45–50°). **Prerequisite for either fix: give `droneflight/tether.py` a PX4
  corner-speed prior** (measurable directly from `sim-logs/4.7/23_40_58.ulg`),
  then re-derive whether a planner-side trigger buys anything at all. Full result:
  droneflight CLAUDE.md → tether plugin §3, and RUNBOOK §4.7.
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
