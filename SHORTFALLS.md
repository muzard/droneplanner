# Known shortfalls & gaps (pre-simulation review)

Status: **M1 (2.5D concealment) + M1.1 (crossings stub) + M2 (forest flyability) +
kinematic feasibility pass implemented.**
This is an honest inventory of what is *not* yet done or *not* trustworthy, focused
on what will bite when the planner's output is handed to PX4 SITL / Gazebo (M4).

Severity: 🔴 blocks trustworthy sim results · 🟠 material accuracy gap · 🟡 minor / later.

## Recently closed

- ✅ **The concealment was fake: the drone flew ABOVE the canopy — FIXED
  2026-07-27** (item 1c, 🔴🔴; raised by the user immediately after item 1 landed,
  and the more fundamental of the two). The planner held a fixed **30 m above
  bare terrain**. The demo stands are 12–22 m, so the aircraft sat 8–18 m *over
  the crowns* while the cost surface rewarded forest as cover. `FlightBand`
  already had `CANOPY_TOP` and `BELOW_CANOPY` values that nothing ever produced,
  and Preflight-Design already prescribed the posture — *"canopy-top skimming /
  upper-trunk-space flight"* — so this was an unimplemented spec requirement, not
  a design question. The canopy height model needed no new data: it is `DSM - DTM`.

  **Fix, in three parts** (`fields/altitude.py`, `AltitudePolicy`,
  `planner/corridor.py`):
  1. **Altitude is surface-relative and as low as the constraints allow.** Target
     `cruise_agl_m` (default 30 → **10 m**), floored by `min_agl_m`, ceilinged at
     `canopy_top − canopy_clearance_m` wherever the stand is worth entering. The
     drone is either **under** the crowns with clearance or **over** them with the
     same clearance — never inside. (The first cut had the floor stay at
     `min_agl_m` in the over-canopy case, which let the altitude smoother place
     the aircraft at 5 m inside the crowns of a 9 m stand it had just declined to
     enter. Caught by a unit test before it ever ran.)
  2. **`waypoint_spacing_m` is now honoured by the corridor planner** — it was
     read only by the M0 stub, so real routes had legs up to 760 m. This is what
     makes low flight possible at all (below).
  3. **The demo AO grew a canopy.** `dsm = dtm.copy()` meant the whole forest
     story was untestable; there are now 12–22 m stands, two clearings and a belt
     of ridge scrub, laid out so the concealed route and the short one disagree.

  | demo AO, same terrain and observers | old posture | new posture |
  |---|---|---|
  | flight altitude | flat 30 m AGL | **9–15 m**, tracking the canopy |
  | mean drone exposure | 0.5500 | **0.1500** |
  | flown-trajectory exposure | 0.5061 | **0.1294** |
  | route flown in view of observers | 1998 m | **611 m** |
  | route flown below canopy | 0 m | **2654 m of 5299** |
  | waypoints | 145 | 282 |
  | min AGL under the flown profile | 23.99 m | 6.64 m |

  **Drone exposure falls 3.7×, and the drone is now less exposed than its own
  cable** (0.15 vs 0.27) — which is what hiding in a forest means. Under the old
  posture the aircraft was *twice* as exposed as the cable it was laying.

  **What actually limits "as low as possible" is the DTM, not the autopilot.**
  Sweeping the target altitude with 25 m spacing, the achieved clearance loses a
  near-constant ~3.3 m to terrain bulging between waypoints — so 10 m AGL clears
  by 6.7 m and 4 m AGL does not clear at all. Capping the descent *rate* barely
  helps (0.5 m/s instead of 2.0 m/s buys 0.37 m), because once waypoints are 25 m
  apart the route's own ground gradient (max 0.149 m/m) is gentler than the
  descent gradient anyway. The demo's DTM has **25 m cells**; on the 1 m lidar
  DTM a real AO would use, the same argument allows a far lower floor.

  | target AGL | 15 m | 10 m | 8 m | 6 m | 4 m |
  |---|---|---|---|---|---|
  | achieved min AGL (25 m spacing) | 11.67 | 6.67 | 4.67 | 2.67 | **0.67** |

  Waypoint spacing is the lever that made it work at all — at the old 100 m
  spacing a 10 m route flies **12 m underground**:

  | spacing | ∞ (as emitted) | 200 m | 100 m | 50 m | 25 m |
  |---|---|---|---|---|---|
  | min AGL at 10 m target | −12.27 | −1.70 | 2.87 | 4.69 | **6.67** |

  🔴 **The safety argument moved off this planner, deliberately (user decision,
  2026-07-27).** Obstacle clearance below the canopy is now the flight stack's
  job — camera/lidar avoidance — and the route is a corridor to be followed
  *approximately* ("it doesn't matter if it flies past a rock from the right or
  left side"). That is a coherent architecture, and it is why this planner may
  commit a leg below a canopy it can only see the top of. But it means:
  * a below-canopy plan is **not verified**, and the report now says so on every
    such route (`below_canopy_flight`, WARNING, with the length);
  * the capability it leans on **does not exist yet on this hardware** — onboard
    avoidance is Track B and needs an x86+RTX box the M4 lacks;
  * 2654 m of the demo route is currently concealment bought on credit.
- ✅ **Exposure is evaluated at flight altitude — FIXED 2026-07-27** (was item 1,
  🔴). The viewshed asked "can an observer see this **ground** cell"
  (`target_height_m=0`) while the drone flies 30 m above it, or in the trunk
  band. Concealment was measured for the wrong object, so both the metric and
  the route it drove were systematically off.

  **Fix:** the target elevation is now a first-class input to the viewshed
  (`fields/viewshed.py`, `target_elev_m`), and `fields/composite.py` builds
  **two** fields instead of one — `exposure` for the **drone** at
  `dtm + planned flight AGL`, `exposure_ground` for the **cable**, which really
  does lie on the surface. Occlusion still marches over the DSM in both. Cost is
  unchanged (the pipeline already ran two viewsheds; the second is now the
  ground one, handed to `build_survivability` instead of recomputed there).

  The flight altitude comes from `forest.navigability.planned_flight_agl`, which
  is the **vectorized form of the same `altitude_band_at` the corridor planner
  flies** — one definition, pinned cell-for-cell by a test, so the field cannot
  drift to an altitude the drone never occupies.

  **The measurement error was ~2×, in the flattering direction.** On the demo AO
  the old planner reported **0.28** mean exposure for a route whose aircraft was
  actually at **0.55** — it was quoting the ground's concealment, not the
  drone's. Over the whole AO, mean exposure is **0.239 at ground level** and
  **0.509 for a drone at 30 m AGL**: more than half the AO (**52.8 %**) is
  visible to an observer at 30 m and invisible under the canopy.

  ⚠️ **Fixing the measurement is what exposed the bigger defect** (item 1c
  below): an honest metric immediately showed the aircraft was flying above the
  canopy it was supposed to be hiding in. The two are one story — the planner was
  measuring the wrong object *and* putting it at the wrong altitude — and the
  numbers here are quoted after both fixes, on the canopy-bearing demo AO. The
  original item-1 measurements were taken on the old canopy-free demo, where
  raising the target could only ever *add* exposure.

  **A third number re-scores line-of-sight along the flown 3D trajectory** —
  filleted geometry, climb-smoothed altitudes, PX4's front-loaded descent —
  using the same `sample_flown_profile` samples the clearance guarantee uses,
  averaged by *track length* (a fixed sample count per leg would let the short
  fillet arcs carry the metric). It exists because the raster assumes every cell
  is overflown at its *band* altitude, and the route departs from that. On the
  demo AO the two disagree by 10 %, and the disagreement decomposes cleanly:

  | scored along | mean exposure |
  |---|---|
  | raster, over the corridor cells at band altitude (`mean_exposure`) | 0.1500 |
  | trajectory, altitudes interpolated linearly | 0.1490 |
  | trajectory, **altitudes as PX4 flies them** (`mean_exposure_flown`) | **0.1294** |

  Geometry contributes almost nothing now (−0.001): with 25 m waypoint spacing
  the emitted track and the corridor cells are nearly the same line. The
  remaining **−0.020** is the front-loaded descent, which cuts the *other* way
  from item 0 — the behaviour that flew the aircraft into a slope also gets it
  low early, and low is concealed. ⚠️ AO-specific: where cover is canopy rather
  than terrain, arriving low early need not help.

  ⚠️ **The demo route changed**, so the six CFIT verification flights of
  2026-07-27 were flown against the *old* route (and the old DSM). The clearance
  guarantee still passes on the new one (min **6.64 m** AGL, 0 violations) but it
  is a different, lower, much denser route — 282 waypoints against 87 — so
  re-flying it exercises neither the same leg nor the densifier, and the
  vertical-acceptance behaviour that stalled two missions is now being asked of
  3× as many waypoints. **This is the single most important thing to re-fly.**
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

  **RESOLVED over 12 flights: the mission stalls iff the vertical error at some
  waypoint exceeds the window — no exceptions.** 0/2 complete at 0.8, 4/5 at 2.0,
  **5/5 at 4.0**. The inserted vertex is bimodal (usually 0.66–0.73 m, spiking to
  2.1–2.6 m about 1 run in 6 — those spikes are the stalls); the *final* waypoint
  independently sits at ~1.78 m in every healthy run.
  `droneflight/configs/px4/terrain_following.params` is **4.0**, covering the
  whole observed distribution (worst 2.569 m) with 1.56× margin and still 3× under
  the 11.9 m measured clearance. It is an *acceptance tolerance*, not a clearance
  change — the aircraft flies no lower.
  ⚠️ The 5/5 does not by itself prove 4.0: none of those five spiked (max 1.83 m),
  so they would have passed at 2.0 too. The case rests on the distribution.

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

1. ✅ **Exposure at flight altitude — FIXED 2026-07-27**, see "Recently closed"
   for the mechanism and the measured before/after. *Note (user, later
   versions):* the fuller fix — using **lidar to determine cover** in the
   visibility model — belongs to the online/perception side, not this offline
   route planner; treat it as a later-version enhancement.

1b. 🔴 **The M2 navigable altitude band is NOT height above ground — it is height
   above the point cloud's global minimum z** (found 2026-07-27 while wiring the
   flight-altitude viewshed; **flagged, deliberately not fixed**). `voxelize` is
   called with `z_min = cloud.z.min()`, so a voxel level index is an *absolute*
   elevation. `build_flyability` then reports `nav_floor_m` / `nav_ceil_m` /
   `max_flyable_altitude_m` as `level * voxel_size_z` — still absolute — and
   `planner/corridor.py` adds them to the local DTM **as if they were AGL**.

   Measured on a synthetic tile with an identical 20 m canopy above the local
   ground everywhere, over ground rising 0 → 70 m (so every column should report
   the same band):

   | local ground | `nav_floor_m` | `max_flyable_altitude_m` |
   |---|---|---|
   | 0 m | 26.45 | 95.75 |
   | 30 m | 56.45 | 95.75 |
   | 70 m | 2.95 | 2.75 |

   Identical forest, three different answers. The planner would command 56 m
   "AGL" over the 30 m column — through the canopy it is meant to hide in. The
   segmentation is confused too: at the low column it picks the free space
   *above* the canopy (95 m) as "max flyable altitude".

   **Currently masked by exactly the degeneracies this file keeps warning about:**
   no real point cloud has ever been run (`demo_flyability` builds a synthetic
   field directly at `z0 = 0`), and flat inputs make absolute and relative
   heights coincide. It bites the first time real lidar over real relief is
   ingested — the same AGL-vs-AMSL confusion as item 0 and item 0b, in a third
   place. *Fix: normalize the voxel grid (or the derived band) to height above
   the DTM per column before segmentation.* Not done here because it changes the
   forest model and the routes it produces, and because it deserves its own
   verification pass.

   ⚠️ **This bounds items 1 and 1c on real data:** the flight-altitude viewshed
   is only as correct as the flight altitude handed to it, and `altitude_band_at`
   lets a lidar voxel window *override* the canopy-derived ceiling — correctly,
   since a measured free window beats a guess, but that override is exactly the
   broken number. **Without lidar the new canopy path is unaffected** (it uses
   `DSM - DTM`, which is per-cell terrain-relative by construction), so the
   working configuration today is the one with no point cloud. The moment real
   lidar is wired in, this must be fixed first.
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

- ✅ **A cell's own canopy can now occlude what flies beneath it** (roadmap C1,
  fixed 2026-08-11). `single_viewshed` excluded the target cell's own DSM from the
  occluder set — right for an aircraft *above* the surface, wrong for one *under*
  it. Measured on the demo AO from a 300 m sensor: **100 % of the 3722 m flown
  below canopy scored visible** while the aircraft was physically beneath the
  crowns. So the planner **could not represent sub-canopy concealment against any
  look-down observer** — the single threat geometry the below-canopy posture
  exists to defeat. Visibility is now a **weight in [0, 1]**
  (`ThreatModel.canopy_occlusion`, default gap fraction **0.10**), and the same
  rule is applied by `point_exposure`, so the flown re-score and the raster field
  cannot disagree about a crown. `single_viewshed_bool` keeps the boolean contract
  the A4 reference validation needs.
  🔴 **The gap fraction is a guess with a wide range.** Closed Nordic stands run
  5–20 %; no gap-fraction measurement has ever been ingested, and the C1 default
  is the middle of a literature band. Sweep it before trusting any below-canopy
  concealment claim — at 300 m the under-canopy scored-visible length moves
  **675 → 1699 m across gap 0.0 → 0.20**.
  ⚠️ Three conditions must hold before a crown is counted as in the way: the
  target is under its own DSM, the **observer is above that DSM**, and the
  sightline is steeper than `sub_canopy_max_elev_deg` (10°). The middle one was
  missing in the first cut and is load-bearing — without it the model also
  attenuates a trunk-space-to-trunk-space sightline, where nothing is in the way,
  which *invents* concealment.
- ✅ **Observer height is a mission input** (2026-08-11, found while running the
  C1 gate). `build_cost_surface` hard-coded a 2 m eye, so **the planner could not
  route against a mast or airborne sensor at all** — and C1 was therefore inert in
  the pipeline (measured: the gap fraction changed the demo route by exactly
  nothing). `ThreatModel.observer_height_m` now feeds both viewsheds and the flown
  re-score, and `test_canopy_occlusion_reaches_the_cost_surface` pins the wiring.
- 🔴 **The observer's eye sits on the DSM, i.e. on the crowns.**
  `eye = dsm[observer] + observer_height_m`, so a "2 m dismount" standing in a
  22 m stand is modelled at 24 m — above the canopy, with the sightlines of
  something in the treetops. Every ground-observer number in the programme carries
  this. On the demo AO the error is small (the standoff positions sit in 4–5 m
  ridge scrub, not mature stand) but it is systematic and in the
  adversary-favouring direction. Fixing it means choosing whether an observer
  stands on the terrain (DTM) or on the surface, per observer — a mission-contract
  question, and it moves every historical exposure figure.
- ✅ **The viewshed has a range term** (roadmap C2, built 2026-08-11). It had none
  at all: a cell 3 km from an observer scored exactly like one at 300 m, while the
  median exposed sample on every study arm sat at **1.3–1.6 arcmin** against the
  **3–7′** where real observers actually acquired real sUAS — and against sky, a
  far more favourable background than forest. `ThreatModel.detection`
  (`DetectionModel`) turns an acquisition angle and an optic magnification into a
  **slant** range: **1031 m** for this 0.9 m airframe unaided at the empirical 3′,
  full weight to half of that and a linear ramp to zero (`falloff="ramp"`). It
  multiplies with C1's crown weight, so "exposure" is now an approximate
  *probability a searching observer acquires the aircraft*.
  ⚠️ **Visual acquisition only** — not radar, not thermal, and **not acoustic**,
  where published C-UAS ranges (300–500 m for small quadcopters) are comparable to
  the unaided visual threshold and are largely *independent of flight band*. A
  posture argued on this field alone is argued on one of three detection modes.
  🔴 **The default is not conservative in one direction that matters:**
  `optic_magnification` defaults to 1.0, and a 12× observation optic multiplies the
  acquisition range by 12 (12.4 km at 3′) — which covers any AO and erases the
  range argument entirely. It requires the observer to be searching that bearing,
  which is what `threat.sectors` expresses — **implemented 2026-08-15** (C3, below),
  including a per-sector `detection` override, so "a spotting scope watches this
  bearing, dismounts hold the rest" is now a statement the planner can consume.
- ✅ **`exposure_ground` is now `ground_los`, and gets no detection model**
  (roadmap C2's explicit decision, 2026-08-11). It scores the **DSM surface**, not
  the fiber — which at 0.25–0.28 mm is **never optically resolvable at any range**,
  so a visual acquisition model sized for it would zero the field and delete cable
  survivability from the cost surface. It therefore stays a **pure line-of-sight
  openness proxy**, and was renamed precisely so it cannot be read in the same
  currency as `exposure` now that `exposure` is a detection probability.
  `objective_scores.mean_exposure_cable` became `mean_cable_los` for the same
  reason. 🟠 The residual gap: nothing models what the *cable* actually presents to
  an observer — disturbed ground, a corridor through vegetation, a trench line —
  so `ground_los` answers "could someone see this patch of ground", which is a
  proxy for cable survivability and not a measurement of it.
- 🔴 **`min_canopy_m` is a hard band threshold with no hysteresis, and a stand
  sitting at it makes the aircraft porpoise.** Under-canopy and over-canopy
  altitudes are ~5 m apart with **no admissible altitude between them** (the crowns
  are physically in the way), which is fine as a one-off transition and is not fine
  along a canopy gradient that drifts across the threshold. Measured on the demo AO
  at `size=120`: CHM 11.21 / 12.05 / 11.19 / 12.03 m over consecutive waypoints
  gives commanded AGL **14.21 / 9.05 / 14.19 / 9.03**, i.e. **16.9 m/s of climb
  demanded on a 2.46 m leg** against a 3.0 m/s airframe — `climb_violations=1`,
  `feasible=False`. Guarded by an **xfail** test
  (`test_canopy_band_does_not_oscillate_across_the_entry_threshold`).
  ⚠️ **C2 did not cause this but makes it more likely**: with exposure range-limited
  to zero over most of an AO, routes are driven by length and take shorter, steeper
  lines across whatever stand is in the way. And PX4 flies **~3.6×** the vertical
  travel a plan commands, so a sawtooth like this is worse in the air than on paper.
  Fixing it is a **design choice, not a patch** — per-cell rasters cannot carry
  hysteresis (the band raster and `planned_flight_agl` are deliberately one
  definition), so the options are a band-instability term in the cost surface, or a
  path-level rule that commits to over-canopy when the band flips repeatedly over a
  short run. The second is cheap and costs concealment. **User's call.**
- 🟠 **Land cover / vegetation is not ingested.** `LayerPaths.land_cover` exists
  in the contract but is unwired: `snag` is identically 0 and the survivability
  `(1 − veg_cover)` term is 0. Canopy is one of the strongest real concealment
  and snag drivers — currently the only "cover" reward comes from terrain
  defilade (drainage/negative TPI), not vegetation. (`fields/composite.py`,
  `fields/snag.py`, `fields/survivability.py`.)
- ✅ **Threat *sectors* now reach the plan** (roadmap C3, 2026-08-15).
  `ThreatModel.sectors` had been accepted and read by nothing since M0, so the
  premise *"the enemy is over there and we are flying away from them"* could not
  be stated to the planner at all. `fields/sectors.py` implements both halves:
  a sector **with** an observer masks that observer's viewshed to its arc (so it
  reduces the drone `exposure` field *and* the cable's `ground_los`), and a
  sector **without** one becomes `openness_to_threat_sectors` — the terrain-
  masking term the survivability formula has named since M0 and never had.
  Measured on the demo AO with a 12× optic (the case where C2 leaves any exposure
  at all), a 90° arc cuts `mean_exposure_flown` **0.123 → 0.056** below canopy and
  **0.512 → 0.211** at flat 30 m, and the planner spends the slack: the route
  shortens 5316 → 5209 m and flies **240 m less** below canopy.
  🔴 **That is the danger of the feature, and it is not a bug:** a sector is an
  *intelligence assumption* that removes cost from the surface, so it buys
  concealment on credit against the adversary facing where the mission says. An
  adversary who turns around invalidates the plan, and nothing here models that.
  `report.json` carries `threat_sectors` (one line per sector, naming what it
  bound to) and the report raises `threat_sectors_applied`; a mission that
  supplies sectors on a path that cannot use them raises
  `threat_sectors_ignored` **WARNING** instead of silently dropping them.
  🟠 **Residual — the bearing-only field rests on a range assumption.** With no
  observer there is nothing to run a viewshed from, so a cell is masked along a
  bearing when terrain within `horizon_m` rises above it, i.e. the threat is
  treated as being *at the horizon*. The default 2000 m is not principled and the
  field is sensitive to it: on the demo AO the fully-masked fraction moves
  **19.1 % → 28.0 %** between a 500 m and a 2000 m horizon (mean openness 0.340 →
  0.264). Trust the ordering, not the level.
  🟠 **Azimuth only.** A sector has no elevation extent, so an airborne sensor
  searching a 90° arc is modelled as searching it from horizon to zenith.
- 🟡 **Cost weights are uncalibrated.** The composite blend (`BASE_COST`,
  `COVER_SCALE`, `FOREST_COVER_REWARD`, `FOREST_MARGINAL_PENALTY`, `ROAD_COST`,
  the survivability sub-weights) is heuristic; there is no cross-AO normalization
  or tuning study.
- ✅ **The demo AO now has a canopy** (fixed 2026-07-27 with item 1c). It used to
  set `dsm = dtm.copy()`, so the DSM had no trees at all and the whole
  concealment story was unfalsifiable — the cost surface rewarded "forest" the
  line-of-sight model could not see, and altitude had nothing to hide under.
  `demo_canopy()` now builds 12–22 m stands, two clearings on the natural
  corridor, and ridge scrub. ⚠️ **This changed `demo_ao/dsm.tif`**, hence the
  demo route; the Gazebo world is unaffected (`gz-world` reads the DTM, and home
  is the mission start, which did not move).
  🟠 **Residual:** the M2 `demo_flyability()` voxel forest is still built
  independently of this canopy, so the two disagree in detail — the voxel scene
  is illustrative scenery, not a voxelization of the demo DSM.
- ✅ **Viewshed cross-validated 2026-07-28** (TEST-PLAN A4) against an independent
  implementation — per-target straight-segment LOS, a different algorithm rather
  than a refactor (`tests/unit/test_fields_viewshed_reference.py`). It had never
  been checked against anything, while every concealment number rests on it.

  | terrain | agreement |
  |---|---|
  | flat (any observer) | **100 %** |
  | hard wall — shadow behind it | **100 %** |
  | random rough | 99.8 % |
  | canopy belt, drone at 8 m AGL | 99.6 % |
  | smooth gaussian hill / ridge | 92.8–93.6 % |
  | **real demo AO, both observers** | **99.0 %** |

  **On the demo AO the route's mean exposure differs by 0.0085** (0.150 vs
  0.142) — immaterial to routing. And the errors are **conservative**: 0.94 % of
  cells over-report exposure against 0.07 % that under-report, so the field does
  not promise cover the terrain fails to provide.

  Two things the exercise corrected:
  * 🔴 **The documented caveat was false.** This file and the module both claimed
    radial rays "can miss interior cells". Measured over four observer positions:
    **zero cells go untested** — rays to a full perimeter cover a rectangular grid
    exhaustively. The real approximation is a **bearing error** (a cell is tested
    against a ray aimed at a *boundary* cell, not at itself, so occluders are up
    to half a cell off-line), which costs ≤3 % on smooth terrain and ≤0.1 % on
    rough or canopy terrain.
  * The running-max-**angle** test is algebraically identical to a segment-height
    test given the same ray and cells, so ray choice is the *only* source of
    disagreement. Pinned by a test.

  🟡 **Still approximate, and unchanged:** no earth-curvature/refraction
  (negligible <5 km), fixed 2 m observer height, and a target *below* the local
  surface (a drone under canopy) is not occluded by the canopy of its **own**
  cell — a sub-cell effect the surrounding stand dominates. Disagreements
  concentrate exactly where a sightline grazes a slope, i.e. where sub-cell
  terrain detail neither model has would decide the answer; a finer DEM is the
  only real fix.
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

### 🔴 No stem-map capability at all — and TEST-PLAN N4 makes it the critical path

Routing *between mapped stems* is now the only surviving architecture for
below-canopy flight (reactive avoidance is dead: four independent negatives, see
TEST-PLAN N2). It needs a **stem map** — individual trunk positions and diameters.
**Nothing in this project produces one, and nothing has been validated against real
data.** Stated precisely, because the adjacent machinery makes it look closer than
it is:

| capability | state |
|---|---|
| point-cloud reader (`io/pointcloud.py`, laspy 2.7.0) | ✅ built, **never called** by the pipeline or any test |
| voxel occupancy / vertical segmentation / 3D EDT | ✅ built, but **per-column**, at 25 m corridor cells |
| individual **stem detection** (position + DBH) | ❌ **does not exist** |
| stem map from **imagery** | ❌ does not exist — and see the caveat below |
| any real `.laz`/`.las` ever ingested | ❌ **none has ever existed in this repo** |

⚠️ **The distinction that matters:** `forest/segment.py` splits each *column* into
understory / trunk space / canopy and `navigability.py` reports a `trunk_window_m`.
That is a **vertical free window per grid cell**, not a stem. No cylinder fitting,
no clustering, no individual-tree detection. The only place trunks exist as objects
is `io/forest_models.py`, which **generates synthetic** trunks from a canopy height
model at an operator-set density — a *generator, not a detector*. **Every trunk
manifest flown to date is fabricated**, so no result so far says anything about
whether real stems can be found.

⚠️ **Imagery alone probably cannot do this, and that is a geometry problem, not a
software one.** Photogrammetry reconstructs what the camera sees, which from above
is the canopy *top* — the same DSM the planner already has. The stems are
underneath it. Getting them needs returns that penetrate the canopy: airborne or
(better) UAV lidar, or an under-canopy/terrestrial survey. So "imagery and scans"
should be read as **scans; imagery contributes the DSM/DTM context, not the stems.**

🔑 **Map accuracy is a first-class requirement, not a detail.** TEST-PLAN N4 measured
the air-gap budget at **2.4 m**. Stem position inferred from ALS crown centroids
carries roughly 1-2 m of error, which would eat most of that budget before the
aircraft has moved; high-density UAV lidar is substantially better. **So the
required stem-map accuracy has to be derived and then verified against
field-measured plots** — that is the acceptance test, and public forest inventory
datasets with field-surveyed stem maps exist for exactly this purpose.

**What to build, in order:** (1) a stem detector — slice the cloud in a band above
ground, cluster, fit circles → position + DBH; (2) score it against a field-measured
plot for detection rate and position error; (3) only then feed it into the planner,
**as a cost-surface term rather than a post-filter** (N4 measured that composing
concealment-then-stems has no admissible path in dense forest).

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

1. ✅ Flight-altitude viewshed + below-canopy posture — done 2026-07-27; the
   concealment metric measures the aircraft, and the aircraft is under the trees.
2. **Fly the new route in SITL.** It is lower (6.6 m min AGL vs 24), denser (282
   waypoints vs 145) and unflown. The vertical-acceptance behaviour that stalled
   two missions in the CFIT campaign is now being asked of 3× as many waypoints,
   at a third of the altitude — nothing about this is verified.
3. **Fix the M2 band datum (item 1b)** — prerequisite for any real-lidar run, and
   for trusting the altitude band when a voxel model overrides the canopy one.
4. Wire land cover — threat sectors landed 2026-08-15 (C3); land cover is what is
   left of the M1 concealment inputs.
5. Fine-resolution voxelization + aggregation — makes flyability trustworthy.
6. Then SITL/Gazebo export (M4) with a real AO; treat OMPL 3D refinement and
   laydown (M3) as parallel tracks.
