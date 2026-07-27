# CLAUDE.md — droneplanner (offline route planner)

Pure-software subsystem: turns a mission (start, end, DTM/DSM/lidar/roads, threat
model, drone/spool geometry) into a 3D flight-and-laydown plan + a
human-in-the-loop risk report. Emits the QGC `.plan` that `droneflight` executes.
See `README.md` (usage) and `SHORTFALLS.md` (honest gaps — read before trusting a
plan or moving to sim).

## Status — through M2 + kinematic feasibility ✅
- **M0** scaffold, data contracts, I/O, CLI, straight-line stub + report.
- **M1** `fields/` (in-house viewshed, terrain TPI/slope/drainage, survivability,
  snag) → composite cost surface → **cost-aware any-angle Theta\*** corridor.
- **M1.1** road crossings as an open-terrain fly-over stub (`crossings/`).
- **M2** `forest/` voxel flyability (voxelize → vertical segment → 3D EDT
  clearance → FLYABLE/MARGINAL/NON_NAVIGABLE + nav altitude band), wired as an
  obstacle mask + cover reward.
- **Kinematics** (`planner/kinematics.py`) turn-radius filleting + climb-limited
  altitude within the navigable band + reversal flagging → `KinematicReport`.

## Run / test
```bash
cd /Users/arttu/Claude/Projects/DroneAutomation/droneplanner
. /Users/arttu/Claude/Projects/DroneAutomation/.venv/bin/activate
droneplanner plan --mission configs/mission.example.yaml --out out/   # stub (no terrain) -> REVIEW
droneplanner demo-ao --out demo_ao/                    # export demo AO as GeoTIFFs + mission YAML
droneplanner plan --mission demo_ao/mission.yaml --out out_demo/      # REAL terrain route (turns!)
droneplanner gz-world --dtm demo_ao/dtm.tif --plan out_demo/mission.plan \
    --base-world <PX4>/Tools/simulation/gz/worlds/default.sdf --out worlds/t.sdf
                        # Gazebo world = PX4's own world + the planned terrain
                        # add --flat for the diagnostic flat-DEM world
droneplanner viz   --out route.png     # demo AO: fields + flyability + route (static PNG)
droneplanner serve                     # interactive explorer, http://127.0.0.1:8765
ruff check . && ruff format --check . && mypy --strict src && pytest -q   # 211 tests, ~95% cov
UPDATE_GOLDEN=1 pytest tests/regression -q   # regenerate golden snapshot deliberately
```

## Key design decisions (don't regress; some came from user feedback)
- **Altitude frame at the `.plan` boundary (2026-07-21 — don't regress):** the
  planner thinks in **AGL above the terrain under each vertex**; a QGC `.plan`
  waypoint is **relative to home**. Emitting AGL verbatim was a CFIT bug — on the
  demo AO (73.5 m relief) **57 of 77 waypoints were below ground**. Now
  `TrajectoryVertex.ground_elev_m` carries the DTM elevation and `io/mavlink.py`
  emits `(ground+agl) − home_ground`, with `plannedHomePosition[2]` = ground AMSL
  at home. Frame choice (relative-to-home, `MAV_FRAME 3`) is deliberate and
  documented in that module — revisit there if AMSL or terrain-frame is ever
  needed. Guarded by `test_qgc_altitudes_are_relative_to_home_not_agl`.
- **The demo AO has a canopy (2026-07-27).** It used to set `dsm = dtm.copy()` —
  no trees in the DSM at all — so the cost surface rewarded forest the viewshed
  could not see and nothing about concealment was falsifiable. `demo_canopy()`
  builds 12–22 m stands, two clearings on the natural corridor, and ridge scrub.
  This changed `demo_ao/dsm.tif` and therefore the demo route; the Gazebo world
  is unaffected (`gz-world` reads the DTM, and home is the mission start).
- **`demo-ao` exists so `plan` has real terrain.** `configs/mission.example.yaml`
  has no layers → straight-line stub (zero turns, flat altitude), which makes SITL
  tests vacuous. `droneplanner demo-ao` writes the procedural AO as real
  GeoTIFFs + mission YAML, so `plan` runs the true ingestion path (and rehearses
  real DTM/DSM ingest).
- **`gz-world` injects into PX4's `default.sdf`; never hand-write a world.** A
  from-scratch world loses gz-sim's default plugin set (loaded only if the world
  declares *no* `<plugin>`) and PX4's `<magnetic_field>` — sensors die and the EKF
  yaw diverges. `<world name>` must equal the .sdf file stem or PX4 hangs on
  "Waiting for Gazebo world...".
- **`gz-world` hands Gazebo a ZERO-BASED DEM (2026-07-23 — don't regress).**
  Gazebo uses a DEM's **raw elevations**, not a rescale into `<size>`'s z. With
  the raw DTM the terrain surface under home landed at world `z = min_elev`
  (+40 m on the demo AO, +125 m with `--flat`) while PX4 spawns at `z = 0` — the
  vehicle spawned **buried**, which reads as a stable hover and then tumbles on
  translation. That was the long-standing terrain-world bug. `normalise_dem()`
  now zero-bases the elevations so raw-vs-rescaled interpretations coincide, and
  the command writes `<out>_dem.tif` next to the world — **both files must reach
  the sim machine**, and `--dem-path` must match where the DEM will live there.
  Guarded by `surface_z_at_home() == 0`. **Verified on the VM 2026-07-24** — the
  vehicle rests on the terrain (roll matches the DEM gradient at home to 2 %) and
  flew 51/77 waypoints over real relief (RUNBOOK §2b). The only tracked world is
  `worlds/planner_terrain.sdf` + `worlds/planner_terrain_dem.tif`; the pre-fix
  `demo_ao/worlds/` copies were **deleted 2026-07-27** (they pointed at raw DEMs
  and would bury the vehicle — never track a world whose `<uri>` names a raw DTM).
- **Clearance is scored against the profile PX4 FLIES, not linear interpolation
  (2026-07-27 — don't regress).** PX4 front-loads a leg's descent at its own
  vertical rate instead of spreading it along the ground track, so a plan clear at
  every vertex *and* along every linearly-interpolated leg can still fly into a
  falling slope — it did, in SITL, at 0.8 m AGL (SHORTFALLS "Recently closed").
  `planner/terrain_profile.py` (a) re-scores the final altitudes against real
  terrain under the flown profile — **the guarantee** — and (b) splits only the
  legs that check rejects, placing each insert where the level-flight run before
  arrival is longest. Demo AO: **−6.6 m → +9.0 m** min AGL for **1 extra vertex**.
  A breach is CRITICAL → NO-GO and breaks `feasible`. **Climb is modelled as
  linear on purpose** (prompt climbing only adds clearance and is unvalidated —
  assuming it would make the check optimistic).
  🔴 **Never split on a drop threshold instead of the check.** v1 did, split a leg
  already clear at +9.0 m, and that one gratuitous vertex **froze a SITL mission
  at 11/79** — PX4 missed it by 1.58 m vertically (`NAV_MC_ALT_RAD` is 0.8 m),
  never sequenced past it, and flew into the ground. A big drop is not unsafe by
  itself: PX4 flew that leg's full 27.8 m drop *unsplit* to 0.22 m. Every inserted
  vertex is a waypoint PX4 must accept vertically — insert as few as possible.
- **The drone flies UNDER the canopy, as low as the constraints allow
  (2026-07-27 — don't regress).** It used to hold a fixed 30 m above *bare
  terrain*, which in the demo's 12–22 m stands is 8–18 m over the crowns: the
  cost surface rewarded forest as cover while the aircraft was silhouetted above
  it. `fields/altitude.py` derives the band from the canopy height model
  (`DSM - DTM`, no new data needed): target `cruise_agl_m`, floored by
  `AltitudePolicy.min_agl_m`, ceilinged at `canopy_top − canopy_clearance_m`.
  **Under the crowns with clearance, or over them with clearance — never inside**
  (the over-canopy case must raise the *floor*, or the altitude smoother will
  park the drone in the crowns). Measured on the demo AO: drone exposure
  **0.55 → 0.15**, exposed route 1998 m → 611 m, 2654 m of 5299 m flown below
  canopy — and the drone ends up *less* exposed than its own cable (0.15 vs
  0.27), which is the whole point.
  **`waypoint_spacing_m` is what makes low flight possible** and is now honoured
  by the corridor planner (it was read only by the M0 stub). PX4 drives toward
  the next waypoint's altitude at its own rate, so between distant waypoints the
  commanded profile cannot track the ground: at 100 m spacing a 10 m AGL route
  flies **12 m underground**, at 25 m it clears by 6.7 m. The residual loss is
  terrain bulging *between* waypoints, so the useful floor on spacing is the
  DTM's cell size — **the achievable altitude is set by DEM resolution, not by
  the autopilot** (capping the descent rate 2.0 → 0.5 m/s buys 0.37 m).
  🔴 **A below-canopy plan is not verified and must not pretend to be.** The
  planner sees a canopy *top* and nothing beneath it. Per the user (2026-07-27)
  obstacle clearance is the flight stack's job (camera/lidar) and the route is a
  corridor to follow approximately — so the report raises `below_canopy_flight`
  with the length on every such route. That capability does not exist yet on this
  hardware (Track B needs x86+RTX).
- **The viewshed scores the DRONE at flight altitude, not the ground under it
  (2026-07-27 — don't regress).** `fields/viewshed.py` takes a `target_elev_m`
  raster (absolute elevation of the object being looked at) while occlusion still
  marches over the DSM, and `fields/composite.py` builds **two** fields:
  `exposure` = the drone at `dtm + planned flight AGL`, `exposure_ground` = the
  surface the **cable** is laid on (which feeds `build_survivability`). They are
  different objects at different heights — do not re-merge them. Scoring the
  drone at ground level understated its exposure on the demo AO by **3.2×**
  (0.219 reported vs 0.7455 actual) and drove the route with the wrong field.
  The flight altitude comes from `forest.navigability.planned_flight_agl`, the
  **vectorized twin of `altitude_band_at`** that the corridor planner flies —
  keep them one definition (`test_planned_flight_agl_matches_the_per_cell_band_everywhere`),
  or the field silently scores an altitude the drone never occupies.
  `Plan.objective_scores` now carries `mean_exposure` (drone),
  `mean_exposure_cable`, and `mean_exposure_flown` (line-of-sight along the
  *flown* 3D trajectory, sharing `sample_flown_profile` with the clearance
  guarantee); the report separates `exposed_cable` from `exposed_drone`.
- **The plan carries the lateral freedom it grants, and concealment is scored
  over it (2026-07-28 — don't regress).** The flight stack follows the route
  *approximately* (obstacle avoidance is onboard, user 2026-07-27), so scoring
  concealment on the centreline described a line the aircraft will not fly.
  `planner/corridor_width.py` gives every vertex a `corridor_half_width_m` and
  re-scores exposure over the swept area. **The width is not a constant** — it is
  the distance to the nearest cell that would lose the concealment that vertex
  has, under three clauses: never enter ground *more exposed* than here
  (relative, not an absolute threshold — an absolute one lets a corridor around
  an unobserved vertex reach into watched ground); **never leave the stand** you
  are hiding inside, even into unobserved open ground (user's requirement — the
  corridor must not expand outside a forest patch); never enter an obstacle. Demo
  AO: median 50 m (capped), **32/282 vertices get 0 m** where the route runs at
  the edge of its cover, and corridor exposure is **0.164** vs 0.150 on the line.
  ⚠️ Widths come from a distance transform, so they quantise to the cell size —
  25 m on the demo AO. And the radius stops **one cell short** of the offending
  cell, or a corridor of exactly that radius would reach it.
- **CRS discipline:** projected metric CRS only; geographic rejected on load.
- **Determinism:** same mission+seed ⇒ byte-identical output (golden test).
- **Human-in-the-loop:** report caps at REVIEW (never blind GO) until analysis is
  complete; a fully-analyzed, low-risk plan can reach GO.
- **Forest = high-value cover** (user): FLYABLE forest is *rewarded* in the cost
  surface (concealment), MARGINAL penalized, NON_NAVIGABLE is an obstacle.
- **Altitude is a free variable** (user): the drone varies height within the
  navigable band (not "fly high"); kinematic smoother keeps climb feasible.
- **Reversals allowed** (user): low payout speed mitigates snag → reversal is an
  INFO finding, not NO-GO; only turn-radius/climb violations break feasibility.
- **Ingested today:** DTM, DSM, threat observers, weights, lidar point cloud,
  roads. **Not yet:** land cover (snag/veg terms are 0), threat *sectors*.

## What ingests what (cost surface)
`fields/composite.py` blends: base move cost + drone exposure (viewshed over the
DSM, evaluated at flight altitude) + survivability (which carries the *ground*
viewshed, for the cable) + snag + road penalty + forest term (reward/penalty) −
cover (drainage defilade). Non-navigable forest → Theta\* obstacle.
`planner/corridor.py` runs the search then the kinematic pass.

## Top gaps (SHORTFALLS.md, before sims)
1. 🔴 **The new low route is unflown.** 6.6 m min AGL (was 24) and 282 waypoints
   (was 145). PX4's vertical acceptance stalled two missions in the CFIT campaign
   at 87 waypoints and 30 m; nothing about this configuration is verified.
2. 🔴 **The M2 navigable band is not AGL** — it is height above the point
   cloud's global min z, added to the local DTM as if it were AGL (SHORTFALLS
   1b). Identical forest over 0/30/70 m ground reports `nav_floor` 26/56/3 m.
   Masked today because no real lidar has been run; it bites the first one, and
   a voxel window *overrides* the canopy-derived altitude band, so it bounds the
   below-canopy posture too. (Without a point cloud, the canopy path is fine —
   `DSM - DTM` is terrain-relative by construction.)
3. 🔴 **Below-canopy flight assumes onboard obstacle avoidance that does not
   exist yet** on this hardware (Track B, x86+RTX). The report says so per route.
4. Forest voxelized at corridor grid res, not 0.25–0.5 m (flyability optimistic).
5. Performance won't scale to a real 5 km AO (pure-Python viewshed/Theta\*).
Plus land cover + threat sectors unwired; OMPL 3D refinement deferred; no real-AO
validation yet.

## Layout
`model/` pydantic contracts + `Grid`. `io/` readers/writers (lazy GDAL/laspy).
`fields/` cost layers (incl. `altitude.py`, the below-canopy band). `forest/`
flyability. `planner/` Theta\* + corridor + kinematics + `terrain_profile.py`
(CFIT) + `corridor_width.py` (lateral freedom).
`crossings/` roads. `report/` GO/REVIEW/NO-GO. `demo.py` procedural AO.
`viz.py`/`serve.py` visualization. `pipeline.py` orchestration. `cli.py`.
