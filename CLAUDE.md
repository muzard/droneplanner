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
                        # add --forest-dsm demo_ao/dsm.tif for TREES (TEST-PLAN B2):
                        #   static trunk cylinders + a <out>_trunks.csv manifest that
                        #   droneflight's perception oracle senses from.
                        # ⚠️ --base-world must be PX4's default.sdf, never an already-
                        #   generated world (terrain is APPENDED -> injected twice). On
                        #   the Mac there is no PX4 checkout: use ../sim-logs/default.sdf.
                        # ⚠️ --dem-path is the path on the VM, not here.
                        # ⚠️ Keep --max-stems loadable: 1166 and 2500 load, 4000 hangs
                        #   Gazebo. ONE MODEL PER TRUNK is required for collisions to
                        #   work at all — see io/forest_models.py.
droneplanner viz   --out route.png     # demo AO: fields + flyability + route (static PNG)
droneplanner serve                     # interactive explorer, http://127.0.0.1:8765
ruff check . && ruff format --check . && mypy --strict src && pytest -q   # 380 tests, ~95% cov
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
- **Visibility is a WEIGHT, and a stand can hide what flies under it
  (2026-08-11, roadmap C1 — don't regress).** `single_viewshed` excluded the
  target cell's own DSM from its occluders — correct above the surface, wrong
  beneath it — so from a 300 m sensor **100 % of the 3722 m flown below canopy
  scored visible** while the aircraft was under the crowns. The planner could not
  represent sub-canopy concealment against a look-down observer *at all*, which
  is the one geometry the whole posture exists for. It now returns **float32
  weights in [0, 1]**; `ThreatModel.canopy_occlusion` supplies the crown gap
  fraction (default **0.10**) and the 10° steepness threshold, and `point_exposure`
  applies the identical rule so the flown re-score cannot disagree with the raster.
  `single_viewshed_bool` keeps the boolean contract the A4 reference test needs.
  🔑 **Three conditions, and the middle one is load-bearing:** target under its own
  DSM, **observer above that DSM**, sightline steeper than the threshold. Drop the
  middle one and the model also attenuates trunk-space-to-trunk-space sightlines,
  where nothing is in the way — that *invents* concealment, the one direction a
  concealment planner must never err in.
- **Exposure is a DETECTION probability, not "a sightline exists" (2026-08-11,
  roadmap C2 — don't regress).** The viewshed had **no range term at all**: a cell
  3 km from an observer scored exactly like one at 300 m, while the median exposed
  sample on every study arm sat at **1.3–1.6 arcmin** against the **3–7′** where
  observers actually acquired real sUAS in trials — and against sky, not forest.
  `ThreatModel.detection` (`DetectionModel`) turns an acquisition angle x an optic
  magnification into a **slant** range: **1031 m** for the 0.9 m airframe unaided
  at 3′, full weight to half of that, linear ramp to zero. It **multiplies** with
  C1's crown weight, so occlusion x canopy x range compose.
  🔑 **On the demo AO this takes unaided exposure to exactly 0.000 for every
  flight band** (was 0.123 below canopy / 0.319 canopy top / 0.512 flat 30) —
  the route simply stays outside 1031 m of both observers, and against a dismount
  at these ranges *the flight band stops mattering*. **A 12× optic restores the
  entire pre-C2 picture to three decimals**, so the live question is whether the
  adversary has magnified optics on that bearing — i.e. `threat.sectors` (C3),
  now the highest-value unbuilt item.
  ⚠️ Beyond the acquisition range the term is flat zero, so concealment stops being
  a graded field and becomes a **keep-out radius** the planner is indifferent
  outside of. Fine while a zero-exposure route exists; not a general property.
  ⚠️ **Visual acquisition only** — not radar, not thermal, and not acoustic, where
  ranges are 300–500 m and *independent of flight band* (C8 unbuilt).
- **`exposure_ground` → `ground_los`, and it gets no detection model (C2's
  explicit decision).** The laid fiber is 0.25–0.28 mm and is **never optically
  resolvable at any range**, so a visual model sized for it would zero the field
  and delete cable survivability from the cost surface. It stays a pure
  line-of-sight openness proxy and was **renamed so it cannot be read in the same
  currency as `exposure`** now that `exposure` is a probability.
  `objective_scores.mean_exposure_cable` → `mean_cable_los` likewise.
- **`threat.observer_height_m` is a mission input (2026-08-11 — don't regress).**
  `build_cost_surface` hard-coded a 2 m eye, so the planner **could not route
  against a mast or an airborne sensor**, and C1 was measurably inert in the
  pipeline: the gap fraction changed the demo route by *exactly nothing*, because
  a dismount never looks down into a stand. Pinned by
  `test_canopy_occlusion_reaches_the_cost_surface`. 🔴 Note the eye sits on the
  **DSM**, so a "2 m dismount" in a 22 m stand is modelled at 24 m — in the
  treetops (SHORTFALLS).
- **The demo AO has room behind the start for a threat sensor (2026-08-11,
  roadmap C0).** The start sat ~400 m from its own SW corner, so every "receding
  threat" number was measured against a sensor that could see the CP directly.
  `demo-ao --threat-margin` (default **2500 m**) pads the raster south and west —
  the reverse-of-route bearing — as analytic continuation of the same surface, so
  start, end, observers and the scene's terrain are **bit-identical** and the route
  barely moves (5299 → 5316 m). Standoff 407 → **2528 m** for **0.09 s** of extra
  plan time. ⚠️ It is not perfectly route-neutral: `fields/terrain` and
  `fields/survivability` normalize over the whole raster, so extending the extent
  rescales those layers everywhere. Endpoints were deliberately *not* moved inward
  (the roadmap's suggested alternative) — that shortens the route 5.3 → ~3.1 km and
  would silently close the N1 endurance NO-GO by shrinking the mission.
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
- **A plan now has to fit the battery, and the concealed one does not
  (2026-08-03 — TEST-PLAN N1/N3).** `planner/endurance.py` turns route length,
  waypoint count, altitude and speed into a flight time and raises
  `endurance_exceeded` **CRITICAL → NO-GO**, in the same shape as the CFIT
  clearance guarantee. Calibrated on B1's ULog: flight time is **not**
  `length/cruise` — that is **7.6 % low** (883 s vs a flown **956.0 s**), because
  PX4 loses **0.161 s per waypoint** decelerating into each acceptance radius
  (**paid 282 times** at 25 m spacing) plus **19.1 s** at the route ends. The model
  reproduces the flight to 948.3 s. ⚠️ Both constants were *fitted to that same
  flight*, so it is a calibration, not a validation; N2's speed sweep is its first
  independent test. Corner deceleration is deliberately omitted — the demo's
  sharpest vertex is **6.7°**, where PX4's own cap is still ~6 m/s.
  `fields/stem_density.py` supplies the other half: canopy height → stem density
  (self-thinning, `N ~ H^-1.5`) → mean spacing → **local speed cap** → seconds per
  metre, wired into `Weights.clutter` and every vertex's `speed_hint_ms`.
  🔴 **The two constraints have crossed.** Endurance floors cruise at **4.00 m/s**
  on a 30 min airframe; B2's forest ceiling is **3.5 m/s** at the density actually
  flown. The demo AO's stands are **520–1290 stems/ha** (cap **1.07–1.68 m/s**) and
  **0 of 19 272 treed cells** clear the floor — clearing it needs ≤92 stems/ha,
  i.e. a ~70 m canopy. So the concealed route is **NO-GO at 57.1 min against 30**,
  and routing between sparser stands cannot fix it.
  **`weights.clutter` defaults to 0.0 and must stay there:** measuring the clutter
  is unconditional (the cap always feeds the budget), but any nonzero weight drives
  the route out of the forest — 57.1 → 16.3 min for **2.7× drone exposure**
  (0.150 → 0.403). Trading concealment away is the operator's call, not a default.
  ⚠️ The density relation is **forestry literature, not a measurement** — no stem
  inventory has ever been ingested. Trust the ordering (taller = sparser), not the
  absolute stems/ha; a `stem_speed_capped` WARNING says so beside the verdict.
- **Threat sectors are implemented, and they buy concealment on credit
  (2026-08-15, roadmap C3 — don't regress).** `ThreatModel.sectors` was in the
  contract from M0 and **read by nothing**, so "the enemy is that way and we are
  flying away from them" could not be said to the planner — which is why the demo
  AO ended up with observers *flanking* the route. `fields/sectors.py` implements
  both halves the spec names: a sector **with** an observer masks that observer's
  viewshed to its arc (reducing the drone `exposure` *and* the cable `ground_los`),
  a sector **without** one becomes `openness_to_threat_sectors`, the terrain-
  masking term `build_survivability` has always listed as `w1` and never had.
  Compass azimuths, swept **clockwise**; a degenerate arc (start == end) is the
  **full circle**, never a zero-width one — the safe reading of an ambiguous input
  is the one that cannot silently erase a threat. Same reason a sector naming an
  observer that is *not* in `threat.observers` **adds** that observer instead of
  being dropped.
  🔑 **The mask is applied to the finished viewshed, never to the march** —
  occlusion is geometry and does not care where anyone is looking, so terrain
  outside the arc must still raise the running horizon for cells inside it.
  Exposure aggregates as **max over an observer's elements, mean over observers**:
  one observer holding two optics is not two threats. `point_exposure` runs the
  identical bearing test so the flown re-score cannot disagree with the raster.
  **Per-sector `detection` override** is the input C2 left dangling: with unaided
  exposure at 0.000 for every band on this AO and 12× restoring the whole pre-C2
  picture, the live question is not how high to fly but whether a *magnified optic
  searches that bearing*. Now expressible per arc.
  🔴 **A sector is an intelligence assumption that removes cost from the surface,
  and the planner spends the slack.** Demo AO at 12×, 90° arc: `exp_flown`
  **0.123 → 0.056** below canopy, **0.512 → 0.211** at flat 30 m, route 5316 →
  5209 m and **240 m less flown below canopy**. So the aircraft is *less concealed
  in absolute terms* than the all-round plan and relies on the adversary facing
  where the mission says. The report carries every sector by label
  (`threat_sectors`) plus a `threat_sectors_applied` INFO; supplying sectors on a
  path that cannot use them raises `threat_sectors_ignored` **WARNING** rather
  than dropping them silently.
  ⚠️ Bearing-only sectors have **no sensor**, so masking is scored against a
  threat assumed to be *at the horizon* within `horizon_m` (default 2000 m). That
  assumption moves the answer — fully-masked AO fraction **19.1 % → 28.0 %**
  between a 500 m and a 2000 m horizon. ⚠️ Sectors are **azimuth only**: no
  elevation extent, so a 300 m sensor searching a 90° arc searches it to zenith.
- **Heights derived from a voxel grid are referred to each column's ground
  (2026-09-14 — don't regress).** A voxel level index counts from `VoxelGrid.z0`,
  the point cloud's *global minimum* elevation, but `planner/corridor.py` and
  `fields/altitude.py` add the band to the **local DTM** as if it were AGL.
  `VoxelGrid.ground_level(dtm)` supplies the per-column datum; `segment_vertical`
  and `build_flyability` take it and refer the understory band, the free-window
  search and every reported height to it. 🔑 **Free-window level indices stay
  ABSOLUTE on purpose** — the 3D EDT clearance grid is indexed in true geometry,
  and height-normalizing the occupancy itself would shear the space and corrupt
  horizontal clearance between columns at different elevations. Pinned by three
  tests in `tests/unit/test_forest.py`; the damning one is that a column standing
  above `z0` used to find the **void beneath its own ground** and report it as
  navigable.
- **The climb check is AGL; the airframe limit is not (2026-09-14).**
  `smooth_altitudes` constrains the AGL profile, so terrain-following motion is
  invisible to `climb_violations`. `kinematics.amsl_vertical_rates` measures the
  true rate from `ground_elev_m + altitude_agl_m`, `KinematicReport` carries it,
  and `vertical_rate_amsl_exceeded` WARNS when it beats the airframe. On the demo
  route that is **2.56 m/s descent against a 2.0 m/s limit with zero violations
  reported**. **Deliberately not enforced** — constraining it changes routes and
  trades against the below-canopy posture, so it follows `weights.clutter`:
  measure unconditionally, let the operator trade.
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
- **Ingested today:** DTM, DSM, threat observers, threat *sectors*, weights,
  lidar point cloud, roads. **Not yet:** land cover (snag/veg terms are 0).

## What ingests what (cost surface)
`fields/composite.py` blends: base move cost + drone exposure (viewshed over the
DSM, evaluated at flight altitude) + survivability (which carries the *ground*
viewshed, for the cable) + snag + road penalty + forest term (reward/penalty) −
cover (drainage defilade). Non-navigable forest → Theta\* obstacle.
`planner/corridor.py` runs the search then the kinematic pass.

## ✅ MVP MISSION + ONE-SORTIE ENDURANCE (2026-09-23) — decisions, and why
Working autonomously toward the MVP under two user rules: **assume no lidar unless
absolutely necessary**, and **measure-and-decide rather than ask** (both in
memory). Four changes, each with the choice recorded here so it can be overturned
on evidence.

**1. The speed cap is now the ONBOARD PLANNER's measured envelope, not reactive
coherence.** `fields/stem_density.ONBOARD_PLANNING_ENVELOPE`:
**107 stems/ha → 6.0 m/s, 200 → 3.0, 400 → 2.0** (⚠️ re-measured 2026-09-24 after
the planner fixes and anchored to SITL: 200 → 3.0, 400 → 2.0, 800 → 1.25, 1200 →
0.4 — see "HOW MUCH THE ZONE MUST SHRINK"), measured in droneflight's
accel/jerk-limited harness with the new local planner (`droneflight/local_planner.py`).
At 800/ha it stalls or strikes at every speed tried, so anything past 400 is
flagged `beyond_envelope` and raises **`stem_density_beyond_envelope` CRITICAL**;
the speed there is an extrapolation (∝ 1/density) kept only so the cost surface
still prefers the sparser of two unflyable stands. The old
`max_reactive_speed_ms` stays in `components` for comparison; `corridor.py` reads
`max_speed_ms`. Also fixed: the cap used to apply to cells flown **over** the
canopy — it now applies only where the band is below canopy.

**2. "Only the first part needs cover" is `Mission.covered_radius_m`.** Radial from
the start (a property of the ground, fixed before any route exists). Inside:
the old behaviour. Outside: the drone flies **over** the canopy, drone exposure and
the cover reward are zeroed, no stem speed cap. **The cable's survivability term
stays everywhere** — the fiber remains on the ground after the aircraft has gone.
None = whole route (unchanged default).

**3. DECISION: 1500 m** (re-checked 2026-09-24: closes up to ~400 stems/ha; denser Finnish stands need 0.8-1.1 km). `tools/concealment_zone_study.py` sweeps radius × density:

| stand density | largest radius that closes | energy at 1500 m |
|---|---|---|
| 107/ha | all (even the whole route, 0.67) | 0.67 |
| 200/ha | all (whole route exactly 1.00) | 0.82 |
| **400/ha** (top of demonstrated envelope) | **1500 m** | **0.96** |
| canopy-height inference (520–1290/ha) | 500 m only (i.e. no cover) | beyond envelope |

1500 m is the largest zone that closes on the 30 min airframe at **every density
the onboard planner has been shown to fly**; it gives ~1.2 km of below-canopy
flight on the demo route. **Flown GPS-denied in SITL at 200/ha: 0 strikes, 18.6 min
(planned 19.0)** — droneflight RUNBOOK §4.14.
⚠️ Its binding case, 400/ha at 2.0 m/s, is **marginal GPS-denied** in the harness
(1 graze of −0.016 m in 6 seeds; clean on truth). If that edge does not hold, the
same study says the zone must shrink toward 1000 m at 400/ha. ⚠️ Overturn it with a real stand density: at 107/ha the
whole route could be covered. And note what this AO does NOT constrain: with the
default unaided ground observers `exp_flown` is **0.000 at every radius** (C2), so
on this fixture the zone is set by endurance alone.

**4. `stem.density_per_ha` overrides the canopy-height density inference.** The
literature relation says the demo stands are 520–1290/ha while every Gazebo forest
is 107–200/ha, so a SITL mission must state the density of the world it flies or
its endurance is priced for a different forest.

**5. The `.plan` finally carries speed.** `io/mavlink.py` emits
`MAV_CMD_DO_CHANGE_SPEED` (178) wherever the leg speed (slower end of the leg,
floored to 0.5 m/s) changes. Until now the endurance verdict assumed forest speeds
the vehicle was never told — the 2026-09-14 MVP flew 6.0 m/s everywhere.
`droneflight`'s loader reads them back (`Waypoint.speed_ms`, `leg_speed_ms`).
🐛 Found while doing it: `gz-world` read route points from **any** item with
params[4:6], so a speed item would have added a route point at (0, 0) and drawn
the forest band along a line to the equator. It now reads waypoints only.

**The MVP mission is `demo_ao/mission_mvp.yaml`** (radius 1500, density 200):
**5180 m, 1166 m below canopy at 3.0 m/s, 19.0 min, energy 0.82 → REVIEW (was
NO-GO at 47.6 min)** — 3.0 m/s flown clean GPS-denied on 4/4 noise-seed sets. Its world: `gz-world --stems-per-ha 200 --forest-radius
1500` → `worlds/forest_mvp.*` (1562 trunks over 7.8 ha, median NN 3.4 m).
⚠️ `demo_ao/dtm.tif` is 220×220 since the AO was enlarged; the older worlds were
built from the 160×160 version. Terrain in the overlap is identical (a constant
7.2 m zero-basing offset the world pose absorbs), so both are valid.

## 📏 HOW MUCH THE ZONE MUST SHRINK IN DENSER FOREST (2026-09-24)
User: *"Calculate how much the zone has to shrink if the forest is denser."*
`tools/concealment_zone_study.py --solve --mission mission_mvp.yaml` steps the
covered radius in 100 m and reports, per stand density, the largest radius below
which every step still closes one sortie (30 min airframe, 20 % reserve), on the
**SITL-anchored** envelope (`ONBOARD_PLANNING_ENVELOPE`: 200 → 3.0 m/s and 400 →
2.0 as flown over four noise-seed sets; 500-600 → 1.5, 800 → 1.25, 1000 → 0.75,
1200 → 0.4 discounted from the harness):

| stems/ha | largest zone that closes | energy with the 1500 m zone |
|---|---|---|
| 200 | the whole route | 0.82 |
| 300 | 3700 m | 0.88 |
| **400** | **1800 m** | 0.96 |
| 500-600 | **1100 m** | 1.11 — NO-GO |
| 700 | 1000 m | 1.16 — NO-GO |
| 800 | 900 m | 1.23 — NO-GO |
| 900 | **800 m** | 1.41 — NO-GO |
| ≥ 1000 | 700 m | NO-GO |

🔑 **For Finnish managed stands (~400-900 stems/ha under the canopy) the zone is
1.8 km at the sparse end and ~0.8-1.1 km through most of the range.** The 1500 m
MVP decision holds up to ~400/ha. **On a real Finnish AO use `layers.stand_data`
and re-solve**: stands are then priced at their own density.
⚠️ **Why SITL-anchored:** the harness (`HARNESS_ENVELOPE`) flies 200/ha at 5 m/s
clean; GPS-denied SITL on the final stack is clean at 3 m/s on 4/4 seed sets and
touches at 4 m/s (200/ha) and 3 m/s (400/ha). Raising an anchor = a four-seed
SITL sweep (`~/sweep.sh`), each confirmed step enlarging the zone.
✅ Four-seed SITL sweeps on the final code: 400/ha at 2 m/s clean 4/4 (min air
+0.28 m); **600/ha at 1.5 m/s clean 4/4** (world `forest_mvp600n`, 1725 trunks,
±15 m band) — so the 500-600 anchor is measured, not discounted. ⚠️ Its thinnest
pass was **+0.03 m of air**: at 600/ha (median gap 1.9 m) the 0.9 m airframe is
near what can be threaded at all; 2 m/s was also strike-free (min +0.12 m) and
is not adopted on margins that thin. Askola's median flyable stand is 640/ha.

## 🇫🇮 FIRST REAL AO — ASKOLA, FROM OPEN DATA (2026-09-25)
User: *"do that"* (build a real Finnish AO). `tools/build_finnish_ao.py` builds an AO
from three open sources (all CC BY 4.0, downloads approved; files under `data/`,
gitignored): **MML 10 m terrain** (read for the window from the Funet/CSC mirror's
VRT — no registration), **Metsäkeskus Latvusmalli 1 m canopy height** (p95 per 16 m
cell = canopy top), and **Metsäkeskus stand polygons** as `layers.stand_data`.
AO: 4.5 × 4.5 km on Askola's west side (281² cells of 16 m, terrain 29-84 m, canopy
≥ 12 m on 59 % of cells), 5.5 km route SW → NE, two observers 1.5 km off one side.
Mission: `data/askola_ao/mission.yaml`; run the zone study with `--solve --as-is`
(radius only, the AO's own densities).

| `weights.clutter` | largest zone that closes | below-canopy flight at it | 1500 m zone |
|---|---|---|---|
| 0 (default: pure concealment) | **600 m** (0.98) | ~620 m | 1.50 — NO-GO |
| 0.5 | 1200 m (0.98) | ~590 m | 1.03 — NO-GO |
| 1.0 | 1200 m (0.99) | ~580 m | 1.04 — NO-GO |

🔑 **On a real Finnish AO one battery buys ~550-600 m of below-canopy flight**,
however the zone is drawn: with `clutter` on, the planner simply routes round the
densest stands (over the canopy / in the open) and the zone grows while the
covered flight does not. The 1500 m MVP zone does not close here.
⚠️ The first ~900 m of forest by the start is **not in the stand data** (outside
private-forest coverage), so it is priced from the canopy relation (now fitted to
Askola) — and real canopy relief costs vertical work: even a 100 m zone uses 0.78
of usable energy (`vertical_rate_amsl_exceeded`: 4.2 / 5.0 m/s true vertical).
⚠️ With the default unaided 2 m ground observers 1.5 km away, exposure is 0
everywhere (C2 detection range ~1 km), so this AO does not test concealment — only
endurance and stems. Harness/SITL anchors as in "HOW MUCH THE ZONE MUST SHRINK".

### ✅ Askola 600 m zone FLOWN in SITL — GPS-denied, **4/4 CLEAN** (2026-09-25)
`askola600` world (real MML terrain, 442 trunks at the AO's density — *inferred*
in the zone, see below), `out/askola600/mission.plan` (314 wp, forest legs
1.0-1.5 m/s), `fly-local` + EV bridge, four noise-seed sets (`sim-logs/askola/`):

| seeds | `b2-score` | min trunk-surface clearance | time | max nav error |
|---|---|---|---|---|
| 4/5 | 0 strikes, 0 ground | +0.728 m | 23.4 min | 11.2 m |
| 0/1 | 0 / 0 | +0.732 m | 23.3 min | 14.1 m |
| 2/3 | 0 / 0 | +0.763 m | 23.2 min | 14.0 m |
| 6/7 | 0 / 0 | +0.765 m | 23.3 min | 17.1 m |

Planned 22.4 min, harness 23.1 min — the endurance model holds on real relief to
~4 %. **The first real-terrain Finnish sortie: one battery, GPS-denied, no lidar,
0 strikes.** It tests terrain + trunks at 1.0-1.5 m/s; it does not test measured
density or crowns (next section).

### 🔴 Askola, MEASURED zone + crowns (2026-09-25, later the same day)
**1. The 600 m result above was priced on inference, not measurement.** Within
600 m of the original start, canopy ≥ 14 m covers 92 % of cells and the stand
layer covers **0 %** (`below_canopy_density_measured_m` = 0.0 in its report) —
Metsävarakuviot is mainly *private* forest, so state/company blocks are simply
absent. Over the whole AO only 74 % of tall canopy is covered. So on a real AO,
"measured density" must be checked per route, not assumed from `layers.stand_data`.
**2. On measured stands the zone shrinks by half.** `mission_measured.yaml` moves
the start into the south-centre block (~80 % of tall canopy surveyed; measured
median **746 stems/ha** where tall; the zone's own stands ~1000-1200). Both
observers stay west of the new line, 1.8-2.3 km off it.

| `weights.clutter` | largest zone that closes, **in envelope** | below canopy |
|---|---|---|
| 0 | **300 m** (0.77) | ~280-300 m |
| 0.5 | 1500 m (0.74) | plateaus at **~450 m** — routes round the densest stands |

One battery buys **~300-450 m of below-canopy flight on measured stands**, against
~550-600 m on the inferred ones. `m300` flies its forest legs at **0.5 m/s**
(1000-1200 stems/ha — the extrapolated end of the envelope). Harness 2/2 clean
(+0.49 m); SITL queued (`askolam300` world, 1702 trunks).
🐛 **Solver defect fixed:** `--solve --as-is` checked energy only, and returned
400 m for a plan the planner itself calls NO-GO (32 m beyond the envelope,
`stem_density_beyond_envelope` CRITICAL). A radius now closes only if it is
also in envelope. `--workers` / `--max-radius` keep the Mac quiet.

**3. The "below canopy" band was inside the crowns.** Metsäkeskus strata give
species per stand: Askola's fly-under stands are **49 % spruce, 26 % broadleaf,
21 % pine by basal area**; along `m300`'s below-canopy stretch 57 / 18 / 24 % by
stems, canopy median **15.4 m**, flown at **10.5-12 m AGL**. Every one of those
trees has live crown at that height unless its crown ratio is below ~0.25
(managed pine ~0.35-0.5, spruce ~0.55-0.8, birch ~0.4-0.55 — the ranges behind
this estimate; Hynynen 1995, Can. J. For. Res. 25:57, models it for pine). So the
aircraft was planned through the **crown layer** — branches metres wide, where
the Gazebo world has bare 0.15 m cylinders. `min_canopy_m` was always a stand-in
for crown base (its own docstring said so).
➡️ **`AltitudePolicy.crown_ratio`** (opt-in; default None keeps every baseline)
holds the below-canopy ceiling under `height × (1 − crown_ratio) − crown_clearance_m`
in all three places the band is computed (pinned together by test). At the
Askola mix (0.55): zone sizes and energy barely move — the stands are tall enough
to keep *some* trunk space — but the flight drops to **5.1-7.0 m AGL**, onto the
5 m floor, and the terrain guarantee then breaches it (**4.5-4.9 m, NO-GO**) at
25, 16 and 10 m waypoint spacing alike.
🔑 **In spruce-heavy managed forest the trunk space is a ~5-7 m band sitting on the
planner's floor.** Flying it needs a lower floor (understory, DTM error — a risk
call) or better terrain data (MML's 2 m DEM instead of the 10 m one); the
alternative is to accept flight through crowns, which nothing here can simulate.
Measured: **`min_agl_m` 4.0 makes it REVIEW** (0 violations, min 4.02 m, flown at
5.1-6.9 m, energy 0.77 — unchanged). Crowns are not flyable at these densities
anyway: ~1000 stems/ha with ~1.5 m crown radii is ~70 % cover at the crown base.
⚠️ **Understory is unmeasured, not absent.** Storey 2 (*alikasvos*) appears in 92
present-day strata against ~8800 dominant ones; most tall stands are inventoried
by remote sensing (storey 5), which cannot see under the crowns. Where it *is*
recorded it averages **9.4 m and 323 stems/ha** — exactly the 5-7 m trunk-space
band. So the 4 m floor rests on an understory the data is silent about.
❓ Whether `crown_ratio` becomes the default is a baseline-changing call, left
opt-in and set only in the Askola `*_cr*.yaml` variants.

## 🌲 FINNISH MANAGED FOREST + MEASURED STAND DENSITY (2026-09-24)
User: *"look up finnish forest data from a reliable source, and keep in mind that
a large part of it is talousmetsää."* Detail and sources in the
`fields/stem_density.py` docstring; the short version:

* **Most of it is managed.** Luke (2025): 13 % of forest + poorly productive land
  is protected (6 % south, 20 % north); the rest is talousmetsä, thinned to Tapio's
  published targets. VMI11 development classes on production forest land:
  **19 % seedling, 36 % young thinning, 30 % advanced thinning, 13 % mature.**
* **Stems/ha by class** (Tapio *Metsänhoidon suositukset*): first thinning at
  12–17 m dominant height leaves **pine 900–1200, spruce 700–1200, silver birch
  700–800**; later thinnings and mature stands come down to **~400–900**. The
  existing self-thinning anchors (600/ha at 20 m, N ∝ H^-1.5 → 1291 @ 12 m, 838 @
  16 m, 429 @ 25 m) sit inside those ranges — the literature relation holds up.
* 🔑 **The stands tall enough to fly under (~43 % of production forest) are
  ~400–900 stems/ha — at and past the onboard planner's envelope.** Young thinning
  stands are denser and their live crowns reach into the 5–10 m flight band, which
  a trunk-only count flatters. Thinned talousmetsä is *easier* than unmanaged
  forest of the same height — the one favourable fact.
* **`layers.stand_data` — a measured density source now exists** (`io/stand_data.py`).
  Suomen metsäkeskus publishes *Metsävarakuviot*: ~1.4 ha stand polygons from
  airborne-laser inventory, updated twice a week, **free, CC BY 4.0**, GeoPackage
  in EPSG:3067. `stand → treestand (type 2 = grown to today) → treestandsummary.stemcount`
  replaces the canopy-height inference on treed cells; the canopy model still
  decides treed vs open, and `stem.density_per_ha` still wins. Reported as
  `below_canopy_density_measured_m`. ⚠️ Accuracy targets are on height (±2 m),
  diameter (±3 cm) and basal area (±3 m²/ha) on 8 stands in 10 — stem count is
  derived, roughly ±30 %. A planning prior only (user rule: never depended on in
  flight); staleness errs *denser*, since stands are thinned and felled between
  updates.
* ✅ **Run on a real download (Askola, 13 MB zip, `data/metsakeskus/`, gitignored)
  — and the data model had already changed**: `treestand.type` is now
  `treestandclass` (present-day = '2') and summaries are split by `storey`. The
  reader handles both and refuses an unknown one. Trunks at flight height = storeys
  1+3+4; understory left out. Rasterises 7 959 stands in 1.6 s, reprojects fine.
* 📊 **Askola, measured** (present-day, dominant storey): advanced thinning
  **706 stems/ha** (IQR 565-887, 21.8 m), mature **494** (400-580, 25.8 m), young
  thinning 1671 (13 m). Stands ≥ 16 m dominant height — the ones to fly under —
  are **56 % of stand area, median 640 (IQR 506-846, p90 1139)** → a **~1.1 km**
  zone at the median (table above).
* 🔧 **The canopy-height relation was 29 % low** against those stands. Refitted:
  **730 stems/ha at 20 m, exponent 1.36** (was 600 / 1.5 from literature) —
  `fields/stem_density.py`. One municipality; use stand data where it exists.

## ✅ STEM DETECTION FROM LIDAR (2026-09-14) — `forest/stem_detect.py`
`layers.point_cloud` → `pipeline._load_stems` → detected `StemMap` → the stem
router below. A supplied `layers.stem_map` manifest wins over detection, because an
explicit statement of where the stems are should not be overridden by a detector's
opinion.

**Pipeline:** height-normalize against the DTM → slice 1.5–5.0 m AGL → single-
linkage cluster in plan view (union-find over a KD-tree, not an O(n²) linkage) →
reject clusters that are sparse / vertically thin / too wide → **algebraic circle
fit** for centre and radius.

🔑 **The circle fit removes a SYSTEMATIC error.** An arc's centroid sits `2r/π` =
0.64r in front of the axis, toward the scanner (measured 0.096 m for a 0.15 m
trunk, analytic 0.095 m). Every stem is displaced toward wherever it was seen
from, so a centroid map is **shifted, not noisy** — it does not average out.
Fitting took RMSE to **0.017–0.029 m**.
🔑 **Rejecting a merged cluster is the safe direction.** A *missed* stem is an
obstacle the planner ignores; a *merged* one is a phantom centre sitting in the gap
between two real trunks, so the planner routes confidently into a trunk it believes
is clear. Hence `max_radius_m`.
🔴 **Accuracy, not detection rate, is the requirement** — the stem-routed sortie
flew with **0.708 m** of clearance, so a 1–2 m ALS-style position error eats the
whole budget. `match_stems` scores a map against known truth for exactly this.
⚠️ Everything measured so far used `forest/synthetic_scan.py`, a **validation
fixture** (no beam, no traced occlusion, no registration error) — an upper bound
that can falsify a detector but cannot validate one. **No real `.laz` has ever been
ingested; that is the remaining gap** (SHORTFALLS M2).

## ✅ STEM ROUTING IS BUILT (2026-09-14) — the route now threads the trunks
`fields/stems.py` (the map) + `planner/stem_route.py` (the refinement), switched on
by `layers.stem_map` pointing at a `*_trunks.csv` manifest. Measured on the demo
route against the manifests B2 actually flew:

| | dense (2500 stems, 4.6 m spacing) | sparse (1166 stems, 6.8 m) |
|---|---|---|
| min clearance **before** | **−0.033 m** (through a trunk) | −0.083 m |
| sub-clearance samples before | **37** / 10501 | 35 / 10501 |
| min clearance **after** | **+1.387 m** | **+1.769 m** |
| sub-clearance samples after | **0** | **0** |
| route length | **×1.0225** | ×1.0214 |

🔑 **The before-number is the point:** the concealment search runs on 25 m cells and
cannot see a 2.4 m gap, so its centreline was laid straight through a trunk — and
its **37 sub-clearance samples match the 37 strikes B2 flew** at that density.
The +2.25 % cost lands exactly on N4's predicted +1.6–2.9 %.

**How it works, and why it is a separate pass rather than a cost term.** Both are
needed and answer different questions: a coarse term can only say "this cell is
cluttered" (that is `fields/stem_density.py`, and N3 showed density routing cannot
rescue the mission), while threading a *specific* gap is geometry at 0.5 m. The
refinement keeps the coarse route's shape and re-lays the line inside the lateral
freedom A1 already granted — it runs **before** the altitude solve, so clearance,
kinematics and endurance all score the line that is actually emitted. A windowed
A* (a local 0.5 m grid per few vertices, pinned at window boundaries) keeps it to
~1.4 s for the whole route instead of a 42 M-cell global grid.

🔴 **Three bugs the first run exposed, all worth keeping in mind:**
1. **Given A1's median 50 m corridor the A* SHORTCUTS instead of refining** — it
   took the straight chord across every bend, collapsed 287 vertices to 25, lost
   the concealment shape, and sub-clearance samples went **UP** (37 → 52). The
   search gets `max_half_width_m` (8 m) — enough to dodge, not enough to re-plan.
2. **Collinearity-based simplification is not error-bounded.** Dropping a vertex
   within tolerance of its *already thinned* neighbours compounds, and it put the
   simplified line back through the trunks the A* had just avoided. It is now
   Douglas-Peucker against the original, **and the result is re-checked against
   the stems**, falling back to a denser polyline if it does not clear.
3. **A* only tests cell CENTRES** while the vehicle flies the segment between
   them, which can pass closer. Passability is inflated by the cell half-diagonal.

🔴 **`stem_vertices_snapped` = 2 on the demo AO — that is N4's composition failure,
now measured by the product.** Two waypoints sat inside the clearance requirement
of a trunk (N4 found exactly 2 of the 32 zero-freedom vertices with a trunk within
0.86 m) and had to be moved. The concealment search places vertices without seeing
trunks, so **the most concealed vertices are the least flyable** — reported as
`stem_vertex_moved`, never papered over.

⚠️ **Every stem map is SYNTHETIC** and generated in a band around the route planned
*without* it, so a refined route that wandered outside that band would find open
space a real forest would have filled — `stem_mapped_fraction` (88.8 % here) is
reported next to every clearance claim. This shows **the planner can use a stem
map**; it does not show one can be built. That remains SHORTFALLS M2.

## 🟢 NEXT ARCHITECTURE: route between mapped stems (TEST-PLAN N4, 2026-08-11)
Reactive avoidance is dead in the flight stack (four independent negatives). The
surviving option is a planner that routes *through* the stem gaps. Measured on the
real dense manifest it is cheap — **+1.6 % distance at ±25 m corridor, +2.9 % at
3.0 m clearance** — and the forest is **99.9 % free area**, so the room is there.
It converts avoidance into **localisation: know position to ≤2.4 m** (GPS-denied
VTRN currently holds 11.8 m), and it **reopens the 6.0 m/s speed window** N1/N3
closed, because a pre-planned path has nothing to react to.
🔴 **Two planner-side blockers, both measured:**
1. **Composition fails.** Against A1's real per-vertex corridor (32/282 vertices at
   zero freedom) the dense manifest has **no admissible path at any clearance** —
   2 pinch points have a trunk within 0.86 m. **Stem clearance must become a term
   in the cost surface**, not a post-filter, so the centreline is never laid where
   the aircraft cannot pass. The zero-freedom vertices are the concealment-critical
   ones, so this is a trade to price, not a constraint to satisfy.
2. **No stem map, and no way to make one** — see SHORTFALLS M2. Every trunk
   manifest flown so far is *synthetic*.

## Top gaps (SHORTFALLS.md, before sims)
0. 🟢 **The concealed route does not fit the battery — but 3 relay legs do
   (2026-09-14).** 47.6 min against a 30 min airframe as one sortie; `planner/relay.py`
   splits it into **3 balanced legs of 16.1 min at 0.67–0.69 of usable endurance**,
   with both relay points on ground at `ground_los` **0.000**. Same route, so the
   concealment is untouched. Cost is a CONOPS change (ground stops, fiber splices —
   both unstated). 🔴 Relay fixes **endurance only**; below-canopy's avoidance
   blocker is untouched. The other three ways out are still decisions, not fixes.
1. ✅ **The low route is FLOWN — 287/287, 2026-09-14** (`sim-logs/mvp/16_53_04.ulg`,
   RUNBOOK §4.12). 947.8 s, min AGL **7.05 m** against 6.59 predicted, 0 samples
   under 5 m, no stalls. The predicted vertical-acceptance stall did not happen at
   287 waypoints and ~12 m AGL. ⚠️ No trees in that world and the plan commands
   6.0 m/s, so it tests altitude tracking and sequencing only.
2. ✅ **The M2 navigable band is AGL as of 2026-09-14** (was 🔴 SHORTFALLS 1b).
   `VoxelGrid.ground_level(dtm)` gives segmentation and flyability a per-column
   height datum; identical forest over 0/30/70 m ground now reports one band
   instead of three, and a raised column no longer mistakes the void beneath its
   own terrain for the tallest free window. No route changes (no mission has a
   point cloud). 🟠 Still open and now characterised: **which** free window wins
   is an artifact of the voxel grid's vertical extent — open sky above a stand
   runs to `cloud.z.max()`, so low ground prefers flying *over* the canopy.
   Bounding the window at each column's canopy top changes routes → user's call.
3. 🔴 **Below-canopy flight assumes onboard obstacle avoidance that does not
   exist yet** on this hardware (Track B, x86+RTX). The report says so per route.
4. Forest voxelized at corridor grid res, not 0.25–0.5 m (flyability optimistic).
5. Performance won't scale to a real 5 km AO (pure-Python viewshed/Theta\*).
Plus land cover unwired; OMPL 3D refinement deferred; no real-AO validation yet.
(Threat sectors were on this list until 2026-08-15 — see the C3 entry above.)

## Layout
`model/` pydantic contracts + `Grid`. `io/` readers/writers (lazy GDAL/laspy).
`fields/` cost layers (incl. `altitude.py`, the below-canopy band, and
`stem_density.py`, the clutter speed cap). `forest/` flyability. `planner/`
Theta\* + corridor + kinematics + `terrain_profile.py` (CFIT) +
`corridor_width.py` (lateral freedom) + `endurance.py` (does the mission close?)
+ `gates.py` (route reduction — analysis only, deliberately not wired into
`.plan` emission; see TEST-PLAN A2)
+ `relay.py` (split a route into legs one battery can fly — analysis only; the
`.plan` still carries a single sortie, because a relay is a CONOPS decision).
`crossings/` roads. `report/` GO/REVIEW/NO-GO. `demo.py` procedural AO.
`viz.py`/`serve.py` visualization. `pipeline.py` orchestration. `cli.py`.
