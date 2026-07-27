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
ruff check . && ruff format --check . && mypy --strict src && pytest -q   # 142 tests, ~96% cov
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
  `planner/terrain_profile.py` now (a) splits any leg dropping more than
  `cruise_agl_m − 5 m` so the autopilot cannot get ahead of the terrain, and
  (b) re-scores the final altitudes against real terrain under the flown profile —
  (b) is the guarantee, (a) is only the mechanism. Demo AO: **−6.6 m → +11.9 m**
  min AGL for **2 extra vertices**. A breach is CRITICAL → NO-GO and breaks
  `feasible`. **Climb is modelled as linear on purpose** (prompt climbing only
  adds clearance and is unvalidated — assuming it would make the check optimistic).
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
`fields/composite.py` blends: base move cost + exposure (viewshed over DSM) +
survivability + snag + road penalty + forest term (reward/penalty) − cover
(drainage defilade). Non-navigable forest → Theta\* obstacle. `planner/corridor.py`
runs the search then the kinematic pass.

## Top gaps (SHORTFALLS.md, before sims)
1. Viewshed evaluated at ground level, not flight altitude (concealment metric
   systematically off). *User note: the fuller lidar-based-cover fix is a later,
   perception-side thing, not this planner.*
2. Forest voxelized at corridor grid res, not 0.25–0.5 m (flyability optimistic).
3. Performance won't scale to a real 5 km AO (pure-Python viewshed/Theta\*).
Plus land cover + threat sectors unwired; OMPL 3D refinement deferred; no real-AO
validation yet.

## Layout
`model/` pydantic contracts + `Grid`. `io/` readers/writers (lazy GDAL/laspy).
`fields/` cost layers. `forest/` flyability. `planner/` Theta\* + corridor +
kinematics. `crossings/` roads. `report/` GO/REVIEW/NO-GO. `demo.py` procedural AO.
`viz.py`/`serve.py` visualization. `pipeline.py` orchestration. `cli.py`.
