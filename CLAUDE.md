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
droneplanner viz   --out route.png     # demo AO: fields + flyability + route (static PNG)
droneplanner serve                     # interactive explorer, http://127.0.0.1:8765
ruff check . && ruff format --check . && mypy --strict src && pytest -q   # 132 tests, ~96% cov
UPDATE_GOLDEN=1 pytest tests/regression -q   # regenerate golden snapshot deliberately
```

## Key design decisions (don't regress; some came from user feedback)
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
