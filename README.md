# droneplanner

Offline **preflight route planner** for the autonomous fiber-laying drone. It
turns a mission — start, end, map layers, threat model, drone/spool geometry —
into a 3D flight-and-laydown plan plus a **human-in-the-loop risk report**. It is
the only pure-software subsystem of the larger program; the flight/navigation
stack is a separate, hardware-dependent effort (see `../Software-Spec.md` §1, §8).

This repository implements the module specified in **`../Software-Spec.md`** with
algorithmic rationale in **`../Preflight-Planning-Module-Design.md`**.

## Status — milestone M1 (2.5D concealment planner)

`droneplanner plan` runs end-to-end. When a DTM+DSM is supplied it plans a
**concealment- and defilade-aware route**: it builds the field stack (in-house
line-of-sight viewshed, terrain TPI/slope/drainage, cable survivability, snag
proxy), combines them into a weighted composite cost surface, and runs a
**cost-aware any-angle Theta\*** search that bends the route around observed
ridgelines toward cover. Without terrain it falls back to the M0 straight-line
stub. Forest (3D) flyability is M2, so the report still caps at **REVIEW** (never
a blind GO) with a `flyability_not_analyzed` flag.

Try it with **zero data** on the built-in demo terrain:

```bash
droneplanner viz   --out route.png      # static multi-panel figure of the route + fields
droneplanner serve                      # interactive explorer at http://127.0.0.1:8765
```

The explorer lets you drag observers and the start/end, move the objective-weight
sliders, and switch display layers while the route re-plans live — the fastest way
to build intuition for the cost function.

| Milestone | Scope | State |
|---|---|---|
| **M0** | scaffold, data contracts, I/O, CLI, straight-line stub + report | ✅ |
| **M1** | `fields/` (viewshed, terrain, survivability, snag) + cost-aware Theta\* corridor + viz | ✅ this repo |
| M1.1 | road/linear-obstacle crossings to existing culverts/bridges | next |
| M2 | `forest/` voxel flyability + 3D (OMPL) refinement | later |
| M3 | laydown + k-diverse survivability routing | later |
| M4 | PX4 SITL / Isaac Sim validation | later |

> **M1 remaining sub-item:** the `crossings/` solver (route to existing
> culverts/bridges, flag open/overhead-span crossings) is scaffolded but not yet
> implemented — the concealment/defilade route is complete.

## Install

```bash
python -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"      # core + lint/type/test tooling
```

Python **3.11+**. Core geospatial deps (`rasterio`, `geopandas`, `pyproj`,
`shapely`, `scikit-image`, `scipy`, `laspy`) install from wheels. The forest
point-cloud path additionally needs the system PDAL library:
`pip install ".[forest]"` (see `THIRD_PARTY.md`).

## Usage

```bash
droneplanner plan --mission configs/mission.example.yaml --out out/
droneplanner viz  --out route.png            # demo terrain: fields + route figure
droneplanner serve --port 8765               # interactive explorer (demo terrain)
droneplanner version
```

`plan` writes into `out/`:

| File | Contents |
|---|---|
| `trajectory.geojson` | 3D `LineString` (projected CRS coords) + per-vertex altitude-AGL / band / speed |
| `mission.plan` | QGroundControl `.plan` (waypoints in WGS84 lat/lon, PX4 firmware) |
| `report.json` | machine-readable risk report |
| `report.md` | human-readable risk report with the **GO / REVIEW / NO-GO** recommendation |

With `k_routes > 1` the artifacts are suffixed `.route0`, `.route1`, … (M3 makes
the routes spatially diverse; M0 emits identical straight lines).

### Mission file

See [`configs/mission.example.yaml`](configs/mission.example.yaml). Coordinates
may be projected `{x, y}` in the mission `crs` **or** WGS84 `{lon, lat}` (converted
on load). `crs` **must** be a projected metric CRS — the pipeline does all
distance/clearance math in meters, never degrees.

## Design invariants (do not regress)

- **CRS discipline.** Every geospatial input carries an explicit CRS; all math
  runs in one projected metric CRS. Geographic CRSs are rejected on load.
- **Determinism.** Same mission + seed ⇒ byte-identical plan/report (wall-clock
  timestamp aside). All RNGs seed from `mission.seed`.
- **Human-in-the-loop.** The planner emits a plan **and** a risk report; it never
  certifies a blind GO. Without analysis layers the recommendation caps at REVIEW.
- **License hygiene.** Shippable code uses permissive deps only. GPL tools (GRASS
  viewshed, etc.) are process/CLI-boundary or R&D-only and are never linked in.
  Every dependency is inventoried in `THIRD_PARTY.md`.
- **Scope.** Offline/preflight only. No flight control, no targeting/munitions —
  "threat" enters solely as a passive survivability cost on cable placement.

## Development

```bash
ruff check . && ruff format --check .   # lint + format
mypy --strict src                       # types
pytest --cov=droneplanner --cov-report=term-missing --cov-fail-under=85
```

CI (`.github/workflows/ci.yml`) runs all four gates on Python 3.11 and 3.12.

Regenerate golden regression snapshots deliberately (review the diff in the PR):

```bash
UPDATE_GOLDEN=1 pytest tests/regression -q
```

## Human/plausibility validation (M1+)

Overlay `out/trajectory.geojson` on the DSM/viewshed in QGIS and compare the
planned "hidden path" to a manually drawn concealed route (Software-Spec §6.6).
The straight-line M0 stub has nothing to conceal yet — this check begins at M1.
