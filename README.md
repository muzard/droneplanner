# droneplanner

Offline **preflight route planner** for the autonomous fiber-laying drone. It
turns a mission — start, end, map layers, threat model, drone/spool geometry —
into a 3D flight-and-laydown plan plus a **human-in-the-loop risk report**. It is
the only pure-software subsystem of the larger program; the flight/navigation
stack is a separate, hardware-dependent effort (see `../Software-Spec.md` §1, §8).

This repository implements the module specified in **`../Software-Spec.md`** with
algorithmic rationale in **`../Preflight-Planning-Module-Design.md`**.

## Status — through milestone M2 (forest flyability)

`droneplanner plan` runs end-to-end. Given a DTM+DSM it plans a **concealment-
and defilade-aware route** (in-house viewshed + terrain TPI/slope/drainage +
survivability, combined into a composite cost surface, solved by a **cost-aware
any-angle Theta\***). The aircraft flies **under the canopy** where the canopy
height model allows — as low as the terrain and the airframe permit, not at a
fixed cruise altitude — and visibility is scored for **two objects**: the
aircraft at its planned flight altitude, and the cable on the ground beneath it.
They hide in different places. Add a **lidar point cloud** and it builds the **forest
flyability** field (voxelize → vertical segmentation → 3D EDT clearance →
FLYABLE/MARGINAL/NON_NAVIGABLE), routes around non-navigable clutter, and flies
the upper-trunk band. Add a **roads** layer and each route/road crossing is
flagged as an OPEN (fly-over) crossing. Without terrain it falls back to the M0
straight-line stub.

A **kinematic feasibility pass** then makes the trajectory flyable: corners are
rounded to arcs within the airframe's `min_turn_radius`, altitude varies within
the navigable band under the climb/descent-rate limits, and reversals (which
would slacken the paying-out fiber) are flagged. The report carries a
`KinematicReport` — a reversal is NO-GO, turn/climb violations are REVIEW.

The report caps at **REVIEW** (never a blind GO) until the open gaps in
[SHORTFALLS.md](SHORTFALLS.md) are closed — read that before trusting a plan or
moving to sims.

Try it with **zero data** on the built-in demo terrain (now with a forest patch):

```bash
droneplanner viz   --out route.png      # static multi-panel figure (fields + flyability + route)
droneplanner serve                      # interactive explorer at http://127.0.0.1:8765
```

The explorer lets you drag observers and the start/end, move the objective-weight
sliders, and switch display layers (incl. flyability) while the route re-plans
live — the fastest way to build intuition for the cost function.

| Milestone | Scope | State |
|---|---|---|
| **M0** | scaffold, data contracts, I/O, CLI, straight-line stub + report | ✅ |
| **M1** | `fields/` (viewshed, terrain, survivability, snag) + cost-aware Theta\* corridor + viz | ✅ |
| **M1.1** | road crossings (open-terrain fly-over stub) | ✅ |
| **M2** | `forest/` voxel flyability (2.5D navigability + non-navigable mask + altitude band) | ✅ |
| **Kinematics** | turn-radius filleting + climb-limited altitude + reversal flagging | ✅ this repo |
| M2 (3D) | OMPL BIT\*/RRT\* refinement (full 3D kinodynamic) | ⏳ see SHORTFALLS.md |
| M3 | laydown + k-diverse survivability routing | later |
| M4 | PX4 SITL / Isaac Sim validation | later |

> **Before sims:** fly the new low route (it is 3× denser and a third of the
> altitude, and unflown), fix the M2 navigable-altitude band datum (it is not
> AGL), wire land cover, and voxelize at fine resolution — see
> [SHORTFALLS.md](SHORTFALLS.md) for the prioritized list. (Threat sectors landed
> 2026-08-15; `threat.sectors` in the mission YAML now shapes the route.)

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

Threat-model keys that decide what "exposed" means:

```yaml
threat:
  observers: [{x: 501687.5, y: 5202787.5}]
  observer_height_m: 2.0        # 2 dismount | 12-30 mast | 300-1000 airborne.
                                # Raising it is the single biggest lever on the
                                # route — see Threat-Geometry-vs-Flight-Band §6a.
  canopy_occlusion:
    sub_canopy_gap_fraction: 0.10   # chance a sub-canopy target is seen through a
                                    # crown gap. 5-20 % for closed Nordic stands;
                                    # 0.0 = fully hidden, 1.0 = no canopy occlusion.
    sub_canopy_max_elev_deg: 10.0   # only steeper sightlines cross the crown layer;
                                    # shallower ones are already handled by the
                                    # DSM march and are left untouched.
  detection:
    threshold_arcmin: 3.0           # angular size at acquisition. 3' is the trials
                                    # figure, 12' is NTSB's more forgiving one.
    optic_magnification: 1.0        # 1 unaided, 7 and 12 are the cases that matter.
    falloff: ramp                   # full weight to half range, linear to 0; or
                                    # "hard" for a stated-range sensor.
  sectors:                          # which way the threat is looking. Compass
                                    # azimuths, swept CLOCKWISE start -> end, so
                                    # 350 -> 20 is a 30 deg arc through north.
                                    # start == end means the FULL circle.
    - azimuth_start_deg: 180.0      # with an observer: masks THAT observer's
      azimuth_end_deg: 270.0        # viewshed to the arc (drone + cable both).
      observer: {x: 501687.5, y: 5202787.5}
      label: overwatch-sw
      detection:                    # optional: the optic searching THIS arc,
        optic_magnification: 12.0   # overriding the mission-wide model.
    - azimuth_start_deg: 90.0       # with NO observer: a threat *bearing*, scored
      azimuth_end_deg: 180.0        # as terrain masking (openness_to_threat_
      label: fires-from-east        # sectors) into cable survivability.
```

`detection` is what makes `exposure` mean "probably acquired" rather than "a
sightline exists": a 0.9 m airframe unaided at 3' is resolvable to **1031 m**, and
beyond that it scores zero however open the ground is.

`sectors` is the other half of that question: a 12x optic only matters if it is
searching your bearing. A sector **removes** cost from the surface and the planner
spends the slack — on the demo AO at 12x a 90 deg arc takes `mean_exposure_flown`
from 0.123 to 0.056 below canopy and shortens the route by 107 m, flying 240 m
*less* below the canopy. Read that as concealment bought on credit against an
intelligence assumption: an adversary who turns around invalidates the plan, and
nothing here models that. Every applied sector is listed in `report.json`
(`threat_sectors`), and sectors supplied to a run that cannot use them raise a
`threat_sectors_ignored` warning rather than vanishing.

⚠️ Two knobs carry most of the uncertainty. `sub_canopy_gap_fraction` is a
literature figure, not a measurement, and the below-canopy result is roughly
proportional to it. `optic_magnification` is the input that can erase the range
argument outright — at 12x the acquisition range is 12.4 km, which covers a whole
AO and restores the entire pre-detection-model exposure picture. Sweep both before
trusting any posture conclusion.

The two visibility layers are **not** in the same currency and must not be compared:
`exposure` is the drone, weighted by crown occlusion and detection range;
`ground_los` is plain line of sight to the surface the cable lies on, with no
detection model, because a 0.28 mm fiber is not optically resolvable at any range.

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
