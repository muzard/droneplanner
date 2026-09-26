"""Does below-canopy flight on REAL canopy stay under the 0.15 exposure cap from every
threat bearing — and does the occlusion model change the answer?

    python tools/bearing_sweep_real_ao.py [--bearings 12] [--range 600]

Askola measured zone (300 m, `data/askola_ao/mission_measured.yaml`). A 300 m airborne
sensor with a 12x optic is placed ``--range`` metres from the launch on each of
``--bearings`` compass bearings (all-round search, default off-axis vigilance). Each
posture is PLANNED against the occlusion model it is scored with — binary DSM wall
(measured gap map) or crown transmission — and the flown vertices inside the zone are
scored. The user's stated cap (2026-09-14) is 0.15 mean exposure.
"""

from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path

import numpy as np
import rasterio
import yaml

from droneplanner.fields.composite import crown_medium_for, gap_map_for
from droneplanner.fields.viewshed import point_exposure
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission
from droneplanner.planner.corridor import plan_over_terrain

CAP = 0.15
ZONE_M = 300.0

ap = argparse.ArgumentParser()
ap.add_argument("--bearings", type=int, default=12)
ap.add_argument("--range", type=float, default=600.0)
ap.add_argument("--height", type=float, default=300.0)
args = ap.parse_args()

base = yaml.safe_load(Path("data/askola_ao/mission_measured.yaml").read_text())
base["covered_radius_m"] = ZONE_M
with rasterio.open(base["layers"]["dtm"]) as d:
    dtm = np.array(d.read(1), dtype=float)
    t = d.transform
with rasterio.open(base["layers"]["dsm"]) as d:
    dsm = np.array(d.read(1), dtype=float)
grid = Grid(
    transform=(t.a, t.b, t.c, t.d, t.e, t.f), rows=dtm.shape[0], cols=dtm.shape[1], crs=base["crs"]
)
sx, sy = base["start"]["x"], base["start"]["y"]
postures = {
    "below": {},
    "crown-aware": {"altitude": {"crown_ratio": 0.55, "min_agl_m": 4.0}},
}
models = ("binary", "transmission")
results: dict[tuple[str, str], list[tuple[float, float]]] = {}

for k in range(args.bearings):
    az = 360.0 * k / args.bearings
    ox = sx + args.range * math.sin(math.radians(az))
    oy = sy + args.range * math.cos(math.radians(az))
    for pname, over in postures.items():
        for model in models:
            m = copy.deepcopy(base)
            m["threat"]["observers"] = [{"x": ox, "y": oy}]
            m["threat"]["observer_height_m"] = args.height
            m["threat"].setdefault("detection", {})["optic_magnification"] = 12.0
            m["threat"].setdefault("canopy_occlusion", {})["crown_transmission"] = (
                model == "transmission"
            )
            for key, v in over.items():
                m[key].update(v)
            mission = Mission.model_validate(m)
            plan, surface = plan_over_terrain(mission, dtm, dsm, grid, route_index=0)
            v = [p for p in plan.vertices if math.hypot(p.x - sx, p.y - sy) <= ZONE_M]
            pts = [(p.x, p.y, p.ground_elev_m + p.altitude_agl_m) for p in v]
            exp = point_exposure(
                dsm,
                grid,
                [Coordinate(x=ox, y=oy)],
                pts,
                observer_height_m=args.height,
                canopy=mission.threat.canopy_occlusion,
                detection=mission.threat.detection,
                target_span_m=mission.drone.rotor_span_m,
                off_axis_vigilance=mission.threat.off_axis_vigilance,
                gap_map=gap_map_for(mission, grid),
                medium=crown_medium_for(
                    mission, dtm, dsm, surface.components["stem_density_per_ha"], grid
                ),
            )
            mean = float(exp.mean()) if exp.size else 0.0
            peak = float(exp.max()) if exp.size else 0.0
            results.setdefault((pname, model), []).append((mean, peak))
            print(
                f"az {az:5.1f}  {pname:12s} {model:12s} zone vertices {len(v):3d}  "
                f"mean {mean:.3f}  max {peak:.3f}  {'PASS' if mean <= CAP else 'FAIL'}",
                flush=True,
            )

print(f"\nsummary: {args.bearings} bearings, {args.height:.0f} m sensor at {args.range:.0f} m")
for (pname, model), rows in results.items():
    means = [r[0] for r in rows]
    peaks = [r[1] for r in rows]
    fails = sum(1 for x in means if x > CAP)
    print(
        f"  {pname:12s} {model:12s} mean of means {np.mean(means):.3f}  worst mean "
        f"{max(means):.3f}  worst peak {max(peaks):.3f}  over the {CAP} cap: "
        f"{fails}/{len(rows)}"
    )
