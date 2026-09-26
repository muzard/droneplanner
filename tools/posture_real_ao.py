"""Posture vs threat on REAL canopy (Askola measured zone), scored inside the zone.

    python tools/posture_real_ao.py

Threat-Geometry-vs-Flight-Band.md §6g: the posture gate (below vs canopy-top) asked of
real Finnish forest instead of the synthetic demo AO. Plans the measured-zone mission in
three postures and scores the flown vertices inside the 300 m zone against a one-sided
ground observer and a 300 m airborne sensor, both with a 12x optic.
"""

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

base = yaml.safe_load(Path("data/askola_ao/mission_measured.yaml").read_text())
base["covered_radius_m"] = 300.0
with rasterio.open(base["layers"]["dtm"]) as d:
    dtm = d.read(1).astype(float)
    t = d.transform
with rasterio.open(base["layers"]["dsm"]) as d:
    dsm = d.read(1).astype(float)
grid = Grid(
    transform=(t.a, t.b, t.c, t.d, t.e, t.f), rows=dtm.shape[0], cols=dtm.shape[1], crs=base["crs"]
)
sx, sy = base["start"]["x"], base["start"]["y"]
postures = {
    "below (default)": {},
    "below, crown-aware 5-7 m": {"altitude": {"crown_ratio": 0.55, "min_agl_m": 4.0}},
    "canopy-top": {"altitude": {"min_canopy_m": 999.0}},
}
threats = {
    "ground 2 m, 12x, 600 m W": (2.0, [(sx - 600.0, sy + 100.0)]),
    "airborne 300 m, 12x, 600 m W": (300.0, [(sx - 600.0, sy + 100.0)]),
}
for tname, (h, obs) in threats.items():
    for pname, over in postures.items():
        m = copy.deepcopy(base)
        m["threat"]["observers"] = [{"x": x, "y": y} for x, y in obs]
        m["threat"]["observer_height_m"] = h
        m["threat"].setdefault("detection", {})["optic_magnification"] = 12.0
        for k, v in over.items():
            m[k].update(v)
        mission = Mission.model_validate(m)
        plan, surface = plan_over_terrain(mission, dtm, dsm, grid, route_index=0)
        v = [p for p in plan.vertices if math.hypot(p.x - sx, p.y - sy) <= 300.0]
        pts = [(p.x, p.y, p.ground_elev_m + p.altitude_agl_m) for p in v]
        # Three occlusion models on the same flown points: the binary DSM wall with
        # the scalar and the measured gap, and crown transmission (measured gap).
        tx = copy.deepcopy(m)
        tx["threat"].setdefault("canopy_occlusion", {})["crown_transmission"] = True
        medium = crown_medium_for(
            Mission.model_validate(tx), dtm, dsm, surface.components["stem_density_per_ha"], grid
        )
        scored = {}
        for label, gaps, med in (
            ("binary 0.10", None, None),
            ("binary measured", gap_map_for(mission, grid), None),
            ("transmission", None, medium),
        ):
            scored[label] = point_exposure(
                dsm,
                grid,
                [Coordinate(x=x, y=y) for x, y in obs],
                pts,
                observer_height_m=h,
                canopy=mission.threat.canopy_occlusion,
                detection=mission.threat.detection,
                target_span_m=mission.drone.rotor_span_m,
                off_axis_vigilance=mission.threat.off_axis_vigilance,
                gap_map=gaps,
                medium=med,
            )
        exp = scored["binary measured"]
        agl = np.median([p.altitude_agl_m for p in v])
        energy = plan.endurance.energy_fraction if plan.endurance else float("nan")
        print(
            f"{tname:32s} {pname:28s} zone vertices {len(v):3d}  median AGL {agl:5.1f}  "
            f"zone exposure mean {exp.mean():.3f} max {exp.max():.3f}  energy {energy:.2f}",
            flush=True,
        )
        print(
            "    "
            + "  ".join(f"{k}: {v.mean():.3f} (max {v.max():.3f})" for k, v in scored.items()),
            flush=True,
        )
