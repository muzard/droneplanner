"""3D trajectory writer — GeoJSON LineString with Z (Software-Spec §4, §5.3).

Coordinates are written in the mission's *projected* working CRS (so the output
overlays directly on the projected DSM/viewshed in QGIS, per Software-Spec §6.6).
Because RFC 7946 assumes WGS84, the working CRS is recorded both in a legacy
top-level ``crs`` member and in ``properties`` so downstream tools are not misled.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from droneplanner.model.plan import Plan


def trajectory_to_geojson(plan: Plan) -> dict[str, Any]:
    """Serialize a plan's 3D track as a GeoJSON Feature (projected coords)."""
    coordinates = [[v.x, v.y, v.z] for v in plan.vertices]
    vertex_props = [
        {
            "altitude_agl_m": v.altitude_agl_m,
            "flight_band": v.flight_band.value,
            "speed_hint_ms": v.speed_hint_ms,
        }
        for v in plan.vertices
    ]
    return {
        "type": "Feature",
        # Legacy GeoJSON CRS member: coordinates are projected, not WGS84.
        "crs": {"type": "name", "properties": {"name": plan.crs}},
        "geometry": {"type": "LineString", "coordinates": coordinates},
        "properties": {
            "mission": plan.mission_name,
            "route_index": plan.route_index,
            "crs": plan.crs,
            "total_length_m": plan.total_length_m,
            "fiber_used_m": plan.fiber_used_m,
            "vertices": vertex_props,
        },
    }


def write_trajectory(path: str | Path, plan: Plan) -> None:
    """Write the trajectory GeoJSON to ``path`` (pretty-printed, deterministic)."""
    feature = trajectory_to_geojson(plan)
    Path(path).write_text(json.dumps(feature, indent=2, sort_keys=True) + "\n", encoding="utf-8")
