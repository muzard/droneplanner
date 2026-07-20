"""MAVLink mission writer — QGroundControl ``.plan`` JSON (Software-Spec §4, §5.3).

QGC ``.plan`` files carry waypoints in WGS84 lat/lon with a relative altitude,
so this writer converts the plan's projected vertices back to lon/lat via
pyproj. Altitudes are AGL, interpreted as relative-to-home (MAV_FRAME 3,
``AltitudeMode`` = relative).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from droneplanner.model.geometry import crs_to_lonlat
from droneplanner.model.plan import Plan

_MAV_CMD_NAV_WAYPOINT = 16
_MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
_PX4_FIRMWARE = 12  # QGC firmwareType for PX4 Pro
_MULTIROTOR = 2  # QGC vehicleType


def _simple_item(lat: float, lon: float, alt_m: float, jump_id: int) -> dict[str, Any]:
    return {
        "type": "SimpleItem",
        "command": _MAV_CMD_NAV_WAYPOINT,
        "frame": _MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "params": [0.0, 0.0, 0.0, None, lat, lon, alt_m],
        "Altitude": alt_m,
        "AltitudeMode": 1,  # relative to home
        "AMSLAltAboveTerrain": None,
        "autoContinue": True,
        "doJumpId": jump_id,
    }


def plan_to_qgc(plan: Plan) -> dict[str, Any]:
    """Build a QGroundControl ``.plan`` (v1 file / v2 mission) dict from a plan."""
    lonlat = [crs_to_lonlat(v.coordinate, plan.crs) for v in plan.vertices]
    items = [
        _simple_item(lat=lat, lon=lon, alt_m=v.altitude_agl_m, jump_id=i + 1)
        for i, (v, (lon, lat)) in enumerate(zip(plan.vertices, lonlat, strict=True))
    ]
    home_lon, home_lat = lonlat[0]
    home_alt = plan.vertices[0].altitude_agl_m
    cruise = plan.vertices[0].speed_hint_ms if plan.vertices else 5.0
    return {
        "fileType": "Plan",
        "version": 1,
        "groundStation": "droneplanner",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {
            "version": 2,
            "firmwareType": _PX4_FIRMWARE,
            "vehicleType": _MULTIROTOR,
            "cruiseSpeed": cruise,
            "hoverSpeed": cruise,
            "globalPlanAltitudeMode": 1,
            "plannedHomePosition": [home_lat, home_lon, home_alt],
            "items": items,
        },
    }


def write_plan(path: str | Path, plan: Plan) -> None:
    """Write a QGC ``.plan`` file (deterministic JSON)."""
    doc = plan_to_qgc(plan)
    Path(path).write_text(json.dumps(doc, indent=4, sort_keys=True) + "\n", encoding="utf-8")
