"""MAVLink mission writer — QGroundControl ``.plan`` JSON (Software-Spec §4, §5.3).

QGC ``.plan`` files carry waypoints in WGS84 lat/lon with a **relative-to-home**
altitude, so this writer converts the plan's projected vertices back to lon/lat
via pyproj *and* converts altitudes out of the planner's terrain-relative frame.

**Altitude frame — read before changing.** The planner thinks in **AGL above the
terrain beneath each vertex** (nap-of-the-earth flight). A `.plan` waypoint under
`MAV_FRAME_GLOBAL_RELATIVE_ALT` is **height above the home point**, which is a
different thing the moment the ground is not flat. Emitting AGL directly is a
controlled-flight-into-terrain bug: on the demo AO (73.5 m of relief) it put
**57 of 77 waypoints below ground**. It stayed invisible because the no-terrain
stub is flat *and* Gazebo's default world is flat.

So each vertex carries `ground_elev_m` (DTM elevation under it) and we emit::

    alt_rel_home = (ground_elev + agl) - home_ground_elev

`plannedHomePosition` altitude is the **ground elevation AMSL at home**, not an
AGL value.

*Chosen frame (revisit if needed):* relative-to-home keeps `MAV_FRAME 3` and
needs no change in `droneflight` (MAVSDK's `MissionItem` takes
`relative_altitude_m`). The alternatives, if this ever proves limiting, are
absolute AMSL (`MAV_FRAME_GLOBAL` — unambiguous, but the flight stack must switch
frames too) or `MAV_FRAME_GLOBAL_TERRAIN_ALT` (closest to the nap-of-the-earth
intent, but PX4 support is limited and it needs onboard terrain data).
"""

from __future__ import annotations

import itertools
import json
import math
from pathlib import Path
from typing import Any

from droneplanner.model.geometry import crs_to_lonlat
from droneplanner.model.plan import Plan

_MAV_CMD_NAV_WAYPOINT = 16
_MAV_CMD_DO_CHANGE_SPEED = 178
_SPEED_TYPE_GROUND = 1
_SPEED_STEP_MS = 0.5
"""Leg speeds are floored to this step, so a speed that wobbles by centimetres per
second along a stand does not emit a change item at every vertex."""
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


def _speed_item(speed_ms: float, jump_id: int) -> dict[str, Any]:
    return {
        "type": "SimpleItem",
        "command": _MAV_CMD_DO_CHANGE_SPEED,
        "frame": 2,  # MAV_FRAME_MISSION
        "params": [_SPEED_TYPE_GROUND, speed_ms, -1, 0, 0, 0, 0],
        "autoContinue": True,
        "doJumpId": jump_id,
    }


def leg_speeds_ms(plan: Plan) -> list[float]:
    """Speed for each leg ``i -> i+1``: the slower end's hint, floored to the step.

    The slower end, because a leg that *enters* a dense stand must already be
    slow when it gets there — PX4 applies a speed change at the waypoint, not
    ahead of it.
    """
    vs = plan.vertices
    out = []
    for a, b in itertools.pairwise(vs):
        v = min(a.speed_hint_ms, b.speed_hint_ms)
        out.append(max(_SPEED_STEP_MS, math.floor(v / _SPEED_STEP_MS) * _SPEED_STEP_MS))
    return out


def plan_to_qgc(plan: Plan) -> dict[str, Any]:
    """Build a QGroundControl ``.plan`` (v1 file / v2 mission) dict from a plan.

    🔴 **Speed is emitted (2026-09-23).** Until then the ``.plan`` carried one
    ``cruiseSpeed`` and nothing else, while the endurance verdict in
    ``report.json`` was computed at the per-vertex forest speed caps — so the
    planner's NO-GO rested on a speed the vehicle was never told to fly, and the
    2026-09-14 MVP flight did the whole route at 6.0 m/s. Now every change in leg
    speed becomes a ``MAV_CMD_DO_CHANGE_SPEED`` item, which PX4 mission mode obeys
    and ``droneflight``'s loader reads back.
    """
    lonlat = [crs_to_lonlat(v.coordinate, plan.crs) for v in plan.vertices]
    # Home is the first vertex; its GROUND elevation is the datum every relative
    # altitude is measured from (see the module docstring on altitude frames).
    home_ground_m = plan.vertices[0].ground_elev_m if plan.vertices else 0.0
    legs = leg_speeds_ms(plan)
    cruise = legs[0] if legs else (plan.vertices[0].speed_hint_ms if plan.vertices else 5.0)
    items: list[dict[str, Any]] = []
    current = cruise
    for i, (v, (lon, lat)) in enumerate(zip(plan.vertices, lonlat, strict=True)):
        items.append(
            _simple_item(
                lat=lat, lon=lon, alt_m=v.altitude_amsl_m - home_ground_m, jump_id=len(items) + 1
            )
        )
        if i < len(legs) and legs[i] != current:
            current = legs[i]
            items.append(_speed_item(current, jump_id=len(items) + 1))
    home_lon, home_lat = lonlat[0]
    home_alt = home_ground_m
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
        # Not part of QGC's schema: PX4 never sees it (only `items` is uploaded).
        # The lateral freedom the plan grants at each NAV_WAYPOINT, in order —
        # the A1 corridor, which until 2026-09-24 lived only in report.json, so the
        # executor could not know where leaving the line leaves the cover.
        "droneplanner": {
            "version": 1,
            "corridor_half_width_m": [round(v.corridor_half_width_m, 1) for v in plan.vertices],
        },
    }


def write_plan(path: str | Path, plan: Plan) -> None:
    """Write a QGC ``.plan`` file (deterministic JSON)."""
    doc = plan_to_qgc(plan)
    Path(path).write_text(json.dumps(doc, indent=4, sort_keys=True) + "\n", encoding="utf-8")
