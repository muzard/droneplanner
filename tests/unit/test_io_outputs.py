"""Unit tests for output writers: trajectory GeoJSON + QGC .plan (§5.3)."""

from __future__ import annotations

import json
from pathlib import Path

from droneplanner.io.mavlink import plan_to_qgc, write_plan
from droneplanner.io.trajectory import trajectory_to_geojson, write_trajectory
from droneplanner.model.geometry import crs_to_lonlat
from droneplanner.planner.stub import straight_line_plan
from tests.fixtures.synthetic import make_mission


def test_trajectory_geojson_structure() -> None:
    plan = straight_line_plan(make_mission())
    feature = trajectory_to_geojson(plan)
    assert feature["type"] == "Feature"
    assert feature["geometry"]["type"] == "LineString"
    coords = feature["geometry"]["coordinates"]
    assert len(coords) == len(plan.vertices)
    assert all(len(c) == 3 for c in coords)  # x, y, z
    assert feature["properties"]["crs"] == plan.crs
    assert len(feature["properties"]["vertices"]) == len(plan.vertices)


def test_trajectory_written_and_valid_json(tmp_path: Path) -> None:
    plan = straight_line_plan(make_mission())
    out = tmp_path / "trajectory.geojson"
    write_trajectory(out, plan)
    loaded = json.loads(out.read_text())
    assert loaded["geometry"]["coordinates"][0][:2] == [plan.start.x, plan.start.y]


def test_qgc_plan_structure() -> None:
    plan = straight_line_plan(make_mission())
    doc = plan_to_qgc(plan)
    assert doc["fileType"] == "Plan"
    assert doc["mission"]["firmwareType"] == 12  # PX4
    items = doc["mission"]["items"]
    assert len(items) == len(plan.vertices)
    first = items[0]
    assert first["command"] == 16  # MAV_CMD_NAV_WAYPOINT
    assert first["doJumpId"] == 1
    # Item coordinate is WGS84 lat/lon derived from the projected start.
    exp_lon, exp_lat = crs_to_lonlat(plan.start, plan.crs)
    assert first["params"][4] == exp_lat
    assert first["params"][5] == exp_lon
    # Home altitude is the GROUND elevation at home (AMSL), never an AGL value.
    home = doc["mission"]["plannedHomePosition"]
    assert home == [exp_lat, exp_lon, plan.vertices[0].ground_elev_m]


def test_qgc_altitudes_are_relative_to_home_not_agl() -> None:
    """Terrain-relative AGL must be converted to relative-to-home altitudes.

    Regression guard: emitting the planner's AGL verbatim is a CFIT bug — over
    73.5 m of relief on the demo AO it put 57 of 77 waypoints *below ground*.
    A vertex 40 m higher up the hill than home, flown at the same 30 m AGL, must
    come out as 70 m relative — not 30.
    """
    base = straight_line_plan(make_mission())
    v0, v1 = base.vertices[0], base.vertices[1]
    plan = base.model_copy(
        update={
            "vertices": (
                v0.model_copy(update={"ground_elev_m": 100.0, "altitude_agl_m": 30.0, "z": 30.0}),
                v1.model_copy(update={"ground_elev_m": 140.0, "altitude_agl_m": 30.0, "z": 30.0}),
                *base.vertices[2:],
            )
        }
    )
    doc = plan_to_qgc(plan)
    items = doc["mission"]["items"]
    assert doc["mission"]["plannedHomePosition"][2] == 100.0  # ground AMSL at home
    assert items[0]["params"][6] == 30.0  # (100+30) - 100
    assert items[1]["params"][6] == 70.0  # (140+30) - 100  <- would be 30 if buggy
    assert items[1]["Altitude"] == 70.0


def test_qgc_plan_written_and_valid_json(tmp_path: Path) -> None:
    plan = straight_line_plan(make_mission())
    out = tmp_path / "mission.plan"
    write_plan(out, plan)
    loaded = json.loads(out.read_text())
    assert loaded["version"] == 1
    assert loaded["mission"]["items"][-1]["doJumpId"] == len(plan.vertices)


def test_leg_speeds_become_do_change_speed_items() -> None:
    """🔴 The planner's endurance verdict rests on per-leg speeds the .plan never
    carried until 2026-09-23 — the vehicle flew cruise everywhere."""
    plan = straight_line_plan(make_mission())
    slowed = [
        v.model_copy(update={"speed_hint_ms": 3.2 if 2 <= i <= 3 else v.speed_hint_ms})
        for i, v in enumerate(plan.vertices)
    ]
    plan = plan.model_copy(update={"vertices": tuple(slowed)})
    items = plan_to_qgc(plan)["mission"]["items"]
    kinds = [it["command"] for it in items]
    assert kinds.count(16) == len(plan.vertices)
    speeds = [it["params"][1] for it in items if it["command"] == 178]
    # Entering the slow stretch slows the leg INTO it (the slower end), floored to 0.5.
    assert speeds == [3.0, plan.vertices[0].speed_hint_ms]
    assert items[kinds.index(178) - 1]["command"] == 16


def test_the_corridor_width_travels_with_the_plan() -> None:
    """B3's blocker: the executor could not see the lateral freedom it was granted."""
    plan = straight_line_plan(make_mission())
    widths = [float(i % 7) for i in range(len(plan.vertices))]
    plan = plan.model_copy(
        update={
            "vertices": tuple(
                v.model_copy(update={"corridor_half_width_m": w})
                for v, w in zip(plan.vertices, widths, strict=True)
            )
        }
    )
    doc = plan_to_qgc(plan)
    assert doc["droneplanner"]["corridor_half_width_m"] == widths
    n_wp = sum(1 for it in doc["mission"]["items"] if it["command"] == 16)
    assert len(doc["droneplanner"]["corridor_half_width_m"]) == n_wp
