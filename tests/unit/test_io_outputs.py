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
    home = doc["mission"]["plannedHomePosition"]
    assert home == [exp_lat, exp_lon, plan.vertices[0].altitude_agl_m]


def test_qgc_plan_written_and_valid_json(tmp_path: Path) -> None:
    plan = straight_line_plan(make_mission())
    out = tmp_path / "mission.plan"
    write_plan(out, plan)
    loaded = json.loads(out.read_text())
    assert loaded["version"] == 1
    assert loaded["mission"]["items"][-1]["doJumpId"] == len(plan.vertices)
