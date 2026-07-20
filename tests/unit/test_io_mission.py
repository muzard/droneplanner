"""Unit tests for the mission YAML loader (Software-Spec §4)."""

from __future__ import annotations

from pathlib import Path

import pytest

from droneplanner.io.mission import MissionLoadError, load_mission


def test_load_example_mission(example_mission_path: Path) -> None:
    mission = load_mission(example_mission_path)
    assert mission.name == "example-ao"
    assert mission.crs == "EPSG:32633"
    assert mission.start.x == pytest.approx(500000.0)
    assert mission.spool.length_m == pytest.approx(5000.0)
    assert len(mission.threat.observers) == 1


def test_load_lonlat_coordinates(tmp_path: Path) -> None:
    yaml_text = """
name: ll-ao
crs: EPSG:32633
start: {lon: 15.0, lat: 46.9}
end: {x: 501000.0, y: 5200800.0}
drone: {rotor_span_m: 0.9, mass_kg: 6.5, min_turn_radius_m: 8.0,
        climb_rate_max_ms: 3.0, descent_rate_max_ms: 2.0}
spool: {length_m: 5000.0, mass_kg: 1.2}
"""
    path = tmp_path / "m.yaml"
    path.write_text(yaml_text)
    mission = load_mission(path)
    # 15E/46.9N converts to a UTM 33N easting near 500 km.
    assert 498000.0 < mission.start.x < 502000.0


def test_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(MissionLoadError, match="cannot read"):
        load_mission(tmp_path / "nope.yaml")


def test_invalid_yaml_raises(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("name: [unclosed\n")
    with pytest.raises(MissionLoadError, match="invalid YAML"):
        load_mission(path)


def test_non_mapping_top_level_raises(tmp_path: Path) -> None:
    path = tmp_path / "list.yaml"
    path.write_text("- a\n- b\n")
    with pytest.raises(MissionLoadError, match="must contain a mapping"):
        load_mission(path)


def test_missing_crs_raises(tmp_path: Path) -> None:
    path = tmp_path / "nocrs.yaml"
    path.write_text("name: x\nstart: {x: 0, y: 0}\nend: {x: 1, y: 1}\n")
    with pytest.raises(MissionLoadError, match="missing a string 'crs'"):
        load_mission(path)


def test_missing_start_raises(tmp_path: Path) -> None:
    path = tmp_path / "nostart.yaml"
    path.write_text("name: x\ncrs: EPSG:32633\nend: {x: 1, y: 1}\n")
    with pytest.raises(MissionLoadError, match="missing required field 'start'"):
        load_mission(path)


def test_bad_coordinate_keys_raise(tmp_path: Path) -> None:
    path = tmp_path / "badcoord.yaml"
    path.write_text("name: x\ncrs: EPSG:32633\nstart: {a: 1}\nend: {x: 1, y: 1}\n")
    with pytest.raises(MissionLoadError, match="either \\(x, y\\) or \\(lon, lat\\)"):
        load_mission(path)


def test_invalid_mission_values_raise(tmp_path: Path) -> None:
    path = tmp_path / "badspool.yaml"
    path.write_text(
        "name: x\ncrs: EPSG:32633\nstart: {x: 0, y: 0}\nend: {x: 1, y: 1}\n"
        "drone: {rotor_span_m: 0.9, mass_kg: 6.5, min_turn_radius_m: 8.0,"
        " climb_rate_max_ms: 3.0, descent_rate_max_ms: 2.0}\n"
        "spool: {length_m: -1.0, mass_kg: 1.2}\n"
    )
    with pytest.raises(MissionLoadError, match="invalid mission"):
        load_mission(path)
