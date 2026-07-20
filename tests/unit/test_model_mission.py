"""Unit tests for the mission/geometry data contracts (Software-Spec §4)."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from droneplanner.model.geometry import (
    Coordinate,
    crs_to_lonlat,
    lonlat_to_crs,
    validate_projected_crs,
)
from droneplanner.model.mission import DroneSpec
from tests.fixtures.synthetic import make_mission


def test_coordinate_distance() -> None:
    a = Coordinate(x=0.0, y=0.0)
    b = Coordinate(x=3.0, y=4.0)
    assert a.distance_to(b) == pytest.approx(5.0)


def test_drone_rotor_radius() -> None:
    drone = DroneSpec(
        rotor_span_m=1.0,
        mass_kg=5.0,
        min_turn_radius_m=8.0,
        climb_rate_max_ms=3.0,
        descent_rate_max_ms=2.0,
    )
    assert drone.rotor_radius_m == pytest.approx(0.5)


def test_validate_projected_crs_accepts_utm() -> None:
    assert validate_projected_crs("EPSG:32633") == "EPSG:32633"


def test_validate_projected_crs_rejects_geographic() -> None:
    with pytest.raises(ValueError, match="not a projected"):
        validate_projected_crs("EPSG:4326")


def test_validate_projected_crs_rejects_garbage() -> None:
    with pytest.raises(ValueError, match="unparseable"):
        validate_projected_crs("not-a-crs")


def test_mission_rejects_geographic_crs() -> None:
    with pytest.raises(ValidationError):
        make_mission(crs="EPSG:4326")


def test_mission_rejects_coincident_endpoints() -> None:
    with pytest.raises(ValidationError):
        make_mission(start={"x": 1.0, "y": 2.0}, end={"x": 1.0, "y": 2.0})


def test_mission_defaults() -> None:
    m = make_mission()
    assert m.k_routes == 1
    assert m.weights.exposure == 1.0
    assert m.threat.observers == ()
    assert m.fiber_slack_factor >= 1.0


def test_mission_is_frozen() -> None:
    m = make_mission()
    with pytest.raises(ValidationError):
        m.name = "changed"  # type: ignore[misc]


def test_lonlat_roundtrip() -> None:
    coord = lonlat_to_crs(15.0, 46.9, "EPSG:32633")
    lon, lat = crs_to_lonlat(coord, "EPSG:32633")
    assert lon == pytest.approx(15.0, abs=1e-6)
    assert lat == pytest.approx(46.9, abs=1e-6)
    # UTM 33N easting near the central meridian (15E) is ~500 km.
    assert math.isclose(coord.x, 500000.0, abs_tol=2000.0)
