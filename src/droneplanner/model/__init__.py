"""Pydantic data contracts for the planner (Software-Spec §4)."""

from __future__ import annotations

from droneplanner.model.geometry import (
    Coordinate,
    crs_to_lonlat,
    lonlat_to_crs,
    validate_projected_crs,
)
from droneplanner.model.mission import (
    DroneSpec,
    LayerPaths,
    Mission,
    PayoutSide,
    SpoolSpec,
    ThreatModel,
    ThreatSector,
    Weights,
)
from droneplanner.model.plan import (
    Crossing,
    CrossingMethod,
    FlightBand,
    FlyabilityClass,
    KinematicReport,
    Plan,
    RouteSegment,
    TrajectoryVertex,
)
from droneplanner.model.report import (
    CrossingReport,
    Finding,
    Recommendation,
    RiskReport,
    SegmentReport,
    Severity,
)

__all__ = [
    "Coordinate",
    "Crossing",
    "CrossingMethod",
    "CrossingReport",
    "DroneSpec",
    "Finding",
    "FlightBand",
    "FlyabilityClass",
    "KinematicReport",
    "LayerPaths",
    "Mission",
    "PayoutSide",
    "Plan",
    "Recommendation",
    "RiskReport",
    "RouteSegment",
    "SegmentReport",
    "Severity",
    "SpoolSpec",
    "ThreatModel",
    "ThreatSector",
    "TrajectoryVertex",
    "Weights",
    "crs_to_lonlat",
    "lonlat_to_crs",
    "validate_projected_crs",
]
