"""Mission YAML loader (Software-Spec §4).

Accepts coordinates either as projected ``{x, y}`` in the mission CRS or as
WGS84 ``{lon, lat}`` (converted to the working CRS on load, honoring the
CRS-discipline invariant). All other fields map straight onto the ``Mission``
model, which validates them.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from droneplanner.model.geometry import Coordinate, lonlat_to_crs
from droneplanner.model.mission import Mission


class MissionLoadError(ValueError):
    """Raised when a mission file cannot be parsed into a valid ``Mission``."""


def _parse_coordinate(raw: Any, working_crs: str, field: str) -> Coordinate:
    if not isinstance(raw, dict):
        raise MissionLoadError(f"{field!r} must be a mapping, got {type(raw).__name__}")
    keys = set(raw)
    if {"x", "y"} <= keys:
        return Coordinate(x=float(raw["x"]), y=float(raw["y"]))
    if {"lon", "lat"} <= keys:
        return lonlat_to_crs(float(raw["lon"]), float(raw["lat"]), working_crs)
    raise MissionLoadError(f"{field!r} needs either (x, y) or (lon, lat); got keys {sorted(keys)}")


def _resolve_coordinates(data: dict[str, Any]) -> dict[str, Any]:
    """Convert start/end and any observer coordinates into projected x/y."""
    crs = data.get("crs")
    if not isinstance(crs, str):
        raise MissionLoadError("mission is missing a string 'crs'")

    resolved = dict(data)
    for field in ("start", "end"):
        if field not in resolved:
            raise MissionLoadError(f"mission is missing required field {field!r}")
        resolved[field] = _parse_coordinate(resolved[field], crs, field)

    threat = resolved.get("threat")
    if isinstance(threat, dict):
        threat = dict(threat)
        if isinstance(threat.get("observers"), list):
            threat["observers"] = [
                _parse_coordinate(o, crs, f"threat.observers[{i}]")
                for i, o in enumerate(threat["observers"])
            ]
        if isinstance(threat.get("sectors"), list):
            new_sectors = []
            for i, sector in enumerate(threat["sectors"]):
                sector = dict(sector)
                if isinstance(sector.get("observer"), dict):
                    sector["observer"] = _parse_coordinate(
                        sector["observer"], crs, f"threat.sectors[{i}].observer"
                    )
                new_sectors.append(sector)
            threat["sectors"] = new_sectors
        resolved["threat"] = threat

    return resolved


def load_mission(path: str | Path) -> Mission:
    """Load and validate a mission YAML file into a ``Mission``."""
    path = Path(path)
    try:
        raw_text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise MissionLoadError(f"cannot read mission file {path}: {exc}") from exc

    try:
        data = yaml.safe_load(raw_text)
    except yaml.YAMLError as exc:
        raise MissionLoadError(f"invalid YAML in {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise MissionLoadError(f"mission file {path} must contain a mapping at the top level")

    resolved = _resolve_coordinates(data)
    try:
        return Mission.model_validate(resolved)
    except Exception as exc:  # pydantic ValidationError -> a single clear error type
        raise MissionLoadError(f"invalid mission {path}: {exc}") from exc
