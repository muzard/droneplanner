"""Shared pytest fixtures."""

from __future__ import annotations

from pathlib import Path

import pytest

from droneplanner.model.mission import Mission
from tests.fixtures.synthetic import make_mission


@pytest.fixture
def mission() -> Mission:
    return make_mission()


@pytest.fixture
def example_mission_path() -> Path:
    return Path(__file__).resolve().parents[1] / "configs" / "mission.example.yaml"
