"""The checked-in demo missions must load and find their layers on any machine."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from droneplanner.demo import export_demo_ao
from droneplanner.io.mission import load_mission

REPO = Path(__file__).resolve().parents[2]
DEMO_MISSIONS = sorted((REPO / "demo_ao").glob("*.yaml"))

# Not checked in (``*.las`` is gitignored); ``tools/make_synthetic_scan.py`` writes it.
GENERATED = {"synthetic_stems.las"}


def test_the_demo_missions_were_found() -> None:
    assert DEMO_MISSIONS


@pytest.mark.parametrize("mission_file", DEMO_MISSIONS, ids=lambda p: p.name)
def test_every_referenced_layer_exists_from_any_working_directory(
    mission_file: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)

    layers = load_mission(mission_file).layers

    referenced = [p for p in layers.model_dump().values() if p is not None]
    assert referenced
    for layer in referenced:
        if layer.name in GENERATED:
            assert layer.parent == mission_file.parent
        else:
            assert layer.exists(), f"{mission_file.name}: {layer} does not exist"


@pytest.mark.parametrize("mission_file", DEMO_MISSIONS, ids=lambda p: p.name)
def test_demo_missions_hold_no_absolute_paths(mission_file: Path) -> None:
    layers = yaml.safe_load(mission_file.read_text())["layers"]
    assert layers
    for value in layers.values():
        assert not Path(value).is_absolute(), f"{mission_file.name}: {value}"


def test_exported_demo_ao_is_relocatable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    mission_path = export_demo_ao(tmp_path / "ao")
    layers = yaml.safe_load(mission_path.read_text())["layers"]
    assert layers == {"dtm": "dtm.tif", "dsm": "dsm.tif"}

    moved = tmp_path / "moved"
    (tmp_path / "ao").rename(moved)
    monkeypatch.chdir(tmp_path / "moved" / "..")
    loaded = load_mission(moved / "mission.yaml").layers
    assert loaded.dtm is not None and loaded.dtm.exists()
    assert loaded.dsm is not None and loaded.dsm.exists()
