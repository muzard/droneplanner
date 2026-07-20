"""Unit tests for the demo AO and the visualizers (Software-Spec §6.6)."""

from __future__ import annotations

from pathlib import Path

from droneplanner.demo import build_demo, demo_flyability, demo_mission, demo_terrain
from droneplanner.forest.navigability import NON_NAVIGABLE
from droneplanner.planner.corridor import plan_over_terrain
from droneplanner.viz import LAYERS, render_interactive, save_figure


def test_demo_terrain_shapes() -> None:
    dtm, dsm, grid = demo_terrain(size=60)
    assert dtm.shape == (60, 60)
    assert dsm.shape == (60, 60)
    assert grid.rows == 60 and grid.cols == 60


def test_demo_mission_valid_and_in_grid() -> None:
    dtm, dsm, grid = demo_terrain(size=60)
    mission = demo_mission(grid)
    assert grid.contains(mission.start)
    assert grid.contains(mission.end)
    assert len(mission.threat.observers) == 2


def test_save_figure_writes_png(tmp_path: Path) -> None:
    mission, dtm, dsm, grid = build_demo(size=70)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid)
    out = tmp_path / "route.png"
    save_figure(out, mission, surface, plan)
    assert out.is_file()
    assert out.stat().st_size > 1000  # a non-trivial PNG
    assert out.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_render_interactive_returns_png_bytes() -> None:
    mission, dtm, dsm, grid = build_demo(size=70)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid)
    for layer in LAYERS:
        png = render_interactive(mission, surface, plan, layer=layer, size_px=320)
        assert png[:8] == b"\x89PNG\r\n\x1a\n"


def test_demo_flyability_has_non_navigable_patch() -> None:
    _, _, grid = demo_terrain(size=70)
    mission = demo_mission(grid)
    field = demo_flyability(grid, mission.drone)
    assert (field.navigable_class == NON_NAVIGABLE).sum() > 0
    assert field.non_navigable.shape == (70, 70)


def test_viz_with_forest_flyability_layer() -> None:
    mission, dtm, dsm, grid = build_demo(size=70)
    field = demo_flyability(grid, mission.drone)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid, flyability=field)
    assert "flyability_class" in surface.components
    png = render_interactive(mission, surface, plan, layer="flyability", size_px=320)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
