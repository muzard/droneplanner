"""Unit tests for the demo AO and the visualizers (Software-Spec §6.6)."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

from droneplanner.demo import (
    THREAT_MARGIN_M,
    build_demo,
    demo_flyability,
    demo_layout,
    demo_mission,
    demo_terrain,
    export_demo_ao,
    point_behind_start,
    reverse_bearing_standoff_m,
)
from droneplanner.forest.navigability import NON_NAVIGABLE
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
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


# --- C0: room to stand a threat sensor behind the launch point ----------------


def test_standoff_is_the_distance_to_the_far_edge_behind_the_start() -> None:
    # 10x10 cells of 100 m, origin (0, 1000) top-left -> extent x 0..1000, y 0..1000.
    grid = Grid(
        transform=(100.0, 0.0, 0.0, 0.0, -100.0, 1000.0), rows=10, cols=10, crs="EPSG:32633"
    )
    # Route runs due east from x=400, so "behind" is due west: 400 m of grid left.
    start, end = Coordinate(x=400.0, y=500.0), Coordinate(x=900.0, y=500.0)
    assert reverse_bearing_standoff_m(grid, start, end) == 400.0
    # Reverse the route and the standoff becomes the eastward remainder.
    assert reverse_bearing_standoff_m(grid, end, start) == 100.0


def test_default_export_can_stand_a_sensor_at_threat_margin(tmp_path: Path) -> None:
    """The roadmap C0 acceptance: 2.5 km of grid behind the start, on the raster."""
    mission_path = export_demo_ao(tmp_path / "ao")
    doc = yaml.safe_load(mission_path.read_text())
    start = Coordinate(**doc["start"])
    end = Coordinate(**doc["end"])

    _, _, grid = demo_terrain(threat_margin_m=THREAT_MARGIN_M)
    assert reverse_bearing_standoff_m(grid, start, end) >= THREAT_MARGIN_M
    # ...and the sensor position that margin implies is actually on the raster,
    # which is the whole point: an observer off-grid cannot be modelled at all.
    assert grid.contains(point_behind_start(start, end, THREAT_MARGIN_M))


def test_supplied_threat_margin_is_honoured_and_minimal() -> None:
    for margin in (0.0, 1000.0, 2500.0, 5000.0):
        layout = demo_layout(threat_margin_m=margin)
        assert layout.threat_standoff_m() >= margin
        if layout.margin_cells:
            # One cell less must fail, or the AO is bigger than it needs to be —
            # padding is quadratic in cells and every cell costs search time.
            leaner = replace(layout, margin_cells=layout.margin_cells - 1)
            assert leaner.threat_standoff_m() < margin


def test_threat_margin_moves_neither_the_mission_nor_the_scene() -> None:
    """Padding adds room behind the start; it must not restate the problem."""
    plain_dtm, plain_dsm, plain_grid = demo_terrain(size=60)
    padded_dtm, padded_dsm, padded_grid = demo_terrain(size=60, threat_margin_m=1000.0)
    plain, padded = demo_mission(plain_grid), demo_mission(padded_grid)

    assert (plain.start, plain.end) == (padded.start, padded.end)
    assert plain.threat.observers == padded.threat.observers

    pad = demo_layout(size=60, threat_margin_m=1000.0).margin_cells
    assert pad > 0
    scene = (slice(0, 60), slice(pad, pad + 60))
    np.testing.assert_array_equal(padded_dtm[scene], plain_dtm)
    np.testing.assert_array_equal(padded_dsm[scene], plain_dsm)


def test_viz_with_forest_flyability_layer() -> None:
    mission, dtm, dsm, grid = build_demo(size=70)
    field = demo_flyability(grid, mission.drone)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid, flyability=field)
    assert "flyability_class" in surface.components
    png = render_interactive(mission, surface, plan, layer="flyability", size_px=320)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
