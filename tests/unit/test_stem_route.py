"""Unit tests for stem mapping and stem-aware routing (TEST-PLAN N4)."""

from __future__ import annotations

import numpy as np
import pytest

from droneplanner.fields.stems import (
    StemMap,
    assess_stem_clearance,
    load_stem_manifest,
    required_clearance_m,
    sample_polyline,
)
from droneplanner.model.geometry import Coordinate
from droneplanner.planner.stem_route import refine_through_stems

HOME = Coordinate(x=1000.0, y=2000.0)


def _stems(points: list[tuple[float, float]], radius: float = 0.15) -> StemMap:
    arr = np.array(points, dtype=float).reshape(-1, 2)
    return StemMap(x=arr[:, 0], y=arr[:, 1], radius_m=np.full(len(arr), radius))


# --- The map itself ---------------------------------------------------------------


def test_manifest_is_home_relative_and_is_converted_to_world(tmp_path) -> None:
    """The fourth absolute-vs-home-relative bug is the one that does not happen."""
    p = tmp_path / "t.csv"
    p.write_text("east_m,north_m,base_z_m,radius_m,height_m\n10.0,20.0,0.0,0.15,12.0\n")
    stems = load_stem_manifest(p, HOME)
    assert len(stems) == 1
    assert stems.x[0] == pytest.approx(HOME.x + 10.0)
    assert stems.y[0] == pytest.approx(HOME.y + 20.0)


def test_clearance_is_to_the_trunk_SURFACE_not_its_centre() -> None:
    """A 0.15 m trunk 0.30 m from the centre leaves 0.15 m of air, not 0.30."""
    stems = _stems([(0.0, 0.0)], radius=0.15)
    gap = stems.clearance_at(np.array([0.30]), np.array([0.0]))
    assert gap[0] == pytest.approx(0.15)


def test_a_point_inside_a_trunk_reads_negative() -> None:
    stems = _stems([(0.0, 0.0)], radius=0.5)
    assert stems.clearance_at(np.array([0.1]), np.array([0.0]))[0] < 0.0


def test_an_empty_map_is_infinitely_clear() -> None:
    empty = StemMap(np.empty(0), np.empty(0), np.empty(0))
    assert np.isinf(empty.clearance_at(np.array([0.0]), np.array([0.0]))[0])


def test_clearance_is_scored_along_the_line_not_at_the_vertices() -> None:
    """A trunk mid-leg must be found; sampling only the ends would miss it."""
    stems = _stems([(50.0, 0.0)])
    pts = [(0.0, 0.0), (100.0, 0.0)]
    req = required_clearance_m(0.9)
    report = assess_stem_clearance(pts, stems, required_m=req)
    assert report.min_clearance_m < 0.0
    assert report.strikes > 0
    # Both endpoints are 50 m away, so a vertex-only check sees nothing.
    ends = assess_stem_clearance(pts, stems, required_m=req, step_m=1000.0)
    assert ends.strikes == 0


def test_sample_polyline_includes_both_ends() -> None:
    xs, ys = sample_polyline([(0.0, 0.0), (10.0, 0.0)], step_m=3.0)
    assert (xs[0], ys[0]) == (0.0, 0.0)
    assert (xs[-1], ys[-1]) == (10.0, 0.0)


# --- The refinement ---------------------------------------------------------------


def test_it_routes_around_a_trunk_standing_on_the_line() -> None:
    """The headline case: a stem directly on the route, and a way past it."""
    stems = _stems([(50.0, 0.0)])
    pts = [(0.0, 0.0), (25.0, 0.0), (50.0, 0.0), (75.0, 0.0), (100.0, 0.0)]
    req = required_clearance_m(0.9)
    assert assess_stem_clearance(pts, stems, required_m=req).strikes > 0

    refined, report = refine_through_stems(pts, stems, required_m=req)
    after = assess_stem_clearance(refined, stems, required_m=req)
    assert after.strikes == 0
    assert after.min_clearance_m >= req
    assert report.windows_failed == 0
    assert report.max_excursion_m > 0.0  # it really did move


def test_refining_a_clear_route_barely_moves_it() -> None:
    stems = _stems([(50.0, 40.0)])  # well off to one side
    pts = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)]
    refined, report = refine_through_stems(pts, stems, required_m=required_clearance_m(0.9))
    assert report.windows_failed == 0
    assert report.length_ratio == pytest.approx(1.0, abs=0.02)


def test_a_wall_of_stems_with_no_gap_fails_loudly() -> None:
    """No admissible path must be reported, never silently smoothed over."""
    wall = [(50.0, y) for y in np.arange(-30.0, 30.0, 0.8)]
    pts = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)]
    _refined, report = refine_through_stems(pts, _stems(wall), required_m=required_clearance_m(0.9))
    assert report.windows_failed > 0
    assert not report.clean


def test_the_search_prefers_the_middle_of_a_gap() -> None:
    """N4: 5x the margin costs 1.3 % distance, so take the middle, not the edge."""
    stems = _stems([(50.0, 3.0), (50.0, -3.0)])
    pts = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)]
    refined, _ = refine_through_stems(pts, stems, required_m=required_clearance_m(0.9))
    report = assess_stem_clearance(refined, stems, required_m=required_clearance_m(0.9))
    # The gap's centre is y=0 with ~2.85 m of air either side; hugging one trunk
    # would leave far less.
    assert report.min_clearance_m > 2.0


def test_it_will_not_leave_the_corridor_it_was_given() -> None:
    """Concealment is the objective; the refinement may only use granted freedom."""
    stems = _stems([(50.0, 0.0)])
    pts = [(0.0, 0.0), (25.0, 0.0), (50.0, 0.0), (75.0, 0.0), (100.0, 0.0)]
    tight = [0.5] * len(pts)
    _refined, report = refine_through_stems(
        pts,
        stems,
        required_m=required_clearance_m(0.9),
        corridor_half_width_m=tight,
        min_half_width_m=1.0,
        max_half_width_m=1.0,
    )
    # 1 m of freedom cannot clear a trunk needing 0.95 m of air plus the airframe.
    assert report.windows_failed > 0


def test_mismatched_corridor_length_is_rejected() -> None:
    stems = _stems([(50.0, 0.0)])
    pts = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)]
    with pytest.raises(ValueError, match="one value per vertex"):
        refine_through_stems(pts, stems, required_m=1.0, corridor_half_width_m=[5.0, 5.0])


def test_no_stems_is_a_no_op() -> None:
    pts = [(0.0, 0.0), (50.0, 0.0)]
    empty = StemMap(np.empty(0), np.empty(0), np.empty(0))
    refined, report = refine_through_stems(pts, empty, required_m=1.0)
    assert refined == pts
    assert report.windows == 0
