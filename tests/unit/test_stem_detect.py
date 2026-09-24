"""Unit tests for stem detection from a point cloud (SHORTFALLS M2)."""

from __future__ import annotations

import numpy as np
import pytest

from droneplanner.fields.stems import StemMap
from droneplanner.forest.stem_detect import (
    StemDetectConfig,
    detect_stems,
    match_stems,
)
from droneplanner.forest.synthetic_scan import simulate_scan
from droneplanner.model.grid import Grid

CELL = 5.0
SIZE = 40


def _grid(origin_x: float = 0.0, origin_y: float = 0.0) -> Grid:
    return Grid(
        transform=(CELL, 0.0, origin_x, 0.0, -CELL, origin_y + SIZE * CELL),
        rows=SIZE,
        cols=SIZE,
        crs="EPSG:32633",
    )


def _flat() -> np.ndarray:
    return np.zeros((SIZE, SIZE), dtype=float)


def _map(points: list[tuple[float, float]], radius: float = 0.15) -> StemMap:
    arr = np.array(points, dtype=float).reshape(-1, 2)
    return StemMap(x=arr[:, 0], y=arr[:, 1], radius_m=np.full(len(arr), radius))


def test_it_finds_a_stem_where_the_stem_is() -> None:
    truth = _map([(100.0, 100.0)])
    cloud = simulate_scan(truth, returns_per_stem_m2=200.0, seed=1)
    found, stats = detect_stems(cloud, _flat(), _grid())
    assert len(found) == 1
    assert stats.detected == 1
    assert np.hypot(found.x[0] - 100.0, found.y[0] - 100.0) < 0.10


def test_the_circle_fit_removes_the_arc_centroid_BIAS() -> None:
    """A half-arc's centroid sits 2r/pi in front of the axis, toward the scanner.

    That bias is systematic — every stem is displaced toward wherever it was seen
    from — so it does not average out over a stand. It is the single largest error
    term when the detector uses a plain centroid.
    """
    truth = _map([(100.0, 100.0)], radius=0.30)
    cloud = simulate_scan(
        truth, returns_per_stem_m2=400.0, visible_arc_fraction=0.5, range_noise_m=0.0, seed=3
    )
    found, _ = detect_stems(cloud, _flat(), _grid())
    fitted_error = float(np.hypot(found.x[0] - 100.0, found.y[0] - 100.0))

    slab = cloud[(cloud[:, 2] >= 1.5) & (cloud[:, 2] <= 5.0)]
    centroid_error = float(np.hypot(slab[:, 0].mean() - 100.0, slab[:, 1].mean() - 100.0))

    assert centroid_error == pytest.approx(2 * 0.30 / np.pi, abs=0.02)  # the analytic bias
    assert fitted_error < centroid_error / 3.0


def test_heights_are_taken_from_the_TERRAIN_under_each_stem() -> None:
    """Trunk space is AGL. On a slope an absolute slice cuts different parts of
    different trees — the same per-column datum error that broke the M2 band."""
    truth = _map([(60.0, 100.0), (140.0, 100.0)])
    dtm = _flat()
    grid = _grid()
    # Put 50 m of ground under the second stem, and raise its returns to match.
    ground = np.array([0.0, 50.0])
    for col in range(SIZE):
        if grid.pixel_to_world(0, col).x > 100.0:
            dtm[:, col] = 50.0
    cloud = simulate_scan(truth, ground_elev=ground, returns_per_stem_m2=200.0, seed=5)

    found, _ = detect_stems(cloud, dtm, grid)
    assert len(found) == 2, "the raised stem was missed — slice is not height-normalized"
    # Without the DTM the raised stem's returns are 50 m above the slice.
    naive, _ = detect_stems(cloud, _flat(), grid)
    assert len(naive) == 1


def test_understory_clutter_does_not_become_phantom_stems() -> None:
    truth = _map([(100.0, 100.0)])
    cloud = simulate_scan(
        truth,
        returns_per_stem_m2=200.0,
        understory_points=20000,
        understory_top_m=1.4,
        area_bounds=(50.0, 50.0, 150.0, 150.0),
        seed=2,
    )
    found, _ = detect_stems(cloud, _flat(), _grid())
    assert len(found) == 1


def test_a_thin_blob_is_rejected_because_a_stem_spans_the_slice() -> None:
    """A rock or a bush makes a horizontally-tight cluster with no vertical extent."""
    rng = np.random.default_rng(0)
    blob = np.column_stack(
        [
            100.0 + rng.normal(0, 0.05, 60),
            100.0 + rng.normal(0, 0.05, 60),
            np.full(60, 2.0) + rng.normal(0, 0.02, 60),
        ]
    )
    found, stats = detect_stems(blob, _flat(), _grid())
    assert len(found) == 0
    assert stats.rejected_thin == 1


def test_a_sparse_cluster_is_rejected() -> None:
    rng = np.random.default_rng(0)
    few = np.column_stack(
        [100.0 + rng.normal(0, 0.05, 3), 100.0 + rng.normal(0, 0.05, 3), [2.0, 3.0, 4.0]]
    )
    found, stats = detect_stems(few, _flat(), _grid())
    assert len(found) == 0
    assert stats.rejected_few_points == 1


def test_neighbouring_stems_stay_separate() -> None:
    """At 4.6 m median spacing the clusters are far apart; merging them would put a
    phantom centre in the gap the aircraft is meant to fly through."""
    truth = _map([(98.0, 100.0), (103.0, 100.0)])
    cloud = simulate_scan(truth, returns_per_stem_m2=200.0, seed=4)
    found, _ = detect_stems(cloud, _flat(), _grid())
    assert len(found) == 2
    assert float(np.hypot(found.x[0] - found.x[1], found.y[0] - found.y[1])) == pytest.approx(
        5.0, abs=0.2
    )


def test_an_empty_cloud_detects_nothing_without_raising() -> None:
    found, stats = detect_stems(np.empty((0, 3)), _flat(), _grid())
    assert len(found) == 0
    assert stats.points_in == 0


def test_a_badly_shaped_array_is_rejected() -> None:
    with pytest.raises(ValueError, match=r"\(n, 3\)"):
        detect_stems(np.zeros((5, 2)), _flat(), _grid())


# --- Scoring a map against truth ---------------------------------------------------


def test_match_counts_missed_and_spurious_separately() -> None:
    """They are not symmetric: a MISSED stem is an obstacle the planner flies into."""
    truth = _map([(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)])
    detected = _map([(0.1, 0.0), (50.0, 50.0)])  # one hit, one phantom, two missed
    report = match_stems(detected, truth, tolerance_m=1.0)
    assert report.matched == 1
    assert report.missed == 2
    assert report.spurious == 1
    assert report.position_rmse_m == pytest.approx(0.1, abs=1e-6)


def test_matching_is_one_to_one() -> None:
    """One detection cannot satisfy two truth stems."""
    truth = _map([(0.0, 0.0), (0.4, 0.0)])
    detected = _map([(0.2, 0.0)])
    report = match_stems(detected, truth, tolerance_m=1.0)
    assert report.matched == 1
    assert report.missed == 1


def test_an_empty_detection_is_all_misses_not_a_perfect_score() -> None:
    truth = _map([(0.0, 0.0), (10.0, 0.0)])
    report = match_stems(_map([]), truth)
    assert report.matched == 0
    assert report.missed == 2
    assert report.detection_rate == 0.0


def test_the_detector_recovers_a_stand_end_to_end() -> None:
    """The acceptance case: a realistic patch, scored on position, not just count."""
    rng = np.random.default_rng(11)
    pts = rng.uniform(60.0, 140.0, size=(40, 2))
    truth = StemMap(x=pts[:, 0], y=pts[:, 1], radius_m=np.full(40, 0.15))
    cloud = simulate_scan(truth, returns_per_stem_m2=100.0, range_noise_m=0.02, seed=12)
    found, _ = detect_stems(cloud, _flat(), _grid(), StemDetectConfig())
    report = match_stems(found, truth, tolerance_m=1.0)
    assert report.detection_rate > 0.9
    # Flown clearance on the stem-routed sortie was 0.708 m; the map must not eat it.
    assert report.position_rmse_m < 0.10
