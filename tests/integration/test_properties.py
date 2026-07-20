"""Property-based invariants for the planner (Software-Spec §6.3).

Stub (M0) and 2.5D corridor (M1) invariants; the forest / 3D-navigability
invariants land with M2.
"""

from __future__ import annotations

import math
from itertools import pairwise

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st

from droneplanner.planner.stub import straight_line_plan
from droneplanner.planner.theta_star import plan_path
from droneplanner.raster_lines import bresenham
from tests.fixtures.synthetic import make_mission

_coord = st.floats(min_value=-1e5, max_value=1e5, allow_nan=False, allow_infinity=False)


@given(
    dx=st.floats(min_value=-5000, max_value=5000, allow_nan=False),
    dy=st.floats(min_value=-5000, max_value=5000, allow_nan=False),
    spacing=st.floats(min_value=10.0, max_value=500.0),
)
def test_path_starts_and_ends_at_endpoints(dx: float, dy: float, spacing: float) -> None:
    # Require a non-degenerate start/end (mission validation forbids coincident).
    if math.hypot(dx, dy) < 1.0:
        return
    m = make_mission(
        start={"x": 500000.0, "y": 5200000.0},
        end={"x": 500000.0 + dx, "y": 5200000.0 + dy},
        waypoint_spacing_m=spacing,
    )
    plan = straight_line_plan(m)
    assert plan.start == m.start
    assert plan.end == m.end


@given(spacing=st.floats(min_value=10.0, max_value=2000.0))
def test_segment_lengths_sum_to_total(spacing: float) -> None:
    m = make_mission(waypoint_spacing_m=spacing)
    plan = straight_line_plan(m)
    assert math.isclose(
        sum(s.length_m for s in plan.segments), plan.total_length_m, rel_tol=1e-9, abs_tol=1e-6
    )


@given(spacing=st.floats(min_value=10.0, max_value=2000.0))
def test_no_segment_longer_than_spacing(spacing: float) -> None:
    m = make_mission(waypoint_spacing_m=spacing)
    plan = straight_line_plan(m)
    # Uniform sampling: each sub-segment must be <= the requested spacing (+eps).
    assert all(s.length_m <= spacing + 1e-6 for s in plan.segments)


@given(
    length=st.floats(min_value=100.0, max_value=20000.0),
    slack=st.floats(min_value=1.0, max_value=1.5),
)
def test_fiber_used_scales_with_slack(length: float, slack: float) -> None:
    m = make_mission(
        start={"x": 0.0, "y": 0.0},
        end={"x": length, "y": 0.0},
        fiber_slack_factor=slack,
    )
    plan = straight_line_plan(m)
    assert math.isclose(plan.fiber_used_m, plan.total_length_m * slack, rel_tol=1e-9)


# --- M1 corridor (cost-aware Theta*) invariants ---


@given(
    seed=st.integers(min_value=0, max_value=50),
    sr=st.integers(min_value=0, max_value=19),
    gr=st.integers(min_value=0, max_value=19),
)
@settings(max_examples=25, deadline=None)
def test_theta_path_endpoints_and_contiguity(seed: int, sr: int, gr: int) -> None:
    rng = np.random.default_rng(seed)
    cost = (rng.random((20, 20)).astype(np.float32) * 5.0) + 0.1
    start, goal = (sr, 0), (gr, 19)
    path = plan_path(cost, start, goal)
    assert path[0] == start
    assert path[-1] == goal
    # Consecutive any-angle vertices are connected by a valid grid line.
    for a, b in pairwise(path):
        assert bresenham(a[0], a[1], b[0], b[1])[0] == a


@given(gap_row=st.integers(min_value=15, max_value=20))
@settings(max_examples=10, deadline=None)
def test_theta_never_enters_obstacle(gap_row: int) -> None:
    cost = np.ones((21, 21), dtype=np.float32)
    obstacle = np.zeros((21, 21), dtype=bool)
    obstacle[0:gap_row, 10] = True  # wall with a gap below gap_row
    path = plan_path(cost, (5, 0), (5, 20), obstacle=obstacle)
    assert not any(obstacle[r, c] for r, c in path)
