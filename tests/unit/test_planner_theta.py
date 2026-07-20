"""Unit tests for the cost-aware Theta* planner (Software-Spec §5.3, §6.2)."""

from __future__ import annotations

import numpy as np
import pytest

from droneplanner.planner.theta_star import PlanningError, path_cost, plan_path


def test_uniform_cost_is_near_straight() -> None:
    cost = np.ones((20, 20), dtype=np.float32)
    path = plan_path(cost, (0, 0), (19, 19))
    assert path[0] == (0, 0)
    assert path[-1] == (19, 19)
    # Any-angle over uniform cost collapses to a straight line (few vertices).
    assert len(path) <= 4


def test_detours_around_high_cost_barrier() -> None:
    cost = np.ones((21, 21), dtype=np.float32)
    cost[0:16, 10] = 1000.0  # vertical wall with a gap at rows 16..20
    path = plan_path(cost, (8, 0), (8, 20))
    assert path[0] == (8, 0)
    assert path[-1] == (8, 20)
    # The route must avoid the expensive wall cells.
    max_cell = max(cost[r, c] for r, c in path)
    assert max_cell < 1000.0


def test_obstacle_gap_is_threaded() -> None:
    cost = np.ones((21, 21), dtype=np.float32)
    obstacle = np.zeros((21, 21), dtype=bool)
    obstacle[0:16, 10] = True  # wall with a gap at the bottom
    path = plan_path(cost, (8, 0), (8, 20), obstacle=obstacle)
    assert not any(obstacle[r, c] for r, c in path)
    assert path[-1] == (8, 20)


def test_fully_blocked_raises() -> None:
    cost = np.ones((11, 11), dtype=np.float32)
    obstacle = np.zeros((11, 11), dtype=bool)
    obstacle[:, 5] = True  # full wall, no gap
    with pytest.raises(PlanningError, match="no path"):
        plan_path(cost, (5, 0), (5, 10), obstacle=obstacle)


def test_start_on_obstacle_raises() -> None:
    cost = np.ones((5, 5), dtype=np.float32)
    obstacle = np.zeros((5, 5), dtype=bool)
    obstacle[0, 0] = True
    with pytest.raises(PlanningError, match="obstacle cell"):
        plan_path(cost, (0, 0), (4, 4), obstacle=obstacle)


def test_path_cost_matches_cheaper_route() -> None:
    cost = np.ones((21, 21), dtype=np.float32)
    cost[0:16, 10] = 1000.0
    detour = plan_path(cost, (8, 0), (8, 20))
    straight = [(8, 0), (8, 20)]
    assert path_cost(cost, detour) < path_cost(cost, straight)
