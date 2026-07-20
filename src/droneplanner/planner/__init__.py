"""Route planners (Software-Spec §5.3).

M0: ``straight_line_plan`` (stub). M1: ``plan_corridor`` — the any-angle Theta*
concealment-aware corridor search over the composite cost surface. M2+ adds the
OMPL 3D refinement inside the forest-navigable corridor.
"""

from __future__ import annotations

from droneplanner.planner.corridor import plan_corridor
from droneplanner.planner.stub import straight_line_plan
from droneplanner.planner.theta_star import PlanningError, path_cost, plan_path

__all__ = [
    "PlanningError",
    "path_cost",
    "plan_corridor",
    "plan_path",
    "straight_line_plan",
]
