"""Route planners (Software-Spec §5.3).

M0: ``straight_line_plan`` (stub). M1+ will add the Theta* corridor search and
the OMPL 3D refinement inside the forest-navigable corridor.
"""

from __future__ import annotations

from droneplanner.planner.stub import straight_line_plan

__all__ = ["straight_line_plan"]
