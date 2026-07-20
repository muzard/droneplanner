"""Road / linear-obstacle crossing handling (Software-Spec §5.4).

M1.1 stub: roads are open terrain the drone flies over. ``rasterize_roads``
builds an on-road mask (fed into the cost surface as an open-terrain penalty so
the planner minimizes on-road distance), and ``detect_crossings`` flags each
route/road intersection as an OPEN crossing for the risk report. The full
feasibility-ranked solver (culvert / canopy bridge / overhead span) is future
work.
"""

from __future__ import annotations

from droneplanner.crossings.detect import detect_crossings
from droneplanner.crossings.roads import rasterize_roads

# On-road cells are open/exposed: this per-cell penalty makes the planner spend
# minimal distance on a road (cross roughly perpendicular).
ROAD_COST = 3.0

__all__ = ["ROAD_COST", "detect_crossings", "rasterize_roads"]
