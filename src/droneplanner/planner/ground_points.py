"""Exposure of the places where people stand: launch and end (roadmap P3).

WHY THIS EXISTS
Every concealment number the planner reports is about the *aircraft* (``exposure``
at flight altitude) or the *cable* (``ground_los``). Neither one says anything
about the launch site, where a crew stands for minutes preparing, arming and
recovering, or about the end point, where the fiber terminates and someone may
have to go. The doctrine research names those ground points as what actually gets
found and shelled: a well-concealed route that starts in an open field in view of
the observer has given away its launch site.

WHAT IS SCORED
A person-sized target (:data:`CREW_TARGET_SPAN_M` at :data:`CREW_CENTRE_HEIGHT_M`
above the DTM), through exactly the occlusion, canopy, sector and range rules
that score the aircraft — so the two can never disagree about the adversary:

* ``*_crew_exposure`` — with the **detection model**: a mean over observers in
  [0, 1], read like ``exposure``.
* ``*_crew_los`` — the same sightlines **without** the range term. A site hidden
  only by range today is seen by the first magnified optic on that bearing, so
  the report says which kind of hidden it is.

Not the ``ground_los`` raster: that scores the DSM *surface* (the top of whatever
the cable lies on), so a launch site in 4 m scrub reads fully open there while a
person standing in the scrub is behind it.

⚠️ A standing person is a *better* visual target than a 0.9 m airframe against
forest, and the crew is there for minutes, not seconds; nothing here models
dwell time, movement, vehicles, thermal signature or emissions (the operator's
radio). So the crew number is a floor on how findable the site is, not a ceiling.
⚠️ Relay points (``planner/relay.py``) are the same kind of place but are not on
the emitted plan, so they are scored by ``tools/relay_study.py`` with
:func:`score_ground_points`, not here.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission

CREW_TARGET_SPAN_M = 1.7
"""Size the detection model resolves for a standing person (height)."""

CREW_CENTRE_HEIGHT_M = 1.0
"""Height above the DTM at which a standing person is scored (body centre)."""

GROUND_POINT_EXPOSED = 0.15
"""Crew exposure above which the report warns. The same 0.15 the bearing sweeps
use as the acceptance cap for the aircraft (CLAUDE.md, crown transmission)."""

Scorer = Callable[[list[tuple[float, float, float]], bool], np.ndarray]
"""``(points, with_detection) -> weights`` for a person-sized target."""


def score_ground_points(
    points: Sequence[Coordinate],
    dtm: np.ndarray,
    grid: Grid,
    scorer: Scorer,
) -> list[tuple[float, float]]:
    """``(crew_exposure, crew_los)`` for each ground point."""
    if not points:
        return []
    xyz = [(p.x, p.y, float(dtm[grid.world_to_pixel(p)]) + CREW_CENTRE_HEIGHT_M) for p in points]
    seen = scorer(xyz, True)
    los = scorer(xyz, False)
    return [(float(seen[i]), float(los[i])) for i in range(len(points))]


def ground_point_scores(
    mission: Mission,
    dtm: np.ndarray,
    grid: Grid,
    scorer: Scorer,
) -> dict[str, float]:
    """Objective scores for the launch and end points of a plan."""
    (lc, ll), (ec, el) = score_ground_points([mission.start, mission.end], dtm, grid, scorer)
    return {
        "launch_crew_exposure": lc,
        "launch_crew_los": ll,
        "end_crew_exposure": ec,
        "end_crew_los": el,
    }
