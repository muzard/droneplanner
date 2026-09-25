"""Flying OVER a stand that cannot be flown under — and telling the operator first.

User, 2026-09-25: "do some kind of system for it to rise over the canopy if it looks
unpassable below. The operator should also be informed of that beforehand, so no
surprises happen and the drone jumps up to height right from the launch site."
"""

from __future__ import annotations

import numpy as np

from droneplanner.fields.composite import _regularise_forced, build_cost_surface
from droneplanner.model.grid import Grid
from droneplanner.model.plan import FlightBand
from droneplanner.planner.corridor import plan_corridor
from droneplanner.report.build import build_report
from tests.fixtures.synthetic import make_mission

CELL = 25.0
N = 41


def _ao(dense_cols: slice) -> tuple[np.ndarray, np.ndarray, Grid]:
    """Open-structured 25 m forest (~420 stems/ha, flyable at 2 m/s) with a strip of
    13 m thicket (~1300 stems/ha, beyond the envelope) across the whole AO."""
    grid = Grid(transform=(CELL, 0.0, 0.0, 0.0, -CELL, N * CELL), rows=N, cols=N, crs="EPSG:32633")
    dtm = np.zeros((N, N), dtype=np.float32)
    dsm = np.full((N, N), 25.0, dtype=np.float32)
    dsm[:, dense_cols] = 13.0
    return dtm, dsm, grid


def _plan(dense_cols: slice, *, fallback: bool):  # type: ignore[no-untyped-def]
    dtm, dsm, grid = _ao(dense_cols)
    start = grid.pixel_to_world(N // 2, 1)
    end = grid.pixel_to_world(N // 2, N - 2)
    mission = make_mission(
        name="thicket-ao",
        start={"x": start.x, "y": start.y},
        end={"x": end.x, "y": end.y},
        waypoint_spacing_m=CELL,
        cruise_agl_m=8.0,
    )
    mission = mission.model_copy(
        update={
            "altitude": mission.altitude.model_copy(
                update={"over_canopy_when_unpassable": fallback}
            )
        }
    )
    surface = build_cost_surface(dtm, dsm, grid, mission)
    plan = plan_corridor(mission, surface)
    return mission, plan, surface


def test_a_thicket_is_flown_over_not_through_and_reported() -> None:
    mission, plan, _ = _plan(slice(20, 23), fallback=True)
    assert plan.objective_scores["below_canopy_beyond_envelope_m"] == 0.0
    bands = [v.flight_band for v in plan.vertices]
    assert FlightBand.BELOW_CANOPY in bands  # still under the good forest
    assert FlightBand.CANOPY_TOP in bands  # and over the thicket
    [climb] = plan.canopy_climbs
    assert climb.reason == "dense_stand"
    # The strip is ~19 cells in; padded by a cell each side and started from col 1.
    assert 350.0 < climb.start_m < 480.0
    assert climb.max_agl_m >= 13.0 + mission.altitude.canopy_clearance_m - 1e-3
    report = build_report(mission, plan, generated_at=None)
    [finding] = [f for f in report.findings if f.code == "canopy_climb_in_zone"]
    assert "FIRST CLIMB" not in finding.message  # not near launch
    assert report.canopy_climbs == plan.canopy_climbs
    assert not any(f.code == "stem_density_beyond_envelope" for f in report.findings)


def test_a_climb_right_after_launch_is_called_out_first() -> None:
    mission, plan, _ = _plan(slice(3, 6), fallback=True)
    assert plan.canopy_climbs and plan.canopy_climbs[0].start_m <= 200.0
    report = build_report(mission, plan, generated_at=None)
    [finding] = [f for f in report.findings if f.code == "canopy_climb_in_zone"]
    assert finding.message.startswith("FIRST CLIMB")


def test_switched_off_it_plans_through_and_says_no_go() -> None:
    mission, plan, _ = _plan(slice(20, 23), fallback=False)
    assert plan.objective_scores["below_canopy_beyond_envelope_m"] > 0.0
    report = build_report(mission, plan, generated_at=None)
    assert any(f.code == "stem_density_beyond_envelope" for f in report.findings)


def test_forced_mask_closes_small_gaps_and_pads_so_the_aircraft_does_not_porpoise() -> None:
    m = np.zeros((1, 12), dtype=bool)
    m[0, [3, 4, 7]] = True  # two blocks with a 2-cell gap
    out = _regularise_forced(np.repeat(m, 5, axis=0))[2]
    assert out[2:9].all()  # gap closed, padded one cell each side
    assert not out[0] and not out[10:].any()


def test_the_report_states_how_high_an_in_flight_escape_could_go() -> None:
    """The escape climb is decided in flight, but its worst case is known before
    launch: the tallest canopy near any below-canopy vertex, plus the margin."""
    mission, plan, _ = _plan(slice(20, 23), fallback=True)
    assert plan.objective_scores["escape_max_agl_m"] >= 25.0 + 3.0 - 1e-3
    report = build_report(mission, plan, generated_at=None)
    [finding] = [f for f in report.findings if f.code == "escape_climb_policy"]
    assert "STRAIGHT UP" in finding.message
