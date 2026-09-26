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


def test_the_report_says_where_an_escape_is_likely_not_just_how_high() -> None:
    """SITL 2026-09-26: past ~1000 stems/ha one traverse in four met a gap it could not
    thread and escaped. The briefing names the below-canopy length in such stands."""
    mission, plan, _ = _plan(slice(20, 23), fallback=True)
    assert plan.objective_scores["escape_likely_length_m"] == 0.0  # thicket flown over
    report = build_report(mission, plan, generated_at=None)
    [finding] = [f for f in report.findings if f.code == "escape_climb_policy"]
    assert "denser than 1000" not in finding.message

    mission, plan, _ = _plan(slice(20, 23), fallback=False)  # planned through it
    assert plan.objective_scores["escape_likely_length_m"] >= 2 * CELL
    report = build_report(mission, plan, generated_at=None)
    [finding] = [f for f in report.findings if f.code == "escape_climb_policy"]
    assert "denser than 1000 stems/ha" in finding.message


def test_a_climb_moves_back_to_a_canopy_opening_and_an_unopened_one_is_reported() -> None:
    """The aircraft cannot climb through closed crowns (2026-09-25): an under->over
    transition is moved back along the route to the nearest opening, and one with
    none in reach is counted for the report."""
    from droneplanner.fields.composite import build_cost_surface

    def climbs(opening_col: int | None):  # type: ignore[no-untyped-def]
        dtm, dsm, grid = _ao(slice(20, 23))
        if opening_col is not None:
            dsm[:, opening_col] = 0.0  # a cleared strip across the AO (a strip road)
        start = grid.pixel_to_world(N // 2, 1)
        end = grid.pixel_to_world(N // 2, N - 2)
        m = make_mission(
            name="t",
            start={"x": start.x, "y": start.y},
            end={"x": end.x, "y": end.y},
            waypoint_spacing_m=CELL,
            cruise_agl_m=8.0,
        )
        plan = plan_corridor(m, build_cost_surface(dtm, dsm, grid, m))
        return m, plan

    _, closed = climbs(None)
    assert closed.objective_scores["canopy_transitions_without_opening"] == 2.0  # in and out
    m, opened = climbs(15)
    assert opened.objective_scores["canopy_transitions_without_opening"] == 1.0  # out only
    assert opened.canopy_climbs[0].start_m < closed.canopy_climbs[0].start_m  # moved back
    report = build_report(m, opened, generated_at=None)
    assert any(f.code == "canopy_transition_no_opening" for f in report.findings)


def test_the_markdown_report_opens_with_an_operator_briefing() -> None:
    from droneplanner.io.report import report_to_markdown

    mission, plan, _ = _plan(slice(3, 6), fallback=True)
    md = report_to_markdown(build_report(mission, plan, generated_at=None))
    briefing = md.split("## Summary")[0]
    assert "## Operator briefing" in briefing
    assert "| 1 |" in briefing  # the climb table
    assert "FIRST CLIMB" in briefing


def test_flight_inside_the_crown_layer_is_reported_unless_crown_ratio_keeps_it_under() -> None:
    """2026-09-26: at 12 m AGL under a ~16 m canopy the aircraft is inside live crowns
    (crown base ~8 m) — branches to hit and little crown overhead for cover. The report
    says so while crown_ratio (opt-in) is off, and stays quiet once it is on."""
    grid = Grid(transform=(CELL, 0.0, 0.0, 0.0, -CELL, N * CELL), rows=N, cols=N, crs="EPSG:32633")
    dtm = np.zeros((N, N), dtype=np.float32)
    dsm = np.full((N, N), 16.0, dtype=np.float32)
    start = grid.pixel_to_world(N // 2, 1)
    end = grid.pixel_to_world(N // 2, N - 2)
    base = make_mission(
        name="crowns-ao",
        start={"x": start.x, "y": start.y},
        end={"x": end.x, "y": end.y},
        waypoint_spacing_m=CELL,
        cruise_agl_m=12.0,
    )
    for ratio, expect in ((None, True), (0.5, False)):
        mission = base.model_copy(
            update={"altitude": base.altitude.model_copy(update={"crown_ratio": ratio})}
        )
        plan = plan_corridor(mission, build_cost_surface(dtm, dsm, grid, mission))
        report = build_report(mission, plan, generated_at=None)
        found = [f for f in report.findings if f.code == "flight_in_crown_layer"]
        assert bool(found) is expect, ratio
        if expect:
            assert plan.objective_scores["below_canopy_in_crown_layer_m"] > 10 * CELL
            from droneplanner.io.report import report_to_markdown

            briefing = report_to_markdown(report).split("## Summary")[0]
            assert "Flying inside the crowns" in briefing


def test_an_observer_standing_in_a_stand_is_flagged_under_crown_transmission() -> None:
    """2026-09-26: with crown transmission the eye is on the ground, so an observer placed
    inside vegetation taller than its eye looks out through it and sees little. The report
    says so — observer placement now decides what they see."""
    from droneplanner.model.geometry import Coordinate
    from droneplanner.model.mission import CanopyOcclusion

    grid = Grid(transform=(CELL, 0.0, 0.0, 0.0, -CELL, N * CELL), rows=N, cols=N, crs="EPSG:32633")
    dtm = np.zeros((N, N), dtype=np.float32)
    dsm = np.full((N, N), 16.0, dtype=np.float32)
    start = grid.pixel_to_world(N // 2, 1)
    end = grid.pixel_to_world(N // 2, N - 2)
    base = make_mission(
        name="observer-in-stand",
        start={"x": start.x, "y": start.y},
        end={"x": end.x, "y": end.y},
        waypoint_spacing_m=CELL,
        cruise_agl_m=6.0,
    )
    eye = grid.pixel_to_world(5, 20)
    for transmission, expect in ((True, True), (False, False)):
        threat = base.threat.model_copy(
            update={
                "observers": [Coordinate(x=eye.x, y=eye.y)],
                "canopy_occlusion": CanopyOcclusion(crown_transmission=transmission),
            }
        )
        mission = base.model_copy(update={"threat": threat})
        plan = plan_corridor(mission, build_cost_surface(dtm, dsm, grid, mission))
        report = build_report(mission, plan, generated_at=None)
        assert any(f.code == "observers_in_vegetation" for f in report.findings) is expect


def test_an_observer_with_a_vantage_radius_steps_out_to_the_nearest_clearing() -> None:
    """CanopyOcclusion.observer_vantage_m: an observer placed in a stand stands at the
    nearest open cell within the radius — the warning goes quiet and it sees more."""
    from droneplanner.fields.altitude import canopy_height
    from droneplanner.fields.composite import vantage_mission
    from droneplanner.model.geometry import Coordinate
    from droneplanner.model.mission import CanopyOcclusion

    grid = Grid(transform=(CELL, 0.0, 0.0, 0.0, -CELL, N * CELL), rows=N, cols=N, crs="EPSG:32633")
    dtm = np.zeros((N, N), dtype=np.float32)
    dsm = np.full((N, N), 16.0, dtype=np.float32)
    dsm[3:8, 23:26] = 0.0  # a clearing 3 cells (75 m) east of the observer
    start = grid.pixel_to_world(N // 2, 1)
    end = grid.pixel_to_world(N // 2, N - 2)
    base = make_mission(
        name="vantage",
        start={"x": start.x, "y": start.y},
        end={"x": end.x, "y": end.y},
        waypoint_spacing_m=CELL,
        cruise_agl_m=6.0,
    )
    eye = grid.pixel_to_world(5, 20)
    results = {}
    for vantage in (0.0, 100.0):
        threat = base.threat.model_copy(
            update={
                "observers": [Coordinate(x=eye.x, y=eye.y)],
                "observer_height_m": 2.0,
                "canopy_occlusion": CanopyOcclusion(
                    crown_transmission=True, observer_vantage_m=vantage
                ),
            }
        )
        mission = base.model_copy(update={"threat": threat})
        moved = vantage_mission(mission, canopy_height(dtm, dsm), grid).threat.observers[0]
        assert grid.world_to_pixel(moved) == ((5, 20) if vantage == 0.0 else (5, 23))
        surface = build_cost_surface(dtm, dsm, grid, mission)
        plan = plan_corridor(mission, surface)
        report = build_report(mission, plan, generated_at=None)
        warned = any(f.code == "observers_in_vegetation" for f in report.findings)
        assert warned is (vantage == 0.0)
        results[vantage] = float(surface.components["exposure"].mean())
    assert results[100.0] > results[0.0]  # out of the stand it sees further
