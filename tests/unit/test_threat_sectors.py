"""Unit tests for threat fire sectors (roadmap C3, Software-Spec §4/§5.1).

``ThreatSector`` was in the mission contract from M0 and read by nothing, so the
premise "the enemy is over there" could not reach the planner. These tests pin
the two things that makes true: an observer only sees inside its arc, and a
bearing with no observer becomes a terrain-masking openness field.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from droneplanner.demo import build_demo
from droneplanner.fields.sectors import (
    arc_contains,
    arc_span_deg,
    bearing_deg,
    describe_threat_sectors,
    element_mask,
    element_sees_bearing,
    openness_to_threat_sectors,
    resolve_threat_elements,
)
from droneplanner.fields.viewshed import cumulative_exposure, point_exposure
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import DetectionModel, ThreatModel, ThreatSector
from droneplanner.planner.corridor import plan_over_terrain
from droneplanner.planner.stub import straight_line_plan
from droneplanner.raster_lines import bresenham
from droneplanner.report.build import build_report
from tests.fixtures.synthetic import make_mission


def _flat_grid(size: int, cell_m: float = 10.0) -> Grid:
    transform = (cell_m, 0.0, 0.0, 0.0, -cell_m, size * cell_m)
    return Grid(transform=transform, rows=size, cols=size, crs="EPSG:32633")


# --- Arc geometry ----------------------------------------------------------------


@pytest.mark.parametrize(
    ("azimuth", "start", "end", "inside"),
    [
        (10.0, 350.0, 20.0, True),  # wraparound through north
        (355.0, 350.0, 20.0, True),
        (30.0, 350.0, 20.0, False),
        (180.0, 350.0, 20.0, False),
        (90.0, 45.0, 135.0, True),
        (180.0, 45.0, 135.0, False),
        (270.0, 135.0, 45.0, True),  # the complement arc, swept clockwise
    ],
)
def test_arc_containment_including_wraparound(
    azimuth: float, start: float, end: float, inside: bool
) -> None:
    assert arc_contains(azimuth, start, end) is inside


def test_a_degenerate_arc_is_the_full_circle_not_a_zero_width_one() -> None:
    """The safe reading of an ambiguous input: never silently erase a threat."""
    assert arc_span_deg(90.0, 90.0) == 360.0
    assert arc_span_deg(0.0, 360.0) == 360.0
    assert arc_contains(200.0, 90.0, 90.0)


def test_bearing_is_a_compass_bearing() -> None:
    assert bearing_deg(0.0, 0.0, 0.0, 100.0) == pytest.approx(0.0)  # north
    assert bearing_deg(0.0, 0.0, 100.0, 0.0) == pytest.approx(90.0)  # east
    assert bearing_deg(0.0, 0.0, 0.0, -100.0) == pytest.approx(180.0)  # south
    assert bearing_deg(0.0, 0.0, -100.0, 0.0) == pytest.approx(270.0)  # west


# --- Binding sectors to observers ------------------------------------------------


def test_no_sectors_leaves_every_observer_looking_all_round() -> None:
    observers = [Coordinate(x=0.0, y=0.0), Coordinate(x=100.0, y=0.0)]
    views = resolve_threat_elements(observers, ())
    assert len(views) == 2
    assert all(len(group) == 1 and not group[0].restricted for group in views)


def test_a_sector_replaces_its_observers_full_circle_rather_than_adding_to_it() -> None:
    observer = Coordinate(x=0.0, y=0.0)
    sector = ThreatSector(azimuth_start_deg=0.0, azimuth_end_deg=90.0, observer=observer)
    views = resolve_threat_elements([observer], [sector])
    assert len(views) == 1
    assert [e.arcs for e in views[0]] == [((0.0, 90.0),)]


def test_a_sector_naming_an_unlisted_observer_adds_it_instead_of_dropping_it() -> None:
    """Dropping it would delete a declared threat — the unsafe direction."""
    listed = Coordinate(x=0.0, y=0.0)
    unlisted = Coordinate(x=500.0, y=500.0)
    sector = ThreatSector(azimuth_start_deg=0.0, azimuth_end_deg=90.0, observer=unlisted)
    views = resolve_threat_elements([listed], [sector])
    assert [group[0].observer for group in views] == [listed, unlisted]
    assert not views[0][0].restricted
    assert views[1][0].restricted


def test_two_optics_on_one_observer_are_two_elements_but_still_one_observer() -> None:
    observer = Coordinate(x=0.0, y=0.0)
    sectors = [
        ThreatSector(azimuth_start_deg=0.0, azimuth_end_deg=90.0, observer=observer),
        ThreatSector(
            azimuth_start_deg=180.0,
            azimuth_end_deg=270.0,
            observer=observer,
            detection=DetectionModel(optic_magnification=12.0),
        ),
    ]
    views = resolve_threat_elements([observer], sectors)
    assert len(views) == 1  # one observer
    assert len(views[0]) == 2  # two elements
    assert {e.detection for e in views[0]} == {None, DetectionModel(optic_magnification=12.0)}


def test_arcs_sharing_an_optic_collapse_into_one_element() -> None:
    observer = Coordinate(x=0.0, y=0.0)
    sectors = [
        ThreatSector(azimuth_start_deg=0.0, azimuth_end_deg=45.0, observer=observer),
        ThreatSector(azimuth_start_deg=180.0, azimuth_end_deg=225.0, observer=observer),
    ]
    views = resolve_threat_elements([observer], sectors)
    assert len(views[0]) == 1
    assert views[0][0].arcs == ((0.0, 45.0), (180.0, 225.0))


def test_a_bearing_only_sector_binds_to_no_observer() -> None:
    observer = Coordinate(x=0.0, y=0.0)
    bearing = ThreatSector(azimuth_start_deg=90.0, azimuth_end_deg=180.0)
    views = resolve_threat_elements([observer], [bearing])
    assert len(views) == 1
    assert not views[0][0].restricted


# --- The mask, and its single-point twin -----------------------------------------


def test_the_mask_and_the_point_test_agree_everywhere() -> None:
    """The raster field and the flown re-score must not disagree about facing."""
    size = 21
    grid = _flat_grid(size)
    observer = grid.pixel_to_world(10, 10)
    element = resolve_threat_elements(
        [observer],
        [
            ThreatSector(
                azimuth_start_deg=350.0, azimuth_end_deg=20.0, observer=observer, label="north"
            )
        ],
    )[0][0]
    mask = element_mask(grid, element)
    assert mask is not None
    for r in range(size):
        for c in range(size):
            world = grid.pixel_to_world(r, c)
            assert bool(mask[r, c]) == element_sees_bearing(element, world.x, world.y)


def test_an_unrestricted_element_has_no_mask() -> None:
    grid = _flat_grid(11)
    observer = grid.pixel_to_world(5, 5)
    element = resolve_threat_elements([observer], ())[0][0]
    assert element_mask(grid, element) is None


# --- Exposure under sectors ------------------------------------------------------


def test_empty_sectors_reproduce_the_field_exactly() -> None:
    """Regression guard: C3 is inert until a mission states a posture."""
    size = 24
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, 12] = 40.0
    grid = _flat_grid(size)
    observers = [grid.pixel_to_world(6, 3), grid.pixel_to_world(18, 20)]
    before = cumulative_exposure(dsm, grid, observers)
    after = cumulative_exposure(dsm, grid, observers, sectors=())
    assert np.array_equal(before, after)


def test_a_cell_inside_the_sector_scores_above_an_identical_cell_outside_it() -> None:
    """The acceptance case: two cells the same distance out, on flat ground."""
    size = 21
    dsm = np.zeros((size, size), dtype=np.float32)
    grid = _flat_grid(size)
    observer = grid.pixel_to_world(10, 10)
    # A 90 deg arc centred on east: the cell east of the observer is inside it,
    # the cell west of it is not, and on flat ground the two are otherwise twins.
    sectors = [
        ThreatSector(azimuth_start_deg=45.0, azimuth_end_deg=135.0, observer=observer, label="east")
    ]
    plain = cumulative_exposure(dsm, grid, [observer])
    # vigilance 0.0 is the hard-mask reading this test was written for; the
    # default is now a partially-attentive adversary, tested separately below.
    sectored = cumulative_exposure(dsm, grid, [observer], sectors=sectors, off_axis_vigilance=0.0)

    assert plain[10, 15] == plain[10, 5]  # twins before the sector
    assert sectored[10, 15] == plain[10, 15]  # inside the arc: untouched
    assert sectored[10, 5] == 0.0  # outside it: unseen
    assert sectored.mean() < plain.mean()


def test_a_sector_narrows_attention_and_never_invents_occlusion() -> None:
    """A sector can only remove exposure, never add it."""
    size = 21
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, 12] = 40.0
    grid = _flat_grid(size)
    observers = [grid.pixel_to_world(10, 2)]
    sectors = [
        ThreatSector(azimuth_start_deg=0.0, azimuth_end_deg=180.0, observer=observers[0]),
    ]
    plain = cumulative_exposure(dsm, grid, observers)
    sectored = cumulative_exposure(dsm, grid, observers, sectors=sectors)
    assert np.all(sectored <= plain + 1e-6)


def test_terrain_outside_the_arc_still_occludes_inside_it() -> None:
    """The mask is applied to the answer, not to the march.

    A wall that stands outside the searched arc still blocks the sightline to
    cells inside it — occlusion is geometry and does not care where anyone looks.
    """
    size = 21
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, 10] = 100.0
    grid = _flat_grid(size)
    observer = grid.pixel_to_world(10, 0)
    # Arc covering only the row the observer is on, wide enough to include the
    # cells both in front of and behind the wall.
    sectors = [ThreatSector(azimuth_start_deg=80.0, azimuth_end_deg=100.0, observer=observer)]
    sectored = cumulative_exposure(dsm, grid, [observer], sectors=sectors)
    assert sectored[10, 5] > 0.0  # in front of the wall, inside the arc
    assert sectored[10, 15] == 0.0  # behind the wall, inside the arc, still hidden


def test_a_per_sector_optic_overrides_the_mission_detection_model() -> None:
    """Roadmap C2's live question: is a *magnified* optic searching that bearing?"""
    size = 41
    cell_m = 50.0  # 2 km across, so the unaided 1031 m range actually bites
    dsm = np.zeros((size, size), dtype=np.float32)
    grid = _flat_grid(size, cell_m=cell_m)
    observer = grid.pixel_to_world(20, 20)
    unaided = DetectionModel()
    far_east = (20, 39)  # ~950 m east — inside unaided range but near its edge
    far_west = (20, 1)

    scoped = [
        ThreatSector(
            azimuth_start_deg=45.0,
            azimuth_end_deg=135.0,
            observer=observer,
            detection=DetectionModel(optic_magnification=12.0),
            label="scoped-east",
        ),
        ThreatSector(azimuth_start_deg=225.0, azimuth_end_deg=315.0, observer=observer),
    ]
    field = cumulative_exposure(
        dsm,
        grid,
        [observer],
        detection=unaided,
        target_span_m=0.9,
        target_elev_m=dsm + 12.0,
        sectors=scoped,
    )
    # Same range, same terrain, same target — only the optic differs.
    assert field[far_east] > field[far_west]
    assert field[far_east] == pytest.approx(1.0)


def test_point_exposure_agrees_with_the_raster_under_sectors() -> None:
    size = 21
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, 12] = 30.0
    grid = _flat_grid(size)
    observers = [grid.pixel_to_world(4, 4), grid.pixel_to_world(16, 16)]
    sectors = [
        ThreatSector(azimuth_start_deg=45.0, azimuth_end_deg=200.0, observer=observers[0]),
        ThreatSector(azimuth_start_deg=250.0, azimuth_end_deg=20.0, observer=observers[1]),
    ]
    target = dsm + 15.0
    raster = cumulative_exposure(dsm, grid, observers, target_elev_m=target, sectors=sectors)

    cells = [(r, c) for r in range(2, size, 5) for c in range(2, size, 5)]
    points = []
    for r, c in cells:
        world = grid.pixel_to_world(r, c)
        points.append((world.x, world.y, float(target[r, c])))
    at_points = point_exposure(dsm, grid, observers, points, sectors=sectors)
    for (r, c), value in zip(cells, at_points, strict=True):
        assert value == pytest.approx(float(raster[r, c]), abs=1e-6)


def test_an_unlisted_sector_observer_reaches_the_field() -> None:
    size = 21
    dsm = np.zeros((size, size), dtype=np.float32)
    grid = _flat_grid(size)
    unlisted = grid.pixel_to_world(10, 10)
    sectors = [ThreatSector(azimuth_start_deg=45.0, azimuth_end_deg=135.0, observer=unlisted)]
    field = cumulative_exposure(dsm, grid, [], sectors=sectors, off_axis_vigilance=0.0)
    assert field[10, 15] == pytest.approx(1.0)
    assert field[10, 5] == 0.0


# --- Bearing-only sectors: openness to a threat azimuth ---------------------------


def test_openness_with_no_sectors_is_zero() -> None:
    grid = _flat_grid(16)
    dsm = np.zeros((16, 16), dtype=np.float32)
    assert np.all(openness_to_threat_sectors(dsm, grid, ()) == 0.0)


def test_flat_ground_is_fully_open_to_every_bearing() -> None:
    grid = _flat_grid(16)
    dsm = np.zeros((16, 16), dtype=np.float32)
    sectors = [ThreatSector(azimuth_start_deg=0.0, azimuth_end_deg=90.0)]
    assert np.all(openness_to_threat_sectors(dsm, grid, sectors) == pytest.approx(1.0))


def test_a_ridge_masks_the_ground_behind_it_from_that_bearing_only() -> None:
    """The reverse-slope case Preflight-Design §5.1 is asking for."""
    size = 21
    grid = _flat_grid(size)
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, 10] = 60.0  # a north-south wall
    east = [ThreatSector(azimuth_start_deg=80.0, azimuth_end_deg=100.0, label="east")]
    west = [ThreatSector(azimuth_start_deg=260.0, azimuth_end_deg=280.0, label="west")]
    to_east = openness_to_threat_sectors(dsm, grid, east, horizon_m=2000.0)
    to_west = openness_to_threat_sectors(dsm, grid, west, horizon_m=2000.0)

    # A cell west of the wall is in defilade from an eastern threat and exposed
    # to a western one; the cell east of the wall is the mirror image.
    assert to_east[10, 5] == 0.0
    assert to_west[10, 5] == pytest.approx(1.0)
    assert to_west[10, 15] == 0.0
    assert to_east[10, 15] == pytest.approx(1.0)


def test_the_horizon_bounds_what_is_credited_as_cover() -> None:
    """A ridge beyond the horizon stops counting — the field's live assumption."""
    size = 41
    grid = _flat_grid(size)  # 10 m cells
    dsm = np.zeros((size, size), dtype=np.float32)
    dsm[:, 30] = 60.0
    sectors = [ThreatSector(azimuth_start_deg=80.0, azimuth_end_deg=100.0)]
    near = openness_to_threat_sectors(dsm, grid, sectors, horizon_m=300.0)
    far = openness_to_threat_sectors(dsm, grid, sectors, horizon_m=50.0)
    assert near[20, 10] == 0.0  # the wall is 200 m east: inside the horizon
    assert far[20, 10] == pytest.approx(1.0)  # beyond a 50 m horizon: not credited


def test_openness_matches_a_per_cell_bresenham_reference() -> None:
    """The shifted-array march is an optimization; this is what it must equal.

    The production field steps by rounded offsets so it can work on whole arrays
    at once. The obvious implementation walks a Bresenham ray from each cell.
    They should agree on the overwhelming majority of cells; where they differ it
    is the same half-cell bearing error the A4 viewshed validation measured.
    """
    rng = np.random.default_rng(11)
    size = 25
    grid = _flat_grid(size)
    dsm = rng.uniform(0.0, 40.0, (size, size)).astype(np.float32)
    azimuth = 90.0
    # A hair-wide arc, because a degenerate one is the full circle by convention;
    # both ends dedupe to the same bearing, so exactly one ray is cast.
    produced = openness_to_threat_sectors(
        dsm,
        grid,
        [ThreatSector(azimuth_start_deg=azimuth, azimuth_end_deg=azimuth + 1e-9)],
        horizon_m=1e6,
    )

    # Reference: for each cell, walk a Bresenham line to the grid edge along the
    # bearing and ask whether anything on it stands higher.
    dr = -math.cos(math.radians(azimuth))
    dc = math.sin(math.radians(azimuth))
    reference = np.zeros((size, size), dtype=np.float32)
    for r in range(size):
        for c in range(size):
            far_r = round(r + dr * size * 2)
            far_c = round(c + dc * size * 2)
            far_r = min(max(far_r, 0), size - 1)
            far_c = min(max(far_c, 0), size - 1)
            highest = -np.inf
            for rr, cc in bresenham(r, c, far_r, far_c)[1:]:
                highest = max(highest, float(dsm[rr, cc]))
            reference[r, c] = 1.0 if highest <= float(dsm[r, c]) else 0.0
    agreement = float(np.mean(produced == reference))
    assert agreement > 0.98, f"only {agreement:.1%} agreement with the reference"


# --- What the report says --------------------------------------------------------


def test_every_sector_is_described_including_the_unlisted_observer() -> None:
    listed = Coordinate(x=0.0, y=0.0)
    sectors = [
        ThreatSector(azimuth_start_deg=350.0, azimuth_end_deg=20.0, observer=listed, label="mast"),
        ThreatSector(
            azimuth_start_deg=90.0,
            azimuth_end_deg=180.0,
            observer=Coordinate(x=900.0, y=900.0),
            label="op",
        ),
        ThreatSector(azimuth_start_deg=200.0, azimuth_end_deg=260.0, label="fires"),
    ]
    lines = describe_threat_sectors([listed], sectors)
    assert len(lines) == 3
    assert "mast" in lines[0] and "masking the observer" in lines[0]
    assert "UNLISTED" in lines[1]
    assert "bearing-only" in lines[2]


def test_sectors_supplied_to_a_stub_run_are_a_warning_not_a_silence() -> None:
    """The failure mode C3 exists to close: supplied, and quietly used by nothing."""
    threat = ThreatModel(
        observers=(Coordinate(x=500500.0, y=5200500.0),),
        sectors=(
            ThreatSector(
                azimuth_start_deg=0.0,
                azimuth_end_deg=90.0,
                observer=Coordinate(x=500500.0, y=5200500.0),
                label="ne",
            ),
        ),
    )
    mission = make_mission(threat=threat.model_dump(mode="json"))
    plan = straight_line_plan(mission)

    stub = build_report(mission, plan)
    assert "threat_sectors_ignored" in {f.code for f in stub.findings}
    assert stub.threat_sectors  # still described, so the operator sees what was lost

    analyzed = build_report(mission, plan, concealment_analyzed=True, flyability_analyzed=True)
    codes = {f.code for f in analyzed.findings}
    assert "threat_sectors_applied" in codes
    assert "threat_sectors_ignored" not in codes


def test_a_mission_with_no_sectors_says_nothing_about_them() -> None:
    mission = make_mission()
    report = build_report(mission, straight_line_plan(mission), concealment_analyzed=True)
    assert report.threat_sectors == ()
    assert not any("threat_sector" in f.code for f in report.findings)


# --- End to end: a rear sector changes the route ---------------------------------


def test_a_rear_sector_produces_a_measurably_different_route() -> None:
    """The C3 acceptance case: 'the enemy is behind us' must reach the planner."""
    mission, dtm, dsm, grid = build_demo(size=90)
    all_round, _ = plan_over_terrain(mission, dtm, dsm, grid)

    # Face both observers hard away from the route: a 60 deg arc pointing at the
    # far corner of the AO, which the route does not use.
    away = grid.pixel_to_world(0, 0)
    sectors = tuple(
        ThreatSector(
            azimuth_start_deg=(bearing_deg(obs.x, obs.y, away.x, away.y) - 30.0) % 360.0,
            azimuth_end_deg=(bearing_deg(obs.x, obs.y, away.x, away.y) + 30.0) % 360.0,
            observer=obs,
            label=f"obs{i}",
        )
        for i, obs in enumerate(mission.threat.observers)
    )
    facing_away = mission.model_copy(
        update={"threat": ThreatModel(observers=mission.threat.observers, sectors=sectors)}
    )
    sectored, _ = plan_over_terrain(facing_away, dtm, dsm, grid)

    # An observer that is not looking cannot see: exposure must not rise, and the
    # route is free to take a shorter line through ground it was avoiding.
    assert sectored.objective_scores["mean_exposure"] <= all_round.objective_scores["mean_exposure"]
    assert sectored.objective_scores["path_cost"] != all_round.objective_scores["path_cost"]


# --- Off-axis vigilance: a sector concentrates attention, it does not blind ------
#
# User, 2026-09-14: "realistically observers will be looking around, even by
# accident." A hard mask is the optimistic direction, and a concealment planner
# must never err that way — it lets the route hug ground the adversary is merely
# less likely to be watching and score it as unobservable.

_ARC = (0.0, 90.0)


def _element(vigilance: float):
    obs = Coordinate(x=50.0, y=50.0)
    sector = ThreatSector(azimuth_start_deg=_ARC[0], azimuth_end_deg=_ARC[1], observer=obs)
    return resolve_threat_elements([obs], [sector], vigilance)[0][0]


def test_zero_vigilance_reproduces_the_hard_mask() -> None:
    grid = _flat_grid(11)
    weights = element_mask(grid, _element(0.0))
    assert weights is not None
    assert set(np.unique(weights)).issubset({0.0, 1.0})


def test_off_axis_cells_get_a_reduced_but_nonzero_weight() -> None:
    grid = _flat_grid(11)
    weights = element_mask(grid, _element(0.25))
    assert weights is not None
    assert weights.max() == pytest.approx(1.0)
    assert weights.min() == pytest.approx(0.25)
    # Every cell is either on-axis or off-axis — nothing in between.
    assert set(np.unique(np.round(weights, 6))) == {0.25, 1.0}


def test_full_vigilance_is_an_all_round_threat_and_builds_no_raster() -> None:
    """Attention spread evenly over every bearing is not a restriction at all."""
    element = _element(1.0)
    assert not element.restricted
    assert element_mask(_flat_grid(11), element) is None


def test_the_point_path_applies_the_same_weight_as_the_raster() -> None:
    """The flown re-score and the raster must not disagree about attention."""
    from droneplanner.fields.sectors import element_bearing_weight

    grid = _flat_grid(11)
    element = _element(0.25)
    weights = element_mask(grid, element)
    assert weights is not None
    for x, y in ((90.0, 90.0), (10.0, 10.0), (90.0, 10.0), (10.0, 90.0)):
        rc = grid.world_to_pixel(Coordinate(x=x, y=y))
        assert element_bearing_weight(element, x, y) == pytest.approx(float(weights[rc]))


def test_vigilance_is_still_visible_so_a_sector_cannot_erase_a_threat() -> None:
    element = _element(0.25)
    # Due south of the observer is well outside the 0-90 arc.
    assert element_sees_bearing(element, 50.0, 0.0)
    assert element_sees_bearing(element, 50.0, 0.0) == (
        element_mask(_flat_grid(11), element) is not None
    )


def test_exposure_sits_between_the_hard_mask_and_an_all_round_threat() -> None:
    """The whole point: partial attention is partial exposure, not zero."""
    grid = _flat_grid(21)
    dsm = np.zeros((grid.rows, grid.cols), dtype=np.float64)
    obs = [Coordinate(x=105.0, y=105.0)]
    sector = ThreatSector(azimuth_start_deg=0.0, azimuth_end_deg=90.0, observer=obs[0])

    masked = cumulative_exposure(dsm, grid, obs, sectors=[sector], off_axis_vigilance=0.0)
    partial = cumulative_exposure(dsm, grid, obs, sectors=[sector], off_axis_vigilance=0.25)
    all_round = cumulative_exposure(dsm, grid, obs, sectors=[sector], off_axis_vigilance=1.0)

    assert masked.mean() < partial.mean() < all_round.mean()
    # And it must never promise more cover than the hard mask did.
    assert np.all(partial >= masked - 1e-6)


def test_the_mission_default_is_not_a_hard_mask() -> None:
    """A model that assumed a blind adversary shipped for a month; pin the fix."""
    assert ThreatModel().off_axis_vigilance > 0.0


def test_vigilance_reaches_the_FLOWN_score_not_just_the_raster() -> None:
    """The invariant C3 named, and the one adding vigilance broke for an afternoon.

    ``build_cost_surface`` and ``corridor.py``'s flown re-score are two separate
    call paths into the viewshed. When ``off_axis_vigilance`` was added with a
    default of 0.0, the raster got the mission's value and the flown re-score
    silently kept the hard mask — so a route scored ``mean_exposure`` 0.25 and
    ``mean_exposure_flown`` 0.000 at the same time, which is not a rounding
    disagreement but two different adversaries.
    """
    mission, dtm, dsm, grid = build_demo(size=90)
    away = grid.pixel_to_world(0, 0)
    sectors = tuple(
        ThreatSector(
            azimuth_start_deg=(bearing_deg(obs.x, obs.y, away.x, away.y) - 30.0) % 360.0,
            azimuth_end_deg=(bearing_deg(obs.x, obs.y, away.x, away.y) + 30.0) % 360.0,
            observer=obs,
        )
        for obs in mission.threat.observers
    )

    def scores(vigilance: float) -> tuple[float, float]:
        m = mission.model_copy(
            update={
                "threat": ThreatModel(
                    observers=mission.threat.observers,
                    sectors=sectors,
                    off_axis_vigilance=vigilance,
                )
            }
        )
        plan, _ = plan_over_terrain(m, dtm, dsm, grid)
        return (
            float(plan.objective_scores["mean_exposure"]),
            float(plan.objective_scores["mean_exposure_flown"]),
        )

    blind_raster, blind_flown = scores(0.0)
    watchful_raster, watchful_flown = scores(0.5)

    # Raising vigilance must raise BOTH scores, not just the raster one.
    assert watchful_raster > blind_raster
    assert watchful_flown > blind_flown, (
        "the flown re-score ignored off_axis_vigilance — it is grading the route "
        "against a blind adversary while the raster grades a watchful one"
    )
