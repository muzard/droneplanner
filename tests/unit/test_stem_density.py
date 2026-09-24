"""Unit tests for the stem-clutter speed cap (TEST-PLAN N3).

The chain these pin: canopy height -> stem density -> mean spacing -> the fastest
speed reactive avoidance stays coherent at (TEST-PLAN B2) -> extra seconds per
metre, which is the currency the N1 endurance budget spends.

The demo AO cannot exercise the *routing* half of this — every one of its treed
cells sits deep in the infeasible region — so, as with the A1 corridor's
stay-in-patch clause, the behaviour is proved here rather than by the demo.
"""

from __future__ import annotations

import numpy as np
import pytest

from droneplanner.fields.stem_density import (
    REACTION_TIME_S,
    build_stem_clutter,
    max_reactive_speed_ms,
    mean_stem_spacing_m,
    stem_density_per_ha,
    traversal_time_penalty,
)

CRUISE = 6.0


class TestDensityFromHeight:
    def test_taller_stands_are_sparser(self) -> None:
        """The ordering is the trustworthy part of the allometry (see the module doc)."""
        chm = np.array([[12.0, 16.0, 20.0, 25.0, 30.0]], dtype=np.float32)
        d = stem_density_per_ha(chm, min_canopy_m=2.0)
        assert np.all(np.diff(d[0]) < 0.0)

    def test_reference_anchor_is_reproduced_exactly(self) -> None:
        chm = np.full((1, 1), 20.0, dtype=np.float32)
        assert stem_density_per_ha(chm)[0, 0] == pytest.approx(600.0, rel=1e-4)

    def test_open_ground_has_no_stems(self) -> None:
        chm = np.array([[0.0, 1.0, 11.9, 12.0]], dtype=np.float32)
        d = stem_density_per_ha(chm, min_canopy_m=12.0)
        assert list(d[0][:3]) == [0.0, 0.0, 0.0]
        assert d[0][3] > 0.0

    def test_the_demo_ao_band_lands_where_measured(self) -> None:
        """12-22 m stands -> ~520-1290 stems/ha, which is what the demo reports."""
        chm = np.array([[12.0, 22.0]], dtype=np.float32)
        d = stem_density_per_ha(chm, min_canopy_m=12.0)
        assert d[0, 0] == pytest.approx(1291.0, rel=0.01)
        assert d[0, 1] == pytest.approx(520.0, rel=0.01)


class TestSpacing:
    def test_poisson_spacing_reproduces_the_real_trunk_manifests(self) -> None:
        """Cross-check against ``io/forest_models.py``, an independent generator.

        Its measured median nearest-neighbour spacing was 7.0 m at 50 stems/ha and
        4.6 m at 107. Poisson predicts 7.07 and 4.83 — the manifests cluster
        slightly, so they run a little tighter, which is the expected direction.
        """
        density = np.array([[50.0, 107.0]], dtype=np.float32)
        spacing = mean_stem_spacing_m(density)
        assert spacing[0, 0] == pytest.approx(7.07, abs=0.05)
        assert spacing[0, 1] == pytest.approx(4.83, abs=0.05)

    def test_open_ground_is_unobstructed(self) -> None:
        assert np.isinf(mean_stem_spacing_m(np.zeros((1, 1), dtype=np.float32))[0, 0])


class TestSpeedCap:
    def test_reaction_time_matches_the_droneflight_maneuver_model(self) -> None:
        """The one constant quoted across the repo boundary — pin it or it drifts.

        ``droneflight.maneuver.required_react_distance_m`` measures 5.21 m at
        4.0 m/s and 7.82 m at 6.0 m/s (1.10 m lateral clearance, PX4's
        ``MPC_ACC_HOR`` 3.0 / ``MPC_JERK_AUTO`` 4.0).
        """
        assert pytest.approx(5.21, abs=0.01) == 4.0 * REACTION_TIME_S
        assert pytest.approx(7.82, abs=0.01) == 6.0 * REACTION_TIME_S

    def test_b2_measured_densities_give_the_b2_measured_caps(self) -> None:
        """50 stems/ha -> 5.4 m/s and 107 -> 3.5 m/s, as TEST-PLAN B2 reports."""
        spacing = mean_stem_spacing_m(np.array([[50.0, 107.0]], dtype=np.float32))
        speed = max_reactive_speed_ms(spacing, cruise_speed_ms=CRUISE)
        assert speed[0, 0] == pytest.approx(5.4, abs=0.1)
        assert speed[0, 1] == pytest.approx(3.7, abs=0.2)

    def test_the_cap_never_exceeds_the_airframe_cruise(self) -> None:
        spacing = np.array([[np.inf, 1000.0, 5.0]], dtype=np.float32)
        speed = max_reactive_speed_ms(spacing, cruise_speed_ms=CRUISE)
        assert np.all(speed <= CRUISE)
        assert speed[0, 0] == pytest.approx(CRUISE)

    def test_no_realistic_stand_clears_the_endurance_floor(self) -> None:
        """N3's headline, and the reason routing between stands cannot rescue N1.

        Clearing the 4.00 m/s endurance floor needs <=92 stems/ha. Under the
        self-thinning relation that takes a **~70 m canopy** — taller than any
        managed forest — so the sparse-stand lever the TEST-PLAN hoped for does
        not exist at realistic stand heights.
        """
        realistic = np.array([[10.0, 15.0, 20.0, 25.0, 30.0, 40.0]], dtype=np.float32)
        speed = max_reactive_speed_ms(
            mean_stem_spacing_m(stem_density_per_ha(realistic)), cruise_speed_ms=CRUISE
        )
        assert np.all(speed < 4.0)
        # It takes a redwood to get there.
        tall = np.array([[70.0]], dtype=np.float32)
        assert (
            max_reactive_speed_ms(
                mean_stem_spacing_m(stem_density_per_ha(tall)), cruise_speed_ms=CRUISE
            )[0, 0]
            >= 4.0
        )


class TestTimePenalty:
    def test_cruise_speed_cells_are_free(self) -> None:
        speed = np.full((1, 1), CRUISE, dtype=np.float32)
        assert traversal_time_penalty(speed, cruise_speed_ms=CRUISE)[0, 0] == pytest.approx(0.0)

    def test_half_cruise_costs_one_extra_second_per_cruise_second(self) -> None:
        speed = np.full((1, 1), CRUISE / 2.0, dtype=np.float32)
        assert traversal_time_penalty(speed, cruise_speed_ms=CRUISE)[0, 0] == pytest.approx(1.0)

    def test_a_near_stopped_cell_cannot_dominate_the_whole_objective(self) -> None:
        speed = np.full((1, 1), 1e-6, dtype=np.float32)
        assert traversal_time_penalty(speed, cruise_speed_ms=CRUISE)[0, 0] == 10.0


class TestBuildStemClutter:
    def test_returns_the_four_rasters_the_cost_surface_consumes(self) -> None:
        chm = np.array([[0.0, 12.0, 22.0]], dtype=np.float32)
        out = build_stem_clutter(chm, cruise_speed_ms=CRUISE, min_canopy_m=12.0)
        assert set(out) == {
            "stem_density_per_ha",
            "stem_spacing_m",
            "max_reactive_speed_ms",
            "max_speed_ms",
            "beyond_envelope",
            "stem_density_measured",
            "clutter_time_penalty",
        }
        # Open ground: no stems, no penalty, full cruise.
        assert out["stem_density_per_ha"][0, 0] == 0.0
        assert out["max_reactive_speed_ms"][0, 0] == pytest.approx(CRUISE)
        assert out["clutter_time_penalty"][0, 0] == pytest.approx(0.0)
        # Forest: slower and therefore penalised, and the taller stand less so.
        assert out["clutter_time_penalty"][0, 1] > out["clutter_time_penalty"][0, 2] > 0.0

    def test_spacing_raster_is_finite_for_serialization(self) -> None:
        """``inf`` in a component raster would poison normalization and any GeoTIFF."""
        out = build_stem_clutter(np.zeros((2, 2), dtype=np.float32), cruise_speed_ms=CRUISE)
        assert np.isfinite(out["stem_spacing_m"]).all()


class TestRoutingBehaviour:
    """Does the term actually steer the route? The demo AO cannot show this."""

    def test_the_planner_prefers_a_sparser_corridor_when_one_exists(self) -> None:
        from droneplanner.fields.composite import build_cost_surface
        from droneplanner.model.grid import Grid
        from droneplanner.planner.corridor import plan_corridor
        from tests.fixtures.synthetic import make_mission

        cell = 25.0
        n = 41
        grid = Grid(
            transform=(cell, 0.0, 0.0, 0.0, -cell, n * cell), rows=n, cols=n, crs="EPSG:32633"
        )
        dtm = np.zeros((n, n), dtype=np.float32)
        # Dense 12 m forest everywhere, with one tall (therefore sparser) lane.
        dsm = np.full((n, n), 12.0, dtype=np.float32)
        lane = n // 4
        dsm[lane, :] = 45.0

        start = grid.pixel_to_world(n // 2, 1)
        end = grid.pixel_to_world(n // 2, n - 2)
        mission = make_mission(
            name="clutter-ao",
            start={"x": start.x, "y": start.y},
            end={"x": end.x, "y": end.y},
            waypoint_spacing_m=cell,
            cruise_agl_m=8.0,
        )

        def rows_used(clutter_weight: float) -> set[int]:
            m = mission.model_copy(
                update={
                    "weights": mission.weights.model_copy(update={"clutter": clutter_weight}),
                }
            )
            plan = plan_corridor(m, build_cost_surface(dtm, dsm, grid, m))
            return {grid.world_to_pixel(v.coordinate)[0] for v in plan.vertices}

        assert lane not in rows_used(0.0)
        assert lane in rows_used(20.0)


class TestOnboardPlanningEnvelope:
    """The speed cap is the local planner's MEASURED envelope, not reactive coherence."""

    def test_the_planning_envelope_never_exceeds_the_harness(self) -> None:
        """SITL is the arbiter and it is slower: 5 m/s at 200/ha failed 2/2 in SITL
        where the harness flew it clean. The planner must never price faster."""
        from droneplanner.fields.stem_density import (
            HARNESS_ENVELOPE,
            ONBOARD_PLANNING_ENVELOPE,
        )

        harness = dict(HARNESS_ENVELOPE)
        for density, speed in ONBOARD_PLANNING_ENVELOPE:
            assert speed <= harness[density]
        assert (
            dict(ONBOARD_PLANNING_ENVELOPE)[200.0] == 2.0
        )  # the speed SITL flew on every seed set

    def test_anchors_are_reproduced(self) -> None:
        from droneplanner.fields.stem_density import (
            ONBOARD_PLANNING_ENVELOPE,
            max_planned_speed_ms,
        )

        for density, speed in ONBOARD_PLANNING_ENVELOPE:
            got = max_planned_speed_ms(np.array([density]), cruise_speed_ms=6.0)[0]
            assert got == pytest.approx(speed)

    def test_beyond_the_envelope_is_flagged_and_still_monotone(self) -> None:
        from droneplanner.fields.stem_density import beyond_envelope, max_planned_speed_ms

        d = np.array([800.0, 1300.0, 1500.0])  # above the 0.3 m/s extrapolation floor
        v = max_planned_speed_ms(d, cruise_speed_ms=6.0)
        assert v[0] > v[1] > v[2] > 0.0
        assert list(beyond_envelope(d)) == [False, True, True]

    def test_cells_flown_over_the_canopy_are_not_slowed(self) -> None:
        chm = np.array([[22.0, 22.0]], dtype=np.float32)
        among = np.array([[True, False]])
        out = build_stem_clutter(chm, cruise_speed_ms=CRUISE, min_canopy_m=12.0, among_stems=among)
        assert out["max_speed_ms"][0, 0] < CRUISE
        assert out["max_speed_ms"][0, 1] == pytest.approx(CRUISE)
        assert out["beyond_envelope"][0, 1] == 0.0

    def test_a_stated_density_overrides_the_canopy_inference(self) -> None:
        chm = np.array([[0.0, 22.0]], dtype=np.float32)
        out = build_stem_clutter(
            chm, cruise_speed_ms=CRUISE, min_canopy_m=12.0, density_override_per_ha=107.0
        )
        assert out["stem_density_per_ha"][0, 0] == 0.0  # open ground stays open
        assert out["stem_density_per_ha"][0, 1] == pytest.approx(107.0)
        from droneplanner.fields.stem_density import ONBOARD_PLANNING_ENVELOPE

        assert out["max_speed_ms"][0, 1] == pytest.approx(ONBOARD_PLANNING_ENVELOPE[0][1])
