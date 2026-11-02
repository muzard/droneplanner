"""Cost the relay / multi-sortie concept on a real planned route (TEST-PLAN N1 way out #1).

The concealed below-canopy route is NO-GO on endurance (57.1 min against 30), and
`CLAUDE.md` lists four ways out. Three are expensive. The fourth — *"shorter legs
with relay points"* — is called the cheapest and has never been given a number.
This script gives it one:

    python tools/relay_study.py --ao demo_ao

For each flight band it plans the route, splits it into the fewest legs that each
fit one battery (`planner/relay.py`), and reports where the relay points land, how
exposed that ground is, and what a bigger battery would buy instead.
"""

from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path

import numpy as np
import rasterio
import yaml

from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission
from droneplanner.planner.corridor import plan_over_terrain
from droneplanner.planner.endurance import model_from_mission
from droneplanner.planner.relay import (
    balance_legs,
    legs_required_for_endurance,
    split_into_legs,
)

# Same three arms the concealment study uses, so the numbers line up with the
# gate tables: cruise_agl_m and the canopy height at which a stand is entered.
BANDS = {
    "below_canopy": dict(cruise_agl_m=12.0, min_canopy_m=12.0),
    "canopy_top": dict(cruise_agl_m=12.0, min_canopy_m=999.0),
}

ENDURANCE_SWEEP_MIN = (30.0, 40.0, 45.0, 60.0, 90.0)


def load_ao(ao: Path) -> tuple[dict, np.ndarray, np.ndarray, Grid]:
    base = yaml.safe_load((ao / "mission.yaml").read_text())
    with rasterio.open(ao / "dtm.tif") as ds:
        dtm = ds.read(1).astype(np.float64)
        t = ds.transform
    with rasterio.open(ao / "dsm.tif") as ds:
        dsm = ds.read(1).astype(np.float64)
    grid = Grid(
        transform=(t.a, t.b, t.c, t.d, t.e, t.f),
        rows=dtm.shape[0],
        cols=dtm.shape[1],
        crs=base["crs"],
    )
    return base, dtm, dsm, grid


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ao", type=Path, default=Path("demo_ao"))
    args = ap.parse_args()

    base, dtm, dsm, grid = load_ao(args.ao)

    for band, cfg in BANDS.items():
        m = copy.deepcopy(base)
        m["cruise_agl_m"] = cfg["cruise_agl_m"]
        m.setdefault("altitude", {})["min_canopy_m"] = cfg["min_canopy_m"]
        mission = Mission.model_validate(m)
        plan, surface = plan_over_terrain(mission, dtm, dsm, grid, route_index=0)

        pts = [(v.x, v.y) for v in plan.vertices]
        amsl = [v.ground_elev_m + v.altitude_agl_m for v in plan.vertices]
        limits = [v.speed_hint_ms for v in plan.vertices]
        model = model_from_mission(mission)

        budget = plan.endurance
        print(f"\n{'=' * 78}\n{band.upper()}\n{'=' * 78}")
        assert budget is not None
        print(
            f"  whole route: {plan.total_length_m:,.0f} m, "
            f"{len(pts)} vertices, {budget.flight_time_s / 60:.1f} min, "
            f"energy {budget.energy_fraction:.2f} of usable "
            f"({model.usable_endurance_s:.0f} s of {model.endurance_s:.0f} s)"
        )

        greedy = split_into_legs(pts, amsl, model, speed_limits_ms=limits)
        if not greedy.feasible:
            print(
                f"  🔴 UNSPLITTABLE — vertex {greedy.blocking_index} "
                "cannot even reach its neighbour"
            )
            continue
        rp = balance_legs(pts, amsl, model, speed_limits_ms=limits)
        print(
            f"\n  greedy (fewest legs): {greedy.leg_count} legs, "
            f"worst leg at {greedy.worst_energy_fraction:.2f} of usable — "
            f"lengths {', '.join(f'{leg.length_m:.0f}' for leg in greedy.legs)} m"
        )
        print(f"\n  ---> BALANCED: {rp.leg_count} LEG(S), {len(rp.relay_indices)} relay point(s)")
        hdr = (
            f"  {'leg':>4}{'vtx':>10}{'length':>9}{'time':>8}{'energy':>8}"
            f"{'eff m/s':>9}{'capped':>8}"
        )
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))
        for i, leg in enumerate(rp.legs, 1):
            e = leg.endurance
            print(
                f"  {i:>4}{f'{leg.start_index}-{leg.end_index}':>10}{leg.length_m:>9.0f}"
                f"{e.flight_time_s / 60:>8.1f}{e.energy_fraction:>8.2f}"
                f"{e.effective_speed_ms:>9.2f}{e.speed_capped_legs:>8}"
            )

        # A relay point is a place a person has to stand. Score that ground with
        # the same field the cable is scored on: ground_los is "can an observer
        # see this surface point", which is exactly the question for a ground site.
        if rp.relay_indices:
            ground_los = surface.components["ground_los"]
            exposure = surface.components["exposure"]
            print(f"\n  relay points ({len(rp.relay_indices)}):")
            print(f"  {'#':>3}{'x':>12}{'y':>12}{'along':>9}{'grnd LOS':>10}{'drone exp':>11}")
            cum = np.concatenate(
                [[0.0], np.cumsum([math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)])]
            )
            for k, idx in enumerate(rp.relay_indices, 1):
                from droneplanner.model.geometry import Coordinate

                rc = grid.world_to_pixel(Coordinate(x=pts[idx][0], y=pts[idx][1]))
                print(
                    f"  {k:>3}{pts[idx][0]:>12.0f}{pts[idx][1]:>12.0f}{cum[idx]:>9.0f}"
                    f"{float(ground_los[rc]):>10.3f}{float(exposure[rc]):>11.3f}"
                )

        print("\n  what a bigger battery buys instead:")
        print(f"  {'endurance':>11}{'legs':>7}{'relays':>8}{'worst leg energy':>18}")
        for minutes in ENDURANCE_SWEEP_MIN:
            sweep = legs_required_for_endurance(
                pts, amsl, model, minutes * 60.0, speed_limits_ms=limits
            )
            if not sweep.feasible:
                print(f"  {minutes:>9.0f} m{'UNSPLITTABLE':>25}")
                continue
            print(
                f"  {minutes:>9.0f} m{sweep.leg_count:>7}{len(sweep.relay_indices):>8}"
                f"{sweep.worst_energy_fraction:>18.2f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
