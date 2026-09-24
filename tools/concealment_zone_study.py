"""How much of the route can be flown under cover and still close on one battery?

The user's concept (2026-09-14): *"realistically only the first part has to be
flown under cover."* How long "the first part" is was never stated, and the
2026-09-23 instruction is to measure such things and decide rather than ask. This
sweeps it:

    python tools/concealment_zone_study.py --ao demo_ao

For each concealment radius (``Mission.covered_radius_m``) and each stand density
it plans the route and reports the below-canopy length, the flight time against
the usable endurance, and how much of the covered stretch lies outside the
onboard planner's demonstrated envelope. The densities are the anchors of
``fields/stem_density.ONBOARD_PLANNING_ENVELOPE`` plus the canopy-height inference
the demo AO would otherwise get, because the answer depends on the forest far
more than on anything else.
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import numpy as np
import rasterio
import yaml

from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission
from droneplanner.model.plan import FlightBand
from droneplanner.planner.corridor import plan_over_terrain

RADII_M: tuple[float | None, ...] = (500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, None)
DENSITIES: tuple[float | None, ...] = (107.0, 200.0, 400.0, None)
SOLVE_DENSITIES: tuple[float, ...] = (
    200.0,
    300.0,
    400.0,
    500.0,
    600.0,
    700.0,
    800.0,
    900.0,
    1000.0,
    1200.0,
    1500.0,
)
SOLVE_STEP_M = 100.0
SOLVE_MAX_M = 5000.0


def load_ao(ao: Path) -> tuple[dict[str, object], np.ndarray, np.ndarray, Grid]:
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
        crs=str(base["crs"]),
    )
    return base, dtm, dsm, grid


def _plan(
    ao: Path, density: float | None, radius: float | None, mission_file: str
) -> dict[str, float]:
    _, dtm, dsm, grid = load_ao(ao)
    m = yaml.safe_load((ao / mission_file).read_text())
    m["covered_radius_m"] = radius
    stem = dict(m.get("stem", {}) or {})
    stem["density_per_ha"] = density
    m["stem"] = stem
    plan, _ = plan_over_terrain(Mission.model_validate(m), dtm, dsm, grid, route_index=0)
    end = plan.endurance
    assert end is not None
    return {
        "radius": radius or 0.0,
        "energy": end.energy_fraction,
        "time_min": end.flight_time_s / 60.0,
        "closes": float(end.closes),
        "beyond": plan.objective_scores.get("below_canopy_beyond_envelope_m", 0.0),
    }


def solve(ao: Path, mission_file: str) -> None:
    """Largest covered radius that closes one sortie, per density.

    Radii are stepped, not bisected: the route changes with the radius, so energy
    is not guaranteed monotone, and the answer reported is the largest radius
    below which EVERY step closes.
    """
    from concurrent.futures import ProcessPoolExecutor

    radii = [SOLVE_STEP_M * k for k in range(1, int(SOLVE_MAX_M / SOLVE_STEP_M) + 1)]
    print(f"{'stems/ha':>9}{'max radius':>12}{'energy':>8}{'time':>7}  at 1500 m  envelope")
    with ProcessPoolExecutor() as ex:
        for density in SOLVE_DENSITIES:
            rows = list(
                ex.map(
                    _plan,
                    [ao] * len(radii),
                    [density] * len(radii),
                    radii,
                    [mission_file] * len(radii),
                )
            )
            best = None
            for row in rows:
                if not row["closes"]:
                    break
                best = row
            at1500 = next(r for r in rows if r["radius"] == 1500.0)
            max_r = f"{best['radius']:.0f} m" if best else "< 100 m"
            if best is rows[-1]:
                max_r = "whole route"
            energy = f"{best['energy']:.2f}" if best else "-"
            time_min = f"{best['time_min']:.1f}m" if best else "-"
            verdict = "closes" if at1500["closes"] else "NO-GO"
            envelope = "extrapolated" if at1500["beyond"] > 0 else "measured"
            print(
                f"{density:>9.0f}{max_r:>12}{energy:>8}{time_min:>7}"
                f"  {at1500['energy']:.2f} {verdict:>6}  {envelope}"
            )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ao", type=Path, default=Path("demo_ao"))
    ap.add_argument(
        "--solve",
        action="store_true",
        help="Per density, find the largest covered radius that closes one sortie.",
    )
    ap.add_argument("--mission", default="mission.yaml", help="Mission file in the AO for --solve.")
    args = ap.parse_args()
    if args.solve:
        solve(args.ao, args.mission)
        return 0
    base, dtm, dsm, grid = load_ao(args.ao)

    hdr = (
        f"{'density':>9}{'radius':>8}{'route':>7}{'below':>7}{'beyond':>8}"
        f"{'time':>7}{'energy':>8}{'exp_fl':>8}  verdict"
    )
    for density in DENSITIES:
        label = f"{density:.0f}/ha" if density else "inferred"
        print(f"\n{hdr}\n{'-' * len(hdr)}")
        for radius in RADII_M:
            m = copy.deepcopy(base)
            m["covered_radius_m"] = radius
            stem = dict(m.get("stem", {}) or {})  # type: ignore[call-overload]
            stem["density_per_ha"] = density
            m["stem"] = stem
            mission = Mission.model_validate(m)
            plan, _ = plan_over_terrain(mission, dtm, dsm, grid, route_index=0)
            end = plan.endurance
            assert end is not None
            below = sum(
                a_len
                for a_len, v in zip(
                    [
                        float(np.hypot(b.x - a.x, b.y - a.y))
                        for a, b in zip(plan.vertices, plan.vertices[1:], strict=False)
                    ],
                    plan.vertices,
                    strict=False,
                )
                if v.flight_band in (FlightBand.BELOW_CANOPY, FlightBand.UPPER_TRUNK)
            )
            beyond = plan.objective_scores.get("below_canopy_beyond_envelope_m", 0.0)
            verdict = "closes" if end.closes else "NO-GO"
            if beyond > 0:
                verdict += " (extrapolated)"
            print(
                f"{label:>9}{(f'{radius:.0f}' if radius else 'all'):>8}"
                f"{plan.total_length_m:>7.0f}{below:>7.0f}{beyond:>8.0f}"
                f"{end.flight_time_s / 60:>6.1f}m{end.energy_fraction:>8.2f}"
                f"{plan.objective_scores['mean_exposure_flown']:>8.3f}  {verdict}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
