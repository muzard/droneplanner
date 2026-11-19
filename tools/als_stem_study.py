"""Can airborne lidar feed the stem detector? (SHORTFALLS M2 / roadmap P1)

Sweeps pulse density x crown transmission over a synthetic managed stand, traces
pulses with :func:`forest.als_model.simulate_als_scan`, runs
:func:`forest.stem_detect.detect_stems` (default and relaxed thresholds) and
scores the map with :func:`match_stems` — next to the analytic expectation.

    python tools/als_stem_study.py                 # default sweep, ~1 min
    python tools/als_stem_study.py --stems-per-ha 1000 --area 60

Every number is an UPPER BOUND on what a real scan gives (see the
``forest.als_model`` docstring: no footprint, branches, understory or waveform).
"""

from __future__ import annotations

import argparse
from dataclasses import replace

import numpy as np

from droneplanner.fields.stems import StemMap
from droneplanner.forest.als_model import (
    AlsScanConfig,
    expected_trunk_returns,
    required_pulse_density,
    returns_per_stem,
    simulate_als_scan,
)
from droneplanner.forest.stem_detect import StemDetectConfig, detect_stems, match_stems
from droneplanner.model.grid import Grid

DENSITIES = (0.5, 5.0, 20.0, 50.0, 100.0, 300.0, 1000.0)
TRANSMISSIONS = (0.1, 0.2, 0.4)
RELAXED = StemDetectConfig(min_points=3, min_vertical_extent_m=0.5)


def managed_stand(stems_per_ha: float, side_m: float, seed: int, min_gap_m: float = 1.2) -> StemMap:
    """Poisson stand with a hard-core spacing (thinned talousmetsä), r 0.10-0.20 m."""
    rng = np.random.default_rng(seed)
    want = round(stems_per_ha * side_m * side_m / 10_000.0)
    pts: list[tuple[float, float]] = []
    tries = 0
    while len(pts) < want and tries < want * 200:
        tries += 1
        p = rng.uniform(1.0, side_m - 1.0, 2)
        if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 >= min_gap_m**2 for q in pts):
            pts.append((float(p[0]), float(p[1])))
    arr = np.array(pts, dtype=float).reshape(-1, 2)
    return StemMap(x=arr[:, 0], y=arr[:, 1], radius_m=rng.uniform(0.10, 0.20, len(arr)))


def flat_grid(side_m: float, cell: float = 5.0) -> tuple[Grid, np.ndarray]:
    n = int(np.ceil(side_m / cell))
    grid = Grid(transform=(cell, 0.0, 0.0, 0.0, -cell, n * cell), rows=n, cols=n, crs="EPSG:3067")
    return grid, np.zeros((n, n), dtype=float)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--stems-per-ha", type=float, default=700.0)
    ap.add_argument("--area", type=float, default=50.0, help="square side, m")
    ap.add_argument("--crown-base", type=float, default=8.0)
    ap.add_argument("--max-off-nadir", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    stand = managed_stand(args.stems_per_ha, args.area, args.seed)
    grid, dtm = flat_grid(args.area)
    det = StemDetectConfig()
    r_mean = float(np.mean(stand.radius_m))
    print(
        f"stand: {len(stand)} stems over {args.area:.0f} m square "
        f"({args.stems_per_ha:.0f}/ha), mean r {r_mean:.3f} m; slice "
        f"{det.slice_lo_m}-{det.slice_hi_m} m, crown base {args.crown_base} m, "
        f"±{args.max_off_nadir}° off nadir"
    )
    print(
        f"{'p/m²':>7} {'T':>4} | {'E[ret]':>6} {'sim':>6} | "
        f"{'default det':>11} {'rmse':>5} {'spur':>4} | {'relaxed det':>11} {'rmse':>5} {'spur':>4}"
    )
    for t in TRANSMISSIONS:
        for d in DENSITIES:
            cfg = AlsScanConfig(
                pulse_density_m2=d,
                crown_transmission=t,
                crown_base_m=args.crown_base,
                max_off_nadir_deg=args.max_off_nadir,
            )
            cloud = simulate_als_scan(
                stand, (0.0, 0.0, args.area, args.area), cfg, seed=args.seed + 1
            )
            sim = float(np.mean(returns_per_stem(cloud, stand, det.slice_lo_m, det.slice_hi_m)))
            exp = expected_trunk_returns(
                d,
                r_mean,
                det.slice_lo_m,
                det.slice_hi_m,
                crown_base_m=args.crown_base,
                crown_transmission=t,
                max_off_nadir_deg=args.max_off_nadir,
            )
            cols = []
            for dc in (det, replace(RELAXED)):
                found, _ = detect_stems(cloud, dtm, grid, dc)
                rep = match_stems(found, stand, tolerance_m=0.5)
                rmse = "-" if rep.matched == 0 else f"{rep.position_rmse_m:.2f}"
                cols.append(f"{rep.detection_rate:>10.0%} {rmse:>5} {rep.spurious:>4}")
            print(f"{d:>7g} {t:>4g} | {exp:>6.2f} {sim:>6.2f} | {cols[0]} | {cols[1]}")
        print()
    for t in TRANSMISSIONS:
        need = required_pulse_density(
            det.min_points,
            r_mean,
            det.slice_lo_m,
            det.slice_hi_m,
            crown_base_m=args.crown_base,
            crown_transmission=t,
            max_off_nadir_deg=args.max_off_nadir,
        )
        print(
            f"T={t:g}: {need:.0f} p/m² for the mean stem to expect {det.min_points} slice returns"
        )


if __name__ == "__main__":
    main()
