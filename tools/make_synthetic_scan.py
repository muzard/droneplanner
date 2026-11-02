"""Write a synthetic LAS point cloud from a trunk manifest, plus a mission to plan it.

This regenerates the fixture behind the stem-detection results (the `.las` itself
is gitignored — it is 25 MB of derived data, not source):

    python tools/make_synthetic_scan.py \\
        --manifest worlds/forest_dense_trunks.csv --ao demo_ao

It exists so the **real ingest path** can be exercised end to end — LAS on disk ->
`io/pointcloud.read_pointcloud` -> `forest/stem_detect` -> the stem router — rather
than handing the planner an in-memory array and calling that an integration.

⚠️ Its output is NOT data. See `forest/synthetic_scan` for exactly what is missing
(no beam, no traced occlusion, no registration error). It can falsify a detector;
it cannot validate one. Only real lidar over a field-surveyed plot does that.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rasterio
import yaml

from droneplanner.fields.stems import load_stem_manifest
from droneplanner.forest.synthetic_scan import simulate_scan
from droneplanner.model.geometry import Coordinate


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifest", type=Path, default=Path("worlds/forest_dense_trunks.csv"))
    ap.add_argument("--ao", type=Path, default=Path("demo_ao"))
    ap.add_argument("--out", type=Path, default=None, help="Defaults to <ao>/synthetic_stems.las")
    ap.add_argument("--density", type=float, default=50.0, help="Returns per m^2 of trunk surface.")
    ap.add_argument(
        "--arc",
        type=float,
        default=0.5,
        help="Fraction of each trunk's circumference the scanner saw. The dominant "
        "error term: a narrow arc biases the fitted centre toward the scanner.",
    )
    ap.add_argument("--noise", type=float, default=0.03, help="Range noise sigma, metres.")
    ap.add_argument("--stem-height", type=float, default=14.0)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    import laspy

    home_mission = yaml.safe_load((args.ao / "mission.yaml").read_text())
    home = Coordinate(x=home_mission["start"]["x"], y=home_mission["start"]["y"])
    stems = load_stem_manifest(args.manifest, home)

    with rasterio.open(args.ao / "dtm.tif") as ds:
        dtm = ds.read(1).astype(float)
        tr = ds.transform
    col = ((stems.x - tr.c) / tr.a).astype(int)
    row = ((stems.y - tr.f) / tr.e).astype(int)
    ok = (row >= 0) & (row < dtm.shape[0]) & (col >= 0) & (col < dtm.shape[1])
    ground = np.where(
        ok, dtm[np.clip(row, 0, dtm.shape[0] - 1), np.clip(col, 0, dtm.shape[1] - 1)], 0.0
    )

    cloud = simulate_scan(
        stems,
        ground_elev=ground,
        returns_per_stem_m2=args.density,
        visible_arc_fraction=args.arc,
        range_noise_m=args.noise,
        stem_height_m=args.stem_height,
        seed=args.seed,
    )
    out = args.out or (args.ao / "synthetic_stems.las")
    header = laspy.LasHeader(point_format=6, version="1.4")
    header.offsets = cloud.min(axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])
    las = laspy.LasData(header)
    las.x, las.y, las.z = cloud[:, 0], cloud[:, 1], cloud[:, 2]
    las.write(out)
    size_mb = out.stat().st_size / 1e6
    print(f"{len(cloud):,} returns from {len(stems)} stems -> {out} ({size_mb:.1f} MB)")

    mission = dict(home_mission)
    mission["layers"] = {**mission["layers"], "point_cloud": str(out.resolve())}
    mission["name"] = "demo-ao-lidar-stems"
    mission_path = args.ao / "mission_lidar.yaml"
    mission_path.write_text(yaml.safe_dump(mission, sort_keys=False))
    print(f"wrote {mission_path} — plan it with:")
    print(f"  droneplanner plan --mission {mission_path} --out out/lidar")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
