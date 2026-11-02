"""Plant an impassable wall of trunks across a route in an existing Gazebo world.

    python tools/add_trunk_wall.py --world worlds/askolam300.sdf \\
        --plan out/askola_m300/mission.plan --at-m 150 --out worlds/askolam300w.sdf

The test the in-flight ESCAPE CLIMB (droneflight ``fly-local --escape-after``) needs:
a stand the preflight data calls passable that the aircraft cannot get through.
Two staggered rows, trunks ``--spacing`` apart (default 1.0 m: 0.7 m gaps against a
0.9 m airframe), ``--half-width`` either side of the route, ``--height`` tall, on
the terrain of the nearest existing trunk. Writes ``<out>`` and ``<out>_trunks.csv``
(the original manifest plus the wall), so perception and physics agree.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import re
from pathlib import Path

from droneflight.offboard import route_to_local
from droneflight.plan_loader import load_qgc_plan


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world", type=Path, required=True)
    ap.add_argument("--plan", type=Path, required=True)
    ap.add_argument("--at-m", type=float, required=True, help="Route distance of the wall.")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--half-width", type=float, default=50.0)
    ap.add_argument("--spacing", type=float, default=1.0)
    ap.add_argument("--height", type=float, default=16.0)
    args = ap.parse_args()

    route = route_to_local(load_qgc_plan(args.plan))
    run = 0.0
    for a, b in itertools.pairwise(route):
        seg = math.hypot(b.east_m - a.east_m, b.north_m - a.north_m)
        if run + seg >= args.at_m:
            f = (args.at_m - run) / seg
            cx, cy = a.east_m + f * (b.east_m - a.east_m), a.north_m + f * (b.north_m - a.north_m)
            ux, uy = (b.east_m - a.east_m) / seg, (b.north_m - a.north_m) / seg
            break
        run += seg
    else:
        raise SystemExit("route shorter than --at-m")

    src_csv = args.world.with_name(args.world.stem + "_trunks.csv")
    rows = list(csv.DictReader(src_csv.open()))
    bases = [(float(r["east_m"]), float(r["north_m"]), float(r["base_z_m"])) for r in rows]

    def ground(x: float, y: float) -> float:
        return min(bases, key=lambda b: (b[0] - x) ** 2 + (b[1] - y) ** 2)[2]

    wall = []
    n = int(args.half_width / args.spacing)
    for row, along in enumerate((-0.5 * args.spacing, 0.5 * args.spacing)):
        for k in range(-n, n + 1):
            across = k * args.spacing + (0.5 * args.spacing if row else 0.0)
            x = cx + along * ux - across * uy
            y = cy + along * uy + across * ux
            wall.append((x, y, ground(x, y)))

    text = args.world.read_text()
    name = args.out.stem
    text = re.sub(r"<world name=\"[^\"]+\"", f'<world name="{name}"', text, count=1)
    text = text.replace(args.world.stem + "_dem.tif", name + "_dem.tif")
    models = "".join(
        f"""    <model name="wall_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z + args.height / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.150</radius>
              <length>{args.height:.3f}</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
"""
        for i, (x, y, z) in enumerate(wall)
    )
    text = text.replace("  </world>", models + "  </world>", 1)
    args.out.write_text(text)
    out_csv = args.out.with_name(args.out.stem + "_trunks.csv")
    with out_csv.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["east_m", "north_m", "base_z_m", "radius_m", "height_m"])
        for r in rows:
            w.writerow([r["east_m"], r["north_m"], r["base_z_m"], r["radius_m"], r["height_m"]])
        for x, y, z in wall:
            w.writerow([f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", "0.150", f"{args.height:.3f}"])
    print(
        f"{args.out}: {len(rows)} trunks + {len(wall)} in a wall at {args.at_m:.0f} m "
        f"({cx:.1f}, {cy:.1f}) -> {out_csv}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
