"""Reproduction harness for the concealment / flight-band study.

Every number in ``Threat-Geometry-vs-Flight-Band.md`` came out of this script.
It plans the same mission under several *arms* — flight band x threat geometry —
and then re-scores each flown trajectory for what an observer could actually
detect, which the planner's own binary viewshed does not answer.

    python tools/concealment_arms.py --ao demo_ao --out out/arms

Arms are switched with two mission knobs and no code changes:

* ``cruise_agl_m``          — the target height above the surface;
* ``altitude.min_canopy_m`` — the canopy height at which a stand is *entered*.
  Set it above the tallest stand (999) and no cell is enterable, so every treed
  cell falls to ``chm + canopy_clearance_m``: canopy-top skimming.

WHY THE RE-SCORING EXISTS
``fields/viewshed.py`` answers "is there a line of sight", with no range term
(roadmap C2) and no way for a cell's own canopy to occlude something beneath it
(roadmap C1). Both defects are *in* the numbers this script reports from
``report.json``; the extra columns it computes — angular subtense at the range
the sightline actually spans, dwell time, track fragmentation — are the honest
reading of the same geometry, and are what the roadmap's C1/C2 are meant to move
into the planner itself. Until they land, treat the report's ``mean_exposure*``
as "LOS exists somewhere", not as "detected".
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rasterio
import yaml

from droneplanner.fields.sectors import arc_contains, bearing_deg
from droneplanner.fields.viewshed import _point_weight
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import CanopyOcclusion, DetectionModel

ARCMIN = math.pi / (180 * 60)

# Angular thresholds a human observer needs, and where they come from. The 3'
# figure is the empirical one: DJI Mavic Pro (0.335 m) identified at 310 m =
# 3.7', Inspire 2 (0.6 m) at 698 m = 2.95', AeroVironment Wasp (0.72 m) acquired
# at 760 m = 3.26'. 12' is the NTSB "reasonable chance of detection" figure. 1'
# is the acuity limit and is included only to bound the optimistic end.
THRESHOLDS_ARCMIN = {"NTSB 12'": 12.0, "trials 3'": 3.0, "acuity 1'": 1.0}

RANGE_BANDS = ((250, "<250"), (500, "250-500"), (1000, "0.5-1k"), (2000, "1-2k"))


@dataclass(frozen=True)
class Arm:
    name: str
    cruise_agl_m: float
    min_canopy_m: float
    observers: list[tuple[float, float]] | None = None
    observer_height_m: float = 2.0
    speed_ms: float | None = None  # filled from the report when None


# Flight bands. `below_canopy` is the posture as-implemented; `canopy_top` is the
# one Preflight-Design §4 actually prescribes; `flat30` is the pre-2026-07-27
# posture, kept as a control.
BANDS = {
    "below_canopy": dict(cruise_agl_m=12.0, min_canopy_m=12.0),
    "canopy_top": dict(cruise_agl_m=12.0, min_canopy_m=999.0),
    "flat30": dict(cruise_agl_m=30.0, min_canopy_m=999.0),
}


def build_arms(base: dict, enemy: list[tuple[float, float]] | None) -> list[Arm]:
    arms = [Arm(name=f"{b}_asis", **cfg) for b, cfg in BANDS.items()]
    if enemy:
        arms += [Arm(name=f"{b}_receding", observers=enemy, **cfg) for b, cfg in BANDS.items()]
    return arms


def write_mission(
    base: dict,
    arm: Arm,
    dest: Path,
    canopy: CanopyOcclusion | None = None,
    observer_height_m: float | None = None,
    detection: DetectionModel | None = None,
    sectors: Sequence[tuple[float, float]] = (),
) -> Path:
    m = copy.deepcopy(base)
    m["name"] = arm.name
    m["cruise_agl_m"] = arm.cruise_agl_m
    m.setdefault("altitude", {})["min_canopy_m"] = arm.min_canopy_m
    if arm.observers is not None:
        m.setdefault("threat", {})["observers"] = [{"x": x, "y": y} for x, y in arm.observers]
    if sectors:
        # Every observer searches every arc. This is the "receding geometry" arm
        # expressed as a posture rather than by hand-placing observers where they
        # happen not to look — roadmap C3's acceptance case.
        observers = m.get("threat", {}).get("observers", [])
        m.setdefault("threat", {})["sectors"] = [
            {
                "azimuth_start_deg": start,
                "azimuth_end_deg": end,
                "observer": dict(observer),
                "label": f"arc{i}_obs{j}",
            }
            for i, (start, end) in enumerate(sectors)
            for j, observer in enumerate(observers)
        ]
    # The planner must route against the same threat the re-score grades against,
    # or the arm optimises one world and is scored in another. Sensor height is the
    # variable the whole gate turns on, so it is the one that must not drift.
    if canopy is not None:
        m.setdefault("threat", {})["canopy_occlusion"] = canopy.model_dump(mode="json")
    if observer_height_m is not None:
        m.setdefault("threat", {})["observer_height_m"] = observer_height_m
    if detection is not None:
        m.setdefault("threat", {})["detection"] = detection.model_dump(mode="json")
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(yaml.safe_dump(m, sort_keys=False))
    return dest


def plan(mission_path: Path, out_dir: Path) -> dict:
    subprocess.run(
        [
            sys.executable,
            "-m",
            "droneplanner.cli",
            "plan",
            "--mission",
            str(mission_path),
            "--out",
            str(out_dir),
        ],
        check=True,
        capture_output=True,
    )
    return json.loads((out_dir / "report.json").read_text())


def densify(coords: list[tuple[float, float, float]], step: float = 5.0):
    """Walk a 3D polyline at ~``step`` m, yielding (x, y, z_agl, seg_len)."""
    for i in range(len(coords) - 1):
        (x0, y0, z0), (x1, y1, z1) = coords[i], coords[i + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        if d == 0.0:
            continue
        n = max(1, int(d / step))
        for k in range(n):
            f = k / n
            yield x0 + f * (x1 - x0), y0 + f * (y1 - y0), z0 + f * (z1 - z0), d / n


def rescore(
    traj_path: Path,
    dtm: np.ndarray,
    dsm: np.ndarray,
    grid: Grid,
    observers: list[tuple[float, float]],
    *,
    span_m: float,
    speed_ms: float,
    observer_height_m: float = 2.0,
    detect_range_m: float = float("inf"),
    canopy: CanopyOcclusion | None = None,
    detection: DetectionModel | None = None,
    sectors: Sequence[tuple[float, float]] = (),
) -> dict:
    """What an observer could actually detect along the *flown* trajectory.

    Returns exposed length split by range band, angular-threshold survival,
    track fragmentation, and dwell time — the quantities the binary viewshed
    cannot express.

    Since roadmap C1 visibility is a **weight**, not a boolean, so two lengths are
    reported and they answer different questions:

    * ``los_m`` — metres where a sightline exists at all (the pre-C1 quantity);
    * ``seen_m`` — *expected* metres seen, weighting each sample by
      ``1 - prod(1 - w_i)`` over observers. With 0/1 weights the two coincide, so
      every historical number is reproduced exactly when ``canopy`` is None.

    ``under_canopy_m`` / ``under_canopy_seen_m`` isolate C1's acceptance case: the
    part of the route flown *beneath* the surface, and how much of it still scores
    as seen.

    ``sectors`` (roadmap C3) are ``(start, end)`` arcs every observer is searching.
    They apply here as well as in the planned mission, using the same containment
    rule ``fields.sectors`` uses, because an arm planned against a facing threat
    and scored against an all-round one would be measuring two different worlds.
    """
    gj = json.loads(traj_path.read_text())
    coords = [(c[0], c[1], c[2]) for c in gj["geometry"]["coordinates"]]
    orc = [grid.world_to_pixel(Coordinate(x=o[0], y=o[1])) for o in observers]
    eyes = [float(dsm[rc]) + observer_height_m for rc in orc]

    total = seen = los = 0.0
    under_canopy = under_canopy_seen = under_canopy_los = 0.0
    bands = {label: 0.0 for _, label in RANGE_BANDS}
    bands[">2k"] = 0.0
    angles: list[tuple[float, float]] = []
    runs: list[float] = []
    current = 0.0
    first_contact_s: float | None = None
    s = 0.0

    for x, y, z_agl, dl in densify(coords):
        total += dl
        rc = grid.world_to_pixel(Coordinate(x=x, y=y))
        z = float(dtm[rc]) + z_agl
        beneath = z < float(dsm[rc])
        under_canopy += dl if beneath else 0.0

        nearest = None
        miss = 1.0  # probability every observer misses
        for rc_o, eye, o in zip(orc, eyes, observers, strict=True):
            rng = math.hypot(x - o[0], y - o[1])
            if rng > detect_range_m:
                continue
            if sectors and not any(
                arc_contains(bearing_deg(o[0], o[1], x, y), start, end) for start, end in sectors
            ):
                continue  # this observer is not searching that bearing
            w = _point_weight(dsm, rc_o, eye, rc, z, grid.cell_size_m, canopy, detection, span_m)
            if w > 0.0:
                miss *= 1.0 - w
                nearest = rng if nearest is None else min(nearest, rng)
        p_seen = 1.0 - miss

        if nearest is not None:
            los += dl
            seen += p_seen * dl
            if beneath:
                under_canopy_los += dl
                under_canopy_seen += p_seen * dl
            current += dl
            if first_contact_s is None:
                first_contact_s = s
            angles.append((span_m / max(nearest, 1.0) / ARCMIN, p_seen * dl))
            for limit, label in RANGE_BANDS:
                if nearest < limit:
                    bands[label] += p_seen * dl
                    break
            else:
                bands[">2k"] += p_seen * dl
        elif current > 0.0:
            runs.append(current)
            current = 0.0
        s += dl
    if current > 0.0:
        runs.append(current)

    return dict(
        route_m=total,
        seen_m=seen,
        los_m=los,
        seen_pct=100.0 * seen / total if total else 0.0,
        under_canopy_m=under_canopy,
        under_canopy_los_m=under_canopy_los,
        under_canopy_seen_m=under_canopy_seen,
        median_arcmin=float(np.median([a for a, _ in angles])) if angles else float("nan"),
        bands=bands,
        above_threshold={
            k: sum(dl for a, dl in angles if a >= th) for k, th in THRESHOLDS_ARCMIN.items()
        },
        episodes=len(runs),
        longest_episode_m=max(runs) if runs else 0.0,
        dwell_s=seen / speed_ms if speed_ms else float("nan"),
        longest_episode_s=(max(runs) / speed_ms) if runs and speed_ms else 0.0,
        first_contact_m=first_contact_s,
    )


def detection_range_table(span_m: float) -> str:
    lines = [f"  Angular thresholds for a {span_m} m airframe:"]
    for label, th in THRESHOLDS_ARCMIN.items():
        r = span_m / (th * ARCMIN)
        lines.append(
            f"    {label:<12} unaided {r:>7,.0f} m | 7x {7 * r:>8,.0f} m | 12x {12 * r:>8,.0f} m"
        )
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--ao",
        type=Path,
        default=Path("demo_ao"),
        help="Directory with mission.yaml + dtm.tif + dsm.tif (droneplanner demo-ao).",
    )
    ap.add_argument("--out", type=Path, default=Path("out/arms"))
    ap.add_argument(
        "--enemy",
        type=float,
        nargs="*",
        default=None,
        help="x y [x y ...] observer positions for the 'receding' arms.",
    )
    ap.add_argument(
        "--observer-height",
        type=float,
        default=2.0,
        help="Eye/mast/aperture height. 2=dismount, 12/30=radar mast, 300=airborne.",
    )
    ap.add_argument(
        "--detect-range",
        type=float,
        default=float("inf"),
        help="Hard sensor range gate in metres (e.g. 2000 for a C-UAS radar).",
    )
    ap.add_argument(
        "--gap-fraction",
        type=float,
        default=None,
        help=(
            "Sub-canopy crown gap fraction (roadmap C1). Omit for the model default "
            "(0.10); 1.0 reproduces the pre-C1 numbers, 0.0 is the fully-closed bound."
        ),
    )
    ap.add_argument(
        "--max-elev-deg",
        type=float,
        default=None,
        help="Sightlines steeper than this get the gap-fraction treatment (default 10).",
    )
    ap.add_argument(
        "--threshold-arcmin",
        type=float,
        default=None,
        help=(
            "Angular size at which the airframe is acquired (roadmap C2). Default 3.0, "
            "the empirical trials figure; 12.0 is NTSB's far more forgiving one. Pass a "
            "tiny value (1e-4) to reproduce the pre-C2 unweighted numbers."
        ),
    )
    ap.add_argument(
        "--optic",
        type=float,
        default=None,
        help="Observer's optic magnification (1 unaided, 7 and 12 are the cases that matter).",
    )
    ap.add_argument(
        "--sector",
        type=float,
        nargs=2,
        action="append",
        metavar=("START", "END"),
        default=None,
        help=(
            "Azimuth arc (compass deg, swept clockwise) every observer is searching "
            "(roadmap C3). Repeatable. Applied to the planned mission AND to the "
            "re-score, so the arm is graded in the world it was planned for."
        ),
    )
    args = ap.parse_args()
    sectors: list[tuple[float, float]] = [(s, e) for s, e in (args.sector or [])]

    fields = {}
    if args.gap_fraction is not None:
        fields["sub_canopy_gap_fraction"] = args.gap_fraction
    if args.max_elev_deg is not None:
        fields["sub_canopy_max_elev_deg"] = args.max_elev_deg
    canopy = CanopyOcclusion(**fields)

    det_fields = {}
    if args.threshold_arcmin is not None:
        det_fields["threshold_arcmin"] = args.threshold_arcmin
    if args.optic is not None:
        det_fields["optic_magnification"] = args.optic
    detection = DetectionModel(**det_fields)

    base = yaml.safe_load((args.ao / "mission.yaml").read_text())
    span = float(base["drone"]["rotor_span_m"])
    enemy = None
    if args.enemy:
        enemy = list(zip(args.enemy[0::2], args.enemy[1::2], strict=True))

    with rasterio.open(args.ao / "dtm.tif") as ds:
        dtm = ds.read(1).astype(np.float64)
        t = ds.transform
    with rasterio.open(args.ao / "dsm.tif") as ds:
        dsm = ds.read(1).astype(np.float64)
    grid = Grid(
        transform=(t.a, t.b, t.c, t.d, t.e, t.f),
        rows=dtm.shape[0],
        cols=dtm.shape[1],
        crs=base["crs"],
    )

    print(detection_range_table(span), "\n")
    print(
        f"  observer height {args.observer_height:g} m | detect range "
        f"{args.detect_range:,.0f} m | canopy gap {canopy.sub_canopy_gap_fraction:g} "
        f"above {canopy.sub_canopy_max_elev_deg:g} deg | sectors "
        + (", ".join(f"{s:g}-{e:g}" for s, e in sectors) if sectors else "all-round")
        + "\n"
    )
    hdr = (
        f"{'arm':<22}{'len':>7}{'exp_flown':>10}{'exposed':>9}{'time':>8}"
        f"{'energy':>7}{'los_m':>8}{'seen_m':>8}{'undrCnpy':>9}{'ucSeen':>8}"
        f"{'medArcm':>9}{'episodes':>9}{'dwell_s':>9}{'rec':>8}"
    )
    print(hdr)
    print("-" * len(hdr))

    for arm in build_arms(base, enemy):
        mp = write_mission(
            base,
            arm,
            args.out / f"{arm.name}.yaml",
            canopy,
            args.observer_height,
            detection,
            sectors,
        )
        rep = plan(mp, args.out / arm.name)
        base_obs = base.get("threat", {}).get("observers", [])
        obs = arm.observers or [(o["x"], o["y"]) for o in base_obs]
        speed = rep["endurance"]["effective_speed_ms"]
        rs = rescore(
            args.out / arm.name / "trajectory.geojson",
            dtm,
            dsm,
            grid,
            obs,
            span_m=span,
            speed_ms=speed,
            observer_height_m=args.observer_height,
            detect_range_m=args.detect_range,
            canopy=canopy,
            detection=detection,
            sectors=sectors,
        )
        o = rep["objective_scores"]
        print(
            f"{arm.name:<22}{rep['total_length_m']:>7.0f}{o['mean_exposure_flown']:>10.3f}"
            f"{rep['drone_exposed_length_m']:>9.0f}{rep['endurance']['flight_time_s']:>8.0f}"
            f"{rep['endurance']['energy_fraction']:>7.2f}{rs['los_m']:>8.0f}"
            f"{rs['seen_m']:>8.0f}{rs['under_canopy_m']:>9.0f}{rs['under_canopy_seen_m']:>8.0f}"
            f"{rs['median_arcmin']:>9.2f}{rs['episodes']:>9}{rs['dwell_s']:>9.0f}"
            f"{rep['recommendation']:>8}"
        )
        (args.out / arm.name / "rescore.json").write_text(json.dumps(rs, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
