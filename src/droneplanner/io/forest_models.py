"""Tree trunks as Gazebo collision geometry — a world you can actually hit.

WHY THIS EXISTS (TEST-PLAN B2)
Every SITL result so far flew over an empty world. The planner now routes *under
the canopy* and grants a lateral corridor on the promise that the flight stack
avoids what it finds there — and none of that has ever met an obstacle. As the
user put it (2026-07-28): the corridor "serves little purpose on an open field
where the route could actually be planned perfectly in advance".

So this generates the clutter. Trunks are placed only where the planner believed
there was canopy to hide under (``chm >= min_canopy_m``), inside a band around
the planned route, at a stem density the operator sets. Two artifacts come out
and **both matter**:

* a block of SDF ``<model>`` elements — static collision cylinders, so a Gazebo
  contact is ground-truth "you hit a tree";
* a **trunk manifest** (CSV) in world ENU metres relative to home, which is the
  frame ``droneflight``'s oracle perception works in.

The manifest is not a convenience: it is what makes the experiment honest. The
avoidance layer must sense *the same trees the physics engine will collide with*.
Generating them twice from the same seed would be a second implementation and a
second chance to disagree — so they are generated once, here, and both consumers
read the same file.

⚠️ **Trunk count is the tractability knob, and it is not free.** Realistic stem
densities over a 5 km route run to five figures, which Gazebo will not enjoy.
``max_stems`` subsamples **deterministically and reports the density it actually
achieved** — the band is deliberately *not* narrowed instead, because a drone
that can leave the treed band has an escape the real one does not, and the test
would measure that instead of avoidance. Lower density is an honest weaker test;
a hole in the forest is a misleading one.

⚠️ **These are bare cylinders from ground to canopy top.** No branches, no crown,
no understory, no lean — so the sim forest is *easier* than a real one, in the
direction that flatters avoidance. It is a floor, not a fidelity claim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid

_SQM_PER_HA = 10_000.0


@dataclass(frozen=True)
class TrunkModel:
    """One trunk, in the Gazebo world frame (ENU metres, origin = plan home)."""

    east_m: float
    north_m: float
    base_z_m: float
    """World z of the terrain surface it stands on."""

    radius_m: float
    height_m: float


@dataclass(frozen=True)
class ForestStats:
    """What was actually generated — the clutter the test really ran against."""

    trunks: int
    band_area_ha: float
    requested_stems_per_ha: float
    effective_stems_per_ha: float
    subsampled: bool
    max_route_distance_m: float = 0.0
    """Furthest any trunk ended up from the route — the band as *achieved*.

    The band mask is a distance transform over the corridor grid, so it quantises
    to the cell: on the 25 m demo AO a requested 25 m band delivers trunks up to
    ~50 m out, because both the route point and the trunk can sit anywhere within
    their cells. Reported rather than assumed.
    """

    on_route_trunks: int = 0
    """Trunks within a rotor span of the planned centreline.

    These are the ones a route flown *without* avoidance would hit — the control
    arm's expected failures, and the reason the experiment is worth running.
    """


def _route_band_mask(
    route: list[Coordinate], grid: Grid, band_m: float, shape: tuple[int, int]
) -> np.ndarray:
    """Cells within ``band_m`` of the planned route."""
    from scipy import ndimage

    track = np.zeros(shape, dtype=bool)
    for p in route:
        track[grid.world_to_pixel(p)] = True
    if not track.any():
        return track
    distance = ndimage.distance_transform_edt(~track) * grid.cell_size_m
    return np.asarray(distance <= band_m)


def generate_trunks(
    chm: np.ndarray,
    dtm: np.ndarray,
    grid: Grid,
    route: list[Coordinate],
    *,
    home: Coordinate,
    home_elev_m: float,
    band_m: float = 25.0,
    stems_per_ha: float = 400.0,
    min_canopy_m: float = 12.0,
    trunk_radius_m: float = 0.15,
    max_stems: int = 4000,
    rotor_span_m: float = 0.9,
    seed: int = 0,
    within_m: float | None = None,
    density_per_ha: np.ndarray | None = None,
) -> tuple[list[TrunkModel], ForestStats]:
    """Place trunks where the planner believed there was cover, near the route.

    ``within_m`` limits the forest to that distance of ``home`` — the mission's
    concealment zone, outside which the route flies over the canopy.

    ``density_per_ha`` (per cell, e.g. measured stand data) replaces the uniform
    ``stems_per_ha``: each forested cell gets a Poisson draw of its own expected
    count, so the world carries the density the plan was priced on.
    """
    if stems_per_ha <= 0.0:
        raise ValueError("stems_per_ha must be positive")
    if max_stems < 1:
        raise ValueError("max_stems must be at least 1")

    forested = (chm >= min_canopy_m) & _route_band_mask(route, grid, band_m, chm.shape)
    if within_m is not None:
        a, b, c, d, e, f = grid.transform
        rr, cc = np.mgrid[0 : chm.shape[0], 0 : chm.shape[1]].astype(np.float64)
        wx = a * (cc + 0.5) + b * (rr + 0.5) + c
        wy = d * (cc + 0.5) + e * (rr + 0.5) + f
        forested &= np.hypot(wx - home.x, wy - home.y) <= within_m
    cells = np.argwhere(forested)
    cell_area = grid.cell_size_m**2
    area_ha = len(cells) * cell_area / _SQM_PER_HA
    if not len(cells):
        return [], ForestStats(0, 0.0, stems_per_ha, 0.0, False, 0.0, 0)

    rng = np.random.default_rng(seed)
    if density_per_ha is not None:
        expected = np.clip(density_per_ha[cells[:, 0], cells[:, 1]], 0.0, None) * (
            cell_area / _SQM_PER_HA
        )
        stems_per_ha = float(expected.sum() / area_ha)
        subsampled = expected.sum() > max_stems
        if subsampled:
            expected = expected * (max_stems / expected.sum())
        per_cell = rng.poisson(expected)
        chosen = np.repeat(np.arange(len(cells)), per_cell)
        count = len(chosen)
    else:
        target = max(1, round(area_ha * stems_per_ha))
        subsampled = target > max_stems
        count = min(target, max_stems)
        chosen = rng.choice(len(cells), size=count, replace=True)
    jitter = rng.uniform(-0.5, 0.5, size=(count, 2))

    trunks: list[TrunkModel] = []
    for idx, (jr, jc) in zip(chosen, jitter, strict=True):
        r, c = cells[idx]
        world = grid.pixel_to_world(float(r) + jr, float(c) + jc)
        height = float(chm[r, c])
        if height <= 0.0:
            continue
        trunks.append(
            TrunkModel(
                east_m=world.x - home.x,
                north_m=world.y - home.y,
                base_z_m=float(dtm[r, c]) - home_elev_m,
                radius_m=trunk_radius_m,
                height_m=height,
            )
        )
    max_dist, on_route = _route_proximity(trunks, route, home, rotor_span_m)
    stats = ForestStats(
        trunks=len(trunks),
        band_area_ha=area_ha,
        requested_stems_per_ha=stems_per_ha,
        effective_stems_per_ha=len(trunks) / area_ha if area_ha > 0 else 0.0,
        subsampled=subsampled,
        max_route_distance_m=max_dist,
        on_route_trunks=on_route,
    )
    return trunks, stats


def _route_proximity(
    trunks: list[TrunkModel], route: list[Coordinate], home: Coordinate, rotor_span_m: float
) -> tuple[float, int]:
    """Measure the band actually achieved, and how many trunks block the centreline."""
    if not trunks or not route:
        return 0.0, 0
    from scipy import spatial

    track = spatial.cKDTree([(p.x - home.x, p.y - home.y) for p in route])
    distances, _ = track.query([(t.east_m, t.north_m) for t in trunks])
    return float(distances.max()), int((distances < rotor_span_m).sum())


def trunks_to_sdf(trunks: list[TrunkModel], *, merged: bool = False, visuals: bool = False) -> str:
    """Render trunks as SDF. **Default is one model per trunk — do not change it lightly.**

    Both shapes have been flown, and they fail in opposite directions:

    * **one model per trunk (default)** — collisions demonstrably work: mission
      mode aborted at 64/282 in a 4000-trunk world where the empty world flew
      282/282. But Gazebo takes a long time to build the scene, and at 4000
      models PX4 sat on *"Waiting for Gazebo world..."* indefinitely.
    * **``merged=True``** — one static model, one link, N posed collisions. Loads
      fast (2.8 MB -> 1.0 MB), and the scene graph is happy. But in the flight
      that used it the vehicle passed **through** trunks: 269 of 270 horizontal
      overlaps had it inside the trunk's vertical span, and it completed the
      route anyway. Something about many collisions in a single static link is
      not producing contacts, and it cannot be diagnosed from macOS.

    So the count, not the shape, is the knob: keep ``--max-stems`` low enough that
    Gazebo loads (~1000-2000 is known-good) and leave the models separate. A world
    that loads quickly and does not collide is worse than one that loads slowly,
    because it looks like a pass.

    ``visuals`` is off by default: SITL runs headless, so the meshes are pure cost.
    """
    if merged:
        return _merged_sdf(trunks, visuals=visuals)
    parts: list[str] = []
    for i, t in enumerate(trunks):
        # A model's pose is the cylinder's centre, so lift it by half its height.
        pose = f"{t.east_m:.3f} {t.north_m:.3f} {t.base_z_m + t.height_m / 2.0:.3f} 0 0 0"
        geometry = (
            f"<geometry><cylinder><radius>{t.radius_m:.3f}</radius>"
            f"<length>{t.height_m:.3f}</length></cylinder></geometry>"
        )
        visual = (
            f'<visual name="visual">{geometry}<material>'
            "<ambient>0.25 0.18 0.11 1</ambient><diffuse>0.3 0.22 0.13 1</diffuse>"
            "</material></visual>"
            if visuals
            else ""
        )
        parts.append(
            f'    <model name="trunk_{i}">\n'
            f"      <static>true</static>\n"
            f"      <pose>{pose}</pose>\n"
            f'      <link name="link">\n'
            f'        <collision name="collision">{geometry}</collision>{visual}\n'
            f"      </link>\n"
            f"    </model>"
        )
    return "\n".join(parts)


def _merged_sdf(trunks: list[TrunkModel], *, visuals: bool) -> str:
    """One static model with N posed collisions — fast to load, see the warning above."""
    parts = [
        '    <model name="forest">',
        "      <static>true</static>",
        '      <link name="trunks">',
    ]
    for i, t in enumerate(trunks):
        pose = f"{t.east_m:.3f} {t.north_m:.3f} {t.base_z_m + t.height_m / 2.0:.3f} 0 0 0"
        geometry = (
            f"<geometry><cylinder><radius>{t.radius_m:.3f}</radius>"
            f"<length>{t.height_m:.3f}</length></cylinder></geometry>"
        )
        parts.append(f'        <collision name="c{i}"><pose>{pose}</pose>{geometry}</collision>')
        if visuals:
            parts.append(f'        <visual name="v{i}"><pose>{pose}</pose>{geometry}</visual>')
    parts.extend(["      </link>", "    </model>"])
    return "\n".join(parts)


def trunk_manifest_csv(trunks: list[TrunkModel]) -> str:
    """The same trunks, for the flight stack's perception oracle.

    World ENU metres relative to home — the frame ``droneflight`` senses in, so
    the trees it avoids are bit-for-bit the trees Gazebo will collide with.
    """
    lines = ["east_m,north_m,base_z_m,radius_m,height_m"]
    lines.extend(
        f"{t.east_m:.3f},{t.north_m:.3f},{t.base_z_m:.3f},{t.radius_m:.3f},{t.height_m:.3f}"
        for t in trunks
    )
    return "\n".join(lines) + "\n"


def nearest_trunk_spacing_m(trunks: list[TrunkModel]) -> float:
    """Median distance to the nearest neighbouring trunk — the gap the drone must fly.

    Stem density is the headline number, but what avoidance actually meets is the
    spacing. A 0.9 m airframe needs this to be comfortably more than its own width
    plus the state-estimation error, or no policy can succeed.
    """
    if len(trunks) < 2:
        return math.inf
    xy = np.array([(t.east_m, t.north_m) for t in trunks])
    from scipy import spatial

    tree = spatial.cKDTree(xy)
    distances, _ = tree.query(xy, k=2)  # k=1 is the point itself
    return float(np.median(distances[:, 1]))
