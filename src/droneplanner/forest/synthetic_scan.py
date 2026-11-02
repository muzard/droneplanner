"""Synthesise a lidar point cloud from known trunks — a VALIDATION FIXTURE.

⚠️⚠️ **THIS IS NOT A SENSOR MODEL AND ITS OUTPUT IS NOT DATA.** It exists for one
purpose: to give :mod:`droneplanner.forest.stem_detect` an input whose ground
truth is known exactly, so the detector's *position error* can be measured rather
than assumed. Any accuracy figure derived from it is an **upper bound on a real
sensor**, because everything hard about real lidar in a forest is either
simplified here or absent:

* returns are drawn on trunk surfaces directly — there is no beam, no footprint,
  no multiple-return logic and no mixed pixels;
* occlusion is a blunt per-stem visible-arc fraction, not a traced line of sight,
  so a stem behind three others is as visible as one in the open;
* the understory is uniform clutter, not the dense, stem-like regeneration that
  actually defeats trunk detectors in young stands;
* no branches, no lean, no buttressing, no bark roughness, no wind, no
  registration error between flight lines — and registration error in particular
  is a leading contributor to real stem-position error.

So a detector that fails **here** would certainly fail on real data, which makes
this useful as a *falsifier*. A detector that succeeds here has not been shown to
work at all — that needs real lidar over a field-surveyed plot, which this
programme does not have (SHORTFALLS M2).
"""

from __future__ import annotations

import numpy as np

from droneplanner.fields.stems import StemMap


def simulate_scan(
    stems: StemMap,
    *,
    ground_elev: float | np.ndarray = 0.0,
    stem_height_m: float = 18.0,
    returns_per_stem_m2: float = 400.0,
    visible_arc_fraction: float = 0.5,
    range_noise_m: float = 0.02,
    understory_points: int = 0,
    understory_top_m: float = 1.5,
    ground_points: int = 0,
    area_bounds: tuple[float, float, float, float] | None = None,
    seed: int = 0,
) -> np.ndarray:
    """Return an (n, 3) world-coordinate cloud of returns off ``stems``.

    ``returns_per_stem_m2`` is areal density on the trunk surface — the knob the
    requirements sweep turns. ``visible_arc_fraction`` is the share of each
    trunk's circumference the scanner saw, which is what makes the fitted centre
    biased toward the scanner: a one-sided arc has its centroid *in front of* the
    true axis, and that bias, not random noise, is what limits position accuracy.
    """
    rng = np.random.default_rng(seed)
    if len(stems) == 0:
        return np.empty((0, 3), dtype=float)

    ground = np.broadcast_to(np.asarray(ground_elev, dtype=float), (len(stems),)).astype(float)
    chunks: list[np.ndarray] = []
    for k in range(len(stems)):
        radius = float(stems.radius_m[k])
        area = 2.0 * np.pi * radius * stem_height_m * visible_arc_fraction
        count = rng.poisson(area * returns_per_stem_m2)
        if count == 0:
            continue
        # Each stem is seen from its own bearing, so the visible arc is centred
        # somewhere different for every trunk.
        centre_bearing = rng.uniform(0.0, 2.0 * np.pi)
        span = 2.0 * np.pi * visible_arc_fraction
        theta = centre_bearing + rng.uniform(-span / 2.0, span / 2.0, size=count)
        r = radius + rng.normal(0.0, range_noise_m, size=count)
        z = ground[k] + rng.uniform(0.0, stem_height_m, size=count)
        chunks.append(
            np.column_stack([stems.x[k] + r * np.cos(theta), stems.y[k] + r * np.sin(theta), z])
        )

    if area_bounds is not None and (understory_points or ground_points):
        x0, y0, x1, y1 = area_bounds
        base = float(np.mean(ground))
        if understory_points:
            chunks.append(
                np.column_stack(
                    [
                        rng.uniform(x0, x1, understory_points),
                        rng.uniform(y0, y1, understory_points),
                        base + rng.uniform(0.0, understory_top_m, understory_points),
                    ]
                )
            )
        if ground_points:
            chunks.append(
                np.column_stack(
                    [
                        rng.uniform(x0, x1, ground_points),
                        rng.uniform(y0, y1, ground_points),
                        base + rng.normal(0.0, range_noise_m, ground_points),
                    ]
                )
            )
    if not chunks:
        return np.empty((0, 3), dtype=float)
    return np.vstack(chunks)
