"""How many trunk returns does AIRBORNE lidar put on a stem? (SHORTFALLS M2 / roadmap P1)

WHY THIS EXISTS
The stem map is the planner's route to below-canopy flight and the flight stack's
best GPS-denied fix (``droneflight/stem_localizer.py``: 0.03 m against VTRN's 11.8 m).
The question that decides where that map can come from is whether the **open
national airborne lidar** (MML *Laserkeilausaineisto*, ~5 pulses/m² in the current
programme, 0.5 in the old one) can be fed to :func:`forest.stem_detect.detect_stems`,
or whether the programme needs its own UAV lidar.

:mod:`forest.synthetic_scan` cannot answer that: it draws returns *directly on the
trunk surface* at a stated areal density, so it assumes the answer. This module
models the one mechanism that decides it — **geometry of near-nadir pulses against
a vertical cylinder under a crown** — and nothing else.

THE MECHANISM
An airborne pulse arrives from above, ``theta`` off nadir. To leave a return on a
trunk below the crown it must (1) get through the crown layer, and (2) cross the
trunk's vertical silhouette on the way down. A vertical surface intercepts a
near-vertical flux only in proportion to ``tan(theta)``: a pulse density ``D`` per
m² of ground puts ``D * tan(theta)`` pulses per m² onto a vertical surface facing
the beam. So, in a slice of height ``h`` on a trunk of radius ``r``:

    E[returns] = D * T * 2r * h * E[tan(theta)]  (+ D * T * pi r² at the crown base)

where ``T`` is the crown transmission. With the scan angle uniform on
``[0, theta_max]``, ``E[tan(theta)] = -ln(cos theta_max) / theta_max`` (0.178 at
20 degrees). For MML-like ALS (5 p/m², ±20°, T = 0.2) on a 0.15 m trunk over the
detector's 3.5 m slice that is **~0.2 returns per stem** — against the detector's
``min_points`` of 8. :func:`expected_trunk_returns` is that formula;
:func:`simulate_als_scan` traces pulses to check it, including trunks shadowing
each other.

⚠️ WHAT THIS IS NOT. Still a synthetic geometry model: no beam footprint (a 0.3-0.5 m
ALS footprint hitting a 0.3 m trunk returns a *mixed* echo whose range is not the
trunk surface), no branches, no understory, no waveform. Every one of those makes a
real trunk return rarer or worse, so this is an **upper bound** on what ALS gives a
stem detector — useful for saying "no", not for saying "yes". Crown transmission is
a single per-pulse probability; leaf-off broadleaf and pine let more through than
spruce, which is why it is swept rather than fixed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from droneplanner.fields.stems import StemMap


def mean_tan_off_nadir(max_off_nadir_deg: float) -> float:
    """E[tan(theta)] for a scan angle uniform on [0, max_off_nadir_deg]."""
    if max_off_nadir_deg <= 0.0:
        return 0.0
    t = np.radians(max_off_nadir_deg)
    return float(-np.log(np.cos(t)) / t)


def expected_trunk_returns(
    pulse_density_m2: float,
    trunk_radius_m: float,
    slice_lo_m: float,
    slice_hi_m: float,
    *,
    crown_base_m: float,
    crown_transmission: float,
    max_off_nadir_deg: float = 20.0,
) -> float:
    """Expected returns on one isolated trunk inside the slice [lo, hi] AGL.

    Only the part of the slice under the crown base can be reached (the crown
    above stops the pulse first, or hides the trunk inside foliage), and a pulse
    falling straight onto the trunk's cross-section leaves its return at the crown
    base — counted when the crown base is inside the slice.
    """
    hi = min(slice_hi_m, crown_base_m)
    h = max(0.0, hi - slice_lo_m)
    side = 2.0 * trunk_radius_m * h * mean_tan_off_nadir(max_off_nadir_deg)
    top = np.pi * trunk_radius_m**2 if slice_lo_m <= crown_base_m <= slice_hi_m else 0.0
    return float(pulse_density_m2 * crown_transmission * (side + top))


def required_pulse_density(
    returns_needed: float,
    trunk_radius_m: float,
    slice_lo_m: float,
    slice_hi_m: float,
    *,
    crown_base_m: float,
    crown_transmission: float,
    max_off_nadir_deg: float = 20.0,
) -> float:
    """Pulse density (per m² of ground) at which a trunk expects ``returns_needed``."""
    per_unit = expected_trunk_returns(
        1.0,
        trunk_radius_m,
        slice_lo_m,
        slice_hi_m,
        crown_base_m=crown_base_m,
        crown_transmission=crown_transmission,
        max_off_nadir_deg=max_off_nadir_deg,
    )
    return float("inf") if per_unit <= 0.0 else returns_needed / per_unit


@dataclass(frozen=True)
class AlsScanConfig:
    """The airborne survey being imitated. Defaults: MML-like national ALS."""

    pulse_density_m2: float = 5.0
    max_off_nadir_deg: float = 20.0
    crown_base_m: float = 8.0
    """Height below which the trunk is bare. Askola's spruce-heavy stands are
    ~7 m at a 15 m canopy (droneplanner CLAUDE.md, crown ratio 0.55)."""
    crown_transmission: float = 0.2
    """Share of pulses that reach the trunk space with energy left for a return.
    Closed Nordic stands transmit ~5-20 % leaf-on (the planner's
    ``sub_canopy_gap_fraction`` default is 0.10)."""
    range_noise_m: float = 0.03
    ground_return_fraction: float = 0.05
    """Ground returns are kept at this rate so the cloud is not trunk-only; they sit
    below the detector's slice and only matter as a sanity check."""


def _entry_heights(
    dx: np.ndarray,
    dy: np.ndarray,
    r: np.ndarray,
    s: np.ndarray,
    ux: np.ndarray,
    uy: np.ndarray,
    zb: float,
) -> np.ndarray:
    """Height at which each (pulse, trunk) pair first meets, or -1 for a miss.

    The pulse's plan position at height ``z`` is ``g + z*s*u`` (``g`` its ground
    point, ``d = g - c`` relative to the trunk axis); solve ``|d + z*s*u|^2 = r^2``
    and take the upper root — the descending pulse enters from above. A pulse
    already inside the trunk at the crown base meets it where it leaves the crown;
    a vertical pulse hits at the crown base iff its ground point is inside the disc.
    """
    a = s * s
    b = 2.0 * s * (dx * ux + dy * uy)
    c = dx * dx + dy * dy - r * r
    out = np.full(len(dx), -1.0)
    vertical = a < 1e-12
    out[vertical & (c <= 0.0)] = zb
    tilted = ~vertical
    disc = b * b - 4.0 * a * c
    ok = tilted & (disc >= 0.0)
    sq = np.sqrt(np.where(ok, disc, 0.0))
    a_safe = np.where(tilted, a, 1.0)
    z_hi = (-b + sq) / (2.0 * a_safe)
    z_lo = (-b - sq) / (2.0 * a_safe)
    inside_at_base = ok & (z_hi > zb) & (z_lo <= zb)
    out[inside_at_base] = zb
    normal = ok & (z_hi >= 0.0) & (z_hi <= zb)
    out[normal] = z_hi[normal]
    return out


def simulate_als_scan(
    stems: StemMap,
    area_bounds: tuple[float, float, float, float],
    config: AlsScanConfig | None = None,
    *,
    ground_elev: float = 0.0,
    seed: int = 0,
) -> np.ndarray:
    """Trace airborne pulses through a crown layer onto trunks; return (n, 3) points.

    Each pulse aims at a uniform ground point, arrives ``theta`` off nadir
    (uniform on [0, max]) from a uniform azimuth — overlapping strips flown in
    different directions — and survives the crown with probability
    ``crown_transmission``. Below the crown base it is traced to the first trunk it
    meets (trunks shadow each other), else to the ground.
    """
    cfg = config or AlsScanConfig()
    rng = np.random.default_rng(seed)
    x0, y0, x1, y1 = area_bounds
    area = (x1 - x0) * (y1 - y0)
    n = int(rng.poisson(cfg.pulse_density_m2 * area))
    if n == 0:
        return np.empty((0, 3), dtype=float)

    gx = rng.uniform(x0, x1, n)
    gy = rng.uniform(y0, y1, n)
    through = rng.random(n) < cfg.crown_transmission
    gx, gy = gx[through], gy[through]
    m = len(gx)
    theta = rng.uniform(0.0, np.radians(cfg.max_off_nadir_deg), m)
    phi = rng.uniform(0.0, 2.0 * np.pi, m)
    slope = np.tan(theta)  # horizontal metres per metre of height
    ux, uy = np.cos(phi), np.sin(phi)
    zb = cfg.crown_base_m
    # Horizontal track from the ground point (z = 0) back up to the crown base.
    reach = slope * zb

    hit_z = np.full(m, -1.0)
    hit_x = gx.copy()
    hit_y = gy.copy()
    if len(stems):
        rmax = float(np.max(stems.radius_m))
        tree = stems.tree
        mid_x = gx + 0.5 * reach * ux
        mid_y = gy + 0.5 * reach * uy
        cand = tree.query_ball_point(np.c_[mid_x, mid_y], r=0.5 * reach + rmax)
        lengths = np.fromiter((len(c) for c in cand), dtype=np.int64, count=m)
        if lengths.sum():
            ip = np.repeat(np.arange(m), lengths)
            ks = np.fromiter((k for c in cand for k in c), dtype=np.int64, count=int(lengths.sum()))
            z = _entry_heights(
                gx[ip] - stems.x[ks],
                gy[ip] - stems.y[ks],
                stems.radius_m[ks],
                slope[ip],
                ux[ip],
                uy[ip],
                zb,
            )
            # The highest entry is the first trunk the descending pulse meets.
            np.maximum.at(hit_z, ip, z)
            hit = hit_z >= 0.0
            hit_x[hit] = gx[hit] + hit_z[hit] * slope[hit] * ux[hit]
            hit_y[hit] = gy[hit] + hit_z[hit] * slope[hit] * uy[hit]

    trunk = hit_z >= 0.0
    noise = rng.normal(0.0, cfg.range_noise_m, (int(trunk.sum()), 3))
    trunk_pts = np.c_[hit_x[trunk], hit_y[trunk], ground_elev + hit_z[trunk]] + noise
    ground_mask = ~trunk & (rng.random(m) < cfg.ground_return_fraction)
    ground_pts = np.c_[
        gx[ground_mask],
        gy[ground_mask],
        ground_elev + rng.normal(0.0, cfg.range_noise_m, int(ground_mask.sum())),
    ]
    return np.vstack([trunk_pts, ground_pts])


def returns_per_stem(
    cloud: np.ndarray,
    stems: StemMap,
    slice_lo_m: float,
    slice_hi_m: float,
    *,
    ground_elev: float = 0.0,
    within_m: float = 0.1,
) -> np.ndarray:
    """Count cloud points in the slice lying on each stem (within ``within_m`` of it)."""
    counts = np.zeros(len(stems), dtype=np.int64)
    if len(cloud) == 0 or len(stems) == 0:
        return counts
    agl = cloud[:, 2] - ground_elev
    sl = cloud[(agl >= slice_lo_m) & (agl <= slice_hi_m)]
    if len(sl) == 0:
        return counts
    d, k = cKDTree(np.c_[stems.x, stems.y]).query(sl[:, :2])
    on = d <= stems.radius_m[k] + within_m
    np.add.at(counts, k[on], 1)
    return counts
