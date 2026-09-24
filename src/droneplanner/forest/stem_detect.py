"""Detect individual stems in a point cloud (SHORTFALLS M2 — the real stem map).

WHY THIS EXISTS
Routing between mapped stems is the only surviving architecture for below-canopy
flight — reactive avoidance is dead after four independent negatives, and the
stem-routed plan flew 330/330 through 2500 trunks with **0 strikes** where the
reactive arm took 36. But every stem map behind that result is **synthetic**:
``io/forest_models.generate_trunks`` *invents* trunks from a canopy height model.
It is a generator, not a detector, so nothing so far says whether real stems can
be found at all. This module is the detector.

WHAT IT DOES, AND WHY THIS ALGORITHM
A stem is a near-vertical, near-cylindrical cluster of returns in the space above
the understory and below the crown. So:

1. **Height-normalize** against the DTM. Trunk space is defined relative to the
   ground under each point, not to a global datum — the same per-column datum
   mistake that made the M2 navigable band wrong until 2026-09-14.
2. **Slice** to ``[slice_lo_m, slice_hi_m]`` AGL. Low enough to be under the crown,
   high enough to clear understory and ground-return noise. 1.5-5.0 m is the
   conventional band and brackets breast height (1.3 m), where DBH is defined.
3. **Cluster in plan view** with single-linkage at ``cluster_eps_m``. A trunk's
   returns project to a blob a few decimetres across; the gap to the next stem is
   metres (4.6 m median on the demo manifests). The separation is large, which is
   what makes this tractable.
4. **Reject** clusters that are not stem-shaped: too few points, too little
   vertical extent (a bush or a rock is not a stem), or too wide in plan (two
   merged trunks, or a branch).
5. **Fit** a centre and a radius per surviving cluster.

⚠️ **WHAT THIS IS NOT.** It is not a canopy-penetration model. Whether the returns
exist at all is a property of the *sensor and the flight*, not of this code:
photogrammetry reconstructs the canopy top and gives you the DSM you already have,
so stems need canopy-penetrating returns — airborne or (better) UAV lidar, or an
under-canopy survey. Feeding this module an aerial-photogrammetry cloud will find
nothing, and correctly so.

🔑 **Accuracy is the requirement, not detection rate.** The stem-routed flight had
**1.387 m of planned clearance and only 0.708 m of flown clearance** once PX4's
tracking error was paid. A stem map whose positions are wrong by 1-2 m — which is
what ALS crown-centroid inference typically gives — consumes that entire budget
before the aircraft has moved. So a detection is useless unless its *position* is
good, and :func:`match_stems` exists to measure exactly that against a known
truth.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from droneplanner.fields.stems import StemMap
from droneplanner.model.grid import Grid


@dataclass(frozen=True)
class StemDetectConfig:
    """Thresholds for stem detection. Defaults are conventional, not calibrated."""

    slice_lo_m: float = 1.5
    """Bottom of the trunk-space slice, AGL. Above understory and ground noise."""

    slice_hi_m: float = 5.0
    """Top of the slice. Below the crown base in a mature stand."""

    cluster_eps_m: float = 0.35
    """Single-linkage radius in plan view. Must exceed the widest trunk's radius
    plus range noise, and stay well under the stem spacing (4.6 m here)."""

    min_points: int = 8
    """Below this a cluster is noise. Scales with point density — a sparse cloud
    needs this lowered, which is exactly the trade the density sweep measures."""

    min_vertical_extent_m: float = 1.0
    """A stem spans the slice; a rock, a stump top or a bush does not."""

    max_radius_m: float = 0.6
    """Plan-view radius above which a cluster is two merged stems or a branch,
    not one trunk. Rejecting is the safe direction: a *missed* stem is an obstacle
    the planner will not avoid, but a *merged* stem is a phantom centre sitting in
    the gap between two real trunks — which is worse, because the planner would
    route confidently into the trunk it thinks is clear."""

    default_radius_m: float = 0.15
    """Radius assigned when a cluster is too sparse to size. Matches the synthetic
    manifests so a detected map is comparable with a generated one."""


@dataclass(frozen=True)
class StemDetectStats:
    """What the detector saw on the way to its answer."""

    points_in: int
    points_in_slice: int
    clusters: int
    rejected_few_points: int
    rejected_thin: int
    rejected_wide: int
    detected: int


def _cluster_xy(xy: np.ndarray, eps: float) -> np.ndarray:
    """Single-linkage clusters of 2D points within ``eps``; returns labels.

    Union-find over the KD-tree's pairs rather than a distance matrix: a stand of
    a few thousand stems at realistic density is ~1e5-1e6 returns, and an O(n^2)
    linkage is not an option.
    """
    n = len(xy)
    parent = np.arange(n)

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = int(parent[a])
        return a

    for i, j in cKDTree(xy).query_pairs(eps):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj
    return np.array([find(i) for i in range(n)])


def _fit_circle(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float] | None:
    """Algebraic (Kasa) circle fit; returns (cx, cy, r) or None if ill-conditioned.

    🔑 **This is not a refinement, it removes a systematic error.** A scanner sees
    one side of a trunk, and the CENTROID of an arc is not its centre — for a half
    arc it sits ``2r/pi`` = 0.64r in front of the axis, toward the scanner. Measured
    on the synthetic fixture that is 0.096 m of RMSE for a 0.15 m trunk against an
    analytic 0.095 m: the whole error was this bias. It does not average out over a
    stand, because every stem is biased toward wherever it was seen from, so a
    centroid map is *shifted*, not merely noisy.
    """
    n = len(x)
    if n < 4:
        return None
    mx, my = float(x.mean()), float(y.mean())
    u, v = x - mx, y - my
    # Solve the linear system for the centre of the circle through these points.
    suu, svv, suv = float((u * u).sum()), float((v * v).sum()), float((u * v).sum())
    suuu, svvv = float((u * u * u).sum()), float((v * v * v).sum())
    suvv, svuu = float((u * v * v).sum()), float((v * u * u).sum())
    det = 2.0 * (suu * svv - suv * suv)
    if abs(det) < 1e-12:
        return None
    uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / det
    vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / det
    cx, cy = mx + uc, my + vc
    r = float(np.sqrt(np.mean((x - cx) ** 2 + (y - cy) ** 2)))
    return cx, cy, r


def detect_stems(
    xyz: np.ndarray,
    dtm: np.ndarray,
    grid: Grid,
    config: StemDetectConfig | None = None,
) -> tuple[StemMap, StemDetectStats]:
    """Find individual stems in a height-normalized slice of ``xyz``.

    ``xyz`` is in world coordinates (the same CRS as ``grid``); ``dtm`` supplies
    the per-point ground elevation. Returns the map and the bookkeeping behind it —
    the rejection counts matter, because a detector that finds nothing and a
    detector that rejects everything look identical from the map alone.
    """
    cfg = config or StemDetectConfig()
    pts = np.asarray(xyz, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("xyz must be an (n, 3) array")
    if pts.size == 0:
        empty = StemMap(np.empty(0), np.empty(0), np.empty(0))
        return empty, StemDetectStats(0, 0, 0, 0, 0, 0, 0)

    a, b, c, d, e, f = grid.transform
    det = a * e - b * d
    wx, wy = pts[:, 0] - c, pts[:, 1] - f
    col = np.floor((e * wx - b * wy) / det).astype(int)
    row = np.floor((-d * wx + a * wy) / det).astype(int)
    inside = (row >= 0) & (row < grid.rows) & (col >= 0) & (col < grid.cols)

    agl = np.full(len(pts), -np.inf)
    agl[inside] = pts[inside, 2] - dtm[row[inside], col[inside]]
    in_slice = inside & (agl >= cfg.slice_lo_m) & (agl <= cfg.slice_hi_m)
    sliced = pts[in_slice]
    if len(sliced) == 0:
        empty = StemMap(np.empty(0), np.empty(0), np.empty(0))
        return empty, StemDetectStats(len(pts), 0, 0, 0, 0, 0, 0)

    labels = _cluster_xy(sliced[:, :2], cfg.cluster_eps_m)
    xs: list[float] = []
    ys: list[float] = []
    radii: list[float] = []
    few = thin = wide = 0
    for label in np.unique(labels):
        member = sliced[labels == label]
        if len(member) < cfg.min_points:
            few += 1
            continue
        if member[:, 2].max() - member[:, 2].min() < cfg.min_vertical_extent_m:
            thin += 1
            continue
        cx, cy = float(member[:, 0].mean()), float(member[:, 1].mean())
        spread = float(np.hypot(member[:, 0] - cx, member[:, 1] - cy).max())
        if spread > cfg.max_radius_m:
            wide += 1
            continue
        radius = cfg.default_radius_m
        fit = _fit_circle(member[:, 0], member[:, 1])
        if fit is not None:
            fcx, fcy, fr = fit
            # Trust the fit only if it lands somewhere sane: an arc shorter than
            # ~90 deg makes the algebraic fit run away, and a runaway centre is a
            # phantom trunk in the middle of a gap, which is worse than a biased
            # one. Falling back to the centroid keeps the error bounded by 0.64r.
            if fr <= cfg.max_radius_m and np.hypot(fcx - cx, fcy - cy) <= cfg.max_radius_m:
                cx, cy, radius = fcx, fcy, max(fr, 0.01)
        xs.append(cx)
        ys.append(cy)
        radii.append(radius)

    stems = StemMap(
        x=np.asarray(xs, dtype=float),
        y=np.asarray(ys, dtype=float),
        radius_m=np.asarray(radii, dtype=float),
    )
    stats = StemDetectStats(
        points_in=len(pts),
        points_in_slice=len(sliced),
        clusters=len(np.unique(labels)),
        rejected_few_points=few,
        rejected_thin=thin,
        rejected_wide=wide,
        detected=len(xs),
    )
    return stems, stats


@dataclass(frozen=True)
class StemMatchReport:
    """Detected map scored against a known truth — the acceptance measurement."""

    truth: int
    detected: int
    matched: int
    """Truth stems with a detection inside ``tolerance_m``."""

    missed: int
    """Truth stems with no detection. **The dangerous failure**: an obstacle the
    planner will route straight through, exactly as the unrefined route did."""

    spurious: int
    """Detections matching no truth stem. Costs clearance and route length, but
    the aircraft does not hit a tree that is not there."""

    position_rmse_m: float
    position_p95_m: float
    max_error_m: float

    @property
    def detection_rate(self) -> float:
        return self.matched / self.truth if self.truth else 0.0


def match_stems(detected: StemMap, truth: StemMap, tolerance_m: float = 1.0) -> StemMatchReport:
    """Nearest-neighbour match detections to truth, one-to-one.

    Position error is what decides whether a map is usable: the flown clearance on
    the stem-routed sortie was **0.708 m**, so an error comparable to that puts the
    aircraft into a trunk the map said it had cleared.
    """
    if len(truth) == 0:
        return StemMatchReport(0, len(detected), 0, 0, len(detected), 0.0, 0.0, 0.0)
    if len(detected) == 0:
        return StemMatchReport(len(truth), 0, 0, len(truth), 0, float("nan"), float("nan"), 0.0)

    tree = cKDTree(np.c_[detected.x, detected.y])
    dist, idx = tree.query(np.c_[truth.x, truth.y], k=1)
    claimed: dict[int, tuple[int, float]] = {}
    for t, (dd, ii) in enumerate(zip(dist, idx, strict=True)):
        if dd > tolerance_m:
            continue
        prev = claimed.get(int(ii))
        if prev is None or dd < prev[1]:
            claimed[int(ii)] = (t, float(dd))
    errors = np.array([d for _, d in claimed.values()]) if claimed else np.empty(0)
    matched = len(claimed)
    return StemMatchReport(
        truth=len(truth),
        detected=len(detected),
        matched=matched,
        missed=len(truth) - matched,
        spurious=len(detected) - matched,
        position_rmse_m=float(np.sqrt((errors**2).mean())) if errors.size else float("nan"),
        position_p95_m=float(np.percentile(errors, 95)) if errors.size else float("nan"),
        max_error_m=float(errors.max()) if errors.size else 0.0,
    )
