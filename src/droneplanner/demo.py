"""Built-in procedural demo area of operations.

Lets the M1 planner run — and the visualizer/interactive explorer show it — with
zero input data. The terrain is a small analytic scene: two observer hills and a
ridge with an off-center saddle between start and end, so the naive straight line
crosses an exposed crest and the concealment-aware planner has an interesting
detour to find (through the saddle / lower-exposure flank).

THE SCENE AND THE GRID ARE NOT THE SAME EXTENT (roadmap C0)
The scene above is drawn at scene indices ``0..size-1``; the exported *grid* can
be larger. The concealment study needs somewhere to stand a threat sensor at
realistic standoff **behind** the launch point, and the scene alone does not
provide it: with the historical 4x4 km AO the start sits ~400 m from its own SW
corner, so every "receding threat" measurement was taken against sensors ~336 m
from the CP — a worst case, not a representative one, and one where the sensor
sees the CP directly (``first_contact_m == 0`` on every arm).

:class:`DemoLayout` therefore pads cells to the **south and west** — the reverse
of the route bearing — until :func:`reverse_bearing_standoff_m` reaches the
requested margin. The padding is analytic continuation of the same surface, not
new scenery, and the scene keeps its historical world coordinates, so start, end,
observers and the terrain under them are all unchanged. ⚠️ The *route* can still
move a little: ``fields.terrain`` and ``fields.survivability`` normalize over the
whole raster, so extending the extent rescales those layers everywhere.

This is synthetic scenery for demonstration only; real runs supply a DTM/DSM.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from droneplanner.forest.navigability import FlyabilityConfig, FlyabilityField, build_flyability
from droneplanner.forest.voxelize import VoxelGrid
from droneplanner.io.raster import RasterGrid, write_raster
from droneplanner.model.geometry import Coordinate
from droneplanner.model.grid import Grid
from droneplanner.model.mission import DroneSpec, Mission, SpoolSpec, ThreatModel, Weights

DEMO_CRS = "EPSG:32633"
_X0 = 500_000.0
_Y0 = 5_200_000.0

# Standoff the exported AO guarantees behind the start, so a threat sensor has
# somewhere realistic to stand. 2.5 km is chosen against measured C-UAS numbers:
# Robin Radar's IRIS classifies a 3 kg drone at ~2 km typical, and this airframe
# is 6.5 kg plus a spool, so a 2 km detection gate must be *inside* the raster
# with room to spare or the gate is testing the fixture instead of the route.
THREAT_MARGIN_M = 2500.0


@dataclass(frozen=True)
class DemoLayout:
    """Where the analytic scene sits inside the exported grid.

    The scene occupies scene indices ``0..size-1`` in both axes and keeps its
    historical world footprint (SW corner at ``(_X0, _Y0)``, side
    ``size * cell_m``). ``margin_cells`` pads the grid to the south and west —
    behind the start on the reverse-of-route bearing — leaving every scene cell's
    world coordinate and elevation untouched.

    Scene indices are used for *feature placement* and run ``0..size-1``; grid
    indices are what the raster is addressed by. Rows are padded at the bottom
    (south) so a scene row equals its grid row; columns are padded on the left
    (west) so a scene column is ``margin_cells`` less than its grid column.
    """

    size: int
    margin_cells: int = 0
    cell_m: float = 25.0

    @property
    def rows(self) -> int:
        return self.size + self.margin_cells

    @property
    def cols(self) -> int:
        return self.size + self.margin_cells

    def grid(self, crs: str = DEMO_CRS) -> Grid:
        """The padded raster grid. The scene's world footprint is unchanged."""
        transform = (
            self.cell_m,
            0.0,
            _X0 - self.margin_cells * self.cell_m,
            0.0,
            -self.cell_m,
            _Y0 + self.size * self.cell_m,
        )
        return Grid(transform=transform, rows=self.rows, cols=self.cols, crs=crs)

    def scene_mgrid(self) -> tuple[np.ndarray, np.ndarray]:
        """``(yy, xx)`` scene indices for every grid cell — negative in the padding."""
        yy, xx = np.mgrid[0 : self.rows, 0 : self.cols].astype(np.float64)
        return yy, xx - float(self.margin_cells)

    def scene_pixel(self, scene_row: int, scene_col: int) -> tuple[int, int]:
        """Grid ``(row, col)`` of a scene cell."""
        return scene_row, scene_col + self.margin_cells

    def endpoints(self) -> tuple[Coordinate, Coordinate]:
        """``(start, end)`` — opposite scene corners, 12 cells in from the edge."""
        grid = self.grid()
        start = grid.pixel_to_world(*self.scene_pixel(self.size - 12, 12))
        end = grid.pixel_to_world(*self.scene_pixel(12, self.size - 12))
        return start, end

    def threat_standoff_m(self) -> float:
        """Metres of grid behind the start on the reverse-of-route bearing."""
        start, end = self.endpoints()
        return reverse_bearing_standoff_m(self.grid(), start, end)


def reverse_bearing_standoff_m(grid: Grid, start: Coordinate, end: Coordinate) -> float:
    """Metres of ``grid`` remaining behind ``start``, opposite the route bearing.

    How far a threat sensor can be placed *behind* the launch point and still sit
    on the raster — the quantity roadmap C0 exists to make non-trivial. A sensor
    off the raster cannot be modelled at all, so this bounds what standoff the
    fixture can express.
    """
    a, b, c, d, e, f = grid.transform
    if b != 0.0 or d != 0.0:
        raise ValueError("standoff is defined for north-up (axis-aligned) grids only")
    dx, dy = start.x - end.x, start.y - end.y
    norm = math.hypot(dx, dy)
    if norm == 0.0:
        raise ValueError("start and end must be distinct")
    ux, uy = dx / norm, dy / norm

    x_lo, x_hi = sorted((c, c + a * grid.cols))
    y_lo, y_hi = sorted((f, f + e * grid.rows))
    limits: list[float] = []
    if ux > 0.0:
        limits.append((x_hi - start.x) / ux)
    elif ux < 0.0:
        limits.append((x_lo - start.x) / ux)
    if uy > 0.0:
        limits.append((y_hi - start.y) / uy)
    elif uy < 0.0:
        limits.append((y_lo - start.y) / uy)
    return min(limits) if limits else math.inf


def point_behind_start(start: Coordinate, end: Coordinate, distance_m: float) -> Coordinate:
    """The point ``distance_m`` behind ``start``, opposite the route bearing.

    Where a threat sensor at that standoff goes — this is what turns C0's margin
    into the ``--enemy x y`` arguments the G1 gate is run with.
    """
    dx, dy = start.x - end.x, start.y - end.y
    norm = math.hypot(dx, dy)
    if norm == 0.0:
        raise ValueError("start and end must be distinct")
    return Coordinate(x=start.x + distance_m * dx / norm, y=start.y + distance_m * dy / norm)


def demo_layout(
    size: int = 160, cell_m: float = 25.0, threat_margin_m: float = THREAT_MARGIN_M
) -> DemoLayout:
    """The layout :func:`export_demo_ao` will use, without writing any rasters."""
    return _layout_for_margin(size, cell_m, threat_margin_m)


def _layout_for_margin(size: int, cell_m: float, threat_margin_m: float) -> DemoLayout:
    """Smallest layout whose reverse-bearing standoff reaches ``threat_margin_m``."""
    layout = DemoLayout(size=size, cell_m=cell_m)
    if threat_margin_m <= 0.0:
        return layout
    # A padded cell buys cell_m / max(|ux|, |uy|) of standoff — 35.4 m per 25 m cell
    # on this AO's 45° route bearing — so measuring the gain rather than assuming
    # cell_m lands on the *smallest* grid that clears the margin. Padding is
    # quadratic in cells, and every extra cell costs viewshed and search time.
    # Looped rather than inverted because the binding edge can change with bearing.
    while (have := layout.threat_standoff_m()) < threat_margin_m:
        stepped = replace(layout, margin_cells=layout.margin_cells + 1)
        per_cell = stepped.threat_standoff_m() - have
        extra = max(1, math.ceil((threat_margin_m - have) / per_cell))
        layout = replace(layout, margin_cells=layout.margin_cells + extra)
    return layout


def _layout_of(grid: Grid) -> DemoLayout:
    """Recover the layout of a grid built by :func:`demo_terrain`.

    The padding is readable off the transform — the scene's west edge is pinned at
    ``_X0`` — which keeps ``demo_mission`` and ``demo_flyability`` consistent with
    the terrain without threading a layout through every call site. Assumes a demo
    grid; a foreign grid is treated as unpadded.
    """
    cell_m = grid.cell_size_m
    margin = round((_X0 - grid.transform[2]) / cell_m)
    margin = min(max(margin, 0), grid.rows - 1)
    return DemoLayout(size=grid.rows - margin, margin_cells=margin, cell_m=cell_m)


def _gauss(
    xx: np.ndarray, yy: np.ndarray, cx: float, cy: float, amp: float, sig: float
) -> np.ndarray:
    return amp * np.exp(-(((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * sig**2)))


def demo_terrain(
    size: int = 160, cell_m: float = 25.0, threat_margin_m: float = 0.0
) -> tuple[np.ndarray, np.ndarray, Grid]:
    """Return (dtm, dsm, grid) for the analytic demo AO.

    ``size`` is the **scene** side in cells; ``threat_margin_m`` pads the grid to
    the south and west until a threat sensor can stand that far behind the start
    (see :class:`DemoLayout`). The padding continues the same analytic surface, so
    the scene's own terrain is bit-identical either way.
    """
    layout = _layout_for_margin(size, cell_m, threat_margin_m)
    yy, xx = layout.scene_mgrid()

    dtm = np.full((layout.rows, layout.cols), 40.0)
    dtm += 0.12 * (xx + yy)  # gentle regional tilt

    # Two observer hills flanking the centre.
    dtm += _gauss(xx, yy, cx=0.42 * size, cy=0.30 * size, amp=95.0, sig=0.10 * size)
    dtm += _gauss(xx, yy, cx=0.62 * size, cy=0.70 * size, amp=85.0, sig=0.10 * size)

    # A ridge along the anti-diagonal, with a low saddle offset toward one end.
    d_anti = (xx + yy - size) / np.sqrt(2.0)  # signed distance to anti-diagonal
    ridge = 70.0 * np.exp(-(d_anti**2) / (2.0 * (0.06 * size) ** 2))
    saddle = 1.0 - 0.75 * np.exp(
        -((xx - 0.30 * size) ** 2 + (yy - 0.70 * size) ** 2) / (2 * (0.12 * size) ** 2)
    )
    dtm += ridge * saddle

    # Flanking valleys (defilade) to reward.
    dtm -= _gauss(xx, yy, cx=0.25 * size, cy=0.78 * size, amp=25.0, sig=0.11 * size)
    dtm -= _gauss(xx, yy, cx=0.78 * size, cy=0.25 * size, amp=25.0, sig=0.11 * size)

    dtm = dtm.astype(np.float32)
    dsm = (dtm + demo_canopy(layout)).astype(np.float32)
    return dtm, dsm, layout.grid()


def demo_canopy(layout: DemoLayout) -> np.ndarray:
    """Canopy height (m) for the demo AO — mature stands, clearings, and scrub.

    The DSM used to be a copy of the DTM: no canopy at all. That made the whole
    concealment story unfalsifiable — the cost surface rewarded "forest" that the
    line-of-sight model could not see, and the drone's altitude had nothing to
    hide under. The stands here are laid out so the concealed route and the short
    one disagree: a mature belt runs across the AO between the observer hills,
    broken by clearings the route has to cross or detour around.
    """
    size = layout.size
    yy, xx = layout.scene_mgrid()

    # A broad mature stand (22 m) over most of the AO, thinning to the north-east.
    canopy = 22.0 - 6.0 * (xx / size) - 4.0 * (yy / size)

    # Two clearings straddling the natural corridor: no cover, must be crossed low.
    canopy -= _gauss(xx, yy, cx=0.35 * size, cy=0.55 * size, amp=26.0, sig=0.07 * size)
    canopy -= _gauss(xx, yy, cx=0.70 * size, cy=0.40 * size, amp=26.0, sig=0.06 * size)

    # Regenerating scrub (too short to fly under) along the ridge crest.
    d_anti = (xx + yy - size) / np.sqrt(2.0)
    canopy -= 14.0 * np.exp(-(d_anti**2) / (2.0 * (0.05 * size) ** 2))

    chm: np.ndarray = np.clip(canopy, 0.0, None).astype(np.float32)
    return chm


def demo_mission(grid: Grid, weights: Weights | None = None) -> Mission:
    """A demo mission on the demo grid: opposite-corner start/end, two observers.

    Placement is in **scene** cells, so a grid padded for threat standoff (C0) puts
    start, end and both observers at exactly the same world coordinates as an
    unpadded one — the padding adds room behind the launch point, not a new route.
    """
    layout = _layout_of(grid)
    start, end = layout.endpoints()
    observers = (
        grid.pixel_to_world(*layout.scene_pixel(int(0.30 * layout.size), int(0.42 * layout.size))),
        grid.pixel_to_world(*layout.scene_pixel(int(0.70 * layout.size), int(0.62 * layout.size))),
    )
    return Mission(
        name="demo-ao",
        crs=grid.crs,
        start=Coordinate(x=start.x, y=start.y),
        end=Coordinate(x=end.x, y=end.y),
        drone=DroneSpec(
            rotor_span_m=0.9,
            mass_kg=6.5,
            min_turn_radius_m=8.0,
            climb_rate_max_ms=3.0,
            descent_rate_max_ms=2.0,
            cruise_speed_ms=6.0,
            # Usable hover endurance with the 5 km spool aboard (user, 2026-08-03):
            # a Blue-UAS-class electric multirotor of this mass carrying ~1.6 kg.
            # This is what makes TEST-PLAN N1 a check rather than a report.
            endurance_s=1800.0,
        ),
        spool=SpoolSpec(length_m=8000.0, mass_kg=1.6),
        threat=ThreatModel(observers=observers),
        weights=weights or Weights(),
        # Fly under the canopy, not over it. The stands are 12-22 m, so a 12 m
        # target sits inside them wherever the ceiling allows — and above ~12 m the
        # canopy ceiling binds instead of the target, so asking for 15 m changes
        # nothing. 12 m is also the lowest that clears terrain at *every* demo grid
        # size: this AO's DTM has 25 m cells, and the ground bulges ~5 m between
        # samples, which is what sets the floor (not the autopilot).
        cruise_agl_m=12.0,
        seed=1,
    )


def build_demo(
    size: int = 160, *, threat_margin_m: float = 0.0
) -> tuple[Mission, np.ndarray, np.ndarray, Grid]:
    """Convenience: (mission, dtm, dsm, grid) for the demo AO.

    ``threat_margin_m`` defaults to 0 here and to :data:`THREAT_MARGIN_M` in
    :func:`export_demo_ao`. The in-memory scene is scenery for ``viz``/``serve``
    and for unit tests that build 60-120 cell grids, where a 2.5 km margin would
    be most of the raster; the *exported* AO is what the concealment study plans
    over and is the one that has to hold a sensor at standoff.
    """
    dtm, dsm, grid = demo_terrain(size=size, threat_margin_m=threat_margin_m)
    mission = demo_mission(grid)
    return mission, dtm, dsm, grid


def demo_flyability(grid: Grid, drone: DroneSpec) -> FlyabilityField:
    """A synthetic forest flyability field for the demo AO (M2 demonstration).

    Canopy everywhere plus a dense, non-navigable clutter patch straddling the
    concealment corridor, so the M2 route has to divert around it. This is
    illustrative scenery built directly at the demo grid resolution — real runs
    voxelize an actual lidar survey.
    """
    layout = _layout_of(grid)
    size, rows, cols = layout.size, grid.rows, grid.cols
    nz, vz = 26, 0.5
    yy, xx = layout.scene_mgrid()

    occ = np.zeros((nz, rows, cols), dtype=bool)
    occ[0, :, :] = True  # ground plane (so clearance peaks in the upper trunk space)
    # Canopy base varies across the AO (lower on the left) so the navigable band
    # shifts and the drone's altitude has to vary (climb-limited) to follow it.
    # Clipped because scene coordinates go negative in C0's western padding.
    canopy_base = np.clip(16.0 + 8.0 * (xx / size), 1.0, nz - 1.0).astype(np.int64)
    levels = np.arange(nz)[:, None, None]
    occ |= levels >= canopy_base[None, :, :]

    # Dense trunk-space clutter (fills the flight band -> non-navigable) as an
    # ellipse placed on the lower-right stretch of the route.
    cy, cx = 0.60 * size, 0.56 * size
    ry, rx = 0.15 * size, 0.12 * size
    dense = ((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2 <= 1.0
    occ[0:22] |= dense[None, :, :]

    voxels = VoxelGrid(occupancy=occ, grid=grid, z0=0.0, voxel_size_z=vz)
    return build_flyability(voxels, FlyabilityConfig.from_drone(drone))


def _mission_doc(mission: Mission, grid: Grid, dtm_path: Path, dsm_path: Path) -> dict[str, Any]:
    """Serialize a Mission (plus layer paths) into mission-YAML form."""
    return {
        "name": "demo-ao",
        "crs": grid.crs,
        "start": {"x": float(mission.start.x), "y": float(mission.start.y)},
        "end": {"x": float(mission.end.x), "y": float(mission.end.y)},
        "drone": mission.drone.model_dump(mode="json"),
        "spool": mission.spool.model_dump(mode="json"),
        "threat": {
            "observers": [{"x": float(o.x), "y": float(o.y)} for o in mission.threat.observers]
        },
        "weights": mission.weights.model_dump(mode="json"),
        # Relative to the mission file, which is what the loader anchors them to,
        # so an exported AO can be moved or committed without breaking.
        "layers": {"dtm": str(dtm_path), "dsm": str(dsm_path)},
        "k_routes": mission.k_routes,
        "seed": mission.seed,
        "waypoint_spacing_m": mission.waypoint_spacing_m,
        "cruise_agl_m": mission.cruise_agl_m,
        "altitude": mission.altitude.model_dump(mode="json"),
    }


def export_demo_ao(
    out_dir: str | Path, size: int = 160, threat_margin_m: float = THREAT_MARGIN_M
) -> Path:
    """Write the procedural demo AO as real GeoTIFF layers + a mission YAML.

    The demo AO is otherwise consumed in-memory by ``viz``/``serve`` only, so the
    sole way to get a ``.plan`` was ``configs/mission.example.yaml`` — which
    supplies *no terrain* and therefore emits the straight-line M0 stub (constant
    altitude, zero turns). Flying that in SITL exercises none of the mission
    profile: the tether-safe speed policy never engages, and the altitude band is
    never tested.

    Exporting the AO as actual layers means ``droneplanner plan --mission
    <out>/mission.yaml`` takes the **same ingestion path a real AO does**
    (``read_raster`` → cost surface → cost-aware Theta\\* → kinematics), yielding
    a route with genuine turns and a terrain-following altitude profile. It is
    also a rehearsal for real DTM/DSM data, which enters by exactly this route.

    ``threat_margin_m`` pads the raster behind the start so a threat sensor can be
    placed at realistic standoff (roadmap C0). It does not move the route.

    Returns the path to the written mission YAML.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    dtm, dsm, grid = demo_terrain(size=size, threat_margin_m=threat_margin_m)
    mission = demo_mission(grid)

    dtm_path = out / "dtm.tif"
    dsm_path = out / "dsm.tif"
    for path, data in ((dtm_path, dtm), (dsm_path, dsm)):
        write_raster(
            path,
            RasterGrid(data=data, transform=grid.transform, crs=grid.crs, nodata=None),
        )

    mission_path = out / "mission.yaml"
    mission_path.write_text(
        yaml.safe_dump(
            _mission_doc(mission, grid, Path(dtm_path.name), Path(dsm_path.name)),
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return mission_path
