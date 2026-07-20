"""End-to-end planning pipeline: mission -> plan(s) + report -> output files.

This is the orchestration the CLI calls. In M0 it runs the straight-line stub;
later milestones swap ``_plan_route`` for the real corridor/3D planners without
changing the output contract or the written artifacts.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from droneplanner.crossings.roads import rasterize_roads
from droneplanner.forest.navigability import FlyabilityConfig, FlyabilityField, build_flyability
from droneplanner.forest.voxelize import voxelize
from droneplanner.io.mavlink import write_plan
from droneplanner.io.mission import load_mission
from droneplanner.io.pointcloud import read_pointcloud
from droneplanner.io.raster import read_raster
from droneplanner.io.report import write_report
from droneplanner.io.trajectory import write_trajectory
from droneplanner.io.vector import read_vector
from droneplanner.logging_config import get_logger
from droneplanner.model.grid import Grid
from droneplanner.model.mission import Mission
from droneplanner.model.plan import Plan
from droneplanner.model.report import RiskReport
from droneplanner.planner.corridor import plan_over_terrain
from droneplanner.planner.stub import straight_line_plan
from droneplanner.report.build import build_report

_log = get_logger(__name__)

# Vertical voxel size for the forest occupancy grid (meters).
VOXEL_SIZE_Z = 0.5


@dataclass(frozen=True)
class PlanResult:
    """A planned route paired with its report and the files written for it."""

    plan: Plan
    report: RiskReport
    trajectory_path: Path
    plan_path: Path
    report_json_path: Path
    report_md_path: Path


def _seed_everything(seed: int) -> None:
    """Seed all RNGs from the single master seed (Software-Spec §0 determinism)."""
    random.seed(seed)
    np.random.seed(seed % (2**32))


def _load_terrain(mission: Mission) -> tuple[np.ndarray, np.ndarray, Grid] | None:
    """Load aligned DTM+DSM from the mission's layers, or None if not supplied."""
    if mission.layers.dtm is None or mission.layers.dsm is None:
        return None
    dtm_r = read_raster(mission.layers.dtm)
    dsm_r = read_raster(mission.layers.dsm)
    if dtm_r.shape != dsm_r.shape or dtm_r.transform != dsm_r.transform:
        raise ValueError("DTM and DSM must share the same grid (shape + transform)")
    grid = Grid(
        transform=dtm_r.transform, rows=dtm_r.shape[0], cols=dtm_r.shape[1], crs=mission.crs
    )
    return dtm_r.data, dsm_r.data, grid


def _load_road_mask(mission: Mission, grid: Grid) -> np.ndarray | None:
    """Rasterize the roads layer to an on-road mask, or None if not supplied."""
    if mission.layers.roads is None:
        return None
    return rasterize_roads(read_vector(mission.layers.roads), grid)


def _load_flyability(mission: Mission, grid: Grid) -> FlyabilityField | None:
    """Voxelize the lidar layer and build the forest flyability field, or None."""
    if mission.layers.point_cloud is None:
        return None
    pc = read_pointcloud(mission.layers.point_cloud)
    if pc.count == 0:
        return None
    z = pc.xyz[:, 2]
    voxels = voxelize(
        pc.xyz,
        grid,
        z_min=float(z.min()),
        z_max=float(z.max()) + VOXEL_SIZE_Z,
        voxel_size_z=VOXEL_SIZE_Z,
    )
    return build_flyability(voxels, FlyabilityConfig.from_drone(mission.drone))


def _plan_route(mission: Mission, route_index: int) -> tuple[Plan, bool, bool]:
    """Plan one route, returning (plan, concealment_analyzed, flyability_analyzed).

    Runs the M1/M2 corridor planner when a DTM+DSM is supplied (adding road
    crossings and forest flyability when those layers exist), otherwise the M0
    straight-line stub.
    """
    terrain = _load_terrain(mission)
    if terrain is None:
        return straight_line_plan(mission, route_index=route_index), False, False
    dtm, dsm, grid = terrain
    road_mask = _load_road_mask(mission, grid)
    flyability = _load_flyability(mission, grid)
    plan, _surface = plan_over_terrain(
        mission,
        dtm,
        dsm,
        grid,
        road_mask=road_mask,
        flyability=flyability,
        route_index=route_index,
    )
    return plan, True, flyability is not None


def plan_mission(
    mission: Mission,
    out_dir: str | Path,
    *,
    generated_at: datetime | None = None,
) -> list[PlanResult]:
    """Plan a mission's ``k`` routes and write all output artifacts.

    Returns one ``PlanResult`` per route. Determinism: identical mission + seed
    yields byte-identical trajectory/plan/report (``generated_at`` aside).
    """
    _seed_everything(mission.seed)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    results: list[PlanResult] = []
    multi = mission.k_routes > 1
    for route_index in range(mission.k_routes):
        plan, concealment_analyzed, flyability_analyzed = _plan_route(mission, route_index)
        report = build_report(
            mission,
            plan,
            generated_at=generated_at,
            concealment_analyzed=concealment_analyzed,
            flyability_analyzed=flyability_analyzed,
        )

        suffix = f".route{route_index}" if multi else ""
        trajectory_path = out / f"trajectory{suffix}.geojson"
        plan_path = out / f"mission{suffix}.plan"
        report_json_path = out / f"report{suffix}.json"
        report_md_path = out / f"report{suffix}.md"

        write_trajectory(trajectory_path, plan)
        write_plan(plan_path, plan)
        write_report(report_json_path, report_md_path, report)

        _log.info(
            "planned route %d/%d: %.0f m, %s",
            route_index + 1,
            mission.k_routes,
            plan.total_length_m,
            report.recommendation.value,
        )
        results.append(
            PlanResult(
                plan=plan,
                report=report,
                trajectory_path=trajectory_path,
                plan_path=plan_path,
                report_json_path=report_json_path,
                report_md_path=report_md_path,
            )
        )
    return results


def plan_from_file(
    mission_path: str | Path,
    out_dir: str | Path,
    *,
    seed_override: int | None = None,
    generated_at: datetime | None = None,
) -> list[PlanResult]:
    """Load a mission YAML and plan it end-to-end."""
    mission = load_mission(mission_path)
    if seed_override is not None:
        mission = mission.model_copy(update={"seed": seed_override})
    return plan_mission(mission, out_dir, generated_at=generated_at)
