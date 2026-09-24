"""``droneplanner`` command-line interface (Software-Spec §3, §7).

Usage:
    droneplanner plan --mission configs/mission.example.yaml --out out/
    droneplanner version
"""

from __future__ import annotations

import logging
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path

import typer

from droneplanner import __version__
from droneplanner.io.mission import MissionLoadError
from droneplanner.logging_config import configure_logging
from droneplanner.pipeline import plan_from_file

app = typer.Typer(
    add_completion=False,
    help="Offline preflight route planner for the fiber-laying drone.",
    no_args_is_help=True,
)


@app.command()
def plan(
    mission: Path = typer.Option(
        ...,
        "--mission",
        "-m",
        exists=True,
        dir_okay=False,
        readable=True,
        help="Mission YAML file.",
    ),
    out: Path = typer.Option(
        ...,
        "--out",
        "-o",
        file_okay=False,
        help="Output directory for trajectory, .plan, and report.",
    ),
    seed: int | None = typer.Option(
        None, "--seed", help="Override the mission seed (reproducible runs)."
    ),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Verbose (DEBUG) logging."),
) -> None:
    """Plan a mission end-to-end and write the trajectory, .plan, and report."""
    configure_logging(logging.DEBUG if verbose else logging.INFO)
    try:
        results = plan_from_file(
            mission,
            out,
            seed_override=seed,
            generated_at=datetime.now(UTC),
        )
    except MissionLoadError as exc:
        typer.secho(f"error: {exc}", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=2) from exc

    worst = _worst_recommendation(results)
    for result in results:
        typer.echo(
            f"route {result.plan.route_index}: "
            f"{result.plan.total_length_m:.0f} m, "
            f"{result.report.recommendation.value} "
            f"-> {result.trajectory_path}"
        )
    typer.secho(
        f"overall recommendation: {worst}",
        fg=_recommendation_color(worst),
        bold=True,
    )


@app.command("demo-ao")
def demo_ao(
    out: Path = typer.Option(
        Path("demo_ao"), "--out", "-o", help="Directory for the layers + mission YAML."
    ),
    size: int = typer.Option(160, "--size", help="Demo *scene* size (cells per side)."),
    threat_margin: float | None = typer.Option(
        None,
        "--threat-margin",
        help="Metres of grid to keep behind the start for a threat sensor (default 2500).",
    ),
) -> None:
    """Export the built-in demo AO as GeoTIFF layers + a mission YAML.

    Gives `plan` real terrain to work with, so it produces a cost-aware route
    with turns and a varying altitude profile instead of the no-terrain
    straight-line stub — and exercises the same ingestion path real DTM/DSM data
    will use.

    `--threat-margin` pads the raster south and west of the start so an observer or
    radar can be placed at realistic standoff *behind* the launch point. Without
    it the start sits ~400 m from its own corner, the only sensor placement
    available sees the CP directly, and every "receding threat" measurement is a
    worst case (roadmap C0).
    """
    from droneplanner.demo import (
        THREAT_MARGIN_M,
        demo_layout,
        export_demo_ao,
        point_behind_start,
    )

    configure_logging()
    margin = THREAT_MARGIN_M if threat_margin is None else threat_margin
    mission_path = export_demo_ao(out, size=size, threat_margin_m=margin)
    typer.secho(f"wrote demo AO layers + mission -> {mission_path}", fg=typer.colors.GREEN)

    layout = demo_layout(size=size, threat_margin_m=margin)
    start, end = layout.endpoints()
    typer.echo(
        f"grid {layout.rows}x{layout.cols} cells "
        f"({layout.size} scene + {layout.margin_cells} threat margin); "
        f"standoff behind start {layout.threat_standoff_m():,.0f} m"
    )
    if margin > 0.0:
        sensor = point_behind_start(start, end, margin)
        typer.echo(
            f"a sensor {margin:,.0f} m behind the start sits at {sensor.x:.1f} {sensor.y:.1f}"
        )
    typer.echo(f"next: droneplanner plan --mission {mission_path} --out out/")


def _build_forest(
    dtm_raster: object,
    forest_dsm: Path,
    plan: Path,
    out: Path,
    *,
    home: tuple[float, float, float],
    band_m: float,
    stems_per_ha: float,
    max_stems: int,
    trunk_radius_m: float,
    radius_m: float | None = None,
) -> str:
    """Generate trunks along the plan's route and write the perception manifest.

    The route comes from the ``.plan`` itself, so the forest is placed around the
    path that will actually be flown — clutter somewhere else tests nothing.
    """
    import json

    from pyproj import Transformer

    from droneplanner.fields.altitude import canopy_height
    from droneplanner.io.forest_models import (
        generate_trunks,
        nearest_trunk_spacing_m,
        trunk_manifest_csv,
        trunks_to_sdf,
    )
    from droneplanner.io.raster import read_raster
    from droneplanner.model.geometry import Coordinate
    from droneplanner.model.grid import Grid

    dsm_raster = read_raster(forest_dsm)
    if dsm_raster.shape != dtm_raster.shape:  # type: ignore[attr-defined]
        raise typer.BadParameter("--forest-dsm must share the DTM's grid")

    grid = Grid(
        transform=dtm_raster.transform,  # type: ignore[attr-defined]
        rows=dtm_raster.shape[0],  # type: ignore[attr-defined]
        cols=dtm_raster.shape[1],  # type: ignore[attr-defined]
        crs=dsm_raster.crs,
    )
    chm = canopy_height(dtm_raster.data, dsm_raster.data)  # type: ignore[attr-defined]

    items = json.loads(plan.read_text())["mission"]["items"]
    to_crs = Transformer.from_crs("EPSG:4326", grid.crs, always_xy=True)
    route = []
    for item in items:
        # Waypoints only. A DO_CHANGE_SPEED item has zeros in params[4:6], which
        # read as lat/lon would add a route point at (0, 0) and draw the forest
        # band along a line to the equator.
        if item.get("command") != 16:
            continue
        params = item.get("params") or []
        if len(params) >= 6 and params[4] is not None and params[5] is not None:
            x, y = to_crs.transform(float(params[5]), float(params[4]))
            route.append(Coordinate(x=x, y=y))

    home_x, home_y, home_elev = home
    trunks, stats = generate_trunks(
        chm,
        dtm_raster.data,  # type: ignore[attr-defined]
        grid,
        route,
        home=Coordinate(x=home_x, y=home_y),
        home_elev_m=home_elev,
        band_m=band_m,
        stems_per_ha=stems_per_ha,
        trunk_radius_m=trunk_radius_m,
        max_stems=max_stems,
        within_m=radius_m,
    )

    manifest = out.parent / f"{out.stem}_trunks.csv"
    manifest.write_text(trunk_manifest_csv(trunks), encoding="utf-8")
    spacing = nearest_trunk_spacing_m(trunks)
    typer.secho(
        f"forest: {stats.trunks} trunks over {stats.band_area_ha:.1f} ha "
        f"({stats.effective_stems_per_ha:.0f} stems/ha, median nearest-neighbour "
        f"{spacing:.1f} m, band reaches {stats.max_route_distance_m:.0f} m) -> {manifest}\n"
        f"        {stats.on_route_trunks} trunk(s) within a rotor span of the planned "
        f"centreline — a run WITHOUT avoidance should hit them",
        fg=typer.colors.GREEN,
    )
    if stats.subsampled:
        typer.secho(
            f"  NOTE: capped at --max-stems {max_stems}, so the density is "
            f"{stats.effective_stems_per_ha:.0f}/ha, not the {stats.requested_stems_per_ha:.0f}/ha "
            "requested. The band was NOT narrowed — a hole in the forest would let the "
            "aircraft escape it, which would test the wrong thing.",
            fg=typer.colors.YELLOW,
        )
    return trunks_to_sdf(trunks)


@app.command("gz-world")
def gz_world(
    dtm: Path = typer.Option(
        ..., "--dtm", exists=True, dir_okay=False, help="DTM GeoTIFF the route was planned over."
    ),
    plan: Path = typer.Option(
        ..., "--plan", exists=True, dir_okay=False, help="The .plan whose home anchors the world."
    ),
    base_world: Path = typer.Option(
        ...,
        "--base-world",
        exists=True,
        dir_okay=False,
        help="PX4's Tools/simulation/gz/worlds/default.sdf — the terrain is injected into it.",
    ),
    out: Path = typer.Option(Path("terrain.sdf"), "--out", "-o", help="Output SDF world path."),
    dem_path: Path | None = typer.Option(
        None,
        "--dem-path",
        help=(
            "Path the DEM will have on the machine running Gazebo (e.g. the VM). "
            "Defaults to the local --dtm path, which is wrong if you run the sim elsewhere."
        ),
    ),
    no_collision: bool = typer.Option(
        False,
        "--no-collision",
        help=(
            "DIAGNOSTIC: visual-only terrain (no collision geometry). If the vehicle is "
            "stable here but not with collision on, the heightmap's collision is the culprit."
        ),
    ),
    flat: bool = typer.Option(
        False,
        "--flat",
        help=(
            "DIAGNOSTIC: emit a structurally identical world with an effectively FLAT DEM. "
            "Bisects 'heightmap present' from 'terrain has slopes' when the vehicle is unstable."
        ),
    ),
    forest_dsm: Path | None = typer.Option(
        None,
        "--forest-dsm",
        exists=True,
        dir_okay=False,
        help=(
            "TEST-PLAN B2: also populate the world with tree trunks derived from this DSM "
            "(canopy = DSM - DTM), near the plan's route. Writes a <out>_trunks.csv manifest "
            "for droneflight's perception oracle. Omit for the plain terrain world."
        ),
    ),
    stems_per_ha: float = typer.Option(400.0, "--stems-per-ha", help="Target stem density."),
    forest_band_m: float = typer.Option(
        25.0, "--forest-band", help="Half-width of the treed band around the route (m)."
    ),
    max_stems: int = typer.Option(
        4000, "--max-stems", help="Cap on trunk count; density is reduced, never the band."
    ),
    trunk_radius_m: float = typer.Option(0.15, "--trunk-radius", help="Trunk radius (m)."),
    forest_radius_m: float | None = typer.Option(
        None,
        "--forest-radius",
        help=(
            "Only place trunks within this distance of home. Set it to the mission's "
            "covered_radius_m: beyond that the route flies OVER the canopy, and "
            "droneflight's perception is 2D (it would dodge trunks far beneath it)."
        ),
    ),
) -> None:
    """Generate a Gazebo world whose heightmap is the planned terrain.

    PX4's default world is flat, so a terrain-aware plan flown in SITL proves
    nothing about nap-of-the-earth flight (and hides altitude-frame bugs). Copy
    the result into `PX4-Autopilot/Tools/simulation/gz/worlds/` and select it with
    `PX4_GZ_WORLD=<name>`.
    """
    import json

    from pyproj import Transformer

    from droneplanner.io.gzworld import build_world_sdf
    from droneplanner.io.raster import read_raster

    configure_logging()
    raster = read_raster(dtm)
    home = json.loads(plan.read_text())["mission"]["plannedHomePosition"]
    if flat:
        # Same extent/pose/world, but no relief: a 1 m ramp over 4 km (~0.02%).
        # If the vehicle is stable here but not over real terrain, the problem is
        # slope/normals; if it is unstable here too, it is the heightmap itself.
        import numpy as np

        from droneplanner.io.raster import write_raster

        base = float(home[2])
        rows, cols = raster.shape
        ramp = np.linspace(base, base + 1.0, rows * cols, dtype="float32").reshape(rows, cols)
        flat_path = out.parent / "dtm_flat.tif"
        out.parent.mkdir(parents=True, exist_ok=True)
        write_raster(flat_path, replace(raster, data=ramp))
        raster = read_raster(flat_path)
        typer.secho(f"--flat: wrote diagnostic DEM {flat_path}", fg=typer.colors.YELLOW)
    home_lat, home_lon, home_elev = float(home[0]), float(home[1]), float(home[2])
    to_crs = Transformer.from_crs("EPSG:4326", raster.crs, always_xy=True)
    home_x, home_y = to_crs.transform(home_lon, home_lat)

    out.parent.mkdir(parents=True, exist_ok=True)

    # Gazebo loads a ZERO-BASED DEM, not the raw one: that makes the vertical
    # datum identical whether Gazebo rescales elevations into <size>'s z or uses
    # them raw. Getting this wrong spawned the vehicle 40-125 m underground and
    # is what made terrain worlds tumble (io/gzworld.py module docstring).
    from droneplanner.io.gzworld import normalise_dem
    from droneplanner.io.raster import write_raster

    norm_path = out.parent / f"{out.stem}_dem.tif"
    write_raster(norm_path, normalise_dem(raster))
    typer.secho(
        f"wrote zero-based DEM {norm_path} (Gazebo loads this, not the raw DTM)",
        fg=typer.colors.YELLOW,
    )
    # --dem-path is where the DEM will live on the sim machine; keep its dir but
    # point at the normalised file we just wrote.
    gazebo_dem = (dem_path.parent / norm_path.name) if dem_path is not None else norm_path.resolve()

    trunks_sdf = ""
    if forest_dsm is not None:
        trunks_sdf = _build_forest(
            raster,
            forest_dsm,
            plan,
            out,
            home=(home_x, home_y, home_elev),
            band_m=forest_band_m,
            stems_per_ha=stems_per_ha,
            max_stems=max_stems,
            trunk_radius_m=trunk_radius_m,
            radius_m=forest_radius_m,
        )

    out.write_text(
        build_world_sdf(
            raster,
            gazebo_dem,
            base_world.read_text(encoding="utf-8"),
            home_lat=home_lat,
            home_lon=home_lon,
            home_x=home_x,
            home_y=home_y,
            home_elev_m=home_elev,
            world_name=out.stem,
            collision=not no_collision,
            extra_models_sdf=trunks_sdf,
        ),
        encoding="utf-8",
    )
    typer.secho(
        f"wrote {out}  (world '{out.stem}', home {home_lat:.5f},{home_lon:.5f})",
        fg=typer.colors.GREEN,
    )
    typer.echo(
        f"next: cp {out} $PX4_DIR/Tools/simulation/gz/worlds/ && "
        f"PX4_GZ_WORLD={out.stem} ./sitl/run_sitl.sh <plan> $PX4_DIR"
    )


@app.command()
def viz(
    out: Path = typer.Option(Path("demo_route.png"), "--out", "-o", help="Output PNG path."),
    size: int = typer.Option(160, "--size", help="Demo grid size (cells per side)."),
    forest: bool = typer.Option(
        True, "--forest/--no-forest", help="Include the M2 forest flyability layer."
    ),
) -> None:
    """Render the built-in demo AO: fields + concealment-aware route (PNG)."""
    from droneplanner.demo import build_demo, demo_flyability
    from droneplanner.planner.corridor import plan_over_terrain
    from droneplanner.viz import save_figure

    configure_logging()
    mission, dtm, dsm, grid = build_demo(size=size)
    flyability = demo_flyability(grid, mission.drone) if forest else None
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid, flyability=flyability)
    save_figure(out, mission, surface, plan)
    typer.secho(
        f"wrote {out}  (mean exposure {plan.objective_scores['mean_exposure']:.2f}, "
        f"{plan.total_length_m:.0f} m)",
        fg=typer.colors.GREEN,
    )


@app.command()
def serve(
    port: int = typer.Option(8765, "--port", "-p", help="Port to listen on."),
    host: str = typer.Option("127.0.0.1", "--host", help="Host/interface to bind."),
    size: int = typer.Option(130, "--size", help="Demo grid size (cells per side)."),
) -> None:
    """Launch the interactive route explorer (open http://HOST:PORT in a browser)."""
    from droneplanner.serve import serve as run_server

    configure_logging()
    typer.secho(f"explorer running at http://{host}:{port}  (Ctrl-C to stop)", fg=typer.colors.CYAN)
    run_server(host=host, port=port, size=size)


@app.command()
def version() -> None:
    """Print the droneplanner version."""
    typer.echo(__version__)


def _worst_recommendation(results: list) -> str:  # type: ignore[type-arg]
    order = {"NO-GO": 0, "REVIEW": 1, "GO": 2}
    values = [r.report.recommendation.value for r in results]
    return min(values, key=lambda v: order[v]) if values else "NO-GO"


def _recommendation_color(recommendation: str) -> str:
    return {
        "GO": typer.colors.GREEN,
        "REVIEW": typer.colors.YELLOW,
        "NO-GO": typer.colors.RED,
    }.get(recommendation, typer.colors.WHITE)


if __name__ == "__main__":  # pragma: no cover
    app()
