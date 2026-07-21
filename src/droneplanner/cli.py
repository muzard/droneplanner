"""``droneplanner`` command-line interface (Software-Spec §3, §7).

Usage:
    droneplanner plan --mission configs/mission.example.yaml --out out/
    droneplanner version
"""

from __future__ import annotations

import logging
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
    size: int = typer.Option(160, "--size", help="Demo grid size (cells per side)."),
) -> None:
    """Export the built-in demo AO as GeoTIFF layers + a mission YAML.

    Gives `plan` real terrain to work with, so it produces a cost-aware route
    with turns and a varying altitude profile instead of the no-terrain
    straight-line stub — and exercises the same ingestion path real DTM/DSM data
    will use.
    """
    from droneplanner.demo import export_demo_ao

    configure_logging()
    mission_path = export_demo_ao(out, size=size)
    typer.secho(f"wrote demo AO layers + mission -> {mission_path}", fg=typer.colors.GREEN)
    typer.echo(f"next: droneplanner plan --mission {mission_path} --out out/")


@app.command("gz-world")
def gz_world(
    dtm: Path = typer.Option(
        ..., "--dtm", exists=True, dir_okay=False, help="DTM GeoTIFF the route was planned over."
    ),
    plan: Path = typer.Option(
        ..., "--plan", exists=True, dir_okay=False, help="The .plan whose home anchors the world."
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
    home_lat, home_lon, home_elev = float(home[0]), float(home[1]), float(home[2])
    to_crs = Transformer.from_crs("EPSG:4326", raster.crs, always_xy=True)
    home_x, home_y = to_crs.transform(home_lon, home_lat)

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        build_world_sdf(
            raster,
            dem_path or dtm.resolve(),
            home_lat=home_lat,
            home_lon=home_lon,
            home_x=home_x,
            home_y=home_y,
            home_elev_m=home_elev,
            world_name=out.stem,
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
