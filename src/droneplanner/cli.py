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


@app.command()
def viz(
    out: Path = typer.Option(Path("demo_route.png"), "--out", "-o", help="Output PNG path."),
    size: int = typer.Option(160, "--size", help="Demo grid size (cells per side)."),
) -> None:
    """Render the built-in demo AO: fields + concealment-aware route (PNG)."""
    from droneplanner.demo import build_demo
    from droneplanner.planner.corridor import plan_over_terrain
    from droneplanner.viz import save_figure

    configure_logging()
    mission, dtm, dsm, grid = build_demo(size=size)
    plan, surface = plan_over_terrain(mission, dtm, dsm, grid)
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
