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
