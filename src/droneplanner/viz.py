"""Visualization of the planner's fields and route (Software-Spec §6.6).

Renders the *real* planner output — terrain, viewshed exposure, composite cost,
and survivability — with the concealment-aware route overlaid against the naive
straight line, so an operator can see at a glance why the planner chose its
detour. Uses the non-interactive Agg backend so it works headless (CLI, CI, and
the interactive server).
"""

from __future__ import annotations

import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from droneplanner.fields.composite import CostSurface
from droneplanner.model.mission import Mission
from droneplanner.model.plan import Plan


def hillshade(
    dtm: np.ndarray, cell_size_m: float, azimuth: float = 315.0, altitude: float = 45.0
) -> np.ndarray:
    """Simple analytical hillshade of a DTM, in [0, 1]."""
    dy, dx = np.gradient(dtm.astype(np.float64), cell_size_m)
    slope = np.pi / 2.0 - np.arctan(np.hypot(dx, dy))
    aspect = np.arctan2(-dx, dy)
    az = np.radians(360.0 - azimuth + 90.0)
    alt = np.radians(altitude)
    shaded = np.sin(alt) * np.sin(slope) + np.cos(alt) * np.cos(slope) * np.cos(az - aspect)
    return np.asarray(np.clip((shaded + 1.0) / 2.0, 0.0, 1.0), dtype=np.float64)


def _extent(surface: CostSurface) -> tuple[float, float, float, float]:
    a, _, c, _, e, f = surface.grid.transform
    left, right = c, c + a * surface.grid.cols
    top, bottom = f, f + e * surface.grid.rows
    return (left, right, bottom, top)


def _path_xy(plan: Plan) -> tuple[list[float], list[float]]:
    return [v.x for v in plan.vertices], [v.y for v in plan.vertices]


def _overlay(ax: Axes, mission: Mission, plan: Plan) -> None:
    px, py = _path_xy(plan)
    ax.plot(
        [mission.start.x, mission.end.x],
        [mission.start.y, mission.end.y],
        "--",
        color="white",
        lw=1.2,
        alpha=0.7,
        label="naive straight line",
    )
    ax.plot(px, py, "-", color="#00e5ff", lw=2.0, label="planned route")
    for obs in mission.threat.observers:
        ax.plot(obs.x, obs.y, "^", color="red", ms=9, mec="black", label="_observer")
    ax.plot(mission.start.x, mission.start.y, "o", color="#00ff6a", ms=9, mec="black")
    ax.plot(mission.end.x, mission.end.y, "s", color="#ffd400", ms=9, mec="black")
    ax.set_xticks([])
    ax.set_yticks([])


def render(mission: Mission, surface: CostSurface, plan: Plan) -> Figure:
    """Build the 2x2 fields+route figure for a planned mission."""
    extent = _extent(surface)
    fig, axes = plt.subplots(2, 2, figsize=(12, 11))
    scores = plan.objective_scores
    fig.suptitle(
        f"{mission.name}: concealment-aware route  "
        f"(drone exposure {scores.get('mean_exposure', 0):.2f} vs cable "
        f"{scores.get('mean_cable_los', 0):.2f}, length {plan.total_length_m:.0f} m)",
        fontsize=13,
    )

    if "flyability_class" in surface.components:
        fourth = (
            axes[1, 1],
            surface.components["flyability_class"],
            "forest flyability (green=flyable, red=non-navigable)",
            "RdYlGn_r",
        )
    elif "canopy_height" in surface.components:
        # The cover the route is hiding under — the reason the drone's exposure
        # and the cable's differ at all.
        fourth = (axes[1, 1], surface.components["canopy_height"], "canopy height (m)", "YlGn")
    else:
        fourth = (
            axes[1, 1],
            surface.components["survivability"],
            "cable survivability cost",
            "magma",
        )

    panels = [
        (
            axes[0, 0],
            hillshade(surface.dtm, surface.grid.cell_size_m),
            "terrain (hillshade)",
            "gray",
        ),
        (
            axes[0, 1],
            surface.components["exposure"],
            "drone visual exposure (viewshed at flight altitude)",
            "inferno",
        ),
        (axes[1, 0], surface.components["cost_normalized"], "composite cost surface", "viridis"),
        fourth,
    ]
    for ax, data, title, cmap in panels:
        im = ax.imshow(data, extent=extent, origin="upper", cmap=cmap, aspect="equal")
        ax.set_title(title, fontsize=11)
        _overlay(ax, mission, plan)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    axes[0, 0].legend(loc="upper right", fontsize=8, framealpha=0.8)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def save_figure(path: str | Path, mission: Mission, surface: CostSurface, plan: Plan) -> None:
    """Render and save the figure to ``path`` (PNG)."""
    fig = render(mission, surface, plan)
    fig.savefig(Path(path), dpi=130)
    plt.close(fig)


LAYERS = (
    "exposure",
    "ground_los",
    "canopy",
    "flight_agl",
    "cost",
    "terrain",
    "survivability",
    "flyability",
)


def layer_data(surface: CostSurface, layer: str) -> tuple[np.ndarray, str]:
    """Return (array, colormap) for a named display layer."""
    if layer == "cost":
        return surface.components["cost_normalized"], "viridis"
    if layer == "terrain":
        return hillshade(surface.dtm, surface.grid.cell_size_m), "gray"
    if layer == "survivability":
        return surface.components["survivability"], "magma"
    # What the drone is hiding under, and how low it flies to do it.
    if layer == "canopy" and "canopy_height" in surface.components:
        return surface.components["canopy_height"], "YlGn"
    if layer == "flight_agl" and "flight_agl" in surface.components:
        return surface.components["flight_agl"], "cividis"
    # Ground-level exposure: what the *cable* is exposed to, and the contrast that
    # shows how much the drone's altitude costs it (SHORTFALLS item 1).
    if layer == "ground_los" and "ground_los" in surface.components:
        return surface.components["ground_los"], "inferno"
    if layer == "flyability" and "flyability_class" in surface.components:
        # 0 flyable (green) -> 1 marginal -> 2 non-navigable (red)
        return surface.components["flyability_class"], "RdYlGn_r"
    return surface.components["exposure"], "inferno"  # default


def render_interactive(
    mission: Mission, surface: CostSurface, plan: Plan, layer: str = "exposure", size_px: int = 640
) -> bytes:
    """Render a single borderless layer+route PNG that maps 1:1 to the extent.

    The axes fill the whole canvas with no margins/ticks/colorbar, so a click at
    image pixel (px, py) maps linearly to world coordinates via the extent — this
    is what the interactive server uses to place observers and endpoints.
    """
    extent = _extent(surface)
    data, cmap = layer_data(surface, layer)

    fig = Figure(figsize=(size_px / 100.0, size_px / 100.0), dpi=100)
    FigureCanvasAgg(fig)
    ax = fig.add_axes((0.0, 0.0, 1.0, 1.0))
    ax.axis("off")
    ax.imshow(data, extent=extent, origin="upper", cmap=cmap, aspect="auto")
    _overlay(ax, mission, plan)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])

    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    return buf.getvalue()
