# Third-party dependency license inventory

The shippable planner links **only permissive (BSD/MIT/Apache) libraries**. GPL
tools are used, if at all, across a process/CLI boundary or for R&D/validation
only, never linked into shippable code (Software-Spec §2, §5.1).

## Runtime dependencies (linked — must stay permissive)

| Package | Purpose | License |
|---|---|---|
| `pydantic` | Data contracts / validation | MIT |
| `pyyaml` | Mission YAML parsing | MIT |
| `typer` (+ `click`) | CLI | MIT / BSD-3 |
| `numpy` | Arrays / numerics | BSD-3 |
| `scipy` | `ndimage` EDT/morphology (M1/M2) | BSD-3 |
| `shapely` | Vector geometry | BSD-3 |
| `pyproj` | CRS transforms (wraps PROJ) | MIT (PROJ: MIT/X) |
| `rasterio` | GeoTIFF I/O (wraps GDAL) | BSD-3 (GDAL: MIT/X-style) |
| `geopandas` | Vector layer I/O | BSD-3 |
| `scikit-image` | `route_through_array`, image/graph ops (M1) | BSD-3 |
| `networkx` | Crossing graph (M1) | BSD-3 |
| `laspy` | LAS/LAZ point-cloud read | BSD-2 |

## Optional extras

| Extra | Package | Purpose | License / note |
|---|---|---|---|
| `forest` | `pdal` | Point-cloud pipeline (M2) | BSD-3. Needs the **system PDAL C++ library**; not reliably pip-installable — install via conda/OS packages. |
| `viz` | `matplotlib`, `folium` | Debug visualization | matplotlib: PSF-based (BSD-compatible); folium: MIT |
| `accel` | `numba` | Optional JIT (viewshed hotspot) | BSD-2 |

## Dev/CI-only

`ruff` (MIT), `mypy` (MIT), `pytest` (MIT), `pytest-cov` (MIT), `hypothesis`
(MPL-2.0 — dev/test only, not shipped), `build` (MIT), `pre-commit` (MIT),
`types-PyYAML` (Apache-2.0).

## Explicitly NOT linked (GPL — process/CLI boundary or R&D only)

| Tool | Use | Boundary |
|---|---|---|
| GRASS GIS `r.viewshed` | R&D viewshed cross-check | External process/CLI only. The shippable viewshed is an in-house NumPy/Numba line-of-sight path (Software-Spec §5.1). |
| GPL SLAM/LIO (ORB-SLAM3, VINS, OpenVINS) | Navigation R&D | Separate nav subsystem, not this module; not linked here. |

> When adding a dependency: confirm the license is permissive (or keep it behind
> a process/CLI boundary), and update this file in the same change.
