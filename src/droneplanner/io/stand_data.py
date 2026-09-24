"""Measured stand density from Finnish Forest Centre stand data (Metsävarakuviot).

WHY
Every stems/ha the planner has used so far was either the canopy-height inference
in ``fields/stem_density.py`` (forestry literature, not a measurement) or an
operator-stated constant. For a Finnish AO there is a measured, free source: the
Finnish Forest Centre's (Suomen metsäkeskus) open forest-resource stands,
*Metsävarakuviot* — ~1.4 ha stand polygons from airborne-laser inventory, updated
twice a week, CC BY 4.0, GeoPackage in ETRS-TM35FIN (EPSG:3067). Each stand carries
a tree-stand summary with ``stemcount`` (stems/ha), ``meanheight`` and
``meandiameter``. Speed and endurance are priced on stems/ha, so this is the most
valuable number the programme can get, and it covers most of Finland's forest.

SCHEMA (Metsäkeskus, *Metsävarakuvio GeoPackage -ohje*, 30.4.2021)
``stand`` (polygons, ``standid``) → ``treestand`` (``standid``, ``treestandid``,
``type``: 1 inventoried, **2 updated to the present**, 3 forecast +10 years) →
``treestandsummary`` (``treestandid``, ``stemcount``, ...). Only ``type = 2`` is
read: it is the inventory grown forward to today. ⚠️ Metsäkeskus has announced a
data-model renewal (*Tietomallin uudistus*); if a newer download lacks these
tables this reader fails loudly rather than guessing.

HOW MUCH TO TRUST IT
The published accuracy target is on mean height ±2 m, mean diameter ±3 cm and
basal area ±3 m²/ha on 8 stands in 10 — not on stem count, which is derived. At
20 cm and 20 m²/ha those tolerances alone move stems/ha by roughly ±30 %. It
describes the stand as inventoried and grown forward by a model: a felling since
the last update, or storm damage, makes it wrong, and in a battlespace more so.
Like every survey here it is a **planning prior**, never something the aircraft
depends on in flight (user rule, 2026-09-15) — and a stale value errs toward
*denser* than reality, since stands are thinned and felled, not replanted, between
updates.
"""

from __future__ import annotations

import sqlite3
from pathlib import Path

import numpy as np

from droneplanner.model.grid import Grid

_QUERY = """
SELECT treestand.standid, treestandsummary.stemcount
FROM treestand
JOIN treestandsummary ON treestand.treestandid = treestandsummary.treestandid
WHERE treestand.type = 2 AND treestandsummary.stemcount IS NOT NULL
"""


class StandDataError(RuntimeError):
    """The file is not a Metsävarakuviot GeoPackage this reader understands."""


def read_stand_stemcounts(path: Path) -> dict[int, float]:
    """``standid -> stems/ha`` from the present-day tree-stand summaries."""
    try:
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as db:
            rows = db.execute(_QUERY).fetchall()
    except sqlite3.Error as exc:
        raise StandDataError(
            f"{path}: not a Metsävarakuviot GeoPackage (stand/treestand/"
            f"treestandsummary tables): {exc}"
        ) from exc
    return {int(standid): float(count) for standid, count in rows}


def stand_density_raster(path: Path, grid: Grid) -> np.ndarray:
    """Stems/ha per grid cell from the stand polygons; NaN where no stand covers it."""
    import geopandas as gpd
    from rasterio.features import rasterize
    from rasterio.transform import Affine

    counts = read_stand_stemcounts(path)
    stands = gpd.read_file(path, layer="stand")
    if "standid" not in stands.columns:
        raise StandDataError(f"{path}: 'stand' layer has no standid column")
    if stands.crs is None:
        raise StandDataError(f"{path}: 'stand' layer has no CRS")
    stands = stands.to_crs(grid.crs)
    shapes = [
        (geom, counts[int(sid)])
        for geom, sid in zip(stands.geometry, stands["standid"], strict=True)
        if geom is not None and int(sid) in counts
    ]
    out = np.full((grid.rows, grid.cols), np.nan, dtype=np.float32)
    if not shapes:
        return out
    burned: np.ndarray = rasterize(
        shapes,
        out_shape=(grid.rows, grid.cols),
        transform=Affine(*grid.transform),
        fill=np.nan,
        dtype="float32",
    )
    return burned.astype(np.float32)
