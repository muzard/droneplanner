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

SCHEMA — two versions, both read (checked on a real download, Askola, 2026-09-24)
``stand`` (polygons, ``standid``) → ``treestand`` (``standid``, ``treestandid``,
class) → ``treestandsummary`` (``treestandid``, ``stemcount``, ...). The class
column says which estimate a tree stand is; only **"present-day" (2)** is read —
the inventory grown forward to today, not the raw inventory (1) or the +10-year
forecast (3):

* 2021 guide (*Metsävarakuvio GeoPackage -ohje*): ``treestand.type``.
* Current data model (*Tietomallin uudistus*, in the 2026 downloads):
  ``treestand.treestandclass`` (text code), and summaries split by ``storey`` —
  1 dominant, 2 understory, 3 overstory, 4 retention trees, 5 remote sensing.
  Trunks at flight height are storeys 1, 3 and 4, summed; the understory is
  left out (it is mostly below the 5-10 m band — a stated simplification), and
  storey 5 is used only when a stand has nothing else.

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
import warnings
from pathlib import Path

import numpy as np

from droneplanner.model.grid import Grid

_TRUNK_STOREYS = {"1", "3", "4"}
_UNDERSTOREY = "2"
_REMOTE_SENSING = "5"


class StandDataError(RuntimeError):
    """The file is not a Metsävarakuviot GeoPackage this reader understands."""


def read_stand_stemcounts(path: Path) -> dict[int, float]:
    """``standid -> stems/ha`` at flight height, from the present-day summaries."""
    try:
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as db:
            stand_cols = {r[1] for r in db.execute("PRAGMA table_info(treestand)")}
            summary_cols = {r[1] for r in db.execute("PRAGMA table_info(treestandsummary)")}
            if not stand_cols or not summary_cols:
                raise StandDataError(
                    f"{path}: not a Metsävarakuviot GeoPackage (no treestand/"
                    "treestandsummary tables)"
                )
            if "treestandclass" in stand_cols:
                klass = "treestandclass"
            elif "type" in stand_cols:
                klass = "type"
            else:
                raise StandDataError(
                    f"{path}: treestand has neither 'treestandclass' nor 'type' — "
                    "an unknown Metsävarakuviot data model; not guessing"
                )
            storey = "s.storey" if "storey" in summary_cols else "'1'"
            rows = db.execute(
                f"SELECT t.standid, s.stemcount, CAST({storey} AS TEXT) "
                "FROM treestand t JOIN treestandsummary s ON t.treestandid = s.treestandid "
                f"WHERE CAST(t.{klass} AS TEXT) = '2' AND s.stemcount IS NOT NULL"
            ).fetchall()
    except sqlite3.Error as exc:
        raise StandDataError(
            f"{path}: not a Metsävarakuviot GeoPackage (stand/treestand/"
            f"treestandsummary tables): {exc}"
        ) from exc
    trunks: dict[int, float] = {}
    remote: dict[int, float] = {}
    for standid, count, level in rows:
        sid = int(standid)
        if level in _TRUNK_STOREYS:
            trunks[sid] = trunks.get(sid, 0.0) + float(count)
        elif level == _REMOTE_SENSING:
            remote[sid] = remote.get(sid, 0.0) + float(count)
    for sid, count in remote.items():
        trunks.setdefault(sid, count)
    return trunks


def stand_density_raster(path: Path, grid: Grid) -> np.ndarray:
    """Stems/ha per grid cell from the stand polygons; NaN where no stand covers it."""
    import geopandas as gpd
    from rasterio.features import rasterize
    from rasterio.transform import Affine

    counts = read_stand_stemcounts(path)
    with warnings.catch_warnings():
        # The real files store "Measured 3D Polygon"; pyogrio drops M and Z is
        # ignored by rasterize — say it once here, not on every read.
        warnings.filterwarnings("ignore", message="Measured \\(M\\) geometry types")
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
