"""Build a planner AO from Finnish open data: terrain, canopy, and measured stands.

    python tools/build_finnish_ao.py --bounds 412968 6707620 417464 6712116 \
        --chm data/metsakeskus/latvusmalli/*.tif --out data/askola_ao

Sources (all open, fetched 2026-09-25; the user approved the downloads):

* **Terrain** — National Land Survey of Finland (MML) 10 m elevation model, read
  for the AO window straight from the CSC/Funet mirror's VRT
  (``www.nic.funet.fi/index/geodata/mml/dem10m/dem10m_direct.vrt``), CC BY 4.0.
* **Canopy** — Finnish Forest Centre (Metsäkeskus) *Latvusmalli*, 1 m canopy height
  model per map sheet (``avoin.metsakeskus.fi/aineistot/Latvusmalli``), CC BY 4.0.
  Each output cell takes the **95th percentile** of the 1 m heights in it — the
  canopy top an observer sees, not the mean of crowns and gaps.
* **Stands** — Metsäkeskus *Metsävarakuviot* (``io/stand_data.py``), passed to the
  mission as ``layers.stand_data`` so speed is priced on measured stems/ha.

Outputs ``dtm.tif``, ``chm.tif``, ``dsm.tif`` (= dtm + chm), ``gaps.tif`` (1 where
a cell holds a canopy opening a climb can use — ``layers.canopy_gaps``) and
``roads.gpkg`` (MML vehicle roads for the window — ``layers.roads``) in EPSG:3067 on a
16 m grid — the Metsäkeskus *Hila* cell size, and close to the demo AO's 25 m.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.merge import merge
from rasterio.transform import from_origin
from rasterio.warp import reproject

OPENING_M = 3
"""Side of the open column (1 m pixels) a climb out of the canopy needs."""

GAP_FRACTION_BELOW_M = 8.0
"""Canopy lower than this counts as a gap for the look-down probability — the top of
the trunk-space flight band."""

OPENING_HEIGHT_M = 5.0
"""A pixel is open below this canopy height — under the trunk-space flight band."""

ROADS_GPKG = (
    "/vsicurl/https://www.nic.funet.fi/index/geodata/mml/maastotietokanta/2025/gpkg/"
    "MTK-tie_25-04-03.gpkg"
)
"""MML topographic database roads (CC BY 4.0), read for the AO window only — the file
is 5 GB nationally; its spatial index makes the window a few hundred kB."""

VEHICLE_ROAD_MAX_CLASS = 12300
"""MTK ``kohdeluokka`` below this is a road a vehicle drives on (motorway ... private
road); 123xx are tracks and footpaths, left out of crossing detection."""

DEM_VRT = "/vsicurl/https://www.nic.funet.fi/index/geodata/mml/dem10m/dem10m_direct.vrt"
CRS = "EPSG:3067"


def build(
    bounds: tuple[float, float, float, float], chm_files: list[Path], out: Path, cell: float
) -> None:
    x0, y0, x1, y1 = bounds
    cols, rows = round((x1 - x0) / cell), round((y1 - y0) / cell)
    transform = from_origin(x0, y1, cell, cell)
    profile = {
        "driver": "GTiff",
        "width": cols,
        "height": rows,
        "count": 1,
        "dtype": "float32",
        "crs": CRS,
        "transform": transform,
        "nodata": -9999.0,
        "compress": "deflate",
    }
    out.mkdir(parents=True, exist_ok=True)

    # Terrain: bilinear onto the grid. EPSG:25835 (the MML file) and EPSG:3067
    # are the same projection, so this is a resample, not a reprojection.
    dtm = np.full((rows, cols), np.nan, dtype=np.float32)
    with rasterio.Env(GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR"), rasterio.open(DEM_VRT) as dem:
        reproject(
            rasterio.band(dem, 1),
            dtm,
            dst_transform=transform,
            dst_crs=CRS,
            resampling=Resampling.bilinear,
            dst_nodata=np.nan,
        )

    # Canopy: 95th percentile of the 1 m model per cell.
    sources = [rasterio.open(f) for f in chm_files]
    try:
        mosaic, _ = merge(sources, bounds=(x0, y0, x1, y1), res=1.0, nodata=-9999.0)
    finally:
        for s in sources:
            s.close()
    fine = mosaic[0].astype(np.float32)
    fine[fine < 0] = np.nan
    k = round(cell)
    fine = fine[: rows * k, : cols * k].reshape(rows, k, cols, k).transpose(0, 2, 1, 3)
    fine = fine.reshape(rows, cols, k * k)
    with np.errstate(all="ignore"):
        chm = np.nanpercentile(fine, 95, axis=2).astype(np.float32)
    chm = np.where(np.isfinite(chm), np.clip(chm, 0.0, None), 0.0).astype(np.float32)

    # Canopy OPENINGS: a cell counts if it holds at least one OPENING_M x OPENING_M
    # column of 1 m pixels all below OPENING_HEIGHT_M — room for the 0.9 m airframe
    # to climb out of (or back down into) the trunk layer. Thinned talousmetsä has
    # them every ~20 m (harvester strip roads, 4-5 m wide); a closed stand does not.
    from scipy.ndimage import maximum_filter

    fine_full = mosaic[0].astype(np.float32)
    fine_full[~np.isfinite(fine_full) | (fine_full < 0)] = 0.0
    tallest = maximum_filter(fine_full, size=OPENING_M)
    open_px = (tallest < OPENING_HEIGHT_M)[: rows * k, : cols * k]
    gaps = open_px.reshape(rows, k, cols, k).any(axis=(1, 3)).astype(np.float32)
    # Crown-gap FRACTION: share of 1 m pixels open below the flight band — the per-cell
    # probability a look-down sightline reaches the aircraft (layers.canopy_gap_fraction).
    below = (fine_full < GAP_FRACTION_BELOW_M)[: rows * k, : cols * k]
    gapfrac = below.reshape(rows, k, cols, k).mean(axis=(1, 3)).astype(np.float32)

    if not np.isfinite(dtm).all():
        raise SystemExit("terrain has holes over the AO — choose bounds inside Finland's DEM")
    dsm = (dtm + chm).astype(np.float32)
    for name, arr in (
        ("dtm", dtm),
        ("chm", chm),
        ("dsm", dsm),
        ("gaps", gaps),
        ("gapfrac", gapfrac),
    ):
        with rasterio.open(out / f"{name}.tif", "w", **profile) as dst:
            dst.write(arr, 1)
    import pyogrio

    roads = pyogrio.read_dataframe(ROADS_GPKG, layer="tieviiva", bbox=(x0, y0, x1, y1))
    vehicle = roads[roads["kohdeluokka"] < VEHICLE_ROAD_MAX_CLASS]
    vehicle.to_file(out / "roads.gpkg", driver="GPKG")
    print(
        f"{out}: {len(vehicle)} vehicle road segments ({vehicle.length.sum() / 1000:.1f} km) "
        "-> roads.gpkg (layers.roads)"
    )
    print(
        f"{out}: {cols}x{rows} cells of {cell:.0f} m | terrain {dtm.min():.0f}-{dtm.max():.0f} m | "
        f"canopy >= 12 m on {100 * (chm >= 12).mean():.0f} % of cells, max {chm.max():.0f} m | "
        f"openings in {100 * gaps[chm >= 12].mean() if (chm >= 12).any() else 0:.0f} % "
        "of those"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--bounds", type=float, nargs=4, required=True, metavar=("X0", "Y0", "X1", "Y1")
    )
    ap.add_argument("--chm", type=Path, nargs="+", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cell", type=float, default=16.0)
    args = ap.parse_args()
    build(tuple(args.bounds), args.chm, args.out, args.cell)  # type: ignore[arg-type]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
