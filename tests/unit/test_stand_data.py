"""Measured stand density from a Metsävarakuviot GeoPackage (io/stand_data.py)."""

from __future__ import annotations

import sqlite3
from pathlib import Path

import numpy as np
import pytest

from droneplanner.fields.stem_density import build_stem_clutter
from droneplanner.io.stand_data import (
    StandDataError,
    read_stand_stemcounts,
    stand_density_raster,
)
from droneplanner.model.grid import Grid

CRS = "EPSG:3067"
X0, Y0 = 380_000.0, 6_700_000.0


def _grid() -> Grid:
    # 10 x 10 cells of 10 m, north-up, top-left at (X0, Y0 + 100).
    return Grid(transform=(10.0, 0.0, X0, 0.0, -10.0, Y0 + 100.0), rows=10, cols=10, crs=CRS)


def _gpkg(tmp_path: Path) -> Path:
    """Two stands: west half 900/ha, east half 450/ha — with a forecast row that
    must be ignored and an inventoried (type 1) row that must be ignored too."""
    import geopandas as gpd
    from shapely.geometry import box

    path = tmp_path / "mvk.gpkg"
    stands = gpd.GeoDataFrame(
        {"standid": [11, 12]},
        geometry=[box(X0, Y0, X0 + 50, Y0 + 100), box(X0 + 50, Y0, X0 + 100, Y0 + 100)],
        crs=CRS,
    )
    stands.to_file(path, layer="stand", driver="GPKG")
    with sqlite3.connect(path) as db:
        db.execute(
            "CREATE TABLE treestand "
            "(id INTEGER, standid INTEGER, treestandid INTEGER, type INTEGER)"
        )
        db.execute(
            "CREATE TABLE treestandsummary (id INTEGER, treestandid INTEGER, stemcount REAL)"
        )
        for tid, sid, typ, count in [
            (1, 11, 1, 1500.0),  # inventoried years ago — superseded
            (2, 11, 2, 900.0),  # updated to the present — the one to read
            (3, 11, 3, 700.0),  # forecast +10 y
            (4, 12, 2, 450.0),
        ]:
            db.execute("INSERT INTO treestand VALUES (?, ?, ?, ?)", (tid, sid, tid, typ))
            db.execute("INSERT INTO treestandsummary VALUES (?, ?, ?)", (tid, tid, count))
    return path


def test_it_reads_the_present_day_summary_only(tmp_path: Path) -> None:
    assert read_stand_stemcounts(_gpkg(tmp_path)) == {11: 900.0, 12: 450.0}


def test_stands_are_burned_into_the_grid(tmp_path: Path) -> None:
    raster = stand_density_raster(_gpkg(tmp_path), _grid())
    assert raster.shape == (10, 10)
    assert np.all(raster[:, :5] == 900.0)
    assert np.all(raster[:, 5:] == 450.0)


def test_a_file_without_the_schema_fails_loudly(tmp_path: Path) -> None:
    bad = tmp_path / "bad.gpkg"
    sqlite3.connect(bad).close()
    with pytest.raises(StandDataError, match="treestandsummary"):
        read_stand_stemcounts(bad)


def test_measured_density_replaces_the_inference_on_treed_cells_only() -> None:
    chm = np.full((2, 3), 20.0, dtype=np.float32)
    chm[1, 2] = 0.0  # open ground — felled since the inventory, say
    measured = np.array([[900.0, np.nan, 450.0], [900.0, 450.0, 450.0]], dtype=np.float32)
    out = build_stem_clutter(chm, cruise_speed_ms=6.0, measured_per_ha=measured)
    density = out["stem_density_per_ha"]
    inferred = build_stem_clutter(chm, cruise_speed_ms=6.0)["stem_density_per_ha"]
    assert density[0, 0] == 900.0 and density[1, 1] == 450.0
    assert density[0, 1] == inferred[0, 1]  # no stand here -> the inference
    assert density[1, 2] == 0.0  # the canopy model says open, and it is newer
    assert out["stem_density_measured"].tolist() == [[1.0, 0.0, 1.0], [1.0, 1.0, 0.0]]


def test_a_stated_density_still_wins_over_stand_data() -> None:
    chm = np.full((1, 2), 20.0, dtype=np.float32)
    measured = np.array([[900.0, 900.0]], dtype=np.float32)
    out = build_stem_clutter(
        chm, cruise_speed_ms=6.0, measured_per_ha=measured, density_override_per_ha=200.0
    )
    assert out["stem_density_per_ha"].tolist() == [[200.0, 200.0]]
    assert not out["stem_density_measured"].any()


def _gpkg_current_model(tmp_path: Path) -> Path:
    """The data model in the 2026 downloads: treestandclass + storeys."""
    import geopandas as gpd
    from shapely.geometry import box

    path = tmp_path / "mvk2.gpkg"
    gpd.GeoDataFrame(
        {"standid": [21]}, geometry=[box(X0, Y0, X0 + 100, Y0 + 100)], crs=CRS
    ).to_file(path, layer="stand", driver="GPKG")
    with sqlite3.connect(path) as db:
        db.execute(
            "CREATE TABLE treestand (id INTEGER, treestandid INTEGER, standid INTEGER, "
            "treestandclass TEXT)"
        )
        db.execute(
            "CREATE TABLE treestandsummary (id INTEGER, treestandid INTEGER, "
            "stemcount INTEGER, storey TEXT)"
        )
        db.execute("INSERT INTO treestand VALUES (1, 1, 21, '1')")  # inventory: ignored
        db.execute("INSERT INTO treestand VALUES (2, 2, 21, '2')")  # present day
        for sid, tid, count, storey in [
            (1, 1, 2000, "1"),
            (2, 2, 600, "1"),  # dominant
            (3, 2, 1500, "2"),  # understory — below the flight band, left out
            (4, 2, 40, "3"),  # overstory — trunks, counted
        ]:
            db.execute(
                "INSERT INTO treestandsummary VALUES (?, ?, ?, ?)", (sid, tid, count, storey)
            )
    return path


def test_the_current_data_model_is_read_and_the_understorey_left_out(tmp_path: Path) -> None:
    assert read_stand_stemcounts(_gpkg_current_model(tmp_path)) == {21: 640.0}


def test_an_unknown_data_model_is_refused_not_guessed(tmp_path: Path) -> None:
    bad = tmp_path / "bad2.gpkg"
    with sqlite3.connect(bad) as db:
        db.execute("CREATE TABLE treestand (id INTEGER, standid INTEGER, treestandid INTEGER)")
        db.execute("CREATE TABLE treestandsummary (treestandid INTEGER, stemcount INTEGER)")
    with pytest.raises(StandDataError, match="unknown"):
        read_stand_stemcounts(bad)
