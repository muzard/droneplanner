# data/

Sample areas of operation (AOs) for tests and demos. **Large geospatial files
are gitignored** (see `.gitignore`) — keep only small samples or a fetch script
here.

Planned contents (M1):

- `example/` — a small public lidar tile (LAZ) + a Copernicus/SRTM DEM patch for
  the real-AO integration test (Software-Spec §6.1). Fetched by a script rather
  than committed, to keep the repo light.

The M0 scaffold needs no data on disk: its tests generate synthetic terrain and
point clouds in-memory (`tests/fixtures/synthetic.py`).
