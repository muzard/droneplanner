"""Gazebo (gz-sim) world generator — fly SITL over the *planned* terrain.

PX4 SITL's default Gazebo world is **flat**. Flying a terrain-aware `.plan` over
flat ground is a degenerate test: it validates the autopilot's ability to fly a
polyline and nothing about nap-of-the-earth flight. Worse, it actively *hides*
altitude-frame bugs — the relative-to-home CFIT defect (see ``io/mavlink.py``)
survived a green SITL flight precisely because the world had no relief.

This emits an SDF world whose heightmap is the **same DTM the planner routed
over**, georeferenced so the plan's home sits at the world origin.

Gazebo loads a GeoTIFF DEM directly (it auto-detects DEM vs plain image), so the
DTM needs no conversion.

Frames, and how the pieces line up
----------------------------------
* PX4 spawns the vehicle at the Gazebo **world origin** and calls that
  ``PX4_HOME_LAT/LON/ALT``. So the world origin must be the plan's home.
* ``<spherical_coordinates>`` therefore carries the plan's home lat/lon, giving
  an ENU tangent plane at that point.
* A heightmap is centred on its model pose, but home is generally *not* the DTM's
  centre — so the model is offset by (dtm_centre - home) in projected meters,
  placing home over the world origin.

Vertical datum — the tumble bug, diagnosed and fixed (2026-07-23)
----------------------------------------------------------------
**Terrain worlds tumbled on translation because the vehicle spawned buried.**

The heightmap's vertical placement depends on whether Gazebo *rescales* a DEM's
elevations into ``<size>``'s z, or uses the DEM's **raw** elevation values. The
old code assumed rescaling. ODE said otherwise: it reported ``AABB max z =
150.32``, the demo DEM's max **elevation**, not the ``110.32`` **relief** passed
as ``<size>`` — so Gazebo was using raw elevations.

With raw elevations and the old offset ``-(home_elev - min_elev)``, the surface
at home landed at world ``z = min_elev`` instead of 0::

    real DEM: home 124.68, min 40.00  ->  surface at world z = +40.0 m
    flat DEM: home 124.68, min 124.68 ->  surface at world z = +124.7 m

PX4 spawns the vehicle at the world origin (z = 0), so it started **40 m (real)
or 125 m (flat) underneath the terrain**, held in permanent collision
penetration. That reads as a stable hover — and then flings the vehicle as soon
as it translates. It also explains why the ``--flat`` bisection tumbled too: not
"the heightmap itself", just the *same* datum bug, worse. That earlier bisection
conclusion was measuring this bug twice.

**The fix: hand Gazebo a zero-based DEM.** :func:`normalise_dem` subtracts the
minimum elevation so the written DEM spans ``0 .. relief``. Then the surface at
home sits at world z = 0 under *either* interpretation — raw values and
rescale-to-``size.z`` become numerically identical — so the datum no longer
depends on a Gazebo behaviour we cannot pin down from the Mac. ``size.z`` is
still the relief, and the model pose keeps ``-(home_elev - min_elev)``.

To *verify* on the VM, compare Gazebo's own model pose (``gz model -m terrain
-p``) against a height query — never infer it from PX4's local z, which is zeroed
at the spawn point and therefore tautological.

**We inject into PX4's own world rather than writing one.** See
:func:`build_world_sdf`; a from-scratch world silently loses the default gz-sim
plugin set and the ``<magnetic_field>``, which kills the sensors and then the
attitude estimate. Pass PX4's ``Tools/simulation/gz/worlds/default.sdf``.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path

from droneplanner.io.raster import RasterGrid

_COLLISION = """
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>file://{dem_uri}</uri>
          <size>{size_x:.2f} {size_y:.2f} {size_z:.2f}</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>"""

_TERRAIN_MODEL = """
<model name="terrain">
  <static>true</static>
  <!-- x,y: (DTM centre - home) so home lands on the world origin.
       z: vertical datum offset - if the vehicle spawns buried, increase;
          if it spawns hovering, decrease. -->
  <pose>{off_x:.3f} {off_y:.3f} {off_z:.3f} 0 0 0</pose>
  <link name="link">{collision}
    <visual name="visual">
      <geometry>
        <heightmap>
          <use_terrain_paging>false</use_terrain_paging>
          <uri>file://{dem_uri}</uri>
          <size>{size_x:.2f} {size_y:.2f} {size_z:.2f}</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
"""


# A heightmap needs a strictly positive vertical scale; a perfectly flat DEM has
# relief 0, and `<size ... 0>` is degenerate (Gazebo divides by it when scaling).
_MIN_RELIEF_M = 1.0


@dataclass(frozen=True)
class WorldGeometry:
    """The computed placement of a DTM heightmap relative to the plan's home."""

    size_x_m: float  # DTM width in meters
    size_y_m: float  # DTM height in meters
    relief_m: float  # elevation range (max - min), floored at _MIN_RELIEF_M
    min_elev_m: float
    offset_x_m: float  # model pose x: DTM centre - home (projected meters)
    offset_y_m: float
    offset_z_m: float  # vertical datum offset


def normalise_dem(dtm: RasterGrid) -> RasterGrid:
    """Zero-base a DTM's elevations so the DEM Gazebo loads spans ``0 .. relief``.

    This is what makes the vertical datum independent of whether Gazebo rescales
    a DEM into ``<size>``'s z or uses its raw elevation values: once the minimum
    is 0 and ``size.z`` is the relief, the two interpretations coincide. See the
    module docstring — assuming the wrong one is what buried the vehicle and made
    terrain worlds tumble.
    """
    import numpy as np

    data = dtm.data.astype(np.float32)
    if dtm.nodata is not None:
        valid = data != np.float32(dtm.nodata)
        if not valid.any():
            raise ValueError("DTM has no valid (non-nodata) cells")
        floor = float(data[valid].min())
        shifted = np.where(valid, data - np.float32(floor), data)
    else:
        floor = float(data.min())
        shifted = data - np.float32(floor)
    return replace(dtm, data=shifted.astype(np.float32))


def compute_geometry(
    dtm: RasterGrid, home_x: float, home_y: float, home_elev_m: float
) -> WorldGeometry:
    """Place the DTM heightmap so the plan's home sits at the world origin.

    ``home_x``/``home_y`` are in the DTM's projected CRS (meters). Returns the
    model pose offsets and heightmap extents used by the SDF.
    """
    a, _b, c, _d, e, f = dtm.transform
    rows, cols = dtm.shape
    size_x = abs(a) * cols
    size_y = abs(e) * rows
    # Raster corners: (c, f) is the top-left; e is negative (north-up).
    centre_x = c + (a * cols) / 2.0
    centre_y = f + (e * rows) / 2.0

    min_elev = float(dtm.data.min())
    relief = max(float(dtm.data.max()) - min_elev, _MIN_RELIEF_M)
    return WorldGeometry(
        size_x_m=size_x,
        size_y_m=size_y,
        relief_m=relief,
        min_elev_m=min_elev,
        offset_x_m=centre_x - home_x,
        offset_y_m=centre_y - home_y,
        # Ground at home -> world z = 0. Valid because the DEM handed to Gazebo is
        # zero-based (`normalise_dem`): home's surface sits at (home_elev -
        # min_elev) in the heightmap's own frame, and this cancels it exactly.
        offset_z_m=-(home_elev_m - min_elev),
    )


def surface_z_at_home(geom: WorldGeometry, home_elev_m: float) -> float:
    """World z of the terrain surface under home — must be 0.0.

    The regression guard for the tumble bug: any non-zero value means the vehicle
    spawns buried (positive) or hovering above the ground (negative).
    """
    return (home_elev_m - geom.min_elev_m) + geom.offset_z_m


def build_world_sdf(
    dtm: RasterGrid,
    dem_path: Path,
    base_world: str,
    *,
    home_lat: float,
    home_lon: float,
    home_x: float,
    home_y: float,
    home_elev_m: float,
    world_name: str,
    collision: bool = True,
) -> str:
    """Inject the terrain heightmap into PX4's own world SDF.

    ⚠️ **Contract:** ``dtm`` is the *original* DTM (its true elevations set the
    relief and the datum offset), while ``dem_path`` must point at the
    **zero-based** DEM produced by :func:`normalise_dem` — that is what Gazebo
    loads. Pointing it at the raw DEM re-introduces the tumble bug (module
    docstring); ``droneplanner gz-world`` writes the normalised file for you.

    ``base_world`` is the text of PX4's ``Tools/simulation/gz/worlds/default.sdf``.
    We start from it rather than writing a world from scratch because a
    hand-written world silently diverges from what PX4 needs, in ways that do not
    error — they just break flight:

    * gz-sim loads its **default plugin set only when the world declares no
      ``<plugin>`` elements**. Declaring any (as a from-scratch world naturally
      does) disables the rest, so IMU/barometer/magnetometer/NavSat go dead and
      PX4 reports "No valid data from Accel 0", "Found 0 compass".
    * PX4's world sets ``<magnetic_field>``, ``<atmosphere>``, ``<gravity>`` and
      ``<physics type="ode"><real_time_update_rate>250``. Omitting the magnetic
      field leaves the magnetometer with nothing to measure, so EKF yaw diverges:
      the vehicle hovers perfectly, then tumbles as soon as it translates.

    So the only edits made here are the ones we actually need:

    1. remove the flat ``ground_plane`` (our terrain is the ground; leaving it in
       puts a floor at z=0 that low-lying terrain would sit beneath),
    2. point ``<spherical_coordinates>`` at the plan's home,
    3. append the terrain heightmap model,
    4. rename the world to ``world_name``.

    (4) is not cosmetic: PX4 selects a world by **file name** via ``PX4_GZ_WORLD``
    and then waits for a Gazebo world of that same name. Leaving the base file's
    ``name="default"`` in a file called ``planner_terrain.sdf`` makes PX4 hang on
    *"Waiting for Gazebo world..."* forever — the world loads fine, PX4 just never
    recognises it. Pass the output file's stem.
    """
    geom = compute_geometry(dtm, home_x, home_y, home_elev_m)
    root = ET.fromstring(base_world)
    world = root.find("world")
    if world is None:
        raise ValueError("base world SDF has no <world> element")
    world.set("name", world_name)

    # 1. The flat ground plane would shadow terrain below z=0 — but only drop it
    #    when the terrain actually provides collision. With `collision=False`
    #    (the visual-only diagnostic) removing it too would leave the world with
    #    NO collision geometry at all: the vehicle falls through, and PX4 reports
    #    "ekf2 missing data" and never becomes ready. Keeping the plane is also
    #    what makes that diagnostic valid — known-good flat collision underneath,
    #    heightmap present only visually.
    if collision:
        for model in world.findall("model"):
            if model.get("name") == "ground_plane":
                world.remove(model)

    # 2. World origin := the plan's home, so it agrees with PX4_HOME_*.
    sph = world.find("spherical_coordinates")
    if sph is None:
        sph = ET.SubElement(world, "spherical_coordinates")
        ET.SubElement(sph, "surface_model").text = "EARTH_WGS84"
        ET.SubElement(sph, "world_frame_orientation").text = "ENU"
    for tag, value in (
        ("latitude_deg", f"{home_lat:.8f}"),
        ("longitude_deg", f"{home_lon:.8f}"),
        ("elevation", f"{home_elev_m:.2f}"),
    ):
        node = sph.find(tag)
        if node is None:
            node = ET.SubElement(sph, tag)
        node.text = value

    # 3. The terrain itself.
    geom_fields = {
        "dem_uri": dem_path,
        "size_x": geom.size_x_m,
        "size_y": geom.size_y_m,
        "size_z": geom.relief_m,
    }
    terrain = ET.fromstring(
        _TERRAIN_MODEL.format(
            collision=_COLLISION.format(**geom_fields) if collision else "",
            off_x=geom.offset_x_m,
            off_y=geom.offset_y_m,
            off_z=geom.offset_z_m,
            **geom_fields,
        )
    )
    world.append(terrain)

    ET.indent(root, space="  ")
    header = '<?xml version="1.0" encoding="UTF-8"?>\n'
    note = "<!-- Generated by droneplanner gz-world: PX4 default world + planned terrain. -->\n"
    return header + note + ET.tostring(root, encoding="unicode") + "\n"
