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

✅ **Vertical datum confirmed in SITL** (2026-07-21): Gazebo places a DEM's
*minimum* elevation at the model pose's z, so offsetting by
``-(home_elev - dtm_min)`` puts the ground at home at exactly z=0 — the vehicle
was measured resting at z = -0.00 m with ``dist_bottom`` 0.01.

**We inject into PX4's own world rather than writing one.** See
:func:`build_world_sdf`; a from-scratch world silently loses the default gz-sim
plugin set and the ``<magnetic_field>``, which kills the sensors and then the
attitude estimate. Pass PX4's ``Tools/simulation/gz/worlds/default.sdf``.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from droneplanner.io.raster import RasterGrid

_TERRAIN_MODEL = """
<model name="terrain">
  <static>true</static>
  <!-- x,y: (DTM centre - home) so home lands on the world origin.
       z: vertical datum offset - if the vehicle spawns buried, increase;
          if it spawns hovering, decrease. -->
  <pose>{off_x:.3f} {off_y:.3f} {off_z:.3f} 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>file://{dem_uri}</uri>
          <size>{size_x:.2f} {size_y:.2f} {size_z:.2f}</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>
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


@dataclass(frozen=True)
class WorldGeometry:
    """The computed placement of a DTM heightmap relative to the plan's home."""

    size_x_m: float  # DTM width in meters
    size_y_m: float  # DTM height in meters
    relief_m: float  # elevation range (max - min)
    min_elev_m: float
    offset_x_m: float  # model pose x: DTM centre - home (projected meters)
    offset_y_m: float
    offset_z_m: float  # vertical datum offset


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
    relief = float(dtm.data.max()) - min_elev
    return WorldGeometry(
        size_x_m=size_x,
        size_y_m=size_y,
        relief_m=relief,
        min_elev_m=min_elev,
        offset_x_m=centre_x - home_x,
        offset_y_m=centre_y - home_y,
        # Put the ground at home at world z=0 (see the module docstring caveat).
        offset_z_m=-(home_elev_m - min_elev),
    )


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
) -> str:
    """Inject the terrain heightmap into PX4's own world SDF.

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

    # 1. The flat ground plane would shadow the terrain below z=0.
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
    terrain = ET.fromstring(
        _TERRAIN_MODEL.format(
            off_x=geom.offset_x_m,
            off_y=geom.offset_y_m,
            off_z=geom.offset_z_m,
            size_x=geom.size_x_m,
            size_y=geom.size_y_m,
            size_z=geom.relief_m,
            dem_uri=dem_path,
        )
    )
    world.append(terrain)

    ET.indent(root, space="  ")
    header = '<?xml version="1.0" encoding="UTF-8"?>\n'
    note = "<!-- Generated by droneplanner gz-world: PX4 default world + planned terrain. -->\n"
    return header + note + ET.tostring(root, encoding="unicode") + "\n"
