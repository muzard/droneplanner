"""Forest flyability model (Software-Spec §5.2) — implemented in M1/M2.

Planned pipeline: voxelize lidar -> vertical segmentation -> EDT clearance ->
navigability classification (FLYABLE / MARGINAL / NON_NAVIGABLE) -> connected
corridor test. Not yet built in the M0 scaffold.
"""

from __future__ import annotations

__all__: list[str] = []
