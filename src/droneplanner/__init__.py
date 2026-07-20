"""droneplanner — offline preflight route planner for the fiber-laying drone.

Implements the Preflight Planning Module described in ``Software-Spec.md`` and
``Preflight-Planning-Module-Design.md``. This package is pure software: it turns
a mission (start, end, map layers, threat model, drone/spool geometry) into a
3D flight-and-laydown plan plus a human-in-the-loop risk report.

Milestone status: **M0 — scaffold** (straight-line stub planner + report).
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
