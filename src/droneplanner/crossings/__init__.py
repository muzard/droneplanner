"""Road / linear-obstacle crossing solver (Software-Spec §5.4) — M1.

Builds a graph of crossings and assigns each the lowest-cost feasible method by
rank: existing culvert/underpass/bridge -> natural canopy bridge -> overhead
branch span (flagged: needs manipulation) -> open crossing. Not yet built.
"""

from __future__ import annotations

__all__: list[str] = []
