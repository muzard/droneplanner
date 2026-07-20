"""Derived 2.5D cost/constraint fields (Software-Spec §5.1) — implemented in M1.

Planned builders (each ``build(inputs, config) -> np.ndarray`` normalized 0..1):
``viewshed`` (visual exposure / hideability), ``terrain`` (TPI, slope, openness,
drainage), ``survivability`` (defilade / fragment exposure), ``snag`` (fiber
snag risk). Not yet built in the M0 scaffold.
"""

from __future__ import annotations

__all__: list[str] = []
