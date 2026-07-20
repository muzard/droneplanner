"""Cable-laydown prediction (Software-Spec §5, Preflight-Design §6C) — M2.

Predicts where a paying-out fiber settles through vegetation from flight
altitude, feeding the snag-risk and survivability fields. The design doc flags
this as likely the least reliable field, needing empirical calibration. Not yet
built in the M0 scaffold.
"""

from __future__ import annotations

__all__: list[str] = []
