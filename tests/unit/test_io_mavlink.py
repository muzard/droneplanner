"""Speed commanding in the QGC .plan writer."""

from __future__ import annotations

import pytest


def test_commanded_speed_never_exceeds_the_cap_and_endurance_uses_it() -> None:
    """🔴 Askola measured zone (2026-09-25): the emitter floored 0.76 m/s to 0.5 while
    endurance charged 0.76 (SITL 21.3 min flown vs 17.5 planned), and a 0.4 m/s cap
    was emitted as 0.5 — faster than the planner's own limit."""
    from droneplanner.model.plan import commanded_speed_ms

    assert commanded_speed_ms(6.0) == 6.0
    assert commanded_speed_ms(0.76) == 0.5
    assert commanded_speed_ms(1.49) == 1.0
    assert commanded_speed_ms(0.4) == pytest.approx(0.4)
    assert commanded_speed_ms(0.43) == pytest.approx(0.4)
    assert commanded_speed_ms(0.05) == pytest.approx(0.1)  # floor, not a stop
    for cap in (0.1, 0.3, 0.49, 0.5, 0.99, 2.2, 5.9):
        assert commanded_speed_ms(cap) <= cap + 1e-9
