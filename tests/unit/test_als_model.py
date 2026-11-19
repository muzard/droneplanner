"""Unit tests for the airborne-lidar trunk-return model (SHORTFALLS M2)."""

from __future__ import annotations

import numpy as np
import pytest

from droneplanner.fields.stems import StemMap
from droneplanner.forest.als_model import (
    AlsScanConfig,
    expected_trunk_returns,
    mean_tan_off_nadir,
    required_pulse_density,
    returns_per_stem,
    simulate_als_scan,
)


def _lattice(spacing: float = 10.0, side: float = 200.0, r: float = 0.15) -> StemMap:
    xs, ys = np.meshgrid(
        np.arange(spacing / 2, side, spacing), np.arange(spacing / 2, side, spacing)
    )
    return StemMap(x=xs.ravel(), y=ys.ravel(), radius_m=np.full(xs.size, r))


def test_mean_tan_matches_numeric_integral() -> None:
    t = np.linspace(0.0, np.radians(20.0), 200_001)
    assert mean_tan_off_nadir(20.0) == pytest.approx(float(np.mean(np.tan(t))), rel=1e-4)
    assert mean_tan_off_nadir(20.0) == pytest.approx(0.178, abs=0.001)
    assert mean_tan_off_nadir(0.0) == 0.0


def test_national_als_puts_a_fraction_of_one_return_on_a_stem() -> None:
    """The headline: MML-like ALS gives ~0.2 slice returns against min_points 8."""
    e = expected_trunk_returns(5.0, 0.15, 1.5, 5.0, crown_base_m=8.0, crown_transmission=0.2)
    assert 0.15 < e < 0.25
    need = required_pulse_density(8, 0.15, 1.5, 5.0, crown_base_m=8.0, crown_transmission=0.2)
    assert need == pytest.approx(8.0 / (e / 5.0))
    assert need > 200.0


def test_slice_above_the_crown_base_is_unreachable() -> None:
    e = expected_trunk_returns(100.0, 0.15, 9.0, 12.0, crown_base_m=8.0, crown_transmission=0.2)
    assert e == 0.0
    assert required_pulse_density(
        8, 0.15, 9.0, 12.0, crown_base_m=8.0, crown_transmission=0.2
    ) == float("inf")


def test_crown_base_inside_slice_adds_the_top_disc() -> None:
    side_only = expected_trunk_returns(
        10.0, 0.2, 1.0, 4.0, crown_base_m=4.0 + 1e-9, crown_transmission=1.0, max_off_nadir_deg=0.0
    )
    with_top = expected_trunk_returns(
        10.0, 0.2, 1.0, 4.0, crown_base_m=4.0, crown_transmission=1.0, max_off_nadir_deg=0.0
    )
    assert side_only == 0.0  # vertical pulses never touch the side
    assert with_top == pytest.approx(10.0 * np.pi * 0.2**2)


@pytest.mark.parametrize("density", [20.0, 200.0])
def test_ray_trace_agrees_with_the_formula_on_isolated_stems(density: float) -> None:
    stems = _lattice()
    cfg = AlsScanConfig(pulse_density_m2=density)
    cloud = simulate_als_scan(stems, (0.0, 0.0, 200.0, 200.0), cfg, seed=3)
    got = float(np.mean(returns_per_stem(cloud, stems, 1.5, 5.0)))
    want = expected_trunk_returns(
        density, 0.15, 1.5, 5.0, crown_base_m=cfg.crown_base_m, crown_transmission=0.2
    )
    assert got == pytest.approx(want, rel=0.08)


def test_trunks_shadow_each_other() -> None:
    """A trunk right beside another one gets fewer returns than an isolated trunk."""
    lone = StemMap(x=np.array([50.0]), y=np.array([50.0]), radius_m=np.array([0.15]))
    ring = 20
    ang = np.linspace(0.0, 2 * np.pi, ring, endpoint=False)
    crowd = StemMap(
        x=np.r_[50.0, 50.0 + 0.6 * np.cos(ang)],
        y=np.r_[50.0, 50.0 + 0.6 * np.sin(ang)],
        radius_m=np.full(ring + 1, 0.15),
    )
    cfg = AlsScanConfig(pulse_density_m2=20_000.0, crown_transmission=1.0)
    box = (45.0, 45.0, 55.0, 55.0)
    lone_n = returns_per_stem(
        simulate_als_scan(lone, box, cfg, seed=1), lone, 1.5, 5.0, within_m=0.05
    )[0]
    crowd_n = returns_per_stem(
        simulate_als_scan(crowd, box, cfg, seed=1), crowd, 1.5, 5.0, within_m=0.05
    )[0]
    assert crowd_n < 0.5 * lone_n


def test_vertical_pulses_hit_only_at_the_crown_base() -> None:
    stems = StemMap(x=np.array([5.0]), y=np.array([5.0]), radius_m=np.array([0.5]))
    cfg = AlsScanConfig(
        pulse_density_m2=2000.0,
        max_off_nadir_deg=0.0,
        crown_transmission=1.0,
        range_noise_m=0.0,
        ground_return_fraction=0.0,
    )
    cloud = simulate_als_scan(stems, (0.0, 0.0, 10.0, 10.0), cfg, seed=2)
    assert len(cloud) > 0
    assert np.allclose(cloud[:, 2], cfg.crown_base_m)
    assert np.all(np.hypot(cloud[:, 0] - 5.0, cloud[:, 1] - 5.0) <= 0.5 + 1e-9)


def test_hits_lie_on_the_trunk_surface() -> None:
    stems = _lattice(spacing=8.0, side=40.0)
    cfg = AlsScanConfig(pulse_density_m2=2000.0, range_noise_m=0.0, ground_return_fraction=0.0)
    cloud = simulate_als_scan(stems, (0.0, 0.0, 40.0, 40.0), cfg, seed=4)
    side = cloud[cloud[:, 2] < cfg.crown_base_m - 1e-6]
    assert len(side) > 100
    d, _ = stems.tree.query(side[:, :2])
    assert np.allclose(d, 0.15, atol=1e-6)


def test_no_stems_gives_ground_only_and_is_deterministic() -> None:
    empty = StemMap(x=np.empty(0), y=np.empty(0), radius_m=np.empty(0))
    cfg = AlsScanConfig(pulse_density_m2=5.0)
    a = simulate_als_scan(empty, (0.0, 0.0, 50.0, 50.0), cfg, ground_elev=30.0, seed=9)
    b = simulate_als_scan(empty, (0.0, 0.0, 50.0, 50.0), cfg, ground_elev=30.0, seed=9)
    assert np.array_equal(a, b)
    assert np.all(np.abs(a[:, 2] - 30.0) < 0.2)
    assert len(returns_per_stem(a, empty, 1.5, 5.0)) == 0
    zero = AlsScanConfig(pulse_density_m2=0.0)
    assert simulate_als_scan(empty, (0.0, 0.0, 1.0, 1.0), zero).shape == (0, 3)
