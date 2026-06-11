"""Testes do pareamento célula de carga × touch sensor (force_sync_node).

Só a função pura pair_if_fresh — sem nós, sem UDP. Requer rclpy/msgs no
ambiente (import do módulo) — `colcon test --packages-select touch_pack`.
"""
import pytest

pytest.importorskip('rclpy')
from touch_pack.force_sync_node import pair_if_fresh  # noqa: E402

MAX_AGE = 0.25


def test_none_when_missing_source():
    assert pair_if_fresh(10.0, None, (1.0, 9.99), MAX_AGE) is None
    assert pair_if_fresh(10.0, (2.0, 9.99), None, MAX_AGE) is None
    assert pair_if_fresh(10.0, None, None, MAX_AGE) is None


def test_pair_when_both_fresh():
    pair = pair_if_fresh(10.0, (2.5, 9.99), (0.7, 9.95), MAX_AGE)
    assert pair is not None
    lc, touch, lc_age_ms, th_age_ms = pair
    assert lc == 2.5 and touch == 0.7
    assert lc_age_ms == pytest.approx(10.0, abs=1e-6)
    assert th_age_ms == pytest.approx(50.0, abs=1e-6)


def test_none_when_either_stale():
    fresh = (1.0, 9.99)
    stale = (1.0, 9.99 - MAX_AGE - 0.1)
    assert pair_if_fresh(10.0, stale, fresh, MAX_AGE) is None
    assert pair_if_fresh(10.0, fresh, stale, MAX_AGE) is None


def test_boundary_age_is_accepted():
    # Exatamente na idade limite ainda entra (> max_age descarta).
    on_edge = (1.0, 10.0 - MAX_AGE)
    assert pair_if_fresh(10.0, on_edge, on_edge, MAX_AGE) is not None
