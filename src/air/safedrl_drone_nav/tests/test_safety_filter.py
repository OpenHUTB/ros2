"""
Unit tests for the control-barrier-function safety filter.

These are deliberately *analytic*: every case below has a hand-derived
closed-form optimum, so the test pins the projection math rather than merely
checking that the code runs.  Execute with::

    python tests/test_safety_filter.py          # no pytest required
    python -m pytest tests/ -v                  # if pytest is installed
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from safety import CBFFilter, CBFConfig, LagrangeMultiplier, project_halfspace_ball  # noqa: E402


def _approx(a, b, tol=1e-7):
    return np.allclose(np.asarray(a, float), np.asarray(b, float), atol=tol)


# --------------------------------------------------------------------------
# closed-form projection cases
# --------------------------------------------------------------------------


def test_feasible_command_is_untouched():
    """If the desired velocity already satisfies the barrier, keep it."""
    v = project_halfspace_ball(np.array([0.5, 0.2, 0.0]), np.array([1.0, 0, 0]), 0.1, 2.0)
    assert _approx(v, [0.5, 0.2, 0.0]), v


def test_saturating_against_obstacle():
    """Command flies straight into the obstacle -> pushed to the boundary."""
    v = project_halfspace_ball(np.array([-1.0, 0.0, 0.0]), np.array([1.0, 0, 0]), 1.0, 2.0)
    assert _approx(v, [1.0, 0.0, 0.0]), v


def test_minimum_intervention_is_not_axis_aligned():
    """The optimum is the *smallest* correction, which generally tilts."""
    v = project_halfspace_ball(np.array([0.0, 1.0, 0.0]), np.array([1.0, 0, 0]), 1.0, 2.0)
    assert _approx(v, [1.0, 1.0, 0.0]), v
    # correction magnitude is minimal among all feasible points
    tangent = np.array([1.0, np.sqrt(3.0), 0.0])  # also on the ball, a.v = 1
    assert np.linalg.norm(v - [0, 1, 0]) <= np.linalg.norm(tangent - [0, 1, 0]) + 1e-9


def test_unreachable_constraint_saturates_on_ball():
    """b > radius cannot be met; the filter saturates along the gradient."""
    v = project_halfspace_ball(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0, 0]), 3.0, 2.0)
    assert _approx(v, [2.0, 0.0, 0.0]), v


def test_inactive_constraint_respects_actuator_ball():
    """A far-away obstacle must not defeat the velocity limit."""
    v = project_halfspace_ball(np.array([3.0, 0.0, 0.0]), np.array([1.0, 0, 0]), -5.0, 2.0)
    assert _approx(v, [2.0, 0.0, 0.0]), v


def test_infeasible_desire_clamped_into_ball():
    """v_des outside the ball is clamped, yet the barrier still holds."""
    a = np.array([1.0, 0.0, 0.0])
    v_des = np.array([-5.0, 0.0, 0.0])          # outside the ball, pointing in
    v = project_halfspace_ball(v_des, a, 0.5, 2.0)
    assert float(np.linalg.norm(v)) <= 2.0 + 1e-9
    assert float(np.dot(a, v)) >= 0.5 - 1e-9
    # v_des clamps to [-2,0,0]; the plane projection [0.5,0,0] is inside the
    # ball, so it is the true optimum (and closer than any ball-boundary point)
    assert _approx(v, [0.5, 0.0, 0.0]), v
    for boundary in ([0.5, 1.9365, 0.0], [0.5, -1.9365, 0.0]):
        assert np.linalg.norm(v - [-2, 0, 0]) <= np.linalg.norm(np.subtract(boundary, [-2, 0, 0])) + 1e-9


def test_projection_lands_on_ball_boundary_when_plane_falls_outside():
    """Force the spherical-cap branch: plane projection is unreachable."""
    a = np.array([1.0, 0.0, 0.0])
    v = project_halfspace_ball(np.array([0.0, 1.5, 0.0]), a, 1.9, 2.0)
    assert abs(float(np.dot(a, v)) - 1.9) < 1e-9          # active constraint
    assert abs(float(np.linalg.norm(v)) - 2.0) < 1e-9     # on the ball
    assert _approx(v, [1.9, np.sqrt(4.0 - 1.9 ** 2), 0.0]), v


# --------------------------------------------------------------------------
# discrete-time barrier condition
# --------------------------------------------------------------------------


def test_filter_enforces_discrete_cbf_condition():
    """h(x_{k+1}) >= (1 - gamma) h(x_k) for a straight approach."""
    dt, gamma, d_safe, radius = 1 / 30.0, 0.55, 0.34, 2.6
    filt = CBFFilter(CBFConfig(d_safe=d_safe, gamma=gamma, v_max=radius), dt=dt)

    pos = np.array([0.0, 0.0, 0.0])
    obstacle = np.array([1.0, 0.0, 0.0])
    obs_r = 0.3
    drone_r = 0.1

    for _ in range(40):
        d = pos - obstacle
        n = np.linalg.norm(d)
        h = n - obs_r - drone_r - d_safe
        grad = d / n
        v_des = np.array([2.6, 0.0, 0.0])          # drive straight at it
        v = filt.filter(v_des, np.array([h]), np.array([grad]))
        pos = pos + v * dt

    d = pos - obstacle
    h_next = np.linalg.norm(d) - obs_r - drone_r - d_safe
    # the drone must never have entered the unsafe set
    assert h_next > 0.0, f"unsafe set entered, h={h_next:.4f}"
    assert filt.stats.n_interventions > 0


def test_filter_never_exceeds_velocity_limit():
    dt = 1 / 30.0
    filt = CBFFilter(CBFConfig(d_safe=0.3, gamma=0.5, v_max=2.6), dt=dt)
    rng = np.random.default_rng(3)
    for _ in range(200):
        h = rng.uniform(-0.5, 1.0, size=3)
        g = rng.normal(size=(3, 3))
        g /= np.linalg.norm(g, axis=1, keepdims=True)
        v = filt.filter(rng.uniform(-1, 1, size=3) * 2.6, h, g)
        assert np.linalg.norm(v) <= 2.6 + 1e-9


def test_filter_is_deterministic_and_stateless_across_calls():
    filt = CBFFilter(CBFConfig(), dt=1 / 30.0)
    args = (np.array([1.0, 0.5, 0.0]), np.array([0.05, 0.2]),
            np.array([[1.0, 0, 0], [0, 1.0, 0]]))
    a = filt.filter(*args)
    b = filt.filter(*args)
    assert _approx(a, b)


# --------------------------------------------------------------------------
# Lagrange multiplier dual ascent
# --------------------------------------------------------------------------


def test_lagrange_rises_on_violation_and_decays_when_satisfied():
    lm = LagrangeMultiplier(cost_limit=1.0, lr=0.1, lambda_max=10.0)
    for _ in range(40):
        lm.update(3.0)                     # persistently above the limit
    assert lm.lam > 0.5, lm.lam

    peak = lm.lam
    for _ in range(200):
        lm.update(0.0)                     # constraint comfortably satisfied
    assert lm.lam < peak
    assert lm.lam >= 0.0                   # projected onto the non-negative orthant


# --------------------------------------------------------------------------


def _main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {fn.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"  ERROR {fn.__name__}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} safety-filter tests passed.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_main())
