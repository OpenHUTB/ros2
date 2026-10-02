"""
Integration tests for the navigation environment and the safety guarantee.

Unlike test_safety_filter.py (which pins the projection maths), these tests
exercise the *closed-loop* system in PyBullet and assert the properties the
report claims.  They are slower (a few seconds each) but need no ROS.

    python tests/test_environment.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from safe_nav_env import EnvConfig, SafeNavEnv  # noqa: E402


def _rollout(env, policy, seed, max_steps=900):
    obs, info = env.reset(seed=seed)
    done, last, n = False, info, 0
    while not done and n < max_steps:
        obs, _r, term, trunc, info = env.step(policy(env, obs))
        last = info
        done = term or trunc
        n += 1
    return last, n


def _hover_policy(env, obs):
    return np.zeros(4)


def _pursuit_policy(env, obs):
    pos = env.drone_state()["position"]
    gate = env.gates[min(env._gate_idx, env.cfg.n_gates - 1)]
    d = gate - pos
    n = float(np.linalg.norm(d))
    u = d / n if n > 1e-6 else np.zeros(3)
    return np.array([u[0] * 0.9, u[1] * 0.9, u[2] * 0.9, 0.0])


# --------------------------------------------------------------------------


def test_zero_action_hovers_for_a_full_episode():
    """A do-nothing policy must be able to stay airborne; if this fails the
    plant or the controller is broken, not the learning."""
    env = SafeNavEnv(EnvConfig(gui=False, use_cbf=False))
    last, n = _rollout(env, _hover_policy, seed=0)
    z = env.drone_state()["position"][2]
    env.close()
    assert n >= 890, f"hover only survived {n} steps"
    assert abs(z - 1.5) < 0.15, f"altitude drifted to {z:.3f}"


def test_observation_and_action_spaces():
    env = SafeNavEnv(EnvConfig(gui=False))
    obs, info = env.reset(seed=0)
    c = env.cfg
    expected = 3 + 3 + 3 + c.n_nearest_obs * 4 + c.n_rays + 4
    assert obs.shape == (expected,), obs.shape
    assert np.isfinite(obs).all(), "observation contains NaN/inf"
    assert env.action_space.shape == (4,)
    env.close()


def test_gates_are_a_slalom_not_a_line():
    """If the gates were colinear a constant forward command would solve the
    task, and the DRL would be pointless."""
    cfg = EnvConfig()
    g = cfg.gate_positions
    assert len(np.unique(np.round(g[:, 1], 3))) > 1, "gates share a lateral offset"
    assert len(np.unique(np.round(g[:, 2], 3))) > 1, "gates share an altitude"
    assert np.ptp(g[:, 1]) > 1.0, "lateral spread too small to demand navigation"


def test_pure_pursuit_solves_the_task():
    """The task must be achievable by a competent controller - otherwise
    'the DRL failed' would be unfalsifiable."""
    env = SafeNavEnv(EnvConfig(gui=False, use_cbf=True))
    ok = 0
    for seed in range(6):
        last, _ = _rollout(env, _pursuit_policy, seed=seed)
        ok += int(last.get("is_success", False))
    env.close()
    assert ok >= 5, f"pure pursuit only succeeded {ok}/6 - task may be unsolvable"


def test_cbf_never_permits_contact_with_an_obstacle():
    """The headline safety claim: with the filter on, the vehicle must never
    touch a static/dynamic obstacle, even under an adversarial policy that
    steers straight at the nearest one."""
    def adversarial(env, obs):
        pos = env.drone_state()["position"]
        d, idx = env._nearest_obstacle_distance(pos)
        target = (env.obstacles[idx]["centre"] if (d < 3.0 and idx >= 0)
                  else pos + np.array([0.0, 0.0, -5.0]))
        v = target - pos
        n = float(np.linalg.norm(v))
        u = v / n if n > 1e-6 else np.zeros(3)
        return np.array([u[0], u[1], u[2], 0.0])

    env = SafeNavEnv(EnvConfig(gui=False, use_cbf=True))
    worst = np.inf
    for seed in range(6):
        last, _ = _rollout(env, adversarial, seed=seed)
        worst = min(worst, last.get("min_contact_clearance", np.inf))
    env.close()
    # clearances are margin + d_safe, so > 0 means the airframe never touched
    assert worst > 0.0, f"adversarial policy reached contact, clearance {worst:+.3f} m"


def test_cbf_reduces_adversarial_penetration():
    """With the filter off the same adversary penetrates; this is the
    comparison that makes the previous test meaningful."""
    def adversarial(env, obs):
        pos = env.drone_state()["position"]
        d, idx = env._nearest_obstacle_distance(pos)
        target = (env.obstacles[idx]["centre"] if (d < 3.0 and idx >= 0)
                  else pos + np.array([0.0, 0.0, -5.0]))
        v = target - pos
        n = float(np.linalg.norm(v))
        u = v / n if n > 1e-6 else np.zeros(3)
        return np.array([u[0], u[1], u[2], 0.0])

    env = SafeNavEnv(EnvConfig(gui=False, use_cbf=False))
    worst_off = np.inf
    for seed in range(6):
        last, _ = _rollout(env, adversarial, seed=seed)
        worst_off = min(worst_off, last.get("min_contact_clearance", np.inf))
    env.close()
    assert worst_off < 0.0, (
        "expected the unfiltered adversary to penetrate an obstacle, "
        f"but worst clearance was {worst_off:+.3f} m"
    )


def test_cost_is_reported_and_bounded():
    env = SafeNavEnv(EnvConfig(gui=False, use_cbf=True))
    last, _ = _rollout(env, _hover_policy, seed=0)
    env.close()
    assert "episode_cost" in last
    assert last["episode_cost"] >= 0.0


def test_episodes_are_reproducible_from_a_seed():
    """Same seed => same obstacle layout, gate positions and gust sequence."""
    def signature(seed):
        env = SafeNavEnv(EnvConfig(gui=False, seed=seed))
        env.reset(seed=seed)
        sig = (
            tuple(np.round(env.gates.ravel(), 6)),
            tuple(np.round(np.concatenate([o["centre"] for o in env.obstacles]), 6)),
            tuple(round(o["radius"], 6) for o in env.obstacles),
        )
        env.close()
        return sig

    assert signature(7) == signature(7)
    assert signature(7) != signature(8)


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
    print(f"\n{len(tests) - failed}/{len(tests)} environment tests passed.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_main())
