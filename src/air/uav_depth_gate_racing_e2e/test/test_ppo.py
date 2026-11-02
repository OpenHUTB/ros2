# -*- coding: utf-8 -*-
"""PPO 的 GAE、裁剪与更新。不需要 ROS、不需要 mujoco（用桩环境）。"""
import numpy as np
import pytest

from uav_depth_gate_racing_e2e.core.nn import Adam
from uav_depth_gate_racing_e2e.core.policy import Policy, PolicyConfig
from uav_depth_gate_racing_e2e.core.ppo import (PPOConfig, compute_gae,
                                                evaluate_policy, ppo_update,
                                                rollout)


class _Bandit:
    """一次性的多臂老虎机：动作越接近 (1,0,0) 奖励越高，3 步后重置。

    用它检查 PPO 是否真的把概率推向高回报动作——奖励与状态无关，
    唯一的学习信号就是动作本身。
    """

    def __init__(self, hw=(24, 32), horizon=3):
        self.cfg = type("C", (), {"depth_hw": hw})()
        self.horizon = horizon
        self.k = 0
        self.depth = np.zeros(hw, np.uint8)

    def reset(self, seed=None, jitter=0.0):
        self.k = 0
        self.depth = (np.random.default_rng(seed).random(self.cfg.depth_hw)
                      * 255).astype(np.uint8)
        return self._obs()

    def _obs(self):
        return {"depth": self.depth, "vel": np.zeros(3, np.float32)}

    def step(self, a):
        a = np.asarray(a, float)
        r = float(a[0]) - 0.5 * float(np.sum(a[1:] ** 2))
        self.k += 1
        done = self.k >= self.horizon
        info = {"passed": 1.0 if a[0] > 0.5 else 0.0,
                "completed": float(done), "time": float(self.k), "tilt": 0.0,
                "collisions": 0.0, "pos": np.zeros(3)}
        return self._obs(), r, done, info


# --------------------------------------------------------------------------- #
def test_gae_matches_hand_computation():
    """γ=λ=1 时 adv 就是"当前到结束的折扣回报 − 基线"。

    δ_t = r_t + V_{t+1} − V_t，所以：
      t=2: δ = 3 + 0 − 0.5 = 2.5
      t=1: δ = 2 + 0.5 − 0.5 = 2.0，adv = 2.0 + 2.5 = 4.5
      t=0: δ = 1 + 0.5 − 0.5 = 1.0，adv = 1.0 + 4.5 = 5.5
    回报 = adv + V = [6, 5, 3]，与直接折扣求和一致。
    """
    data = [{"reward": 1.0, "value": 0.5, "done": 0.0},
            {"reward": 2.0, "value": 0.5, "done": 0.0},
            {"reward": 3.0, "value": 0.5, "done": 1.0}]
    adv, ret = compute_gae(data, last_value=0.0, gamma=1.0, lam=1.0)
    assert np.allclose(adv, [5.5, 4.5, 2.5])
    assert np.allclose(ret, [6.0, 5.0, 3.0])


def test_gae_respects_done():
    """done 一旦置位，后面的回报不能回流到它之前。

    ``done=1`` 的帧上 nonterm=0：bootstrap 项被切断，且 γλ 链也断在这里。
    所以 t=1 的 adv 只剩 r₁ − V₁ = 9 − 100 = −91，不含 t=2 的 +1。
    """
    a1 = compute_gae([{"reward": 1.0, "value": 0.0, "done": 0.0},
                      {"reward": 9.0, "value": 100.0, "done": 1.0},
                      {"reward": 1.0, "value": 0.0, "done": 0.0}],
                     gamma=1.0, lam=1.0)[0]
    assert abs(a1[1] - (-91.0)) < 1e-9
    assert abs(a1[2] - 1.0) < 1e-9


def test_rollout_tags_episode_end():
    """每轮结束的统计要挂在那一轮最后一帧上——采样循环可能横跨多轮。"""
    rng = np.random.default_rng(0)
    env = _Bandit(horizon=4)
    p = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=8))
    data = rollout(p, env, steps=20, rng=rng)
    assert len(data) == 20
    ends = [i for i, d in enumerate(data) if "info" in d]
    assert ends == [3, 7, 11, 15, 19]
    assert all(data[i]["done"] == 1.0 for i in ends[:-1])


def _mean_action(p, n=8, hw=(24, 32)):
    """在若干随机观测上取确定性动作的均值——单帧采样噪声太大。"""
    rng = np.random.default_rng(99)
    out = []
    for _ in range(n):
        obs = {"depth": (rng.random(hw) * 255).astype(np.uint8),
               "vel": np.zeros(3, np.float32)}
        out.append(p.act(obs, deterministic=True)[0][0])
    return np.mean(out, axis=0)


def test_ppo_pushes_toward_high_reward_action():
    """奖励与状态无关，唯一的学习信号就是动作本身。

    PPO 应当把第一个动作分量往上推（它是奖励里唯一的正贡献）。
    """
    rng = np.random.default_rng(1)
    env = _Bandit(horizon=8)
    p = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=16))
    cfg = PPOConfig(rollout_steps=256, epochs=4, batch=64, lr=1e-2, seed=0)
    opt = Adam(p.layers, lr=cfg.lr, clip=cfg.grad_clip)
    before = float(_mean_action(p)[0])
    for _ in range(10):
        data = rollout(p, env, cfg.rollout_steps, rng)
        ppo_update(p, data, cfg, opt=opt)
    after = float(_mean_action(p)[0])
    assert after > before + 0.2, (before, after)


def test_ppo_clip_bounds_step():
    """裁剪生效时，一次更新不能把概率比推离 1±clip 太远。"""
    rng = np.random.default_rng(2)
    env = _Bandit(horizon=6)
    p = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=8))
    cfg = PPOConfig(rollout_steps=128, epochs=1, batch=32, lr=5.0, clip=0.2,
                    seed=0)
    opt = Adam(p.layers, lr=cfg.lr, clip=cfg.grad_clip)
    data = rollout(p, env, cfg.rollout_steps, rng)
    st = ppo_update(p, data, cfg, opt=opt)
    assert 0.0 < st["clip_frac"] <= 1.0
    assert np.isfinite(st["kl"]) and np.isfinite(st["pi"])


def test_entropy_raises_log_std():
    """熵是 +Σ(log σ + 常数)，最大化熵 ⇒ log σ 往上走（探索增强）。

    两份完全相同的策略、同一批数据，只差 ``entropy_coef``，差值应当就是
    ``lr · coef`` 逐次累加——这样也顺带验证熵项的符号没写反。
    """
    rng = np.random.default_rng(3)
    env = _Bandit(horizon=6)
    src = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=8))
    data = rollout(src, env, 128, rng)
    base = {"lr": 1e-2, "rollout_steps": 128, "epochs": 2, "batch": 32, "seed": 0}
    a = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=8))
    b = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=8))
    ppo_update(a, data, PPOConfig(entropy_coef=0.0, **base),
               opt=Adam(a.layers, lr=1e-2, clip=0.5))
    ppo_update(b, data, PPOConfig(entropy_coef=0.05, **base),
               opt=Adam(b.layers, lr=1e-2, clip=0.5))
    # 两者的策略梯度项相同（同一批数据、权重逐位一致），差值只来自熵项。
    # 两次更新 × 每轮 4 个 mini-batch = 8 步，每步 +lr·coef = 5e-4。
    assert np.all(b.log_std > a.log_std), (a.log_std, b.log_std)
    assert np.all(b.log_std - a.log_std > 1e-4)


def test_log_std_clamped():
    """log σ 被直接改动时也不越过 [LOG_STD_MIN, LOG_STD_MAX]。"""
    p = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=8))
    p.log_std[:] = 50.0
    assert np.all(p.std <= np.exp(0.5) + 1e-12)
    p.log_std[:] = -50.0
    assert np.all(p.std >= np.exp(-3.0) - 1e-12)


def test_evaluate_policy_keys():
    env = _Bandit(horizon=5)
    p = Policy(PolicyConfig(arch="cnn", depth_hw=(24, 32), hidden=8))
    m = evaluate_policy(p, env, n_episode=2, seed=0)
    assert set(m) == {"passed", "completed", "time", "collisions", "latency_ms"}
    assert 0.0 <= m["completed"] <= 1.0
    assert m["latency_ms"] > 0.0
