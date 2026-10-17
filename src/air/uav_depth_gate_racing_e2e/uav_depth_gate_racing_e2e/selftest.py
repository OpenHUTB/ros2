#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自检：不需要 ROS、不需要显示器。

    python main.py --selftest
"""
from __future__ import annotations

import math
import time
from typing import List

import numpy as np

from .core.bc import BCConfig, bc_metrics, norm_scale, normalise, train_bc
from .core.expert import ExpertConfig, GateExpert
from .core.gate_env import EnvConfig, GateRacingEnv, GateTrack, depth_to_u8
from .core.nn import (Adam, Conv2d, GRUCell, LayerNorm, Linear, SelfAttention,
                      col2im_add, im2col, log_softmax, softmax)
from .core.policy import ARCHS, Policy, PolicyConfig
from .core.ppo import PPOConfig, compute_gae, evaluate_policy, ppo_update, rollout

__all__ = ["run_selftest"]


class _Report:
    def __init__(self):
        self.n = self.ok = 0
        self.fail: List[str] = []

    def check(self, name: str, ok: bool, note: str = "") -> None:
        self.n += 1
        if ok:
            self.ok += 1
        else:
            self.fail.append(name)
        print("  [%s] %-26s %s" % ("PASS" if ok else "FAIL", name, note))


def _grad_check(layer, loss_fn, tol=2e-4, eps=1e-6):
    _, d = loss_fn()
    layer.backward(2.0 * d / d.size)
    worst = 0.0
    for p, g in zip(layer.params, layer.grads):
        it = np.nditer(p, flags=["multi_index"])
        for _ in it:
            idx = it.multi_index
            old = p[idx]
            p[idx] = old + eps
            lp = loss_fn()[0]
            p[idx] = old - eps
            lm = loss_fn()[0]
            p[idx] = old
            num = (lp - lm) / (2 * eps)
            worst = max(worst, abs(num - g[idx]) / max(abs(num), abs(g[idx]), 1e-8))
    return worst


# --------------------------------------------------------------------------- #
def _sec_nn(rep: _Report, rng) -> None:
    print("\n[1/6] 神经网络")
    cv = Conv2d(3, 5, 3, stride=2, rng=np.random.default_rng(1))
    x = rng.normal(size=(2, 3, 12, 10))
    t = rng.normal(size=(2, 5, 5, 4))

    def lc():
        d = cv.forward(x) - t
        return float(np.mean(d * d)), d

    e = _grad_check(cv, lc)
    rep.check("Conv2d 梯度", e < 2e-4, "最大相对误差 %.2e" % e)

    lin = Linear(6, 4, rng=np.random.default_rng(2))
    xl, tl = rng.normal(size=(9, 6)), rng.normal(size=(9, 4))

    def ll():
        d = lin.forward(xl) - tl
        return float(np.mean(d * d)), d

    e = _grad_check(lin, ll)
    rep.check("Linear 梯度", e < 2e-4, "最大相对误差 %.2e" % e)

    gru = GRUCell(6, 5, rng=np.random.default_rng(3))
    xg, hg = rng.normal(size=(4, 6)), rng.normal(size=(4, 5)) * 0.3
    tg = rng.normal(size=(4, 5))

    def lg():
        d = gru.forward(xg, hg) - tg
        return float(np.mean(d * d)), d

    e = _grad_check(gru, lg)
    rep.check("GRU 参数梯度", e < 2e-4, "最大相对误差 %.2e" % e)

    at = SelfAttention(8, rng=np.random.default_rng(4))
    xa = rng.normal(size=(2, 5, 8))
    ta = rng.normal(size=(2, 5, 8))

    def la():
        d = at.forward(xa) - ta
        return float(np.mean(d * d)), d

    e = _grad_check(at, la)
    rep.check("自注意力梯度", e < 2e-4, "最大相对误差 %.2e" % e)

    A = rng.normal(size=(1, 2, 8, 8))
    cols = im2col(A, 3, 3, 1)
    B = rng.normal(size=cols.shape)
    lhs = float(np.sum(A * col2im_add(B, A.shape, 3, 3, 1)))
    rep.check("im2col 伴随恒等式", abs(lhs - float(np.sum(cols * B))) < 1e-9,
              "%.8f" % lhs)

    lg_ = rng.normal(size=(3, 5))
    rep.check("log_softmax 与 log(softmax) 一致",
              np.abs(log_softmax(lg_) - np.log(softmax(lg_))).max() < 1e-12,
              "最大差 %.2e" % np.abs(log_softmax(lg_)
                                     - np.log(softmax(lg_))).max())

    xx = rng.normal(size=(4, 7))
    ln = LayerNorm()
    y = ln.forward(xx)
    dy = rng.normal(size=(4, 7))
    dx = ln.backward(dy)
    worst = 0.0
    for i in range(4):
        for j in range(7):
            old = xx[i, j]
            xx[i, j] = old + 1e-6
            lp = float((ln.forward(xx) * dy).sum())
            xx[i, j] = old - 1e-6
            lm = float((ln.forward(xx) * dy).sum())
            xx[i, j] = old
            worst = max(worst, abs((lp - lm) / 2e-6 - dx[i, j])
                        / max(abs(dx[i, j]), 1e-7))
    rep.check("LayerNorm 反向", worst < 1e-4, "最大相对误差 %.2e" % worst)


def _sec_env(rep: _Report) -> None:
    print("\n[2/6] 竞速环境")
    env = GateRacingEnv(EnvConfig(max_time=6.0))
    rep.check("门数正确", len(env.track) == 8, "%d 个门" % len(env.track))
    obs = env.reset(seed=0, jitter=0.0)
    rep.check("观测形状", obs["depth"].shape == tuple(env.cfg.depth_hw)
              and obs["vel"].shape == (3,),
              "depth%s vel%s" % (obs["depth"].shape, obs["vel"].shape))
    # 深度图是这个任务唯一的输入。全零图不会让任何形状断言失败，只会让策略
    # 退化成"照平均速度平飞"——所以必须查内容。
    d0 = obs["depth"]
    rep.check("深度图非空", int(d0.max()) > 20 and float((d0 > 0).mean()) > 0.2,
              "最大值 %d，有效像素 %.0f%%" % (d0.max(), 100 * (d0 > 0).mean()))
    obs2, r, done, info = env.step(np.zeros(3))
    rep.check("step 返回结构与奖励", isinstance(r, float)
              and set(info) >= {"passed", "collisions", "time"},
              "r = %.3f，通过 %d" % (r, info["passed"]))

    # 门开口判定：门内通过算通过，门外通过算碰撞
    g = env.track.gates[0]
    rep.check("门内判定", g.contains(np.array([g.x, g.y, g.z])),
              "中心 (%.1f, %.1f, %.1f)" % (g.x, g.y, g.z))
    rep.check("门外判定",
              not g.contains(np.array([g.x, g.y + 2.5, g.z])),
              "y 偏移 2.5 m 判为未通过")

    d = np.array([[0.0, 0.5, 3.0, 20.0]])
    q = depth_to_u8(d)
    rep.check("深度编码：超量程置 0", q[0, 0] == 0 and q[0, 3] == 0
              and q[0, 2] > 0, "%s" % q.tolist())
    env.close()


def _sec_expert(rep: _Report) -> None:
    print("\n[3/6] 专家")
    env = GateRacingEnv(EnvConfig(max_time=25.0))
    ex = GateExpert(ExpertConfig(look_ahead=1.5))
    obs = env.reset(seed=0, jitter=0.0)
    done, info, k = False, {}, 0
    while not done and k < 600:
        obs, _, done, info = env.step(ex.act(env))
        k += 1
    rep.check("专家能跑完全程", info.get("completed", False),
              "通过 %d/%d 门，用时 %.1f s，碰撞 %d"
              % (info.get("passed", 0), len(env.track), info.get("time", 0),
                 info.get("collisions", 0)))
    a = ex.act(env)
    rep.check("专家动作在限幅内",
              abs(a[0]) <= env.cfg.max_horiz + 1e-6
              and abs(a[2]) <= env.cfg.max_vert + 1e-6,
              "v = (%.2f, %.2f, %.2f)" % tuple(a))
    env.close()


def _sec_policy(rep: _Report, rng) -> None:
    print("\n[4/6] 策略网络（四种主干）")
    obs = {"depth": rng.integers(0, 255, (1, 48, 64)).astype(np.uint8),
           "vel": rng.normal(size=(1, 3)).astype(np.float32)}
    for arch in ARCHS:
        p = Policy(PolicyConfig(arch=arch, seed=0))
        h = p.init_hidden(1)
        a, logp, v, h2, raw = p.act(obs, h, rng=rng)
        ok = (a.shape == (1, 3) and np.all(np.abs(a) <= 1.0)
              and np.isfinite(logp[0]) and np.isfinite(v[0]))
        rep.check("%s 前向" % arch, ok,
                  "参数 %6d  动作 (%5.2f,%5.2f,%5.2f)"
                  % (p.n_params(), *a[0]))
    # 反向能跑通且梯度非零
    p = Policy(PolicyConfig(arch="cnn", seed=0))
    h = p.init_hidden(2)
    obs2 = {"depth": rng.integers(0, 255, (2, 48, 64)).astype(np.uint8),
            "vel": rng.normal(size=(2, 3)).astype(np.float32)}
    p.forward(obs2, h)
    p.zero_grad()
    p.backward(np.ones((2, 3)), np.ones(2), np.zeros(3))
    rep.check("策略反向", any(np.abs(g).sum() > 0 for g in p.grads),
              "梯度范数 %.4f"
              % math.sqrt(sum(float((g ** 2).sum()) for g in p.grads)))

    # tanh 采样后 logp 的修正：动作贴近 ±1 时 logp 不应爆炸
    p = Policy(PolicyConfig(arch="mlp", seed=1))
    obs3 = {"depth": np.zeros((1, 48, 64), np.uint8),
            "vel": np.zeros((1, 3), np.float32)}
    p.log_std[:] = 3.0
    a, logp, _, _, _ = p.act(obs3, p.init_hidden(1), rng=rng)
    rep.check("logp 有限（tanh 修正）", np.isfinite(logp[0]),
              "logp = %.2f  动作 %.3f" % (logp[0], a[0, 0]))


def _sec_ppo(rep: _Report, rng) -> None:
    print("\n[5/6] PPO")
    # GAE 在"全部奖励为 1、价值恒为 0、γ=λ=1"时应收敛到剩余步数
    data = [{"reward": 1.0, "value": 0.0, "done": 0.0} for _ in range(5)]
    adv, ret = compute_gae(data, gamma=1.0, lam=1.0)
    rep.check("GAE 数值", np.allclose(adv, [5, 4, 3, 2, 1]),
              "adv = %s" % adv.tolist())

    env = GateRacingEnv(EnvConfig(max_time=4.0))
    p = Policy(PolicyConfig(arch="mlp", seed=0))
    cfg = PPOConfig(rollout_steps=24, epochs=1, batch=8, seed=0)
    traj = rollout(p, env, 24, rng, max_steps_per_ep=20, jitter=0.3)
    rep.check("采样轨迹长度", len(traj) >= 24, "%d 条" % len(traj))
    st = ppo_update(p, traj, cfg)
    rep.check("PPO 更新可跑", all(np.isfinite(v) for k, v in st.items()
                                  if k != "clip_frac"),
              "KL %.4f  价值 %.4f" % (st["kl"], st["v"]))

    m = evaluate_policy(p, env, n_episode=1, seed=0, max_steps=40)
    rep.check("评估产出指标", np.isfinite(m["passed"]) and np.isfinite(m["latency_ms"]),
              "通过 %.1f，延迟 %.2f ms" % (m["passed"], m["latency_ms"]))
    env.close()


def _sec_bc(rep: _Report, rng) -> None:
    print("\n[6/6] 行为克隆")
    from .core.gate_env import EnvConfig as EC
    env = GateRacingEnv(EC(max_time=4.0))
    p = Policy(PolicyConfig(arch="mlp", seed=0))
    scale = norm_scale(env.cfg)
    # 造一批"专家动作 = 常数"的样本，BC 应当能拟合到接近零误差
    samples = [{"depth": rng.integers(0, 255, (48, 64)).astype(np.uint8),
                "vel": rng.normal(size=3).astype(np.float32),
                "act": np.array([1.0, 0.0, 0.5], np.float32)}
               for _ in range(64)]
    hist = train_bc(p, samples, scale, BCConfig(epochs=6, batch=16, seed=0),
                    verbose=False)
    rep.check("BC 损失下降", hist["train"][-1] < hist["train"][0],
              "%.4f → %.4f" % (hist["train"][0], hist["train"][-1]))
    m = bc_metrics(p, samples, scale)
    rep.check("BC 拟合常数动作", m["mse"] < 0.05, "MSE %.5f" % m["mse"])
    rep.check("归一化到 [−1,1]",
              abs(normalise(np.array([99.0, -99.0, 0.0]), scale)).max() <= 1.0,
              "scale = %s" % scale.tolist())
    env.close()


# --------------------------------------------------------------------------- #
def run_selftest() -> int:
    print("=" * 76)
    print("  uav_depth_gate_racing_e2e 自检")
    print("=" * 76)
    rng = np.random.default_rng(0)
    rep = _Report()
    t0 = time.time()
    _sec_nn(rep, rng)
    _sec_env(rep)
    _sec_expert(rep)
    _sec_policy(rep, rng)
    _sec_ppo(rep, rng)
    _sec_bc(rep, rng)
    print("\n" + "=" * 76)
    print("  通过 %d/%d   总耗时 %.1f s" % (rep.ok, rep.n, time.time() - t0))
    if rep.fail:
        print("  未通过：%s" % "、".join(rep.fail))
    print("=" * 76)
    return 0 if rep.ok == rep.n else 1


if __name__ == "__main__":
    raise SystemExit(run_selftest())
