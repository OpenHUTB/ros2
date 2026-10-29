#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练：PPO 策略学会沿固定航线巡航（并避开随机摆放的障碍）.

训练用 stable-baselines3（需要 torch），**部署不需要** —— 训练结束后把 MLP 权重导出成
.npz，ROS 节点用 policy.MlpPolicy 做纯 numpy 前向即可。

评价指标（与"点到点导航"不同，这里看的是循迹质量）：
    完成一圈率       —— 走完一整圈的比例（主指标）
    平均横向偏差     —— 离航线多远（越小越贴合）
    平均完成度       —— 航线被访问过的弧长占比（未完成一圈时也有区分度）
    碰撞率 / 平均用时
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import argparse
import os

import numpy as np

import policy as P
from route import make_route
from route_env import RouteEnv


def make_env(seed: int = 0, **kw):
    return RouteEnv(seed=seed, **kw)


def _mk_env_factory(seed: int, env_kw: dict):
    """返回一个无参工厂（DummyVecEnv 要求），内部固定该 env 的种子。"""
    kw = dict(env_kw)

    def _f():
        return RouteEnv(seed=seed, **kw)

    return _f


class CurriculumCallback(object):
    """课程学习：压线障碍比例从 start 逐步升到 final。

    为什么需要：一开始就让 35% 的障碍压在航线上，策略几乎不可能靠随机探索
    发现「绕过去再回到航线」这种长序列动作，成功率长期卡在 60% 上下。
    先给简单的（障碍少压线），让它先学会稳定循迹，再逐步加大避障难度。

    这是 SB3 的 BaseCallback 子类；写成独立类而非闭包，便于单测。
    """

    def __init__(self, start: float = 0.05, final: float = 0.35,
                 total_steps: int = 500000, warmup_ratio: float = 0.6):
        self.start = float(start)
        self.final = float(final)
        self.total = int(max(total_steps, 1))
        self.warmup = float(max(1e-6, warmup_ratio))
        self._last = None

    def frac(self, num_timesteps: int) -> float:
        """按训练进度给出当前的 on_route_frac（纯函数，便于单测）。"""
        p = min(1.0, float(num_timesteps) / self.total / self.warmup)
        return self.start + (self.final - self.start) * p

    def __call__(self, vec_env):
        """返回一个 SB3 回调（延迟导入 sb3，保证没有 sb3 时模块仍可 import）。"""
        from stable_baselines3.common.callbacks import BaseCallback

        outer = self

        class _Cb(BaseCallback):
            def _on_step(self) -> bool:
                f = outer.frac(self.num_timesteps)
                for env in getattr(self.training_env, "envs", []):
                    if hasattr(env, "on_route_frac"):
                        env.on_route_frac = f
                if outer._last is None or abs(f - outer._last) > 0.02:
                    outer._last = f
                    if self.verbose:
                        print("    [课程] on_route_frac -> %.2f" % f, flush=True)
                return True

        return _Cb()


def evaluate(act_fn, episodes: int = 20, seed: int = 1000, **env_kw) -> dict:
    """用给定动作函数（obs -> action）评估。"""
    env = make_env(seed=seed, **env_kw)
    ok = crash = timeout = 0
    crosses, comps, steps_all = [], [], []
    for ep in range(int(episodes)):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        steps = 0
        info = {}
        crosses_ep = []
        while not done:
            a = act_fn(obs)
            obs, _r, term, trunc, info = env.step(a)
            _, _s, _, cross = env.route.project(env.pos[:2])
            crosses_ep.append(abs(cross))
            steps += 1
            done = bool(term or trunc)
        steps_all.append(steps)
        crosses.append(float(np.mean(crosses_ep)))
        comps.append(env.route.completion(env.visited_s))
        if info.get("success"):
            ok += 1
        elif info.get("crash_reason"):
            crash += 1
        else:
            timeout += 1
    n = float(max(episodes, 1))
    return {"episodes": int(episodes), "lap": ok, "crash": crash, "timeout": timeout,
            "lap_rate": ok / n, "crash_rate": crash / n, "timeout_rate": timeout / n,
            "mean_cross": float(np.mean(crosses)),
            "mean_completion": float(np.mean(comps)),
            "mean_steps": float(np.mean(steps_all)),
            "route_length": float(env.route.length)}


def train(steps: int = 200000, seed: int = 0, out: str = "models/route_ppo_s0.npz",
          eval_episodes: int = 20, verbose: int = 1, **env_kw) -> dict:
    """训练一个 PPO 策略并导出 .npz；返回评估指标。"""
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv
    except Exception as exc:                              # noqa: BLE001
        raise RuntimeError(
            "训练需要 stable-baselines3 与 torch：\n"
            "    pip install stable-baselines3 torch\n原始错误：%s" % exc)

    # 4 个并行环境：PPO 每轮能拿到更多样的经验，同种子下方差明显更小
    n_envs = int(env_kw.pop("n_envs", 4))
    print("  训练步数 %d  种子 %d  航线 %s  并行环境 %d"
          % (steps, seed, env_kw.get("route_name", "rect"), n_envs))
    venv = DummyVecEnv([_mk_env_factory(seed + 17 * i, env_kw)
                        for i in range(max(1, n_envs))])
    model = PPO(
        "MlpPolicy", venv, seed=seed, verbose=verbose,
        n_steps=1024, batch_size=256, learning_rate=3e-4, gamma=0.99,
        gae_lambda=0.95, ent_coef=0.005, clip_range=0.2,
        policy_kwargs=dict(net_arch=[128, 128]),
    )
    cur = CurriculumCallback(start=0.05, final=float(env_kw.get('on_route_frac', 0.35)),
                             total_steps=int(steps), warmup_ratio=0.6)
    model.learn(total_timesteps=int(steps), callback=cur(venv))

    # 导出纯 numpy 权重
    d = P.extract_sb3_weights(model)
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    np.savez_compressed(out, **d)
    print("  权重已导出:", out)

    pol = P.MlpPolicy(d)
    res = evaluate(lambda o: pol.forward(o), episodes=eval_episodes, seed=seed + 1000, **env_kw)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PPO 训练：无人机固定航线巡航")
    ap.add_argument("--steps", type=int, default=200000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="models/route_ppo_s0.npz")
    ap.add_argument("--eval-episodes", type=int, default=20)
    ap.add_argument("--route", default="rect", choices=["rect", "figure8"])
    ap.add_argument("--obstacles", type=int, default=14)
    ap.add_argument("--max-steps", type=int, default=600)
    args = ap.parse_args(argv)

    print("=" * 64)
    print("  PPO：无人机固定路线巡航")
    print("=" * 64)
    res = train(args.steps, args.seed, args.out, args.eval_episodes,
                route_name=args.route, num_obstacles=args.obstacles,
                max_steps=args.max_steps)
    print()
    print("  " + "=" * 56)
    print("  %-16s %s" % ("完成一圈率", "%.0f%%" % (res["lap_rate"] * 100)))
    print("  %-16s %s" % ("碰撞率", "%.0f%%" % (res["crash_rate"] * 100)))
    print("  %-16s %s" % ("超时率", "%.0f%%" % (res["timeout_rate"] * 100)))
    print("  %-16s %.3f m（航线总长 %.1f m）" % ("平均横向偏差", res["mean_cross"], res["route_length"]))
    print("  %-16s %.1f%%" % ("平均完成度", res["mean_completion"] * 100))
    print("  %-16s %.0f 步" % ("平均用时", res["mean_steps"]))
    print("  " + "=" * 56)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
