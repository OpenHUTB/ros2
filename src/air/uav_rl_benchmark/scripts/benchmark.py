#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比实验核心：跑一组配置（算法 × 结构 × 种子），评估并写入 results.csv.

评估协议（对所有算法完全一致，保证可比）
----------------------------------------
* 在**同一批随机任务**上评估：固定 `seed`，`episodes=100`，障碍数一致；
* 全部用**确定性动作**（``deterministic=True``，即取策略均值，不采样）；
* 指标除成功率外，还包括**路径效率**与**动作平滑度** —— 只比成功率会漏掉
  "能到但乱飞"和"能到但抖得厉害"这两类实际不可用的情况。

指标定义
--------
* ``path_efficiency`` = 起点到目标的直线距离 / 实际飞行路程（0~1，越接近 1 越直）
* ``action_smoothness`` = 相邻两步动作的 L2 距离均值（越小越平滑）
"""
from __future__ import annotations

import csv
import json
import os
import time

import numpy as np

from algos import ALGOS, DEFAULT_ARCH, arch_str, make_model
from nav_env import load_nav_env, make_vec_env

RESULT_FIELDS = [
    "algo", "arch", "seed", "steps", "obstacles", "eval_episodes",
    "train_time_s", "train_fps",
    "success_rate", "crash_rate", "timeout_rate",
    "mean_steps", "mean_reward", "mean_path_len", "path_efficiency",
    "action_smoothness",
]


def _make_eval_callback_class():
    """惰性构造 NavEvalCallback（需要 stable_baselines3 才能定义基类）."""
    from stable_baselines3.common.callbacks import BaseCallback  # noqa: WPS433

    class NavEvalCallback(BaseCallback):
        """训练过程中的周期评估：记录**成功率曲线**，并按成功率保存最优模型.

        不用 SB3 自带的 EvalCallback，是因为它只记录"回合回报"——回报里混着路径长度、
        平滑惩罚、时间惩罚，读不出"到底成没成功"。这里直接跑固定任务集算成功率。
        """

        def __init__(self, NavEnv, obstacles=16, eval_freq=20000, n_episodes=20,
                     seed=50000, run_dir=".", verbose=True):
            super().__init__(verbose=0)
            self.NavEnv = NavEnv
            self.obstacles = obstacles
            self.eval_freq = max(1, int(eval_freq))
            self.n_episodes = n_episodes
            self.seed = seed
            self.run_dir = run_dir
            self.verbose = verbose
            self.history = []
            self.best_success = -1.0
            self._next = 0

        def _on_step(self) -> bool:
            if self.num_timesteps < self._next:
                return True
            self._next = self.num_timesteps + self.eval_freq
            m = evaluate(self.model, self.NavEnv, episodes=self.n_episodes,
                         obstacles=self.obstacles, seed=self.seed)
            m["timesteps"] = int(self.num_timesteps)
            self.history.append(m)
            with open(os.path.join(self.run_dir, "eval_log.json"), "w",
                      encoding="utf-8") as fh:
                json.dump(self.history, fh, ensure_ascii=False, indent=2)
            if m["success_rate"] > self.best_success:
                self.best_success = m["success_rate"]
                self.model.save(os.path.join(self.run_dir, "best_model"))
            if self.verbose:
                print("      @%6d 步  成功率 %.0f%%（最优 %.0f%%）"
                      % (self.num_timesteps, m["success_rate"] * 100,
                         self.best_success * 100), flush=True)
            return True

    return NavEvalCallback


def evaluate(model, NavEnv, episodes=100, obstacles=16, seed=12345):
    """在固定任务集上评估策略，返回指标字典."""
    env = NavEnv(num_obstacles=obstacles, seed=seed)
    succ = crash = timeout = 0
    steps_l, rew_l, path_l, eff_l, smooth_l = [], [], [], [], []

    for _ in range(episodes):
        obs, _ = env.reset()
        straight = float(np.linalg.norm(env.goal - env.pos))
        pos_prev = env.pos.copy()
        path = 0.0
        ep_r, n = 0.0, 0
        prev_a = None
        smooths = []
        info = {}
        done = False
        while not done:
            act, _ = model.predict(obs, deterministic=True)
            obs, r, te, tr, info = env.step(act)
            ep_r += r
            n += 1
            path += float(np.linalg.norm(env.pos - pos_prev))
            pos_prev = env.pos.copy()
            if prev_a is not None:
                smooths.append(float(np.linalg.norm(np.asarray(act) - prev_a)))
            prev_a = np.asarray(act)
            done = te or tr

        steps_l.append(n)
        rew_l.append(ep_r)
        path_l.append(path)
        # 路径效率 = 直线距离 / 实际路程。注意：路程≈0（原地悬停）时不能算成"满分"——
        # 没动就没有效率可言，记 0 分。
        if straight > 1e-6 and path > 1e-3:
            eff_l.append(min(1.0, straight / path))
        else:
            eff_l.append(0.0)
        if smooths:
            smooth_l.append(float(np.mean(smooths)))
        if info.get("success"):
            succ += 1
        elif info.get("crash_reason"):
            crash += 1
        else:
            timeout += 1

    env.close()
    n_ep = float(episodes)
    return {
        "success_rate": succ / n_ep,
        "crash_rate": crash / n_ep,
        "timeout_rate": timeout / n_ep,
        "mean_steps": float(np.mean(steps_l)),
        "mean_reward": float(np.mean(rew_l)),
        "mean_path_len": float(np.mean(path_l)),
        "path_efficiency": float(np.mean(eff_l)) if eff_l else 0.0,
        "action_smoothness": float(np.mean(smooth_l)) if smooth_l else 0.0,
    }


def run_one(algo, arch=DEFAULT_ARCH, steps=100000, seed=0,
            obstacles=16, eval_episodes=100, out_dir="results",
            eval_freq=20000, eval_episodes_during=20, verbose=True):
    """跑一组配置：训练（周期评估记曲线）-> 最终评估 -> 追加一行到 results.csv."""
    NavEnv, _ = load_nav_env()
    tag = "%s_%s_seed%d" % (algo, arch_str(arch), seed)
    run_dir = os.path.join(out_dir, "runs", tag)
    os.makedirs(run_dir, exist_ok=True)

    env = make_vec_env(NavEnv, obstacles=obstacles, seed=seed)
    model = make_model(algo, env, arch=arch, seed=seed)

    # 用自己的回调而不是 SB3 的 EvalCallback：后者只记录"回合回报"，
    # 而我们要的是**成功率曲线**（回报里混着路径长度、平滑惩罚等，读不出成功与否）
    NavEvalCallback = _make_eval_callback_class()
    callback = NavEvalCallback(
        NavEnv, obstacles=obstacles, eval_freq=eval_freq,
        n_episodes=eval_episodes_during, seed=seed + 50000,
        run_dir=run_dir, verbose=verbose)

    t0 = time.time()
    model.learn(total_timesteps=steps, callback=callback)
    train_time = time.time() - t0

    metrics = evaluate(model, NavEnv, episodes=eval_episodes,
                       obstacles=obstacles, seed=12345)
    row = {
        "algo": algo, "arch": arch_str(arch), "seed": seed, "steps": steps,
        "obstacles": obstacles, "eval_episodes": eval_episodes,
        "train_time_s": round(train_time, 1),
        "train_fps": round(steps / max(train_time, 1e-9), 1),
    }
    row.update({k: round(v, 4) for k, v in metrics.items()})

    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "results.csv")
    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=RESULT_FIELDS)
        if write_header:
            w.writeheader()
        w.writerow(row)

    with open(os.path.join(run_dir, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(row, fh, ensure_ascii=False, indent=2)

    if verbose:
        print("  [%s] 成功率 %.0f%%  超时 %.0f%%  步数 %.1f  路径效率 %.2f  平滑度 %.3f  用时 %.0f s"
              % (tag, row["success_rate"] * 100, row["timeout_rate"] * 100,
                 row["mean_steps"], row["path_efficiency"],
                 row["action_smoothness"], row["train_time_s"]), flush=True)
    return row


def run_sweep(algos=ALGOS, archs=(DEFAULT_ARCH,), steps=100000, seeds=(0, 1, 2),
              obstacles=16, out_dir="results", eval_episodes=100,
              eval_freq=20000, eval_episodes_during=20):
    """跑完整对比矩阵；已存在的结果会跳过（可断点续跑）.

    ``eval_freq`` / ``eval_episodes_during`` 必须在这里接收并透传给 ``run_one`` ——
    否则调用方传了会被 TypeError 顶掉，或者用了 run_one 的默认值而**让配置文件失效**
    （曾经就出过这个问题：配置里写 25000，实际按默认 20000 跑）。
    """
    done = set()
    csv_path = os.path.join(out_dir, "results.csv")
    if os.path.exists(csv_path):
        with open(csv_path, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                done.add((r["algo"], r["arch"], int(r["seed"])))

    total = len(algos) * len(archs) * len(seeds)
    print("共 %d 组配置（%d 算法 × %d 结构 × %d 种子），已完成 %d 组；"
          "eval_freq=%d eval_episodes_during=%d"
          % (total, len(algos), len(archs), len(seeds), len(done),
             eval_freq, eval_episodes_during), flush=True)
    i = 0
    for algo in algos:
        for arch in archs:
            for seed in seeds:
                i += 1
                key = (algo, arch_str(arch), seed)
                if key in done:
                    print("  [%d/%d] 跳过已完成 %s" % (i, total, key), flush=True)
                    continue
                print("  [%d/%d] 开始 %s" % (i, total, key), flush=True)
                try:
                    run_one(algo, arch=arch, steps=steps, seed=seed,
                            obstacles=obstacles, out_dir=out_dir,
                            eval_episodes=eval_episodes,
                            eval_freq=eval_freq,
                            eval_episodes_during=eval_episodes_during)
                except Exception as exc:                    # noqa: BLE001
                    print("     ! 失败：%s" % exc, flush=True)
    print("全部完成，结果：%s" % csv_path, flush=True)
    return csv_path
