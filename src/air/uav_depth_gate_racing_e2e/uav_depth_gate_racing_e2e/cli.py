#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口：专家 / DAgger / PPO / 评估 / 结构对比 / 出图 / 自检。

    python main.py --selftest
    python main.py --expert   --out data/
    python main.py --dagger   --arch cnn --out data/policy_bc.npz
    python main.py --ppo      --init data/policy_bc.npz --out data/policy_ppo.npz
    python main.py --evaluate --model data/policy_ppo.npz
    python main.py --compare  --out results.json      # 对比实验②
    python main.py --figures  --model data/policy_ppo.npz --outdir figs/

需要 mujoco（Python ≥3.10）：训练与评估都在仿真里闭环跑。
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import time

import numpy as np


def _env(a):
    from .core.gate_env import EnvConfig, GateRacingEnv
    return GateRacingEnv(EnvConfig(scene=a.scene, seed=a.seed,
                                   max_time=a.max_time))


def _expert(a) -> int:
    from .core.expert import expert_metrics, rollout_expert
    env = _env(a)
    t0 = time.time()
    data = rollout_expert(env, n_episode=a.episodes, max_steps=a.max_steps,
                          jitter=a.jitter, seed=a.seed, verbose=True)
    m = expert_metrics(env, n_episode=3, seed=a.seed + 1)
    print("专家成绩：通过 %.1f/%d 门，完成率 %.0f%%，用时 %.2f s，碰撞 %.2f"
          % (m["passed"], len(env.track), 100 * m["completed"],
             m["time"], m["collisions"]))
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "expert.pkl"), "wb") as f:
        pickle.dump(data, f)
    print("示范 %d 条 → %s/expert.pkl（%.0f s）" % (len(data), a.out,
                                                 time.time() - t0))
    env.close()
    return 0


def _dagger(a) -> int:
    from .core.dagger import DaggerConfig, run_dagger
    from .core.policy import Policy, PolicyConfig
    from .core.ppo import evaluate_policy
    import pickle as pk

    env = _env(a)
    policy = Policy(PolicyConfig(arch=a.arch, seed=a.seed,
                                 depth_hw=env.cfg.depth_hw))
    print(policy.summary())
    cfg = DaggerConfig(rounds=a.rounds, expert_episodes=a.episodes,
                       policy_episodes=max(a.episodes // 2, 1),
                       max_steps=a.max_steps, jitter=a.jitter,
                       bc_epochs=a.bc_epochs, seed=a.seed, loss=a.bc_loss,
                       mirror=a.mirror)
    if a.bc_epochs_first > 0:
        cfg.bc_epochs_first = a.bc_epochs_first
    hist = run_dagger(env, policy, cfg, verbose=True)
    m = evaluate_policy(policy, env, n_episode=3, seed=a.seed + 7,
                        deterministic=True, jitter=a.jitter)
    print("BC 后：通过 %.1f/%d，完成率 %.0f%%，用时 %.2f s，碰撞 %.1f，延迟 %.2f ms"
          % (m["passed"], len(env.track), 100 * m["completed"], m["time"],
             m["collisions"], m["latency_ms"]))
    policy.save(a.out, meta={"stage": "dagger", "arch": a.arch, **m})
    if a.hist:
        with open(a.hist, "w", encoding="utf-8") as f:
            json.dump({"dagger": hist, "eval": m}, f, ensure_ascii=False, indent=1)
        print("训练曲线 → %s" % a.hist)
    print("已保存 %s" % a.out)
    env.close()
    return 0


def _ppo(a) -> int:
    from .core.policy import Policy, PolicyConfig
    from .core.ppo import PPOConfig, evaluate_policy, ppo_update, rollout
    from .core.nn import Adam

    env = _env(a)
    policy = Policy.load(a.init) if a.init and os.path.exists(a.init) else \
        Policy(PolicyConfig(arch=a.arch, seed=a.seed, depth_hw=env.cfg.depth_hw))
    if a.log_std_init is not None:
        # 从 BC 权重微调时要把探索噪声压小：log_std=-0.5 对应 σ=0.6（归一化），
        # 折算成物理量是 2.4 m/s 的抖动，足够把刚学好的策略一出门就打散。
        policy.log_std[:] = float(a.log_std_init)
    print(policy.summary())
    cfg = PPOConfig(rollout_steps=a.rollout_steps, epochs=a.ppo_epochs,
                    batch=a.batch, lr=a.lr, seed=a.seed)
    opt = Adam(policy.layers, lr=cfg.lr, clip=cfg.grad_clip)
    rng = np.random.default_rng(a.seed)
    t0 = time.time()
    best = -1e18
    hist = {"reward": [], "pi": [], "v": [], "kl": [], "clip_frac": [],
            "passed": [], "completed": []}
    for it in range(a.iterations):
        data = rollout(policy, env, cfg.rollout_steps, rng,
                       max_steps_per_ep=a.max_steps, jitter=a.jitter)
        st = ppo_update(policy, data, cfg, opt=opt)
        ret = float(np.sum([d["reward"] for d in data]))
        eps = [d["info"] for d in data if "info" in d]
        hist["reward"].append(ret)
        hist["passed"].append(float(np.mean([e["passed"] for e in eps]))
                              if eps else float("nan"))
        hist["completed"].append(float(np.mean([e["completed"] for e in eps]))
                                 if eps else float("nan"))
        for k in ("pi", "v", "kl", "clip_frac"):
            hist[k].append(float(st[k]))
        if it % 2 == 0 or it == a.iterations - 1:
            print("  PPO %2d  回报 %8.1f  策略损失 %7.3f  价值 %.4f  KL %.4f  "
                  "裁剪比 %.2f  (%.0f s)"
                  % (it, ret, st["pi"], st["v"], st["kl"], st["clip_frac"],
                     time.time() - t0))
        if ret > best:
            best = ret
            policy.save(a.out, meta={"stage": "ppo", "iter": it, "reward": ret})
    m = evaluate_policy(policy, env, n_episode=5, seed=a.seed + 11,
                        deterministic=True, jitter=a.jitter)
    print("PPO 后：通过 %.1f/%d，完成率 %.0f%%，用时 %.2f s，碰撞 %.1f，延迟 %.2f ms"
          % (m["passed"], len(env.track), 100 * m["completed"], m["time"],
             m["collisions"], m["latency_ms"]))
    if a.hist:
        with open(a.hist, "w", encoding="utf-8") as f:
            json.dump({"ppo": hist, "eval": m}, f, ensure_ascii=False, indent=1)
        print("训练曲线 → %s" % a.hist)
    env.close()
    return 0


def _evaluate(a) -> int:
    from .core.expert import expert_metrics
    from .core.policy import Policy
    from .core.ppo import evaluate_policy

    env = _env(a)
    out = {}
    ex = expert_metrics(env, n_episode=a.episodes, seed=a.seed)
    print("专家       ：通过 %.1f/%d  完成率 %.0f%%  用时 %.2f s  碰撞 %.2f"
          % (ex["passed"], len(env.track), 100 * ex["completed"], ex["time"],
             ex["collisions"]))
    out["expert"] = ex
    if a.model and os.path.exists(a.model):
        p = Policy.load(a.model)
        m = evaluate_policy(p, env, n_episode=a.episodes, seed=a.seed,
                            deterministic=True, jitter=a.jitter)
        print("%-8s：通过 %.1f/%d  完成率 %.0f%%  用时 %.2f s  碰撞 %.2f  延迟 %.2f ms"
              % (p.cfg.arch, m["passed"], len(env.track), 100 * m["completed"],
                 m["time"], m["collisions"], m["latency_ms"]))
        out[p.cfg.arch] = m
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print("已写入 %s" % a.out)
    env.close()
    return 0


def _compare(a) -> int:
    """对比实验②：同一 PPO、不同网络结构。

    **示范数据只采一次**，四个结构在同一份数据、同一套超参、同一个随机种子上
    各训一遍。每个结构各采一遍数据的话，比的是"数据 + 结构"而不是结构。
    """
    from .core.bc import BCConfig, norm_scale, train_bc
    from .core.expert import rollout_expert
    from .core.policy import ARCHS, Policy, PolicyConfig
    from .core.ppo import PPOConfig, evaluate_policy, ppo_update, rollout
    from .core.nn import Adam

    env = _env(a)
    scale = norm_scale(env.cfg)
    print("采集一次专家示范（%d 轮）…" % a.episodes)
    data = rollout_expert(env, n_episode=a.episodes, max_steps=a.max_steps,
                          jitter=a.jitter, seed=a.seed)
    print("  %d 条" % len(data))

    rows = []
    print("%-6s %8s %8s %9s %9s %9s %9s"
          % ("结构", "参数量", "延迟ms", "BC通过", "BC擦框", "PPO通过", "PPO擦框"))
    for arch in ARCHS:
        t0 = time.time()
        policy = Policy(PolicyConfig(arch=arch, seed=a.seed,
                                     depth_hw=env.cfg.depth_hw))
        train_bc(policy, data, scale,
                 BCConfig(epochs=a.bc_epochs, batch=32, lr=a.lr * 3,
                          seed=a.seed, loss=a.bc_loss, mirror=a.mirror),
                 verbose=False)
        bc = evaluate_policy(policy, env, n_episode=a.eval_episodes,
                             seed=a.seed + 3, jitter=a.jitter,
                             max_steps=a.max_steps)
        if a.log_std_init is not None:
            policy.log_std[:] = float(a.log_std_init)
        pcfg = PPOConfig(rollout_steps=a.rollout_steps, epochs=a.ppo_epochs,
                         batch=a.batch, lr=a.lr, seed=a.seed)
        opt = Adam(policy.layers, lr=pcfg.lr, clip=pcfg.grad_clip)
        rng = np.random.default_rng(a.seed)
        for _ in range(a.iterations):
            traj = rollout(policy, env, pcfg.rollout_steps, rng,
                           max_steps_per_ep=a.max_steps, jitter=a.jitter)
            ppo_update(policy, traj, pcfg, opt=opt)
        pp = evaluate_policy(policy, env, n_episode=a.eval_episodes,
                             seed=a.seed + 5, jitter=a.jitter,
                             max_steps=a.max_steps)
        rows.append({"arch": arch, "params": policy.n_params(),
                     "bc_completed": bc["completed"], "bc_passed": bc["passed"],
                     "bc_collisions": bc["collisions"],
                     "ppo_completed": pp["completed"], "ppo_passed": pp["passed"],
                     "ppo_collisions": pp["collisions"],
                     "latency_ms": pp["latency_ms"],
                     "seconds": time.time() - t0})
        print("%-6s %8d %8.2f %9.2f %9.2f %9.2f %9.2f"
              % (arch, rows[-1]["params"], pp["latency_ms"], bc["passed"],
                 bc["collisions"], pp["passed"], pp["collisions"]))
        if arch == "cnn":
            policy.save(a.model_out, meta={"stage": "compare"})
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(rows, f, ensure_ascii=False, indent=1)
        print("已写入 %s" % a.out)
    env.close()
    return 0


def _figures(a) -> int:
    from . import plotting
    from .core.expert import expert_metrics
    from .core.policy import Policy
    from .core.ppo import evaluate_policy

    plotting.setup_font()
    os.makedirs(a.outdir, exist_ok=True)
    env = _env(a)
    plotting.fig_track(env, os.path.join(a.outdir, "01_track.png"))
    plotting.fig_expert_rollout(env, os.path.join(a.outdir, "02_expert.png"))
    if a.model and os.path.exists(a.model):
        p = Policy.load(a.model)
        plotting.fig_policy_rollout(env, p, os.path.join(a.outdir, "03_policy.png"))
    if a.hist:
        with open(a.hist, encoding="utf-8") as f:
            plotting.fig_training(json.load(f),
                                  os.path.join(a.outdir, "04_training.png"))
    if a.compare:
        with open(a.compare, encoding="utf-8") as f:
            plotting.fig_compare(json.load(f),
                                 os.path.join(a.outdir, "05_compare.png"))
    plotting.fig_latency(os.path.join(a.outdir, "06_latency.png"))
    print("配图已写入 %s" % a.outdir)
    env.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="main", description="uav_depth_gate_racing_e2e：深度图端到端穿越门竞速")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--expert", action="store_true")
    ap.add_argument("--dagger", action="store_true")
    ap.add_argument("--ppo", action="store_true")
    ap.add_argument("--evaluate", action="store_true")
    ap.add_argument("--compare-structures", dest="compare_structures",
                    action="store_true", help="对比实验②：四种网络结构")
    ap.add_argument("--figures", action="store_true")
    ap.add_argument("--arch", default="cnn", choices=["mlp", "cnn", "gru", "attn"])
    ap.add_argument("--scene", default="gate_track")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--eval-episodes", dest="eval_episodes", type=int, default=5,
                    help="每次评测跑多少轮")
    ap.add_argument("--max-steps", type=int, default=600)
    ap.add_argument("--max-time", type=float, default=30.0)
    ap.add_argument("--jitter", type=float, default=0.6)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--bc-epochs", type=int, default=8)
    ap.add_argument("--bc-epochs-first", dest="bc_epochs_first", type=int,
                    default=0, help="第 0 轮（纯专家数据）的训练轮数，0=用 --bc-epochs")
    ap.add_argument("--iterations", type=int, default=6)
    ap.add_argument("--ppo-epochs", type=int, default=4)
    ap.add_argument("--rollout-steps", type=int, default=1024)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--log-std-init", dest="log_std_init", type=float, default=None,
                    help="覆盖策略的初始 log σ；从 BC 微调时建议 -1.5（σ≈0.22）")
    ap.add_argument("--bc-loss", dest="bc_loss", default="mse",
                    choices=["mse", "huber"],
                    help="BC 损失；Huber 会把横向输出压向中位数")
    ap.add_argument("--no-mirror", dest="mirror", action="store_false",
                    default=True, help="关掉深度图水平镜像增广（消融用）")
    ap.add_argument("--init", default="")
    ap.add_argument("--model", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--outdir", default="figs")
    ap.add_argument("--model-out", default="data/policy_cnn.npz")
    ap.add_argument("--hist", default="", help="训练曲线 JSON（给 --figures 用）")
    ap.add_argument("--compare", default="",
                    help="--compare-structures 写出的 JSON（给 --figures 用）")
    a = ap.parse_args(argv)

    if a.selftest:
        from .selftest import run_selftest
        return run_selftest()
    if a.expert:
        return _expert(a)
    if a.dagger:
        return _dagger(a)
    if a.ppo:
        return _ppo(a)
    if a.evaluate:
        return _evaluate(a)
    if a.compare_structures:
        return _compare(a)
    if a.figures:
        return _figures(a)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
