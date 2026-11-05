#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_rl_benchmark 统一入口.

    roslaunch uav_rl_benchmark main.launch                    # 作为 ROS 节点跑对比并播报进度
    python3 main.py --sweep                                   # 跑完整对比矩阵
    python3 main.py --algo sac --arch 64,64 --seeds 0 --steps 100000
    python3 main.py --report                                  # 由 results.csv 生成表格与图表
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
DEFAULT_CONFIG = os.path.join(PKG_ROOT, "config", "benchmark.yaml")


def _parse_arch(s):
    if isinstance(s, (list, tuple)):
        return tuple(int(x) for x in s)
    return tuple(int(x) for x in s.replace("x", ",").split(",") if x.strip())


def _parse_seeds(s):
    if isinstance(s, (list, tuple)):
        return tuple(int(x) for x in s)
    return tuple(int(x) for x in s.replace(" ", "").split(",") if x.strip())


def load_config(path):
    """读取 YAML 配置；没有 PyYAML 或文件不存在时返回 {}（不报错）."""
    try:
        import yaml  # noqa: WPS433
    except Exception:                                       # noqa: BLE001
        return {}
    if not path or not os.path.isfile(path):
        return {}
    try:
        with open(path, encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except Exception:                                       # noqa: BLE001
        return {}


def main():
    ap = argparse.ArgumentParser(description="无人机导航 RL 算法/结构对比实验")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="对比矩阵配置文件")
    ap.add_argument("--sweep", action="store_true", help="跑完整对比矩阵")
    ap.add_argument("--algo", default=None, help="只跑单个算法：ppo / sac / td3")
    ap.add_argument("--arch", default=None, help="网络结构，如 64,64 或 256,256")
    ap.add_argument("--steps", type=int, default=None, help="每个配置的训练步数预算")
    ap.add_argument("--seeds", default=None, help="随机种子，逗号分隔")
    ap.add_argument("--obstacles", type=int, default=None, help="障碍物数量")
    ap.add_argument("--eval-episodes", type=int, default=None, help="最终评估回合数")
    ap.add_argument("--eval-freq", type=int, default=None, help="周期评估间隔（步）")
    ap.add_argument("--out", default=None, help="结果目录")
    ap.add_argument("--report", action="store_true", help="由 results.csv 生成报告与图表")
    ap.add_argument("--node", action="store_true", help="作为 ROS 节点运行（供 launch 使用）")
    args = ap.parse_args()

    # 未在命令行给出的项，用配置文件里的值兜底
    cfg = load_config(args.config)
    m = cfg.get("matrix", {}) or {}
    e = cfg.get("env", {}) or {}
    v = cfg.get("eval", {}) or {}
    r = cfg.get("report", {}) or {}

    steps = args.steps if args.steps is not None else int(m.get("steps", 100000))
    seeds = _parse_seeds(args.seeds) if args.seeds is not None else _parse_seeds(m.get("seeds", [0, 1, 2]))
    obstacles = args.obstacles if args.obstacles is not None else int(e.get("obstacles", 16))
    eval_episodes = args.eval_episodes if args.eval_episodes is not None else int(v.get("episodes", 100))
    # 兜底值必须与 config/benchmark.yaml 的 eval.freq、launch/main.launch 的 eval_freq
    # 默认值、以及 benchmark.py 里 run_one 的默认值保持一致 —— 四处不一样就会出现
    # 「配置写 A、实际按 B 跑」这种查半天的问题。tests 里有回归测试锁定这个值。
    eval_freq = args.eval_freq if args.eval_freq is not None else int(v.get("freq", 20000))
    during = int(v.get("during_episodes", 20))
    out_dir = args.out or r.get("out") or os.path.join(PKG_ROOT, "results")
    archs = [_parse_arch(a) for a in (m.get("archs") or [[64, 64]])]
    algos = tuple(m.get("algos") or ("ppo", "sac", "td3"))

    if args.report:
        from report import build_report  # noqa: WPS433
        build_report(out_dir)
        return 0

    # 作为 ROS 节点运行：命令行 --node，或由 launch 设置了 UAV_BENCH_NODE=1
    if args.node or os.environ.get("UAV_BENCH_NODE") == "1":
        from benchmark_node import main as node_main  # noqa: WPS433
        return node_main()

    if args.sweep:
        import benchmark as bm  # noqa: WPS433
        base = [a for a in archs if a == (64, 64)] or [(64, 64)]
        wide = [a for a in archs if a != (64, 64)]
        bm.run_sweep(algos=algos, archs=tuple(base), steps=steps, seeds=seeds,
                     obstacles=obstacles, out_dir=out_dir,
                     eval_episodes=eval_episodes, eval_freq=eval_freq,
                     eval_episodes_during=during)
        if wide:
            bm.run_sweep(algos=("ppo",), archs=tuple(wide), steps=steps, seeds=seeds,
                         obstacles=obstacles, out_dir=out_dir,
                         eval_episodes=eval_episodes, eval_freq=eval_freq,
                         eval_episodes_during=during)
        return 0

    if args.algo:
        import benchmark as bm  # noqa: WPS433
        arch = _parse_arch(args.arch) if args.arch else (archs[0] if archs else (64, 64))
        for seed in seeds:
            bm.run_one(args.algo, arch=arch, steps=steps, seed=seed,
                       obstacles=obstacles, out_dir=out_dir,
                       eval_episodes=eval_episodes, eval_freq=eval_freq,
                       eval_episodes_during=during)
        return 0

    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
