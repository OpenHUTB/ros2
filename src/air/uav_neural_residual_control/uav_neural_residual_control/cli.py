#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口：采集 → 训练 → 实验 → 出图，一条流水线。

    python main.py --selftest            # 33 项数值自检，不需要 ROS
    python main.py --pipeline            # 全流程（约 15 分钟）
    python main.py --train               # 只用已有数据集重训
    python main.py --eval                # 只用已有模型重跑实验
    python main.py --plot                # 只用已有结果重出图
    python main.py --demo --payload heavy --wind windy
    python main.py --server --payload extra   # 仿真侧（供 ROS 用）
    python main.py --node                # ROS 侧桥接节点
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np


def _paths():
    from . import paths
    return paths


def _default_specs(seeds=(0, 1, 2), trajs=("hover", "helix", "figure8")):
    """训练集工况：3 载荷 × 3 风况 × 3 轨迹（含定点悬停），每个组合 3 条独立风种子。

    悬停必须进训练集：它和机动飞行的状态分布几乎不重叠（速度小、姿态平），
    纯机动数据训出来的网络在悬停点上会给出 0.19 m 的残差（比几何控制器还差）。
    """
    from .core.disturbances import TRAIN_PAYLOADS, TRAIN_WINDS
    out = []
    for p in TRAIN_PAYLOADS:
        for w in TRAIN_WINDS:
            for tj in trajs:
                for s in seeds:
                    out.append((p, w, tj, s))
    return out


# --------------------------------------------------------------------------- #
def cmd_collect(a) -> int:
    from .core import collect_dataset
    out = a.out or _paths().default_dataset()
    specs = _default_specs()
    print("采集 %d 个 episode（3 载荷 × 3 风况 × 3 轨迹 × 3 种子）…" % len(specs))
    t0 = time.time()
    ds = collect_dataset(specs, excite_std=a.excite, verbose=True)
    ds.save(out)
    print("数据集 %d 样本 → %s（%.1fs，%.1f MB）"
          % (len(ds), out, time.time() - t0, os.path.getsize(out) / 1e6))
    return 0


def cmd_train(a) -> int:
    from .core import Dataset, ResidualNet
    from .core.training import TrainConfig, train_residual, evaluate_residual
    dsp = a.dataset or _paths().default_dataset()
    if not os.path.exists(dsp):
        print("找不到数据集 %s，请先运行 --collect" % dsp)
        return 2
    ds = Dataset.load(dsp)
    print("数据集 %d 样本，特征维 %d" % (len(ds), ds.X.shape[1]))
    cfg = TrainConfig(epochs=a.epochs, hidden=tuple(a.hidden), seed=a.seed)
    net, hist = train_residual(ds, cfg)
    m = evaluate_residual(net, ds)
    print("数据集 RMSE=%.4f m/s²  R²=%.4f  偏差=%.4f"
          % (m["res_rmse"], m["res_r2"], m["res_bias"]))
    out = a.out or _paths().default_net()
    net.save(out, meta={"rmse": m["res_rmse"], "r2": m["res_r2"],
                        "samples": len(ds), "epochs": len(hist["train"])})
    print("模型 → %s（%.0f KB）" % (out, os.path.getsize(out) / 1e3))
    hp = os.path.join(os.path.dirname(out), "train_history.json")
    with open(hp, "w", encoding="utf-8") as f:
        json.dump({k: (v if not isinstance(v, list) else v) for k, v in hist.items()}, f)
    print("训练曲线 → %s" % hp)
    ResidualNet.load(out)                       # 读回自检，确保文件可用
    return 0


def cmd_eval(a) -> int:
    from .core import ResidualNet
    from .core.experiment import (DEFAULT_SEEDS, load_results, run_grid, run_once,
                                  save_results, table, table_by_traj)
    from .core.metrics import format_table

    netp = a.net or _paths().default_net()
    if not os.path.exists(netp):
        print("找不到模型 %s，请先运行 --train" % netp)
        return 2
    net = ResidualNet.load(netp)

    grids = [("none", "calm", "hover"), ("none", "calm", "helix")]
    seeds = DEFAULT_SEEDS if not a.quick else (0,)
    print("主实验：4 控制器 × 4 载荷 × 3 风况 × 3 轨迹 × %d 种子 = %d 次运行"
          % (len(seeds), 4 * 4 * 3 * 3 * len(seeds)))
    t0 = time.time()
    recs = run_grid(seeds=seeds, net=net, keep_series=False, verbose=not a.quiet)
    print("\n全部运行完成，用时 %.1f s" % (time.time() - t0))

    # 展示用序列：只保留少数工况的逐时刻数据，避免 JSON 过大
    show = []
    for traj in ("helix", "figure8"):
        for p, w in (("heavy", "windy"), ("extra", "breeze"), ("heavy", "calm")):
            for c in ("pid", "geo", "geo_res", "geo_rls"):
                show.append(run_once(c, p, w, traj, 0, net=net, keep_series=True))
    recs.extend(show)
    for r in show:                              # 主网格里去掉重复工况
        pass

    out = a.out or os.path.join(_paths().results_dir(), "grid.json")
    save_results(recs, out, meta={"net": os.path.basename(netp), "seeds": list(seeds)})

    print("\n" + "=" * 78)
    print("稳态位置 RMSE（m）—— 按载荷 × 风况（轨迹 %s 平均）" % "helix+figure8")
    print("=" * 78)
    print(table([r for r in recs if r.traj in ("helix", "figure8") and r.series == {}],
                metric="rmse"))
    print("\n重载/外推档位（extra = +0.75 kg，训练集中没有）")
    print(table([r for r in recs if r.payload == "extra" and r.series == {}],
                metric="rmse"))
    print("\n按轨迹（heavy + windy）")
    print(table_by_traj([r for r in recs if r.series == {}], "rmse", "heavy", "windy"))
    print("\n控制代价（总推力平方积分，越小越省）")
    print(table([r for r in recs if r.series == {}], metric="energy", unit="N²·s"))
    return 0


def cmd_pipeline(a) -> int:
    print("=" * 78)
    print("步骤 1/4  采集残差数据")
    print("=" * 78)
    rc = cmd_collect(a)
    if rc:
        return rc
    print("\n" + "=" * 78)
    print("步骤 2/4  训练残差网络")
    print("=" * 78)
    rc = cmd_train(a)
    if rc:
        return rc
    print("\n" + "=" * 78)
    print("步骤 3/4  对比实验")
    print("=" * 78)
    rc = cmd_eval(a)
    if rc:
        return rc
    print("\n" + "=" * 78)
    print("步骤 4/4  生成配图")
    print("=" * 78)
    return cmd_plot(a)


def cmd_plot(a) -> int:
    from .plotting import make_all, use_cjk_font
    if not use_cjk_font():
        print("⚠️ 未找到中文字体，图中中文会显示为方块（不影响数值）")
    res = a.results or os.path.join(_paths().results_dir(), "grid.json")
    if not os.path.exists(res):
        print("找不到结果 %s，请先运行 --eval" % res)
        return 2
    outs = make_all(res, a.dataset or _paths().default_dataset(),
                    a.net or _paths().default_net(),
                    a.outdir or _paths().figures_dir(), gif=not a.no_gif)
    print("\n共 %d 张图" % len(outs))
    return 0


def cmd_demo(a) -> int:
    from .core import ResidualNet, make_tracker, ResidualPlant, make_trajectory, run_once
    from .core.disturbances import make_plant
    from .core.metrics import format_table

    netp = a.net or _paths().default_net()
    net = ResidualNet.load(netp) if os.path.exists(netp) else None
    ctrls = [c for c in a.controllers if net is not None or not c.startswith("geo_r")]
    print("工况：载荷 %s（%.2f kg）  风 %s  轨迹 %s  控制器 %s\n"
          % (a.payload, {"none": 0.0, "light": 0.25, "heavy": 0.5,
                         "extra": 0.75}.get(a.payload, 0.0), a.wind, a.traj, ctrls))

    rows = []
    for c in ctrls:
        if c == "geo_rls" and net is None:
            continue
        for seed in (0, 1, 2):
            r = run_once(c, a.payload, a.wind, a.traj, seed, net=net)
            rows.append(r)
        m = np.mean([r.metrics["rmse"] for r in rows if r.controller == c])
        print("  %-8s 三次种子平均 RMSE = %.4f m" % (c, m))

    metrics = ("rmse", "max", "iae", "final_err", "thrust_mean")
    table_rows = []
    for c in ctrls:
        cells = ["%.4f" % np.mean([r.metrics[k] for r in rows if r.controller == c])
                 for k in metrics]
        table_rows.append([c] + cells)
    print()
    print(format_table(table_rows,
                       ["控制器", "RMSE(m)", "最大误差(m)", "IAE",
                        "末段误差(m)", "平均推力(N)"]))

    if a.viewer:
        _viewer(a, net)
    return 0


def _viewer(a, net) -> None:
    """可选：开一个 MuJoCo 原生窗口现场看。"""
    try:
        import mujoco.viewer
    except Exception as e:                       # noqa: BLE001
        print("无法打开 viewer：%s" % e)
        return
    from .core import ResidualNet, ResidualPlant, make_tracker, make_trajectory
    from .core.disturbances import make_plant

    pl = ResidualPlant(plant=make_plant(a.payload, a.wind, seed=a.seed),
                       scene=a.scene)
    tj = make_trajectory(a.traj)
    tk = make_tracker(a.controller, pl, tj, net=net)
    p0, _, _, y0, _ = tj.eval(0.0)
    pl.reset(pos=tuple(p0), yaw=float(y0))
    print("\n已打开 MuJoCo 窗口，关闭窗口退出。")
    with mujoco.viewer.launch_passive(pl.model, pl.data) as v:
        n = int(tj.duration / pl.dt)
        for k in range(n):
            if not v.is_running():
                break
            u = tk.act(k * pl.dt)
            pl.step_with_control(u)
            tk.observe()
            v.sync()
            if k % 5 == 0:
                time.sleep(0.005)


# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="uav_neural_residual_control",
        description="神经残差动力学与学习式轨迹跟踪",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--selftest", action="store_true", help="内置数值自检")
    g.add_argument("--collect", action="store_true", help="采集残差数据集")
    g.add_argument("--train", action="store_true", help="训练残差网络")
    g.add_argument("--eval", action="store_true", help="运行对比实验")
    g.add_argument("--pipeline", action="store_true", help="采集→训练→实验→出图")
    g.add_argument("--plot", action="store_true", help="由结果生成配图")
    g.add_argument("--demo", action="store_true", help="单次运行并打印指标")
    g.add_argument("--server", action="store_true", help="启动仿真服务端（供 ROS 用）")
    g.add_argument("--node", action="store_true", help="启动 ROS2 桥接节点")

    ap.add_argument("--dataset", default=None, help="数据集 .npz 路径")
    ap.add_argument("--net", default=None, help="残差网络 .npz 路径")
    ap.add_argument("--results", default=None, help="实验结果 .json 路径")
    ap.add_argument("--out", default=None)
    ap.add_argument("--outdir", default=None, help="配图输出目录")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--hidden", type=int, nargs="+", default=[64, 64])
    ap.add_argument("--excite", type=float, default=1.2, help="采集时的激励加速度 σ (m/s²)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quick", action="store_true", help="实验只用 1 个种子")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--no-gif", action="store_true")

    ap.add_argument("--payload", default="heavy")
    ap.add_argument("--wind", default="windy")
    ap.add_argument("--traj", default="helix")
    ap.add_argument("--scene", default="open_ground")
    ap.add_argument("--controller", default="geo_rls")
    ap.add_argument("--controllers", nargs="+",
                    default=["pid", "geo", "geo_res", "geo_rls"])
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--ground-effect", type=float, default=0.12)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=41460)
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    a = ap.parse_args(argv)

    if a.selftest:
        from .selftest import run_selftest
        return run_selftest()
    if a.collect:
        return cmd_collect(a)
    if a.train:
        return cmd_train(a)
    if a.eval:
        return cmd_eval(a)
    if a.pipeline:
        return cmd_pipeline(a)
    if a.plot:
        return cmd_plot(a)
    if a.demo:
        return cmd_demo(a)
    if a.server:
        from .sim.plant_server import main as srv_main
        return srv_main(["--host", "0.0.0.0", "--port", str(a.port),
                         "--scene", a.scene, "--seed", str(a.seed),
                         "--payload", a.payload, "--wind", a.wind,
                         "--ground-effect", str(a.ground_effect)]
                        + (["--headless"] if a.headless else []))
    if a.node:
        from .ros.residual_node import main as node_main
        return node_main(["--host", a.host, "--port", str(a.port),
                          "--controller", a.controller, "--traj", a.traj,
                          "--net", a.net or _paths().default_net()])
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
