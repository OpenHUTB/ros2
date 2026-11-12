#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口：采集 / 训练 / 评估 / 出图 / 自检。

    python main.py --selftest
    python main.py --collect  --out data/vio_dataset.npz
    python main.py --train    --data data/vio_dataset.npz --out data/
    python main.py --evaluate --data data/vio_dataset.npz --model data/
    python main.py --figures  --data data/vio_dataset.npz --model data/ --outdir figs/

``--collect``/``--train`` 需要 mujoco（Python ≥3.10）；``--evaluate``/``--figures``
只需要 numpy + matplotlib，可以在任何解释器里复现已有数据集的结论。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

# --------------------------------------------------------------------------- #
def _collect(a) -> int:
    from .core.dataset import (DEFAULT_SPECS, EpisodeSpec, collect_dataset)

    specs = DEFAULT_SPECS
    if a.quick:
        specs = [EpisodeSpec(0, loops=1.0, ax=7.0, ay=5.0, height=1.6,
                             period=24.0, imu_seed=0)]
    t0 = time.time()
    ds = collect_dataset(specs, seed=a.seed, verbose=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    ds.save(a.out)
    print("已保存 %s（%d 集 / %d 帧，%.0f s，%.1f MB）"
          % (a.out, len(ds.episodes), len(ds), time.time() - t0,
             os.path.getsize(a.out) / 1e6))
    return 0


def _train(a) -> int:
    from .core.dataset import VioDataset
    from .core.landmarks import LandmarkDetector
    from .core.network import VioNet
    from .core.obs_model import LearnedObsModel, build_training_data
    from .core.training import TrainConfig, frontend_metrics, predict_episode, train_vio

    ds = VioDataset.load(a.data)
    os.makedirs(a.out, exist_ok=True)

    t0 = time.time()
    net, hist = train_vio(ds, TrainConfig(epochs=a.epochs, chunk=32, batch=4))
    net.save(os.path.join(a.out, "vio_net.npz"), meta={"val": hist["best_val"]})
    print("前端训练完成 %.0f s，最优验证 NLL %.5f"
          % (time.time() - t0, hist["best_val"]))
    for i, ep in enumerate(ds.episodes):
        m = frontend_metrics(predict_episode(net, ep), ep)
        print("  第%d集 dp RMSE %.4f m  dr RMSE %.3f°  标定 %.2f/%.2f"
              % (i, m["dp_rmse"], m["dr_rmse_deg"], m["calib_p"], m["calib_r"]))

    t1 = time.time()
    d = build_training_data(ds, LandmarkDetector())
    om = LearnedObsModel()
    info = om.fit(d["X"], d["Y"], res=d["res"], verbose=True)
    om.save(os.path.join(a.out, "obs_model.npz"))
    print("观测噪声模型训练完成 %.0f s  MSE %.4f  标定 cal_r %.3f cal_b %.3f"
          % (time.time() - t1, info["mse"], info["cal_r"], info["cal_b"]))

    # 顺便把数据集复制一份进包，供离线复现
    if a.bundle:
        import shutil
        shutil.copy(a.data, os.path.join(a.out, "vio_dataset.npz"))
        print("已复制数据集到 %s" % os.path.join(a.out, "vio_dataset.npz"))
    return 0


def _load_models(a):
    from .core.network import VioNet
    from .core.obs_model import LearnedObsModel
    net = om = None
    if a.model:
        p = os.path.join(a.model, "vio_net.npz")
        if os.path.exists(p):
            net = VioNet.load(p)
        p = os.path.join(a.model, "obs_model.npz")
        if os.path.exists(p):
            om = LearnedObsModel.load(p)
    return net, om


def _evaluate(a) -> int:
    from .core.dataset import VioDataset
    from .core.experiment import BACKENDS, run_grid, summarize, table

    ds = VioDataset.load(a.data)
    net, om = _load_models(a)
    if net is None:
        raise SystemExit("找不到前端权重 %s/vio_net.npz，请先 --train" % a.model)
    episodes = tuple(range(len(ds.episodes))) if a.episodes is None \
        else tuple(int(x) for x in a.episodes.split(","))

    print("=== GPS 正常 ===")
    res = run_grid(ds, net, backends=BACKENDS, outages=((0.0, 0.0),),
                   episodes=episodes, obs_model=om, verbose=True,
                   make_map=True)
    print("\n" + table(res))

    out = {"normal": summarize(res, (0.0, 0.0)), "outages": {}}
    if a.outage:
        print("\n=== GPS 中断 ===")
        outs = [(0.0, float(s)) for s in a.outage.split(",")]
        r2 = run_grid(ds, net, backends=BACKENDS, outages=outs,
                      episodes=episodes, obs_model=om, verbose=False,
                      make_map=False)
        for o in outs:
            out["outages"]["%.0f" % o[1]] = summarize(r2, o)
            print("\n中断 %.0f s：" % o[1])
            print(table(r2, o))

    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print("\n指标已写入 %s" % a.out)
    return 0


def _figures(a) -> int:
    from .core.dataset import VioDataset
    from .core.experiment import BACKENDS, run_grid, summarize
    from .core.training import predict_episode
    from .core.world import LoopTrajectory, build_world
    from . import plotting

    plotting.setup_font()
    os.makedirs(a.outdir, exist_ok=True)
    ds = VioDataset.load(a.data)
    net, om = _load_models(a)
    if net is None:
        raise SystemExit("找不到前端权重，请先 --train")
    ep = ds.episodes[0]
    pred = predict_episode(net, ep)

    world = build_world(seed=a.seed)
    tj = LoopTrajectory()
    plotting.fig_world(world, tj.sample(400),
                       os.path.join(a.outdir, "01_world.png"))
    plotting.fig_frontend(pred, ep, os.path.join(a.outdir, "02_frontend.png"))

    # 轨迹与地图图用第 0 集（可视化需要单集）；指标图用全部集，与评估表口径一致
    vis = run_grid(ds, net, backends=BACKENDS, outages=((0.0, 0.0),),
                   episodes=(0,), obs_model=om, verbose=True, make_map=True)
    plotting.fig_trajectories(vis, os.path.join(a.outdir, "03_trajectories.png"))
    plotting.fig_map(vis, os.path.join(a.outdir, "05_map.png"))

    all_eps = tuple(range(len(ds.episodes)))
    full = run_grid(ds, net, backends=BACKENDS, outages=((0.0, 0.0),),
                    episodes=all_eps, obs_model=om, verbose=False,
                    make_map=True) if len(all_eps) > 1 else vis
    plotting.fig_metrics(summarize(full, (0.0, 0.0)),
                         os.path.join(a.outdir, "04_metrics.png"))

    if a.outage:
        outs = [(0.0, float(s)) for s in a.outage.split(",")]
        r2 = run_grid(ds, net, backends=BACKENDS, outages=outs,
                      episodes=all_eps, obs_model=om, verbose=False,
                      make_map=False)
        by = {o[1]: summarize(r2, o) for o in outs}
        by[0.0] = summarize(full, (0.0, 0.0))
        plotting.fig_outage(by, os.path.join(a.outdir, "06_outage.png"))
    print("配图已写入 %s" % a.outdir)
    return 0


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="main", description="uav_neural_vio_slam：神经 VIO + 三后端 SLAM 对比")
    ap.add_argument("--selftest", action="store_true", help="自检（不需要 ROS）")
    ap.add_argument("--collect", action="store_true", help="采集数据集")
    ap.add_argument("--train", action="store_true", help="训练前端与观测模型")
    ap.add_argument("--evaluate", action="store_true", help="四后端对比评估")
    ap.add_argument("--figures", action="store_true", help="生成配图")
    ap.add_argument("--data", default="data/vio_dataset.npz")
    ap.add_argument("--model", default="data")
    ap.add_argument("--out", default="")
    ap.add_argument("--outdir", default="figs")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quick", action="store_true", help="短数据集（冒烟用）")
    ap.add_argument("--bundle", action="store_true", help="把数据集一并放进模型目录")
    ap.add_argument("--epochs", type=int, default=25)
    ap.add_argument("--episodes", default=None, help="要评估的集号，如 0,2,4")
    ap.add_argument("--outage", default="", help="GPS 中断秒数，如 5,15,30")
    a = ap.parse_args(argv)

    if a.selftest:
        from .selftest import run_selftest
        return run_selftest()
    if a.collect:
        return _collect(a)
    if a.train:
        return _train(a)
    if a.evaluate:
        return _evaluate(a)
    if a.figures:
        return _figures(a)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
