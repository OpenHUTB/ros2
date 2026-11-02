#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口：采集 / 训练 / 建图 / 规划对比 / 出图 / 自检。

    python main.py --selftest
    python main.py --collect  --out data/seg_dataset.npz
    python main.py --train    --data data/seg_dataset.npz --out data/
    python main.py --evaluate --data data/seg_dataset.npz --model data/
    python main.py --figures  --data data/seg_dataset.npz --model data/ --outdir figs/

``--collect``/``--train`` 需要 mujoco；``--evaluate``/``--figures`` 只需要
numpy + matplotlib，可以在任何解释器里复现已有数据集的结论。
"""
from __future__ import annotations

import argparse
import json
import os
import time

from .core.labels import CLASS_NAMES_CN


def _collect(a) -> int:
    from .core.segdata import DEFAULT_SPECS, EpisodeSpec, collect_dataset

    specs = DEFAULT_SPECS
    if a.quick:
        specs = [EpisodeSpec(0, "urban_block", span=14.0, depth_m=12.0, lanes=4,
                             height=3.2, start_x=-8.0, start_y=-8.0)]
    t0 = time.time()
    ds = collect_dataset(specs, verbose=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    ds.save(a.out)
    print("已保存 %s（%d 集 / %d 帧，%.0f s，%.1f MB）"
          % (a.out, len(ds.episodes), len(ds), time.time() - t0,
             os.path.getsize(a.out) / 1e6))
    return 0


def _train(a) -> int:
    from .core.metrics import iou_per_class
    from .core.segdata import SegDataset
    from .core.segnet import SegNet
    from .core.training import SegTrainConfig, evaluate, train_seg

    ds = SegDataset.load(a.data)
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    net, hist = train_seg(ds, SegTrainConfig(epochs=a.epochs, batch=a.batch))
    net.save(os.path.join(a.out, "segnet.npz"),
             meta={"miou": hist["best_miou"][0]})
    print("训练完成 %.0f s，最优验证 mIoU %.4f" % (time.time() - t0,
                                                  hist["best_miou"][0]))
    m = evaluate(net, ds, list(range(len(ds.episodes))))
    print("全数据集：mIoU %.4f  像素准确率 %.4f  损失 %.4f"
          % (m["miou"], m["pixel_acc"], m["loss"]))
    iou = iou_per_class(m["conf"])
    for i, name in enumerate(CLASS_NAMES_CN):
        if iou[i] == iou[i]:
            print("    %-4s IoU %.4f" % (name, iou[i]))
    if a.bundle:
        import shutil
        shutil.copy(a.data, os.path.join(a.out, "seg_dataset.npz"))
    return 0


def _load(a):
    from .core.segnet import SegNet
    p = os.path.join(a.model, "segnet.npz")
    return SegNet.load(p) if os.path.exists(p) else None


def _build(a):
    """跑一遍建图：真值分割与网络预测各建一张，用于对比。"""
    from .core.segdata import SegDataset, u8_to_depth, DEFAULT_HW
    from .core.semantic_map import CameraModel, SemanticGrid, SemMapConfig
    from .core.training import predict_episode

    ds = SegDataset.load(a.data)
    net = _load(a)
    if net is None:
        raise SystemExit("找不到 %s/segnet.npz，请先 --train" % a.model)
    cam = CameraModel(width=DEFAULT_HW[1], height=DEFAULT_HW[0])
    eps = range(len(ds.episodes)) if a.episodes is None else \
        [int(x) for x in a.episodes.split(",")]

    g_pred = SemanticGrid(SemMapConfig())
    g_truth = SemanticGrid(SemMapConfig())
    for i in eps:
        ep = ds.episodes[i]
        pr = predict_episode(net, ep)
        for k in range(len(ep)):
            pose = (float(ep.pos[k, 0]), float(ep.pos[k, 1]), float(ep.yaw[k]))
            depth = u8_to_depth(ep.depth[k])
            g_pred.integrate(pose, depth, pr["seg"][k], cam)
            g_truth.integrate(pose, depth, ep.seg[k], cam)
    print("网络预测建图：%s" % g_pred.stats())
    print("真值分割建图：%s" % g_truth.stats())
    return g_pred, g_truth, net


def _evaluate(a) -> int:
    from .core.experiment import build_maps, make_scenarios, run_comparison, table
    from .core.metrics import confusion, miou, pixel_acc
    from .core.segdata import SegDataset
    from .core.training import evaluate as eval_seg, predict_episode
    from .core.labels import N_CLASS
    import numpy as np

    ds = SegDataset.load(a.data)
    net = _load(a)
    g_pred, g_truth, _ = _build(a)

    print("\n=== 分割精度 ===")
    m = eval_seg(net, ds, list(range(len(ds.episodes))))
    print("mIoU %.4f  像素准确率 %.4f  损失 %.4f"
          % (m["miou"], m["pixel_acc"], m["loss"]))
    # 网络预测 vs 真值建图里的语义一致性
    conf = confusion(g_pred.semantic()[g_pred.observed()],
                     g_truth.semantic()[g_pred.observed()], N_CLASS)
    print("建图后语义一致：mIoU %.4f  像素准确率 %.4f"
          % (miou(conf), pixel_acc(conf)))

    print("\n=== 路径规划对比 ===")
    maps = build_maps(g_pred, oracle_grid=g_truth)
    sc = make_scenarios(g_truth, n=3, oracle_map=maps["oracle"])
    print("场景 %d 组：%s" % (len(sc), "、".join(s.name for s in sc)))
    rows = run_comparison(g_truth, maps, sc)
    print(table(rows))
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump({"seg": {k: float(v) for k, v in m.items()
                               if np.isscalar(v) and np.isfinite(v)},
                       "rows": rows}, f, ensure_ascii=False, indent=1)
        print("\n指标已写入 %s" % a.out)
    return rows


def _figures(a) -> int:
    from . import plotting
    from .core.experiment import build_maps, make_scenarios, run_comparison
    from .core.segdata import SegDataset
    from .core.training import predict_episode

    plotting.setup_font()
    os.makedirs(a.outdir, exist_ok=True)
    ds = SegDataset.load(a.data)
    net = _load(a)
    ep = ds.episodes[0]
    pr = predict_episode(net, ep)

    plotting.fig_seg_samples(ep, pr, os.path.join(a.outdir, "01_seg_samples.png"))
    plotting.fig_training(net, ds, os.path.join(a.outdir, "02_seg_accuracy.png"))

    g_pred, g_truth, _ = _build(a)
    plotting.fig_maps(g_pred, g_truth, os.path.join(a.outdir, "03_maps.png"))
    maps = build_maps(g_pred, oracle_grid=g_truth)
    sc = make_scenarios(g_truth, n=3, oracle_map=maps["oracle"])
    rows = run_comparison(g_truth, maps, sc)
    plotting.fig_paths(g_truth, maps, sc, rows,
                       os.path.join(a.outdir, "04_paths.png"))
    plotting.fig_metrics(rows, os.path.join(a.outdir, "05_metrics.png"))
    plotting.fig_scene(ep, ds, os.path.join(a.outdir, "06_scene.png"))
    print("配图已写入 %s" % a.outdir)
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="main", description="uav_semantic_mapping_nav：语义建图与语义路径规划")
    ap.add_argument("--selftest", action="store_true", help="自检（不需要 ROS）")
    ap.add_argument("--collect", action="store_true")
    ap.add_argument("--train", action="store_true")
    ap.add_argument("--evaluate", action="store_true")
    ap.add_argument("--figures", action="store_true")
    ap.add_argument("--data", default="data/seg_dataset.npz")
    ap.add_argument("--model", default="data")
    ap.add_argument("--out", default="")
    ap.add_argument("--outdir", default="figs")
    ap.add_argument("--epochs", type=int, default=12)
    ap.add_argument("--batch", type=int, default=4)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--bundle", action="store_true")
    ap.add_argument("--episodes", default=None)
    a = ap.parse_args(argv)

    if a.selftest:
        from .selftest import run_selftest
        return run_selftest()
    if a.collect:
        return _collect(a)
    if a.train:
        return _train(a)
    if a.evaluate:
        return 0 if _evaluate(a) is not None else 1
    if a.figures:
        return 0 if _figures(a) is not None else 1
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
