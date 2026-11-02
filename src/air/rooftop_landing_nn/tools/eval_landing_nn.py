#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""学习型落点 vs 几何 polylabel 的对比评估 / 消融实验（开发机运行）。

    python tools/eval_landing_nn.py --scenes 80

对同一批场景、**同一份 ROI 内退化点云**（同样的稀疏度 / 离群点 / 噪声），比较三条路线：

    nn            本模块：高度图 -> 小 CNN -> 热力图 argmax（+ 亚格精化）
    geom(hi)      上游 rooftop_landing：RANSAC 平面 -> 连通域 -> 凸包 -> polylabel
                  （上游 launch / README 的默认参数，面向稠密点云）
    geom(tuned)   同一个检测器，按上游 README「点云稀疏时把 cluster_min_points 调小、
                  cluster_cell 调大」放宽参数（避免"拿差参数踩对手"）

评价指标（为什么不只用"与真值点的距离"）
----------------------------------------
细长或环形屋顶的「离边界最远的点」本身就是**一整条中轴**，任何中轴上的点同样安全；
拿"与某个特定点的距离"当唯一指标，会把正确解判成错误解。因此主指标是：

    clearance(p)   落点处的净空 = 该点到屋面边界的距离（理想几何上算，与选点无关）
    success        落在屋面内 **且** 净空 >= 0.6 m（与上游 select_landing_target 的
                   min_radius 安全常数保持一致）

同时仍然记录 ``err_to_gt_m``（与规范落点的距离）作为参考量。

输出：
    results/ablation_density.csv / results/ablation_outlier.csv / results/by_shape.csv
    results/metrics.json
    docs/img/air/rooftop_landing_nn/ablation_density.png
    docs/img/air/rooftop_landing_nn/ablation_outlier.png
    docs/img/air/rooftop_landing_nn/qualitative_samples.png
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(_PKG)))
sys.path.insert(0, os.path.join(_PKG, "scripts"))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_REPO, "src", "air", "rooftop_landing", "scripts"))

import heightmap as hm_mod          # noqa: E402
import landing_cnn as lcnn          # noqa: E402
import scene_dataset as sd          # noqa: E402
import rooftop_perception as rp     # noqa: E402  （上游几何基线）

RESULT_DIR = os.path.join(_PKG, "results")
FIG_DIR = os.path.join(_REPO, "docs", "img", "air", "rooftop_landing_nn")
PENALTY_M = 8.0            # 没检出 / 拒降落时的惩罚误差（等价于任务失败）
MIN_CLEARANCE_M = 0.6      # 与上游 select_landing_target(min_radius=0.6) 一致
METHODS = ("nn", "geom", "geom_tuned")


def crop_roi(points, center, size_m):
    """裁到 ROI：所有路线吃的点云完全一致（避免"谁看得多谁赢"）。"""
    half = 0.5 * size_m
    pts = np.asarray(points, dtype=np.float64)
    sel = ((np.abs(pts[:, 0] - center[0]) <= half) &
           (np.abs(pts[:, 1] - center[1]) <= half))
    return pts[sel]


def run_nn(model, sc, cloud, cells):
    hm = sd.scene_to_input(sc, cloud, cells=cells)
    heat = lcnn.forward(model, hm_mod.heightmap_channels(hm))
    meta = model.get("meta", {})
    p = lcnn.heat_to_point(heat, hm,
                           min_height_m=float(meta.get("min_height_m", 1.0)),
                           rough_max=meta.get("rough_max"))
    if not p["ok"]:
        return None, hm, heat
    return (p["x"], p["y"]), hm, heat


def run_geom(cloud, seed, prefer="highest", tuned=False):
    """上游几何基线。``tuned`` 见模块 docstring 的说明。"""
    rng = np.random.default_rng(seed)
    info = {}
    kw = dict(min_inliers=40, min_area=2.0, cluster_cell=0.5, cluster_min_points=1) \
        if tuned else dict(min_inliers=120, min_area=8.0, cluster_cell=1.0, cluster_min_points=2)
    roofs = rp.detect_rooftops(cloud, min_height_mode="relative", ground_margin=1.0,
                               max_planes=5, rng=rng, info=info, **kw)
    target = rp.select_landing_target(roofs, prefer=prefer, min_radius=MIN_CLEARANCE_M)
    if target is None:
        return None, roofs
    return target["landing_point"], roofs


def _score_point(sc, p):
    err = float(np.hypot(p[0] - sc.gt_xy[0], p[1] - sc.gt_xy[1]))
    clr = sc.clearance(p[0], p[1])
    inside = bool(sc.in_roof(p[0], p[1]))
    ok = 1.0 if (inside and clr >= MIN_CLEARANCE_M) else 0.0
    return err, clr, (1.0 if inside else 0.0), ok


def evaluate_level(model, scenes, cells, density, outlier, noise, seed):
    rng = np.random.default_rng(seed)
    rec = dict((k, {"err": [], "clr": [], "inside": [], "ok": [], "fail": 0}) for k in METHODS)
    fills = []
    for sc in scenes:
        cloud = sd.sample_cloud(sc, rng, density=density, outlier_ratio=outlier,
                                noise_sigma=noise)
        roi = crop_roi(cloud, sc.center, sc.size_m)
        hm = sd.scene_to_input(sc, roi, cells=cells)
        fills.append(float(hm["mask"].mean()))

        p_nn, _hm, _heat = run_nn(model, sc, roi, cells)
        p_geom, _ = run_geom(roi, seed + 7, prefer="highest", tuned=False)
        p_tuned, _ = run_geom(roi, seed + 7, prefer="largest", tuned=True)

        for name, p in (("nn", p_nn), ("geom", p_geom), ("geom_tuned", p_tuned)):
            e = rec[name]
            if p is None:
                e["err"].append(PENALTY_M)
                e["clr"].append(0.0)
                e["inside"].append(0.0)
                e["ok"].append(0.0)
                e["fail"] += 1
            else:
                err, clr, inside, ok = _score_point(sc, p)
                e["err"].append(err)
                e["clr"].append(clr)
                e["inside"].append(inside)
                e["ok"].append(ok)

    res = {"n": len(scenes), "mask_fill": float(np.mean(fills))}
    for name in METHODS:
        e = rec[name]
        res[name + "_success"] = float(np.mean(e["ok"]))
        res[name + "_mean_clearance_m"] = float(np.mean(e["clr"]))
        res[name + "_mean_err_m"] = float(np.mean(e["err"]))
        res[name + "_median_err_m"] = float(np.median(e["err"]))
        res[name + "_in_roof"] = float(np.mean(e["inside"]))
        res[name + "_fail_rate"] = float(e["fail"] / float(len(scenes)))
    return res


def sweep(model, scenes, cells, name, values, fixed, seed):
    rows = []
    for v in values:
        kw = dict(fixed)
        kw[name] = v
        t0 = time.time()
        r = evaluate_level(model, scenes, cells, seed=seed, **kw)
        r[name] = v
        r["seconds"] = round(time.time() - t0, 2)
        rows.append(r)
        print("[%s=%.4f] success nn=%.0f%% geom=%.0f%% tuned=%.0f%% | clearance nn=%.2f m geom=%.2f tuned=%.2f | fill=%.3f"
              % (name, v, r["nn_success"] * 100.0, r["geom_success"] * 100.0,
                 r["geom_tuned_success"] * 100.0, r["nn_mean_clearance_m"],
                 r["geom_mean_clearance_m"], r["geom_tuned_mean_clearance_m"], r["mask_fill"]))
        sys.stdout.flush()
    return rows


def by_shape_table(model, scenes, cells, seed):
    groups = {}
    for sc in scenes:
        groups.setdefault(sc.shape_name, []).append(sc)
    rows = []
    for name in sorted(groups):
        r = evaluate_level(model, groups[name], cells, density=0.35, outlier=0.01,
                           noise=0.03, seed=seed)
        r["shape"] = name
        rows.append(r)
        print("[shape=%-14s n=%2d] success nn=%.0f%% geom=%.0f%% tuned=%.0f%% | clearance nn=%.2f geom=%.2f tuned=%.2f"
              % (name, r["n"], r["nn_success"] * 100.0, r["geom_success"] * 100.0,
                 r["geom_tuned_success"] * 100.0, r["nn_mean_clearance_m"],
                 r["geom_mean_clearance_m"], r["geom_tuned_mean_clearance_m"]))
        sys.stdout.flush()
    return rows


def write_csv(rows, path):
    if not rows:
        return
    keys = list(rows[0].keys())
    if not os.path.isdir(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))
    with open(path, "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=keys)
        wr.writeheader()
        for r in rows:
            wr.writerow(r)


def figure_ablation(rows, xkey, xlabel, title, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("[warn] matplotlib 不可用: %s" % exc)
        return False
    xs = [r[xkey] for r in rows]
    style = {"nn": ("o-", "learned (ours)"),
             "geom": ("s--", "geometric (upstream default)"),
             "geom_tuned": ("^:", "geometric (sparse-tuned)")}
    fig, ax = plt.subplots(1, 2, figsize=(10.0, 3.6))
    for key in METHODS:
        mk, label = style[key]
        ax[0].plot(xs, [r[key + "_success"] * 100.0 for r in rows], mk, label=label)
        ax[1].plot(xs, [r[key + "_mean_clearance_m"] for r in rows], mk, label=label)
    ax[0].set_xlabel(xlabel)
    ax[0].set_ylabel("usable landing spot (%)")
    ax[0].set_title("success: inside roof & clearance >= %.1f m" % MIN_CLEARANCE_M)
    ax[0].set_ylim(-5, 105)
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    ax[1].set_xlabel(xlabel)
    ax[1].set_ylabel("clearance at chosen point (m)")
    ax[1].set_title(title)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    if not os.path.isdir(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def figure_qualitative(model, scenes, cells, path, seed=99):
    meta = model.get("meta", {})
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("[warn] matplotlib 不可用: %s" % exc)
        return False
    rng = np.random.default_rng(seed)
    picks = scenes[:3]
    fig, axes = plt.subplots(2, len(picks), figsize=(4.2 * len(picks), 7.6))
    for k, sc in enumerate(picks):
        cloud = sd.sample_cloud(sc, rng, density=0.25, outlier_ratio=0.01, noise_sigma=0.03)
        roi = crop_roi(cloud, sc.center, sc.size_m)
        hm = sd.scene_to_input(sc, roi, cells=cells)
        heat = lcnn.forward(model, hm_mod.heightmap_channels(hm))
        p_nn = lcnn.heat_to_point(heat, hm,
                                         min_height_m=float(meta.get("min_height_m", 1.0)),
                                         rough_max=meta.get("rough_max"))
        p_g, _ = run_geom(roi, 3, prefer="highest", tuned=False)
        p_t, _ = run_geom(roi, 3, prefer="largest", tuned=True)
        half = 0.5 * hm["size_m"]
        extent = [hm["center"][0] - half, hm["center"][0] + half,
                  hm["center"][1] - half, hm["center"][1] + half]

        ax = axes[0][k]
        hh = np.where(hm["mask"], hm["height"], np.nan)
        cax = ax.imshow(hh, origin="lower", extent=extent, cmap="viridis", vmin=0.0, vmax=1.0)
        ax.plot(sc.gt_xy[0], sc.gt_xy[1], "w*", ms=15, label="reference point")
        if p_nn["ok"]:
            ax.plot(p_nn["x"], p_nn["y"], "o", mfc="none", mec="red", ms=12,
                    label="learned (clr %.2f m)" % sc.clearance(p_nn["x"], p_nn["y"]))
        if p_g is not None:
            ax.plot(p_g[0], p_g[1], "s", mfc="none", mec="cyan", ms=11,
                    label="geom (clr %.2f m)" % sc.clearance(p_g[0], p_g[1]))
        if p_t is not None:
            ax.plot(p_t[0], p_t[1], "^", mfc="none", mec="orange", ms=11,
                    label="geom tuned (clr %.2f m)" % sc.clearance(p_t[0], p_t[1]))
        ax.set_title("height map, 25%% of roof points kept (%s)" % sc.shape_name)
        ax.legend(loc="lower right", fontsize=6)
        plt.colorbar(cax, ax=ax, fraction=0.046)

        ax = axes[1][k]
        cax = ax.imshow(heat[0], origin="lower", extent=extent, cmap="magma", vmin=0.0, vmax=1.0)
        ax.plot(sc.gt_xy[0], sc.gt_xy[1], "w*", ms=15)
        if p_nn["ok"]:
            ax.plot(p_nn["x"], p_nn["y"], "o", mfc="none", mec="lime", ms=12)
        if p_g is not None:
            ax.plot(p_g[0], p_g[1], "s", mfc="none", mec="cyan", ms=11)
        ax.set_title("network heatmap (argmax = landing point)")
        plt.colorbar(cax, ax=ax, fraction=0.046)
    fig.tight_layout()
    if not os.path.isdir(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser(description="learned vs geometric landing-point selection")
    ap.add_argument("--model", default=os.path.join(_PKG, "models", "landing_cnn.npz"))
    ap.add_argument("--scenes", type=int, default=80)
    ap.add_argument("--cells", type=int, default=64)
    ap.add_argument("--seed", type=int, default=4242)
    args = ap.parse_args()

    model = lcnn.load_model(args.model)
    meta = model.get("meta", {})
    model["meta"] = meta
    print("[model] %s" % args.model)
    print("[model] meta: val=%s" % json.dumps(meta.get("val", {}), ensure_ascii=False))

    rng = np.random.default_rng(args.seed)
    scenes = [sd.make_scene(rng) for _ in range(args.scenes)]

    density_rows = sweep(model, scenes, args.cells, "density",
                         [1.0, 0.5, 0.25, 0.125, 0.0625],
                         {"outlier": 0.005, "noise": 0.02}, args.seed + 1)
    outlier_rows = sweep(model, scenes, args.cells, "outlier",
                         [0.0, 0.005, 0.01, 0.02, 0.04],
                         {"density": 0.3, "noise": 0.02}, args.seed + 2)
    shape_rows = by_shape_table(model, scenes, args.cells, args.seed + 3)

    write_csv(density_rows, os.path.join(RESULT_DIR, "ablation_density.csv"))
    write_csv(outlier_rows, os.path.join(RESULT_DIR, "ablation_outlier.csv"))
    write_csv(shape_rows, os.path.join(RESULT_DIR, "by_shape.csv"))
    figure_ablation(density_rows, "density", "point density (fraction of roof points kept)",
                    "clearance vs LiDAR sparsity",
                    os.path.join(FIG_DIR, "ablation_density.png"))
    figure_ablation(outlier_rows, "outlier", "outlier ratio",
                    "clearance vs clutter / outliers",
                    os.path.join(FIG_DIR, "ablation_outlier.png"))
    figure_qualitative(model, scenes, args.cells, os.path.join(FIG_DIR, "qualitative_samples.png"))

    summary = {
        "scenes": args.scenes,
        "metric": {
            "success": "chosen point inside roof AND clearance >= %.1f m" % MIN_CLEARANCE_M,
            "clearance": "distance from chosen point to nearest roof boundary (ideal geometry)",
            "penalty_m": PENALTY_M,
        },
        "model_meta": meta,
        "density_sweep": density_rows,
        "outlier_sweep": outlier_rows,
        "by_shape": shape_rows,
    }
    if not os.path.isdir(RESULT_DIR):
        os.makedirs(RESULT_DIR)
    with open(os.path.join(RESULT_DIR, "metrics.json"), "w") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=2)
    print("[out] results/{ablation_density,ablation_outlier,by_shape}.csv + metrics.json")
    print("[out] docs/img/air/rooftop_landing_nn/{ablation_density,ablation_outlier,qualitative_samples}.png")


if __name__ == "__main__":
    main()
