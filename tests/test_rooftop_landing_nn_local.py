#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rooftop_landing_nn 本地回归测试（无需 ROS / 仿真器 / GPU / torch）。

    python3 tests/test_rooftop_landing_nn_local.py

覆盖：
  * 高度图栅格化：栅格归属、掩码、栅格<->世界坐标往返、参考面估计、格内上表面高度
  * 网络前向的基础算子：conv2d / maxpool2 / 双线性上采样与朴素实现的逐元素一致性
  * 模型文件：仓库里那份 models/landing_cnn.npz 的键名、形状、meta 自洽
  * 落点提取：argmax、亚格精化、以及两条硬约束（最小高度 / 最大粗糙度）的退让顺序
  * 端到端合成场景回归：真实权重 + 6 种屋顶形状 × 2 种点云退化，
    要求落点落在屋面内且净空 >= 0.6 m 的比例不低于阈值

（numpy 前向与 torch 前向的逐元素对齐测试放在 tools/train_landing_cnn.py 训练结束处，
因为开发机才有 torch；本文件保证"没有 torch 的机器"也能完整验证部署链路。）
"""
from __future__ import print_function

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.abspath(os.path.join(_HERE, "..", "src", "air", "rooftop_landing_nn"))
sys.path.insert(0, os.path.join(_PKG, "scripts"))
sys.path.insert(0, os.path.join(_PKG, "tools"))

import heightmap as hm_mod      # noqa: E402
import landing_cnn as lcnn      # noqa: E402
import scene_dataset as sd      # noqa: E402

PASS = [0]
FAIL = []


def check(cond, msg):
    if cond:
        PASS[0] += 1
    else:
        FAIL.append(msg)
        print("  [FAIL] %s" % msg)


def check_close(a, b, tol, msg):
    check(abs(float(a) - float(b)) <= tol, "%s (got %.4f, want %.4f +- %g)" % (msg, a, b, tol))


# --------------------------------------------------------------------------
def test_heightmap():
    print("[1] 高度图栅格化")
    # 8x8 m 的屋顶（z=6） + 地面（z=0），ROI 中心在原点
    xs = np.arange(-4.0, 4.0 + 1e-9, 0.25)
    ys = np.arange(-4.0, 4.0 + 1e-9, 0.25)
    gx, gy = np.meshgrid(xs, ys)
    roof = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, 6.0)], axis=1)
    gxg, gyg = np.meshgrid(np.arange(-8.0, 8.0, 0.5), np.arange(-8.0, 8.0, 0.5))
    ground = np.stack([gxg.ravel(), gyg.ravel(), np.zeros(gxg.size)], axis=1)
    pts = np.vstack([roof, ground]).astype(np.float32)

    hm = hm_mod.points_to_heightmap(pts, (0.0, 0.0), size_m=16.0, cells=64)
    check(hm["cells"] == 64 and hm["mask"].shape == (64, 64), "高度图尺寸")
    check_close(hm["z_ref"], 0.0, 0.05, "参考面应是地面 z=0")
    # 屋顶格子：cell 0.25 m，屋顶覆盖 x,y in [-4,4] -> 栅格 16..48
    inner = hm["height"][33:36, 33:36]
    check(inner.min() > 0.5, "屋顶格高度应接近 1（归一化后 >0.5）")
    # 边角（远离屋顶）应该是地面高度 0 或者没点
    check(hm["height"][0, 0] < 0.05, "ROI 边角的高度应接近 0")
    # 世界坐标 <-> 栅格往返
    x, y = hm_mod.cell_to_world(30, 40, hm)
    rc = hm_mod.world_to_cell(x, y, hm)
    check(rc == (30, 40), "world_to_cell(cell_to_world()) 应回到同一格")
    check(hm_mod.world_to_cell(100.0, 0.0, hm) is None, "ROI 外的点应返回 None")
    # 格内上表面高度：屋顶格应约等于 6 m
    z = hm_mod.cell_surface_z(pts, 32, 32, hm)
    check_close(z, 6.0, 0.15, "格内上表面高度应约等于 6 m")
    # 空输入
    empty = hm_mod.points_to_heightmap(np.zeros((0, 3)), (0.0, 0.0), cells=32)
    check(empty["mask"].sum() == 0, "空点云应得到全 False 掩码")
    check(hm_mod.heightmap_channels(hm).shape == (3, 64, 64), "三通道张量形状")


def test_ops():
    print("[2] 网络基础算子")
    rng = np.random.default_rng(0)
    x = rng.normal(size=(2, 7, 7)).astype(np.float32)
    w = rng.normal(size=(3, 2, 3, 3)).astype(np.float32)
    b = rng.normal(size=(3,)).astype(np.float32)
    got = lcnn.conv2d(x, w, b, pad=1)
    # 朴素参考实现
    xp = np.pad(x, ((0, 0), (1, 1), (1, 1)))
    want = np.zeros((3, 7, 7), dtype=np.float32)
    for co in range(3):
        for i in range(7):
            for j in range(7):
                patch = xp[:, i:i + 3, j:j + 3]
                want[co, i, j] = float((patch * w[co]).sum() + b[co])
    check_close(np.abs(got - want).max(), 0.0, 1e-5, "conv2d 与朴素实现一致")
    pooled = lcnn.maxpool2(x)
    check(pooled.shape == (2, 3, 3), "maxpool2 形状")
    check_close(pooled[0, 0, 0], float(x[0, :2, :2].max()), 0.0, "maxpool2 取值")
    up = lcnn.upsample_bilinear(x, 14, 14)
    check(up.shape == (2, 14, 14), "上采样形状")
    check(np.all(np.isfinite(up)), "上采样结果有限")
    check(up.min() >= x.min() - 1e-5 and up.max() <= x.max() + 1e-5,
          "双线性上采样不会超出输入取值范围")
    # 越界检查
    try:
        lcnn.conv2d(x, w[:, :1], b, pad=1)
        check(False, "通道不匹配应抛异常")
    except ValueError:
        check(True, "")


def test_model_file():
    print("[3] 权重文件")
    path = os.path.join(_PKG, "models", "landing_cnn.npz")
    check(os.path.isfile(path), "models/landing_cnn.npz 存在")
    model = lcnn.load_model(path)
    for name in lcnn.CONV_LAYERS + (lcnn.HEAD_LAYER,):
        check((name + ".w") in model["w"], "权重键 %s.w" % name)
        check((name + ".b") in model["w"], "权重键 %s.b" % name)
    meta = model.get("meta", {})
    check(bool(meta), "meta 非空")
    check(meta.get("cells") == 64, "meta.cells == 64")
    check("channels" in meta, "meta 记录输入通道顺序")
    n_par = sum(int(np.prod(model["w"][k].shape)) for k in model["w"] if k.endswith(".w"))
    check(0 < n_par < 2_000_000, "参数量在合理范围（实际 %d）" % n_par)
    out = lcnn.forward(model, np.zeros((3, 64, 64), dtype=np.float32))
    check(out.shape == (1, 64, 64), "前向输出形状")
    check(0.0 <= out.min() and out.max() <= 1.0, "sigmoid 输出在 [0,1]")


def test_heat_to_point():
    print("[4] 落点提取与硬约束")
    cells = 32
    pts = []
    rng = np.random.default_rng(3)
    for i in range(cells):
        for j in range(cells):
            if 8 <= i < 24 and 8 <= j < 24:                 # 16x16 格的"屋顶"（z=10）
                pts.append((i * 0.5, j * 0.5, 10.0 + rng.normal(0, 0.01)))
    # 必须有地面：否则"参考面"会取到屋顶本身（z_ref=10），高度图整张都是 0，
    # 两条硬约束会因为"无解"而整体退让 —— 那样测的就不是约束本身了。
    for i in range(0, cells):
        for j in range(0, cells):
            if not (8 <= i < 24 and 8 <= j < 24):
                pts.append((i * 0.5, j * 0.5, 0.0))
    pts = np.asarray(pts, dtype=np.float32)
    hm = hm_mod.points_to_heightmap(pts, (8.0, 8.0), size_m=16.0, cells=cells)
    check_close(hm["z_ref"], 0.0, 0.3, "参考面应是地面 z=0")
    check(hm["height"][16, 16] > 0.5, "屋顶格归一化高度应 > 0.5")

    heat = np.zeros((1, cells, cells), dtype=np.float32)
    heat[0, 16, 16] = 1.0
    p = lcnn.heat_to_point(heat, hm, min_height_m=1.0)
    check(p["ok"] and p["row"] == 16 and p["col"] == 16, "argmax 命中注入的峰值")
    check_close(p["x"], hm_mod.cell_to_world(16, 16, hm)[0], 0.3, "落点 x 与格中心一致")

    # 峰值放在地面上：min_height 约束应把它排除
    heat2 = np.zeros((1, cells, cells), dtype=np.float32)
    heat2[0, 2, 2] = 5.0
    heat2[0, 20, 20] = 1.0
    p2 = lcnn.heat_to_point(heat2, hm, min_height_m=1.0)
    check(p2["ok"] and (p2["row"], p2["col"]) == (20, 20),
          "最小高度约束应排除地面上的高分格")

    # 粗糙度约束：给屋顶格一个高粗糙度，应被排除
    hm2 = dict(hm)
    hm2["rough"] = np.array(hm["rough"], copy=True)
    hm2["rough"][15:18, 15:18] = 1.0
    p3 = lcnn.heat_to_point(heat, hm2, min_height_m=1.0, rough_max=0.2)
    check(p3["ok"] and not (15 <= p3["row"] < 18 and 15 <= p3["col"] < 18),
          "粗糙度约束应排除高粗糙格（得到 row=%s col=%s rough=%s）"
          % (p3.get("row"), p3.get("col"),
             None if not hm2["rough"].size else float(hm2["rough"][p3["row"], p3["col"]])
             if p3.get("row", -1) >= 0 else None))

    # 约束全部不满足时仍应有解（退让到掩码）
    empty = {"height": np.zeros((8, 8), dtype=np.float32), "mask": np.ones((8, 8), bool),
             "rough": np.ones((8, 8), dtype=np.float32), "cells": 8, "size_m": 4.0,
             "cell": 0.5, "center": (0.0, 0.0), "z_scale": 1.0, "z_ref": 0.0}
    p4 = lcnn.heat_to_point(np.ones((1, 8, 8), dtype=np.float32), empty,
                            min_height_m=99.0, rough_max=0.0)
    check(p4["ok"], "约束太严时退让到掩码，仍给出结果")


def test_end_to_end():
    print("[5] 端到端合成场景回归（真实权重）")
    model = lcnn.load_model(os.path.join(_PKG, "models", "landing_cnn.npz"))
    meta = model.get("meta", {})
    rng = np.random.default_rng(2026)
    shapes = ["rect", "L", "U", "courtyard", "twin", "rect_obstacle"]
    for density, outlier in ((0.5, 0.0), (0.2, 0.02)):
        ok, total, clearances = 0, 0, []
        for shape in shapes:
            for _ in range(4):
                sc = sd.make_scene(rng, shape=shape)
                cloud = sd.sample_cloud(sc, rng, density=density, outlier_ratio=outlier,
                                        noise_sigma=0.03)
                hm = sd.scene_to_input(sc, cloud)
                heat = lcnn.forward(model, hm_mod.heightmap_channels(hm))
                p = lcnn.heat_to_point(heat, hm,
                                       min_height_m=float(meta.get("min_height_m", 1.0)),
                                       rough_max=meta.get("rough_max"))
                total += 1
                if p["ok"]:
                    clr = sc.clearance(p["x"], p["y"])
                    clearances.append(clr)
                    if sc.in_roof(p["x"], p["y"]) and clr >= 0.6:
                        ok += 1
        rate = float(ok) / max(total, 1)
        print("    density=%.2f outlier=%.3f -> 可用落点 %d/%d (%.0f%%)，平均净空 %.2f m"
              % (density, outlier, ok, total, rate * 100.0,
                 float(np.mean(clearances)) if clearances else 0.0))
        check(rate >= 0.80, "density=%.2f 时可用落点比例应 >= 80%%（实际 %.0f%%）"
              % (density, rate * 100.0))


def main():
    test_heightmap()
    test_ops()
    test_model_file()
    test_heat_to_point()
    test_end_to_end()
    print("=" * 60)
    print("结果：%d/%d 通过" % (PASS[0], PASS[0] + len(FAIL)))
    if FAIL:
        for m in FAIL:
            print("  - %s" % m)
        sys.exit(1)


if __name__ == "__main__":
    main()
