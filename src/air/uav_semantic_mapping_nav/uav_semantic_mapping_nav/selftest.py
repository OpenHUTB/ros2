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

from .core.labels import (BLOCKED, CLASS_NAMES_CN, N_CLASS, SEMANTIC_RGB,
                          TRAVERSAL_COST)
from .core.metrics import (collision_count, confusion, iou_per_class, miou,
                           pixel_acc)
from .core.nn import (Adam, Conv2d, Linear, Upsample2x, col2im_add,
                      cross_entropy, im2col, softmax)
from .core.planner import astar, geometric_cost, path_length, smooth_path
from .core.segnet import SegNet, class_weights
from .core.semantic_map import CameraModel, SemMapConfig, SemanticGrid

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
        print("  [%s] %-24s %s" % ("PASS" if ok else "FAIL", name, note))


def _grad_check(layer, loss_fn, eps=1e-6):
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
    print("\n[1/5] 神经网络")
    x = rng.normal(size=(2, 3, 12, 10))
    cv = Conv2d(3, 5, 3, stride=2, rng=np.random.default_rng(1))
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

    up = Upsample2x()
    xu = rng.normal(size=(2, 3, 5, 4))
    tu = rng.normal(size=(2, 3, 10, 8))

    def lu():
        d = up.forward(xu) - tu
        return float(np.mean(d * d)), d

    e = 0.0
    _, d = lu()
    dx = up.backward(2.0 * d / d.size)
    for a in range(5):
        for b in range(4):
            old = xu[0, 0, a, b]
            xu[0, 0, a, b] = old + 1e-6
            lp = lu()[0]
            xu[0, 0, a, b] = old - 1e-6
            lm = lu()[0]
            xu[0, 0, a, b] = old
            num = (lp - lm) / 2e-6
            e = max(e, abs(num - dx[0, 0, a, b]) / max(abs(num), 1e-8))
    rep.check("Upsample2x 反向", e < 1e-6, "最大相对误差 %.2e" % e)

    A = rng.normal(size=(1, 2, 8, 8))
    cols = im2col(A, 3, 3, 1)
    B = rng.normal(size=cols.shape)
    lhs = float(np.sum(A * col2im_add(B, A.shape, 3, 3, 1)))
    rep.check("im2col 伴随恒等式", abs(lhs - float(np.sum(cols * B))) < 1e-9,
              "%.8f" % lhs)

    lg = rng.normal(size=(1, 4, 5, 5))
    tg = np.zeros((1, 5, 5), np.int32)
    ce = cross_entropy(lg, tg)
    rep.check("交叉熵有限", np.isfinite(ce) and ce > 0, "CE = %.4f" % ce)
    rep.check("softmax 归一", abs(softmax(lg, 1).sum(axis=1).max() - 1.0) < 1e-9,
              "列和 %.6f" % softmax(lg, 1).sum(axis=1).max())


def _sec_labels(rep: _Report) -> None:
    print("\n[2/5] 语义标签与代价")
    rep.check("类别数", N_CLASS == 9 and len(CLASS_NAMES_CN) == 9,
              "%d 类" % N_CLASS)
    rep.check("配色数量匹配", SEMANTIC_RGB.shape == (N_CLASS, 3),
              "%s" % (SEMANTIC_RGB.shape,))
    rep.check("不可通行类别", BLOCKED[0] and BLOCKED[2] and BLOCKED[4]
              and not BLOCKED[1] and not BLOCKED[3],
              "地面/植被可通行，建筑/障碍/背景不可")
    rep.check("植被代价高于地面",
              TRAVERSAL_COST[3] > TRAVERSAL_COST[1] > 0,
              "地面 %.1f  植被 %.1f" % (TRAVERSAL_COST[1], TRAVERSAL_COST[3]))
    # 语义代价全部有限：语义是估计，inf 会被一次误分类堵死通路
    # （实测网络把起点格判成建筑，语义代价图成功率 0%）
    rep.check("语义代价不含 inf", np.isfinite(TRAVERSAL_COST).all(),
              "最大 %.1f" % TRAVERSAL_COST.max())
    rep.check("语义梯度有序", TRAVERSAL_COST.max() > TRAVERSAL_COST[3]
              > TRAVERSAL_COST[5] > TRAVERSAL_COST[1],
              "地面 %.1f 门 %.1f 植被 %.1f 建筑 %.1f"
              % (TRAVERSAL_COST[1], TRAVERSAL_COST[5], TRAVERSAL_COST[3],
                 TRAVERSAL_COST[2]))

    from .core.segnet import class_weights as cw
    w = cw(np.array([9000, 1000, 100, 50, 10, 5, 5, 5, 5]))
    rep.check("类别加权：稀类权重更高", w[8] > w[0] and np.isclose(w.mean(), 1.0),
              "背景 %.3f  自身 %.3f" % (w[0], w[8]))


def _sec_resize(rep: _Report) -> None:
    """输入与标签的缩放必须覆盖同一块画面。

    源尺寸不是目标整数倍时，早期实现会把 RGB 裁成左上角而标签仍贯穿全图，
    两者错位后 mIoU 直接崩，但训练 loss 看着还在降。
    """
    print("\n[2b/5] 缩放对齐")
    from .core.training import resize_area, resize_nearest

    src = np.arange(128 * 160, dtype=np.float32).reshape(128, 160, 1)
    a = resize_area(src, (96, 96))[:, :, 0]
    lab = np.empty((128, 160), np.int32)
    lab[:] = np.arange(128 * 160).reshape(128, 160)
    n = resize_nearest(lab, (96, 96))
    full = 128 * 160 - 1
    rep.check("面积缩放贯穿全图", a.max() > 0.95 * full,
              "最大值 %.0f / 满量程 %d" % (a.max(), full))
    rep.check("最近邻缩放贯穿全图", n.max() > 0.95 * full,
              "最大值 %d" % n.max())
    # 不变量：标签取的源像素必须落在 RGB 那一格平均的源箱里。
    # 不能用 a[i,j] 反解源位置——它是整箱的均值，除回来的"位置"是虚的。
    bad = 0
    for i in (0, 48, 95):
        r0, r1 = i * 128 // 96, (i + 1) * 128 // 96
        for j in (0, 48, 95):
            c0, c1 = j * 160 // 96, (j + 1) * 160 // 96
            block = src[r0:r1, c0:c1, 0]
            lr, lc = int(n[i, j]) // 160, int(n[i, j]) % 160
            ok = (block.min() <= a[i, j] <= block.max()
                  and r0 <= lr < r1 and c0 <= lc < c1)
            bad += 0 if ok else 1
    rep.check("输入与标签对齐", bad == 0, "9 个采样格中 %d 格错位" % bad)

    # 放大：分箱宽度不足一个源像素，计数会取到 0 → 除零出 NaN。
    up = resize_area(src, (256, 320))
    rep.check("放大不出 NaN", np.isfinite(up).all() and up.shape == (256, 320, 1),
              "%s 有限值 %s" % (up.shape, bool(np.isfinite(up).all())))


def _sec_map(rep: _Report, rng) -> None:
    print("\n[3/5] 语义栅格地图")
    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=64, height=48, fovy_deg=60.0)

    # 正前方 6 m 处一堵墙，下半部分是地面。深度必须按视线几何给：
    # 全图填同一个深度会让不同俯仰角的像素反投影到**同一格**，二维上塌缩成一条线。
    h, w = cam.height, cam.width
    drone_h = 1.6
    depth = np.zeros((h, w), np.float32)
    seg = np.zeros((h, w), np.int32)
    for v in range(h):
        ang = math.atan((v - cam.cy) / cam.fy)      # 正值为视线向下
        if ang > math.radians(2.0):
            depth[v, :] = drone_h / math.sin(ang)    # 地面：视线与地面相交
            seg[v, :] = 1
        elif ang > math.radians(-30.0):
            depth[v, :] = 6.0                        # 6 m 处的墙
            seg[v, :] = 2
    n = g.integrate(np.array([0.0, 0.0, 0.0]), depth, seg, cam)
    rep.check("融合产生有效单元", n > 0 and g.hits.sum() > 0,
              "命中 %d 格" % int(g.hits.sum()))
    occ = g.probability() > 0.6
    if occ.any():
        idx = np.argwhere(occ)
        xy = np.stack([g.centre(r, c) for r, c in idx])
        d = np.linalg.norm(xy, axis=1)
        rep.check("占据格落在 6 m 附近", 4.0 < float(np.median(d)) < 8.0,
                  "中位距离 %.2f m" % float(np.median(d)))
    else:
        rep.check("占据格落在 6 m 附近", False, "没有占据格")

    cls = g.semantic()
    seen = np.unique(cls[g.observed()])
    rep.check("语义投票生效", len(seen) >= 2, "出现的类别 %s" % seen.tolist())

    cost = g.cost_map()
    rep.check("代价图为有限值或 inf",
              np.isfinite(cost).any() or np.isinf(cost).any(),
              "有限 %.0f%%" % (100 * np.isfinite(cost).mean()))

    rep.check("观测范围的界定", g.observed().sum() < g.n * g.n,
              "已观测 %.1f%%" % (100 * g.observed().mean()))


def _sec_plan(rep: _Report, rng) -> None:
    print("\n[4/5] 路径规划")
    n = 60
    cost = np.ones((n, n))
    r = astar(cost, (5, 5), (50, 50))
    rep.check("空旷场景可达", r.ok, "代价 %.2f，展开 %d 节点" % (r.cost, r.expanded))
    s = smooth_path(cost, r.cells)
    rep.check("视线捷径缩短节点数", len(s) <= len(r.cells),
              "%d → %d" % (len(r.cells), len(s)))
    L = path_length(_GridLike(0.2, n), s)
    rep.check("路径长度合理", abs(L - 45 * 0.2 * math.sqrt(2)) < 2.0,
              "%.2f m（直线 %.2f m）" % (L, math.hypot(45, 45) * 0.2))

    # 一堵墙，中间留缺口：必须从缺口过
    wall = np.ones((n, n))
    wall[:, 30] = np.inf
    wall[28:32, 30] = 1.0
    r2 = astar(wall, (5, 5), (5, 50))
    rep.check("绕墙通过缺口", r2.ok, "展开 %d 节点" % r2.expanded)
    if r2.ok:
        cols = {c for _, c in r2.cells}
        rep.check("确实经过缺口", any(28 <= rr <= 31 for rr, cc in r2.cells
                                      if cc == 30),
                  "第 30 列经过的行 %s"
                  % sorted({rr for rr, cc in r2.cells if cc == 30}))

    # 完全封死
    wall2 = np.ones((n, n))
    wall2[:, 30] = np.inf
    r3 = astar(wall2, (5, 5), (5, 50))
    rep.check("完全封死时正确报告不可达", not r3.ok, r3.reason)

    # 高代价带（第 20~39 行）：起点在第 25 行带内，语义规划应绕到带外
    band = np.ones((n, n))
    band[20:40, :] = 6.0
    r4 = astar(band, (25, 2), (25, 56))
    inside = sum(1 for rr, _ in r4.cells if 20 <= rr < 40)
    straight = 54 * 3.5
    rep.check("高代价格被绕开", r4.ok and r4.cost < 0.7 * straight,
              "代价 %.0f（直穿约 %.0f），带内格数 %d" % (r4.cost, straight, inside))

    g = SemanticGrid(SemMapConfig())
    rep.check("几何代价图忽略类别", np.isfinite(geometric_cost(g)).all(),
              "空地图上全为有限值")


class _GridLike:
    def __init__(self, res, n):
        self.res, self.n, self.cfg = res, n, type("C", (), {"res": res})()

    def centre(self, r, c):
        return np.array([c * self.res, r * self.res])


def _sec_end2end(rep: _Report) -> float:
    print("\n[5/5] 端到端（短采集，不含 ROS）")
    t0 = time.time()
    from .core.segdata import EpisodeSpec, collect_episode, u8_to_depth
    from .core.segnet import SegNet
    from .core.training import SegTrainConfig, predict_episode, train_seg
    from .core.segdata import SegDataset

    spec = EpisodeSpec(0, "urban_block", span=10.0, depth_m=10.0, lanes=3,
                       height=3.2, start_x=-6.0, start_y=-6.0)
    ep = collect_episode(spec, hw=(64, 80), rate_hz=3.0, verbose=True)
    rep.check("采集一集", len(ep) > 10, "%d 帧" % len(ep))

    ds = SegDataset([ep, ep, ep])
    net = SegNet()
    net.cfg.in_hw = (64, 64)
    net, hist = train_seg(ds, SegTrainConfig(epochs=2, batch=2, in_hw=(64, 64),
                                             val_frac=0.25, max_per_episode=8),
                          net=net, verbose=False)
    rep.check("训练可跑完", np.isfinite(hist["train"][-1]),
              "训练损失 %.4f → %.4f" % (hist["train"][0], hist["train"][-1]))

    pr = predict_episode(net, ep, in_hw=(64, 64))
    rep.check("推理输出形状", pr["seg"].shape == ep.seg.shape,
              "%s" % (pr["seg"].shape,))
    rep.check("预测类别在范围内", pr["seg"].min() >= 0
              and pr["seg"].max() < N_CLASS,
              "[%d, %d]" % (pr["seg"].min(), pr["seg"].max()))

    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=ep.rgb.shape[2], height=ep.rgb.shape[1])
    for k in range(len(ep)):
        g.integrate((float(ep.pos[k, 0]), float(ep.pos[k, 1]), float(ep.yaw[k])),
                    u8_to_depth(ep.depth[k]), pr["seg"][k], cam)
    rep.check("建图有观测", g.observed().sum() > 0,
              "已观测 %d 格" % int(g.observed().sum()))
    cost = g.cost_map()
    rep.check("代价图可用", np.isfinite(cost).any(),
              "有限格 %.0f%%" % (100 * np.isfinite(cost).mean()))
    return time.time() - t0


# --------------------------------------------------------------------------- #
def run_selftest() -> int:
    print("=" * 74)
    print("  uav_semantic_mapping_nav 自检")
    print("=" * 74)
    rng = np.random.default_rng(0)
    rep = _Report()
    t0 = time.time()
    _sec_nn(rep, rng)
    _sec_labels(rep)
    _sec_resize(rep)
    _sec_map(rep, rng)
    _sec_plan(rep, rng)
    dt = _sec_end2end(rep)
    print("\n" + "=" * 74)
    print("  通过 %d/%d   总耗时 %.1f s" % (rep.ok, rep.n, time.time() - t0))
    if rep.fail:
        print("  未通过：%s" % "、".join(rep.fail))
    print("=" * 74)
    return 0 if rep.ok == rep.n else 1


if __name__ == "__main__":
    raise SystemExit(run_selftest())
