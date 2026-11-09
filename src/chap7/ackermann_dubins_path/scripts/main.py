#!/usr/bin/env python3
"""Dubins 最短路径演示：打印统计并生成配图（纯标准库，无需 ROS）。

用法：
    python3 main.py                # 生成 PNG 到脚本目录
    python3 main.py --outdir ./fig
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim
from dubins import dubins_path, all_paths
from simple_plot import Figure

HERE = os.path.dirname(os.path.abspath(__file__))


def _aspect_fig(width=720, height=420, xlim=None, ylim=None, title=''):
    """等比例图：绘图区像素比与坐标跨度比一致，否则圆会被画成椭圆。"""
    fig = Figure(width=width, height=height)
    if xlim and ylim:
        fig.set_xlim(*xlim)
        fig.set_ylim(*ylim)
    fig.title = title
    return fig


def _fit_limits(points, width=720, height=420, pad=0.06):
    """按点集算等比例坐标范围（扩展较短的一维，而不是压扁图形）。"""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    span = max(x1 - x0, y1 - y0, 1e-6)
    x0 -= span * pad
    x1 += span * pad
    y0 -= span * pad
    y1 += span * pad
    aspect = float(width - 72 - 28) / float(height - 52 - 60)
    w, h = x1 - x0, y1 - y0
    if w / h < aspect:
        need = h * aspect
        cx = 0.5 * (x0 + x1)
        x0, x1 = cx - need / 2.0, cx + need / 2.0
    else:
        need = w / aspect
        cy = 0.5 * (y0 + y1)
        y0, y1 = cy - need / 2.0, cy + need / 2.0
    return (x0, x1), (y0, y1)


def plot_cases(outdir):
    """图 1：两种工况的最优路径，标出圆弧与直线段。"""
    picks = [
        ((0.0, 0.0, 0.0), (10.0, 0.0, math.pi / 2)),
        ((0.0, 0.0, 0.0), (10.0, 0.0, math.pi)),
    ]
    paths = [dubins_path(s, g, 1.0) for s, g in picks]
    samples = [p.sample(step=0.04) for p in paths]
    allpts = [pt for pts in samples for pt in pts]
    xlim, ylim = _fit_limits(allpts)
    fig = _aspect_fig(xlim=xlim, ylim=ylim, title='DUBINS PATHS  R=1M')
    colors = [Figure.BLUE, Figure.RED]
    for i, (path, pts) in enumerate(zip(paths, samples)):
        fig.plot([q[0] for q in pts], [q[1] for q in pts],
                 color=colors[i],
                 label='%s %.2fM' % (path.case, path.length))
        for pose, mark in ((picks[i][0], 'start'), (picks[i][1], 'goal')):
            mx, my = pose[0], pose[1]
            fig.plot([mx - 0.15, mx + 0.15], [my, my], color=Figure.BLACK)
            if mark == 'goal':
                fig.plot([mx, mx], [my - 0.15, my + 0.15], color=Figure.BLACK)
    fig.xlabel = 'x (m)'
    fig.ylabel = 'y (m)'
    path = os.path.join(outdir, 'dubins_cases.png')
    fig.save(path)
    return path


def plot_length_vs_heading(outdir):
    """图 2：末端朝向从 -180° 扫到 180°，最短长度与获胜情形。"""
    heads, lengths, cases = sim.heading_sweep(n=181)
    fig = Figure(width=760, height=420)
    fig.plot([h * 180.0 / math.pi for h in heads], lengths,
             color=Figure.BLUE, label='MIN LENGTH')
    fig.xlabel = 'FINAL HEADING (DEG)'
    fig.ylabel = 'LENGTH (M)'
    fig.title = 'LENGTH VS FINAL HEADING  DIST=10M R=1M'
    path = os.path.join(outdir, 'dubins_length_vs_heading.png')
    fig.save(path)

    # 各情形覆盖的朝向区间，用来在文档里说明「何时该用哪种」
    spans = []
    cur = cases[0]
    begin = heads[0]
    for h, c in zip(heads[1:], cases[1:]):
        if c != cur:
            spans.append((cur, begin, h))
            cur, begin = c, h
    spans.append((cur, begin, heads[-1]))
    return path, spans, min(lengths), max(lengths)


def plot_radius_effect(outdir):
    """图 3：同一掉头工况，R=1/2/3 的路径形状。"""
    rs = sim.radius_sweep()
    samples = [p.sample(step=0.05) for _r, p in rs]
    allpts = [pt for pts in samples for pt in pts]
    xlim, ylim = _fit_limits(allpts)
    fig = _aspect_fig(xlim=xlim, ylim=ylim, title='TURN RADIUS EFFECT  U-TURN')
    colors = [Figure.BLUE, Figure.ORANGE, Figure.PURPLE]
    for i, ((r, p), pts) in enumerate(zip(rs, samples)):
        fig.plot([q[0] for q in pts], [q[1] for q in pts],
                 color=colors[i], label='R=%.0fM  %.1fM' % (r, p.length))
    fig.xlabel = 'x (m)'
    fig.ylabel = 'y (m)'
    path = os.path.join(outdir, 'dubins_radius_effect.png')
    fig.save(path)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--outdir', default=HERE)
    args = ap.parse_args()
    outdir = args.outdir
    if not os.path.isdir(outdir):
        os.makedirs(outdir)

    print('=' * 72)
    print('工况汇总（R = 1 m）')
    print('%-10s %-5s %9s %9s %9s %8s' %
          ('工况', '情形', '长度m', '次优m', '省下', '直线m'))
    for row in sim.scenario_table(1.0):
        print('%-10s %-5s %9.4f %9.4f %8.1f%% %8.4f'
              % (row['name'], row['case'], row['length'], row['second'],
                 row['saving'] * 100.0, row['straight']))

    print('\n' + '=' * 72)
    print('六情形对比：(0,0,0) -> (10,0,180°)')
    for case, length, best in sim.case_breakdown((0.0, 0.0, 0.0),
                                                 (10.0, 0.0, math.pi), 1.0):
        if length is None:
            print('  %-4s  不可行' % case)
        else:
            print('  %-4s  %9.4f m %s' % (case, length, '<- 最优' if best else ''))

    worst_pos, worst_ang = sim.endpoint_precision([
        ((0.0, 0.0, 0.0), (4.0, 0.0, math.pi)),
        ((0.0, 0.0, 0.0), (5.5, -2.5, 1.1)),
        ((1.0, 2.0, 0.7), (6.5, -1.2, 2.4)),
        ((0.0, 0.0, 0.0), (0.4, 0.1, -2.9)),
    ])
    print('\n端点闭环最大误差：位置 %.2e m，朝向 %.2e rad' % (worst_pos, worst_ang))

    stats = sim.curvature_stats(dubins_path((0.0, 0.0, 0.0),
                                            (10.0, 0.0, math.pi), 1.0))
    print('曲率上限 %.3f 1/m，采样最大 |kappa| = %.3f 1/m'
          % (stats['limit'], stats['max_kappa']))
    print('圆弧总长 %.4f m，直线总长 %.4f m'
          % (stats['arc_length'], stats['straight_length']))

    f1 = plot_cases(outdir)
    f2, spans, lo, hi = plot_length_vs_heading(outdir)
    f3 = plot_radius_effect(outdir)

    print('\n' + '=' * 72)
    print('末端朝向扫描（位置固定 10 m，R=1）：长度 %.4f ~ %.4f m' % (lo, hi))
    for case, a, b in spans:
        print('  %-4s  覆盖 %7.1f° ~ %7.1f°' % (case, a * 180 / math.pi,
                                                b * 180 / math.pi))
    print('\n输出：')
    for p in (f1, f2, f3):
        print('  ', p, '(%d bytes)' % os.path.getsize(p))


if __name__ == '__main__':
    main()
