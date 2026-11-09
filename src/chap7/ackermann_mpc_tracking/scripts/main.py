#!/usr/bin/env python3
"""阿克曼小车 MPC 跟踪演示：打印对比指标并生成配图（纯标准库，无需 ROS）。

用法：
    python3 main.py
    python3 main.py --outdir ./fig
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim
from simple_plot import Figure

HERE = os.path.dirname(os.path.abspath(__file__))


def _fit_limits(points, width=720, height=420, pad=0.06):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
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


def plot_tracks(path, trace_m, trace_s, outdir):
    ref = path.sample(step=0.2)
    pts = [(p[0], p[1]) for p in ref]
    pts += [(p['x'], p['y']) for p in trace_m]
    pts += [(p['x'], p['y']) for p in trace_s]
    xlim, ylim = _fit_limits(pts)
    fig = Figure(width=760, height=430)
    fig.set_xlim(*xlim)
    fig.set_ylim(*ylim)
    fig.title = 'S-CURVE TRACKING  V=5M/S  E0=0.8M'
    fig.plot([p[0] for p in ref], [p[1] for p in ref],
             color=Figure.BLACK, label='REFERENCE')
    fig.plot([p['x'] for p in trace_m], [p['y'] for p in trace_m],
             color=Figure.BLUE, label='MPC')
    fig.plot([p['x'] for p in trace_s], [p['y'] for p in trace_s],
             color=Figure.RED, label='STANLEY')
    fig.xlabel = 'x (m)'
    fig.ylabel = 'y (m)'
    out = os.path.join(outdir, 'mpc_tracking_path.png')
    fig.save(out)
    return out


def plot_error(trace_m, trace_s, outdir):
    fig = Figure(width=760, height=420)
    fig.plot([p['s'] for p in trace_m], [abs(p['e_y']) for p in trace_m],
             color=Figure.BLUE, label='MPC')
    fig.plot([p['s'] for p in trace_s], [abs(p['e_y']) for p in trace_s],
             color=Figure.RED, label='STANLEY')
    fig.xlabel = 'ARC LENGTH S (M)'
    fig.ylabel = 'ABS EY (M)'
    fig.title = 'LATERAL ERROR'
    out = os.path.join(outdir, 'mpc_error.png')
    fig.save(out)
    return out


def plot_steer(trace_m, trace_s, outdir, delta_max=sim.DELTA_MAX):
    fig = Figure(width=760, height=420)
    fig.plot([p['s'] for p in trace_m], [p['delta'] for p in trace_m],
             color=Figure.BLUE, label='MPC')
    fig.plot([p['s'] for p in trace_s], [p['delta'] for p in trace_s],
             color=Figure.RED, label='STANLEY')
    fig.plot([0.0, max(p['s'] for p in trace_m)], [delta_max, delta_max],
             color=Figure.ORANGE, label='LIMIT %.2f' % delta_max)
    fig.plot([0.0, max(p['s'] for p in trace_m)], [-delta_max, -delta_max],
             color=Figure.ORANGE)
    fig.xlabel = 'ARC LENGTH S (M)'
    fig.ylabel = 'STEER DELTA (RAD)'
    fig.title = 'FRONT WHEEL ANGLE'
    out = os.path.join(outdir, 'mpc_steer.png')
    fig.save(out)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--outdir', default=HERE)
    args = ap.parse_args()
    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    path, trace_m, mm, trace_s, ms = sim.benchmark()

    print('=' * 74)
    print('参考路径：直线 6 m + 右转 R=6 90° + 左转 R=6 90° + 直线 8 m，总长 %.3f m'
          % path.length)
    print('车辆：轴距 %.1f m，速度 %.1f m/s，转角上限 %.2f rad，转向速率上限 %.1f rad/s'
          % (sim.WHEELBASE, sim.V_CRUISE, sim.DELTA_MAX, 1.6))
    print('初始状态：距路径横向 %.1f m，航向与路径一致' % 0.8)

    print('\n%-10s %10s %10s %10s %10s %10s' %
          ('控制器', 'RMSE(m)', '最大(m)', '末值(m)', '峰值转角', '转向速率'))
    for name, m in (('MPC', mm), ('Stanley', ms)):
        print('%-10s %10.4f %10.4f %10.4f %10.3f %10.3f'
              % (name, m['rmse'], m['max_err'], m['final_err'],
                 m['max_delta'], m['rms_rate']))

    print('\n--- Stanley 的 k 扫描（v=5, e0=0.8）')
    print('%6s %10s %10s %10s' % ('k', 'RMSE(m)', '最大(m)', '末值(m)'))
    best = None
    for k in (0.5, 1.0, 1.5, 2.5, 4.0, 6.0, 10.0):
        p2 = sim.build_s_curve()
        _t, m = sim.run_stanley(p2, (0.0, 0.8, 0.0), v=sim.V_CRUISE, k=k)
        print('%6.1f %10.4f %10.4f %10.4f' % (k, m['rmse'], m['max_err'],
                                              m['final_err']))
        if best is None or m['rmse'] < best[1]:
            best = (k, m['rmse'])
    print('  最优 k = %.1f，RMSE = %.4f（MPC 为 %.4f）' % (best[0], best[1], mm['rmse']))

    print('\n--- MPC 的 q_y 扫描（v=5, e0=0.8）')
    print('%6s %10s %10s %10s' % ('q_y', 'RMSE(m)', '最大(m)', '末值(m)'))
    for qy in (2.0, 5.0, 10.0, 30.0, 60.0):
        p3 = sim.build_s_curve()
        _t, m = sim.run_mpc(p3, (0.0, 0.8, 0.0), v=sim.V_CRUISE, q=(qy, 5.0))
        print('%6.1f %10.4f %10.4f %10.4f' % (qy, m['rmse'], m['max_err'],
                                              m['final_err']))

    print('\n--- 初始偏差扫描')
    print('%8s %12s %12s %10s' % ('e0(m)', 'MPC RMSE', 'Stanley RMSE', '倍数'))
    for e0 in (0.3, 0.6, 0.9, 1.2):
        p4 = sim.build_s_curve()
        _t, m_a = sim.run_mpc(p4, (0.0, e0, 0.0), v=sim.V_CRUISE)
        p5 = sim.build_s_curve()
        _t2, m_b = sim.run_stanley(p5, (0.0, e0, 0.0), v=sim.V_CRUISE)
        print('%8.1f %12.4f %12.4f %10.2f'
              % (e0, m_a['rmse'], m_b['rmse'], m_b['rmse'] / m_a['rmse']))

    f1 = plot_tracks(path, trace_m, trace_s, args.outdir)
    f2 = plot_error(trace_m, trace_s, args.outdir)
    f3 = plot_steer(trace_m, trace_s, args.outdir)
    print('\n输出：')
    for p in (f1, f2, f3):
        print('  ', p, '(%d bytes)' % os.path.getsize(p))


if __name__ == '__main__':
    main()
