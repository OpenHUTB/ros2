#!/usr/bin/env python3
"""ROS 2 生命周期节点演示：打印时间轴并生成配图（纯标准库，无需 ROS）。

用法：
    python3 main.py
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim
from simple_plot import Canvas, Figure

HERE = os.path.dirname(os.path.abspath(__file__))

STATE_LABELS = ['UNCONFIG', 'INACTIVE', 'ACTIVE', 'FINALIZED']


def _arrow(c, x0, y0, x1, y1, color, head=7.0):
    """画一条带箭头的线。"""
    c.line(x0, y0, x1, y1, color)
    import math
    ang = math.atan2(y1 - y0, x1 - x0)
    for d in (0.45, -0.45):
        c.line(x1, y1,
               x1 - head * math.cos(ang + d),
               y1 - head * math.sin(ang + d), color)


def plot_state_diagram(outdir):
    """状态机转移图（手绘）。"""
    w, h = 760, 470
    c = Canvas(w, h, (255, 255, 255))
    dark = (40, 40, 40)
    blue = (30, 90, 200)
    red = (200, 30, 30)
    grey = (120, 120, 120)
    box_fill = (238, 245, 255)
    active_fill = (225, 245, 230)

    boxes = {
        'unconfigured': (70, 70, 250, 130),
        'inactive': (430, 70, 610, 130),
        'active': (430, 320, 610, 380),
        'finalized': (70, 320, 250, 380),
    }
    for name, (x0, y0, x1, y1) in boxes.items():
        fill = active_fill if name == 'active' else box_fill
        c.rect(x0, y0, x1, y1, fill, fill=True)
        c.rect(x0, y0, x1, y1, dark)
        label = name.upper()
        tw, th = c.text_metrics(label, 2)
        c.text((x0 + x1) / 2 - tw / 2, (y0 + y1) / 2 - th / 2, label, dark, 2)

    # configure: unconfigured -> inactive（上方）
    _arrow(c, 250, 90, 430, 90, blue)
    c.text(285, 72, 'CONFIGURE', blue, 2)
    # cleanup: inactive -> unconfigured（下方）
    _arrow(c, 430, 115, 250, 115, blue)
    c.text(295, 122, 'CLEANUP', blue, 2)
    # activate / deactivate: 右侧竖线
    _arrow(c, 555, 130, 555, 320, blue)
    c.text(565, 215, 'ACTIVATE', blue, 2)
    _arrow(c, 495, 320, 495, 130, red)
    c.text(360, 215, 'DEACTIVATE', red, 2)
    # shutdown: 任意 -> finalized
    _arrow(c, 160, 130, 160, 320, grey)
    c.text(172, 215, 'SHUTDOWN', grey, 2)
    c.text(300, 300, 'FROM ANY STATE', grey, 2)

    title = 'ROS 2 LIFECYCLE STATES'
    tw, _th = c.text_metrics(title, 2)
    c.text((w - tw) / 2, 24, title, dark, 2)
    out = os.path.join(outdir, 'lifecycle_states.png')
    c.save(out)
    return out


def plot_timeline(result, outdir):
    """状态随时间变化（台阶图）。"""
    trace = result['trace']
    figs = Figure(width=760, height=400)
    figs.plot([p['t'] for p in trace], [p['state_idx'] for p in trace],
              color=Figure.BLUE, label='STATE')
    figs.set_ylim(-0.3, 3.3)
    figs.xlabel = 'TIME (S)'
    figs.ylabel = 'STATE INDEX'
    figs.title = 'STATE 0=UNCONFIG 1=INACTIVE 2=ACTIVE 3=FINALIZED'
    out = os.path.join(outdir, 'lifecycle_state_timeline.png')
    figs.save(out)
    return out


def plot_publish(result, outdir):
    """被接受 / 被丢弃的累计条数。"""
    trace = result['trace']
    fig = Figure(width=760, height=400)
    fig.plot([p['t'] for p in trace], [p['published'] for p in trace],
             color=Figure.GREEN, label='ACCEPTED')
    fig.plot([p['t'] for p in trace], [p['dropped'] for p in trace],
             color=Figure.RED, label='DROPPED')
    peak = max(max(p['published'], p['dropped']) for p in trace)
    fig.set_ylim(-2.0, peak + 6.0)
    fig.xlabel = 'TIME (S)'
    fig.ylabel = 'MESSAGES'
    fig.title = 'PUBLISH ACCEPTED VS DROPPED'
    out = os.path.join(outdir, 'lifecycle_publish.png')
    fig.save(out)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--outdir', default=HERE)
    args = ap.parse_args()
    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    result = sim.run()
    stats = sim.summary(result)

    print('=' * 70)
    print('时间轴脚本（每 %.1f s 一个 tick，每个 tick 尝试发布一条心跳）' % sim.TICK)
    for t, act, note in sim.SCRIPT:
        print('  t=%5.1fs  %-10s %s' % (t, act, ('# ' + note) if note else ''))

    print('\n实际执行的转移：')
    for t, act, state in result['applied']:
        print('  t=%5.1fs  %-10s -> %s' % (t, act, state))

    print('\n被拒绝的转移：')
    for t, act, why in result['rejected']:
        print('  t=%5.1fs  %-10s %s' % (t, act, why))

    print('\n统计：')
    print('  tick 总数        %d' % stats['ticks'])
    print('  发布被接受       %d' % stats['published'])
    print('  发布被丢弃       %d' % stats['dropped'])
    print('  激活时间占比     %.1f%%' % (stats['active_ratio'] * 100.0))
    print('  进入 active 次数 %d' % stats['active_count'])
    print('  合法/非法转移    %d / %d' % (stats['applied'], stats['rejected']))
    print('  终态             %s' % stats['final_state'])

    print('\n完整合法序列（configure -> ... -> shutdown）：')
    for act, state in sim.transitions_demo():
        print('  %-11s -> %s' % (act, state))

    f1 = plot_state_diagram(args.outdir)
    f2 = plot_timeline(result, args.outdir)
    f3 = plot_publish(result, args.outdir)
    print('\n输出：')
    for p in (f1, f2, f3):
        print('  ', p, '(%d bytes)' % os.path.getsize(p))


if __name__ == '__main__':
    main()
