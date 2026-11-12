#!/usr/bin/env python3
"""Animate the closed-loop mission simulation as a GIF (no AirSim needed).

复用 simulate_mission 的闭环仿真结果，生成螺旋搜索过程的动画：
左侧为轨迹与相机视场锥，右侧为任务状态指标。生成的 GIF 可用于
PR 运行效果演示（标注为离线仿真动画）。

用法：
    python3 tools/make_demo_gif.py --target-x 12 --target-y -14 \
        --output ../../docs/air/uav_spiral_search/images/spiral_search_demo.gif
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib
matplotlib.use('Agg')
# Windows 自带微软雅黑；其他系统找不到时回退默认字体。
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei',
                                          'Noto Sans CJK SC', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Wedge

from simulate_mission import run_simulation

STATE_CN = {'waiting_odom': '等待里程计', 'climbing': '爬升到搜索高度',
            'searching': '螺旋搜索中', 'locked': '已锁定目标，飞往估计点',
            'exhausted': '搜索区域耗尽', 'odom_stale': '里程计中断'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-x', type=float, default=12.0)
    parser.add_argument('--target-y', type=float, default=-14.0)
    parser.add_argument('--start-x', type=float, default=0.0)
    parser.add_argument('--start-y', type=float, default=0.0)
    parser.add_argument('--start-z', type=float, default=-1.0)
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--dt', type=float, default=0.05)
    parser.add_argument('--max-time', type=float, default=300.0)
    parser.add_argument('--sample-step', type=int, default=2,
                        help='每 N 个日志点取一帧（日志 10 Hz）')
    parser.add_argument('--fps', type=int, default=8)
    parser.add_argument('--output', type=str, default='spiral_search_demo.gif')
    args = parser.parse_args()

    controller, report, target, log = run_simulation(args)
    n_frames = len(log['t'])
    indices = list(range(0, n_frames, args.sample_step))
    if indices[-1] != n_frames - 1:
        indices.append(n_frames - 1)

    # 每帧的累计飞行路程（沿日志轨迹累加）。
    cum_path = [0.0]
    for i in range(1, n_frames):
        d = math.hypot(log['pos'][i][0] - log['pos'][i - 1][0],
                       log['pos'][i][1] - log['pos'][i - 1][1])
        cum_path.append(cum_path[-1] + d)
    origin = controller.origin or (log['pos'][0][0], log['pos'][0][1])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.6, 4.8))

    # 参考螺旋（静态浅色）与固定要素
    rx = [r[0] for r in log['ref'] if r]
    ry = [r[1] for r in log['ref'] if r]
    ax1.plot(rx, ry, ':', lw=0.9, color='0.6', label='spiral reference', zorder=1)
    ax1.plot(log['pos'][0][0], log['pos'][0][1], 'k^', ms=8, label='start')
    ax1.plot(*target, 'r*', ms=14, label='target (truth)', zorder=5)
    if controller.target_estimate is not None:
        ax1.plot(*controller.target_estimate, 'rx', ms=10, mew=2,
                 label='estimate', zorder=5)
    ax1.set_xlabel('x / NED m')
    ax1.set_ylabel('y / NED m')
    ax1.set_aspect('equal', adjustable='datalim')
    ax1.legend(loc='upper left', fontsize=7)
    ax1.grid(alpha=0.3)
    all_x = [p[0] for p in log['pos']]
    all_y = [p[1] for p in log['pos']]
    margin = 6.0
    ax1.set_xlim(min(all_x + [target[0]]) - margin,
                 max(all_x + [target[0]]) + margin)
    ax1.set_ylim(min(all_y + [target[1]]) - margin,
                 max(all_y + [target[1]]) + margin)

    (traj_line,) = ax1.plot([], [], '-', lw=1.6, color='tab:blue',
                            label='trajectory', zorder=3)
    (drone_dot,) = ax1.plot([], [], 'o', ms=7, color='tab:blue', zorder=4)
    wedge = Wedge((0, 0), 30, -28.6, 28.6, alpha=0.13,
                  facecolor='tab:blue', edgecolor='none', zorder=2)
    ax1.add_patch(wedge)
    fov_edge, = ax1.plot([], [], '-', lw=1.0, color='tab:red', alpha=0.8, zorder=2)

    title = ax1.set_title('offline closed-loop simulation')

    info = ax2.axis('off')
    lines = {key: ax2.text(0.02, 0.9 - i * 0.11, '', fontsize=12,
                           transform=ax2.transAxes)
             for i, key in enumerate(['state', 't', 'radius', 'path',
                                      'confirm', 'dist', 'estimate'])}
    ax2.text(0.02, 0.06, 'uav_spiral_search offline simulation\n'
             '(一阶惯性模型代替 AirSim，控制器为真实代码)',
             fontsize=8, color='0.4', transform=ax2.transAxes)

    half_fov_deg = 28.6

    def update(i):
        k = indices[i]
        t = log['t'][k]
        x, y = log['pos'][k]
        state = log['state'][k]
        traj_line.set_data([p[0] for p in log['pos'][:k + 1]],
                           [p[1] for p in log['pos'][:k + 1]])
        drone_dot.set_data([x], [y])
        wedge.set_center((x, y))
        # 视场锥：机头朝北（yaw=0），半视场角 28.6°
        if log['confirm'][k] > 0:
            wedge.set_facecolor('tab:red')
            wedge.set_alpha(0.18)
        else:
            wedge.set_facecolor('tab:blue')
            wedge.set_alpha(0.10)
        fov_edge.set_data([], [])
        title.set_text(f'offline closed-loop simulation  '
                       f't={t:.1f}s  state={state}')
        lines['state'].set_text(f'状态      : {STATE_CN.get(state, state)}')
        lines['t'].set_text(f'时间      : {t:6.1f} s')
        radius = math.hypot(x - origin[0], y - origin[1])
        lines['radius'].set_text(f'搜索半径  : {radius:5.1f} m')
        lines['path'].set_text(f'飞行路程  : {cum_path[k]:5.1f} m')
        lines['confirm'].set_text(f'连续确认  : {log["confirm"][k]} 帧')
        lines['dist'].set_text(f'距目标    : {log["dist"][k]:5.1f} m')
        est = ''
        if controller.target_estimate is not None and controller.lock_time \
                is not None and t >= controller.lock_time:
            ex, ey = controller.target_estimate
            est = f'({ex:.1f}, {ey:.1f}) m'
        lines['estimate'].set_text(f'目标估计  : {est}')
        return (traj_line, drone_dot, wedge, title,
                *lines.values())

    anim = FuncAnimation(fig, update, frames=len(indices),
                         interval=1000.0 / args.fps, blit=False)
    anim.save(args.output, writer=PillowWriter(fps=args.fps), dpi=75)
    size_mb = Path(args.output).stat().st_size / 1e6
    print(f'GIF 已保存: {args.output} ({size_mb:.2f} MB, '
          f'{len(indices)} 帧, 最终状态 {report["state"]})')


if __name__ == '__main__':
    main()
