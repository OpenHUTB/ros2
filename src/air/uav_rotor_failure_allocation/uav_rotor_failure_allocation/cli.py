"""离线演示: 打印分配结果与余量, 并生成力矩集合图、偏航权限图和 GIF.

所有数字都由 core.QuadrotorAllocator 直接算出, 不依赖 ROS, 也不读外部数据.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .core import QuadrotorAllocator

# 演示用的默认参数: 2 kg 四旋翼, 力臂 0.25 m, 反扭矩系数 0.016 m, T/W = 2.
DEFAULTS = {'mass': 2.0, 'arm_length': 0.25, 'torque_ratio': 0.016, 'max_thrust': None}

# 推重比扫描点, 用于对比"单桨失效后还能不能悬停".
TWR_SWEEP = (1.2, 1.4, 1.6, 1.8, 2.0, 2.4, 3.0)


def build_allocator(args):
    """按命令行参数建立分配模型."""
    return QuadrotorAllocator(mass=args.mass, arm_length=args.arm_length,
                              torque_ratio=args.torque_ratio, max_thrust=args.max_thrust)


def summarize(allocator, failed_rotor=0):
    """汇总完好与单桨失效两种状态下的关键指标, 便于打印和写 JSON."""
    hover = allocator.hover_thrust
    hover_thrusts, hover_ok = allocator.hover_allocation()
    healthy_polygon = allocator.feasible_polygon(failed_rotor=None)
    failed_polygon = allocator.feasible_polygon(failed_rotor=failed_rotor)
    failed_thrusts, failed_ok, achieved = allocator.allocate_with_failure(
        [hover, 0.0, 0.0, 0.0], failed_rotor)

    healthy_area = allocator.polygon_area(healthy_polygon)
    failed_area = allocator.polygon_area(failed_polygon)
    roll_healthy, pitch_healthy = allocator.polygon_extents(healthy_polygon)
    roll_failed, pitch_failed = allocator.polygon_extents(failed_polygon)

    sweep = []
    for ratio in TWR_SWEEP:
        max_thrust = ratio * hover / 4.0
        scaled = QuadrotorAllocator(mass=allocator.mass, arm_length=allocator.arm_length,
                                   torque_ratio=allocator.torque_ratio,
                                   max_thrust=max_thrust, gravity=allocator.gravity)
        limit = scaled.max_thrust_zero_moment(failed_rotor)
        sweep.append({'thrust_to_weight': round(ratio, 3),
                      'max_thrust_zero_moment_over_mg': round(limit / hover, 4),
                      'hover_after_failure': bool(limit >= hover - 1e-9)})

    return {
        'parameters': {
            'mass_kg': allocator.mass,
            'arm_length_m': allocator.arm_length,
            'torque_ratio_m': allocator.torque_ratio,
            'max_thrust_n': round(allocator.max_thrust, 4),
            'hover_thrust_n': round(hover, 4),
            'thrust_to_weight': round(allocator.thrust_to_weight, 4),
            'gravity': allocator.gravity,
            'rotor_angles_deg': list(QuadrotorAllocator.ROTOR_ANGLES_DEG),
            'spin_signs': list(QuadrotorAllocator.SPIN_SIGNS),
        },
        'healthy': {
            'hover_thrust_per_rotor_n': [round(float(v), 4) for v in hover_thrusts],
            'hover_feasible': bool(hover_ok),
            'max_thrust_zero_moment_n': round(allocator.max_thrust_zero_moment(), 4),
            'max_thrust_zero_moment_over_mg':
                round(allocator.max_thrust_zero_moment() / hover, 4),
            'roll_pitch_extent_nm': [round(roll_healthy, 4), round(pitch_healthy, 4)],
            'max_yaw_torque_nm': round(allocator.max_yaw_torque(hover), 5),
            'feasible_area_nm2': round(healthy_area, 4),
        },
        'failure': {
            'failed_rotor': failed_rotor,
            'hover_thrusts_n': [round(float(v), 4) for v in failed_thrusts],
            'hover_feasible': bool(failed_ok),
            'forced_yaw_torque_nm': round(float(achieved[3]), 5),
            'max_thrust_zero_moment_n': round(allocator.max_thrust_zero_moment(failed_rotor), 4),
            'max_thrust_zero_moment_over_mg':
                round(allocator.max_thrust_zero_moment(failed_rotor) / hover, 4),
            'roll_pitch_extent_nm': [round(roll_failed, 4), round(pitch_failed, 4)],
            'feasible_area_nm2': round(failed_area, 4),
            'area_ratio_to_healthy': round(failed_area / healthy_area, 4) if healthy_area else 0.0,
        },
        'thrust_to_weight_sweep': sweep,
    }


def _format(value):
    """打印用: 把 -0.0 和浮点噪声附近的值统一成 0."""
    return '%.2f' % (0.0 if abs(value) < 5e-3 else value)


def print_summary(report):
    """把汇总结果打印成可对照的表."""
    p = report['parameters']
    healthy = report['healthy']
    failure = report['failure']
    hover = p['hover_thrust_n']
    print('四旋翼单桨失效推力分配可行性')
    print('参数: m=%.2f kg, L=%.3f m, κ=%.4f m, f_max=%.2f N (单桨), '
          'T/W=%.2f, mg=%.2f N'
          % (p['mass_kg'], p['arm_length_m'], p['torque_ratio_m'],
             p['max_thrust_n'], p['thrust_to_weight'], hover))
    print()
    print('[完好] 悬停每桨 %.2f N (占单桨上限 %.1f%%)'
          % (healthy['hover_thrust_per_rotor_n'][0],
             100.0 * healthy['hover_thrust_per_rotor_n'][0] / p['max_thrust_n']))
    print('       零力矩最大总推力 %.2f N (%.2f mg)'
          % (healthy['max_thrust_zero_moment_n'], healthy['max_thrust_zero_moment_over_mg']))
    print('       滚转/俯仰力矩权限 ±%.3f / ±%.3f N·m'
          % (healthy['roll_pitch_extent_nm'][0], healthy['roll_pitch_extent_nm'][1]))
    print('       偏航力矩权限 %.4f N·m (κ·mg = %.4f)'
          % (healthy['max_yaw_torque_nm'], p['torque_ratio_m'] * hover))
    print('       可达力矩集合面积 %.3f (N·m)^2' % healthy['feasible_area_nm2'])
    print()
    print('[失效#%d] 悬停解 f = [%s] N (%s)'
          % (failure['failed_rotor'],
             ', '.join(_format(v) for v in failure['hover_thrusts_n']),
             '可行' if failure['hover_feasible'] else '不可行'))
    print('        零力矩最大总推力 %.2f N (%.2f mg), 悬停余量 %.1f%%'
          % (failure['max_thrust_zero_moment_n'],
             failure['max_thrust_zero_moment_over_mg'],
             100.0 * (failure['max_thrust_zero_moment_over_mg'] - 1.0)))
    print('        不可控偏航 %+.4f N·m' % failure['forced_yaw_torque_nm'])
    print('        滚转/俯仰力矩权限 ±%.3f / ±%.3f N·m'
          % (failure['roll_pitch_extent_nm'][0], failure['roll_pitch_extent_nm'][1]))
    print('        可达力矩集合面积 %.3f (N·m)^2 (完好时的 %.1f%%)'
          % (failure['feasible_area_nm2'], 100.0 * failure['area_ratio_to_healthy']))
    print()
    print('[推重比扫描] T/W   零力矩最大推力/mg   失效后能否悬停')
    for row in report['thrust_to_weight_sweep']:
        print('             %.1f    %.3f              %s'
              % (row['thrust_to_weight'], row['max_thrust_zero_moment_over_mg'],
                 '能' if row['hover_after_failure'] else '不能'))


def plot_feasible_set(allocator, output):
    """画完好与失效两种状态下的 (τx, τy) 可达集合."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    hover = allocator.hover_thrust
    healthy = allocator.feasible_polygon(failed_rotor=None)
    failed = allocator.feasible_polygon(failed_rotor=0)
    fig, ax = plt.subplots(figsize=(6.4, 5.2))
    for polygon, label, color in ((healthy, 'all four rotors healthy', '#1f77b4'),
                                  (failed, 'one rotor failed (rotor 0)', '#d62728')):
        if polygon.shape[0] >= 3:
            ax.fill(polygon[:, 0], polygon[:, 1], alpha=0.18, color=color)
            ax.plot(np.append(polygon[:, 0], polygon[0, 0]),
                    np.append(polygon[:, 1], polygon[0, 1]),
                    '-o', ms=3, color=color, label=label)
    ax.plot([0], [0], 'k+', ms=12, label='level attitude (τx=τy=0)')
    ax.axhline(0, color='0.7', lw=0.6)
    ax.axvline(0, color='0.7', lw=0.6)
    ax.set_aspect('equal')
    ax.set_xlabel('roll torque τx (N·m)')
    ax.set_ylabel('pitch torque τy (N·m)')
    ax.set_title('Attainable roll/pitch torque at total thrust = mg')
    ax.text(0.98, 0.02,
            'area: %.2f N²·m² healthy\n        %.2f N²·m² after one rotor loss (%.1f%%)'
            % (allocator.polygon_area(healthy), allocator.polygon_area(failed),
               100.0 * allocator.polygon_area(failed) / allocator.polygon_area(healthy)),
            transform=ax.transAxes, fontsize=8, color='0.25', ha='right', va='bottom')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = output / 'feasible_set.png'
    fig.savefig(path, dpi=150)
    return path, fig, ax


def plot_yaw_authority(allocator, output):
    """画偏航权限随总推力的变化, 并标出失效悬停时的被动偏航."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    hover = allocator.hover_thrust
    limits = 4.0 * allocator.max_thrust
    thrusts = np.linspace(0.0, limits, 200)
    yaw = [allocator.max_yaw_torque(t) for t in thrusts]
    normalised = [value / (allocator.torque_ratio * hover) for value in yaw]

    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    ax.plot(thrusts / hover, normalised, color='#1f77b4',
            label='max |τz| with all rotors healthy')
    ax.axhline(1.0, color='#d62728', ls='--',
               label='forced τz after one rotor fails at hover (κ·mg)')
    ax.axvline(1.0, color='0.6', lw=0.8, ls=':')
    ax.plot([1.0], [1.0], 'o', color='#d62728', ms=5)
    ax.set_xlabel('total thrust T / (mg)')
    ax.set_ylabel('yaw torque / (κ·mg)')
    ax.set_title('Yaw authority vs. total thrust (T/W = %.2f)' % allocator.thrust_to_weight)
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = output / 'yaw_authority.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_authority_margin(allocator, output):
    """画零力矩最大推力随推重比的变化, 给出失效后仍能悬停的门槛."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    hover = allocator.hover_thrust
    ratios = np.linspace(1.0, 3.2, 120)
    healthy, failed = [], []
    for ratio in ratios:
        scaled = QuadrotorAllocator(mass=allocator.mass, arm_length=allocator.arm_length,
                                   torque_ratio=allocator.torque_ratio,
                                   max_thrust=ratio * hover / 4.0, gravity=allocator.gravity)
        healthy.append(scaled.max_thrust_zero_moment() / hover)
        failed.append(scaled.max_thrust_zero_moment(0) / hover)

    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    ax.plot(ratios, healthy, color='#1f77b4', label='all four rotors healthy (4·f_max)')
    ax.plot(ratios, failed, color='#d62728', label='one rotor failed (2·f_max)')
    ax.axhline(1.0, color='0.4', ls='--', lw=1.0, label='hover requirement (mg)')
    ax.axvline(2.0, color='0.6', lw=0.8, ls=':')
    ax.annotate('T/W = 2 is the minimum for\nzero-moment hover after one rotor loss',
                xy=(2.0, 1.0), xytext=(2.15, 0.45), fontsize=8,
                arrowprops={'arrowstyle': '->', 'lw': 0.8})
    ax.set_xlabel('thrust-to-weight ratio T/W')
    ax.set_ylabel('max total thrust without roll/pitch torque / (mg)')
    ax.set_ylim(0.0, 3.4)
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = output / 'authority_margin.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def make_gif(allocator, output):
    """把期望偏航力矩从 -κ·mg 扫到 +κ·mg, 记录可达集合的收缩过程."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image

    hover = allocator.hover_thrust
    limit = allocator.torque_ratio * hover
    bounds = max(allocator.polygon_extents(allocator.feasible_polygon(None))) * 1.25
    fig, ax = plt.subplots(figsize=(5.2, 4.4))
    frames = []
    # 扫到 ±0.9·κ·mg: 到 ±1.0 时可行集合退化成一个点, 画不出面.
    targets = np.linspace(-0.9 * limit, 0.9 * limit, 40)
    for target in targets:
        polygon = allocator.feasible_polygon(total_thrust=hover, yaw=target)
        ax.clear()
        if polygon.shape[0] >= 3:
            ax.fill(polygon[:, 0], polygon[:, 1], alpha=0.25, color='#1f77b4')
            ax.plot(np.append(polygon[:, 0], polygon[0, 0]),
                    np.append(polygon[:, 1], polygon[0, 1]), '-o', ms=3, color='#1f77b4')
        elif polygon.shape[0]:
            ax.plot(polygon[:, 0], polygon[:, 1], 'o', ms=4, color='#1f77b4')
        ax.plot([0], [0], 'k+', ms=10)
        ax.set_xlim(-bounds, bounds)
        ax.set_ylim(-bounds, bounds)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_xlabel('roll torque τx (N·m)', fontsize=9)
        ax.set_ylabel('pitch torque τy (N·m)', fontsize=9)
        ax.set_title('τz / (κ·mg) = %+.2f' % (target / limit), fontsize=10)
        fig.tight_layout()
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))
    plt.close(fig)
    path = output / 'feasible_set.gif'
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=90, loop=0)
    return path


def _configure_console():
    """容错处理终端编码: Windows 默认 GBK 控制台下遇到未收录字符时用 '?' 代替, 不让 CLI 崩.

    只放宽 errors, 不改 encoding: 改成 UTF-8 会在 GBK 终端里显示乱码.
    stdout/stderr 被重定向到文件或测试替身时 reconfigure 可能不可用, 忽略即可.
    """
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, 'reconfigure', None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors='replace')
        except (ValueError, OSError):  # pragma: no cover - 流不支持重配置时
            pass


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='四旋翼单桨失效推力分配与姿态权限评估')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--mass', type=float, default=DEFAULTS['mass'], help='整机质量 kg')
    parser.add_argument('--arm-length', type=float, default=DEFAULTS['arm_length'], help='力臂 m')
    parser.add_argument('--torque-ratio', type=float, default=DEFAULTS['torque_ratio'],
                        help='反扭矩系数 κ m')
    parser.add_argument('--max-thrust', type=float, default=DEFAULTS['max_thrust'],
                        help='单桨最大推力 N, 缺省取 T/W=2')
    parser.add_argument('--failed-rotor', type=int, default=0, help='失效旋翼编号 0..3')
    parser.add_argument('--gif', action='store_true', help='额外生成偏航扫描 GIF')
    parser.add_argument('--ros', action='store_true', help='改为启动 ROS 2 节点')
    args = parser.parse_args(argv)
    if args.ros:
        from .ros_node import main as ros_main
        ros_main()
        return 0

    allocator = build_allocator(args)
    report = summarize(allocator, args.failed_rotor)
    print_summary(report)

    args.output.mkdir(parents=True, exist_ok=True)
    written = [plot_feasible_set(allocator, args.output)[0],
               plot_yaw_authority(allocator, args.output),
               plot_authority_margin(allocator, args.output)]
    if args.gif:
        written.append(make_gif(allocator, args.output))
    (args.output / 'report.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print()
    print('已写入: ' + ', '.join(path.name for path in written) + ', report.json')
    print('输出目录: %s' % args.output.resolve())
    return 0
