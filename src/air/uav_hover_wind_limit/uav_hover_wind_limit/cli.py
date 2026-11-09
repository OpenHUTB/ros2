"""离线演示: 打印抗风能力表, 生成倾角-风速曲线与阻力面积敏感性图.

所有数字由 core.HoverWindLimit 直接算出, 不依赖 ROS、仿真器或外部数据.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .core import HoverWindLimit, beaufort_level
from .core import build_limits as make_limits

# 演示默认: 2 kg 四旋翼, Cd*A = 0.15 m^2, 最大倾角 25 度, 海平面标准大气.
DEFAULTS = {
    'mass': 2.0,
    'drag_area': 0.15,
    'max_tilt': 25.0,
    'altitude': 0.0,
}

# 敏感性图的倾角曲线.
TILT_SERIES = (15.0, 25.0, 35.0)
DRAG_AREA_RANGE = (0.02, 0.30)


def build_limits(args):
    """按命令行参数建立核算对象; 给了 --tw 就按 acos(1/TW) 限制有效倾角."""
    return make_limits(args.mass, args.drag_area, args.max_tilt, args.altitude, args.tw)


def _configure_console():
    """Windows 默认 GBK 控制台下遇到未收录字符时用 '?' 代替, 不让 CLI 崩.

    只放宽 errors, 不改 encoding: 改成 UTF-8 会在 GBK 终端里显示乱码.
    """
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, 'reconfigure', None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors='replace')
        except (ValueError, OSError):  # pragma: no cover - 流不支持重配置时
            pass


def print_report(report, altitude=0.0):
    """把 report 打印成可对照的表 (全部使用 GBK 可编码字符)."""
    p = report['parameters']
    budget = report['tilt_budget']
    print('==> uav_hover_wind_limit')
    print('    参数: m=%.2f kg, Cd*A=%.3f m^2, 最大倾角=%.1f deg, 海拔=%.0f m, '
          'rho=%.4f kg/m^3'
          % (p['mass_kg'], p['drag_area_m2'], p['max_tilt_deg'], altitude,
             p['air_density_kg_m3']))
    print('    悬停重力: %.2f N' % report['weight_n'])
    print('    最大抗风: %.2f m/s (蒲福 %d 级)'
          % (report['max_wind_m_s'], report['max_wind_beaufort']))
    print('    有效倾角: %.2f deg (受%s限制), 所需推重比 T/W = %.4f '
          '(比竖直悬停多 %.1f%% 推力)'
          % (budget['effective_tilt_deg'],
             '姿态上限' if budget['limited_by'] == 'tilt_limit' else '推力余量',
             report['thrust_to_weight'], report['thrust_increase_percent']))
    if p['available_thrust_to_weight'] is not None:
        print('    实际可用推重比 %.3f -> 可达倾角 %.2f deg; 姿态上限 %.1f deg'
              % (p['available_thrust_to_weight'], budget['achievable_tilt_deg'],
                 p['max_tilt_deg']))
    else:
        print('    提示: 未给 --tw, 默认假设实际可用推重比 >= %.4f(姿态上限要求的余量); '
              '若做不到, 用 --tw 传入实际值' % budget['tilt_limit_thrust_to_weight'])
    gust = report['gust_test']
    print('    阵风校验: 在 %.2f m/s 上再加 %.1f m/s, 需要额外倾斜 %.2f deg (共 %.2f deg)'
          % (gust['base_wind_m_s'], gust['gust_delta_m_s'],
             gust['extra_tilt_deg'], gust['total_tilt_deg']))
    print('    注意: 海拔只改空气密度(阻力); 旋翼最大推力也随密度下降, 需单独用 --tw 给出')
    print()
    print('    [倾角-风速] 倾角(deg)   可悬停风速(m/s)   蒲福风级')
    for row in report['tilt_sweep']:
        print('                %6.2f        %8.3f           %2d'
              % (row['tilt_deg'], row['wind_m_s'], beaufort_level(row['wind_m_s'])))


def render_wind_vs_tilt(limits, output_dir):
    """画倾角-风速曲线: 三档阻力面积 + 最大倾角标注."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    tilts = np.linspace(0.0, min(60.0, limits.max_tilt_deg + 15.0), 200)
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    for factor, style in ((0.5, ':'), (1.0, '-'), (2.0, '--')):
        scaled = HoverWindLimit(mass=limits.mass, drag_area=limits.drag_area * factor,
                                max_tilt_deg=limits.max_tilt_deg,
                                gravity=limits.gravity, rho=limits.rho)
        winds = [scaled.wind_for_tilt(tilt) for tilt in tilts]
        ax.plot(tilts, winds, style, label='Cd*A = %.3f m^2' % scaled.drag_area)
    effective = limits.effective_tilt_deg
    ax.axvline(effective, color='0.5', lw=0.8)
    if limits.achievable_tilt_deg is not None and effective < limits.max_tilt_deg:
        ax.axvline(limits.max_tilt_deg, color='0.5', lw=0.8, ls=':')
        ax.annotate('tilt limit %.1f deg\n(clipped by T/W)' % limits.max_tilt_deg,
                    xy=(limits.max_tilt_deg, 1.0), xytext=(limits.max_tilt_deg + 1.0, 2.0),
                    fontsize=8, color='0.35')
    ax.plot([effective], [limits.max_wind_m_s], 'o', color='#C0392B', ms=6,
            label='design point: %.1f deg, %.2f m/s' % (effective, limits.max_wind_m_s))
    ax.axhline(limits.max_wind_m_s, color='#C0392B', lw=0.8, ls='-.')
    ax.annotate('Beaufort %d' % beaufort_level(limits.max_wind_m_s),
                xy=(effective, limits.max_wind_m_s),
                xytext=(effective + 3.0, limits.max_wind_m_s - 1.5),
                fontsize=9, color='#C0392B')
    ax.set_xlabel('tilt angle theta (deg)')
    ax.set_ylabel('wind speed that can be held (m/s)')
    ax.set_title('Hover wind capability vs. tilt limit')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='upper left')
    fig.tight_layout()
    path = output_dir / 'wind_vs_tilt.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def render_capability(limits, output_dir):
    """画阻力面积敏感性: 不同最大倾角下的抗风上限."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    areas = np.linspace(DRAG_AREA_RANGE[0], DRAG_AREA_RANGE[1], 200)
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    for tilt in TILT_SERIES:
        winds = []
        for area in areas:
            scaled = HoverWindLimit(mass=limits.mass, drag_area=area,
                                    max_tilt_deg=tilt, gravity=limits.gravity,
                                    rho=limits.rho)
            winds.append(scaled.max_wind_m_s)
        ax.plot(areas, winds, label='max tilt = %.0f deg' % tilt)
    ax.plot([limits.drag_area], [limits.max_wind_m_s], 'o', color='#C0392B', ms=6,
            label='design point')
    ax.set_xlabel('drag area Cd*A (m^2)')
    ax.set_ylabel('max hover wind speed (m/s)')
    ax.set_title('Wind capability vs. drag area and tilt limit')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = output_dir / 'capability.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='无人机悬停抗风能力核算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--mass', type=float, default=DEFAULTS['mass'], help='整机质量 kg')
    parser.add_argument('--drag-area', type=float, default=DEFAULTS['drag_area'],
                        help='阻力面积 Cd*A, m^2')
    parser.add_argument('--max-tilt', type=float, default=DEFAULTS['max_tilt'],
                        help='最大倾角 deg')
    parser.add_argument('--altitude', type=float, default=DEFAULTS['altitude'],
                        help='海拔 m, 用于换算空气密度')
    parser.add_argument('--tw', type=float, default=None,
                        help='实际可用推重比 T/W (由旋翼最大推力与功率余量决定), '
                             '给定时有效倾角取 min(max_tilt, acos(1/TW))')
    args = parser.parse_args(argv)

    limits = build_limits(args)
    report = limits.report()
    print_report(report, altitude=args.altitude)

    args.output.mkdir(parents=True, exist_ok=True)
    written = [render_wind_vs_tilt(limits, args.output),
               render_capability(limits, args.output)]
    (args.output / 'report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print()
    print('已写入: ' + ', '.join(path.name for path in written) + ', report.json')
    print('输出目录: %s' % args.output.resolve())
    return 0
