"""命令行入口: 打印桨尖马赫数、桨盘载荷与诱导速度, 并写出 report.json.

只用标准库（math/json/argparse），不依赖 NumPy/Matplotlib/ROS。
"""
import argparse
import json
import sys
from pathlib import Path

from .core import DISK_LOADING_HIGH, DISK_LOADING_LOW, MACH_LIMIT, MACH_MARGIN, PropCheck

DEFAULTS = {
    'mass': 2.0,
    'diameter': 0.254,
    'rotor_count': 4,
    'hover_rpm': 6500.0,
    'max_rpm': 9000.0,
    'temperature': 15.0,
}

LEVEL_TEXT = {
    'ok': '正常',
    'caution': '注意',
    'too_high': '过高',
    'low': '偏低',
    'medium': '中等',
    'high': '偏高',
}


def _configure_console():
    """Windows 默认 GBK 控制台下遇到未收录字符时用 '?' 代替, 不让 CLI 崩."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, 'reconfigure', None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors='replace')
        except (ValueError, OSError):  # pragma: no cover - 流不支持重配置时
            pass


def build_check(args):
    """按命令行参数建立核算对象."""
    return PropCheck(mass=args.mass, diameter_m=args.diameter,
                     rotor_count=args.rotor_count, hover_rpm=args.hover_rpm,
                     max_rpm=args.max_rpm, temperature_c=args.temperature)


def print_report(report):
    """打印成可对照的表（全部使用 GBK 可编码字符）."""
    p = report['parameters']
    print('==> uav_prop_tip_mach')
    print('    参数: m=%.2f kg, 桨径=%.3f m, 旋翼数=%d, 悬停转速=%.0f rpm, '
          '最大转速=%.0f rpm, 温度=%.0f C'
          % (p['mass_kg'], p['diameter_m'], p['rotor_count'], p['hover_rpm'],
             p['max_rpm'], p['temperature_c']))
    print('    声速: %.2f m/s, 悬停重力: %.2f N, 单桨推力: %.2f N'
          % (report['speed_of_sound_m_s'], report['weight_n'],
             report['thrust_per_rotor_n']))
    print('    总桨盘面积: %.4f m^2' % report['total_disk_area_m2'])
    print('    桨尖速度(悬停): %.2f m/s' % report['tip_speed_hover_m_s'])
    print('    桨尖马赫数: 悬停 %.3f, 最大转速 %.3f (%s)'
          % (report['tip_mach_hover'], report['tip_mach_max'],
             LEVEL_TEXT[report['tip_mach_level']]))
    print('    桨盘载荷: %.1f N/m^2 (%.2f kg/m^2, %s)'
          % (report['disk_loading_n_m2'], report['disk_loading_kg_m2'],
             LEVEL_TEXT[report['disk_loading_level']]))
    print('    悬停诱导速度: %.2f m/s' % report['induced_velocity_m_s'])
    limits = report['limits']
    print('    参考上限: 马赫 %.1f 对应 %.0f rpm, 马赫 %.1f 对应 %.0f rpm, '
          '桨盘载荷 %.0f N/m^2 对应桨径 %.3f m'
          % (MACH_MARGIN, limits['rpm_for_mach_0_6'], MACH_LIMIT,
             limits['rpm_for_mach_0_7'], DISK_LOADING_LOW,
             limits['diameter_for_100_n_m2']))


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='旋翼桨尖马赫数与桨盘载荷核算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--mass', type=float, default=DEFAULTS['mass'], help='整机质量 kg')
    parser.add_argument('--diameter', type=float, default=DEFAULTS['diameter'],
                        help='单桨直径 m')
    parser.add_argument('--rotor-count', type=int, default=DEFAULTS['rotor_count'],
                        help='旋翼数量')
    parser.add_argument('--hover-rpm', type=float, default=DEFAULTS['hover_rpm'],
                        help='悬停转速 rpm')
    parser.add_argument('--max-rpm', type=float, default=DEFAULTS['max_rpm'],
                        help='最大转速 rpm')
    parser.add_argument('--temperature', type=float, default=DEFAULTS['temperature'],
                        help='环境温度 C, 影响声速')
    args = parser.parse_args(argv)

    check = build_check(args)
    report = check.report()
    print_report(report)

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print()
    print('已写入: report.json')
    print('输出目录: %s' % args.output.resolve())
    return 0
