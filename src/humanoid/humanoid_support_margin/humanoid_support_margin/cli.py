"""命令行入口: 打印支撑多边形与倾倒裕度, 并写出 report.json（只用标准库）."""
import argparse
import json
import sys
from pathlib import Path

from .core import HumanoidSupport

LEVEL_TEXT = {'ok': '正常', 'marginal': '注意', 'over': '超限'}


def _configure_console():
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, 'reconfigure', None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors='replace')
        except (ValueError, OSError):  # pragma: no cover
            pass


def print_report(report):
    p = report['parameters']
    f = report['foot_loads']
    t = report['tipping']
    print('==> humanoid_support_margin')
    print('    参数: %.1f kg, 重心 (%.2f, %.2f) m 高 %.2f m, 双足 x=%.2f/%.2f m '
          '脚掌 %.2f x %.2f m'
          % (p['mass_kg'], p['com_x_m'], p['com_y_m'], p['com_height_m'],
             p['foot1_x_m'], p['foot2_x_m'], p['foot_width_m'], p['foot_length_m']))
    print('    支撑多边形顶点: %s' % ', '.join('(%.2f, %.2f)' % (x, y)
                                             for x, y in report['support_polygon']))
    print('    重心投影在形内: %s, 稳定裕度 %.3f m (%s)'
          % (report['inside_polygon'], report['margin_m'], LEVEL_TEXT[report['level']]))
    print('    倾倒判据: 临界推力 %.1f N, 临界冲量 %.1f N*s'
          % (t['static_force_n'], t['impulse_ns']))
    print('    足底载荷: 左 %.0f%% (%.0f N), 右 %.0f%% (%.0f N)'
          % (f['fraction_left'] * 100.0, f['left_n'],
             f['fraction_right'] * 100.0, f['right_n']))
    for warning in report['warnings']:
        print('    [提示] %s' % warning)


def build_check(args):
    return HumanoidSupport(mass_kg=args.mass, foot1_x_m=args.foot1_x,
                           foot2_x_m=args.foot2_x, foot_width_m=args.foot_width,
                           foot_length_m=args.foot_length, com_x_m=args.com_x,
                           com_y_m=args.com_y, com_height_m=args.com_height)


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='人形机器人支撑多边形与倾倒裕度核算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--mass', type=float, default=40.0, help='质量 kg')
    parser.add_argument('--foot1-x', type=float, default=-0.10, help='左脚中心 x m')
    parser.add_argument('--foot2-x', type=float, default=0.10, help='右脚中心 x m')
    parser.add_argument('--foot-width', type=float, default=0.10, help='脚掌宽 m')
    parser.add_argument('--foot-length', type=float, default=0.25, help='脚掌长 m')
    parser.add_argument('--com-x', type=float, default=0.01, help='重心投影 x m')
    parser.add_argument('--com-y', type=float, default=0.0, help='重心投影 y m')
    parser.add_argument('--com-height', type=float, default=0.8, help='重心高度 m')
    args = parser.parse_args(argv)

    report = build_check(args).report()
    print_report(report)

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print()
    print('已写入: report.json')
    print('输出目录: %s' % args.output.resolve())
    return 0
