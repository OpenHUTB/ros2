"""命令行入口: 打印配平结果, 并写出 report.json（只用标准库）."""
import argparse
import json
import sys
from pathlib import Path

from .core import UavCgTrim

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
    t = report['motor_thrusts_n']
    e = report['envelope']
    p = report['parameters']
    print('==> uav_cg_trim_envelope')
    print('    参数: %.2f kg, 力臂 x=%.3f y=%.3f m, 偏移 dx=%+.1f dy=%+.1f mm, '
          '重心深度 %.1f mm, 单电机上限 %.1f N'
          % (p['mass_kg'], p['arm_x_m'], p['arm_y_m'], p['dx_m'] * 1000.0,
             p['dy_m'] * 1000.0, p['cog_depth_m'] * 1000.0, p['thrust_max_n']))
    print('    电机拉力: 前 %.2f, 右 %.2f, 后 %.2f, 左 %.2f N (最大 %.2f N, %s)'
          % (t['front'], t['right'], t['rear'], t['left'],
             report['max_motor_thrust_n'], LEVEL_TEXT[report['level']]))
    print('    悬停倾角: 俯仰 %+.2f deg, 横滚 %+.2f deg'
          % (report['trim_tilt_deg']['pitch'], report['trim_tilt_deg']['roll']))
    print('    包线: 纵向 +-%.3f m, 横向 +-%.3f m; 饱和倾角 俯仰 %.1f / 横滚 %.1f deg; '
          '悬停可行 %s'
          % (e['dx_max_m'], e['dy_max_m'], e['sat_pitch_deg'], e['sat_roll_deg'],
             e['hover_feasible']))
    print('    裕度: 纵向 %.1f%%, 横向 %.1f%%'
          % (report['margin_pct']['pitch'], report['margin_pct']['roll']))
    for warning in report['warnings']:
        print('    [提示] %s' % warning)


def build_check(args):
    return UavCgTrim(mass_kg=args.mass, arm_x_m=args.arm_x, arm_y_m=args.arm_y,
                     dx_m=args.dx / 1000.0, dy_m=args.dy / 1000.0,
                     cog_depth_m=args.cog_depth / 1000.0, thrust_max_n=args.thrust_max)


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='多旋翼重心偏移配平与饱和包线核算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--mass', type=float, default=1.2, help='起飞质量 kg')
    parser.add_argument('--arm-x', type=float, default=0.25, help='左右力臂 m')
    parser.add_argument('--arm-y', type=float, default=0.25, help='前后力臂 m')
    parser.add_argument('--dx', type=float, default=3.0, help='纵向重心偏移 mm')
    parser.add_argument('--dy', type=float, default=2.0, help='横向重心偏移 mm')
    parser.add_argument('--cog-depth', type=float, default=30.0,
                        help='桨平面到重心高度 mm')
    parser.add_argument('--thrust-max', type=float, default=6.0,
                        help='单电机最大拉力 N')
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
