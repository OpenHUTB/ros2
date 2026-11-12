"""命令行入口: 打印缩比换算结果, 并写出 report.json（只用标准库）."""
import argparse
import json
import sys
from pathlib import Path

from .core import UsvFroude


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
    m = report['model']
    s = report['ship']
    p = report['power']
    params = report['parameters']
    print('==> usv_froude_scaling')
    print('    参数: 实船 %.2f m, %.2f m/s (Fr=%.3f), 缩尺比 %.0f, 模型实测阻力 %.2f N'
          % (params['ship_length_m'], params['ship_speed_m_s'], s['froude_number'],
             params['scale'], params['model_resistance_n']))
    print('    模型: 长 %.2f m, 速度 %.3f m/s, 摩擦 %.3f N, 剩余 %.3f N'
          % (m['length_m'], m['speed_m_s'], m['friction_resistance_n'],
             m['residuary_resistance_n']))
    print('    实船: Re %.2e, 摩擦系数 %.5f, 摩擦 %.1f N, 总阻力 %.1f N'
          % (s['reynolds'], s['friction_coef'], s['friction_resistance_n'],
             s['total_resistance_n']))
    print('    功率: 有效 %.1f W, 轴功率 %.1f W (推进效率 %.2f)'
          % (p['effective_w'], p['delivered_w'], params['propulsive_efficiency']))
    for warning in report['warnings']:
        print('    [提示] %s' % warning)


def build_check(args):
    return UsvFroude(ship_speed_m_s=args.speed, ship_length_m=args.length,
                     scale=args.scale, wetted_area_ship_m2=args.wetted_area,
                     model_resistance_n=args.model_resistance,
                     propulsive_efficiency=args.efficiency, rho=args.rho, nu=args.nu)


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='USV 弗劳德缩比换算与拖曳功率估算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--speed', type=float, default=5.0, help='实船航速 m/s')
    parser.add_argument('--length', type=float, default=10.0, help='实船水线长 m')
    parser.add_argument('--scale', type=float, default=10.0, help='缩尺比 lambda')
    parser.add_argument('--wetted-area', type=float, default=22.0,
                        help='实船湿表面积 m^2')
    parser.add_argument('--model-resistance', type=float, default=2.5,
                        help='船模实测总阻力 N')
    parser.add_argument('--efficiency', type=float, default=0.55,
                        help='准推进效率')
    parser.add_argument('--rho', type=float, default=1025.0, help='水密度 kg/m^3')
    parser.add_argument('--nu', type=float, default=1.19e-6, help='运动黏度 m^2/s')
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
