"""命令行入口: 打印浮力、配平与初稳性结果, 并写出 report.json.

只用标准库（math/json/argparse），不依赖 NumPy/Matplotlib/ROS。
"""
import argparse
import json
import sys
from pathlib import Path

from .core import (
    ADDED_MASS_FACTOR,
    GM_MIN,
    GM_RECOMMENDED,
    G,
    RHO_NOMINAL,
    TRIM_LIMIT_DEG,
    RovTrim,
)

LEVEL_TEXT = {
    'ok': '正常',
    'marginal': '注意',
    'unstable': '失稳',
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


def build_trim(args):
    """按命令行参数建立核算对象."""
    return RovTrim(length_m=args.length, beam_m=args.beam, height_m=args.height,
                   density=None if args.density is None else args.density,
                   gravity=G, tray_limit_x_m=args.tray_x,
                   tray_limit_y_m=args.tray_y, tray_z_m=args.tray_z,
                   added_mass_factor=args.added_mass)


def print_report(report):
    """打印成可对照的表（全部使用 GBK 可编码字符）."""
    p = report['parameters']
    m = report['mass']
    b = report['buoyancy']
    cog_ = report['cog']
    cob = report['cob']
    s = report['stability_dry']
    trim = report['trim_dry']
    plan = report['ballast_plan']
    after = report['state_after_ballast']
    tol = report['tolerance']

    print('==> rov_ballast_trim')
    print('    外壳: %.3f x %.3f x %.3f m, 排水 %.4f m^3 = %.2f kg (rho=%.1f kg/m^3)'
          % (p['length_m'], p['beam_m'], p['height_m'], p['volume_m3'],
             m['displacement_kg'], p['density_kg_m3']))
    print('    干重: %.2f kg, 重心 (%.3f, %.3f, %.3f) m, 浮心 (%.3f, %.3f, %.3f) m'
          % (m['dry_kg'], cog_['x_m'], cog_['y_m'], cog_['z_m'],
             cob['x_m'], cob['y_m'], cob['z_m']))
    print('    浮力/重量: %.1f N / %.1f N, 净浮力 %+.1f N (%+.2f%%)'
          % (b['buoyancy_n'], b['weight_dry_n'], b['net_buoyancy_n'],
             b['net_buoyancy_pct']))
    print('    压载需求: %.2f kg' % m['required_ballast_kg'])
    print('    空载倾角: 横倾 %+.2f deg, 纵倾 %+.2f deg'
          % (trim['roll_deg'], trim['pitch_deg']))
    print('    空载初稳心高: %.3f m (%s)'
          % (s['gm_m'], LEVEL_TEXT[s['gm_level']]))

    if plan['mass_kg'] <= 0.0:
        print('    压载方案: 无需压载 (%s)' % plan['reason'])
    else:
        print('    压载方案: %.2f kg @ (x=%+.3f, y=%+.3f, z=%+.3f) m  需要 '
              '(x=%+.3f, y=%+.3f, z=%+.3f) m  %s'
              % (plan['mass_kg'], plan['x_m'], plan['y_m'], plan['z_m'],
                 plan['x_required_m'], plan['y_required_m'],
                 plan['z_required_m'], '可行' if plan['feasible'] else '不可行'))
        if not plan['feasible']:
            print('    不可行原因: %s' % plan['reason'])

    if after:
        period = after['roll_period_s']
        period_text = '%.2f s' % period if period else 'inf'
        print('    配平后: 横倾 %+.2f deg, 纵倾 %+.2f deg, GM %.3f m (%s), '
              '横摇周期 %s'
              % (after['roll_deg'], after['pitch_deg'], after['gm_m'],
                 LEVEL_TEXT[after['gm_level']], period_text))
        print('    质量/净浮力: %.2f kg, %+.1f N' % (after['mass_kg'],
                                                    after['net_buoyancy_n']))
    if tol:
        print('    容差: %.1f deg 倾角对应横向偏心 %.4f m (%.1f mm); '
              '%.1f deg 扰动对应力矩 %.2f N*m'
              % (tol['trim_limit_deg'], tol['offset_m_for_trim'],
                 tol['offset_m_for_trim'] * 1000.0, tol['disturb_limit_deg'],
                 tol['max_disturbance_moment_n_m']))

    print('    深度影响:')
    for row in report['depth_sweep']:
        print('      %6.0f m  rho=%.2f kg/m^3  净浮力 %+7.2f N  需补压载 %+6.3f kg'
              % (row['depth_m'], row['density_kg_m3'], row['net_buoyancy_n'],
                 row['extra_ballast_kg']))

    print('    判定门限: GM>=%.3f m 正常, %.3f~%.3f m 注意, <=0 失稳; '
          '静止倾角 <=%.1f deg' % (GM_RECOMMENDED, GM_MIN, GM_RECOMMENDED,
                                  TRIM_LIMIT_DEG))
    for warning in report['warnings']:
        print('    [提示] %s' % warning)


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='ROV 浮力-压载配平与初稳性核算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--length', type=float, default=0.90, help='外壳长度 m (x)')
    parser.add_argument('--beam', type=float, default=0.70, help='外壳宽度 m (y)')
    parser.add_argument('--height', type=float, default=0.50, help='外壳高度 m (z)')
    parser.add_argument('--density', type=float, default=None,
                        help='海水密度 kg/m^3, 缺省按 10 C/35 psu 表层取 %.1f'
                             % RHO_NOMINAL)
    parser.add_argument('--tray-x', type=float, default=0.40, help='压载托盘纵向限位 m')
    parser.add_argument('--tray-y', type=float, default=0.30, help='压载托盘横向限位 m')
    parser.add_argument('--tray-z', type=float, default=-0.18, help='压载托盘高度 m')
    parser.add_argument('--added-mass', type=float, default=ADDED_MASS_FACTOR,
                        help='横摇附加惯量系数')
    parser.add_argument('--target-gm', type=float, default=None,
                        help='目标初稳心高 m, 给定后反解压载安装高度')
    args = parser.parse_args(argv)

    trim = build_trim(args)
    report = trim.report(target_gm_m=args.target_gm)
    print_report(report)

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print()
    print('已写入: report.json')
    print('输出目录: %s' % args.output.resolve())
    return 0
