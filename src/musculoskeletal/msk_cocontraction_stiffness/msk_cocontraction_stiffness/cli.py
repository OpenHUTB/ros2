"""命令行入口: 打印共收缩与刚度结果, 并写出 report.json（只用标准库）."""
import argparse
import json
import sys
from pathlib import Path

from .core import MskCocontraction

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
    s = report['solution']
    k = report['stiffness']
    print('==> msk_cocontraction_stiffness')
    print('    参数: 目标力矩 %.1f N*m, 力臂 %.2f m, F_max %.0f N, 静息长 %.2f m, '
          'q=%d, 共收缩 %.2f'
          % (p['torque_nm'], p['moment_arm_m'], p['f_max_n'], p['slack_length_m'],
             p['q_factor'], p['coactivation']))
    if s['feasible']:
        print('    肌力: 伸肌 %.1f N (激活 %.2f), 屈肌 %.1f N (激活 %.2f), '
              '上限 %.1f N*m (%s)'
              % (s['force_agonist_n'], s['activation_agonist'],
                 s['force_antagonist_n'], s['activation_antagonist'],
                 s['torque_max_nm'], LEVEL_TEXT[report['level']]))
        print('    关节刚度: %.2f N*m/rad, %.1f N*m 扰动 -> %.2f deg 偏转'
              % (k['joint_nm_per_rad'], p['disturbance_nm'], k['deflection_deg']))
    else:
        print('    不可行: 目标力矩超过上限 %.1f N*m' % s['torque_max_nm'])
    for warning in report['warnings']:
        print('    [提示] %s' % warning)


def build_check(args):
    return MskCocontraction(torque_nm=args.torque, moment_arm_m=args.moment_arm,
                            f_max_n=args.f_max, slack_length_m=args.slack_length,
                            q_factor=args.q_factor, coactivation=args.coactivation,
                            disturbance_nm=args.disturbance)


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='拮抗肌共收缩与关节刚度核算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--torque', type=float, default=8.0, help='目标净力矩 N*m')
    parser.add_argument('--moment-arm', type=float, default=0.03, help='肌力臂 m')
    parser.add_argument('--f-max', type=float, default=600.0, help='最大等长力 N')
    parser.add_argument('--slack-length', type=float, default=0.25,
                        help='肌肉静息长度 m')
    parser.add_argument('--q-factor', type=float, default=25.0, help='肌刚度系数 q')
    parser.add_argument('--coactivation', type=float, default=0.3,
                        help='共收缩水平 [0,1]')
    parser.add_argument('--disturbance', type=float, default=2.0,
                        help='扰动力矩 N*m')
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
