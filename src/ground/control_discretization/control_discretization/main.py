"""离线入口：四种离散化方法在二阶系统阶跃响应上的精度与稳定性对比。

不需要 ROS 2，只用 Python 标准库::

    cd src/ground/control_discretization
    python3 -m control_discretization.main
"""

import argparse
import csv
import os
from pathlib import Path

from .systems import (METHOD_LABELS, METHODS, SecondOrderSystem,
                      critical_period, discrete_poles, simulate_discrete)

DEFAULT_TS = (0.01, 0.05, 0.1, 0.2)
DURATION = 6.0


def format_pole(pole):
    if abs(pole.imag) < 1e-9:
        return f'{pole.real:+.3f}'
    sign = '+' if pole.imag >= 0 else '-'
    return f'{pole.real:+.3f}{sign}{abs(pole.imag):.3f}j'


def evaluate(system, method, ts, duration=DURATION):
    reference = system.sample_continuous(ts, duration)
    samples, diverged = simulate_discrete(system, method, ts, duration)
    poles = discrete_poles(system, method, ts)
    stable = all(abs(pole) < 1.0 for pole in poles)

    if diverged or not samples:
        return {'max_error': None, 'steady_error': None, 'poles': poles,
                'stable': False, 'diverged': True, 'samples': samples,
                'reference': reference}

    length = min(len(samples), len(reference))
    errors = [abs(samples[index][1] - reference[index][1]) for index in range(length)]
    max_error = max(errors) if errors else None
    steady_error = abs(samples[length - 1][1] - reference[length - 1][1])
    return {'max_error': max_error, 'steady_error': steady_error, 'poles': poles,
            'stable': stable, 'diverged': diverged, 'samples': samples,
            'reference': reference}


def build_parser():
    parser = argparse.ArgumentParser(
        prog='control_discretization.main',
        description='连续二阶系统在四种离散化方法下的精度与稳定性对比')
    parser.add_argument('--wn', type=float, default=3.0, help='自然频率（rad/s）')
    parser.add_argument('--zeta', type=float, default=0.4, help='阻尼比（0~1）')
    parser.add_argument('--ts', type=float, nargs='+', default=list(DEFAULT_TS),
                        help='采样周期列表（秒）')
    parser.add_argument('--duration', type=float, default=DURATION, help='仿真时长（秒）')
    parser.add_argument('--output-dir', default=None,
                        help='CSV 输出目录，默认 ~/.ros/control_discretization')
    return parser


def default_output_dir():
    return (Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
            / 'control_discretization')


def main(argv=None):
    args = build_parser().parse_args(argv)
    future = default_output_dir()
    output_dir = Path(args.output_dir) if args.output_dir else future

    system = SecondOrderSystem(wn=args.wn, zeta=args.zeta)
    print(f'连续对象 G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)，'
          f'wn={system.wn:.2f} rad/s，zeta={system.zeta:.2f}')
    print(f'参考响应：RK4 细网格积分；仿真时长 {args.duration:.1f} s')

    rows = []
    for ts in args.ts:
        header = (f'\n采样周期 Ts = {ts:.3f} s')
        print(header)
        print(f"{'方法':<16}{'极点1':>16}{'极点2':>16}{'稳定':>6}"
              f"{'最大误差':>12}{'稳态误差':>12}")
        print('-' * 80)
        for method in METHODS:
            result = evaluate(system, method, ts, args.duration)
            poles = result['poles']
            stable = '是' if result['stable'] else '否'
            max_error = ('发散' if result['max_error'] is None
                         else f"{result['max_error']:.5f}")
            steady = ('发散' if result['steady_error'] is None
                      else f"{result['steady_error']:.5f}")
            print(f"{METHOD_LABELS[method]:<16}{format_pole(poles[0]):>16}"
                  f"{format_pole(poles[1]):>16}{stable:>6}{max_error:>12}{steady:>12}")
            rows.append({
                'ts': ts,
                'method': method,
                'label': METHOD_LABELS[method],
                'pole1': format_pole(poles[0]),
                'pole2': format_pole(poles[1]),
                'stable': int(result['stable']),
                'max_error': result['max_error'],
                'steady_error': result['steady_error'],
            })

    print('\n临界采样周期（从 1 ms 扫到 2 s，首次失去稳定性的 Ts）')
    print(f"{'方法':<16}{'临界 Ts':>14}{'对应 wn*Ts':>14}")
    print('-' * 46)
    for method in METHODS:
        ts_critical = critical_period(system, method)
        if ts_critical is None:
            print(f'{METHOD_LABELS[method]:<16}{"始终稳定":>14}{"—":>14}')
            rows.append({'ts': None, 'method': method,
                         'label': METHOD_LABELS[method], 'pole1': '',
                         'pole2': '', 'stable': 1, 'max_error': None,
                         'steady_error': None})
        else:
            print(f'{METHOD_LABELS[method]:<16}{ts_critical:>13.4f}s'
                  f'{ts_critical * system.wn:>14.3f}')

    print('\nZOH 在采样点上与连续参考的偏差（理论应为 0）')
    for ts in args.ts:
        result = evaluate(system, 'zoh', ts, args.duration)
        print(f'  Ts={ts:.3f}s  最大误差 {result["max_error"]:.2e}  '
              f'稳态误差 {result["steady_error"]:.2e}')

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'discretization_compare.csv'
    with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            'ts', 'method', 'label', 'pole1', 'pole2', 'stable',
            'max_error', 'steady_error'])
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print(f'\n对比结果已写入 {csv_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
