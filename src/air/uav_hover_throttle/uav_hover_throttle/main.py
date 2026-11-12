"""离线入口：悬停油门求解，以及载重/海拔/电池电压的影响。

不需要 ROS 2，只用 Python 标准库::

    cd src/air/uav_hover_throttle
    python3 -m uav_hover_throttle.main
"""

import argparse
import csv
import os
from pathlib import Path

from .rotor import Rotor, air_density

DEFAULT_MASS = 1.6
DEFAULT_ALTITUDES = (0.0, 1000.0, 2000.0, 3000.0, 4000.0)
DEFAULT_CELL_VOLTAGES = (4.20, 4.00, 3.85, 3.70, 3.50)
DEFAULT_CELLS = 4


def build_parser():
    parser = argparse.ArgumentParser(
        prog='uav_hover_throttle.main',
        description='四旋翼悬停油门求解（推力曲线 + 海拔 + 电池电压）')
    parser.add_argument('--mass', type=float, default=DEFAULT_MASS, help='起飞质量（kg）')
    parser.add_argument('--cells', type=int, default=DEFAULT_CELLS, help='电池串数')
    parser.add_argument('--diameter', type=float, default=0.254, help='桨径（m）')
    parser.add_argument('--rotors', type=int, default=4, help='旋翼数')
    parser.add_argument('--output-dir', default=None,
                        help='CSV 输出目录，默认 ~/.ros/uav_hover_throttle')
    return parser


def default_output_dir():
    return (Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
            / 'uav_hover_throttle')


def print_table(title, rows, columns):
    print(f'\n{title}')
    header = ''.join(f'{name:>{width}}' for name, width in columns)
    print(header)
    print('-' * len(header.encode('gbk')))
    for row in rows:
        print(''.join(f'{value:>{width}}' for value, (_n, width) in
                      zip(row, columns)))


def main(argv=None):
    args = build_parser().parse_args(argv)
    output_dir = Path(args.output_dir) if args.output_dir else default_output_dir()

    rotor = Rotor(diameter_m=args.diameter)
    nominal_voltage = rotor.nominal_voltage
    print(f'机体：{args.rotors} 旋翼，桨径 {rotor.diameter * 1000:.0f} mm，'
          f'起飞质量 {args.mass:.2f} kg（{args.mass * 9.80665:.2f} N）')
    print(f'推力模型 T = Ct*rho*n^2*D^4，Ct={rotor.ct:.3f}，'
          f'悬停功率按动量理论 + 型阻，figure of merit={rotor.fom:.2f}')
    print(f'电机：额定 {rotor.nominal_voltage:.1f} V 时 {rotor.nominal_rpm:.0f} rpm'
          f'（{args.cells}S，单节 {nominal_voltage / args.cells:.2f} V）')
    print(f'海平面空气密度 {air_density(0.0):.4f} kg/m³，'
          f'4000 m 处 {air_density(4000.0):.4f} kg/m³')

    rows = []
    summary = []

    base = rotor.hover_report(args.mass, nominal_voltage, 0.0, args.rotors)
    print(f'\n基准状态（海平面、满电 {nominal_voltage:.1f} V）')
    print(f'  悬停转速 {base["rpm_hover"]:.0f} rpm，'
          f'最大转速 {base["rpm_max"]:.0f} rpm，'
          f'悬停油门 {base["throttle"] * 100:.1f}%')
    print(f'  悬停推力 {base["thrust_n"]:.2f} N（重量 {base["weight_n"]:.2f} N），'
          f'电功率 {base["power_w"]:.1f} W = {base["power_per_kg_w"]:.1f} W/kg')

    print_table(
        '载重变化（海平面、满电）',
        [[f'{mass:.2f} kg',
          f'{rotor.hover_report(mass, nominal_voltage, 0.0, args.rotors)["rpm_hover"]:.0f}',
          f'{rotor.hover_report(mass, nominal_voltage, 0.0, args.rotors)["throttle"] * 100:.1f}%',
          f'{rotor.hover_report(mass, nominal_voltage, 0.0, args.rotors)["power_w"]:.1f}']
         for mass in (args.mass * 0.8, args.mass, args.mass * 1.2, args.mass * 1.4)],
        columns=[('起飞质量', 12), ('悬停转速', 12), ('悬停油门', 12), ('电功率(W)', 12)])

    print_table(
        '海拔影响（满电、基准质量）',
        [[f'{altitude:.0f} m',
          f'{air_density(altitude):.4f}',
          f'{rotor.hover_report(args.mass, nominal_voltage, altitude, args.rotors)["rpm_hover"]:.0f}',
          f'{rotor.hover_report(args.mass, nominal_voltage, altitude, args.rotors)["throttle"] * 100:.1f}%',
          f'{rotor.hover_report(args.mass, nominal_voltage, altitude, args.rotors)["power_w"]:.1f}']
         for altitude in DEFAULT_ALTITUDES],
        columns=[('海拔', 10), ('密度', 10), ('悬停转速', 12),
                 ('悬停油门', 12), ('电功率(W)', 12)])

    print_table(
        f'电池衰减影响（海平面、{args.cells}S）',
        [[f'{voltage * args.cells:.2f} V', f'{voltage:.2f} V/节',
          f'{rotor.hover_report(args.mass, voltage * args.cells, 0.0, args.rotors)["rpm_max"]:.0f}',
          f'{rotor.hover_report(args.mass, voltage * args.cells, 0.0, args.rotors)["throttle"] * 100:.1f}%']
         for voltage in DEFAULT_CELL_VOLTAGES],
        columns=[('总电压', 12), ('单节', 12), ('可用最大转速', 14), ('悬停油门', 12)])

    for altitude in DEFAULT_ALTITUDES:
        report = rotor.hover_report(args.mass, nominal_voltage, altitude, args.rotors)
        rows.append(report)
        summary.append({'kind': 'altitude', **report})
    for voltage in DEFAULT_CELL_VOLTAGES:
        report = rotor.hover_report(args.mass, voltage * args.cells, 0.0, args.rotors)
        rows.append(report)
        summary.append({'kind': 'voltage', **report})

    max_altitude = None
    for altitude in range(0, 8000, 100):
        report = rotor.hover_report(args.mass, nominal_voltage, float(altitude),
                                    args.rotors)
        if not report['feasible']:
            max_altitude = altitude
            break
    if max_altitude is None:
        print('\n满电状态下 8000 m 以内都能悬停（本模型范围）')
    else:
        print(f'\n满电最大悬停高度约 {max_altitude} m（再高油门就打满）')

    weakest = rotor.hover_report(args.mass, DEFAULT_CELL_VOLTAGES[-1] * args.cells,
                                 0.0, args.rotors)
    print(f'电量最低（单节 {DEFAULT_CELL_VOLTAGES[-1]:.2f} V）时悬停油门 '
          f'{weakest["throttle"] * 100:.1f}%，'
          f'{"仍可悬停" if weakest["feasible"] else "已无法悬停——需要减载"}')

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'hover_analysis.csv'
    with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary[0].keys()))
        writer.writeheader()
        for item in summary:
            writer.writerow(item)
    print(f'\n分析结果已写入 {csv_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
