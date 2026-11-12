"""命令行入口: 打印驱动选型结果, 并写出 report.json（只用标准库）."""
import argparse
import json
import sys
from pathlib import Path

from .core import PowertrainCheck

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
    f = report['forces']
    w = report['wheel']
    m = report['motor']
    p = report['parameters']
    print('==> diff_drive_powertrain_sizing')
    print('    工况: %.1f kg, 轮径 %.3f m, %d 轮驱动, 坡度 %.0f deg, '
          '加速 %.2f m/s^2, 巡航 %.2f m/s'
          % (p['mass_kg'], p['wheel_radius_m'], p['driven_wheels'], p['grade_deg'],
             p['accel_m_s2'], p['v_cruise_m_s']))
    print('    阻力: 滚动 %.2f N, 坡度 %.2f N, 加速 %.2f N, 风阻 %.3f N, 合计 %.2f N'
          % (f['rolling_n'], f['grade_n'], f['accel_n'], f['drag_n'],
             report['total_force_n']))
    print('    轮端: 转矩 %.3f N*m, 轮速 %.1f rpm, 电功率 %.2f W/轮'
          % (w['torque_nm'], w['rpm'], w['power_w']))
    print('    直接驱动: 电机转矩 %.3f N*m (%s), 持续 %.2f / 峰值 %.2f N*m'
          % (m['torque_direct_nm'], LEVEL_TEXT[m['torque_direct_level']],
             p['motor_cont_torque_nm'], p['motor_peak_torque_nm']))
    print('    减速方案: 减速比 %.2f, 电机转矩 %.3f N*m (%s), 母线电流 %.2f A/轮'
          % (m['gear_ratio'], m['torque_geared_nm'], LEVEL_TEXT[m['torque_geared_level']],
             m['current_per_motor_a']))
    for warning in report['warnings']:
        print('    [提示] %s' % warning)


def build_check(args):
    return PowertrainCheck(
        mass_kg=args.mass, wheel_radius_m=args.wheel_radius,
        driven_wheels=args.driven_wheels, rolling_coef=args.rolling_coef,
        grade_deg=args.grade, accel_m_s2=args.accel, v_cruise_m_s=args.v_cruise,
        drag_coef=args.drag_coef, front_area_m2=args.front_area,
        drive_efficiency=args.efficiency, spin_factor=args.spin_factor,
        motor_cont_torque_nm=args.motor_cont_torque,
        motor_peak_torque_nm=args.motor_peak_torque,
        motor_rated_rpm=args.motor_rated_rpm, bus_voltage_v=args.bus_voltage)


def main(argv=None):
    _configure_console()
    parser = argparse.ArgumentParser(description='差速底盘驱动系统选型核算')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--mass', type=float, default=12.0, help='整车质量 kg')
    parser.add_argument('--wheel-radius', type=float, default=0.06, help='轮半径 m')
    parser.add_argument('--driven-wheels', type=int, default=2, help='驱动轮数量')
    parser.add_argument('--rolling-coef', type=float, default=0.03,
                        help='滚动阻力系数')
    parser.add_argument('--grade', type=float, default=10.0, help='坡度 deg')
    parser.add_argument('--accel', type=float, default=1.0, help='目标加速度 m/s^2')
    parser.add_argument('--v-cruise', type=float, default=1.5, help='巡航速度 m/s')
    parser.add_argument('--drag-coef', type=float, default=0.8, help='风阻系数')
    parser.add_argument('--front-area', type=float, default=0.06, help='迎风面积 m^2')
    parser.add_argument('--efficiency', type=float, default=0.85, help='传动效率')
    parser.add_argument('--spin-factor', type=float, default=1.08,
                        help='旋转惯量系数')
    parser.add_argument('--motor-cont-torque', type=float, default=1.2,
                        help='电机持续转矩 N*m')
    parser.add_argument('--motor-peak-torque', type=float, default=2.5,
                        help='电机峰值转矩 N*m')
    parser.add_argument('--motor-rated-rpm', type=float, default=4000.0,
                        help='电机额定转速 rpm')
    parser.add_argument('--bus-voltage', type=float, default=24.0, help='母线电压 V')
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
