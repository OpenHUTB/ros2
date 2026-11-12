"""检查驱动选型的闭式解与边界；期望值全部可以手算.

测试不依赖 pytest 专有接口：`pytest test -v` 与 `python test/test_diff_drive_powertrain_sizing.py`
两种方式都能跑。
"""
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from diff_drive_powertrain_sizing import cli  # noqa: E402
from diff_drive_powertrain_sizing.core import (  # noqa: E402
    G,
    PowertrainCheck,
    accel_force_n,
    drag_force_n,
    grade_force_n,
    rolling_force_n,
    torque_level,
    wheel_rpm,
)
from diff_drive_powertrain_sizing.cli import print_report  # noqa: E402


def close(value, expected, tol=1e-9):
    return abs(value - expected) <= max(tol, abs(expected) * 1e-9)


def raises(callable_):
    try:
        callable_()
    except ValueError:
        return True
    return False


def test_force_formulas():
    assert close(rolling_force_n(12.0, 0.03), 0.03 * 12.0 * G)
    assert close(grade_force_n(12.0, 10.0), 12.0 * G * math.sin(math.radians(10.0)))
    assert close(accel_force_n(12.0, 1.0, 1.08), 12.96)
    assert close(drag_force_n(1.5, 0.8, 0.06), 0.5 * 1.225 * 0.8 * 0.06 * 2.25)
    assert close(rolling_force_n(12.0, 0.03), 3.5304, 1e-4)


def test_wheel_rpm_and_torque_level():
    assert close(wheel_rpm(1.5, 0.06), 1.5 / (2 * math.pi * 0.06) * 60.0)
    assert close(wheel_rpm(1.5, 0.06), 238.732, 1e-2)
    assert torque_level(1.0, 1.2, 2.5) == 'ok'
    assert torque_level(1.2, 1.2, 2.5) == 'ok'
    assert torque_level(1.3, 1.2, 2.5) == 'marginal'
    assert torque_level(2.5, 1.2, 2.5) == 'marginal'
    assert torque_level(2.6, 1.2, 2.5) == 'over'


def test_default_quantities():
    check = PowertrainCheck()
    forces = check.forces()
    assert close(forces['rolling_n'], 3.5304, 1e-4)
    assert close(forces['grade_n'], 20.4349, 1e-4)
    assert close(forces['accel_n'], 12.96)
    assert close(forces['drag_n'], 0.06615, 1e-5)
    total = sum(forces.values())
    assert close(check.total_force_n(), total)
    assert close(total, 36.9914, 1e-4)
    wheel = check.wheel_quantities()
    assert close(wheel['torque_nm'], total * 0.06 / 2 / 0.85, 1e-9)
    assert close(wheel['torque_nm'], 1.3056, 1e-4)
    assert close(wheel['rpm'], 238.732, 1e-2)
    assert close(wheel['power_w'], total * 1.5 / 2 / 0.85, 1e-9)
    assert close(wheel['power_w'], 32.6395, 1e-4)


def test_motor_match_direct_vs_geared():
    check = PowertrainCheck()
    motor = check.motor_match()
    assert close(motor['gear_ratio'], 4000.0 / wheel_rpm(1.5, 0.06), 1e-12)
    assert close(motor['gear_ratio'], 16.755, 1e-3)
    # 直接驱动：1.536 N*m 介于持续 1.2 与峰值 2.5 之间 -> marginal
    assert close(motor['torque_direct_nm'], 1.536, 1e-3)
    assert motor['torque_direct_level'] == 'marginal'
    # 减速后：0.0917 N*m -> ok
    assert close(motor['torque_geared_nm'], 0.09167, 1e-4)
    assert motor['torque_geared_level'] == 'ok'
    assert close(motor['current_per_motor_a'], 32.6395 / 24.0, 1e-6)
    assert motor['current_per_motor_a'] < 2.0


def test_report_is_consistent():
    check = PowertrainCheck()
    report = check.report()
    assert close(report['total_force_n'], sum(report['forces'].values()), 1e-12)
    wheel = report['wheel']
    assert close(wheel['torque_nm'],
                 report['total_force_n'] * 0.06 / 2 / 0.85, 1e-12)
    assert report['motor']['torque_direct_level'] == 'marginal'
    assert any('减速' in text for text in report['warnings'])
    assert any('附着力' in text for text in report['notes'])
    json.dumps(report)


def test_invalid_parameters_are_rejected():
    base = {'mass_kg': 12.0, 'wheel_radius_m': 0.06, 'driven_wheels': 2,
            'rolling_coef': 0.03, 'grade_deg': 10.0, 'accel_m_s2': 1.0,
            'v_cruise_m_s': 1.5, 'drag_coef': 0.8, 'front_area_m2': 0.06,
            'drive_efficiency': 0.85, 'spin_factor': 1.08,
            'motor_cont_torque_nm': 1.2, 'motor_peak_torque_nm': 2.5,
            'motor_rated_rpm': 4000.0, 'bus_voltage_v': 24.0}
    for kwargs in ({'mass_kg': 0.0}, {'wheel_radius_m': 0.0}, {'driven_wheels': 0},
                   {'grade_deg': 46.0}, {'grade_deg': -1.0}, {'drive_efficiency': 1.2},
                   {'drive_efficiency': 0.0}, {'spin_factor': 0.9},
                   {'motor_peak_torque_nm': 1.0}, {'v_cruise_m_s': 0.0},
                   {'bus_voltage_v': 0.0}):
        params = dict(base)
        params.update(kwargs)
        assert raises(lambda: PowertrainCheck(**params)), kwargs
    assert raises(lambda: rolling_force_n(0.0, 0.03))
    assert raises(lambda: grade_force_n(12.0, 90.0))
    assert raises(lambda: accel_force_n(12.0, -1.0, 1.08))
    assert raises(lambda: drag_force_n(-1.0, 0.8, 0.06))
    assert raises(lambda: wheel_rpm(1.0, 0.0))
    assert raises(lambda: torque_level(1.0, 0.0, 2.0))


def test_print_report_is_gbk_safe():
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        print_report(PowertrainCheck().report())
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'diff_drive_powertrain_sizing' in text
    assert 'N*m' in text


def test_cli_smoke_writes_report():
    output = Path(__file__).resolve().parent / '_out'
    assert cli.main(['--output', str(output)]) == 0
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert close(report['wheel']['torque_nm'], 1.3056, 1e-4)
    assert report['motor']['torque_geared_level'] == 'ok'


def main():
    failures = []
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]
    for name, func in tests:
        try:
            func()
        except AssertionError as error:
            failures.append('%s: %s' % (name, error))
            print('FAIL %s' % name)
        else:
            print('ok   %s' % name)
    print()
    print('%d passed, %d failed' % (len(tests) - len(failures), len(failures)))
    for item in failures:
        print('  - ' + item)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
