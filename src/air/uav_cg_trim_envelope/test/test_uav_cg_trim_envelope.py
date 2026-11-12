"""检查重心偏移配平的闭式解与包线；期望值全部可以手算.

`pytest test -v` 与 `python test/test_uav_cg_trim_envelope.py` 两种方式都能跑。
"""
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_cg_trim_envelope import cli  # noqa: E402
from uav_cg_trim_envelope.core import (  # noqa: E402
    G,
    UavCgTrim,
    margin_pct,
    per_motor_thrusts,
    saturation_envelope,
    trim_tilt_deg,
)
from uav_cg_trim_envelope.cli import print_report  # noqa: E402


def close(value, expected, tol=1e-9):
    return abs(value - expected) <= max(tol, abs(expected) * 1e-9)


def raises(callable_):
    try:
        callable_()
    except ValueError:
        return True
    return False


def test_symmetric_cog_gives_equal_thrusts():
    thrusts = per_motor_thrusts(1.2, 0.25, 0.25, 0.0, 0.0)
    assert all(close(t, 1.2 * G / 4.0) for t in thrusts)
    assert close(sum(thrusts), 1.2 * G, 1e-9)


def test_forward_offset_loads_front_rear_pair():
    thrusts = per_motor_thrusts(1.2, 0.25, 0.25, 0.003, 0.0)
    weight = 1.2 * G
    assert close(thrusts[0] - thrusts[2], weight * 0.003 / 0.25, 1e-12)
    assert close(thrusts[1], thrusts[3], 1e-12)
    assert close(sum(thrusts), weight, 1e-9)


def test_default_thrusts_spot_values():
    thrusts = per_motor_thrusts(1.2, 0.25, 0.25, 0.003, 0.002)
    assert close(thrusts[0], 3.0126, 1e-4)
    assert close(thrusts[2], 2.8714, 1e-4)
    assert close(thrusts[1], 2.9891, 1e-4)
    assert close(thrusts[3], 2.8949, 1e-4)
    assert close(sum(thrusts), 1.2 * G, 1e-9)


def test_tilt_angle_formula():
    theta, phi = trim_tilt_deg(0.003, 0.002, 0.03)
    assert close(theta, math.degrees(math.atan2(0.003, 0.03)), 1e-12)
    assert close(phi, math.degrees(math.atan2(0.002, 0.03)), 1e-12)
    assert close(theta, 5.7106, 1e-3)
    assert close(phi, 3.8141, 1e-3)
    assert trim_tilt_deg(0.0, 0.0, 0.03) == (0.0, 0.0)


def test_envelope_hand_calc():
    envelope = saturation_envelope(1.2, 0.25, 0.25, 6.0)
    weight = 1.2 * G
    expected = 2.0 * 0.25 * (6.0 / weight - 0.25)
    assert close(envelope['dx_max_m'], expected, 1e-12)
    assert close(envelope['dy_max_m'], expected, 1e-12)
    assert close(envelope['dx_max_m'], 0.12993, 1e-5)
    assert envelope['hover_feasible'] is True


def test_envelope_boundaries():
    # 单电机上限恰好等于 1/4 重量：excess=0，任何偏移都无法配平
    weight = 1.2 * G
    at_limit = saturation_envelope(1.2, 0.25, 0.25, weight / 4.0)
    assert close(at_limit['dx_max_m'], 0.0, 1e-12)
    assert at_limit['hover_feasible'] is False
    below = saturation_envelope(1.2, 0.25, 0.25, weight / 5.0)
    assert below['hover_feasible'] is False
    assert close(below['dx_max_m'], 0.0, 1e-12)


def test_default_report():
    report = UavCgTrim().report()
    assert report['level'] == 'ok'
    assert close(report['max_motor_thrust_n'], 3.0126, 1e-4)
    assert close(report['trim_tilt_deg']['pitch'], 5.7106, 1e-3)
    assert close(report['envelope']['dx_max_m'], 0.12993, 1e-5)
    assert close(report['margin_pct']['pitch'], 97.69, 1e-2)
    assert close(report['margin_pct']['roll'], 98.46, 1e-2)
    assert any('配平可行' in text for text in report['warnings'])
    assert any('plus' in text for text in report['notes'])
    json.dumps(report)


def test_offset_beyond_envelope_is_over():
    check = UavCgTrim(dx_m=0.20, dy_m=0.0)
    report = check.report()
    assert report['level'] == 'over'
    assert report['max_motor_thrust_n'] > 6.0
    assert any('超出包线' in text for text in report['warnings'])


def test_insufficient_thrust_is_infeasible():
    check = UavCgTrim(thrust_max_n=1.2 * G / 5.0, dx_m=0.0, dy_m=0.0)
    report = check.report()
    assert report['level'] == 'over'
    assert report['envelope']['hover_feasible'] is False
    assert any('无法悬停' in text for text in report['warnings'])


def test_margin_pct():
    assert close(margin_pct(0.0, 0.1), 100.0)
    assert close(margin_pct(0.03, 0.1), 70.0, 1e-9)
    assert close(margin_pct(0.1, 0.1), 0.0, 1e-9)
    assert close(margin_pct(0.2, 0.1), 0.0, 1e-9)


def test_invalid_parameters_are_rejected():
    for kwargs in ({'mass_kg': 0.0}, {'arm_x_m': 0.0}, {'arm_y_m': -1.0},
                   {'cog_depth_m': 0.0}, {'thrust_max_n': 0.0},
                   {'dx_m': float('nan')}):
        base = {'mass_kg': 1.2, 'arm_x_m': 0.25, 'arm_y_m': 0.25, 'dx_m': 0.003,
                'dy_m': 0.002, 'cog_depth_m': 0.03, 'thrust_max_n': 6.0}
        base.update(kwargs)
        assert raises(lambda: UavCgTrim(**base)), kwargs
    assert raises(lambda: per_motor_thrusts(0.0, 0.25, 0.25, 0.0, 0.0))
    assert raises(lambda: trim_tilt_deg(0.0, 0.0, 0.0))
    assert raises(lambda: saturation_envelope(1.2, 0.25, 0.25, 0.0))


def test_print_report_is_gbk_safe():
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        print_report(UavCgTrim().report())
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'uav_cg_trim_envelope' in text
    assert 'deg' in text


def test_cli_smoke_writes_report():
    output = Path(__file__).resolve().parent / '_out'
    assert cli.main(['--output', str(output)]) == 0
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert close(report['trim_tilt_deg']['pitch'], 5.7106, 1e-3)
    assert report['level'] == 'ok'


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
