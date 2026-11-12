"""检查弗劳德缩比换算的闭式解与边界；期望值全部可以手算.

`pytest test -v` 与 `python test/test_usv_froude_scaling.py` 两种方式都能跑。
"""
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from usv_froude_scaling import cli  # noqa: E402
from usv_froude_scaling.core import (  # noqa: E402
    G,
    NU_SEA,
    RHO_SEA,
    UsvFroude,
    friction_coef_itfc,
    friction_resistance,
    froude_number,
    model_speed_for_froude,
    scale_power,
    scale_resistance,
)
from usv_froude_scaling.cli import print_report  # noqa: E402


def close(value, expected, tol=1e-9):
    return abs(value - expected) <= max(tol, abs(expected) * 1e-9)


def raises(callable_):
    try:
        callable_()
    except ValueError:
        return True
    return False


def test_froude_number():
    assert close(froude_number(5.0, 10.0), 5.0 / math.sqrt(G * 10.0), 1e-12)
    assert close(froude_number(5.0, 10.0), 0.50491, 1e-5)
    assert froude_number(6.0, 10.0) > froude_number(5.0, 10.0)


def test_model_speed_for_equal_froude():
    assert close(model_speed_for_froude(5.0, 10.0), 5.0 / math.sqrt(10.0), 1e-12)
    assert close(model_speed_for_froude(5.0, 10.0), 1.58114, 1e-5)
    # 同缩尺比下模型速度与实船速度成线性
    assert close(model_speed_for_froude(8.0, 10.0) / model_speed_for_froude(5.0, 10.0),
                 8.0 / 5.0, 1e-12)


def test_scaling_laws():
    assert close(scale_resistance(2.5, 10.0), 2500.0)
    assert close(scale_resistance(2.5, 10.0), 2.5 * 10.0 ** 3, 1e-12)
    assert close(scale_power(10.0, 10.0), 10.0 * 10.0 ** 3.5, 1e-12)
    assert close(scale_power(10.0, 10.0), 31622.78, 0.1)


def test_friction_coef_itfc():
    reynolds = 5.0 * 10.0 / NU_SEA
    expected = 0.075 / (math.log10(reynolds) - 2.0) ** 2
    assert close(friction_coef_itfc(reynolds), expected, 1e-12)
    assert close(friction_coef_itfc(reynolds), 0.0023717, 1e-7)
    # 雷诺数越大摩擦系数越小
    assert friction_coef_itfc(reynolds * 10.0) < friction_coef_itfc(reynolds)


def test_friction_resistance_default():
    value = friction_resistance(5.0, 10.0, 22.0)
    assert close(value, 668.53, 1e-2)
    assert close(value, friction_resistance(5.0, 10.0, 22.0, RHO_SEA, NU_SEA), 1e-12)
    # 黏度变大 -> 雷诺数变小 -> 摩擦阻力变大
    assert friction_resistance(5.0, 10.0, 22.0, nu=1.5e-6) > value


def test_report_spot_values():
    report = UsvFroude().report()
    model = report['model']
    ship = report['ship']
    power = report['power']
    assert close(model['speed_m_s'], 1.58114, 1e-5)
    assert close(model['wetted_area_m2'], 0.22)
    assert close(model['friction_resistance_n'], 1.2434, 1e-4)
    assert close(model['residuary_resistance_n'], 1.2567, 1e-4)
    assert close(ship['froude_number'], 0.50491, 1e-5)
    assert close(ship['friction_resistance_n'], 668.53, 1e-2)
    assert close(ship['total_resistance_n'], 1925.2, 1e-1)
    assert close(power['effective_w'], 9625.7, 1e-1)
    assert close(power['delivered_w'], 17501.3, 1e-1)


def test_decomposition_is_exact():
    report = UsvFroude().report()
    # 实船总阻力 = 实船摩擦 + 模型剩余阻力 * lambda^3
    ship = report['ship']
    model = report['model']
    params = report['parameters']
    assert close(ship['total_resistance_n'],
                 ship['friction_resistance_n']
                 + model['residuary_resistance_n'] * params['scale'] ** 3, 1e-9)
    # 功率链：P_e = R_s * v_s, P_d = P_e / eta
    assert close(report['power']['effective_w'],
                 ship['total_resistance_n'] * params['ship_speed_m_s'], 1e-9)
    assert close(report['power']['delivered_w'],
                 report['power']['effective_w'] / params['propulsive_efficiency'], 1e-9)


def test_negative_residuary_warns():
    report = UsvFroude(model_resistance_n=1.0).report()
    assert report['model']['residuary_resistance_n'] < 0.0
    assert any('剩余阻力' in text for text in report['warnings'])


def test_high_froude_warns():
    report = UsvFroude(ship_speed_m_s=12.0).report()
    assert report['ship']['froude_number'] > 1.0
    assert any('弗劳德换算' in text for text in report['warnings'])


def test_invalid_parameters_are_rejected():
    base = {'ship_speed_m_s': 5.0, 'ship_length_m': 10.0, 'scale': 10.0,
            'wetted_area_ship_m2': 22.0, 'model_resistance_n': 2.5,
            'propulsive_efficiency': 0.55, 'rho': 1025.0, 'nu': 1.19e-6}
    for kwargs in ({'ship_speed_m_s': 0.0}, {'ship_length_m': 0.0}, {'scale': 1.0},
                   {'scale': 0.5}, {'wetted_area_ship_m2': 0.0},
                   {'model_resistance_n': -1.0}, {'propulsive_efficiency': 0.0},
                   {'propulsive_efficiency': 1.2}, {'rho': 0.0}, {'nu': 0.0}):
        params = dict(base)
        params.update(kwargs)
        assert raises(lambda: UsvFroude(**params)), kwargs
    assert raises(lambda: froude_number(0.0, 10.0))
    assert raises(lambda: model_speed_for_froude(5.0, 1.0))
    assert raises(lambda: scale_resistance(-1.0, 10.0))
    assert raises(lambda: scale_power(1.0, 1.0))
    assert raises(lambda: friction_coef_itfc(0.5))
    assert raises(lambda: friction_resistance(5.0, 10.0, 0.0))


def test_print_report_is_gbk_safe():
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        print_report(UsvFroude().report())
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'usv_froude_scaling' in text
    assert 'm/s' in text


def test_cli_smoke_writes_report():
    output = Path(__file__).resolve().parent / '_out'
    assert cli.main(['--output', str(output)]) == 0
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert close(report['ship']['total_resistance_n'], 1925.2, 1e-1)
    assert report['ship']['froude_number'] < 1.0


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
