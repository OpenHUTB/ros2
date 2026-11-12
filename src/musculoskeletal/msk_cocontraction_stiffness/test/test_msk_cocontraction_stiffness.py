"""检查共收缩与关节刚度的闭式解；期望值全部可以手算.

`pytest test -v` 与 `python test/test_msk_cocontraction_stiffness.py` 两种方式都能跑。
"""
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from msk_cocontraction_stiffness import cli  # noqa: E402
from msk_cocontraction_stiffness.core import (  # noqa: E402
    Q_FACTOR_MAX,
    Q_FACTOR_MIN,
    MskCocontraction,
    cocontraction_solve,
    deflection_deg,
    joint_stiffness,
    muscle_force,
)
from msk_cocontraction_stiffness.cli import print_report  # noqa: E402


def close(value, expected, tol=1e-9):
    return abs(value - expected) <= max(tol, abs(expected) * 1e-9)


def raises(callable_):
    try:
        callable_()
    except ValueError:
        return True
    return False


def test_muscle_force_linear():
    assert close(muscle_force(0.5, 600.0), 300.0)
    assert close(muscle_force(0.0, 600.0), 0.0)
    assert close(muscle_force(1.0, 600.0), 600.0)
    assert raises(lambda: muscle_force(1.1, 600.0))
    assert raises(lambda: muscle_force(-0.1, 600.0))
    assert raises(lambda: muscle_force(0.5, 0.0))


def test_cocontraction_solve_hand_calc():
    # tau=8, r=0.03 -> F_net=266.667；c=0.3 -> 基线 180
    solution = cocontraction_solve(8.0, 0.03, 600.0, 0.3)
    assert solution['feasible'] is True
    assert close(solution['force_agonist_n'] - solution['force_antagonist_n'],
                 8.0 / 0.03, 1e-9)
    assert close(solution['force_agonist_n'] + solution['force_antagonist_n'],
                 2.0 * 0.3 * 600.0, 1e-9)
    assert close(solution['force_agonist_n'], 313.333, 1e-3)
    assert close(solution['force_antagonist_n'], 46.667, 1e-3)
    assert close(solution['activation_agonist'], solution['force_agonist_n'] / 600.0,
                 1e-12)


def test_cocontraction_boundary():
    # 上限 tau_max = 2*c*F_max*r = 10.8：恰好取到时拮抗侧为零
    at_limit = cocontraction_solve(10.8, 0.03, 600.0, 0.3)
    assert at_limit['feasible'] is True
    assert close(at_limit['force_antagonist_n'], 0.0, 1e-9)
    beyond = cocontraction_solve(10.81, 0.03, 600.0, 0.3)
    assert beyond['feasible'] is False
    assert close(beyond['torque_max_nm'], 10.8, 1e-9)
    # c=0 时任何净力矩都不可行
    assert cocontraction_solve(0.001, 0.03, 600.0, 0.0)['feasible'] is False


def test_joint_stiffness_hand_calc():
    # K = r^2 * q * (F_ago+F_ant) / L = 9e-4 * 25 * 360 / 0.25 = 32.4
    stiffness = joint_stiffness(0.03, 25.0, 360.0, 0.25)
    assert close(stiffness, 32.4, 1e-9)
    # 共收缩翻倍 -> 刚度翻倍（力之和翻倍）
    assert close(joint_stiffness(0.03, 25.0, 720.0, 0.25), 64.8, 1e-9)
    # q 系数同比例
    assert close(joint_stiffness(0.03, 12.5, 360.0, 0.25), 16.2, 1e-9)
    assert raises(lambda: joint_stiffness(0.03, 9.0, 360.0, 0.25))
    assert raises(lambda: joint_stiffness(0.03, 31.0, 360.0, 0.25))


def test_stiffness_decouples_from_net_torque():
    # 同一共收缩水平下，不同净力矩的肌力之和不变 -> 刚度不变
    low = cocontraction_solve(4.0, 0.03, 600.0, 0.3)
    high = cocontraction_solve(8.0, 0.03, 600.0, 0.3)
    assert close(low['force_agonist_n'] + low['force_antagonist_n'],
                 high['force_agonist_n'] + high['force_antagonist_n'], 1e-9)


def test_deflection_formula():
    assert close(deflection_deg(2.0, 32.4), math.degrees(2.0 / 32.4), 1e-12)
    assert close(deflection_deg(2.0, 32.4), 3.537, 1e-3)
    # 刚度翻倍 -> 偏转减半
    assert close(deflection_deg(2.0, 64.8) * 2.0, deflection_deg(2.0, 32.4), 1e-9)


def test_default_report():
    report = MskCocontraction().report()
    assert report['level'] == 'ok'
    assert report['solution']['feasible'] is True
    assert close(report['stiffness']['joint_nm_per_rad'], 32.4, 1e-9)
    assert close(report['stiffness']['deflection_deg'], 3.537, 1e-3)
    assert any('可行' in text for text in report['warnings'])
    assert any('力-长度' in text for text in report['notes'])
    json.dumps(report)


def test_high_utilization_is_marginal():
    report = MskCocontraction(torque_nm=10.0).report()
    assert report['level'] == 'marginal'
    assert any('上限' in text for text in report['warnings'])


def test_over_torque_is_infeasible():
    report = MskCocontraction(torque_nm=12.0).report()
    assert report['level'] == 'over'
    assert report['solution']['feasible'] is False
    assert report['stiffness']['joint_nm_per_rad'] == 0.0
    assert any('需提高共收缩' in text for text in report['warnings'])


def test_invalid_parameters_are_rejected():
    base = {'torque_nm': 8.0, 'moment_arm_m': 0.03, 'f_max_n': 600.0,
            'slack_length_m': 0.25, 'q_factor': 25.0, 'coactivation': 0.3,
            'disturbance_nm': 2.0}
    for kwargs in ({'torque_nm': -1.0}, {'moment_arm_m': 0.0}, {'f_max_n': 0.0},
                   {'slack_length_m': 0.0}, {'q_factor': Q_FACTOR_MIN - 1.0},
                   {'q_factor': Q_FACTOR_MAX + 1.0}, {'coactivation': 1.1},
                   {'coactivation': -0.1}, {'disturbance_nm': 0.0},
                   {'torque_nm': float('nan')}):
        params = dict(base)
        params.update(kwargs)
        assert raises(lambda: MskCocontraction(**params)), kwargs
    assert raises(lambda: cocontraction_solve(-1.0, 0.03, 600.0, 0.3))
    assert raises(lambda: deflection_deg(0.0, 32.4))


def test_print_report_is_gbk_safe():
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        print_report(MskCocontraction().report())
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'msk_cocontraction_stiffness' in text
    assert 'N*m' in text


def test_cli_smoke_writes_report():
    output = Path(__file__).resolve().parent / '_out'
    assert cli.main(['--output', str(output)]) == 0
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert close(report['stiffness']['joint_nm_per_rad'], 32.4, 1e-9)
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
