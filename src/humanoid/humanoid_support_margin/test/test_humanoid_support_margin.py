"""检查支撑多边形与倾倒判据的闭式解；期望值全部可以手算.

`pytest test -v` 与 `python test/test_humanoid_support_margin.py` 两种方式都能跑。
"""
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from humanoid_support_margin import cli  # noqa: E402
from humanoid_support_margin.core import (  # noqa: E402
    G,
    HumanoidSupport,
    convex_hull,
    foot_load_fractions,
    point_in_polygon,
    support_margin,
    tipping_force_n,
    tipping_impulse_ns,
)
from humanoid_support_margin.cli import print_report  # noqa: E402


def close(value, expected, tol=1e-9):
    return abs(value - expected) <= max(tol, abs(expected) * 1e-9)


def raises(callable_):
    try:
        callable_()
    except ValueError:
        return True
    return False


def test_convex_hull_of_two_rectangles():
    check = HumanoidSupport()
    hull = check.support_polygon()
    # 双足矩形（x in [-0.15,-0.05] 与 [0.05,0.15], y in [-0.125,0.125]）的凸包是外接矩形
    assert close(min(x for x, _ in hull), -0.15, 1e-12)
    assert close(max(x for x, _ in hull), 0.15, 1e-12)
    assert close(min(y for _, y in hull), -0.125, 1e-12)
    assert close(max(y for _, y in hull), 0.125, 1e-12)
    assert len(hull) == 4
    # 凸包覆盖两脚之间的空隙区域（CoP 可以落在凸包内任意处）
    assert point_in_polygon((0.0, 0.0), hull)


def test_margin_hand_calc():
    check = HumanoidSupport()
    polygon = check.support_polygon()
    margin = support_margin((0.01, 0.0), polygon)
    assert close(margin, 0.125, 1e-12)          # 最近边是 y=+-0.125
    assert close(support_margin((0.05, 0.0), polygon), 0.10, 1e-12)   # 到右边界 0.15-0.05
    assert close(support_margin((0.01, 0.08), polygon), 0.045, 1e-12)  # 0.125-0.08


def test_margin_outside_is_negative():
    check = HumanoidSupport()
    polygon = check.support_polygon()
    margin = support_margin((0.20, 0.0), polygon)
    assert margin < 0.0
    assert close(margin, -0.05, 1e-12)
    assert point_in_polygon((0.20, 0.0), polygon) is False


def test_tipping_criteria():
    margin, height, mass = 0.125, 0.8, 40.0
    force = tipping_force_n(mass, margin, height)
    assert close(force, mass * G * margin / height, 1e-12)
    assert close(force, 61.29, 1e-2)
    impulse = tipping_impulse_ns(mass, margin, height)
    lift = math.sqrt(margin ** 2 + height ** 2) - height
    assert close(impulse, mass * math.sqrt(2 * G * lift), 1e-12)
    assert close(impulse, 17.46, 1e-2)
    # 重心越高越容易推倒：推力反比于高度
    assert close(tipping_force_n(mass, margin, 1.6) * 2.0,
                 tipping_force_n(mass, margin, 0.8), 1e-12)


def test_foot_load_fractions():
    # 投影在两脚之间：线性分配，合力矩恒等于零（CoP 恒等于投影点）
    f1, f2 = foot_load_fractions(0.01, -0.10, 0.10)
    assert close(f1, 0.45, 1e-12)
    assert close(f2, 0.55, 1e-12)
    assert close(f1 + f2, 1.0, 1e-12)
    cop = f1 * (-0.10) + f2 * 0.10
    assert close(cop, 0.01, 1e-12)
    # 投影超出两脚范围：单脚支撑
    assert foot_load_fractions(-0.20, -0.10, 0.10) == (1.0, 0.0)
    assert foot_load_fractions(0.20, -0.10, 0.10) == (0.0, 1.0)


def test_default_report():
    report = HumanoidSupport().report()
    assert report['level'] == 'ok'
    assert report['inside_polygon'] is True
    assert close(report['margin_m'], 0.125, 1e-12)
    assert close(report['tipping']['static_force_n'], 61.29, 1e-2)
    assert close(report['tipping']['impulse_ns'], 17.46, 1e-2)
    assert close(report['foot_loads']['fraction_left'], 0.45, 1e-12)
    assert close(report['foot_loads']['left_n'], 0.45 * 40.0 * G, 1e-9)
    assert any('临界推力' in text for text in report['warnings'])
    assert any('凸包' in text for text in report['notes'])
    json.dumps(report)


def test_com_outside_is_over():
    report = HumanoidSupport(com_x_m=0.20).report()
    assert report['level'] == 'over'
    assert report['inside_polygon'] is False
    assert report['margin_m'] < 0.0
    assert close(report['tipping']['static_force_n'], 0.0, 1e-12)
    assert any('必倒' in text for text in report['warnings'])


def test_narrow_margin_is_marginal():
    # 重心压到距边界 10 mm 内
    report = HumanoidSupport(com_y_m=0.115).report()
    assert report['level'] == 'marginal'
    assert close(report['margin_m'], 0.01, 1e-12)


def test_invalid_parameters_are_rejected():
    base = {'mass_kg': 40.0, 'foot1_x_m': -0.10, 'foot2_x_m': 0.10,
            'foot_width_m': 0.10, 'foot_length_m': 0.25, 'com_x_m': 0.01,
            'com_y_m': 0.0, 'com_height_m': 0.8}
    for kwargs in ({'mass_kg': 0.0}, {'foot2_x_m': -0.10}, {'foot_width_m': 0.0},
                   {'foot_length_m': -0.1}, {'com_height_m': 0.0},
                   {'com_x_m': float('nan')}):
        params = dict(base)
        params.update(kwargs)
        assert raises(lambda: HumanoidSupport(**params)), kwargs
    assert raises(lambda: convex_hull([(0.0, 0.0)]))
    assert raises(lambda: tipping_force_n(40.0, 0.0, 0.8))
    assert raises(lambda: tipping_force_n(40.0, 0.1, 0.0))
    assert raises(lambda: tipping_impulse_ns(40.0, -0.1, 0.8))
    assert raises(lambda: foot_load_fractions(0.0, 0.2, 0.1))


def test_print_report_is_gbk_safe():
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        print_report(HumanoidSupport().report())
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'humanoid_support_margin' in text
    assert 'deg' not in text and 'm' in text


def test_cli_smoke_writes_report():
    output = Path(__file__).resolve().parent / '_out'
    assert cli.main(['--output', str(output)]) == 0
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert close(report['margin_m'], 0.125, 1e-12)
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
